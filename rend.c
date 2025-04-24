#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mat4.h"
#include "rend.h"
#include "types.h"
#include "util.h"

#ifndef NDEBUG
#define dprintf printf
#else
#define dprintf(...) {}
#endif

#define INTERVAL_CNT  16

struct split {
	float          cost;
	float          pos;
	unsigned char  axis;
};

struct interval {
	struct vec3   min;
	unsigned int  cnt;
	struct vec3   max;
	unsigned int  pad0;
};

struct bnode { // bvh node, 32 bytes wide
	struct vec3   min;
	unsigned int  sid; // Start index or left child node id
	struct vec3   max;
	unsigned int  cnt; // Tri or inst cnt
};

struct hit {
	float         t;
	float         u;
	float         v;
	unsigned int  id; // tri id < 16 | inst id
};

float calc_area(struct vec3 mi, struct vec3 ma)
{
	struct vec3 d = vec3_sub(ma, mi);
	return d.x * d.y + d.y * d.z + d.z * d.x;
}

struct split find_intervalsplit(const struct bnode *n,
                                const struct aabb *aabbs,
                                const unsigned int *imap)
{
	struct split best = {.cost = FLT_MAX};
	for (unsigned char axis = 0; axis < 3; axis++) {
		// Get axis bounds
		float minc = vec3_getc(n->min, axis);
		float maxc = vec3_getc(n->max, axis);
		if (fabsf(maxc - minc) < EPS)
			continue; // Skip empty axis

		// Init empty intervals
		struct interval ivs[INTERVAL_CNT];
		for (unsigned char i = 0; i < INTERVAL_CNT; i++)
			ivs[i] = (struct interval){
			  .cnt = 0,
			  .min = (struct vec3){FLT_MAX, FLT_MAX, FLT_MAX},
			  .max = (struct vec3){-FLT_MAX, -FLT_MAX, -FLT_MAX}};

		// Count objects per interval and find the combined bounds
		float delta = INTERVAL_CNT / (maxc - minc);
		const unsigned int *ip = &imap[n->sid];
		for (unsigned int i = 0; i < n->cnt; i++) {
			const struct aabb *a = &aabbs[*ip++];
			float c = 0.5f * (vec3_getc(a->min, axis) +
			  vec3_getc(a->max, axis));
			unsigned char ivid  = (unsigned char)max(0,
			  min(INTERVAL_CNT - 1, (c - minc) * delta));
			struct interval *iv = &ivs[ivid];
			iv->min = vec3_min(iv->min, a->min);
			iv->max = vec3_max(iv->max, a->max);
			iv->cnt++;
		}

		// Calc l/r areas and cnts for each interval separating plane
		float lareas[INTERVAL_CNT - 1]; // Areas left
		float rareas[INTERVAL_CNT - 1];
		unsigned int lcnts[INTERVAL_CNT - 1];
		unsigned int rcnts[INTERVAL_CNT - 1];
		struct vec3 lmin = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 lmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
		struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
		unsigned int ltotcnt = 0; // Total cnt
		unsigned int rtotcnt = 0;
		for (unsigned char i = 0; i < INTERVAL_CNT - 1; i++) {
			// From left
			struct interval *iv = &ivs[i];
			ltotcnt += iv->cnt;
			lcnts[i] = ltotcnt;
			lmin = vec3_min(lmin, iv->min);
			lmax = vec3_max(lmax, iv->max);
			lareas[i] = calc_area(lmin, lmax);
			// From right
			iv = &ivs[INTERVAL_CNT - 1 - i];
			rtotcnt += iv->cnt;
			rcnts[INTERVAL_CNT - 2 - i] = rtotcnt;
			rmin = vec3_min(rmin, iv->min);
			rmax = vec3_max(rmax, iv->max);
			rareas[INTERVAL_CNT - 2 - i] = calc_area(rmin, rmax);
		}

		// Find best surface area cost for interval planes
		delta = 1.0f / delta;
		for (unsigned char i = 0; i < INTERVAL_CNT - 1; i++) {
			float c = lcnts[i] * lareas[i] + rcnts[i] * rareas[i];
			if (c < best.cost) {
				best.cost = c;
				best.axis = axis;
				best.pos = minc + (i + 1) * delta;
			}
		}
	}

	return best;
}

void build_bvh(struct bnode *nodes, struct aabb *aabbs, unsigned int *imap,
                 unsigned int cnt, struct vec3 rootmin, struct vec3 rootmax)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Prepare root node
	struct bnode *root = nodes;
	root->min = rootmin;
	root->max = rootmax;
	root->sid = 0;
	root->cnt = cnt;

	unsigned int ncnt = 2; // Root + 1 empty node for cache alignment
	unsigned int nid = 0; // Start with root node

	while (true) {
		struct bnode *n = &nodes[nid];

		/*// Split at longest axis
		struct vec3 e = vec3_scale(vec3_sub(n->max, n->min), 0.5f);
		unsigned char a = e.x > e.y && e.x > e.z ? 0 :
		  (e.y > e.x && e.y > e.z ? 1 : 2); // Split axis
		float s = vec3_getc(n->min, a) + vec3_getc(e, a); // Split pos
		*/

		// Find best split according to SAH
		struct split best = find_intervalsplit(n, aabbs, imap);
		float nosplit = n->cnt * calc_area(n->min, n->max);
		if (nosplit <= best.cost) {
			//dprintf("no split of sid: %d, cnt: %d\n",
			//  n->sid, n->cnt);
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else {
				break;
			}
		}

		// Child bounds
		struct vec3 lmin = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 lmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
		struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};

		// Partition in l and r of split axis
		unsigned int l = n->sid;
		unsigned int r = l + n->cnt;
		for (unsigned int i = 0; i < n->cnt; i++) {
			unsigned int id = imap[l];
			struct aabb *b = &aabbs[id];
			struct vec3 mi = b->min;
			struct vec3 ma = b->max;
			float c = 0.5f * (vec3_getc(mi, best.axis) +
			  vec3_getc(ma, best.axis));
			if (c < best.pos) {
				l++;
				lmin = vec3_min(lmin, mi);
				lmax = vec3_max(lmax, ma);
			} else {
				// Wrong side, swap indices
				imap[l] = imap[--r];
				imap[r] = id;
				rmin = vec3_min(rmin, mi);
				rmax = vec3_max(rmax, ma);
			}
		}

		unsigned int lcnt = l - n->sid;
		if (lcnt == 0 || lcnt == n->cnt) {
			//dprintf("one side of the partition was empty\n");
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else {
				break;
			}
		}

		// Init children
		struct bnode *left = &nodes[ncnt];
		left->sid = n->sid;
		left->cnt = lcnt;
		left->min = lmin;
		left->max = lmax;

		struct bnode *right = &nodes[ncnt + 1];
		right->sid = r;
		right->cnt = n->cnt - lcnt;
		right->min = rmin;
		right->max = rmax;

		// Update current (interior) node's child link
		n->sid = ncnt; // Right child is implicitly + 1
		n->cnt = 0; // No leaf, no tris or instance

		// Push right child on stack and continue with left
		assert(spos < 64);
		stack[spos++] = ncnt + 1;
		nid = ncnt;

		ncnt += 2; // Account for two new nodes
	}
}

void convert_bvh(struct b2node *tgt, struct bnode *src)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Current src and tgt node indices
	unsigned int sn = 0;
	unsigned int tn = 0;

	while (true) {
		struct bnode *s = &src[sn];
		struct b2node *t = &tgt[tn++];

		if (s->cnt > 0) {
			t->start = s->sid;
			t->cnt = s->cnt;

			if (spos > 0) {
				// Assign tgt prnt's right index
				tgt[stack[--spos]].r = tn;
				sn = stack[--spos];
			} else
				return;
		} else {
			sn = s->sid; // Left child

			struct bnode *lc = &src[sn];
			t->l = tn; // Left is next node in tgt
			t->lmin = lc->min;
			t->lmax = lc->max;

			struct bnode *rc = &src[sn + 1]; // Right = left + 1
			t->rmin = rc->min;
			t->rmax = rc->max;
			// Set tgt's right child index when it gets off stack

			t->cnt = 0; // Mark as interior node

			stack[spos++] = sn + 1; // Right src node
			stack[spos++] = tn - 1; // Curr tgt (prnt of right)
		}
	}
}

float intersect_aabb(struct vec3 ori, struct vec3 idir, float tfar,
                     struct vec3 mi, struct vec3 ma)
{
	float tx0 = (mi.x - ori.x) * idir.x;
	float tx1 = (ma.x - ori.x) * idir.x;
	float tmin = min(tx0, tx1);
	float tmax = max(tx0, tx1);

	float ty0 = (mi.y - ori.y) * idir.y;
	float ty1 = (ma.y - ori.y) * idir.y;
	tmin = max(tmin, min(ty0, ty1));
	tmax = min(tmax, max(ty0, ty1));

	float tz0 = (mi.z - ori.z) * idir.z;
	float tz1 = (ma.z - ori.z) * idir.z;
	tmin = max(tmin, min(tz0, tz1));
	tmax = min(tmax, max(tz0, tz1));

	if (tmin <= tmax && tmin < tfar && tmax >= 0.0f)
		return tmin;
	else
		return FLT_MAX;
}

void intersect_tri(struct hit *h, struct vec3 ori, struct vec3 dir,
                   const struct rtri *tris,
                   unsigned short triid, unsigned short instid)
{
	// Vectors of two edges sharing v0
	const struct rtri *t = &tris[triid];
	struct vec3 v0 = t->v0;
	struct vec3 e0 = vec3_sub(t->v1, v0);
	struct vec3 e1 = vec3_sub(t->v2, v0);

	// Calculate determinant
	struct vec3 pv = vec3_cross(dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return;

	float idet = 1.0f / det;

	// Distance v0 to origin
	struct vec3 tv = vec3_sub(ori, v0);

	// Calculate param u and test bounds
	float u = vec3_dot(tv, pv) * idet;
	if (u < 0.0f || u > 1.0f)
		return;

	// Prepare to test for v
	struct vec3 qv = vec3_cross(tv, e0);

	// Calculate param v and test bounds
	float v = vec3_dot(dir, qv) * idet;
	if (v < 0.0f || u + v > 1.0f)
		return;

	// Calculate dist
	float dist = vec3_dot(e1, qv) * idet;
	if (dist > 0.0f && dist < h->t) {
		h->t = dist;
		h->u = u;
		h->v = v;
		h->id = triid << 16 | instid;
	}
}

void intersect_blas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid)
{
	unsigned int stack[64];
	unsigned char spos = 0;

	unsigned int curr = 0;

	struct vec3 idir = (struct vec3){
	  1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z};

	while (true) {
		const struct b2node *n = &blas[curr];

		if (n->cnt > 0) {
			// Leaf, check triangles
			for (unsigned int i = 0; i < n->cnt; i++)
				intersect_tri(h, ori, dir, tris,
				  (unsigned short)imap[n->start + i],
				  (unsigned short)instid);

			// Pop next node from stack if something is left
			if (spos > 0)
				curr = stack[--spos];
			else
				return;
		} else {
			// Interior node, check child aabbs
			float d0 = intersect_aabb(ori, idir, h->t,
			  n->lmin, n->lmax);
			float d1 = intersect_aabb(ori, idir, h->t,
			  n->rmin, n->rmax);

			unsigned int l = n->l;
			unsigned int r = n->r;

			if (d0 > d1) {
				float t = d0;
				d0 = d1;
				d1 = t;

				unsigned int tc = r;
				r = l;
				l = tc;
			}

			if (d0 == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					curr = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				curr = l;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < 64);
					stack[spos++] = r;
				}
			}
		}
	}
}

void intersect_tlas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *nodes, const unsigned int *imap,
                    const struct rinst *insts, const struct rtri *tris,
                    unsigned int tlasofs)
{
	unsigned int stack[64];
	unsigned char spos = 0;

	unsigned int curr = 0;

	const struct b2node *tlas = &nodes[tlasofs << 1];
	const unsigned int *tlasimap = &imap[tlasofs];

	struct vec3 idir = (struct vec3){
	  1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z};

	while (true) {
		const struct b2node *n = &tlas[curr];

		if (n->cnt > 0) {
			// Leaf, check instance blas
			const unsigned int *ip = &tlasimap[n->start];
			for (unsigned int i = 0; i < n->cnt; i++) {
				unsigned int instid = *ip++;
				const struct rinst *ri = &insts[instid];

				// Transform ray into object space of instance
				float inv[16];
				mat4_from3x4(inv, ri->globinv);

				unsigned int o = ri->triofs;
				intersect_blas(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &nodes[o << 1], &imap[o], &tris[o], instid);
			}

			// Pop next node from stack if something is left
			if (spos > 0)
				curr = stack[--spos];
			else
				return;
		} else {
			// Interior node, check child aabbs
			float d0 = intersect_aabb(ori, idir, h->t,
			  n->lmin, n->lmax);
			float d1 = intersect_aabb(ori, idir, h->t,
			  n->rmin, n->rmax);

			unsigned int l = n->l;
			unsigned int r = n->r;

			if (d0 > d1) {
				float t = d0;
				d0 = d1;
				d1 = t;

				unsigned int tc = r;
				r = l;
				l = tc;
			}

			if (d0 == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					curr = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				curr = l;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < 64);
					stack[spos++] = r;
				}
			}
		}
	}
}

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = aligned_alloc(64, maxmtls * sizeof(*rd->mtls));

	rd->tris = aligned_alloc(64, maxtris * sizeof(*rd->tris));
	rd->nrms = aligned_alloc(64, maxtris * sizeof(*rd->nrms));

	rd->insts = aligned_alloc(64, maxinsts * sizeof(*rd->insts));
	rd->aabbs = aligned_alloc(64, maxinsts * sizeof(*rd->aabbs));

	// Index map contains tri and instance ids
	unsigned int idcnt = maxtris + maxinsts;
	rd->imap = aligned_alloc(64, idcnt * sizeof(*rd->imap));

	// Bvh nodes for blas and tlas combined in one array
	rd->nodes = aligned_alloc(64, idcnt * 2 * sizeof(*rd->nodes));
	memset(rd->nodes, 0, idcnt * 2 * sizeof(*rd->nodes));

	// Start of tlas index map and tlas nodes * 2
	rd->tlasofs = maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->nodes);
	free(rd->imap);
	free(rd->aabbs);
	free(rd->insts);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
}

void rend_prepstatic(struct rdata *rd)
{
	for (unsigned int j = 0; j < rd->instcnt; j++) {
		struct rinst *ri = &rd->insts[j];
		struct b2node *rn = &rd->nodes[ri->triofs << 1]; // Root node
		if (rn->l + rn->r == 0) { // Not processed yet
			dprintf("Creating blas for inst: %d, ofs: %d, cnt: %d, addr: %p\n",
			  j, ri->triofs, ri->tricnt, (void *)rn);
			struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
			struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
			struct aabb aabbs[ri->tricnt];
			struct aabb *ap = aabbs;
			struct rtri *tp = &rd->tris[ri->triofs];
			unsigned int *ip = &rd->imap[ri->triofs];
			for (unsigned int i = 0; i < ri->tricnt; i++) {
				ap->min = ap->max = tp->v0;
				ap->min = vec3_min(ap->min, tp->v1);
				ap->max = vec3_max(ap->max, tp->v1);
				ap->min = vec3_min(ap->min, tp->v2);
				ap->max = vec3_max(ap->max, tp->v2);
				rmin = vec3_min(rmin, ap->min);
				rmax = vec3_max(rmax, ap->max);
				*ip++ = i;
				ap++;
				tp++;
			}

			struct bnode nodes[ri->tricnt << 1];
			build_bvh(nodes, aabbs, &rd->imap[ri->triofs],
			  ri->tricnt, rmin, rmax);
			convert_bvh(&rd->nodes[ri->triofs << 1], nodes);
		}
	}
}

void rend_prepdynamic(struct rdata *rd)
{
	struct aabb *ap = rd->aabbs; // World space aabbs of instances
	struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
	struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	unsigned int tlasofs = rd->tlasofs;
	unsigned int *ip = &rd->imap[tlasofs];
	for (unsigned int i = 0; i < rd->instcnt; i++) {
		rmin = vec3_min(rmin, ap->min);
		rmax = vec3_max(rmax, ap->max);
		*ip++ = i;
		ap++;
	}

	struct bnode nodes[rd->instcnt << 1];
	build_bvh(nodes, rd->aabbs, &rd->imap[tlasofs],
	  rd->instcnt, rmin, rmax);
	convert_bvh(&rd->nodes[tlasofs << 1], nodes);
}

struct vec3 calc_nrm(float u, float v, struct rnrm *rn,
                        float inv_transpose[16])
{
	struct vec3 nrm = vec3_add(vec3_scale(rn->n1, u),
	  vec3_add(vec3_scale(rn->n2, v), vec3_scale(rn->n0, 1 - u - v)));
	return vec3_unit(mat4_muldir(inv_transpose, nrm));
}

struct vec3 trace(struct vec3 o, struct vec3 d, const struct rdata *rd)
{
	struct hit h = {.t = FLT_MAX};
	intersect_tlas(&h, o, d, rd->nodes, rd->imap, rd->insts,
	  rd->tris, rd->tlasofs);

	struct vec3 c = rd->bgcol;
	if (h.t < FLT_MAX) {
		unsigned int instid = h.id & 0xffff;
		unsigned int triid = h.id >> 16;
		struct rinst *ri = &rd->insts[instid];
		struct rnrm *rn = &rd->nrms[ri->triofs + triid];
		unsigned int mtlid = rn->mtlid;

		// Inverse transpose, dir mul is 3x4
		float it[16];
		float *rt = ri->globinv;
		for (int j = 0; j < 4; j++)
			for (int i = 0; i < 3; i++)
				it[4 * j + i] = rt[4 * i + j];

		struct vec3 nrm = calc_nrm(h.u, h.v, rn, it);
		nrm = vec3_scale(vec3_add(nrm, (struct vec3){1, 1, 1}), 0.5f);
		c = vec3_mul(nrm, rd->mtls[mtlid].col);
		//c = nrm;
	}

	return c;
}

static inline int fetch_and_add(int *var, int val)
{
	return __atomic_fetch_add(var, val, __ATOMIC_SEQ_CST);
}

int rend_render(void *d)
{
	struct rdata *rd = d;

	struct vec3 eye = rd->cam.eye;
	struct vec3 dx = rd->view.dx;
	struct vec3 dy = rd->view.dy;
	struct vec3 tl = rd->view.tl;

	unsigned int blksx = rd->view.w / rd->blksz;
	unsigned int blksy = rd->view.h / rd->blksz;
	int          blkcnt = blksx * blksy;

	unsigned int *buf = rd->buf;
	unsigned int w = rd->view.w;
	unsigned int bs = rd->blksz;

	while (true) {
		int blk = fetch_and_add(&rd->blknum, 1);
		if (blk >= blkcnt)
			return 0;
		unsigned int bx = (blk % blksx) * bs;
		unsigned int by = (blk / blksx) * bs;
		for (unsigned int j = 0; j < bs; j++) {
			unsigned int y = by + j;
			unsigned int yofs = w * y;
			for (unsigned int i = 0; i < bs; i++) {
				unsigned int x = bx + i;
				struct vec3 p = vec3_add(tl, vec3_add(
				  vec3_scale(dx, x), vec3_scale(dy, y)));

				struct vec3 c =
				  trace(eye, vec3_unit(vec3_sub(p, eye)), rd);

				buf[yofs + x] = 0xffu << 24 |
				  min(255, (unsigned int)(255 * c.x)) << 16 |
				  min(255, (unsigned int)(255 * c.y)) <<  8 |
				  min(255, (unsigned int)(255 * c.z));
			}
		}
	}
}

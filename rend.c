#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mat4.h"
#include "rend.h"
#include "types.h"
#include "util.h"

#ifndef NDEBUG
#define dprintf printf
#else
#define dprintf(...) {}
#endif

// Max 4096 instances
#define INST_ID_BITS  12
#define INST_ID_MASK  0xfff

#define INTERVAL_CNT  16

struct bnode { // bvh node, 1-wide, 32 bytes wide
	struct vec3   min;
	unsigned int  sid; // Start index or left child node id
	struct vec3   max;
	unsigned int  cnt; // Tri or inst cnt
};

struct hit {
	float         t;
	float         u;
	float         v;
	unsigned int  id; // triid << INST_ID_BITS | instid & INST_ID_MASK
};

float calc_area(struct vec3 mi, struct vec3 ma)
{
	struct vec3 d = vec3_sub(ma, mi);
	return d.x * d.y + d.y * d.z + d.z * d.x;
}

unsigned int build_bvh(struct bnode *nodes, struct aabb *aabbs, unsigned int *imap,
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

	struct vec3 minext = // Min extent relative to root aabb
	  vec3_scale(vec3_sub(rootmax, rootmin), 0.00000001f);

	while (true) {
		struct bnode *n = &nodes[nid];

		// Init interval aabbs and counts
		struct vec3 imin[3][INTERVAL_CNT];
		struct vec3 imax[3][INTERVAL_CNT];
		unsigned int icnt[3][INTERVAL_CNT];
		for (unsigned char a = 0; a < 3; a++) {
			for (unsigned int i = 0; i < INTERVAL_CNT; i++) {
				imin[a][i] =
				  (struct vec3){FLT_MAX, FLT_MAX, FLT_MAX};
				imax[a][i] =
				  (struct vec3){-FLT_MAX, -FLT_MAX, -FLT_MAX};
				icnt[a][i] = 0;
			}
		}

		// Count objects per interval and find the combined bounds
		struct vec3 invd = { // 1 / interval dims (delta) per axis
		  (float)INTERVAL_CNT / (n->max.x - n->min.x),
		  (float)INTERVAL_CNT / (n->max.y - n->min.y),
		  (float)INTERVAL_CNT / (n->max.z - n->min.z)};
		const unsigned int *ip = &imap[n->sid];
		for (unsigned int i = 0; i < n->cnt; i++) {
			const struct aabb *a = &aabbs[*ip++];

			struct vec3 c = vec3_mul(vec3_sub(
			  vec3_scale(vec3_add(a->min, a->max), 0.5f), n->min),
			  invd);

			// Clamp interval index per axis
			int binx = min(max((int)c.x, 0), INTERVAL_CNT - 1);
			int biny = min(max((int)c.y, 0), INTERVAL_CNT - 1);
			int binz = min(max((int)c.z, 0), INTERVAL_CNT - 1);

			icnt[0][binx]++;
			imin[0][binx] = vec3_min(imin[0][binx], a->min);
			imax[0][binx] = vec3_max(imax[0][binx], a->max);

			icnt[1][biny]++;
			imin[1][biny] = vec3_min(imin[1][biny], a->min);
			imax[1][biny] = vec3_max(imax[1][biny], a->max);

			icnt[2][binz]++;
			imin[2][binz] = vec3_min(imin[2][binz], a->min);
			imax[2][binz] = vec3_max(imax[2][binz], a->max);
		}

		float bcost = FLT_MAX; // Best split cost
		unsigned int baxis; // Best split axis
		unsigned int bpos; // Best split plane
		struct vec3 blmin; // Best bounding box l/r
		struct vec3 blmax;
		struct vec3 brmin;
		struct vec3 brmax;
		for (unsigned char a = 0; a < 3; a++) {
			// Skip 'empty axis'
			if (vec3_getc(n->max, a) - vec3_getc(n->min, a) <
			  vec3_getc(minext, a))
				continue;
			// Calc l/r areas, cnts and SAH for each interval plane
			struct vec3 lmin[INTERVAL_CNT - 1];
			struct vec3 rmin[INTERVAL_CNT - 1];
			struct vec3 lmax[INTERVAL_CNT - 1];
			struct vec3 rmax[INTERVAL_CNT - 1];
			struct vec3 lmi = {FLT_MAX, FLT_MAX, FLT_MAX};
			struct vec3 lma = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
			struct vec3 rmi = {FLT_MAX, FLT_MAX, FLT_MAX};
			struct vec3 rma = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
			unsigned int lcnt = 0;
			unsigned int rcnt = 0;
			float lsah[INTERVAL_CNT - 1];
			float rsah[INTERVAL_CNT - 1];
			for (unsigned int i = 0; i < INTERVAL_CNT - 1; i++) {
				// Sweep from left
				lmi = vec3_min(lmi, imin[a][i]);
				lmin[i] = lmi;

				lma = vec3_max(lma, imax[a][i]);
				lmax[i] = lma;

				lcnt += icnt[a][i];

				lsah[i] = lcnt > 0 ?
				  (float)lcnt * calc_area(lmi, lma) : FLT_MAX;

				// Sweep from right
				rmi = vec3_min(rmi,
				  imin[a][INTERVAL_CNT - 1 - i]);
				rmin[INTERVAL_CNT - 2 - i] = rmi;

				rma = vec3_max(rma,
				  imax[a][INTERVAL_CNT - 1 - i]);
				rmax[INTERVAL_CNT - 2 - i] = rma;

				rcnt += icnt[a][INTERVAL_CNT - 1 - i];

				rsah[INTERVAL_CNT - 2 - i] = rcnt > 0 ?
				  (float)rcnt * calc_area(rmi, rma) : FLT_MAX;
			}

			// Find best surface area cost for interval planes
			for (unsigned int i = 0; i < INTERVAL_CNT - 1; i++) {
				float c = lsah[i] + rsah[i];
				if (c < bcost) {
					bcost = c;
					baxis = a;
					bpos = i;
					blmin = lmin[i];
					blmax = lmax[i];
					brmin = rmin[i];
					brmax = rmax[i];
				}
			}
		}

		// Decide if split or leaf has better cost
		float nosplit = (float)n->cnt;
		if (nosplit <= 1.0f + bcost / calc_area(n->min, n->max)) {
			//dprintf("no split of sid: %d, cnt: %d\n",
			//  n->sid, n->cnt);
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else {
				break;
			}
		}

		// Partition in l and r of split plane
		unsigned int l = n->sid;
		unsigned int r = l + n->cnt;
		float invda = vec3_getc(invd, baxis); // Axis only
		float nmina = vec3_getc(n->min, baxis);
		for (unsigned int i = 0; i < n->cnt; i++) {
			unsigned int id = imap[l];
			struct aabb *a = &aabbs[id];
			float bin =
			  ((vec3_getc(a->min, baxis) + vec3_getc(a->max, baxis))
			    * 0.5f - nmina) * invda;
			if ((unsigned int)min(max((int)bin, 0),
			  INTERVAL_CNT - 1) <= bpos) {
				l++;
			 } else {
				// Swap tri indices
				imap[l] = imap[--r];
				imap[r] = id;
			}
		}

		unsigned int lcnt = l - n->sid;
		if (lcnt == 0 || lcnt == n->cnt) {
			//dprintf("one side of the partition was empty at sid: %d, l: %d, r: %d\n",
			//  n->sid, lcnt, n->cnt - lcnt);
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
		left->min = blmin;
		left->max = blmax;

		struct bnode *right = &nodes[ncnt + 1];
		right->sid = r;
		right->cnt = n->cnt - lcnt;
		right->min = brmin;
		right->max = brmax;

		// Update current (interior) node's child link
		n->sid = ncnt; // Right child is implicitly + 1
		n->cnt = 0; // No leaf, no tris or instance

		// Push right child on stack and continue with left
		assert(spos < 64);
		stack[spos++] = ncnt + 1;
		nid = ncnt;

		ncnt += 2; // Account for two new nodes
	}

	return ncnt;
}

void convert_b2(struct b2node *tgt, struct bnode *src)
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

			assert(spos < 63);
			stack[spos++] = sn + 1; // Right src node
			stack[spos++] = tn - 1; // Curr tgt (prnt of right)
		}
	}
}

void combine_leafs(struct bnode *src, unsigned int cnt)
{

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
                   unsigned int triid, unsigned int instid)
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
		h->id = (triid << INST_ID_BITS) | instid;
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
				  imap[n->start + i], instid);

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
	for (unsigned int i = 0; i < idcnt * 2; i++)
		rd->nodes[i].l = rd->nodes[i].r = 0;

	// Reserve a node cnt slot for each inst + 1 tlas
	// We do not know how many instances have a unique mesh yet
	rd->nodecnts = malloc(maxinsts + 1 * sizeof(*rd->nodecnts));

	// Start of tlas index map and tlas nodes * 2
	rd->tlasofs = maxtris;

	// Total number of blas + 1 tlas
	rd->bvhcnt = 0;
}

void rend_release(struct rdata *rd)
{
	free(rd->nodecnts);
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
		unsigned int triofs = ri->triofs;
		struct b2node *rn = &rd->nodes[triofs << 1]; // Root node
		if (rn->l + rn->r == 0) { // Not processed yet
			unsigned int tricnt = ri->tricnt;
			struct rtri *tp = &rd->tris[triofs];
			unsigned int *ip = &rd->imap[triofs];
			//struct aabb aabbs[tricnt]; // On stack, can break
			struct aabb *aabbs = malloc(tricnt * sizeof(*aabbs));
			struct aabb *ap = aabbs;
			struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
			struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
			dprintf("Creating blas for inst: %d, ofs: %d, cnt: %d, addr: %p\n",
			  j, triofs, tricnt, (void *)rn);
			for (unsigned int i = 0; i < tricnt; i++) {
				ap->min = ap->max = tp->v0;
				ap->min = vec3_min(ap->min, tp->v1);
				ap->max = vec3_max(ap->max, tp->v1);
				ap->min = vec3_min(ap->min, tp->v2);
				ap->max = vec3_max(ap->max, tp->v2);
				rmin = vec3_min(rmin, ap->min);
				rmax = vec3_max(rmax, ap->max);
				*ip++ = i;
				tp++;
				ap++;
			}

			//struct bnode nodes[tricnt << 1]; // On stack, can break
			struct bnode *nodes = malloc((tricnt << 1) * sizeof(*nodes));
			rd->nodecnts[rd->bvhcnt++] = build_bvh(
			  nodes, aabbs, &rd->imap[triofs],
			  tricnt, rmin, rmax);
			printf("blas has %d nodes\n",
			  rd->nodecnts[rd->bvhcnt - 1]);
			convert_b2(&rd->nodes[triofs << 1], nodes);
			free(nodes);
			free(aabbs);
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

	struct bnode nodes[rd->instcnt << 1]; // On stack, can break
	rd->nodecnts[rd->bvhcnt + 1] = build_bvh(
	  nodes, rd->aabbs, &rd->imap[tlasofs], rd->instcnt, rmin, rmax);
	printf("tlas has %d nodes\n", rd->nodecnts[rd->bvhcnt + 1]);
	convert_b2(&rd->nodes[tlasofs << 1], nodes);
}

struct vec3 calc_nrm(float u, float v, struct rnrm *rn,
                        float inv_transpose[16])
{
	struct vec3 nrm = vec3_add(vec3_scale(rn->n1, u),
	  vec3_add(vec3_scale(rn->n2, v), vec3_scale(rn->n0, 1.0f - u - v)));
	return vec3_unit(mat4_muldir(inv_transpose, nrm));
}

struct vec3 calc_fnrm(struct rtri *t, float inv_transpose[16])
{
	struct vec3 nrm = vec3_unit(vec3_cross(vec3_sub(t->v0, t->v1),
	  vec3_sub(t->v0, t->v2)));
	return vec3_unit(mat4_muldir(inv_transpose, nrm));
}

void create_onb(struct vec3 *b1, struct vec3 *b2, struct vec3 n)
{
	float sign = copysignf(1.0f, n.z);
	float a = -1.0f / (sign + n.z);
	float b = n.x * n.y * a;
	*b1 = (struct vec3){1.0f + sign * n.x * n.x * a, sign * b, -sign * n.x};
	*b2 = (struct vec3){b, sign + n.y * n.y * a, -n.y};
}

struct vec3 rand_hemicos(float r0, float r1)
{
	float phi = TWO_PI * r0;
	float sr1 = sqrtf(r1);
	return (struct vec3){cosf(phi) * sr1, sinf(phi) * sr1, sqrtf(1 - r1)};
}

struct vec3 trace(struct vec3 o, struct vec3 d, const struct rdata *rd)
{
	struct hit h = {.t = FLT_MAX};

	intersect_tlas(&h, o, d, rd->nodes, rd->imap, rd->insts,
	  rd->tris, rd->tlasofs);

	struct vec3 c = rd->bgcol;
	if (h.t < FLT_MAX) {
		unsigned int instid = h.id & INST_ID_MASK;
		unsigned int triid = h.id >> INST_ID_BITS;
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

struct vec3 trace2(struct vec3 o, struct vec3 d, const struct rdata *rd,
                   unsigned char depth)
{
	if (depth >= 2)
		return rd->bgcol;

	struct hit h = {.t = FLT_MAX};

	intersect_tlas(&h, o, d, rd->nodes, rd->imap, rd->insts,
	  rd->tris, rd->tlasofs);

	if (h.t == FLT_MAX)
		return rd->bgcol;

	unsigned int instid = h.id & INST_ID_MASK;
	unsigned int triid = h.id >> INST_ID_BITS;
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
	//struct vec3 nrm = calc_fnrm(&rd->tris[ri->triofs + triid], it);
	if (vec3_dot(nrm, d) > 0.0f)
		nrm = vec3_neg(nrm);

	// New origin and direction
	struct vec3 pos = vec3_add(o, vec3_scale(d, h.t));

	struct vec3 ta, bta;
	create_onb(&ta, &bta, nrm);

	struct vec3 dir = rand_hemicos(pcg_randf(), pcg_randf());

	dir = vec3_add(vec3_add(vec3_scale(ta, dir.x), vec3_scale(bta, dir.y)),
	  vec3_scale(nrm, dir.z));

	//float cos_theta = vec3_dot(nrm, dir);
	//float pdf = cos_theta / PI;

	//struct vec3 brdf = vec3_scale(rd->mtls[mtlid].col, INV_PI);
	struct vec3 brdf = rd->mtls[mtlid].col;

	struct vec3 irr = trace2(vec3_add(pos, vec3_scale(dir, 0.001)), dir,
	  rd, depth + 1);

	//return vec3_scale(vec3_mul(brdf, irr), cos_theta / pdf);
	//return vec3_mul(brdf, irr);
	return vec3_mul(
	  vec3_scale(vec3_add(nrm, (struct vec3){1.0f, 1.0f, 1.0f}), 0.5f),
	  vec3_mul(brdf, irr));
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

	struct vec3 *acc = rd->acc;
	unsigned int *buf = rd->buf;
	unsigned int w = rd->view.w;
	unsigned int bs = rd->blksz;

	float rspp = 1.0f / (1.0f + rd->samplecnt);

	while (true) {
		int blk = __atomic_fetch_add(&rd->blknum, 1, __ATOMIC_SEQ_CST);
		if (blk >= blkcnt)
			break;
		unsigned int bx = (blk % blksx) * bs;
		unsigned int by = (blk / blksx) * bs;
		for (unsigned int j = 0; j < bs; j++) {
			unsigned int y = by + j;
			unsigned int yofs = w * y;
			for (unsigned int i = 0; i < bs; i++) {
				unsigned int x = bx + i;
				struct vec3 p = vec3_add(tl, vec3_add(
				  vec3_scale(dx, x), vec3_scale(dy, y)));
				p = vec3_add(p,
				  vec3_add(vec3_scale(dx, pcg_randf() - 0.5f),
				    vec3_scale(dy, pcg_randf() - 0.5f)));

				struct vec3 c =
				  //trace(eye, vec3_unit(vec3_sub(p, eye)), rd);
				  trace2(eye, vec3_unit(vec3_sub(p, eye)), rd,
				    0);

				acc[yofs + x] = vec3_add(acc[yofs + x], c);
				c = vec3_scale(acc[yofs + x], rspp);

				buf[yofs + x] = 0xffu << 24 |
				  min(255, (unsigned int)(255 * c.x)) << 16 |
				  min(255, (unsigned int)(255 * c.y)) <<  8 |
				  min(255, (unsigned int)(255 * c.z));
			}
		}
	}

	return 0;
}

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aabb.h"
#include "mat4.h"
#include "rend.h"
#include "types.h"
#include "util.h"

#define INTERVAL_CNT  8

struct split {
	float          cost;
	float          pos;
	unsigned char  axis;
};

struct interval {
	struct aabb   box;
	unsigned int  cnt;
};

struct tnode {
	struct vec3   min;
	uint32_t      id;
	struct vec3   max;
	uint32_t      children;
};

struct ray {
	struct vec3  ori;
	float        pad0;
	struct vec3  dir;
	float        pad1;
	struct vec3  idir;
	float        oad2;
};

struct hit {
	float     t;
	float     u;
	float     v;
	uint32_t  e;
};

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
		struct interval *ivp = ivs;
		for (unsigned int i = 0; i < INTERVAL_CNT; i++) {
			ivp->cnt = 0;
			aabb_init(&ivp->box);
			ivp++;
		}

		// Count objects per interval and find the combined bounds
		float delta = INTERVAL_CNT / (maxc - minc);
		const unsigned int *ip = &imap[n->sid];
		for (unsigned int i = 0; i < n->cnt; i++) {
			const struct aabb *a = &aabbs[*ip++];
			float c = 0.5f * (vec3_getc(a->min, axis) +
			  vec3_getc(a->max, axis));
			unsigned int iv_id  = (unsigned int)max(0,
			  min(INTERVAL_CNT - 1, (c - minc) * delta));
			struct interval *iv = &ivs[iv_id];
			struct aabb *iv_box = &iv->box;
			aabb_grow(iv_box, a->min);
			aabb_grow(iv_box, a->max);
			iv->cnt++;
		}

		// Calc l/r areas and cnts for each interval separating plane
		float lareas[INTERVAL_CNT - 1]; // Areas left
		float rareas[INTERVAL_CNT - 1];
		unsigned int lcnts[INTERVAL_CNT - 1];
		unsigned int rcnts[INTERVAL_CNT - 1];
		struct aabb lbox;
		struct aabb rbox;
		unsigned int ltotcnt = 0; // Total cnt
		unsigned int rtotcnt = 0;
		aabb_init(&lbox);
		aabb_init(&rbox);
		for (unsigned int i = 0; i < INTERVAL_CNT - 1; i++) {
			// From left
			struct interval *iv = &ivs[i];
			ltotcnt += iv->cnt;
			lcnts[i] = ltotcnt;
			aabb_combine(&lbox, &lbox, &iv->box);
			lareas[i] = aabb_calcarea(&lbox);
			// From right
			iv = &ivs[INTERVAL_CNT - 1 - i];
			rtotcnt += iv->cnt;
			rcnts[INTERVAL_CNT - 2 - i] = rtotcnt;
			aabb_combine(&rbox, &rbox, &iv->box);
			rareas[INTERVAL_CNT - 2 - i] = aabb_calcarea(&rbox);
		}

		// Find best surface area cost for interval planes
		delta = 1.0f / delta;
		for (unsigned int i = 0; i < INTERVAL_CNT - 1; i++) {
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

void build_blas(struct bnode *nodes, struct aabb *aabbs, unsigned int *imap,
                 unsigned int cnt, const struct aabb *root)
{
#define STACK_SIZE  64
	struct bnode *stack[STACK_SIZE];
	unsigned int spos = 0;

	// Prepare root node
	struct bnode *n = nodes;
	n->min = root->min;
	n->max = root->max;
	n->sid = 0;
	n->cnt = cnt;

	// Node cnt is root + 1 empty node for cache alignment
	unsigned int ncnt = 2;

	while (n) {
		/*// Split at longest axis
		struct vec3 e = vec3_scale(vec3_sub(n->max, n->min), 0.5f);
		unsigned char a = e.x > e.y && e.x > e.z ? 0 :
		  (e.y > e.x && e.y > e.z ? 1 : 2); // Split axis
		float s = vec3_getc(n->min, a) + vec3_getc(e, a); // Split pos
		*/

		// Find best split according to SAH
		struct split best = find_intervalsplit(n, aabbs, imap);
		float nosplit = n->cnt * aabb_calcarea(&(struct aabb){
		  .min = n->min, .max = n->max});
		if (nosplit <= best.cost) {
			//printf("no split of sid: %d, cnt: %d\n",
			//  n->sid, n->cnt);
			if (spos > 0) {
				n = stack[--spos];
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
			//printf("one side of the partition was empty\n");
			if (spos > 0) {
				n = stack[--spos];
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

		ncnt += 2; // Account for two new nodes

		// Push right child on stack and continue with left
		assert(spos < STACK_SIZE);
		stack[spos++] = right;
		n = left;
	}
}

unsigned int find_bestnode(struct tnode *nodes, unsigned int id,
                           unsigned int *node_indices,
                           unsigned int node_indices_cnt)
{
	float best_cost = FLT_MAX;
	unsigned int best_id = id;

	unsigned int cid = node_indices[id];
	struct vec3 cmin = nodes[cid].min;
	struct vec3 cmax = nodes[cid].max;

	// Find smallest combined aabb of current node and any other node
	for (unsigned int i = 0; i < node_indices_cnt; i++) {
		if (id != i) {
			unsigned int other_id = node_indices[i];
			struct vec3 mi = vec3_min(cmin, nodes[other_id].min);
			struct vec3 ma = vec3_max(cmax, nodes[other_id].max);
			float cost = aabb_calcarea(&(struct aabb){.min = mi,
			  .max = ma});
			if (cost < best_cost) {
				best_cost = cost;
				best_id = i;
			}
		}
	}

	return best_id;
}

// Walter et al: Fast Agglomerative Clustering for Rendering
unsigned int cluster_nodes(struct tnode *nodes, unsigned int node_id,
                           unsigned int *node_indices,
                           unsigned int node_indices_cnt)
{
	unsigned int a = 0;
	unsigned int b = find_bestnode(nodes, a, node_indices,
	  node_indices_cnt);

	while (node_indices_cnt > 1) {
		unsigned int c = find_bestnode(nodes, b, node_indices,
		  node_indices_cnt);
		if (a == c) {
			unsigned int id_a = node_indices[a];
			unsigned int id_b = node_indices[b];

			struct tnode *na = &nodes[id_a];
			struct tnode *nb = &nodes[id_b];

			// Claim new node which is combination of node A and B
			struct tnode *nn = &nodes[node_id];
			nn->min = vec3_min(na->min, nb->min);
			nn->max = vec3_max(na->max, nb->max);

			// Each child node index gets 16 bits
			nn->children = (id_b << 16) | id_a;

			// Replace node A with newly created combined node
			node_indices[a] = node_id--;

			// Remove node B by replacing its slot with last node
			node_indices[b] = node_indices[--node_indices_cnt];

			// Restart the loop for remaining nodes
			b = find_bestnode(nodes, a, node_indices,
			  node_indices_cnt);
		} else {
			// Best match B we found for A had a better match in C
			// Hence, A and B are not best matches, try B and C
			a = b;
			b = c;
		}
	}

	// Index of root node - 1
	return node_id;
}

void build_tlas(struct tnode *nodes, struct rinst *insts, struct aabb *aabbs,
                unsigned int instcnt)
{
	unsigned int node_indices[instcnt];
	unsigned int node_indices_cnt = 0;

	// Nodes will be placed beginning at the back of the array
	unsigned int node_id = 2 * instcnt - 2;

	// Construct a leaf node for each instance
	for (unsigned int i = 0; i < instcnt; i++) {
		if (!hasflags(insts[i].flags, DISABLED)) {
			struct tnode *n = &nodes[node_id];

			struct aabb *a = &aabbs[i];
			n->min = a->min;
			n->max = a->max;

			// Interior nodes will have at least one child > 0
			n->children = 0;
			n->id = i;

			node_indices[node_indices_cnt++] = node_id--;
		}
	}

	node_id = cluster_nodes(nodes, node_id, node_indices, node_indices_cnt);

	if(node_id + 1 > 0)
		// Some leafs were not used, move root node to front
		nodes[0] = nodes[node_id + 1];
}

float intersect_aabb(const struct ray *r, float tfar,
                     struct vec3 min_ext, struct vec3 max_ext)
{
	float tx0 = (min_ext.x - r->ori.x) * r->idir.x;
	float tx1 = (max_ext.x - r->ori.x) * r->idir.x;
	float tmin = min(tx0, tx1);
	float tmax = max(tx0, tx1);

	float ty0 = (min_ext.y - r->ori.y) * r->idir.y;
	float ty1 = (max_ext.y - r->ori.y) * r->idir.y;
	tmin = max(tmin, min(ty0, ty1));
	tmax = min(tmax, max(ty0, ty1));

	float tz0 = (min_ext.z - r->ori.z) * r->idir.z;
	float tz1 = (max_ext.z - r->ori.z) * r->idir.z;
	tmin = max(tmin, min(tz0, tz1));
	tmax = min(tmax, max(tz0, tz1));

	if (tmin <= tmax && tmin < tfar && tmax >= 0.0f)
		return tmin;
	else
		return FLT_MAX;
}

void intersect_tri(struct hit *h, const struct ray *r, const struct rtri *tris,
                   uint32_t triid, uint32_t instid)
{
	// Vectors of two edges sharing v0
	const struct rtri *t = &tris[triid];
	const struct vec3 v0 = t->v0;
	const struct vec3 e0 = vec3_sub(t->v1, v0);
	const struct vec3 e1 = vec3_sub(t->v2, v0);

	// Calculate determinant
	const struct vec3 pv = vec3_cross(r->dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return;

	float idet = 1.0f / det;

	// Distance v0 to origin
	const struct vec3 tv = vec3_sub(r->ori, v0);

	// Calculate param u and test bounds
	float u = vec3_dot(tv, pv) * idet;
	if (u < 0.0f || u > 1.0f)
		return;

	// Prepare to test for v
	const struct vec3 qv = vec3_cross(tv, e0);

	// Calculate param v and test bounds
	float v = vec3_dot(r->dir, qv) * idet;
	if (v < 0.0f || u + v > 1.0f)
		return;

	// Calculate dist
	float dist = vec3_dot(e1, qv) * idet;
	if (dist > 0.0f && dist < h->t) {
		h->t = dist;
		h->u = u;
		h->v = v;
		h->e = triid << 16 | instid;
	}
}

void intersect_blas(struct hit *h, const struct ray *r,
                    const struct bnode *blas, const struct rtri *tris,
                    const unsigned int *imap, unsigned int instid)
{
#define STACK_SIZE 64
	const struct bnode *stack[STACK_SIZE];
	unsigned int spos = 0;

	const struct bnode *n = blas;

	while (true) {
		if (n->cnt > 0) {
			// Leaf, check triangles
			for (unsigned int i = 0; i < n->cnt; i++)
				intersect_tri(h, r, tris, imap[n->sid + i],
				  instid);

			// Pop next node from stack if something is left
			if (spos > 0)
				n = stack[--spos];
			else
				return;
		} else {
			// Interior node, check children, right child is + 1
			const struct bnode *c0 = &blas[n->sid];
			const struct bnode *c1 = &blas[n->sid + 1];

			float d0 = intersect_aabb(r, h->t, c0->min, c0->max);
			float d1 = intersect_aabb(r, h->t, c1->min, c1->max);

			if (d0 > d1) {
				float t = d0;
				d0 = d1;
				d1 = t;

				const struct bnode *tc = c0;
				c0 = c1;
				c1 = tc;
			}

			if (d0 == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					n = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				n = c0;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < STACK_SIZE);
					stack[spos++] = c1;
				}
			}
		}
	}
}

void intersect_tlas(struct hit *h, const struct ray *r, const struct rdata *rd)
{
#define STACK_SIZE 64
	const struct tnode *stack[STACK_SIZE];
	unsigned int spos = 0;

	const struct tnode *n = rd->tlas;

	while (true) {
		if (n->children == 0) {
			// Leaf, check instance blas
			const struct rinst *ri = &rd->insts[n->id];

			// Transform ray into object space of instance
			float inv[16];
			mat4_from3x4(inv, ri->globinv);
			struct ray ros = (struct ray){
			  .ori = mat4_mulpos(inv, r->ori),
			  .dir = mat4_muldir(inv, r->dir)};
			ros.idir = (struct vec3){
			  1.0f / ros.dir.x, 1.0f / ros.dir.y, 1.0f / ros.dir.z};

			unsigned int o = ri->triofs;
			intersect_blas(h, &ros, &rd->blas[o << 1], &rd->tris[o],
			               &rd->imap[o], n->id);

			// Pop next node from stack if something is left
			if (spos > 0)
				n = stack[--spos];
			else
				return;
		} else {
			// Interior node, check children
			const struct tnode *c0 = &rd->tlas[n->children & 0xff];
			const struct tnode *c1 = &rd->tlas[n->children >> 16];

			float d0 = intersect_aabb(r, h->t, c0->min, c0->max);
			float d1 = intersect_aabb(r, h->t, c1->min, c1->max);

			if (d0 > d1) {
				float t = d0;
				d0 = d1;
				d1 = t;

				const struct tnode *tc = c0;
				c0 = c1;
				c1 = tc;
			}

			if (d0 == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					n = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				n = c0;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < STACK_SIZE);
					stack[spos++] = c1;
				}
			}
		}
	}
}

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = emalloc(maxmtls * sizeof(*rd->mtls));
	rd->tris = emalloc(maxtris * sizeof(*rd->tris));
	rd->nrms = emalloc(maxtris * sizeof(*rd->nrms));
	rd->imap = emalloc(maxtris * sizeof(*rd->imap));
	rd->insts = emalloc(maxinsts * sizeof(*rd->insts));
	rd->aabbs = emalloc(maxinsts * sizeof(*rd->aabbs));
	rd->blas = emalloc(2 * maxtris * sizeof(*rd->blas));
	memset(rd->blas, 0, 2 * maxtris * sizeof(*rd->blas));
	rd->tlas = emalloc(2 * maxinsts * sizeof(*rd->tlas));
	rd->tlasofs = 2 * maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->tlas);
	free(rd->blas);
	free(rd->aabbs);
	free(rd->insts);
	free(rd->imap);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
}

void rend_prepstatic(struct rdata *rd)
{
	for (unsigned int j = 0; j < rd->instcnt; j++) {
		struct rinst *ri = &rd->insts[j];
		struct bnode *rn = &rd->blas[ri->triofs << 1]; // Root node
		if (rn->cnt + rn->sid == 0) { // Not processed yet
			printf("Creating blas for inst: %d, ofs: %d, cnt: %d\n",
			  j, ri->triofs, ri->tricnt);
			struct aabb root;
			aabb_init(&root);
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
				root.min = vec3_min(ap->min, root.min);
				root.max = vec3_max(ap->max, root.max);
				*ip++ = i;
				ap++;
				tp++;
			}
			build_blas(rn, aabbs, &rd->imap[ri->triofs],
			  ri->tricnt, &root);
		}
	}
}

void rend_prepdynamic(struct rdata *rd)
{
	build_tlas(rd->tlas, rd->insts, rd->aabbs, rd->instcnt);
}

void rend_prepdynamic2(struct rdata *rd)
{
	struct aabb *ap = rd->aabbs; // World space aabbs of instances
	struct aabb root;
	aabb_init(&root);
	unsigned int *ip = &rd->imap[rd->tlasofs];
	for (unsigned int i = 0; i < rd->instcnt; i++) {
		root.min = vec3_min(ap->min, root.min);
		root.max = vec3_max(ap->max, root.max);
		*ip++ = i;
		ap++;
	}

	build_blas(&rd->blas[rd->tlasofs], rd->aabbs, &rd->imap[rd->tlasofs],
	  rd->instcnt, &root);
}

struct vec3 calc_nrm(float u, float v, struct rnrm *rn,
                        float inv_transpose[16])
{
	struct vec3 nrm = vec3_add(vec3_scale(rn->n1, u),
	  vec3_add(vec3_scale(rn->n2, v), vec3_scale(rn->n0, 1 - u - v)));
	return vec3_unit(mat4_muldir(inv_transpose, nrm));
}

void rend_render(void *dst, struct rdata *rd)
{
#define BLK_SZ  4
	struct vec3 eye = rd->cam.eye;
	struct vec3 dx = rd->view.dx;
	struct vec3 dy = rd->view.dy;
	struct vec3 tl = rd->view.tl;

	uint32_t *buf = dst;
	for (unsigned int j = 0; j < rd->view.h; j += BLK_SZ) {
	  for (unsigned int i = 0; i < rd->view.w; i += BLK_SZ) {
	    for (unsigned int y = 0; y < BLK_SZ; y++) {
	      for (unsigned x = 0; x < BLK_SZ; x++) {
			struct vec3 p = vec3_add(tl, vec3_add(
			  vec3_scale(dx, i + x), vec3_scale(dy, j + y)));

			struct ray r = (struct ray){.ori = eye,
			  .dir = vec3_unit(vec3_sub(p, eye))};
			r.idir = (struct vec3){
			  1.0f / r.dir.x, 1.0f / r.dir.y, 1.0f / r.dir.z};

			struct hit h = (struct hit){.t = FLT_MAX};
			intersect_tlas(&h, &r, rd);

			struct vec3 c = rd->bgcol;
			if (h.t < FLT_MAX) {
				unsigned int instid = h.e & 0xffff;
				unsigned int triid = h.e >> 16;
				struct rinst *ri = &rd->insts[instid];
				struct rnrm *rn = &rd->nrms[ri->triofs + triid];
				unsigned int mtlid = rn->mtlid;

				// Inverse transpose, dir mul can be 3x4 only
				float it[16];
				float *rt = ri->globinv;
				for (int j = 0; j < 4; j++)
					for (int i = 0; i < 3; i++)
						it[4 * j + i] = rt[4 * i + j];

				struct vec3 nrm = calc_nrm(h.u, h.v, rn, it);
				nrm = vec3_scale(vec3_add(nrm,
				  (struct vec3){1, 1, 1}), 0.5f);
				c = vec3_mul(nrm, rd->mtls[mtlid].col);
				//c = nrm;
			}

			unsigned int cr = min(255, (unsigned int)(255 * c.x));
			unsigned int cg = min(255, (unsigned int)(255 * c.y));
			unsigned int cb = min(255, (unsigned int)(255 * c.z));
			buf[rd->view.w * (j + y) + (i + x)] =
			  0xff << 24 | cr << 16 | cg << 8 | cb;
	      }
	    }
	  }
	}
}

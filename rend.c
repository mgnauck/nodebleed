#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "aabb.h"
#include "mat4.h"
#include "rend.h"
#include "types.h"
#include "util.h"

#define MIN_SPLIT_CNT  3
#define INTERVAL_CNT  16

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
	struct vec3  dir;
	struct vec3  idir;
};

struct hit {
	float     t;
	float     u;
	float     v;
	uint32_t  e;
};

void update_bounds(struct bnode *n, const struct rtri *tris,
                   const unsigned int *imap)
{
	struct aabb a;
	aabb_init(&a);

	const unsigned int *ip = &imap[n->sid];
	for (unsigned int i = 0; i < n->cnt; i++) {
		const struct rtri *t = &tris[*ip++];
		aabb_grow(&a, t->v0);
		aabb_grow(&a, t->v1);
		aabb_grow(&a, t->v2);
	}

	aabb_pad(&a);
	n->min = a.min;
	n->max = a.max;

	/*
	printf("min: %6.3f, %6.3f, %6.3f\n", a.min.x, a.min.y, a.min.z);
	printf("max: %6.3f, %6.3f, %6.3f\n", a.max.x, a.max.y, a.max.z);
	printf("sid: %d, cnt: %d\n\n", n->sid, n->cnt);
	*/
}

struct split find_intervalsplit(const struct bnode *n,
                                const struct rtri *tris,
                                const unsigned int *imap,
                                const struct vec3 *centers)
{
	struct split best = {.cost = FLT_MAX};
	for (unsigned char axis = 0; axis < 3; axis++) {
		// Calc center bounds
		float minc = FLT_MAX;
		float maxc = -FLT_MAX;
		const unsigned int *ip = &imap[n->sid];
		for (unsigned int i = 0; i < n->cnt; i++) {
			float c = vec3_getc(centers[*ip++], axis);
			minc = min(minc, c);
			maxc = max(maxc, c);
		}

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
		ip = &imap[n->sid];
		for (unsigned int i = 0; i < n->cnt; i++) {
			unsigned int iv_id  = (unsigned int)min(
			  INTERVAL_CNT - 1,
			  (vec3_getc(centers[*ip], axis) - minc) * delta);
			struct interval *iv = &ivs[iv_id];
			struct aabb *iv_box = &iv->box;
			const struct rtri *tri = &tris[*ip++];
			aabb_grow(iv_box, tri->v0);
			aabb_grow(iv_box, tri->v1);
			aabb_grow(iv_box, tri->v2);
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

void subdivide_node(struct bnode *n, struct bnode *blas,
                    const struct rtri *tris, unsigned int *imap,
                    const struct vec3 *centers, unsigned int *ncnt)
{
	// Calc if we need to split
	struct split best = find_intervalsplit(n, tris, imap, centers);
	float nosplit = n->cnt * aabb_calcarea(&(struct aabb){n->min, n->max});
	if (nosplit <= best.cost) {
		printf("no split of sid: %d, cnt: %d\n", n->sid, n->cnt);
		return;
	}

	// Partition data into left and right of found split pos
	int l = n->sid;
	int r = l + n->cnt - 1;
	while (l <= r) {
		if (vec3_getc(centers[imap[l]], best.axis) < best.pos) {
			l++;
		} else {
			unsigned int t = imap[l];
			imap[l] = imap[r];
			imap[r--] = t;
		}
	}

	// Stop if one side of the partition is empty
	unsigned int lcnt = l - n->sid;
	if (l == 0 || lcnt == n->cnt)
		return;

	// Init children
	struct bnode *left = &blas[*ncnt];
	left->sid = n->sid;
	left->cnt = lcnt;
	update_bounds(left, tris, imap);

	struct bnode *right = &blas[*ncnt + 1];
	right->sid = l;
	right->cnt = n->cnt - lcnt;
	update_bounds(right, tris, imap);

	// Update current (interior) node's child links
	n->sid = *ncnt; // Right child is implicitly + 1
	n->cnt = 0; // No leaf, no tris

	*ncnt += 2; // Account for two new blas

	if (lcnt > MIN_SPLIT_CNT)
		subdivide_node(left, blas, tris, imap, centers, ncnt);
	if (right->cnt > MIN_SPLIT_CNT)
		subdivide_node(right, blas, tris, imap, centers, ncnt);
}

void build_blas(struct bnode *blas, const struct rtri *tris,
               unsigned int *imap, unsigned int tricnt)
{
	struct vec3 centers[tricnt];

	unsigned int *ip = imap;
	struct vec3 *cp = centers;
	const struct rtri *tp = tris;
	for (unsigned int i = 0; i < tricnt; i++) {
		*ip++ = i;
		*cp++ = vec3_scale(vec3_add(tp->v0,
		          vec3_add(tp->v1, tp->v2)), 0.3333f);
		tp++;
	}

	// Root
	blas->sid = 0;
	blas->cnt = tricnt;
	update_bounds(blas, tris, imap);

	unsigned int ncnt = 2; // Root + 1 skipped node for mem alignment
	subdivide_node(blas, blas, tris, imap, centers, &ncnt);
}

unsigned int find_bestnode(struct tnode *nodes, unsigned int id,
                           unsigned int *node_indices,
                           unsigned int node_indices_cnt)
{
	float best_cost = FLT_MAX;
	unsigned int best_id;

	unsigned int cid = node_indices[id];
	struct vec3 cmin = nodes[cid].min;
	struct vec3 cmax = nodes[cid].max;

	// Find smallest combined aabb of current node and any other node
	for (unsigned int i = 0; i < node_indices_cnt; i++) {
		if (id != i) {
			unsigned int other_id = node_indices[i];
			struct vec3 mi = vec3_min(cmin, nodes[other_id].min);
			struct vec3 ma = vec3_max(cmax, nodes[other_id].max);
			float cost = aabb_calcarea(&(struct aabb){mi, ma});
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
	struct vec3 t0 = vec3_mul(vec3_sub(min_ext, r->ori), r->idir);
	struct vec3 t1 = vec3_mul(vec3_sub(max_ext, r->ori), r->idir);

	float tmin = max(vec3_maxc(vec3_min(t0, t1)), EPS);
	float tmax = min(vec3_minc(vec3_max(t1, t0)), tfar);

	return tmin <= tmax ? tmin : FLT_MAX;
}

void intersect_tri(struct hit *h, const struct ray *r, const struct vec3 *v0,
                   const struct vec3 *v1, const struct vec3 *v2, uint32_t id)
{
	// Vectors of two edges sharing v0
	const struct vec3 e0 = vec3_sub(*v1, *v0);
	const struct vec3 e1 = vec3_sub(*v2, *v0);

	// Calculate determinat and u param later on
	const struct vec3 pv = vec3_cross(r->dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return;

	float idet = 1.0f / det;

	// Distance v0 to origin
	const struct vec3 tv = vec3_sub(r->ori, *v0);

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
	if (dist > EPS && dist < h->t) {
		h->t = dist;
		h->u = u;
		h->v = v;
		h->e = id;
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
			const unsigned int *ip = &imap[n->sid];
			for (unsigned int i = 0; i < n->cnt; i++) {
				unsigned int triid = *ip++;
				const struct rtri *t = &tris[triid];
				intersect_tri(h, r, &t->v0, &t->v1, &t->v2,
				  triid << 16 | instid);
			}

			// Pop next node of stack if something is left
			if (spos > 0)
				n = stack[--spos];
			else
				return;
		} else {
			// Interior node, check children, right child is + 1
			const struct bnode *c = &blas[n->sid];
			float dist[2] = {
			  intersect_aabb(r, h->t, c[0].min, c[0].max),
			  intersect_aabb(r, h->t, c[1].min, c[1].max)};

			unsigned char near = dist[1] > dist[0] ? 0 : 1;
			if (dist[near] == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					n = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				n = &c[near]; 
				if (dist[1 - near] < FLT_MAX)
					// Put farther child on stack
					stack[spos++] = &c[1 - near];
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
			intersect_blas(h, &ros, &rd->blas[o << 1], &rd->tris[o],			  &rd->imap[o], n->id);

			// Pop next node of stack if something is left
			if (spos > 0)
				n = stack[--spos];
			else
				return;
		} else {
			// Interior node, check children
			const struct tnode *c[2] = {
			  &rd->tlas[n->children & 0xff],
			  &rd->tlas[n->children >> 16]};
			float dist[2] = {
			  intersect_aabb(r, h->t, c[0]->min, c[0]->max),
			  intersect_aabb(r, h->t, c[1]->min, c[1]->max)};

			unsigned char near = dist[1] > dist[0] ? 0 : 1;
			if (dist[near] == FLT_MAX) {
				// Did not hit any child, try the stack
				if (spos > 0)
					n = stack[--spos];
				else
					return;
			} else {
				// Continue with nearer child node
				n = c[near];
				if (dist[1 - near] < FLT_MAX)
					// Put farther child on stack
					stack[spos++] = c[1 - near];
			}
		}
	}
}
void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = malloc(maxmtls * sizeof(*rd->mtls));
	rd->tris = malloc(maxtris * sizeof(*rd->tris));
	rd->nrms = malloc(maxtris * sizeof(*rd->nrms));
	rd->imap = malloc(maxtris * sizeof(*rd->imap));
	rd->insts = malloc(maxinsts * sizeof(*rd->insts));
	rd->aabbs = malloc(maxinsts * sizeof(*rd->aabbs));
	rd->blas = calloc(2 * maxtris, sizeof(*rd->blas));
	rd->tlas = malloc(2 * maxinsts * sizeof(*rd->tlas));
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
		//printf("inst: %d, ofs: %d, cnt: %d\n",
		//  j, ri->triofs, ri->tricnt);
		unsigned int o = ri->triofs;
		struct bnode *rn = &rd->blas[o << 1]; // Root node
		if (rn->cnt + rn->sid == 0) // Not processed yet
			build_blas(rn, &rd->tris[o], &rd->imap[o], ri->tricnt);
	}
}

void rend_prepdynamic(struct rdata *rd)
{
	build_tlas(rd->tlas, rd->insts, rd->aabbs, rd->instcnt);
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
#define BLK_SZ  16
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
			//intersect_insts(&h, &r, rd);
			intersect_tlas(&h, &r, rd);

			struct vec3 c = rd->bgcol;
			if (h.t < FLT_MAX) {
				unsigned int instid = h.e & 0xffff;
				unsigned int triid = h.e >> 16;
				struct rinst *ri = &rd->insts[instid];
				struct rnrm *rn = &rd->nrms[ri->triofs + triid];
				unsigned int mtlid = rn->mtlid;

				// Inverse transpose, dir mul is 3x4 only
				float it[16];
				float *rt = ri->globinv;
				for (int j = 0; j < 4; j++)
					for (int i = 0; i < 3; i++)
						it[4 * j + i] = rt[4 * i + j];

				struct vec3 nrm = calc_nrm(h.u, h.v, rn, it);
				nrm = vec3_scale(vec3_add(nrm, (struct vec3){1, 1, 1}), 0.5f);
				//c = vec3_mul(nrm, rd->mtls[mtlid].col);
				c = nrm;
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

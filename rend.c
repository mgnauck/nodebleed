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

// Max 4096 instances
#define INST_ID_BITS  12
#define INST_ID_MASK  0xfff

#define INTERVAL_CNT  16 // Binning intervals

#define LEAF_TRI_CNT    4

struct bnode { // bvh node, 1-wide, 32 bytes
	struct vec3   min;
	unsigned int  sid; // Start index or left child node id
	struct vec3   max;
	unsigned int  cnt; // Tri or inst cnt
};

struct hit { // 32 bytes
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

unsigned int build_bvh(struct bnode *nodes, struct aabb *aabbs,
                       unsigned int *imap, unsigned int cnt,
                       struct vec3 rootmin, struct vec3 rootmax)
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
			} else
				break;
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
			} else
				break;
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

unsigned int count_nodes(struct bnode *nodes)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int nid = 0;

	unsigned int ncnt = 0;

	while (true) {
		struct bnode *n = &nodes[nid];
		ncnt++;

		if (n->cnt > 0) {
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else
				break;
		}

		nid = n->sid;

		assert(spos < 64);
		stack[spos++] = nid + 1;
	}

	return ncnt;
}

unsigned int count_mnodes(struct bmnode *nodes)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int nid = 0;

	unsigned int ncnt = 0;

	while (true) {
		struct bmnode *n = &nodes[nid];
		ncnt++;

		if (n->childcnt == 0) {
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else
				break;
		}

		nid = n->children[0];

		for (unsigned int i = 1; i < n->childcnt; i++) {
			assert(spos < 64 - n->childcnt + 1);
			stack[spos++] = n->children[i];
		}
	}

	return ncnt;
}

void print_nodes(struct bnode *nodes)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int nid = 0;

	while (true) {
		struct bnode *n = &nodes[nid];

		if (n->cnt > 0) {
			dprintf("leaf %d with sid: %d, cnt: %d\n",
			  nid, n->sid, n->cnt);
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else
				break;
		} else {
			dprintf("interior %d with 2 children: %d %d\n",
			  nid, n->sid, n->sid + 1);
		}

		nid = n->sid; // Continue with left

		assert(spos < 64);
		stack[spos++] = nid + 1; // Right
	}
}

void print_mnodes(struct bmnode *nodes)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int nid = 0;

	while (true) {
		struct bmnode *n = &nodes[nid];

		if (n->childcnt == 0) {
			dprintf("leaf %d with sid: %d, cnt: %d\n",
			  nid, n->start, n->cnt);
			if (spos > 0) {
				nid = stack[--spos];
				continue;
			} else
				break;
		} else {
			dprintf("interior %d with %d children: ",
			  nid, n->childcnt);
			for (unsigned int i = 0; i < n->childcnt; i++)
				dprintf("%d ", n->children[i]);
			dprintf("\n");
		}

		nid = n->children[0];

		for (unsigned int i = 1; i < n->childcnt; i++) {
			assert(spos < 64 - n->childcnt + 1);
			stack[spos++] = n->children[i];
		}
	}
}

unsigned int merge_leafs(struct bnode *nodes, struct bnode *n,
                         unsigned int *sid, unsigned int maxcnt,
                         unsigned int *mergecnt)
{
	if (n->cnt > 0) {
		*sid = n->sid;
		return n->cnt;
	}

	unsigned int lsid;
	unsigned int lcnt = merge_leafs(nodes, &nodes[n->sid],
	  &lsid, maxcnt, mergecnt);

	unsigned int rsid;
	unsigned int rcnt = merge_leafs(nodes, &nodes[n->sid + 1],
	  &rsid, maxcnt, mergecnt);

	*sid = lsid; // Propagate left most start index up

	if (lcnt + rcnt <= maxcnt) {
		//dprintf("Merging nodes\n");
		n->sid = *sid;
		n->cnt = lcnt + rcnt;

		*mergecnt += 1;
	}

	return lcnt + rcnt;
}

void split_leafs(struct bnode *nodes, struct bnode *n, struct aabb *aabbs,
                 unsigned int *imap, unsigned int *nodecnt, unsigned int maxcnt,
                 unsigned int *splitcnt)
{
	if (n->cnt > maxcnt) {
		//dprintf("Splitting node\n");
		struct bnode *l = &nodes[*nodecnt];
		l->sid = n->sid;
		l->cnt = maxcnt; // Split at maxcnt index
		struct vec3 mi = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 ma = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
		unsigned int *ip = &imap[l->sid];
		for (unsigned int i = 0; i < l->cnt; i++) {
			struct aabb *b = &aabbs[*ip++];
			mi = vec3_min(mi, b->min);
			ma = vec3_max(ma, b->max);
		}
		l->min = mi;
		l->max = ma;

		struct bnode *r = &nodes[*nodecnt + 1];
		r->sid = n->sid + maxcnt;
		r->cnt = n->cnt - maxcnt; // Remaining
		mi = (struct vec3){FLT_MAX, FLT_MAX, FLT_MAX};
		ma = (struct vec3){-FLT_MAX, -FLT_MAX, -FLT_MAX};
		ip = &imap[r->sid];
		for (unsigned int i = 0; i < r->cnt; i++) {
			struct aabb *b = &aabbs[*ip++];
			mi = vec3_min(mi, b->min);
			ma = vec3_max(ma, b->max);
		}
		r->min = mi;
		r->max = ma;

		n->sid = *nodecnt;
		n->cnt = 0;

		*nodecnt += 2;

		*splitcnt += 1;
	}

	if (n->cnt == 0) {
		split_leafs(nodes, &nodes[n->sid], aabbs, imap, nodecnt,
		  maxcnt, splitcnt);
		split_leafs(nodes, &nodes[n->sid + 1], aabbs, imap, nodecnt,
		  maxcnt, splitcnt);
	}
}

unsigned int convert_b2node(struct b2node *tgt, struct bnode *src)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Current src and tgt node indices
	unsigned int snid = 0;
	unsigned int tnid = 0;

	while (true) {
		struct bnode *s = &src[snid];
		struct b2node *t = &tgt[tnid++];

		memset(t, 0, sizeof(*t));

		if (s->cnt > 0) {
			t->start = s->sid;
			t->cnt = s->cnt;

			if (spos > 0) {
				// Assign curr tgt node's prnt's right index
				tgt[stack[--spos]].r = tnid;
				snid = stack[--spos];
			} else
				break;
		} else {
			snid = s->sid; // Left child

			struct bnode *lc = &src[snid];
			t->l = tnid; // Left is next node in tgt
			t->lmin = lc->min;
			t->lmax = lc->max;

			struct bnode *rc = &src[snid + 1]; // Right = left + 1
			t->rmin = rc->min;
			t->rmax = rc->max;
			// Set tgt's right child index when it gets off stack

			assert(spos < 63);
			stack[spos++] = snid + 1; // Right src node
			stack[spos++] = tnid - 1; // Curr tgt (prnt of right)
		}
	}

	return tnid;
}

unsigned int convert_bmnode(struct bmnode *tgt, struct bnode *src)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Current src and tgt node indices
	unsigned int snid = 0;
	unsigned int tnid = 0;

	// Create compacted MBVH copy of src BVH
	while (true) {
		struct bnode *s = &src[snid];
		struct bmnode *t = &tgt[tnid++];

		memset(t, 0, sizeof(*t));

		t->min = s->min;
		t->max = s->max;

		if (s->cnt > 0) {
			// Leaf node
			t->start = s->sid; // Copy primitive references
			t->cnt = s->cnt;

			if (spos > 1) {
				// Assign curr tgt node's prnt's right index
				tgt[stack[--spos]].children[1] = tnid;
				snid = stack[--spos];
			} else
				break;
		} else {
			snid = s->sid; // Continue with left src child

			// Init tgt children with left src child only
			t->children[0] = tnid;
			t->childcnt = 2;
			// Set tgt's child id for right child when it's popped

			assert(spos < 63);
			stack[spos++] = snid + 1; // Right src node
			stack[spos++] = tnid - 1; // Curr tgt (prnt of right)
		}
	}

	unsigned int ncnt = tnid;

	tnid = 0;

	// Collapse transformed but still binary src nodes into M-wide nodes
	while (true) {
		struct bmnode *n = &tgt[tnid];

		while (n->childcnt < MBVH_CHILD_CNT) {
			struct bmnode *bc = NULL; // Best child
			float bcost = 0.0f;
			unsigned int bcid;
			for (unsigned int i = 0; i < n->childcnt; i++) {
				struct bmnode *c = &tgt[n->children[i]];
				if (c->cnt == 0 && n->childcnt - 1
				  + c->childcnt <= MBVH_CHILD_CNT) {
					float cost = calc_area(c->min, c->max);
					if (cost > bcost) {
						bc = c;
						bcost = cost;
						bcid = i;
					}
				}
			}

			// Merge children of best child into curr node
			if (bc) {
				// Put first child into the newly formed gap
				n->children[bcid] = bc->children[0];
				for (unsigned int i = 1; i < bc->childcnt; i++)
					// Append the other children if more
					n->children[n->childcnt++] =
					  bc->children[i];
				ncnt--; // Replaced one node
			} else
				break; // No child found that could be collapsed
		}

		// Schedule all interior child nodes for processing
		for (unsigned int i = 0; i < n->childcnt; i++) {
			unsigned int cid = n->children[i];
			if (tgt[cid].cnt == 0) {
				assert(spos < 64);
				stack[spos++] = cid;
			}
		}

		// Retrieve next node
		if (spos > 0)
			tnid = stack[--spos];
		else
			break;
	}

	return ncnt;
}

struct distid {
	float          dist;
	//unsigned char  id;
	unsigned int   id; // TEMP
};

int comp_distid(const void *a, const void *b)
{
	return ((struct distid *)a)->dist < ((struct distid *)b)->dist ? 1 : -1;
}

// TODO
// Use SIMD version of b8node (__m256/__m256i)
// Test 8 child aabbs at once
// Embed tri data in bvh data
// Test 4 tris at once
// Clean unused intersection functions (bmnode intersection etc.)
// Make merge/split leaf functions non-recursive?

unsigned int convert_b8node(struct b8node *tgt, struct bmnode *src)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	// Current src and tgt node indices
	unsigned int snid = 0;
	unsigned int tnid = 0;

	while (true) {
		struct bmnode *s = &src[snid];
		struct b8node *t = &tgt[tnid];

		memset(t, 0, sizeof(*t));

		unsigned int cid = 0; // 'Compacted' child index, i.e. no gaps
		for (unsigned int j = 0; j < 8; j++) {
			// Copy/setup data of non-empty child nodes
			if (s->children[j]) {
				assert(j < s->childcnt);
				struct bmnode *c = &src[s->children[j]];
				t->minx[cid] = c->min.x;
				t->maxx[cid] = c->max.x;
				t->miny[cid] = c->min.y;
				t->maxy[cid] = c->max.y;
				t->minz[cid] = c->min.z;
				t->maxz[cid] = c->max.z;
				if (c->cnt > 0) {
					// Leaf node
					assert(c->cnt <= 4);
					assert(c->start <= 268435455);
					((unsigned int *)&t->children)[cid] =
					  NODE_LEAF
					  | (c->cnt - 1) << 28
					  | (c->start & TRIID_MASK);
				} else {
					// Interior node
					// Push curr tgt node id and the child
					// which id we update when popped
					assert(spos < 128 - 1);
					assert(tnid <= 0x1fffffff);
					stack[spos++] = (tnid << 3) | cid;
					// Push id of src bmnode
					stack[spos++] = s->children[j];
				}
				cid++;
			}

			// Create ordered child traversal permutation map
			// with j being current quadrant
			struct vec3 dir = {
			  j & 1 ? 1.0f : -1.0f,
			  j & 2 ? 1.0f : -1.0f,
			  j & 4 ? 1.0f : -1.0f};
			struct distid cdi[8];
			for (unsigned int i = 0; i < 8; i++) {
				cdi[i].id = i; // 3 bit child index
				if (s->children[i]) {
					struct bmnode *c = &src[s->children[i]];
					// Aabb corner of ray dir
					struct vec3 corner = {
					  j & 1 ? c->min.x : c->max.x,
					  j & 2 ? c->min.y : c->max.y,
					  j & 4 ? c->min.z : c->max.z};
					cdi[i].dist= vec3_dot(dir, corner);
				} else {
					// No child assigned, still gets sorted
					cdi[i].dist = FLT_MAX;
				}
			}

			// Sort dists thereby sorting/permuting child indices
			qsort(cdi, 8, sizeof(*cdi), comp_distid);

			// Set perm map for all children and curr quadrant
			for (unsigned int i = 0; i < 8; i++)
				((unsigned int *)&t->perm)[i]
				  |= cdi[i].id << (j * 3);
		}

		// Set all the remaining children (if any left) to empty
		for (; cid < 8; cid++) {
			t->minx[cid] = FLT_MAX;
			t->maxx[cid] = -FLT_MAX;
			t->miny[cid] = FLT_MAX;
			t->maxy[cid] = -FLT_MAX;
			t->minz[cid] = FLT_MAX;
			t->maxz[cid] = -FLT_MAX;
			((unsigned int *)&t->children)[cid] = NODE_EMPTY;
		}

		tnid++; // Claim next tgt node

		if (spos > 1) {
			// Src bmnode to continue with
			snid = stack[--spos];
			// Set former tgt child node id to curr tgt node's id
			unsigned int tm = stack[--spos];
			assert(tnid <= 0x1fffffff);
			((unsigned int *)&tgt[tm >> 3].children)[tm & 7] = tnid;
		} else
			break;
	}

	return tnid;
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

void intersect_blas3_impl_avx2(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b8node *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid,
                    bool dx, bool dy, bool dz)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	unsigned int curr = 0;

	// TODO Safer inverse ray dir calc avoiding NANs due to _mm256_cmp_ps
	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	__m256 idx8 = _mm256_set1_ps(idx);
	__m256 idy8 = _mm256_set1_ps(idy);
	__m256 idz8 = _mm256_set1_ps(idz);

	__m256 rx8 = _mm256_set1_ps(ori.x * idx);
	__m256 ry8 = _mm256_set1_ps(ori.y * idy);
	__m256 rz8 = _mm256_set1_ps(ori.z * idz);

	__m256 t8 = _mm256_set1_ps(h->t);
	__m256 z8 = _mm256_setzero_ps();

	// Ray dir sign defines how to shift the permutation map
	unsigned char s = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
		while ((curr & NODE_LEAF) == 0) {
			const struct b8node *n = &blas[curr];

			// Slab test with fused multiply sub, swap per ray dir
			__m256 tx0 = _mm256_fmsub_ps(dx ? n->minx : n->maxx,
			  idx8, rx8);
			__m256 ty0 = _mm256_fmsub_ps(dy ? n->miny : n->maxy,
			  idy8, ry8);
			__m256 tz0 = _mm256_fmsub_ps(dz ? n->minz : n->maxz,
			  idz8, rz8);

			__m256 tx1 = _mm256_fmsub_ps(dx ? n->maxx : n->minx,
			  idx8, rx8);
			__m256 ty1 = _mm256_fmsub_ps(dy ? n->maxy : n->miny,
			  idy8, ry8);
			__m256 tz1 = _mm256_fmsub_ps(dz ? n->maxz : n->minz,
			  idz8, rz8);

			__m256 tmin = _mm256_max_ps(_mm256_max_ps(_mm256_max_ps(
			  tx0, ty0), tz0), z8);
			__m256 tmax = _mm256_min_ps(_mm256_min_ps(_mm256_min_ps(
			  tx1, ty1), tz1), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
			unsigned int hitmask = _mm256_movemask_ps(hitmask8);
			unsigned int hitcnt = __builtin_popcount(hitmask);

			switch (hitcnt) {
			case 0:
				// No hit, consult stack
				if (spos > 0)
					curr = stack[--spos];
				else
					return;
				break;
			case 1:
				// Single hit, directly continue with child node
				curr = ((unsigned int *)&n->children)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Compiler warn 'decl after label'

				// More than one hit, order + compress + push
				__m256i ord8 = _mm256_srli_epi32(n->perm, s);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children,
				    ord8);

				/// TEMP
				for (unsigned int i = 0; i < 8; i++) {
					if ((hitmaskord >> i) & 1) {
						stack[spos++] =
						  ((unsigned int *)
						    &childrenord8)[i];
					}
				}

				curr = stack[--spos]; // Next node
				///

				// TODO Push ordered and compressed node ids
				// and distances to stacks at once
			}
		}

		// TODO Intersect 4 triangles at once when data is embedded

		// TEMP: Intersect all assigned tris
		const unsigned int *ip = &imap[curr & TRIID_MASK];
		for (unsigned int i = 0; i < 1 + ((curr >> 28) & 3); i++)
			intersect_tri(h, ori, dir, tris, *ip++, instid);

		// TODO After a hit, compress the stack according to nearer h->t

		if (spos > 0)
			curr = stack[--spos];
		else
			return;
	}
}

// Intersects b8nodes, temporarily used to check if 8-wide bvh is valid
void intersect_blas3_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                   const struct b8node *blas, const unsigned int *imap,
                   const struct rtri *tris, unsigned int instid,
                   bool dx, bool dy, bool dz)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	unsigned int curr = 0;

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	// Ray dir sign defines how to shift the permutation map
	unsigned char s = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
		const struct b8node *n = &blas[curr];

		for (unsigned int k = 0; k < 8; k++) {
			unsigned int j =
			  (((unsigned int *)&n->perm)[k] >> s) & 7;
			unsigned int id = ((unsigned int *)&n->children)[j];
			if (id & NODE_LEAF) {
				// Intersect all assigned tris
				const unsigned int *ip = &imap[id & TRIID_MASK];
				for (unsigned int i = 0;
				  i < 1 + ((id >> 28) & 3); i++)
					intersect_tri(h, ori, dir, tris,
					  *ip++, instid);
			} else if (id & ~NODE_EMPTY) {
				// Interior node, check child aabbs
				float t0x = (dx ? n->minx[j] : n->maxx[j])
				  * idx - rx;
				float t0y = (dy ? n->miny[j] : n->maxy[j])
				  * idy - ry;
				float t0z = (dz ? n->minz[j] : n->maxz[j])
				  * idz - rz;

				float t1x = (dx ? n->maxx[j] : n->minx[j])
				  * idx - rx;
				float t1y = (dy ? n->maxy[j] : n->miny[j])
				  * idy - ry;
				float t1z = (dz ? n->maxz[j] : n->minz[j])
				  * idz - rz;

				float tmin = max(max(t0x, t0y), max(t0z, 0.0f));
				float tmax = min(min(t1x, t1y), min(t1z, h->t));

				if (tmax >= tmin) {
					assert(spos < 128);
					stack[spos++] = id & NODEID_MASK;
				}
			}
		}

		// Pop next node from stack if something is left
		if (spos > 0)
			curr = stack[--spos];
		else
			return;
	}
}

void intersect_blas3(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b8node *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid)
{
	intersect_blas3_impl_avx2(h, ori, dir, blas, imap, tris, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

// Intersects bmnodes (m-wide, currently 8)
void intersect_blas2_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct bmnode *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid,
                    bool dx, bool dy, bool dz)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	unsigned int curr = 0;

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	while (true) {
		const struct bmnode *n = &blas[curr];

		if (n->cnt > 0) {
			// Leaf, check triangles
			for (unsigned int i = 0; i < n->cnt; i++)
				intersect_tri(h, ori, dir, tris,
				  imap[n->start + i], instid);
		} else {
			// Interior node, check child aabbs
			struct distid dist[8];
			unsigned int ccnt = 0;
			for (unsigned int i = 0; i < n->childcnt; i++) {
				// Interior node, check child aabbs
				unsigned int cid = n->children[i];
				assert(cid > 0);
				const struct bmnode *c = &blas[cid];

				float t0x = (dx ? c->min.x : c->max.x)
				  * idx - rx;
				float t0y = (dy ? c->min.y : c->max.y)
				  * idy - ry;
				float t0z = (dz ? c->min.z : c->max.z)
				  * idz - rz;

				float t1x = (dx ? c->max.x : c->min.x)
				  * idx - rx;
				float t1y = (dy ? c->max.y : c->min.y)
				  * idy - ry;
				float t1z = (dz ? c->max.z : c->min.z)
				  * idz - rz;

				float tmin = max(max(t0x, t0y), max(t0z, 0.0f));
				float tmax = min(min(t1x, t1y), min(t1z, h->t));

				if (tmax >= tmin)
					dist[ccnt++] = (struct distid){
					  .dist = tmin, .id = cid};
			}

			// Sort to descending dist order
			qsort(dist, ccnt, sizeof(*dist), comp_distid);

			// On stack
			for (unsigned int i = 0; i < ccnt; i++) {
				assert(spos < 128);
				stack[spos++] = dist[i].id;
			}
		}

		// Pop next node from stack if something is left
		if (spos > 0)
			curr = stack[--spos];
		else
			return;
	}
}

void intersect_blas2(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct bmnode *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid)
{
	intersect_blas2_impl(h, ori, dir, blas, imap, tris, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

// Intersects b2nodes (2-wide)
void intersect_blas1_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid,
                    bool dx, bool dy, bool dz)
{
	unsigned int stack[64];
	float dstack[64]; // Distance stack
	unsigned int spos = 0;

	unsigned int curr = 0;

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	while (true) {
		const struct b2node *n = &blas[curr];

		if (n->cnt > 0) {
			// Leaf, check triangles
			for (unsigned int i = 0; i < n->cnt; i++)
				intersect_tri(h, ori, dir, tris,
				  imap[n->start + i], instid);

			// Pop next node from stack if something is left
			while (spos > 0)
				if (dstack[--spos] < h->t)
					goto next_iter;
			return;
		} else {
			// Interior node, check child aabbs
			float t0xl = (dx ? n->lmin.x : n->lmax.x) * idx - rx;
			float t0yl = (dy ? n->lmin.y : n->lmax.y) * idy - ry;
			float t0zl = (dz ? n->lmin.z : n->lmax.z) * idz - rz;
			float t0xr = (dx ? n->rmin.x : n->rmax.x) * idx - rx;
			float t0yr = (dy ? n->rmin.y : n->rmax.y) * idy - ry;
			float t0zr = (dz ? n->rmin.z : n->rmax.z) * idz - rz;

			float t1xl = (dx ? n->lmax.x : n->lmin.x) * idx - rx;
			float t1yl = (dy ? n->lmax.y : n->lmin.y) * idy - ry;
			float t1zl = (dz ? n->lmax.z : n->lmin.z) * idz - rz;
			float t1xr = (dx ? n->rmax.x : n->rmin.x) * idx - rx;
			float t1yr = (dy ? n->rmax.y : n->rmin.y) * idy - ry;
			float t1zr = (dz ? n->rmax.z : n->rmin.z) * idz - rz;

			float tminl = max(max(t0xl, t0yl), max(t0zl, 0.0f));
			float tminr = max(max(t0xr, t0yr), max(t0zr, 0.0f));

			float tmaxl = min(min(t1xl, t1yl), min(t1zl, h->t));
			float tmaxr = min(min(t1xr, t1yr), min(t1zr, h->t));

			float d0 = tmaxl >= tminl ? tminl : FLT_MAX;
			float d1 = tmaxr >= tminr ? tminr : FLT_MAX;

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
				while (spos > 0)
					if (dstack[--spos] < h->t)
						goto next_iter;
				return;
			} else {
				// Continue with nearer child node
				curr = l;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < 64);
					dstack[spos] = d1;
					stack[spos++] = r;
				}
				continue;
			}
		}
next_iter:
		curr = stack[spos];
	}
}

void intersect_blas1(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *blas, const unsigned int *imap,
                    const struct rtri *tris, unsigned int instid)
{
	intersect_blas1_impl(h, ori, dir, blas, imap, tris, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

void intersect_tlas_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *nodes, /// TEMP
                    const struct b8node *mnodes, const unsigned int *imap,
                    const struct rinst *insts, const struct rtri *tris,
                    unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	unsigned int stack[64];
	float dstack[64]; // Distance stack
	unsigned int spos = 0;

	unsigned int curr = 0;

	const struct b2node *tlas = &nodes[tlasofs << 1];
	const unsigned int *tlasimap = &imap[tlasofs];

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

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
				intersect_blas3(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[o << 1], &imap[o], &tris[o], instid);
			}

			// Pop next node from stack if something is left
			while (spos > 0)
				if (dstack[--spos] < h->t)
					goto next_iter;
			return;
		} else {
			// Interior node, check child aabbs
			float t0xl = (dx ? n->lmin.x : n->lmax.x) * idx - rx;
			float t0yl = (dy ? n->lmin.y : n->lmax.y) * idy - ry;
			float t0zl = (dz ? n->lmin.z : n->lmax.z) * idz - rz;
			float t0xr = (dx ? n->rmin.x : n->rmax.x) * idx - rx;
			float t0yr = (dy ? n->rmin.y : n->rmax.y) * idy - ry;
			float t0zr = (dz ? n->rmin.z : n->rmax.z) * idz - rz;

			float t1xl = (dx ? n->lmax.x : n->lmin.x) * idx - rx;
			float t1yl = (dy ? n->lmax.y : n->lmin.y) * idy - ry;
			float t1zl = (dz ? n->lmax.z : n->lmin.z) * idz - rz;
			float t1xr = (dx ? n->rmax.x : n->rmin.x) * idx - rx;
			float t1yr = (dy ? n->rmax.y : n->rmin.y) * idy - ry;
			float t1zr = (dz ? n->rmax.z : n->rmin.z) * idz - rz;

			float tminl = max(max(t0xl, t0yl), max(t0zl, 0.0f));
			float tminr = max(max(t0xr, t0yr), max(t0zr, 0.0f));

			float tmaxl = min(min(t1xl, t1yl), min(t1zl, h->t));
			float tmaxr = min(min(t1xr, t1yr), min(t1zr, h->t));

			float d0 = tmaxl >= tminl ? tminl : FLT_MAX;
			float d1 = tmaxr >= tminr ? tminr : FLT_MAX;

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
				while (spos > 0)
					if (dstack[--spos] < h->t)
						goto next_iter;
				return;
			} else {
				// Continue with nearer child node
				curr = l;
				if (d1 != FLT_MAX) {
					// Put farther child on stack
					assert(spos < 64);
					dstack[spos] = d1;
					stack[spos++] = r;
				}
				continue;
			}
		}
next_iter:
		curr = stack[spos];
	}
}

void intersect_tlas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    const struct b2node *nodes, /// TEMP
                    const struct b8node *mnodes, const unsigned int *imap,
                    const struct rinst *insts, const struct rtri *tris,
                    unsigned int tlasofs)
{
	intersect_tlas_impl(h, ori, dir, nodes, mnodes, imap, insts, tris,
	  tlasofs, dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
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

	/// TEMP TEMP
	rd->bmnodes = aligned_alloc(64, maxtris * 2 * sizeof(*rd->bmnodes));
	rd->b8nodes = aligned_alloc(64, maxtris * 2 * sizeof(*rd->b8nodes));
	///

	// Start of tlas index map and tlas nodes * 2
	rd->tlasofs = maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->b8nodes);
	free(rd->bmnodes);
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
			dprintf("> Creating blas for inst: %d, ofs: %d, cnt: %d, addr: %p, max node cnt: %d\n",
			  j, triofs, tricnt, (void *)rn, tricnt << 1);
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

			// Create normal sah based bvh
			struct bnode *nodes = malloc((tricnt << 1)
			  * sizeof(*nodes));
			unsigned int ncnt = build_bvh(nodes, aabbs,
			  &rd->imap[triofs], tricnt, rmin, rmax);
			assert(ncnt == count_nodes(nodes) + 1);
			dprintf("Initial bnode cnt: %d\n", ncnt);

			// Make leafs to contain 4 tris at best but not more
			unsigned int sid, mergecnt = 0, splitcnt = 0;
			merge_leafs(nodes, nodes, &sid, LEAF_TRI_CNT,
			  &mergecnt);
			dprintf("Merged which reduced %d bnodes\n", 2 * mergecnt);
			split_leafs(nodes, nodes, aabbs, &rd->imap[triofs],
			  &ncnt, LEAF_TRI_CNT, &splitcnt);
			dprintf("Splitted which added %d bnodes\n",
			  2 * splitcnt);
			dprintf("After merge and split bnode cnt: %d\n",
			  count_nodes(nodes));

			// Create compacted b2node bvh
			ncnt = convert_b2node(&rd->nodes[triofs << 1], nodes);
			dprintf("Compacted b2node cnt: %d\n", ncnt);

			// Create compacted bmnode bvh
			ncnt = convert_bmnode(&rd->bmnodes[triofs << 1], nodes);
			dprintf("Compacted bmnode cnt: %d\n", ncnt);

			// Create compacted b8node bvh
			ncnt = convert_b8node(&rd->b8nodes[triofs << 1],
			  &rd->bmnodes[triofs << 1]);
			dprintf("Compacted b8node cnt: %d\n", ncnt);

			free(nodes);
			free(aabbs);
		}
	}
}

void rend_prepdynamic(struct rdata *rd)
{
	//dprintf("> Creating tlas\n");
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
	build_bvh(nodes, rd->aabbs, &rd->imap[tlasofs], rd->instcnt,
	  rmin, rmax);
	convert_b2node(&rd->nodes[tlasofs << 1], nodes);
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

	intersect_tlas(&h, o, d, rd->nodes, rd->b8nodes, rd->imap, rd->insts,
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
                   unsigned char depth, unsigned int *seed)
{
	if (depth >= 2)
		return rd->bgcol;

	struct hit h = {.t = FLT_MAX};

	intersect_tlas(&h, o, d, rd->nodes, rd->b8nodes, rd->imap, rd->insts,
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

	struct vec3 dir = rand_hemicos(randf(seed), randf(seed));

	dir = vec3_add(vec3_add(vec3_scale(ta, dir.x), vec3_scale(bta, dir.y)),
	  vec3_scale(nrm, dir.z));

	//float cos_theta = vec3_dot(nrm, dir);
	//float pdf = cos_theta / PI;

	//struct vec3 brdf = vec3_scale(rd->mtls[mtlid].col, INV_PI);
	struct vec3 brdf = rd->mtls[mtlid].col;

	struct vec3 irr = trace2(vec3_add(pos, vec3_scale(dir, 0.001)), dir,
	  rd, depth + 1, seed);

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
		unsigned int seed = (blk + 13) * 131317 + rd->samplecnt * 23;
		unsigned int bx = (blk % blksx) * bs;
		unsigned int by = (blk / blksx) * bs;
		for (unsigned int j = 0; j < bs; j++) {
			unsigned int y = by + j;
			unsigned int yofs = w * y;
			for (unsigned int i = 0; i < bs; i++) {
				unsigned int x = bx + i;
				struct vec3 p = vec3_add(tl, vec3_add(
				  vec3_scale(dx, x), vec3_scale(dy, y)));
				p = vec3_add(p, vec3_add(
				  vec3_scale(dx, randf(&seed) - 0.5f),
				  vec3_scale(dy, randf(&seed) - 0.5f)));

				struct vec3 c =
				  //trace(eye, vec3_unit(vec3_sub(p, eye)), rd);
				  trace2(eye, vec3_unit(vec3_sub(p, eye)), rd,
				    0, &seed);

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

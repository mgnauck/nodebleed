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

#define LEAF_TRI_CNT  4

static __m256i compr_lut[256];

struct split { // Result from SAH binning step
	float          cost;
	unsigned char  axis;
	unsigned int   pos;
	struct vec3    lmin;
	struct vec3    lmax;
	struct vec3    rmin;
	struct vec3    rmax;
	struct vec3    invd;
};

struct hit { // 32 bytes
	float         t;
	float         u;
	float         v;
	unsigned int  id; // triid << INST_ID_BITS | instid & INST_ID_MASK
};

// Generate LUT to emulate vpcompressq on AVX2
// https://stackoverflow.com/questions/36932240/avx2-what-is-the-most-efficient-way-to-pack-left-based-on-a-mask/
unsigned long long get_perm(unsigned char mask)
{
	unsigned long long perm = 0ull;
	unsigned char c = 0;
	for (unsigned char i = 0; i < 8; i++)
		if ((mask >> i) & 1) {
			perm |= (unsigned long long)i << c;
			c += 8; // Each byte
		}
	return perm;
}

void rend_init_compresslut(void)
{
	// 8 bit mask: 0 will be compressed, 1's will be considered (= hit)
	for (unsigned int bm = 0; bm < 256; bm++) {
		unsigned long long p = get_perm(bm);
		//dprintf("0x%llx\n", p);
		compr_lut[bm] = _mm256_cvtepu8_epi32( // Convert 8x 8 bit to 32
		  _mm_cvtsi64_si128(p)); // Copy to lower 128, zero upper
	}
}

float calc_area(struct vec3 mi, struct vec3 ma)
{
	struct vec3 d = vec3_sub(ma, mi);
	return d.x * d.y + d.y * d.z + d.z * d.x;
}

// SAH binning step to find best split for given node
void find_best_split(struct split *best, unsigned int start, unsigned int cnt,
                     struct vec3 nmin, struct vec3 nmax, struct vec3 minext,
                     struct aabb *aabbs, unsigned int *imap)
{
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
	best->invd = (struct vec3){ // 1 / interval dims (delta) per axis
	  (float)INTERVAL_CNT / (nmax.x - nmin.x),
	  (float)INTERVAL_CNT / (nmax.y - nmin.y),
	  (float)INTERVAL_CNT / (nmax.z - nmin.z)};
	unsigned int *ip = &imap[start];
	for (unsigned int i = 0; i < cnt; i++) {
		struct aabb *a = &aabbs[*ip++];

		struct vec3 c = vec3_mul(vec3_sub(
		  vec3_scale(vec3_add(a->min, a->max), 0.5f), nmin),
		  best->invd);

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

	best->cost = FLT_MAX;

	for (unsigned char a = 0; a < 3; a++) {
		// Skip 'empty axis'
		if (vec3_getc(nmax, a) - vec3_getc(nmin, a) <
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
			if (c < best->cost) {
				best->cost = c;
				best->axis = a;
				best->pos = i;
				best->lmin = lmin[i];
				best->lmax = lmax[i];
				best->rmin = rmin[i];
				best->rmax = rmax[i];
			}
		}
	}
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

		struct split best;
		find_best_split(&best, n->sid, n->cnt, n->min, n->max,
		  minext, aabbs, imap);

		// Decide if split or leaf has better cost
		float nosplit = (float)n->cnt;
		if (nosplit <= 1.0f + best.cost / calc_area(n->min, n->max)) {
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
		float invda = vec3_getc(best.invd, best.axis); // Axis only
		float nmina = vec3_getc(n->min, best.axis);
		for (unsigned int i = 0; i < n->cnt; i++) {
			unsigned int id = imap[l];
			struct aabb *a = &aabbs[id];
			float bin = ((vec3_getc(a->min, best.axis)
			  + vec3_getc(a->max, best.axis)) * 0.5f - nmina)
			  * invda;
			if ((unsigned int)min(max((int)bin, 0),
			  INTERVAL_CNT - 1) <= best.pos) {
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
		left->min = best.lmin;
		left->max = best.lmax;

		struct bnode *right = &nodes[ncnt + 1];
		right->sid = r;
		right->cnt = n->cnt - lcnt;
		right->min = best.rmin;
		right->max = best.rmax;

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

unsigned int build_bvh8(struct bmnode *nodes, struct aabb *aabbs,
                        unsigned int *imap, unsigned int cnt,
                        struct vec3 rootmin, struct vec3 rootmax)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int curr = 0;
	unsigned int ncnt = 2; // Root + first dummy child

	// Prepare root node and first child
	memset(nodes, 0, 2 * sizeof(*nodes));
	nodes[0].min = nodes[1].min = rootmin;
	nodes[0].max = nodes[1].max = rootmax;
	nodes[0].childcnt = 1;
	nodes[0].children[0] = 1;
	nodes[1].cnt = cnt;

	struct vec3 minext = // Min extent relative to root aabb
	  vec3_scale(vec3_sub(rootmax, rootmin), 0.00000001f);

	while (true) {
		struct bmnode *n = &nodes[curr];

		// Horizontal split
		while (n->childcnt < MBVH_CHILD_CNT) {
			// Find child with biggest surface area
			struct bmnode *bc = NULL;
			float bcost = 0.0f;
			unsigned int bcid;
			for (unsigned int i = 0; i < n->childcnt; i++) {
				unsigned int cid = n->children[i];
				if ((cid & NODE_LEAF) == 0) {
					struct bmnode *c = &nodes[cid];
					float cost = calc_area(c->min, c->max);
					if (cost > bcost) {
						bc = c;
						bcost = cost;
						bcid = i;
					}
				}
			}

			if (!bc) // No child found that can be split
				break;

			// Run a SAH binning step to split the selected child
			struct split best;
			find_best_split(&best, bc->start, bc->cnt,
			  bc->min, bc->max, minext, aabbs, imap);

			// Compare no split cost vs best split
			// (horiz. split doesn't increase traversal steps)
			// 4 triangles per leaf
			if ((float)n->cnt / 4 < best.cost / (4 *
			  calc_area(n->min, n->max))) {
				// Child doesn't want to get split, mark leaf
				n->children[bcid] |= NODE_LEAF;
				continue; // Try next child
			}

			// Partition in l and r of split plane
			unsigned int l = bc->start;
			unsigned int r = l + bc->cnt;
			float invda = vec3_getc(best.invd, best.axis);
			float nmina = vec3_getc(bc->min, best.axis);
			for (unsigned int i = 0; i < bc->cnt; i++) {
				unsigned int id = imap[l];
				struct aabb *a = &aabbs[id];
				float bin = ((vec3_getc(a->min, best.axis)
				  + vec3_getc(a->max, best.axis)) * 0.5f
				  - nmina) * invda;
				if ((unsigned int)min(max((int)bin, 0),
				  INTERVAL_CNT - 1) <= best.pos) {
					l++;
				 } else {
					// Swap tri indices
					imap[l] = imap[--r];
					imap[r] = id;
				}
			}

			unsigned int lcnt = l - bc->start;
			if (lcnt == 0 || lcnt == bc->cnt) {
				//dprintf("one side of the partition was empty at start: %d, l: %d, r: %d\n",
				//  bc->start, lcnt, bc->cnt - lcnt);
				break;
			}

			// Update former child with the left split data
			bc->cnt = lcnt;
			bc->min = best.lmin;
			bc->max = best.lmax;

			// Add a new child node with the right split data
			struct bmnode *right = &nodes[ncnt];
			memset(right, 0, sizeof(*right));
			right->start = r;
			right->cnt = n->cnt - lcnt;
			right->min = best.rmin;
			right->max = best.rmax;

			// Append the new child to our original (parent) node
			n->children[n->childcnt++] = ncnt++;
		}

		// Vertical splitting
		for (unsigned int i = 0; i < n->childcnt; i++) {
			unsigned int cid = n->children[i];
			if ((cid & NODE_LEAF) == 0) {

				// TODO Vertical split

				// Schedule interior child node for processing
				stack[spos++] = cid;
			}
		}

		if (spos > 0)
			curr = stack[--spos];
		else
			break;
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
                 unsigned int *imap, unsigned int *nodecnt,
                 unsigned int maxcnt, unsigned int *splitcnt)
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
				break; // No child found that can be collapsed
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
	return ((struct distid *)a)->dist < ((struct distid *)b)->dist
	  ? 1 : -1;
}

// TODO
// NAN handling on AVX2 traversal functions
// Try packet tracing to speed up first hit
// Do comparisons of bnode vs b2node tlas traversal
// Create 8-wide BVH directly instead of transforming binary bvh? (Wald, 2008)
// Make merge/split leaf functions non-recursive?
// Clean unused intersection functions (bmnode intersection etc.)

unsigned int convert_b8node(struct b8node *tgt, struct bmnode *src,
                            unsigned int *imap, struct rtri *tris)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	// Access our b8node target byte-wise for offset calculation
	unsigned char *tptr = (unsigned char *)tgt;

	unsigned int snid = 0; // Current src node index
	unsigned int tnofs = 0; // Offset to curr tgt node and leaf data
	unsigned int tncnt = 0; // Tgt node cnt

	while (true) {
		struct bmnode *s = &src[snid];
		struct b8node *t = (struct b8node *)(tptr + tnofs);

		memset(t, 0, sizeof(*t));

		tnofs += sizeof(*t);
		tncnt++;

		unsigned int cid = 0; // 'Compacted' child index, i.e. no gaps
		for (unsigned int j = 0; j < 8; j++) {
			// Copy/setup data of non-empty child nodes
			if (s->children[j]) {
				assert(j < s->childcnt);
				struct bmnode *sc = &src[s->children[j]];
				t->minx[cid] = sc->min.x;
				t->maxx[cid] = sc->max.x;
				t->miny[cid] = sc->min.y;
				t->maxy[cid] = sc->max.y;
				t->minz[cid] = sc->min.z;
				t->maxz[cid] = sc->max.z;
				if (sc->cnt > 0) {
					// Leaf node
					assert(sc->cnt <= 4);
					assert(tnofs <= 0x7fffffff);
					// Store offset to embedded leaf data
					((unsigned int *)&t->children)[cid] =
					  NODE_LEAF | tnofs;
					// Embedd leaf data
					struct leaf4 *l = (struct leaf4 *)
					  (tptr + tnofs);
					unsigned int *ip = &imap[sc->start];
					for (unsigned char i = 0; i < 4; i++) {
						struct rtri *tri = &tris[*ip];

						l->v0x[i] = tri->v0.x;
						l->v0y[i] = tri->v0.y;
						l->v0z[i] = tri->v0.z;

						struct vec3 e0 = vec3_sub(
						  tri->v1, tri->v0);
						l->e0x[i] = e0.x;
						l->e0y[i] = e0.y;
						l->e0z[i] = e0.z;

						struct vec3 e1 = vec3_sub(
						  tri->v2, tri->v0);
						l->e1x[i] = e1.x;
						l->e1y[i] = e1.y;
						l->e1z[i] = e1.z;

						l->id[i] = *ip;

						// Replicate last tri if not 4
						if (i < s->cnt - 1)
							ip++;
					}
					// Account for leaf data
					tnofs += sizeof(*l);
				} else {
					// Interior node
					// Push offset to current child, so
					// actual node ofs can be set later on
					assert(spos < 128 - 1);
					stack[spos++] = (unsigned int)
					  (((unsigned int *)&t->children + cid)
					  - (unsigned int *)tptr);
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
					struct bmnode *c =
					  &src[s->children[i]];
					// Aabb corner of ray dir
					struct vec3 corner = {
					  j & 1 ? c->min.x : c->max.x,
					  j & 2 ? c->min.y : c->max.y,
					  j & 4 ? c->min.z : c->max.z};
					cdi[i].dist = vec3_dot(dir, corner);
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
		}

		if (spos > 1) {
			// Src bmnode to continue with
			snid = stack[--spos];
			// Previous tgt child is set to curr ofs of new node
			*((unsigned int *)tptr + stack[--spos]) = tnofs;
		} else
			break;
	}

	return tncnt;
}

bool intersect_tri_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                        struct vec3 v0, struct vec3 e0, struct vec3 e1,
                        unsigned int triid, unsigned int instid)
{
	// Calculate determinant
	struct vec3 pv = vec3_cross(dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return false;

	float idet = 1.0f / det;

	// Distance v0 to origin
	struct vec3 tv = vec3_sub(ori, v0);

	// Calculate param u and test bounds
	float u = vec3_dot(tv, pv) * idet;
	if (u < 0.0f || u > 1.0f)
		return false;

	// Prepare to test for v
	struct vec3 qv = vec3_cross(tv, e0);

	// Calculate param v and test bounds
	float v = vec3_dot(dir, qv) * idet;
	if (v < 0.0f || u + v > 1.0f)
		return false;

	// Calculate dist
	float dist = vec3_dot(e1, qv) * idet;
	if (dist > 0.0f && dist < h->t) {
		h->t = dist;
		h->u = u;
		h->v = v;
		h->id = (triid << INST_ID_BITS) | instid;
		return true;
	}

	return false;
}

bool intersect_any_tri_impl(float tfar, struct vec3 ori, struct vec3 dir,
                            struct vec3 v0, struct vec3 e0, struct vec3 e1)
{
	// Calculate determinant
	struct vec3 pv = vec3_cross(dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return false;

	float idet = 1.0f / det;

	// Distance v0 to origin
	struct vec3 tv = vec3_sub(ori, v0);

	// Calculate param u and test bounds
	float u = vec3_dot(tv, pv) * idet;
	if (u < 0.0f || u > 1.0f)
		return false;

	// Prepare to test for v
	struct vec3 qv = vec3_cross(tv, e0);

	// Calculate param v and test bounds
	float v = vec3_dot(dir, qv) * idet;
	if (v < 0.0f || u + v > 1.0f)
		return false;

	// Calculate dist
	float dist = vec3_dot(e1, qv) * idet;

	return dist > 0.0f && dist < tfar;
}

bool intersect_tri(struct hit *h, struct vec3 ori, struct vec3 dir,
                   struct rtri *tris, unsigned int triid, unsigned int instid)
{
	struct rtri *t = &tris[triid];
	struct vec3 v0 = t->v0;
	return intersect_tri_impl(h, ori, dir, v0, vec3_sub(t->v1, v0),
	  vec3_sub(t->v2, v0), triid, instid);
}

bool intersect_any_tri(float tfar, struct vec3 ori, struct vec3 dir,
                       struct rtri *t)
{
	struct vec3 v0 = t->v0;
	return intersect_any_tri_impl(tfar, ori, dir, v0, vec3_sub(t->v1, v0),
	  vec3_sub(t->v2, v0));
}

void intersect_blas3_impl_avx2(struct hit *h, struct vec3 ori, struct vec3 dir,
                               struct b8node *blas, unsigned int instid,
                               bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[128];
	_Alignas(64) float dstack[128];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)blas;
	unsigned int ofs = 0; // Offset to curr node or leaf data

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
	__m256 zero8 = _mm256_setzero_ps();

	__m128 dx4 = _mm_set1_ps(dir.x);
	__m128 dy4 = _mm_set1_ps(dir.y);
	__m128 dz4 = _mm_set1_ps(dir.z);

	__m128 ox4 = _mm_set1_ps(ori.x);
	__m128 oy4 = _mm_set1_ps(ori.y);
	__m128 oz4 = _mm_set1_ps(ori.z);

	__m128 zero4 = _mm_setzero_ps();
	__m128 one4 = _mm_set1_ps(1.0f);
	__m128 fltmax4 = _mm_set1_ps(FLT_MAX);

	// Ray dir sign defines how to shift the permutation map
	unsigned char s = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct b8node *n = (struct b8node *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
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

			__m256 tmin = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(tx0, ty0), tz0), zero8);

			__m256 tmax = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(tx1, ty1), tz1), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
			unsigned int hitmask = _mm256_movemask_ps(hitmask8);
			unsigned int hitcnt = __builtin_popcount(hitmask);

			switch (hitcnt) {
			case 0:
				// No hit, consult stack
				if (spos > 0)
					ofs = stack[--spos];
				else
					return;
				break;
			case 1:
				// Single hit, directly continue w/ child node
				ofs = ((unsigned int *)&n->children)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit
				// Order, compress and push child nodes + dists
				__m256i ord8 = _mm256_srli_epi32(n->perm, s);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				// Ordered min distances and child node ids
				tmin = _mm256_permutevar8x32_ps(tmin, ord8);
				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children,
				  ord8);

				// Map ordered hit mask to compressed indices
				__m256 cid = compr_lut[hitmaskord];

				// Permute to compress dists and child node ids
				__m256 distsfin8 = _mm256_permutevar8x32_ps(
				  tmin, cid);
				__m256 childfin8 = _mm256_permutevar8x32_epi32(
				  childrenord8, cid);

				// Unaligned store dists and children on stacks
				_mm256_storeu_ps(dstack + spos, distsfin8);
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);

				spos += hitcnt - 1; // Account for pushed nodes
				ofs = stack[spos]; // Next node
			}
		}

		// Intersect 4 embedded tris at once
		struct leaf4 *l = (struct leaf4 *)(ptr + (ofs & ~NODE_LEAF));

		// Moeller, Trumbore: Ray-triangle intersection
		// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/

		// pv = cross(dir, e1)
		__m128 pvx4 = _mm_fmsub_ps(dy4, l->e1z,
		  _mm_mul_ps(dz4, l->e1y));
		__m128 pvy4 = _mm_fmsub_ps(dz4, l->e1x,
		  _mm_mul_ps(dx4, l->e1z));
		__m128 pvz4 = _mm_fmsub_ps(dx4, l->e1y,
		  _mm_mul_ps(dy4, l->e1x));

		// tv = ori - v0
		__m128 tvx4 = _mm_sub_ps(ox4, l->v0x);
		__m128 tvy4 = _mm_sub_ps(oy4, l->v0y);
		__m128 tvz4 = _mm_sub_ps(oz4, l->v0z);

		// det = dot(e0, pv)
		__m128 det4 = _mm_fmadd_ps(l->e0x, pvx4,
		  _mm_fmadd_ps(l->e0y, pvy4, _mm_mul_ps(l->e0z, pvz4)));

		// qv = cross(tv, e0)
		__m128 qvx4 = _mm_fmsub_ps(tvy4, l->e0z,
		  _mm_mul_ps(tvz4, l->e0y));
		__m128 qvy4 = _mm_fmsub_ps(tvz4, l->e0x,
		  _mm_mul_ps(tvx4, l->e0z));
		__m128 qvz4 = _mm_fmsub_ps(tvx4, l->e0y,
		  _mm_mul_ps(tvy4, l->e0x));

		// idet = 1 / det
		// https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision
		__m128 r4 = _mm_rcp_ps(det4);
		__m128 m4 = _mm_mul_ps(det4, _mm_mul_ps(r4, r4));
		__m128 idet4 = _mm_sub_ps(_mm_add_ps(r4, r4), m4);

		// u = idet4 * dot(tv, pv)
		__m128 u4 = _mm_mul_ps(idet4, _mm_fmadd_ps(tvx4, pvx4,
		  _mm_fmadd_ps(tvy4, pvy4, _mm_mul_ps(tvz4, pvz4))));

		// v = idet * dot(dir, qv)
		__m128 v4 = _mm_mul_ps(idet4, _mm_fmadd_ps(dx4, qvx4,
		  _mm_fmadd_ps(dy4, qvy4, _mm_mul_ps(dz4, qvz4))));

		// t = idet * dot(e1, qv)
		__m128 t4 = _mm_mul_ps(idet4, _mm_fmadd_ps(l->e1x, qvx4,
		  _mm_fmadd_ps(l->e1y, qvy4, _mm_mul_ps(l->e1z, qvz4))));

		// u >= 0
		__m128 uzero4 = _mm_cmpge_ps(u4, zero4);

		// v >= 0
		__m128 vzero4 = _mm_cmpge_ps(v4, zero4);

		// u + v <= 1
		__m128 uvone4 = _mm_cmple_ps(_mm_add_ps(u4, v4), one4);

		// t > 0
		__m128 tzero4 = _mm_cmpgt_ps(t4, zero4);

		// t < h->t
		__m128 tcurr4 = _mm_cmplt_ps(t4, _mm256_extractf128_ps(t8, 0));

		// Merge all comparison results into one final mask
		__m128 mask4 = _mm_and_ps(tcurr4, _mm_and_ps(
		  _mm_and_ps(uzero4, vzero4), _mm_and_ps(uvone4, tzero4)));

		if (_mm_movemask_ps(mask4)) { // Any hit? (lowest 4 bits)
			// Replace misses with FLT_MAX
			t4 = _mm_blendv_ps(fltmax4, t4, mask4);

			// Horizontal min of t4, result broadcasted
			__m128 sh4 = _mm_min_ps(t4, _mm_shuffle_ps(t4, t4,
			  _MM_SHUFFLE(2, 3, 0, 1)));
			__m128 min4 = _mm_min_ps(sh4, _mm_shuffle_ps(sh4, sh4,
			  _MM_SHUFFLE(1, 0, 3, 2)));

			// Invert count of leading zeros to get lane
			unsigned int i = 31 - __builtin_clz(
			  _mm_movemask_ps(_mm_cmpeq_ps(min4, t4)));

			h->t = t4[i];
			h->u = u4[i];
			h->v = v4[i];
			h->id = (l->id[i] << INST_ID_BITS) | instid;

			// Track closest h->t for future aabb+tri intersections
			t8 = _mm256_set1_ps(h->t);

			// Compress stack wrt to nearer h->t in batches of 8
			unsigned int spos2 = 0;
			for (i = 0; i < spos; i += 8) {
				__m256 dists8 = _mm256_load_ps(dstack + i);
				__m256i nids8 = _mm256_load_si256(
				  (__m256i *)(stack + i));

				// Nearer/eq ones are 1
				unsigned int nearmask = _mm256_movemask_ps(
				  _mm256_cmp_ps(dists8, t8, _CMP_LE_OQ));

				// Map nearer mask to compressed indices
				__m256i cid = compr_lut[nearmask];

				// Permute to compress dists and node ids
				dists8 = _mm256_permutevar8x32_ps(dists8, cid);
				nids8 =
				  _mm256_permutevar8x32_epi32(nids8, cid);

				// Store compressed dists and node ids
				_mm256_storeu_ps(dstack + spos2, dists8);
				_mm256_storeu_si256((__m256i *)(stack + spos2),
				  nids8);

				unsigned int cnt = min(spos - i, 8); // Last!
				unsigned int cntmask = (1 << cnt) - 1;
				spos2 +=
				  __builtin_popcount(cntmask & nearmask);
			}

			spos = spos2; // Stack rewrite done
		}

		// Pop next node from stack if something is left
		if (spos > 0)
			ofs = stack[--spos];
		else
			return;
	}
}

bool intersect_any_blas3_impl_avx2(float tfar, struct vec3 ori,
                                   struct vec3 dir, struct b8node *blas,
                                   bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[128];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)blas;
	unsigned int ofs = 0; // Offset to curr node or leaf data

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

	__m256 t8 = _mm256_set1_ps(tfar);
	__m256 zero8 = _mm256_setzero_ps();

	__m128 dx4 = _mm_set1_ps(dir.x);
	__m128 dy4 = _mm_set1_ps(dir.y);
	__m128 dz4 = _mm_set1_ps(dir.z);

	__m128 ox4 = _mm_set1_ps(ori.x);
	__m128 oy4 = _mm_set1_ps(ori.y);
	__m128 oz4 = _mm_set1_ps(ori.z);

	__m128 zero4 = _mm_setzero_ps();
	__m128 one4 = _mm_set1_ps(1.0f);

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct b8node *n = (struct b8node *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
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

			__m256 tmin = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(tx0, ty0), tz0), zero8);

			__m256 tmax = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(tx1, ty1), tz1), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
			unsigned int hitmask = _mm256_movemask_ps(hitmask8);
			unsigned int hitcnt = __builtin_popcount(hitmask);

			switch (hitcnt) {
			case 0:
				// No hit, consult stack
				if (spos > 0)
					ofs = stack[--spos];
				else
					return false;
				break;
			case 1:
				// Single hit, directly continue w/ child node
				ofs = ((unsigned int *)&n->children)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit

				// Compress unordered child nodes via lut
				__m256 childfin8 = _mm256_permutevar8x32_epi32(
				  n->children, compr_lut[hitmask]);

				// Unaligned store children on stack
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);

				spos += hitcnt - 1; // Account for pushed nodes
				ofs = stack[spos]; // Next node
			}
		}

		// Intersect 4 embedded tris at once
		struct leaf4 *l = (struct leaf4 *)(ptr + (ofs & ~NODE_LEAF));

		// Moeller, Trumbore: Ray-triangle intersection
		// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/

		// pv = cross(dir, e1)
		__m128 pvx4 = _mm_fmsub_ps(dy4, l->e1z,
		  _mm_mul_ps(dz4, l->e1y));
		__m128 pvy4 = _mm_fmsub_ps(dz4, l->e1x,
		  _mm_mul_ps(dx4, l->e1z));
		__m128 pvz4 = _mm_fmsub_ps(dx4, l->e1y,
		  _mm_mul_ps(dy4, l->e1x));

		// tv = ori - v0
		__m128 tvx4 = _mm_sub_ps(ox4, l->v0x);
		__m128 tvy4 = _mm_sub_ps(oy4, l->v0y);
		__m128 tvz4 = _mm_sub_ps(oz4, l->v0z);

		// det = dot(e0, pv)
		__m128 det4 = _mm_fmadd_ps(l->e0x, pvx4,
		  _mm_fmadd_ps(l->e0y, pvy4, _mm_mul_ps(l->e0z, pvz4)));

		// qv = cross(tv, e0)
		__m128 qvx4 = _mm_fmsub_ps(tvy4, l->e0z,
		  _mm_mul_ps(tvz4, l->e0y));
		__m128 qvy4 = _mm_fmsub_ps(tvz4, l->e0x,
		  _mm_mul_ps(tvx4, l->e0z));
		__m128 qvz4 = _mm_fmsub_ps(tvx4, l->e0y,
		  _mm_mul_ps(tvy4, l->e0x));

		// idet = 1 / det
		// https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision
		__m128 r4 = _mm_rcp_ps(det4);
		__m128 m4 = _mm_mul_ps(det4, _mm_mul_ps(r4, r4));
		__m128 idet4 = _mm_sub_ps(_mm_add_ps(r4, r4), m4);

		// u = idet4 * dot(tv, pv)
		__m128 u4 = _mm_mul_ps(idet4, _mm_fmadd_ps(tvx4, pvx4,
		  _mm_fmadd_ps(tvy4, pvy4, _mm_mul_ps(tvz4, pvz4))));

		// v = idet * dot(dir, qv)
		__m128 v4 = _mm_mul_ps(idet4, _mm_fmadd_ps(dx4, qvx4,
		  _mm_fmadd_ps(dy4, qvy4, _mm_mul_ps(dz4, qvz4))));

		// t = idet * dot(e1, qv)
		__m128 t4 = _mm_mul_ps(idet4, _mm_fmadd_ps(l->e1x, qvx4,
		  _mm_fmadd_ps(l->e1y, qvy4, _mm_mul_ps(l->e1z, qvz4))));

		// u >= 0
		__m128 uzero4 = _mm_cmpge_ps(u4, zero4);

		// v >= 0
		__m128 vzero4 = _mm_cmpge_ps(v4, zero4);

		// u + v <= 1
		__m128 uvone4 = _mm_cmple_ps(_mm_add_ps(u4, v4), one4);

		// t > 0
		__m128 tzero4 = _mm_cmpgt_ps(t4, zero4);

		// t < h->t
		__m128 tcurr4 = _mm_cmplt_ps(t4, _mm256_extractf128_ps(t8, 0));

		// Merge all comparison results into one final mask
		__m128 mask4 = _mm_and_ps(tcurr4, _mm_and_ps(
		  _mm_and_ps(uzero4, vzero4), _mm_and_ps(uvone4, tzero4)));

		// Any hit? (lowest 4 bits)
		if (_mm_movemask_ps(mask4))
			return true;

		// Pop next node from stack if something is left
		if (spos > 0)
			ofs = stack[--spos];
		else
			return false;
	}
}

void intersect_blas3_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                          struct b8node *blas, unsigned int instid,
                          bool dx, bool dy, bool dz)
{
	unsigned int stack[128];
	float dstack[128]; // Distance stack
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)blas;
	unsigned int ofs = 0; // Offset to curr node or leaf data

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	// Ray dir sign defines how to shift the permutation map
	unsigned char s = ((dz << 2) | (dy << 1) | dx) * 3;

	// Performance could be improved by a node empty flag and the tri cnt
	// encoded in a leaf's child id, but this function is only for b8node
	// bvh validation and the AVX2 traversal does not need the additions

	while (true) {
		if ((ofs & NODE_LEAF) == 0) {
			struct b8node *n = (struct b8node *)(ptr + ofs);
			for (unsigned char j = 0; j < 8; j++) {
				unsigned int i =
				  (((unsigned int *)&n->perm)[j] >> s) & 7;
				unsigned int o =
				  ((unsigned int *)&n->children)[i];
				/*if (o & ~NODE_EMPTY)*/ {
					// Interior node, check child aabbs
					float t0x = (dx ? n->minx[i]
					  : n->maxx[i]) * idx - rx;
					float t0y = (dy ? n->miny[i]
					  : n->maxy[i]) * idy - ry;
					float t0z = (dz ? n->minz[i]
					  : n->maxz[i]) * idz - rz;

					float t1x = (dx ? n->maxx[i]
					  : n->minx[i]) * idx - rx;
					float t1y = (dy ? n->maxy[i]
					  : n->miny[i]) * idy - ry;
					float t1z = (dz ? n->maxz[i]
					  : n->minz[i]) * idz - rz;

					float tmin =
					  max(max(t0x, t0y), max(t0z, 0.0f));
					float tmax =
					  min(min(t1x, t1y), min(t1z, h->t));

					if (tmax >= tmin) {
						assert(spos < 128);
						dstack[spos] = tmin;
						stack[spos++] = o;
					}
				}
			}
		} else {
			// Intersect all embedded tris
			struct leaf4 *l = (struct leaf4 *)(ptr
			  + (ofs & ~NODE_LEAF));
			for (unsigned char i = 0; i < 4; i++) {
				intersect_tri_impl(h, ori, dir,
				  (struct vec3){l->v0x[i], l->v0y[i],
				  l->v0z[i]},
				  (struct vec3){l->e0x[i], l->e0y[i],
				  l->e0z[i]},
				  (struct vec3){l->e1x[i], l->e1y[i],
				  l->e1z[i]},
				  l->id[i], instid);
			}
		}

		// Pop next node from stack if something is left
		while (spos > 0)
			if (dstack[--spos] < h->t)
				goto next_iter;
		return;
next_iter:
		ofs = stack[spos];
	}
}

bool intersect_any_blas3_impl(float tfar, struct vec3 ori, struct vec3 dir,
                              struct b8node *blas, bool dx, bool dy, bool dz)
{
	unsigned int stack[128];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)blas;
	unsigned int ofs = 0; // Offset to curr node or leaf data

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	// Performance could be improved by a node empty flag and the tri cnt
	// encoded in a leaf's child id, but this function is only for b8node
	// bvh validation and the AVX2 traversal does not need the additions

	while (true) {
		if ((ofs & NODE_LEAF) == 0) {
			struct b8node *n = (struct b8node *)(ptr + ofs);
			for (unsigned char i = 0; i < 8; i++) {
				unsigned int o =
				  ((unsigned int *)&n->children)[i];
				/*if (o & ~NODE_EMPTY)*/ {
					// Interior node, check child aabbs
					float t0x = (dx ? n->minx[i]
					  : n->maxx[i]) * idx - rx;
					float t0y = (dy ? n->miny[i]
					  : n->maxy[i]) * idy - ry;
					float t0z = (dz ? n->minz[i]
					  : n->maxz[i]) * idz - rz;

					float t1x = (dx ? n->maxx[i]
					  : n->minx[i]) * idx - rx;
					float t1y = (dy ? n->maxy[i]
					  : n->miny[i]) * idy - ry;
					float t1z = (dz ? n->maxz[i]
					  : n->minz[i]) * idz - rz;

					float tmin =
					  max(max(t0x, t0y), max(t0z, 0.0f));
					float tmax =
					  min(min(t1x, t1y), min(t1z, tfar));

					if (tmax >= tmin) {
						assert(spos < 128);
						stack[spos++] = o;
					}
				}
			}
		} else {
			// Intersect all embedded tris
			struct leaf4 *l = (struct leaf4 *)(ptr
			  + (ofs & ~NODE_LEAF));
			for (unsigned char i = 0; i < 4; i++) {
				if (intersect_any_tri_impl(tfar, ori, dir,
				  (struct vec3){l->v0x[i], l->v0y[i],
				  l->v0z[i]},
				  (struct vec3){l->e0x[i], l->e0y[i],
				  l->e0z[i]},
				  (struct vec3){l->e1x[i], l->e1y[i],
				  l->e1z[i]}))
					return true;
			}
		}

		// Pop next node from stack if something is left
		if (spos > 0)
			ofs = stack[--spos];
		else
			return false;
	}
}

void intersect_blas3(struct hit *h, struct vec3 ori, struct vec3 dir,
                     struct b8node *blas, unsigned int instid)
{
	intersect_blas3_impl_avx2(h, ori, dir, blas, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

bool intersect_any_blas3(float tfar, struct vec3 ori, struct vec3 dir,
                        struct b8node *blas)
{
	return intersect_any_blas3_impl_avx2(tfar, ori, dir, blas,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

// Debug only, intersect bmnodes (m-wide, currently 8)
void intersect_blas2_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                          struct bmnode *blas, unsigned int *imap,
                          struct rtri *tris, unsigned int instid,
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
		struct bmnode *n = &blas[curr];

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
				struct bmnode *c = &blas[cid];

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

				float tmin =
				  max(max(t0x, t0y), max(t0z, 0.0f));
				float tmax =
				  min(min(t1x, t1y), min(t1z, h->t));

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
                     struct bmnode *blas, unsigned int *imap,
                     struct rtri *tris, unsigned int instid)
{
	intersect_blas2_impl(h, ori, dir, blas, imap, tris, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

// Intersects b2nodes (2-wide)
void intersect_blas1_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                          struct b2node *blas, unsigned int *imap,
                          struct rtri *tris, unsigned int instid,
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
		struct b2node *n = &blas[curr];

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
                     struct b2node *blas, unsigned int *imap,
                     struct rtri *tris, unsigned int instid)
{
	intersect_blas1_impl(h, ori, dir, blas, imap, tris, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

void intersect_tlas2_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                         struct b2node *nodes, // TODO tlas nodes
                         struct b8node *mnodes, unsigned int *imap,
                         struct rinst *insts,
                         struct rtri *tris, // TODO Not used by b8nodes
                         unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	unsigned int stack[64];
	float dstack[64]; // Distance stack
	unsigned int spos = 0;

	unsigned int curr = 0;

	struct b2node *tlas = &nodes[tlasofs << 1];
	unsigned int *tlasimap = &imap[tlasofs];

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	while (true) {
		struct b2node *n = &tlas[curr];

		if (n->cnt > 0) {
			// Leaf, check instance blas
			unsigned int *ip = &tlasimap[n->start];
			for (unsigned int i = 0; i < n->cnt; i++) {
				unsigned int instid = *ip++;
				struct rinst *ri = &insts[instid];

				// Transform ray into object space of instance
				float inv[16];
				mat4_from3x4(inv, ri->globinv);

				/*
				unsigned int o = ri->triofs;
				intersect_blas2(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[o << 1], &imap[o], &tris[o], instid);
				*/
				intersect_blas3(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[ri->triofs << 1], instid);
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

void intersect_tlas2(struct hit *h, struct vec3 ori, struct vec3 dir,
                     struct b2node *nodes, // TODO tlas nodes
                     struct b8node *mnodes, unsigned int *imap,
                     struct rinst *insts,
                     struct rtri *tris, // TODO Not used by b8nodes
                     unsigned int tlasofs)
{
	intersect_tlas2_impl(h, ori, dir, nodes, mnodes, imap, insts, tris,
	  tlasofs, dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

void intersect_tlas1_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                         struct bnode *nodes, // TODO tlas nodes
                         struct b8node *mnodes, unsigned int *imap,
                         struct rinst *insts,
                         struct rtri *tris, // TODO Not used by b8nodes
                         unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	unsigned int stack[64];
	float dstack[64]; // Distance stack
	unsigned int spos = 0;

	unsigned int curr = 0;

	struct bnode *tlas = &nodes[tlasofs << 1];
	unsigned int *tlasimap = &imap[tlasofs];

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	while (true) {
		struct bnode *n = &tlas[curr];

		if (n->cnt > 0) {
			// Leaf, check instance blas
			unsigned int *ip = &tlasimap[n->sid];
			for (unsigned int i = 0; i < n->cnt; i++) {
				unsigned int instid = *ip++;
				struct rinst *ri = &insts[instid];

				// Transform ray into object space of instance
				float inv[16];
				mat4_from3x4(inv, ri->globinv);

				/*
				unsigned int o = ri->triofs;
				intersect_blas2(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[o << 1], &imap[o], &tris[o], instid);
				*/
				intersect_blas3(h,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[ri->triofs << 1], instid);
			}

			// Pop next node from stack if something is left
			while (spos > 0)
				if (dstack[--spos] < h->t)
					goto next_iter;
			return;
		} else {
			// Interior node, check child aabbs
			unsigned int l = n->sid;
			unsigned int r = l + 1;

			struct bnode *cl = &tlas[l];
			struct bnode *cr = &tlas[r];

			float t0xl = (dx ? cl->min.x : cl->max.x) * idx - rx;
			float t0yl = (dy ? cl->min.y : cl->max.y) * idy - ry;
			float t0zl = (dz ? cl->min.z : cl->max.z) * idz - rz;
			float t0xr = (dx ? cr->min.x : cr->max.x) * idx - rx;
			float t0yr = (dy ? cr->min.y : cr->max.y) * idy - ry;
			float t0zr = (dz ? cr->min.z : cr->max.z) * idz - rz;

			float t1xl = (dx ? cl->max.x : cl->min.x) * idx - rx;
			float t1yl = (dy ? cl->max.y : cl->min.y) * idy - ry;
			float t1zl = (dz ? cl->max.z : cl->min.z) * idz - rz;
			float t1xr = (dx ? cr->max.x : cr->min.x) * idx - rx;
			float t1yr = (dy ? cr->max.y : cr->min.y) * idy - ry;
			float t1zr = (dz ? cr->max.z : cr->min.z) * idz - rz;

			float tminl = max(max(t0xl, t0yl), max(t0zl, 0.0f));
			float tminr = max(max(t0xr, t0yr), max(t0zr, 0.0f));

			float tmaxl = min(min(t1xl, t1yl), min(t1zl, h->t));
			float tmaxr = min(min(t1xr, t1yr), min(t1zr, h->t));

			float d0 = tmaxl >= tminl ? tminl : FLT_MAX;
			float d1 = tmaxr >= tminr ? tminr : FLT_MAX;

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

bool intersect_any_tlas1_impl(float tfar, struct vec3 ori, struct vec3 dir,
                              struct bnode *nodes, // TODO tlas nodes
                              struct b8node *mnodes, unsigned int *imap,
                              struct rinst *insts,
                              struct rtri *tris, // TODO Not used by b8nodes
                              unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	unsigned int curr = 0;

	struct bnode *tlas = &nodes[tlasofs << 1];
	unsigned int *tlasimap = &imap[tlasofs];

	float idx = 1.0f / dir.x;
	float idy = 1.0f / dir.y;
	float idz = 1.0f / dir.z;

	float rx = ori.x * idx;
	float ry = ori.y * idy;
	float rz = ori.z * idz;

	while (true) {
		struct bnode *n = &tlas[curr];

		if (n->cnt > 0) {
			// Leaf, check instance blas
			unsigned int *ip = &tlasimap[n->sid];
			for (unsigned int i = 0; i < n->cnt; i++) {
				unsigned int instid = *ip++;
				struct rinst *ri = &insts[instid];

				// Transform ray into object space of instance
				float inv[16];
				mat4_from3x4(inv, ri->globinv);

				if (intersect_any_blas3(tfar,
				  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
				  &mnodes[ri->triofs << 1]))
					return true;
			}

			// Pop next node from stack if something is left
			if (spos > 0)
				curr = stack[--spos];
			else
				return false;
		} else {
			// Interior node, check child aabbs
			unsigned int l = n->sid;
			unsigned int r = l + 1;

			struct bnode *cl = &tlas[l];
			struct bnode *cr = &tlas[r];

			float t0xl = (dx ? cl->min.x : cl->max.x) * idx - rx;
			float t0yl = (dy ? cl->min.y : cl->max.y) * idy - ry;
			float t0zl = (dz ? cl->min.z : cl->max.z) * idz - rz;
			float t0xr = (dx ? cr->min.x : cr->max.x) * idx - rx;
			float t0yr = (dy ? cr->min.y : cr->max.y) * idy - ry;
			float t0zr = (dz ? cr->min.z : cr->max.z) * idz - rz;

			float t1xl = (dx ? cl->max.x : cl->min.x) * idx - rx;
			float t1yl = (dy ? cl->max.y : cl->min.y) * idy - ry;
			float t1zl = (dz ? cl->max.z : cl->min.z) * idz - rz;
			float t1xr = (dx ? cr->max.x : cr->min.x) * idx - rx;
			float t1yr = (dy ? cr->max.y : cr->min.y) * idy - ry;
			float t1zr = (dz ? cr->max.z : cr->min.z) * idz - rz;

			float tminl = max(max(t0xl, t0yl), max(t0zl, 0.0f));
			float tminr = max(max(t0xr, t0yr), max(t0zr, 0.0f));

			float tmaxl = min(min(t1xl, t1yl), min(t1zl, tfar));
			float tmaxr = min(min(t1xr, t1yr), min(t1zr, tfar));

			float d0 = tmaxl >= tminl ? tminl : FLT_MAX;
			float d1 = tmaxr >= tminr ? tminr : FLT_MAX;

			if (d0 != FLT_MAX) {
				curr = l;
				if (d1 != FLT_MAX) {
					assert(spos < 64);
					stack[spos++] = r;
				}
			} else if (d1 != FLT_MAX) {
				curr = r;
			} else {
				if (spos > 0)
					curr = stack[--spos];
				else
					return false;
			}
		}
	}
}

void intersect_tlas1(struct hit *h, struct vec3 ori, struct vec3 dir,
                     struct bnode *nodes, // TODO tlas nodes
                     struct b8node *mnodes, unsigned int *imap,
                     struct rinst *insts,
                     struct rtri *tris, // TODO Not used by b8nodes
                     unsigned int tlasofs)
{
	intersect_tlas1_impl(h, ori, dir, nodes, mnodes, imap, insts, tris,
	  tlasofs, dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

bool intersect_any_tlas1(float tfar, struct vec3 ori, struct vec3 dir,
                         struct bnode *nodes, // TODO tlas nodes
                         struct b8node *mnodes, unsigned int *imap,
                         struct rinst *insts,
                         struct rtri *tris, // TODO Not used by b8nodes
                         unsigned int tlasofs)
{
	return intersect_any_tlas1_impl(tfar, ori, dir, nodes, mnodes, imap,
	  insts, tris, tlasofs, dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
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
	rd->bnodes = aligned_alloc(64, idcnt * 2 * sizeof(*rd->bnodes));
	rd->b2nodes = aligned_alloc(64, idcnt * 2 * sizeof(*rd->b2nodes));
	memset(rd->b2nodes, 0, idcnt * 2 * sizeof(*rd->b2nodes));

	// Bvh nodes for blas only
	rd->bmnodes = aligned_alloc(64, maxtris * 2 * sizeof(*rd->bmnodes));
	rd->b8nodes = aligned_alloc(64, maxtris * 2 * sizeof(*rd->b8nodes));

	// Start of tlas index map and tlas nodes * 2
	rd->tlasofs = maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->b8nodes);
	free(rd->bmnodes);
	free(rd->b2nodes);
	free(rd->bnodes);
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
		struct b2node *rn = &rd->b2nodes[triofs << 1]; // Root node
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

			// Create classic bvh
			unsigned int ncnt = build_bvh(&rd->bnodes[triofs << 1],
			  aabbs, &rd->imap[triofs], tricnt, rmin, rmax);
			dprintf("Initial bnode cnt: %d\n", ncnt);

			// Make leafs to contain 4 tris at best but not more
			unsigned int sid, mcnt = 0, scnt = 0;
			merge_leafs(&rd->bnodes[triofs << 1],
			  &rd->bnodes[triofs << 1], &sid, LEAF_TRI_CNT,
			  &mcnt);
			dprintf("Merged which reduced %d bnodes\n", 2 * mcnt);

			split_leafs(&rd->bnodes[triofs << 1],
			  &rd->bnodes[triofs << 1], aabbs, &rd->imap[triofs],
			  &ncnt, LEAF_TRI_CNT, &scnt);
			dprintf("Splitted which added %d bnodes\n",
			  2 * scnt);
			dprintf("After merge and split bnode cnt: %d\n",
			  count_nodes(&rd->bnodes[triofs << 1]));

			// Create compacted b2node bvh
			ncnt = convert_b2node(&rd->b2nodes[triofs << 1],
			  &rd->bnodes[triofs << 1]);
			dprintf("Compacted b2node cnt: %d\n", ncnt);

			// Create compacted bmnode bvh
			ncnt = convert_bmnode(&rd->bmnodes[triofs << 1],
			  &rd->bnodes[triofs << 1]);
			dprintf("Compacted bmnode cnt: %d\n", ncnt);

			// Create compacted b8node bvh
			ncnt = convert_b8node(&rd->b8nodes[triofs << 1],
			  &rd->bmnodes[triofs << 1], &rd->imap[triofs],
			  &rd->tris[triofs]);
			dprintf("Compacted b8node cnt: %d\n", ncnt);

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

	build_bvh(&rd->bnodes[tlasofs << 1], rd->aabbs, &rd->imap[tlasofs],
	  rd->instcnt, rmin, rmax);

	convert_b2node(&rd->b2nodes[tlasofs << 1], &rd->bnodes[tlasofs << 1]);
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
	float sgn = copysignf(1.0f, n.z);
	float a = -1.0f / (sgn + n.z);
	float b = n.x * n.y * a;
	*b1 = (struct vec3){1.0f + sgn * n.x * n.x * a, sgn * b, -sgn * n.x};
	*b2 = (struct vec3){b, sgn + n.y * n.y * a, -n.y};
}

struct vec3 rand_hemicos(float r0, float r1)
{
	float phi = TWO_PI * r0;
	float sr1 = sqrtf(r1);
	return (struct vec3){cosf(phi) * sr1, sinf(phi) * sr1, sqrtf(1 - r1)};
}

struct vec3 trace1(struct vec3 o, struct vec3 d, struct rdata *rd)
{
	struct hit h = {.t = FLT_MAX};

	intersect_tlas1(&h, o, d, rd->bnodes, rd->b8nodes, rd->imap, rd->insts,
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

struct vec3 trace2(struct vec3 o, struct vec3 d, struct rdata *rd,
                   unsigned char depth, unsigned int *seed)
{
	if (depth >= 2)
		return rd->bgcol;

	struct hit h = {.t = FLT_MAX};

	intersect_tlas1(&h, o, d, rd->bnodes, rd->b8nodes, rd->imap, rd->insts,
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
	//if (vec3_dot(nrm, d) > 0.0f)
	//	nrm = vec3_neg(nrm);

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

struct vec3 trace3(struct vec3 o, struct vec3 d, struct rdata *rd,
                   unsigned char depth, unsigned int *seed)
{
	struct hit h = {.t = FLT_MAX};
	intersect_tlas1(&h, o, d, rd->bnodes, rd->b8nodes, rd->imap, rd->insts,
	  rd->tris, rd->tlasofs);

	if (h.t == FLT_MAX)
		return rd->bgcol;

	unsigned int instid = h.id & INST_ID_MASK;
	unsigned int triid = h.id >> INST_ID_BITS;

	struct rinst *ri = &rd->insts[instid];
	struct rnrm *rn = &rd->nrms[ri->triofs + triid];

	unsigned int mtlid = rn->mtlid;

	struct vec3 pos = vec3_add(o, vec3_scale(d, h.t));
	//struct vec3 brdf = vec3_scale(rd->mtls[mtlid].col, INV_PI);
	struct vec3 brdf = rd->mtls[mtlid].col;

	// Inverse transpose, dir mul is 3x4
	float invtransp[16];
	float *invtransf = ri->globinv;
	for (int j = 0; j < 4; j++)
		for (int i = 0; i < 3; i++)
			invtransp[4 * j + i] = invtransf[4 * i + j];

	struct vec3 nrm = calc_nrm(h.u, h.v, rn, invtransp);
	//struct vec3 nrm = calc_fnrm(&rd->tris[ri->triofs + triid], invtransp);

	struct vec3 direct = {0.0f, 0.0f, 0.0f};

#define SAMPLE_LIGHT
#define SAMPLE_INDIRECT

#ifdef SAMPLE_LIGHT
	// Sample direct illumination
	struct vec3 lcol = {15.0f, 15.0f, 15.0f};
	struct vec3 lnrm = {0.0f, -1.0f, 0.0f};
	struct vec3 lpos = {randf(seed) * 20.0f - 20.0f, 70.0f,
	  randf(seed) * 20.0f + 50.0f};
	float larea = 20.0f * 20.0f;
	struct vec3 ldir = vec3_sub(lpos, pos);
	float ldist = vec3_len(ldir);
	ldir = vec3_scale(ldir, 1.0f / ldist);
	float ndotl = vec3_dot(nrm, ldir);
	if (ndotl > 0.0f)
		if (!intersect_any_tlas1(ldist, vec3_add(pos,
		  vec3_scale(ldir, EPS2)), ldir, rd->bnodes, rd->b8nodes,
		  rd->imap, rd->insts, rd->tris, rd->tlasofs))
			direct = vec3_scale(vec3_mul(brdf, lcol), INV_PI *
			  ndotl * vec3_dot(lnrm, vec3_neg(ldir)) * larea *
			  (1.0f / (ldist * ldist)));
#endif

	// Sample indirect
	struct vec3 indirect = {0.0f, 0.0f, 0.0f};
#ifdef SAMPLE_INDIRECT
	if (depth < 2) {
		struct vec3 dir = rand_hemicos(randf(seed), randf(seed));
		struct vec3 ta, bta;
		create_onb(&ta, &bta, nrm);
		dir = vec3_add(vec3_add(vec3_scale(ta, dir.x),
		  vec3_scale(bta, dir.y)), vec3_scale(nrm, dir.z));

		//float cos_theta = vec3_dot(nrm, dir);
		//float pdf = cos_theta / PI;

		struct vec3 irr = trace3(vec3_add(pos,
		  vec3_scale(dir, EPS2)), dir, rd, depth + 1, seed);

		//colind = vec3_scale(vec3_mul(brdf, irr),
		//  cos_theta / pdf);
		indirect = vec3_mul(brdf, irr);
	}
#endif

	return vec3_add(direct, indirect);
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
				  //trace(eye, vec3_unit(vec3_sub(p, eye)),
				  //rd);
				  trace3(eye, vec3_unit(vec3_sub(p, eye)), rd,
				  0, &seed);

				acc[yofs + x] = vec3_add(acc[yofs + x], c);
				c = vec3_scale(acc[yofs + x], rspp);

				buf[yofs + x] = 0xff << 24 |
				  ((unsigned int)(255 * c.x) & 0xff) << 16 |
				  ((unsigned int)(255 * c.y) & 0xff) <<  8 |
				  ((unsigned int)(255 * c.z) & 0xff);
			}
		}
	}

	return 0;
}

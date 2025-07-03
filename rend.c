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

struct distid {
	float          dist;
	unsigned char  id;
};

int comp_distid(const void *a, const void *b)
{
	return ((struct distid *)a)->dist < ((struct distid *)b)->dist
	  ? 1 : -1;
}

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

// Wald et al, 2007, Ray Tracing Deformable Scenes Using Dynamic BVH
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
			  (float)lcnt * calc_area(lmi, lma) / BRANCH_MAX
			  : FLT_MAX;

			// Sweep from right
			rmi = vec3_min(rmi,
			  imin[a][INTERVAL_CNT - 1 - i]);
			rmin[INTERVAL_CNT - 2 - i] = rmi;

			rma = vec3_max(rma,
			  imax[a][INTERVAL_CNT - 1 - i]);
			rmax[INTERVAL_CNT - 2 - i] = rma;

			rcnt += icnt[a][INTERVAL_CNT - 1 - i];

			rsah[INTERVAL_CNT - 2 - i] = rcnt > 0 ?
			  (float)rcnt * calc_area(rmi, rma) / BRANCH_MAX
			  : FLT_MAX;
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

unsigned int partition(unsigned int start, unsigned int cnt, struct vec3 nmin,
                       struct aabb *aabbs, unsigned int *imap, struct split *s)
{
	// Partition in l and r of given split plane
	unsigned int l = start;
	unsigned int r = start + cnt;
	float invda = vec3_getc(s->invd, s->axis);
	float nmina = vec3_getc(nmin, s->axis);
	for (unsigned int i = 0; i < cnt; i++) {
		unsigned int id = imap[l];
		struct aabb *a = &aabbs[id];
		float bin = ((vec3_getc(a->min, s->axis)
		  + vec3_getc(a->max, s->axis)) * 0.5f
		  - nmina) * invda;
		if ((unsigned int)min(max((int)bin, 0),
		  INTERVAL_CNT - 1) <= s->pos) {
			l++;
		 } else {
			// Swap indices
			imap[l] = imap[--r];
			imap[r] = id;
		}
	}

	return l - start; // = left cnt
}

unsigned int embed_leaf4(unsigned char *ptr, unsigned int ofs,
                          unsigned int start, unsigned int cnt,
                          unsigned int *imap, struct rtri *tris)
{
	struct leaf4 *l = (struct leaf4 *)(ptr + ofs);
	unsigned int *ip = &imap[start];
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
		if (i < cnt - 1)
			ip++;
	}

	return sizeof(*l);
}

// Wald et al, 2008, Getting Rid of Packets
// Ernst et al, 2008, Multi Bounding Volume Hierarchies
// Dammertz et al, 2008, Shallow Bounding Volume Hierarchies For Fast SIMD
// Ray Tracing of Incoherent Rays
unsigned int build_bvh8(struct b8node *nodes, struct aabb *aabbs,
                        unsigned int *imap, struct rtri *tris,
                        unsigned int pcnt,
                        struct vec3 rootmin, struct vec3 rootmax)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Access our b8node nodes byte-wise for offset calculation
	unsigned char *ptr = (unsigned char *)nodes;

	struct b8node *n = nodes; // Curr node

	unsigned int ofs = sizeof(*n); // Ofs to next in nodes arr
	unsigned char childcnt = 1; // Horiz. child cnt, root starts with one
	unsigned int ncnt = 1; // Total node cnt

	// Prepare start/root node
	memset(n, 0, sizeof(*n));
	n->minx[0] = rootmin.x;
	n->miny[0] = rootmin.y;
	n->minz[0] = rootmin.z;
	n->maxx[0] = rootmax.x;
	n->maxy[0] = rootmax.y;
	n->maxz[0] = rootmax.z;
	((unsigned int *)&n->children)[0] = pcnt;
	// nodes' permutation mask will be used to temporarily store start id

	struct vec3 minext = // Min extent relative to root aabb
	  vec3_scale(vec3_sub(rootmax, rootmin), 0.00000001f);

	while (true) {
		// Horizontal split
		while (childcnt < BRANCH_MAX) {
			// Find child with biggest surface area
			int j = -1;
			float bcost = 0.0f;
			struct vec3 bmin;
			struct vec3 bmax;
			for (unsigned int i = 0; i < childcnt; i++) {
				if ((((unsigned int *)&n->children)[i]
				  & NODE_LEAF) == 0) {
					struct vec3 mi = {
					  n->minx[i], n->miny[i], n->minz[i]};
					struct vec3 ma = {
					  n->maxx[i], n->maxy[i], n->maxz[i]};
					float cost = calc_area(mi, ma);
					if (cost > bcost) {
						bcost = cost;
						j = i;
						bmin = mi;
						bmax = ma;
					}
				}
			}

			if (j < 0)
				break; // No child found that can be split

			unsigned int cnt = ((unsigned int *)&n->children)[j];
			unsigned int start = ((unsigned int *)&n->perm)[j];

			// Do not split this node horizontally?
			if (cnt <= (tris ? BLAS_LEAF_MAX : TLAS_LEAF_MAX)) {
				if (tris) {
					// Blas
					assert(ofs <= 0x7fffffff);
					((unsigned int *)&n->children)[j] =
					  NODE_LEAF | ofs; // Ofs to leaf
					ofs += embed_leaf4(ptr, ofs, start,
					  cnt, imap, tris);
				} else {
					// Tlas
					assert(imap[start] <= 0x7fffffff);
					((unsigned int *)&n->children)[j] =
					  NODE_LEAF | imap[start];
				}
				// Clear 'borrowed' permutation mask
				((unsigned int *)&n->perm)[j] = 0;

				continue;
			}

			struct split best;
			find_best_split(&best, start, cnt, bmin, bmax, minext,
			  aabbs, imap);

			unsigned int lcnt = partition(start, cnt, bmin, aabbs,
			  imap, &best);
			if (lcnt == 0 || lcnt == cnt) {
				dprintf("one side of the partition was empty (horiz. split) at sid: %d, l: %d, r: %d\n",
				  start, lcnt, cnt - lcnt);
				lcnt = min(max(lcnt, 1), lcnt - 1);
			}

			// Add new horiz child node from right split data
			((unsigned int *)&n->children)[childcnt] = cnt - lcnt;
			((unsigned int *)&n->perm)[childcnt] = start + lcnt;
			n->minx[childcnt] = best.rmin.x;
			n->miny[childcnt] = best.rmin.y;
			n->minz[childcnt] = best.rmin.z;
			n->maxx[childcnt] = best.rmax.x;
			n->maxy[childcnt] = best.rmax.y;
			n->maxz[childcnt] = best.rmax.z;

			childcnt++;

			// Update original child with the left split data
			((unsigned int *)&n->children)[j] = lcnt;
			n->minx[j] = best.lmin.x;
			n->miny[j] = best.lmin.y;
			n->minz[j] = best.lmin.z;
			n->maxx[j] = best.lmax.x;
			n->maxy[j] = best.lmax.y;
			n->maxz[j] = best.lmax.z;
		}

		// Vertical splitting, process children in reverse for stack
		for (unsigned char j = 0; j < childcnt; j++) {
			unsigned int cnt = ((unsigned int *)&n->children)[j];
			if (cnt & NODE_LEAF)
				continue;

			// Do not split this node vertically?
			unsigned int start = ((unsigned int *)&n->perm)[j];
			if (cnt <= (tris ? BLAS_LEAF_MAX : TLAS_LEAF_MAX)) {
				if (tris) {
					// Blas
					assert(ofs <= 0x7fffffff);
					((unsigned int *)&n->children)[j] =
					  NODE_LEAF | ofs; // Ofs to leaf
					ofs += embed_leaf4(ptr, ofs,
					  start, cnt, imap, tris);
				} else {
					// Tlas
					assert(imap[start] <= 0x7fffffff);
					((unsigned int *)&n->children)[j] =
					  NODE_LEAF | imap[start];
				}
				// Clear 'borrowed' permutation mask
				((unsigned int *)&n->perm)[j] = 0;

				continue;
			}

			struct vec3 mi = {n->minx[j], n->miny[j], n->minz[j]};
			struct vec3 ma = {n->maxx[j], n->maxy[j], n->maxz[j]};

			// Run a SAH binning step
			struct split best;
			find_best_split(&best, start, cnt, mi, ma, minext,
			  aabbs, imap);

			unsigned int lcnt = partition(start, cnt, mi,
			  aabbs, imap, &best);
			if (lcnt == 0 || lcnt == cnt) {
				dprintf("one side of the partition was empty (vert. split) at sid: %d, l: %d, r: %d\n",
				  start, lcnt, cnt - lcnt);
				// One side of partition is empty
				continue;
			}

			// Add new b8node containing l/r split data
			struct b8node *child = (struct b8node *)(ptr + ofs);
			memset(child, 0, sizeof(*child));

			((unsigned int *)&child->children)[0] = lcnt;
			((unsigned int *)&child->perm)[0] = start;
			child->minx[0] = best.lmin.x;
			child->miny[0] = best.lmin.y;
			child->minz[0] = best.lmin.z;
			child->maxx[0] = best.lmax.x;
			child->maxy[0] = best.lmax.y;
			child->maxz[0] = best.lmax.z;

			((unsigned int *)&child->children)[1] = cnt - lcnt;
			((unsigned int *)&child->perm)[1] = start + lcnt;
			child->minx[1] = best.rmin.x;
			child->miny[1] = best.rmin.y;
			child->minz[1] = best.rmin.z;
			child->maxx[1] = best.rmax.x;
			child->maxy[1] = best.rmax.y;
			child->maxz[1] = best.rmax.z;

			// Make interior node and link new vert child
			assert(ofs <= 0x7fffffff);
			((unsigned int *)&n->children)[j] = ofs;
			((unsigned int *)&n->perm)[j] = 0;

			// Schedule new interior node for horiz. split
			assert(spos < 64);
			stack[spos++] = ofs;

			// Added one new node vertically
			ofs += sizeof(*child);
			ncnt++;
		}

		// Create ordered child traversal permutation map
		for (unsigned char j = 0; j < BRANCH_MAX; j++) { // Quadrant
			struct vec3 dir = {
			  j & 1 ? 1.0f : -1.0f,
			  j & 2 ? 1.0f : -1.0f,
			  j & 4 ? 1.0f : -1.0f};
			struct distid cdi[BRANCH_MAX];
			for (unsigned char i = 0; i < BRANCH_MAX; i++) {
				cdi[i].id = i; // 3 bit child index
				if (i < childcnt) {
					// Aabb corner of ray dir
					struct vec3 corner = {
					  j & 1 ? n->minx[i] : n->maxx[i],
					  j & 2 ? n->miny[i] : n->maxy[i],
					  j & 4 ? n->minz[i] : n->maxz[i]};
					cdi[i].dist = vec3_dot(dir, corner);
				} else {
					// No child assigned, still gets sorted
					cdi[i].dist = FLT_MAX;
				}
			}

			// Sort dists thereby sorting/permuting child indices
			qsort(cdi, BRANCH_MAX, sizeof(*cdi), comp_distid);

			// Set perm map for all children and curr quadrant
			for (unsigned char i = 0; i < BRANCH_MAX; i++)
				((unsigned int *)&n->perm)[i]
				  |= cdi[i].id << (j * 3);
		}

		// Set all the remaining children (if any left) to empty
		for (; childcnt < BRANCH_MAX; childcnt++) {
			n->minx[childcnt] = FLT_MAX;
			n->miny[childcnt] = FLT_MAX;
			n->minz[childcnt] = FLT_MAX;
			n->maxx[childcnt] = -FLT_MAX;
			n->maxy[childcnt] = -FLT_MAX;
			n->maxz[childcnt] = -FLT_MAX;
		}

		if (spos > 0)
			n = (struct b8node *)(ptr + stack[--spos]);
		else
			break;

		// Newly added vert child always contains two horizontal nodes
		childcnt = 2;
	}

	return ncnt;
}

// Fuetterling et al., Accelerated Single Ray Tracing for Wide Vector Units
void intersect_blas_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                         struct b8node *blas, unsigned int instid,
                         bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[64];
	_Alignas(64) float dstack[64];
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

bool intersect_any_blas_impl(float tfar, struct vec3 ori, struct vec3 dir,
                             struct b8node *blas, bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[64];
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

void intersect_blas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    struct b8node *blas, unsigned int instid)
{
	intersect_blas_impl(h, ori, dir, blas, instid,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

bool intersect_any_blas(float tfar, struct vec3 ori, struct vec3 dir,
                        struct b8node *blas)
{
	return intersect_any_blas_impl(tfar, ori, dir, blas,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

void intersect_tlas_impl(struct hit *h, struct vec3 ori, struct vec3 dir,
                         struct b8node *nodes, struct rinst *insts,
                         unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[64];
	_Alignas(64) float dstack[64];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)&nodes[tlasofs];
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

		// Leaf, check instance blas
		unsigned int instid = ofs & ~NODE_LEAF;
		struct rinst *ri = &insts[instid];

		// Transform ray into object space of instance
		float inv[16];
		mat4_from3x4(inv, ri->globinv);

		intersect_blas(h,
		  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
		  &nodes[ri->triofs << 1], instid);

		// Pop next node from stack if something is left
		if (spos > 0)
			ofs = stack[--spos];
		else
			return;
	}
}

bool intersect_any_tlas_impl(float tfar, struct vec3 ori, struct vec3 dir,
                             struct b8node *nodes, struct rinst *insts,
                             unsigned int tlasofs, bool dx, bool dy, bool dz)
{
	_Alignas(64) unsigned int stack[64];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)&nodes[tlasofs];
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

		// Leaf, check instance blas
		struct rinst *ri = &insts[ofs & ~NODE_LEAF];

		// Transform ray into object space of instance
		float inv[16];
		mat4_from3x4(inv, ri->globinv);

		if (intersect_any_blas(tfar, mat4_mulpos(inv, ori),
		  mat4_muldir(inv, dir), &nodes[ri->triofs << 1]))
			return true;

		// Pop next node from stack if something is left
		if (spos > 0)
			ofs = stack[--spos];
		else
			return false;
	}
}

void intersect_tlas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    struct b8node *nodes, struct rinst *insts,
                    unsigned int tlasofs)
{
	intersect_tlas_impl(h, ori, dir, nodes, insts, tlasofs,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

bool intersect_any_tlas(float tfar, struct vec3 ori, struct vec3 dir,
                        struct b8node *nodes, struct rinst *insts,
                        unsigned int tlasofs)
{
	return intersect_any_tlas_impl(tfar, ori, dir, nodes, insts, tlasofs,
	  dir.x >= 0.0f, dir.y >= 0.0f, dir.z >= 0.0f);
}

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = aligned_alloc(64, maxmtls * sizeof(*rd->mtls));

	rd->tris = aligned_alloc(64, maxtris * sizeof(*rd->tris));
	rd->nrms = aligned_alloc(64, maxtris * sizeof(*rd->nrms));

	rd->insts = aligned_alloc(64, maxinsts * sizeof(*rd->insts));
	rd->aabbs = aligned_alloc(64, maxinsts * sizeof(*rd->aabbs));

	// Bvh nodes for blas and tlas
	rd->b8nodes = aligned_alloc(64, 2 * (maxtris + maxinsts)
	  * sizeof(*rd->b8nodes));

	// Start of tlas nodes
	rd->tlasofs = 2 * maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->b8nodes);
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
		struct b8node *rn = &rd->b8nodes[triofs << 1]; // Root node
		if (!((unsigned int *)&rn->children)[0]) { // Not processed
			unsigned int tricnt = ri->tricnt;
			struct rtri *tp = &rd->tris[triofs];
			unsigned int *imap = malloc(tricnt * sizeof(*imap));
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
				imap[i] = i;
				tp++;
				ap++;
			}

			unsigned int ncnt = build_bvh8(
			  &rd->b8nodes[triofs << 1], aabbs, imap,
			  &rd->tris[triofs], tricnt, rmin, rmax);
			dprintf("Node cnt: %d\n", ncnt);

			free(aabbs);
			free(imap);
		}
	}
}

void rend_prepdynamic(struct rdata *rd)
{
	dprintf("> Creating tlas\n");
	struct aabb *ap = rd->aabbs; // World space aabbs of instances
	struct vec3 rmin = {FLT_MAX, FLT_MAX, FLT_MAX};
	struct vec3 rmax = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	unsigned int imap[rd->instcnt]; // On stack
	for (unsigned int i = 0; i < rd->instcnt; i++) {
		rmin = vec3_min(rmin, ap->min);
		rmax = vec3_max(rmax, ap->max);
		imap[i] = i;
		ap++;
	}

	unsigned int ncnt = build_bvh8(&rd->b8nodes[rd->tlasofs],
	  rd->aabbs, imap, NULL /* no tris */, rd->instcnt, rmin, rmax);
	dprintf("Node cnt: %d\n", ncnt);
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

	intersect_tlas(&h, o, d, rd->b8nodes, rd->insts, rd->tlasofs);

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

	intersect_tlas(&h, o, d, rd->b8nodes, rd->insts, rd->tlasofs);

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
	intersect_tlas(&h, o, d, rd->b8nodes, rd->insts, rd->tlasofs);

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
		if (!intersect_any_tlas(ldist, vec3_add(pos,
		  vec3_scale(ldir, EPS2)), ldir, rd->b8nodes, rd->insts,
		  rd->tlasofs))
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

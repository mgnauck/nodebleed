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

//#define PERSP_DIV

// Max 4096 instances
#define INST_ID_BITS  12
#define INST_ID_MASK  0xfff

#define INTERVAL_CNT  16 // Binning intervals

#define PCKT_W     4
#define PCKT_H     2
#define PCKT_SZ    (PCKT_W * PCKT_H)
#define MAX_PCKTS  16

static __m256i compr_lut[256];
static __m256i defchildnum8;

static __m256 pcktxofs8;
static __m256 pcktyofs8;

static __m256 zero8;
static __m256 half8;
static __m256 one8;
static __m256 twopi8;
static __m256 three8;
static __m128 zero4;
static __m128 one4;
static __m128 fltmax4;

static unsigned int blkszx;
static unsigned int blkszy;
static unsigned int blkpcktcnt;

static unsigned int *mortx;
static unsigned int *morty;

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

void rend_staticinit(unsigned int bszx, unsigned int bszy)
{
	// 8 bit mask: 0 will be compressed, 1's will be considered (= hit)
	for (unsigned int bm = 0; bm < 256; bm++) {
		unsigned long long p = get_perm(bm);
		//dprintf("0x%llx\n", p);
		compr_lut[bm] = _mm256_cvtepu8_epi32( // Convert 8x 8 bit to 32
		  _mm_cvtsi64_si128(p)); // Copy to lower 128, zero upper
	}

	// Default child nums/lanes (will be ordered, masked and compressed)
	defchildnum8 = _mm256_setr_epi32(0, 1, 2, 3, 4, 5, 6, 7);

	// Packet position ofs (assuming 4x2)
	pcktxofs8 = _mm256_set_ps(3, 2, 1, 0, 3, 2, 1, 0);
	pcktyofs8 = _mm256_set_ps(1, 1, 1, 1, 0, 0, 0, 0);

	// Default values 4 and 8 lanes
	zero8 = _mm256_setzero_ps();
	half8 = _mm256_set1_ps(0.5f);
	one8 = _mm256_set1_ps(1.0f);
	twopi8 = _mm256_set1_ps(TWO_PI);
	three8 = _mm256_set1_ps(3.0f);
	zero4 = _mm_setzero_ps();
	one4 = _mm_set1_ps(1.0f);
	fltmax4 = _mm_set1_ps(FLT_MAX);

	blkszx = bszx;
	blkszy = bszy;

	blkpcktcnt = bszx * bszy / PCKT_SZ;

	mortx = malloc(blkpcktcnt * sizeof(*mortx));
	morty = malloc(blkpcktcnt * sizeof(*morty));
	for (unsigned int i = 0; i < blkpcktcnt; i++) {
		mortx[i] = mortdecx(i);
		morty[i] = mortdecy(i);
	}
}

void rend_staticrelease(void)
{
	free(morty);
	free(mortx);
}

void mulpos_m256(__m256 * restrict ox8, __m256 * restrict oy8,
                 __m256 * restrict oz8,
                 __m256 x8, __m256 y8, __m256 z8, float m[16])
{
#ifdef PERSP_DIV
	__m256 tx8, ty8, tz8, tw8;
#else
	__m256 tx8, ty8, tz8;
#endif

	__m256 m0 = _mm256_set1_ps(m[0]);
	__m256 m1 = _mm256_set1_ps(m[1]);
	__m256 m2 = _mm256_set1_ps(m[2]);
	__m256 m3 = _mm256_set1_ps(m[3]);

	tx8 = _mm256_mul_ps(x8, m0);
	tx8 = _mm256_fmadd_ps(y8, m1, tx8);
	tx8 = _mm256_fmadd_ps(z8, m2, tx8);
	tx8 = _mm256_add_ps(m3, tx8);

	__m256 m4 = _mm256_set1_ps(m[4]);
	__m256 m5 = _mm256_set1_ps(m[5]);
	__m256 m6 = _mm256_set1_ps(m[6]);
	__m256 m7 = _mm256_set1_ps(m[7]);

	ty8 = _mm256_mul_ps(x8, m4);
	ty8 = _mm256_fmadd_ps(y8, m5, ty8);
	ty8 = _mm256_fmadd_ps(z8, m6, ty8);
	ty8 = _mm256_add_ps(m7, ty8);

	__m256 m8 = _mm256_set1_ps(m[8]);
	__m256 m9 = _mm256_set1_ps(m[9]);
	__m256 m10 = _mm256_set1_ps(m[10]);
	__m256 m11 = _mm256_set1_ps(m[11]);

	tz8 = _mm256_mul_ps(x8, m8);
	tz8 = _mm256_fmadd_ps(y8, m9, tz8);
	tz8 = _mm256_fmadd_ps(z8, m10, tz8);
	tz8 = _mm256_add_ps(m11, tz8);

#ifdef PERSP_DIV
	__m256 m12 = _mm256_set1_ps(m[12]);
	__m256 m13 = _mm256_set1_ps(m[13]);
	__m256 m14 = _mm256_set1_ps(m[14]);
	__m256 m15 = _mm256_set1_ps(m[15]);

	tw8 = _mm256_mul_ps(x8, m12);
	tw8 = _mm256_fmadd_ps(y8, m13, tw8);
	tw8 = _mm256_fmadd_ps(z8, m14, tw8);
	tw8 = _mm256_add_ps(m15, tw8);

	*ox8 = _mm256_div_ps(tx8, tw8);
	*oy8 = _mm256_div_ps(ty8, tw8);
	*oz8 = _mm256_div_ps(tz8, tw8);
#else
	*ox8 = tx8;
	*oy8 = ty8;
	*oz8 = tz8;
#endif
}

void muldir_m256(__m256 * restrict ox8, __m256 * restrict oy8,
                 __m256 * restrict oz8,
                 __m256 x8, __m256 y8, __m256 z8, float m[16])
{
	__m256 m0 = _mm256_set1_ps(m[0]);
	__m256 m1 = _mm256_set1_ps(m[1]);
	__m256 m2 = _mm256_set1_ps(m[2]);

	*ox8 = _mm256_fmadd_ps(z8, m2,
	  _mm256_fmadd_ps(y8, m1,
	  _mm256_mul_ps(x8, m0)));

	__m256 m4 = _mm256_set1_ps(m[4]);
	__m256 m5 = _mm256_set1_ps(m[5]);
	__m256 m6 = _mm256_set1_ps(m[6]);

	*oy8 = _mm256_fmadd_ps(z8, m6,
	  _mm256_fmadd_ps(y8, m5,
	  _mm256_mul_ps(x8, m4)));

	__m256 m8 = _mm256_set1_ps(m[8]);
	__m256 m9 = _mm256_set1_ps(m[9]);
	__m256 m10 = _mm256_set1_ps(m[10]);

	*oz8 = _mm256_fmadd_ps(z8, m10,
	  _mm256_fmadd_ps(y8, m9,
	  _mm256_mul_ps(x8, m8)));
}

bool iscoh(__m256 v8)
{
	unsigned char mask = _mm256_movemask_ps(v8);
	//assert((mask == 0 || mask == 0xff) ==
	//  (abs((~mask & 0xff) - mask) == 0xff));
	return abs((~mask & 0xff) - mask) == 0xff;
}

bool iscoh3(__m256 x8, __m256 y8, __m256 z8)
{
	return iscoh(x8) && iscoh(y8) && iscoh(z8);
}

bool iscohval(unsigned char *bmask, __m256 v8, unsigned char shift)
{
	// bmask is irrelevant if v8 is not coherent
	unsigned char mask = _mm256_movemask_ps(v8);
	*bmask |= (mask & 1) << shift;
	return abs((~mask & 0xff) - mask) == 0xff;
}

bool iscohval3(unsigned char *bmask, __m256 x8, __m256 y8, __m256 z8)
{
	bool res = iscohval(bmask, x8, 0);
	res &= iscohval(bmask, y8, 1);
	return res & iscohval(bmask, z8, 2);
}

float calc_area(struct vec3 mi, struct vec3 ma)
{
	struct vec3 d = vec3_sub(ma, mi);
	return d.x * d.y + d.y * d.z + d.z * d.x;
}

// Wald, 2007, On fast Construction of SAH-based Bounding Volume Hierarchies
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

		l->v0x4[i] = tri->v0.x;
		l->v0y4[i] = tri->v0.y;
		l->v0z4[i] = tri->v0.z;

		struct vec3 e0 = vec3_sub(
		  tri->v1, tri->v0);
		l->e0x4[i] = e0.x;
		l->e0y4[i] = e0.y;
		l->e0z4[i] = e0.z;

		struct vec3 e1 = vec3_sub(
		  tri->v2, tri->v0);
		l->e1x4[i] = e1.x;
		l->e1y4[i] = e1.y;
		l->e1z4[i] = e1.z;

		((unsigned int *)&l->id4)[i] = *ip;

		// Replicate last tri if not 4
		if (i < cnt - 1)
			ip++;
	}

	// Store tri cnt for pckt traversal or non-SIMD variants
	l->tricnt = cnt;

	return sizeof(*l);
}

// Wald et al, 2008, Getting Rid of Packets
// Ernst et al, 2008, Multi Bounding Volume Hierarchies
// Dammertz et al, 2008, Shallow Bounding Volume Hierarchies For Fast SIMD Ray
// Tracing of Incoherent Rays
unsigned int build_bvh8(struct bnode8 *nodes, struct aabb *aabbs,
                        unsigned int *imap, struct rtri *tris,
                        unsigned int pcnt,
                        struct vec3 rootmin, struct vec3 rootmax)
{
	unsigned int stack[64];
	unsigned int spos = 0;

	// Access our bnode8 nodes byte-wise for offset calculation
	unsigned char *ptr = (unsigned char *)nodes;

	struct bnode8 *n = nodes; // Curr node

	unsigned int ofs = sizeof(*n); // Ofs to next in nodes arr
	unsigned char childcnt = 1; // Horiz. child cnt, root starts with one
	unsigned int ncnt = 1; // Total node cnt

	// Prepare start/root node
	memset(n, 0, sizeof(*n));
	n->minx8[0] = rootmin.x;
	n->miny8[0] = rootmin.y;
	n->minz8[0] = rootmin.z;
	n->maxx8[0] = rootmax.x;
	n->maxy8[0] = rootmax.y;
	n->maxz8[0] = rootmax.z;
	((unsigned int *)&n->children8)[0] = pcnt;
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
				if ((((unsigned int *)&n->children8)[i]
				  & NODE_LEAF) == 0) {
					struct vec3 mi = {n->minx8[i],
					  n->miny8[i], n->minz8[i]};
					struct vec3 ma = {n->maxx8[i],
					  n->maxy8[i], n->maxz8[i]};
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

			unsigned int cnt = ((unsigned int *)&n->children8)[j];
			unsigned int start = ((unsigned int *)&n->perm8)[j];

			// Do not split this node horizontally?
			if (cnt <= (tris ? BLAS_LEAF_MAX : TLAS_LEAF_MAX)) {
				if (tris) {
					// Blas
					assert(ofs <= 0x7fffffff);
					((unsigned int *)&n->children8)[j] =
					  NODE_LEAF | ofs; // Ofs to leaf
					ofs += embed_leaf4(ptr, ofs, start,
					  cnt, imap, tris);
				} else {
					// Tlas
					assert(imap[start] <= 0x7fffffff);
					((unsigned int *)&n->children8)[j] =
					  NODE_LEAF | imap[start];
				}
				// Clear 'borrowed' permutation mask
				((unsigned int *)&n->perm8)[j] = 0;

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
			((unsigned int *)&n->children8)[childcnt] = cnt - lcnt;
			((unsigned int *)&n->perm8)[childcnt] = start + lcnt;
			n->minx8[childcnt] = best.rmin.x;
			n->miny8[childcnt] = best.rmin.y;
			n->minz8[childcnt] = best.rmin.z;
			n->maxx8[childcnt] = best.rmax.x;
			n->maxy8[childcnt] = best.rmax.y;
			n->maxz8[childcnt] = best.rmax.z;

			childcnt++;

			// Update original child with the left split data
			((unsigned int *)&n->children8)[j] = lcnt;
			n->minx8[j] = best.lmin.x;
			n->miny8[j] = best.lmin.y;
			n->minz8[j] = best.lmin.z;
			n->maxx8[j] = best.lmax.x;
			n->maxy8[j] = best.lmax.y;
			n->maxz8[j] = best.lmax.z;
		}

		// Vertical splitting, process children in reverse for stack
		for (unsigned char j = 0; j < childcnt; j++) {
			unsigned int cnt = ((unsigned int *)&n->children8)[j];
			if (cnt & NODE_LEAF)
				continue;

			// Do not split this node vertically?
			unsigned int start = ((unsigned int *)&n->perm8)[j];
			if (cnt <= (tris ? BLAS_LEAF_MAX : TLAS_LEAF_MAX)) {
				if (tris) {
					// Blas
					assert(ofs <= 0x7fffffff);
					((unsigned int *)&n->children8)[j] =
					  NODE_LEAF | ofs; // Ofs to leaf
					ofs += embed_leaf4(ptr, ofs,
					  start, cnt, imap, tris);
				} else {
					// Tlas
					assert(imap[start] <= 0x7fffffff);
					((unsigned int *)&n->children8)[j] =
					  NODE_LEAF | imap[start];
				}
				// Clear 'borrowed' permutation mask
				((unsigned int *)&n->perm8)[j] = 0;

				continue;
			}

			struct vec3 mi = {n->minx8[j], n->miny8[j],
			  n->minz8[j]};
			struct vec3 ma = {n->maxx8[j], n->maxy8[j],
			  n->maxz8[j]};

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

			// Add new bnode8 containing l/r split data
			struct bnode8 *child = (struct bnode8 *)(ptr + ofs);
			memset(child, 0, sizeof(*child));

			((unsigned int *)&child->children8)[0] = lcnt;
			((unsigned int *)&child->perm8)[0] = start;
			child->minx8[0] = best.lmin.x;
			child->miny8[0] = best.lmin.y;
			child->minz8[0] = best.lmin.z;
			child->maxx8[0] = best.lmax.x;
			child->maxy8[0] = best.lmax.y;
			child->maxz8[0] = best.lmax.z;

			((unsigned int *)&child->children8)[1] = cnt - lcnt;
			((unsigned int *)&child->perm8)[1] = start + lcnt;
			child->minx8[1] = best.rmin.x;
			child->miny8[1] = best.rmin.y;
			child->minz8[1] = best.rmin.z;
			child->maxx8[1] = best.rmax.x;
			child->maxy8[1] = best.rmax.y;
			child->maxz8[1] = best.rmax.z;

			// Make interior node and link new vert child
			assert(ofs <= 0x7fffffff);
			((unsigned int *)&n->children8)[j] = ofs;
			((unsigned int *)&n->perm8)[j] = 0;

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
					  j & 1 ? n->minx8[i] : n->maxx8[i],
					  j & 2 ? n->miny8[i] : n->maxy8[i],
					  j & 4 ? n->minz8[i] : n->maxz8[i]};
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
				((unsigned int *)&n->perm8)[i]
				  |= cdi[i].id << (j * 3);
		}

		// Set all the remaining children (if any left) to empty
		for (; childcnt < BRANCH_MAX; childcnt++) {
			n->minx8[childcnt] = FLT_MAX;
			n->miny8[childcnt] = FLT_MAX;
			n->minz8[childcnt] = FLT_MAX;
			n->maxx8[childcnt] = -FLT_MAX;
			n->maxy8[childcnt] = -FLT_MAX;
			n->maxz8[childcnt] = -FLT_MAX;
		}

		if (spos > 0)
			n = (struct bnode8 *)(ptr + stack[--spos]);
		else
			break;

		// Newly added vert child always contains two horizontal nodes
		childcnt = 2;
	}

	return ncnt;
}

// Fuetterling, 2017, Accelerated Single Ray Tracing for Wide Vector Units
// Fuetterling, 2019, Scalable Algorithms for Realistic Real-time Rendering
void intersect_blas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    struct bnode8 *blas, unsigned int instid)
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

	__m128 dx4 = _mm_set1_ps(dir.x);
	__m128 dy4 = _mm_set1_ps(dir.y);
	__m128 dz4 = _mm_set1_ps(dir.z);

	__m128 ox4 = _mm_set1_ps(ori.x);
	__m128 oy4 = _mm_set1_ps(ori.y);
	__m128 oz4 = _mm_set1_ps(ori.z);

	// Ray dir sign defines how to shift the permutation map
	bool dx = dir.x >= 0.0f;
	bool dy = dir.y >= 0.0f;
	bool dz = dir.z >= 0.0f;
	unsigned char shft = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
			__m256 t0x8 = _mm256_fmsub_ps(dx ? n->minx8 : n->maxx8,
			  idx8, rx8);
			__m256 t0y8 = _mm256_fmsub_ps(dy ? n->miny8 : n->maxy8,
			  idy8, ry8);
			__m256 t0z8 = _mm256_fmsub_ps(dz ? n->minz8 : n->maxz8,
			  idz8, rz8);

			__m256 t1x8 = _mm256_fmsub_ps(dx ? n->maxx8 : n->minx8,
			  idx8, rx8);
			__m256 t1y8 = _mm256_fmsub_ps(dy ? n->maxy8 : n->miny8,
			  idy8, ry8);
			__m256 t1z8 = _mm256_fmsub_ps(dz ? n->maxz8 : n->minz8,
			  idz8, rz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
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
				ofs = ((unsigned int *)&n->children8)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit
				// Order, compress and push child nodes + dists
				__m256i ord8 = _mm256_srli_epi32(n->perm8,
				  shft);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				// Ordered min distances and child node ids
				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  ord8);
				tmin8 = _mm256_permutevar8x32_ps(tmin8, ord8);

				// Map ordered hit mask to compressed indices
				__m256 cid8 = compr_lut[hitmaskord];

				// Permute to compress dists and child node ids
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(childrenord8,
				  cid8);
				__m256 distsfin8 = _mm256_permutevar8x32_ps(
				  tmin8, cid8);

				// Unaligned store dists and children on stacks
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);
				_mm256_storeu_ps(dstack + spos, distsfin8);

				assert(spos < 64 - hitcnt + 1);
				spos += hitcnt - 1; // Account for pushed nodes
				ofs = stack[spos]; // Next node
			}
		}

		// Intersect 4 embedded tris at once
		struct leaf4 *l = (struct leaf4 *)(ptr + (ofs & ~NODE_LEAF));

		// Moeller, Trumbore: Ray-triangle intersection
		// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/

		// pv = cross(dir, e1)
		__m128 pvx4 = _mm_fmsub_ps(dy4, l->e1z4,
		  _mm_mul_ps(dz4, l->e1y4));
		__m128 pvy4 = _mm_fmsub_ps(dz4, l->e1x4,
		  _mm_mul_ps(dx4, l->e1z4));
		__m128 pvz4 = _mm_fmsub_ps(dx4, l->e1y4,
		  _mm_mul_ps(dy4, l->e1x4));

		// tv = ori - v0
		__m128 tvx4 = _mm_sub_ps(ox4, l->v0x4);
		__m128 tvy4 = _mm_sub_ps(oy4, l->v0y4);
		__m128 tvz4 = _mm_sub_ps(oz4, l->v0z4);

		// det = dot(e0, pv)
		__m128 det4 = _mm_fmadd_ps(l->e0x4, pvx4,
		  _mm_fmadd_ps(l->e0y4, pvy4, _mm_mul_ps(l->e0z4, pvz4)));

		// qv = cross(tv, e0)
		__m128 qvx4 = _mm_fmsub_ps(tvy4, l->e0z4,
		  _mm_mul_ps(tvz4, l->e0y4));
		__m128 qvy4 = _mm_fmsub_ps(tvz4, l->e0x4,
		  _mm_mul_ps(tvx4, l->e0z4));
		__m128 qvz4 = _mm_fmsub_ps(tvx4, l->e0y4,
		  _mm_mul_ps(tvy4, l->e0x4));

		// idet = 1 / det
		__m128 idet4 = rcp4(det4);

		// u = idet * dot(tv, pv)
		__m128 u4 = _mm_mul_ps(idet4, _mm_fmadd_ps(tvx4, pvx4,
		  _mm_fmadd_ps(tvy4, pvy4, _mm_mul_ps(tvz4, pvz4))));

		// v = idet * dot(dir, qv)
		__m128 v4 = _mm_mul_ps(idet4, _mm_fmadd_ps(dx4, qvx4,
		  _mm_fmadd_ps(dy4, qvy4, _mm_mul_ps(dz4, qvz4))));

		// t = idet * dot(e1, qv)
		__m128 t4 = _mm_mul_ps(idet4, _mm_fmadd_ps(l->e1x4, qvx4,
		  _mm_fmadd_ps(l->e1y4, qvy4, _mm_mul_ps(l->e1z4, qvz4))));

		// u >= 0
		__m128 uzero4 = _mm_cmpge_ps(u4, zero4);

		// v >= 0
		__m128 vzero4 = _mm_cmpge_ps(v4, zero4);

		// u + v <= 1
		__m128 uvone4 = _mm_cmple_ps(_mm_add_ps(u4, v4), one4);

		// t > 0
		__m128 tzero4 = _mm_cmpgt_ps(t4, zero4);

		// t < h->t
		__m128 tnear4 = _mm_cmplt_ps(t4, _mm256_extractf128_ps(t8, 0));

		// Merge all comparison results into one final mask
		__m128 mask4 = _mm_and_ps(tnear4, _mm_and_ps(
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
			h->id = (((unsigned int *)&l->id4)[i] << INST_ID_BITS)
			  | instid;

			// Track closest h->t for future aabb+tri intersections
			t8 = _mm256_set1_ps(h->t);

			// Compress stack wrt to nearer h->t in batches of 8
			unsigned int spos2 = 0;
			for (i = 0; i < spos; i += 8) {
				__m256i nids8 = _mm256_load_si256(
				  (__m256i *)(stack + i));
				__m256 dists8 = _mm256_load_ps(dstack + i);

				// Nearer/eq ones are 1
				unsigned int nearmask = _mm256_movemask_ps(
				  _mm256_cmp_ps(dists8, t8, _CMP_LE_OQ));

				// Map nearer mask to compressed indices
				__m256i cid8 = compr_lut[nearmask];

				// Permute to compress dists and node ids
				nids8 =
				  _mm256_permutevar8x32_epi32(nids8, cid8);
				dists8 =
				  _mm256_permutevar8x32_ps(dists8, cid8);

				// Store compressed dists and node ids
				_mm256_storeu_si256((__m256i *)(stack + spos2),
				  nids8);
				_mm256_storeu_ps(dstack + spos2, dists8);

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

// Interval arithmetic from:
// Brian Hayes, 2003, A lucid interval
// Wald et al, 2006, Geometric and Arithmetic Culling Methods for Entire
// Ray Packets
void intersect_pckt_blas(__m256 *t8, __m256 *u8, __m256 *v8, __m256i *id8,
                         __m256 *ox8, __m256 *oy8, __m256 *oz8,
                         __m256 *dx8, __m256 *dy8, __m256 *dz8,
                         unsigned char pcnt, // Packet count
                         struct bnode8 *blas, unsigned int instid)
{
	_Alignas(64) unsigned int stack[64];
	_Alignas(64) unsigned int istack[64];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)blas;
	unsigned int ofs = 0; // Offset to curr node or leaf data

	unsigned char fpi = 0; // First packet index

	__m256i instid8 = _mm256_set1_epi32(instid);

	__m256 idx8[pcnt], idy8[pcnt], idz8[pcnt];
	__m256 rx8[pcnt], ry8[pcnt], rz8[pcnt];
	float miox, mioy, mioz;
	miox = mioy = mioz = FLT_MAX;
	float maox, maoy, maoz;
	maox = maoy = maoz = -FLT_MAX;
	float miidx, miidy, miidz;
	miidx = miidy = miidz = -FLT_MAX;
	float maidx, maidy, maidz;
	maidx = maidy = maidz = FLT_MAX;
	float maxt = -FLT_MAX;
	for (unsigned int i = 0; i < pcnt; i++) {
		idx8[i] = rcp8(dx8[i]);
		idy8[i] = rcp8(dy8[i]);
		idz8[i] = rcp8(dz8[i]);

		rx8[i] = _mm256_mul_ps(ox8[i], idx8[i]);
		ry8[i] = _mm256_mul_ps(oy8[i], idy8[i]);
		rz8[i] = _mm256_mul_ps(oz8[i], idz8[i]);

		// Calc interval ray bounds min/max ori/idir
		miox = min(miox, min8(ox8[i]));
		maox = max(maox, max8(ox8[i]));
		mioy = min(mioy, min8(oy8[i]));
		maoy = max(maoy, max8(oy8[i]));
		mioz = min(mioz, min8(oz8[i]));
		maoz = max(maoz, max8(oz8[i]));

		// Use interval arithmetic to precalc values for aabb intersect
		// a = [ mina, maxa ]
		// 1 / a = [ 1 / maxa, 1 / mina ] (assumes a does not incl 0!)
		miidx = max(miidx, max8(idx8[i]));
		maidx = min(maidx, min8(idx8[i]));
		miidy = max(miidy, max8(idy8[i]));
		maidy = min(maidy, min8(idy8[i]));
		miidz = max(miidz, max8(idz8[i]));
		maidz = min(maidz, min8(idz8[i]));

		// Curr max dist of the ray packets to cull the interval ray
		maxt = max(maxt, max8(t8[i]));
	}

	__m256 maxt8 = _mm256_set1_ps(maxt);

	__m256 miidx8 = _mm256_set1_ps(miidx);
	__m256 miidy8 = _mm256_set1_ps(miidy);
	__m256 miidz8 = _mm256_set1_ps(miidz);

	__m256 maidx8 = _mm256_set1_ps(maidx);
	__m256 maidy8 = _mm256_set1_ps(maidy);
	__m256 maidz8 = _mm256_set1_ps(maidz);

	// 1. ori * idir =
	//   [ min(mio * miid, mio * maid, mao * miid, mao * maid),
	//     max(mio * miid, mio * maid, mao * miid, mao * maid) ]
	// 2. Negate, so we can add (instead of sub) later on
	//  -[ mi, ma ] = [ -ma, -mi ]
	__m256 marx8 = _mm256_set1_ps(-min(miox * miidx, min(miox * maidx,
	  min(maox * miidx, maox * maidx))));
	__m256 mary8 = _mm256_set1_ps(-min(mioy * miidy, min(mioy * maidy,
	  min(maoy * miidy, maoy * maidy))));
	__m256 marz8 = _mm256_set1_ps(-min(mioz * miidz, min(mioz * maidz,
	  min(maoz * miidz, maoz * maidz))));

	__m256 mirx8 = _mm256_set1_ps(-max(miox * miidx, max(miox * maidx,
	  max(maox * miidx, maox * maidx))));
	__m256 miry8 = _mm256_set1_ps(-max(mioy * miidy, max(mioy * maidy,
	  max(maoy * miidy, maoy * maidy))));
	__m256 mirz8 = _mm256_set1_ps(-max(mioz * miidz, max(mioz * maidz,
	  max(maoz * miidz, maoz * maidz))));

	// Ray dir sign defines how to shift the permutation map
	// Assumes (primary) rays of this packet are coherent
	bool dx = miidx >= 0.0f;
	bool dy = miidy >= 0.0f;
	bool dz = miidz >= 0.0f;
	unsigned char shft = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
restart:
		if ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			__m256 minx8 = dx ? n->minx8 : n->maxx8;
			__m256 miny8 = dy ? n->miny8 : n->maxy8;
			__m256 minz8 = dz ? n->minz8 : n->maxz8;

			__m256 maxx8 = dx ? n->maxx8 : n->minx8;
			__m256 maxy8 = dy ? n->maxy8 : n->miny8;
			__m256 maxz8 = dz ? n->maxz8 : n->minz8;

			// Intersect interval ray with all 8 aabbs via
			// interval arithmetic
			__m256 t0x8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(minx8, miidx8),
			  _mm256_min_ps(_mm256_mul_ps(minx8, maidx8),
			  _mm256_min_ps(_mm256_mul_ps(maxx8, miidx8),
			  _mm256_mul_ps(maxx8, maidx8)))), mirx8);

			__m256 t0y8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(miny8, miidy8),
			  _mm256_min_ps(_mm256_mul_ps(miny8, maidy8),
			  _mm256_min_ps(_mm256_mul_ps(maxy8, miidy8),
			  _mm256_mul_ps(maxy8, maidy8)))), miry8);

			__m256 t0z8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(minz8, miidz8),
			  _mm256_min_ps(_mm256_mul_ps(minz8, maidz8),
			  _mm256_min_ps(_mm256_mul_ps(maxz8, miidz8),
			  _mm256_mul_ps(maxz8, maidz8)))), mirz8);

			__m256 t1x8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(minx8, miidx8),
			  _mm256_max_ps(_mm256_mul_ps(minx8, maidx8),
			  _mm256_max_ps(_mm256_mul_ps(maxx8, miidx8),
			  _mm256_mul_ps(maxx8, maidx8)))), marx8);

			__m256 t1y8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(miny8, miidy8),
			  _mm256_max_ps(_mm256_mul_ps(miny8, maidy8),
			  _mm256_max_ps(_mm256_mul_ps(maxy8, miidy8),
			  _mm256_mul_ps(maxy8, maidy8)))), mary8);

			__m256 t1z8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(minz8, miidz8),
			  _mm256_max_ps(_mm256_mul_ps(minz8, maidz8),
			  _mm256_max_ps(_mm256_mul_ps(maxz8, miidz8),
			  _mm256_mul_ps(maxz8, maidz8)))), marz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			// Only use the maximum t of the ray packet for the
			// conservative interval test
			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), maxt8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
			unsigned int hitmask = _mm256_movemask_ps(hitmask8);
			unsigned int hitcnt = __builtin_popcount(hitmask);

			if (hitcnt == 1) {
				// Invert count of leading zeros to get lane
				unsigned int child =
				  31 - __builtin_clz(hitmask);
				// Push ofs to child node
				assert(spos < 64);
				stack[spos] =
				  ((unsigned int *)&n->children8)[child];
				// Push prnt ofs, fpi and lane/child num
				assert(ofs <= 0x1ffffff);
				istack[spos++] =
				  (ofs << 7) | (fpi << 3) | child;
			} else if (hitcnt > 1) {
				// Order, compress, push child node ofs and
				// parent ofs + child num
				__m256i ord8 = _mm256_srli_epi32(n->perm8,
				  shft);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				// Order
				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  ord8);

				assert(ofs <= 0x1ffffff);
				// prnt ofs << 7 | fpi << 3 | child num
				// child num is per lane
				// ofs never points to leaf, so can shift it
				unsigned int ofs_fpi = (ofs << 7) | (fpi << 3);
				__m256i ofs_fpi8 =
				  _mm256_set1_epi32(ofs_fpi);
				__m256i pcnumord8 =
				  _mm256_permutevar8x32_epi32(
				  _mm256_add_epi32(ofs_fpi8, defchildnum8),
				  ord8);

				// Map ordered hit mask to compressed indices
				__m256 cid8 = compr_lut[hitmaskord];

				// Permute to compress
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(childrenord8,
				  cid8);
				__m256i pcnumfin8 =
				  _mm256_permutevar8x32_epi32(pcnumord8, cid8);

				// Unaligned store to stacks
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);
				_mm256_storeu_si256((__m256i *)(istack + spos),
				  pcnumfin8);

				assert(spos < 64 - hitcnt);
				spos += hitcnt; // Account for pushed nodes
			}
		} else {
			// Intersect all rays of each pckt w/ each embedded tri
			struct leaf4 *l = (struct leaf4 *)(ptr
			  + (ofs & ~NODE_LEAF));

			maxt = -FLT_MAX; // Recalc tmax
			for (unsigned char j = 0; j < pcnt; j++) { // Each pckt
				for (unsigned char i = 0; i < l->tricnt; i++) {
					__m256 v0x8 = bcl4to8(l->v0x4, i);
					__m256 v0y8 = bcl4to8(l->v0y4, i);
					__m256 v0z8 = bcl4to8(l->v0z4, i);

					__m256 e0x8 = bcl4to8(l->e0x4, i);
					__m256 e0y8 = bcl4to8(l->e0y4, i);
					__m256 e0z8 = bcl4to8(l->e0z4, i);

					__m256 e1x8 = bcl4to8(l->e1x4, i);
					__m256 e1y8 = bcl4to8(l->e1y4, i);
					__m256 e1z8 = bcl4to8(l->e1z4, i);

					// Moeller, Trumbore: Ray-Tri intersect
					// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/

					// pv = cross(dir, e1)
					__m256 pvx8 = _mm256_fmsub_ps(dy8[j],
					  e1z8, _mm256_mul_ps(dz8[j], e1y8));
					__m256 pvy8 = _mm256_fmsub_ps(dz8[j],
					  e1x8, _mm256_mul_ps(dx8[j], e1z8));
					__m256 pvz8 = _mm256_fmsub_ps(dx8[j],
					  e1y8, _mm256_mul_ps(dy8[j], e1x8));

					// tv = ori - v0
					__m256 tvx8 =
					  _mm256_sub_ps(ox8[j], v0x8);
					__m256 tvy8 =
					  _mm256_sub_ps(oy8[j], v0y8);
					__m256 tvz8 =
					  _mm256_sub_ps(oz8[j], v0z8);

					// det = dot(e0, pv)
					__m256 det8 = _mm256_fmadd_ps(e0x8,
					  pvx8, _mm256_fmadd_ps(e0y8, pvy8,
					  _mm256_mul_ps(e0z8, pvz8)));

					// qv = cross(tv, e0)
					__m256 qvx8 = _mm256_fmsub_ps(tvy8,
					  e0z8, _mm256_mul_ps(tvz8, e0y8));
					__m256 qvy8 = _mm256_fmsub_ps(tvz8,
					  e0x8, _mm256_mul_ps(tvx8, e0z8));
					__m256 qvz8 = _mm256_fmsub_ps(tvx8,
					  e0y8, _mm256_mul_ps(tvy8, e0x8));

					// idet = 1 / det
					__m256 idet8 = rcp8(det8);

					// u = idet * dot(tv, pv)
					__m256 unew8 = _mm256_mul_ps(idet8,
					  _mm256_fmadd_ps(tvx8, pvx8,
					  _mm256_fmadd_ps(tvy8, pvy8,
					  _mm256_mul_ps(tvz8, pvz8))));

					// v = idet * dot(dir, qv)
					__m256 vnew8 = _mm256_mul_ps(idet8,
					  _mm256_fmadd_ps(dx8[j], qvx8,
					  _mm256_fmadd_ps(dy8[j], qvy8,
					  _mm256_mul_ps(dz8[j], qvz8))));

					// t = idet * dot(e1, qv)
					__m256 tnew8 = _mm256_mul_ps(idet8,
					  _mm256_fmadd_ps(e1x8, qvx8,
					  _mm256_fmadd_ps(e1y8, qvy8,
					  _mm256_mul_ps(e1z8, qvz8))));

					// u >= 0
					__m256 uzero8 = _mm256_cmp_ps(unew8,
					  zero8, _CMP_GE_OQ);

					// v >= 0
					__m256 vzero8 = _mm256_cmp_ps(vnew8,
					  zero8, _CMP_GE_OQ);

					// u + v <= 1
					__m256 uvone8 = _mm256_cmp_ps(
					  _mm256_add_ps(unew8, vnew8), one8,
					  _CMP_LE_OQ);

					// t > 0
					__m256 tzero8 = _mm256_cmp_ps(tnew8,
					  zero8, _CMP_GT_OQ);

					// t < h->t
					__m256 tnear8 = _mm256_cmp_ps(tnew8,
					  t8[j], _CMP_LT_OQ);

					// Merge comparison res into final mask
					__m256 mask8 = _mm256_and_ps(tnear8,
					  _mm256_and_ps(_mm256_and_ps(uzero8,
					  vzero8), _mm256_and_ps(uvone8,
					  tzero8)));

					if (_mm256_movemask_ps(mask8)) {

						__m256i idnew8 =
						  bcl4ito8i(l->id4, i);
						idnew8 =
						  _mm256_slli_epi32(idnew8,
						  INST_ID_BITS);
						idnew8 =
						  _mm256_or_si256(idnew8,
						  instid8);

						// Update lane if mask is hit
						t8[j] = _mm256_blendv_ps(t8[j],
						  tnew8, mask8);
						u8[j] = _mm256_blendv_ps(u8[j],
						  unew8, mask8);
						v8[j] = _mm256_blendv_ps(v8[j],
						  vnew8, mask8);
						id8[j] = _mm256_blendv_epi8(
						  id8[j], idnew8, mask8);
					}

					// Recalc tmax, even if no hit
					maxt = max(maxt, max8(t8[j]));
				}
			}

			// Update maxt
			maxt8 = _mm256_set1_ps(maxt);
		}

		// Check actual rays of packets against popped nodes
		while (true) {
			unsigned int ppc; // Parent ofs, fpi, child num
			if (spos > 0) {
				ppc = istack[--spos];
				ofs = stack[spos];
			} else
				return;

			// Fetch node at ofs
			struct bnode8 *p = (struct bnode8 *)(ptr + (ppc >> 7));

			// Intersect packet rays at once with child's aabb
			__m256i child = _mm256_set1_epi32(ppc & 7);

			__m256 minx8 =
			  _mm256_permutevar8x32_ps(p->minx8, child);
			__m256 miny8 =
			  _mm256_permutevar8x32_ps(p->miny8, child);
			__m256 minz8 =
			  _mm256_permutevar8x32_ps(p->minz8, child);
			__m256 maxx8 =
			  _mm256_permutevar8x32_ps(p->maxx8, child);
			__m256 maxy8 =
			  _mm256_permutevar8x32_ps(p->maxy8, child);
			__m256 maxz8 =
			  _mm256_permutevar8x32_ps(p->maxz8, child);

			// Intersect all packets, starting with fpi from stack
			fpi = (ppc >> 3) & 0xf;
			unsigned char i = fpi;
			do {
				__m256 t0x8 =
				  _mm256_fmsub_ps(dx ? minx8 : maxx8,
				  idx8[i], rx8[i]);
				__m256 t0y8 =
				  _mm256_fmsub_ps(dy ? miny8 : maxy8,
				  idy8[i], ry8[i]);
				__m256 t0z8 =
				  _mm256_fmsub_ps(dz ? minz8 : maxz8,
				  idz8[i], rz8[i]);

				__m256 t1x8 =
				  _mm256_fmsub_ps(dx ? maxx8 : minx8,
				  idx8[i], rx8[i]);
				__m256 t1y8 =
				  _mm256_fmsub_ps(dy ? maxy8 : miny8,
				  idy8[i], ry8[i]);
				__m256 t1z8 =
				  _mm256_fmsub_ps(dz ? maxz8 : minz8,
				  idz8[i], rz8[i]);

				__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
				  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

				__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
				  _mm256_min_ps(t1x8, t1y8), t1z8), t8[i]);

				__m256 hit8 = _mm256_cmp_ps(tmin8, tmax8,
				  _CMP_LE_OQ);
				if (_mm256_movemask_ps(hit8) > 0) {
					fpi = i;
					goto restart;
				}

				// Next packet index
				i = (i + 1) % pcnt;
			} while (i != fpi);
		}
	}
}

bool intersect_any_blas(float tfar, struct vec3 ori, struct vec3 dir,
                        struct bnode8 *blas)
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

	__m128 dx4 = _mm_set1_ps(dir.x);
	__m128 dy4 = _mm_set1_ps(dir.y);
	__m128 dz4 = _mm_set1_ps(dir.z);

	__m128 ox4 = _mm_set1_ps(ori.x);
	__m128 oy4 = _mm_set1_ps(ori.y);
	__m128 oz4 = _mm_set1_ps(ori.z);

	bool dx = dir.x >= 0.0f;
	bool dy = dir.y >= 0.0f;
	bool dz = dir.z >= 0.0f;

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
			__m256 t0x8 = _mm256_fmsub_ps(dx ? n->minx8 : n->maxx8,
			  idx8, rx8);
			__m256 t0y8 = _mm256_fmsub_ps(dy ? n->miny8 : n->maxy8,
			  idy8, ry8);
			__m256 t0z8 = _mm256_fmsub_ps(dz ? n->minz8 : n->maxz8,
			  idz8, rz8);

			__m256 t1x8 = _mm256_fmsub_ps(dx ? n->maxx8 : n->minx8,
			  idx8, rx8);
			__m256 t1y8 = _mm256_fmsub_ps(dy ? n->maxy8 : n->miny8,
			  idy8, ry8);
			__m256 t1z8 = _mm256_fmsub_ps(dz ? n->maxz8 : n->minz8,
			  idz8, rz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
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
				ofs = ((unsigned int *)&n->children8)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit

				// Compress unordered child nodes via lut
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  compr_lut[hitmask]);

				// Unaligned store children on stack
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);

				assert(spos < 64 - hitcnt + 1);
				spos += hitcnt - 1; // Account for pushed nodes
				ofs = stack[spos]; // Next node
			}
		}

		// Intersect 4 embedded tris at once
		struct leaf4 *l = (struct leaf4 *)(ptr + (ofs & ~NODE_LEAF));

		// Moeller, Trumbore: Ray-triangle intersection
		// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/raytri/

		// pv = cross(dir, e1)
		__m128 pvx4 = _mm_fmsub_ps(dy4, l->e1z4,
		  _mm_mul_ps(dz4, l->e1y4));
		__m128 pvy4 = _mm_fmsub_ps(dz4, l->e1x4,
		  _mm_mul_ps(dx4, l->e1z4));
		__m128 pvz4 = _mm_fmsub_ps(dx4, l->e1y4,
		  _mm_mul_ps(dy4, l->e1x4));

		// tv = ori - v0
		__m128 tvx4 = _mm_sub_ps(ox4, l->v0x4);
		__m128 tvy4 = _mm_sub_ps(oy4, l->v0y4);
		__m128 tvz4 = _mm_sub_ps(oz4, l->v0z4);

		// det = dot(e0, pv)
		__m128 det4 = _mm_fmadd_ps(l->e0x4, pvx4,
		  _mm_fmadd_ps(l->e0y4, pvy4, _mm_mul_ps(l->e0z4, pvz4)));

		// qv = cross(tv, e0)
		__m128 qvx4 = _mm_fmsub_ps(tvy4, l->e0z4,
		  _mm_mul_ps(tvz4, l->e0y4));
		__m128 qvy4 = _mm_fmsub_ps(tvz4, l->e0x4,
		  _mm_mul_ps(tvx4, l->e0z4));
		__m128 qvz4 = _mm_fmsub_ps(tvx4, l->e0y4,
		  _mm_mul_ps(tvy4, l->e0x4));

		// idet = 1 / det
		__m128 idet4 = rcp4(det4);

		// u = idet * dot(tv, pv)
		__m128 u4 = _mm_mul_ps(idet4, _mm_fmadd_ps(tvx4, pvx4,
		  _mm_fmadd_ps(tvy4, pvy4, _mm_mul_ps(tvz4, pvz4))));

		// v = idet * dot(dir, qv)
		__m128 v4 = _mm_mul_ps(idet4, _mm_fmadd_ps(dx4, qvx4,
		  _mm_fmadd_ps(dy4, qvy4, _mm_mul_ps(dz4, qvz4))));

		// t = idet * dot(e1, qv)
		__m128 t4 = _mm_mul_ps(idet4, _mm_fmadd_ps(l->e1x4, qvx4,
		  _mm_fmadd_ps(l->e1y4, qvy4, _mm_mul_ps(l->e1z4, qvz4))));

		// u >= 0
		__m128 uzero4 = _mm_cmpge_ps(u4, zero4);

		// v >= 0
		__m128 vzero4 = _mm_cmpge_ps(v4, zero4);

		// u + v <= 1
		__m128 uvone4 = _mm_cmple_ps(_mm_add_ps(u4, v4), one4);

		// t > 0
		__m128 tzero4 = _mm_cmpgt_ps(t4, zero4);

		// t < h->t
		__m128 tnear4 = _mm_cmplt_ps(t4, _mm256_extractf128_ps(t8, 0));

		// Merge all comparison results into one final mask
		__m128 mask4 = _mm_and_ps(tnear4, _mm_and_ps(
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

void intersect_tlas(struct hit *h, struct vec3 ori, struct vec3 dir,
                    struct bnode8 *nodes, struct rinst *insts,
                    unsigned int tlasofs)
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

	// Ray dir sign defines how to shift the permutation map
	bool dx = dir.x >= 0.0f;
	bool dy = dir.y >= 0.0f;
	bool dz = dir.z >= 0.0f;
	unsigned char shft = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
			__m256 t0x8 = _mm256_fmsub_ps(dx ? n->minx8 : n->maxx8,
			  idx8, rx8);
			__m256 t0y8 = _mm256_fmsub_ps(dy ? n->miny8 : n->maxy8,
			  idy8, ry8);
			__m256 t0z8 = _mm256_fmsub_ps(dz ? n->minz8 : n->maxz8,
			  idz8, rz8);

			__m256 t1x8 = _mm256_fmsub_ps(dx ? n->maxx8 : n->minx8,
			  idx8, rx8);
			__m256 t1y8 = _mm256_fmsub_ps(dy ? n->maxy8 : n->miny8,
			  idy8, ry8);
			__m256 t1z8 = _mm256_fmsub_ps(dz ? n->maxz8 : n->minz8,
			  idz8, rz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
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
				ofs = ((unsigned int *)&n->children8)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit
				// Order, compress and push child nodes + dists
				__m256i ord8 = _mm256_srli_epi32(n->perm8,
				  shft);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				// Ordered min distances and child node ids
				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  ord8);
				tmin8 = _mm256_permutevar8x32_ps(tmin8, ord8);

				// Map ordered hit mask to compressed indices
				__m256 cid8 = compr_lut[hitmaskord];

				// Permute to compress dists and child node ids
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(childrenord8,
				  cid8);
				__m256 distsfin8 = _mm256_permutevar8x32_ps(
				  tmin8, cid8);

				// Unaligned store dists and children on stacks
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);
				_mm256_storeu_ps(dstack + spos, distsfin8);

				assert(spos < 64 - hitcnt + 1);
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

		float tt = h->t;
		intersect_blas(h,
		  mat4_mulpos(inv, ori), mat4_muldir(inv, dir),
		  &nodes[ri->triofs << 1], instid);

		if (h->t < tt) {
			// Track closest h->t for future aabb+tri intersections
			t8 = _mm256_set1_ps(h->t);

			// Compress stack wrt to nearer h->t in batches of 8
			unsigned int spos2 = 0;
			for (unsigned int i = 0; i < spos; i += 8) {
				__m256i nids8 = _mm256_load_si256(
				  (__m256i *)(stack + i));
				__m256 dists8 = _mm256_load_ps(dstack + i);

				// Nearer/eq ones are 1
				unsigned int nearmask = _mm256_movemask_ps(
				  _mm256_cmp_ps(dists8, t8, _CMP_LE_OQ));

				// Map nearer mask to compressed indices
				__m256i cid8 = compr_lut[nearmask];

				// Permute to compress dists and node ids
				nids8 =
				  _mm256_permutevar8x32_epi32(nids8, cid8);
				dists8 =
				  _mm256_permutevar8x32_ps(dists8, cid8);

				// Store compressed dists and node ids
				_mm256_storeu_si256((__m256i *)(stack + spos2),
				  nids8);
				_mm256_storeu_ps(dstack + spos2, dists8);

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

void intersect_pckt_tlas(__m256 *t8, __m256 *u8, __m256 *v8, __m256i *id8,
                         __m256 *ox8, __m256 *oy8, __m256 *oz8,
                         __m256 *dx8, __m256 *dy8, __m256 *dz8,
                         unsigned char pcnt, // Packet count
                         struct bnode8 *nodes, struct rinst *insts,
                         unsigned int tlasofs)
{
	_Alignas(64) unsigned int stack[64];
	_Alignas(64) unsigned int istack[64];
	unsigned int spos = 0;

	unsigned char *ptr = (unsigned char *)&nodes[tlasofs];
	unsigned int ofs = 0; // Offset to curr node or leaf data

	unsigned char fpi = 0; // First packet index

	__m256 idx8[pcnt], idy8[pcnt], idz8[pcnt];
	__m256 rx8[pcnt], ry8[pcnt], rz8[pcnt];
	float miox, mioy, mioz;
	miox = mioy = mioz = FLT_MAX;
	float maox, maoy, maoz;
	maox = maoy = maoz = -FLT_MAX;
	float miidx, miidy, miidz;
	miidx = miidy = miidz = -FLT_MAX;
	float maidx, maidy, maidz;
	maidx = maidy = maidz = FLT_MAX;
	float maxt = -FLT_MAX;
	for (unsigned int i = 0; i < pcnt; i++) {
		idx8[i] = rcp8(dx8[i]);
		idy8[i] = rcp8(dy8[i]);
		idz8[i] = rcp8(dz8[i]);

		rx8[i] = _mm256_mul_ps(ox8[i], idx8[i]);
		ry8[i] = _mm256_mul_ps(oy8[i], idy8[i]);
		rz8[i] = _mm256_mul_ps(oz8[i], idz8[i]);

		// Calc interval ray bounds min/max ori/idir
		miox = min(miox, min8(ox8[i]));
		maox = max(maox, max8(ox8[i]));
		mioy = min(mioy, min8(oy8[i]));
		maoy = max(maoy, max8(oy8[i]));
		mioz = min(mioz, min8(oz8[i]));
		maoz = max(maoz, max8(oz8[i]));

		// Use interval arithmetic to precalc values for aabb intersect
		// a = [ mina, maxa ]
		// 1 / a = [ 1 / maxa, 1 / mina ] (assumes a does not incl 0!)
		miidx = max(miidx, max8(idx8[i]));
		maidx = min(maidx, min8(idx8[i]));
		miidy = max(miidy, max8(idy8[i]));
		maidy = min(maidy, min8(idy8[i]));
		miidz = max(miidz, max8(idz8[i]));
		maidz = min(maidz, min8(idz8[i]));

		// Curr max dist of the ray packets to cull the interval ray
		maxt = max(maxt, max8(t8[i]));
	}

	__m256 maxt8 = _mm256_set1_ps(maxt);

	__m256 miidx8 = _mm256_set1_ps(miidx);
	__m256 miidy8 = _mm256_set1_ps(miidy);
	__m256 miidz8 = _mm256_set1_ps(miidz);

	__m256 maidx8 = _mm256_set1_ps(maidx);
	__m256 maidy8 = _mm256_set1_ps(maidy);
	__m256 maidz8 = _mm256_set1_ps(maidz);

	// 1. ori * idir =
	//   [ min(mio * miid, mio * maid, mao * miid, mao * maid),
	//     max(mio * miid, mio * maid, mao * miid, mao * maid) ]
	// 2. Negate and switch, so we can add (instead of sub) later on
	//  -[ mi, ma ] = [ -ma, -mi ]
	__m256 marx8 = _mm256_set1_ps(-min(miox * miidx, min(miox * maidx,
	  min(maox * miidx, maox * maidx))));
	__m256 mary8 = _mm256_set1_ps(-min(mioy * miidy, min(mioy * maidy,
	  min(maoy * miidy, maoy * maidy))));
	__m256 marz8 = _mm256_set1_ps(-min(mioz * miidz, min(mioz * maidz,
	  min(maoz * miidz, maoz * maidz))));

	__m256 mirx8 = _mm256_set1_ps(-max(miox * miidx, max(miox * maidx,
	  max(maox * miidx, maox * maidx))));
	__m256 miry8 = _mm256_set1_ps(-max(mioy * miidy, max(mioy * maidy,
	  max(maoy * miidy, maoy * maidy))));
	__m256 mirz8 = _mm256_set1_ps(-max(mioz * miidz, max(mioz * maidz,
	  max(maoz * miidz, maoz * maidz))));

	// Ray dir sign defines how to shift the permutation map
	// Assumes (primary) rays of this packet are coherent
	bool dx = miidx >= 0.0f;
	bool dy = miidy >= 0.0f;
	bool dz = miidz >= 0.0f;
	unsigned char shft = ((dz << 2) | (dy << 1) | dx) * 3;

	while (true) {
restart:
		if ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			__m256 minx8 = dx ? n->minx8 : n->maxx8;
			__m256 miny8 = dy ? n->miny8 : n->maxy8;
			__m256 minz8 = dz ? n->minz8 : n->maxz8;

			__m256 maxx8 = dx ? n->maxx8 : n->minx8;
			__m256 maxy8 = dy ? n->maxy8 : n->miny8;
			__m256 maxz8 = dz ? n->maxz8 : n->minz8;

			// Intersect interval ray with all 8 aabbs via
			// interval arithmetic
			__m256 t0x8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(minx8, miidx8),
			  _mm256_min_ps(_mm256_mul_ps(minx8, maidx8),
			  _mm256_min_ps(_mm256_mul_ps(maxx8, miidx8),
			  _mm256_mul_ps(maxx8, maidx8)))), mirx8);

			__m256 t0y8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(miny8, miidy8),
			  _mm256_min_ps(_mm256_mul_ps(miny8, maidy8),
			  _mm256_min_ps(_mm256_mul_ps(maxy8, miidy8),
			  _mm256_mul_ps(maxy8, maidy8)))), miry8);

			__m256 t0z8 = _mm256_add_ps(
			  _mm256_min_ps(_mm256_mul_ps(minz8, miidz8),
			  _mm256_min_ps(_mm256_mul_ps(minz8, maidz8),
			  _mm256_min_ps(_mm256_mul_ps(maxz8, miidz8),
			  _mm256_mul_ps(maxz8, maidz8)))), mirz8);

			__m256 t1x8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(minx8, miidx8),
			  _mm256_max_ps(_mm256_mul_ps(minx8, maidx8),
			  _mm256_max_ps(_mm256_mul_ps(maxx8, miidx8),
			  _mm256_mul_ps(maxx8, maidx8)))), marx8);

			__m256 t1y8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(miny8, miidy8),
			  _mm256_max_ps(_mm256_mul_ps(miny8, maidy8),
			  _mm256_max_ps(_mm256_mul_ps(maxy8, miidy8),
			  _mm256_mul_ps(maxy8, maidy8)))), mary8);

			__m256 t1z8 = _mm256_add_ps(
			  _mm256_max_ps(_mm256_mul_ps(minz8, miidz8),
			  _mm256_max_ps(_mm256_mul_ps(minz8, maidz8),
			  _mm256_max_ps(_mm256_mul_ps(maxz8, miidz8),
			  _mm256_mul_ps(maxz8, maidz8)))), marz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			// Only use the maximum t of the ray packet for the
			// conservative interval test
			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), maxt8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
			unsigned int hitmask = _mm256_movemask_ps(hitmask8);
			unsigned int hitcnt = __builtin_popcount(hitmask);

			if (hitcnt == 1) {
				// Invert count of leading zeros to get lane
				unsigned int child =
				  31 - __builtin_clz(hitmask);
				// Push ofs to child node
				assert(spos < 64);
				stack[spos] =
				  ((unsigned int *)&n->children8)[child];
				// Push prnt ofs, fpi and lane/child num
				assert(ofs <= 0x1ffffff);
				istack[spos++] =
				  (ofs << 7) | (fpi << 3) | child;
			} else if (hitcnt > 1) {
				// Order, compress, push child node ofs and
				// parent ofs + child num
				__m256i ord8 = _mm256_srli_epi32(n->perm8,
				  shft);

				__m256 hitmaskord8 =
				  _mm256_permutevar8x32_ps(hitmask8, ord8);
				unsigned int hitmaskord =
				  _mm256_movemask_ps(hitmaskord8);

				// Order
				__m256i childrenord8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  ord8);

				assert(ofs <= 0x1ffffff);
				// prnt ofs << 7 | fpi << 3 | child num
				// child num is per lane
				// ofs never points to leaf, so can shift it
				unsigned int ofs_fpi =
				  (ofs << 7) | (fpi << 3);
				__m256i ofs_fpi8 =
				  _mm256_set1_epi32(ofs_fpi);
				__m256i pcnumord8 =
				  _mm256_permutevar8x32_epi32(
				  _mm256_add_epi32(ofs_fpi8, defchildnum8),
				  ord8);

				// Map ordered hit mask to compressed indices
				__m256 cid8 = compr_lut[hitmaskord];

				// Permute to compress
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(childrenord8,
				  cid8);
				__m256i pcnumfin8 =
				  _mm256_permutevar8x32_epi32(pcnumord8, cid8);

				// Unaligned store to stacks
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);
				_mm256_storeu_si256((__m256i *)(istack + spos),
				  pcnumfin8);

				assert(spos < 64 - hitcnt);
				spos += hitcnt; // Account for pushed nodes
			}
		} else {
			// Leaf, check instance blas
			unsigned int instid = ofs & ~NODE_LEAF;
			struct rinst *ri = &insts[instid];
			struct bnode8 *blas = &nodes[ri->triofs << 1];

			// Transform ray into object space of instance
			float inv[16];
			mat4_from3x4(inv, ri->globinv);

			__m256 tox8[pcnt], toy8[pcnt], toz8[pcnt];
			__m256 tdx8[pcnt], tdy8[pcnt], tdz8[pcnt];
			for (unsigned char j = 0; j < pcnt; j++) {
				mulpos_m256(&tox8[j], &toy8[j], &toz8[j],
				  ox8[j], oy8[j], oz8[j], inv);
				muldir_m256(&tdx8[j], &tdy8[j], &tdz8[j],
				  dx8[j], dy8[j], dz8[j], inv);
			}

			intersect_pckt_blas(t8, u8, v8, id8,
			  tox8, toy8, toz8, tdx8, tdy8, tdz8,
			  pcnt, blas, instid);

			// t8 dists could be new, so update our max dist
			// to get proper interval ray culling
			maxt = -FLT_MAX;
			for (unsigned char j = 0; j < pcnt; j++)
				maxt = max(maxt, max8(t8[j]));
			maxt8 = _mm256_set1_ps(maxt);
		}

		// Check actual rays of packets against popped nodes
		while (true) {
			unsigned int ppc; // Parent ofs, fpi, child num
			if (spos > 0) {
				ppc = istack[--spos];
				ofs = stack[spos];
			} else
				return;

			// Fetch node at ofs
			struct bnode8 *p = (struct bnode8 *)(ptr + (ppc >> 7));

			// Intersect packet rays at once with child's aabb
			__m256i child = _mm256_set1_epi32(ppc & 7);
			__m256 minx8 =
			  _mm256_permutevar8x32_ps(p->minx8, child);
			__m256 miny8 =
			  _mm256_permutevar8x32_ps(p->miny8, child);
			__m256 minz8 =
			  _mm256_permutevar8x32_ps(p->minz8, child);
			__m256 maxx8 =
			  _mm256_permutevar8x32_ps(p->maxx8, child);
			__m256 maxy8 =
			  _mm256_permutevar8x32_ps(p->maxy8, child);
			__m256 maxz8 =
			  _mm256_permutevar8x32_ps(p->maxz8, child);

			// Intersect all packets, starting with fpi from stack
			fpi = (ppc >> 3) & 0xf;
			unsigned char i = fpi;
			do {

				__m256 t0x8 =
				  _mm256_fmsub_ps(dx ? minx8 : maxx8,
				  idx8[i], rx8[i]);
				__m256 t0y8 =
				  _mm256_fmsub_ps(dy ? miny8 : maxy8,
				  idy8[i], ry8[i]);
				__m256 t0z8 =
				  _mm256_fmsub_ps(dz ? minz8 : maxz8,
				  idz8[i], rz8[i]);

				__m256 t1x8 =
				  _mm256_fmsub_ps(dx ? maxx8 : minx8,
				  idx8[i], rx8[i]);
				__m256 t1y8 =
				  _mm256_fmsub_ps(dy ? maxy8 : miny8,
				  idy8[i], ry8[i]);
				__m256 t1z8 =
				  _mm256_fmsub_ps(dz ? maxz8 : minz8,
				  idz8[i], rz8[i]);

				__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
				  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

				__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
				  _mm256_min_ps(t1x8, t1y8), t1z8), t8[i]);

				__m256 hit8 = _mm256_cmp_ps(tmin8, tmax8,
				  _CMP_LE_OQ);
				if (_mm256_movemask_ps(hit8) > 0) {
					fpi = i;
					goto restart;
				}

				// Next packet index
				i = (i + 1) % pcnt;
			} while (i != fpi);
		}
	}
}

bool intersect_any_tlas(float tfar, struct vec3 ori, struct vec3 dir,
                        struct bnode8 *nodes, struct rinst *insts,
                        unsigned int tlasofs)
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

	bool dx = dir.x >= 0.0f;
	bool dy = dir.y >= 0.0f;
	bool dz = dir.z >= 0.0f;

	while (true) {
		while ((ofs & NODE_LEAF) == 0) {
			struct bnode8 *n = (struct bnode8 *)(ptr + ofs);

			// Slab test with fused mul sub, swap per ray dir
			__m256 t0x8 = _mm256_fmsub_ps(dx ? n->minx8 : n->maxx8,
			  idx8, rx8);
			__m256 t0y8 = _mm256_fmsub_ps(dy ? n->miny8 : n->maxy8,
			  idy8, ry8);
			__m256 t0z8 = _mm256_fmsub_ps(dz ? n->minz8 : n->maxz8,
			  idz8, rz8);

			__m256 t1x8 = _mm256_fmsub_ps(dx ? n->maxx8 : n->minx8,
			  idx8, rx8);
			__m256 t1y8 = _mm256_fmsub_ps(dy ? n->maxy8 : n->miny8,
			  idy8, ry8);
			__m256 t1z8 = _mm256_fmsub_ps(dz ? n->maxz8 : n->minz8,
			  idz8, rz8);

			__m256 tmin8 = _mm256_max_ps(_mm256_max_ps(
			  _mm256_max_ps(t0x8, t0y8), t0z8), zero8);

			__m256 tmax8 = _mm256_min_ps(_mm256_min_ps(
			  _mm256_min_ps(t1x8, t1y8), t1z8), t8);

			// OQ = ordered/not signaling, 0 if any operand is NAN
			__m256 hitmask8 =
			  _mm256_cmp_ps(tmin8, tmax8, _CMP_LE_OQ);
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
				ofs = ((unsigned int *)&n->children8)[
				  // Invert count of leading zeros to get lane
				  31 - __builtin_clz(hitmask)];
				break;
			default:
				; // Avoid compiler warn 'decl after label'

				// More than one hit

				// Compress unordered child nodes via lut
				__m256i childfin8 =
				  _mm256_permutevar8x32_epi32(n->children8,
				  compr_lut[hitmask]);

				// Unaligned store children on stack
				_mm256_storeu_si256((__m256i *)(stack + spos),
				  childfin8);

				assert(spos < 64 - hitcnt + 1);
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

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = aligned_alloc(64, maxmtls * sizeof(*rd->mtls));

	rd->tris = aligned_alloc(64, maxtris * sizeof(*rd->tris));
	rd->nrms = aligned_alloc(64, maxtris * sizeof(*rd->nrms));

	rd->insts = aligned_alloc(64, maxinsts * sizeof(*rd->insts));
	rd->aabbs = aligned_alloc(64, maxinsts * sizeof(*rd->aabbs));

	// Bvh nodes for blas and tlas
	rd->bnodes = aligned_alloc(64, 2 * (maxtris + maxinsts)
	  * sizeof(*rd->bnodes));

	// Start of tlas nodes
	rd->tlasofs = 2 * maxtris;
}

void rend_release(struct rdata *rd)
{
	free(rd->acc);
	free(rd->bnodes);
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
		struct bnode8 *rn = &rd->bnodes[triofs << 1]; // Root node
		if (!((unsigned int *)&rn->children8)[0]) { // Not processed
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
			  &rd->bnodes[triofs << 1], aabbs, imap,
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

	unsigned int ncnt = build_bvh8(&rd->bnodes[rd->tlasofs],
	  rd->aabbs, imap, NULL /* no tris */, rd->instcnt, rmin, rmax);
	dprintf("Node cnt: %d\n", ncnt);
}

void rend_resaccum(struct rdata *rd, unsigned int w, unsigned int h)
{
	free(rd->acc);

	rd->acc = aligned_alloc(64, w * h * sizeof(*rd->acc));
	rd->width = w;
	rd->height = h;

	rend_clraccum(rd);
}

void rend_clraccum(struct rdata *rd)
{
	memset(rd->acc, 0, rd->width * rd->height * sizeof(*rd->acc));
	rd->samples = 0;
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

struct vec3 rand_hemicos(float u0, float u1)
{
	float phi = TWO_PI * u0;
	float su1 = sqrtf(u1);
	return (struct vec3){cosf(phi) * su1, sinf(phi) * su1, sqrtf(1 - u1)};
}

struct vec3 trace1(struct vec3 o, struct vec3 d, struct rdata *rd,
                   unsigned int *rays)
{
	struct hit h = {.t = FLT_MAX};

	intersect_tlas(&h, o, d, rd->bnodes, rd->insts, rd->tlasofs);
	*rays += 1;

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
                   unsigned char depth, unsigned int *seed, unsigned int *rays)
{
	if (depth >= 2)
		return rd->bgcol;

	struct hit h = {.t = FLT_MAX};

	intersect_tlas(&h, o, d, rd->bnodes, rd->insts, rd->tlasofs);
	*rays += 1;

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
	  rd, depth + 1, seed, rays);

	//return vec3_scale(vec3_mul(brdf, irr), cos_theta / pdf);
	//return vec3_mul(brdf, irr);
	return vec3_mul(
	  vec3_scale(vec3_add(nrm, (struct vec3){1.0f, 1.0f, 1.0f}), 0.5f),
	  vec3_mul(brdf, irr));
}

struct vec3 trace3(struct vec3 o, struct vec3 d, struct rdata *rd,
                   unsigned char depth, unsigned int *seed, unsigned int *rays)
{
	struct hit h = {.t = FLT_MAX};
	intersect_tlas(&h, o, d, rd->bnodes, rd->insts, rd->tlasofs);
	*rays += 1;

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
	//struct vec3 nrm = calc_fnrm(&rd->tris[ri->triofs + triid],
	//  invtransp);

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
	if (ndotl > 0.0f) {
		if (!intersect_any_tlas(ldist, vec3_add(pos,
		  vec3_scale(ldir, EPS2)), ldir, rd->bnodes, rd->insts,
		  rd->tlasofs))
			direct = vec3_scale(vec3_mul(brdf, lcol), INV_PI *
			  ndotl * vec3_dot(lnrm, vec3_neg(ldir)) * larea *
			  (1.0f / (ldist * ldist)));
		*rays += 1;
	}
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
		  vec3_scale(dir, EPS2)), dir, rd, depth + 1, seed, rays);

		//indirect = vec3_scale(vec3_mul(brdf, irr),
		//  cos_theta / pdf);
		indirect = vec3_mul(brdf, irr);
	}
#endif

	return vec3_add(direct, indirect);
}

void make_camray8(__m256 *ox8, __m256 *oy8, __m256 *oz8,
                  __m256 *dx8, __m256 *dy8, __m256 *dz8,
                  unsigned int x, unsigned int y,
                  struct rcam *c, struct rngstate8 *rngstate)
{
	// Create random offset for pixel sample
	__m256 u0 = randf8(rngstate);
	__m256 u1 = randf8(rngstate);

	// Init pixel position in packet (assumes 4x2)
	__m256 sx8 = _mm256_add_ps(_mm256_set1_ps((float)x), pcktxofs8);
	__m256 sy8 = _mm256_add_ps(_mm256_set1_ps((float)y), pcktyofs8);

	// Jitter the pixel position, i.e. px = x + u0 - 0.5
	sx8 = _mm256_add_ps(sx8, _mm256_sub_ps(u0, half8));
	sy8 = _mm256_add_ps(sy8, _mm256_sub_ps(u1, half8));

	// 2 * (px / w - 0.5) * c->tanvfov * c->focdist * c->aspect
	// 2 * (py / h - 0.5) * c->tanvfov * c->focdist
	// (2 is contained in fovfdist product)
	sx8 = _mm256_fmsub_ps(sx8, c->rw8, half8);
	sy8 = _mm256_fmsub_ps(sy8, c->rh8, half8);
	sx8 = _mm256_mul_ps(sx8, c->fovfdist8);
	sy8 = _mm256_mul_ps(sy8, c->fovfdist8);
	sx8 = _mm256_mul_ps(sx8, c->aspect8);

	if (c->tanfangle > 0.0f) {
		// Create random offsets
		__m256 u2 = randf8(rngstate);
		__m256 u3 = randf8(rngstate);

		// Sample disk for jitter offsets
		__m256 rad8 = _mm256_sqrt_ps(u2);
		__m256 theta8 = _mm256_mul_ps(twopi8, u3);
		// TODO sincos?
		__m256 rx8 = _mm256_mul_ps(_ZGVdN8v_cosf(theta8), rad8);
		__m256 ry8 = _mm256_mul_ps(_ZGVdN8v_sinf(theta8), rad8);

		// ori = eye + (ri * rx + up * ry) * focrad
		*ox8 = _mm256_fmadd_ps(_mm256_fmadd_ps(c->rix8, rx8,
		  _mm256_mul_ps(c->upx8, ry8)), c->focrad8, c->eyex8);
		*oy8 = _mm256_fmadd_ps(_mm256_fmadd_ps(c->riy8, rx8,
		  _mm256_mul_ps(c->upy8, ry8)), c->focrad8, c->eyey8);
		*oz8 = _mm256_fmadd_ps(_mm256_fmadd_ps(c->riz8, rx8,
		  _mm256_mul_ps(c->upz8, ry8)), c->focrad8, c->eyez8);

		// upsy = up * sy
		__m256 upxsy8 = _mm256_mul_ps(c->upx8, sy8);
		__m256 upysy8 = _mm256_mul_ps(c->upy8, sy8);
		__m256 upzsy8 = _mm256_mul_ps(c->upz8, sy8);

		// riup = ri * sx - upsy
		__m256 riupx8 = _mm256_fmsub_ps(c->rix8, sx8, upxsy8);
		__m256 riupy8 = _mm256_fmsub_ps(c->riy8, sx8, upysy8);
		__m256 riupz8 = _mm256_fmsub_ps(c->riz8, sx8, upzsy8);

		// fwdeye = fwd * focdist + eye
		__m256 fwdeyex8 = _mm256_fmadd_ps(c->fwdx8, c->fdist8,
		  c->eyex8);
		__m256 fwdeyey8 = _mm256_fmadd_ps(c->fwdy8, c->fdist8,
		  c->eyey8);
		__m256 fwdeyez8 = _mm256_fmadd_ps(c->fwdz8, c->fdist8,
		  c->eyez8);

		// td = fwdeye + riup
		__m256 tdx8 = _mm256_add_ps(fwdeyex8, riupx8);
		__m256 tdy8 = _mm256_add_ps(fwdeyey8, riupy8);
		__m256 tdz8 = _mm256_add_ps(fwdeyez8, riupz8);

		// td = td - ori
		tdx8 = _mm256_sub_ps(tdx8, *ox8);
		tdy8 = _mm256_sub_ps(tdy8, *oy8);
		tdz8 = _mm256_sub_ps(tdz8, *oz8);

		// dir = normalize(td)
		__m256 rlen8 = _mm256_fmadd_ps(tdx8, tdx8, _mm256_fmadd_ps(
		  tdy8, tdy8, _mm256_mul_ps(tdz8, tdz8)));
		rlen8 = rsqrt8_(rlen8, three8, half8);
		*dx8 = _mm256_mul_ps(tdx8, rlen8);
		*dy8 = _mm256_mul_ps(tdy8, rlen8);
		*dz8 = _mm256_mul_ps(tdz8, rlen8);
	} else {
		*ox8 = c->eyex8;
		*oy8 = c->eyey8;
		*oz8 = c->eyez8;

		// upsy = up * sy
		__m256 upxsy8 = _mm256_mul_ps(c->upx8, sy8);
		__m256 upysy8 = _mm256_mul_ps(c->upy8, sy8);
		__m256 upzsy8 = _mm256_mul_ps(c->upz8, sy8);

		// riup = ri * sx - upsy
		__m256 riupx8 = _mm256_fmsub_ps(c->rix8, sx8, upxsy8);
		__m256 riupy8 = _mm256_fmsub_ps(c->riy8, sx8, upysy8);
		__m256 riupz8 = _mm256_fmsub_ps(c->riz8, sx8, upzsy8);

		// td = fwd * focdist + riup
		__m256 tdx8 = _mm256_fmadd_ps(c->fwdx8, c->fdist8, riupx8);
		__m256 tdy8 = _mm256_fmadd_ps(c->fwdy8, c->fdist8, riupy8);
		__m256 tdz8 = _mm256_fmadd_ps(c->fwdz8, c->fdist8, riupz8);

		// dir = normalize(td)
		__m256 rlen8 = _mm256_fmadd_ps(tdx8, tdx8, _mm256_fmadd_ps(
		  tdy8, tdy8, _mm256_mul_ps(tdz8, tdz8)));
		rlen8 = rsqrt8_(rlen8, three8, half8);
		*dx8 = _mm256_mul_ps(tdx8, rlen8);
		*dy8 = _mm256_mul_ps(tdy8, rlen8);
		*dz8 = _mm256_mul_ps(tdz8, rlen8);
	}
}

void trace_pckts(struct vec3 *col,
                 __m256 *ox8, __m256 *oy8, __m256 *oz8,
                 __m256 *dx8, __m256 *dy8, __m256 *dz8,
		 unsigned char pcnt,
                 struct rdata *rd, unsigned int *rays)
{
	__m256 t8[pcnt], u8[pcnt], v8[pcnt];
	__m256i id8[pcnt];

	struct vec3 *c = col;
	for (unsigned char j = 0; j < pcnt; j++) {
		for (unsigned char i = 0; i < PCKT_SZ; i++)
			*c++ = rd->bgcol;
		t8[j] = _mm256_set1_ps(FLT_MAX);
	}

	intersect_pckt_tlas(t8, u8, v8, id8, ox8, oy8, oz8, dx8, dy8, dz8,
	  pcnt, rd->bnodes, rd->insts, rd->tlasofs);

	*rays += pcnt * PCKT_SZ;

// TODO Think about evaluation/shade calc for full package? Bitscan hitmask?
	for (unsigned char k = 0; k < pcnt * PCKT_SZ; k++) {
		float t = ((float *)t8)[k];
		if (t == FLT_MAX)
			continue;

		float u = ((float *)u8)[k];
		float v = ((float *)v8)[k];
		unsigned int id = ((unsigned int *)&id8)[k];

		unsigned int instid = id & INST_ID_MASK;
		unsigned int triid = id >> INST_ID_BITS;
		struct rinst *ri = &rd->insts[instid];
		struct rnrm *rn = &rd->nrms[ri->triofs + triid];
		unsigned int mtlid = rn->mtlid;

		// Inverse transpose, dir mul is 3x4
		float it[16];
		float *rt = ri->globinv;
		for (unsigned int j = 0; j < 4; j++)
			for (unsigned int i = 0; i < 3; i++)
				it[4 * j + i] = rt[4 * i + j];

		struct vec3 nrm = calc_nrm(u, v, rn, it);
		nrm = vec3_scale(vec3_add(nrm, (struct vec3){1, 1, 1}), 0.5f);
		col[k] = vec3_mul(nrm, rd->mtls[mtlid].col);
	}
}

void trace_pckt(struct vec3 *col,
                __m256 ox8, __m256 oy8, __m256 oz8,
                __m256 dx8, __m256 dy8, __m256 dz8,
                struct rdata *rd, unsigned int *rays)
{
	float *ox = (float *)&ox8;
	float *oy = (float *)&oy8;
	float *oz = (float *)&oz8;
	float *dx = (float *)&dx8;
	float *dy = (float *)&dy8;
	float *dz = (float *)&dz8;
	for (unsigned char j = 0; j < PCKT_SZ; j++)
		*col++ = trace1((struct vec3){*ox++, *oy++, *oz++},
		  (struct vec3){*dx++, *dy++, *dz++}, rd, rays);
}

//#define LIVE_SORTED_PCKTS
#define PRE_SORTED_PCKTS
//#define NO_PCKTS

#ifdef LIVE_SORTED_PCKTS

void accum_pckts(struct vec3 *acc, unsigned int *buf, struct vec3 *col,
                 unsigned int pstart, unsigned int pcnt,
                 unsigned int bx, unsigned int by, unsigned int w,
                 float invspp)
{
	for (unsigned int k = pstart; k < pstart + pcnt; k++) {
		unsigned int x = bx + mortx[k] * PCKT_W;
		unsigned int y = by + morty[k] * PCKT_H;
		for (unsigned int j = 0; j < PCKT_H; j++) {
			for (unsigned int i = 0; i < PCKT_W; i++) {
				unsigned int ofs = w * (y + j) + (x + i);
				acc[ofs] = vec3_add(acc[ofs], *col++);
				struct vec3 c = vec3_scale(acc[ofs], invspp);
				buf[ofs] = 0xffu << 24 |
				  ((unsigned int)(255 * c.x) & 0xff) << 16 |
				  ((unsigned int)(255 * c.y) & 0xff) <<  8 |
				  ((unsigned int)(255 * c.z) & 0xff);
			}
		}
	}
}

int rend_render(void *d)
{
	struct rdata *rd = d;

	unsigned int w = rd->width;
	unsigned int blkcntx = w / blkszx;
	unsigned int blkcnty = rd->height / blkszy;
	int          blkcnt = blkcntx * blkcnty;

	unsigned int rays = 0;

	float invspp = 1.0f / (1.0f + rd->samples);

	__m256 ox8[MAX_PCKTS], oy8[MAX_PCKTS], oz8[MAX_PCKTS];
	__m256 dx8[MAX_PCKTS], dy8[MAX_PCKTS], dz8[MAX_PCKTS];
	struct vec3 col[MAX_PCKTS * PCKT_SZ];

	struct rngstate8 rngstate;

	while (true) { // Blocks
		int blk = __atomic_fetch_add(&rd->blknum, 1, __ATOMIC_SEQ_CST);
		if (blk >= blkcnt)
			break;

		rand8_init(&rngstate, (blk + 13) * 1571, rd->samples * 23);

		unsigned int bx = (blk % blkcntx) * blkszx;
		unsigned int by = (blk / blkcntx) * blkszy;
		unsigned int tpcnt = 0; // Total packet cnt

		while (tpcnt < blkpcktcnt) { // Packets per block
			unsigned int pcnt = 0;
			unsigned int dpcnt = 0;
			unsigned char pcmask = 0xff;
			while (pcnt < MAX_PCKTS && tpcnt < blkpcktcnt) {
				make_camray8(
				  &ox8[pcnt], &oy8[pcnt], &oz8[pcnt],
				  &dx8[pcnt], &dy8[pcnt], &dz8[pcnt],
				  bx + mortx[tpcnt] * PCKT_W,
				  by + morty[tpcnt] * PCKT_H,
				  &rd->cam, &rngstate);

				unsigned char cmask = 0;
				if (!iscohval3(&cmask, dx8[pcnt], dy8[pcnt],
				  dz8[pcnt]) ||
				  (pcmask != 0xff && pcmask != cmask)) {
					// Trace last pckt individually
					trace_pckt(col,
					  ox8[pcnt], oy8[pcnt], oz8[pcnt],
					  dx8[pcnt], dy8[pcnt], dz8[pcnt],
					  rd, &rays);
					accum_pckts(rd->acc, rd->buf, col,
					  tpcnt, 1, bx, by, w, invspp);

					// Account for directly handled packet
					dpcnt = 1;
					// Restart packet gathering
					break;
				}

				pcnt++;
				tpcnt++;

				pcmask = cmask;
			}

			if (pcnt > 0) {
				trace_pckts(col, ox8, oy8, oz8, dx8, dy8, dz8,
				  pcnt, rd, &rays);
				accum_pckts(rd->acc, rd->buf, col,
				  tpcnt - pcnt, pcnt, bx, by, w, invspp);
			}

			tpcnt += dpcnt;
		}
	}

	__atomic_fetch_add(&rd->rays, rays, __ATOMIC_SEQ_CST);

	return 0;
}

#endif

#ifdef PRE_SORTED_PCKTS

void accum_pckts(struct vec3 *acc, unsigned int *buf, struct vec3 *col,
                 unsigned int *pmap, unsigned int pcnt,
                 unsigned int bx, unsigned int by, unsigned int w,
                 float invspp)
{
	for (unsigned int k = 0; k < pcnt; k++) {
		unsigned int o = pmap[k];
		unsigned int x = bx + mortx[o] * PCKT_W;
		unsigned int y = by + morty[o] * PCKT_H;
		for (unsigned int j = 0; j < PCKT_H; j++) {
			for (unsigned int i = 0; i < PCKT_W; i++) {
				unsigned int ofs = w * (y + j) + (x + i);
				acc[ofs] = vec3_add(acc[ofs], *col++);
				struct vec3 c = vec3_scale(acc[ofs], invspp);
				buf[ofs] = 0xffu << 24 |
				  ((unsigned int)(255 * c.x) & 0xff) << 16 |
				  ((unsigned int)(255 * c.y) & 0xff) <<  8 |
				  ((unsigned int)(255 * c.z) & 0xff);
			}
		}
	}
}

int rend_render(void *d)
{
	struct rdata *rd = d;

	unsigned int w = rd->width;
	unsigned int blkcntx = w / blkszx;
	unsigned int blkcnty = rd->height / blkszy;
	int          blkcnt = blkcntx * blkcnty;

	unsigned int rays = 0;

	float invspp = 1.0f / (1.0f + rd->samples);

	__m256 ox8[9 * blkpcktcnt], oy8[9 * blkpcktcnt], oz8[9 * blkpcktcnt];
	__m256 dx8[9 * blkpcktcnt], dy8[9 * blkpcktcnt], dz8[9 * blkpcktcnt];
	struct vec3 col[blkpcktcnt * PCKT_SZ];

	struct rngstate8 rngstate;

	while (true) { // Blocks
		int blk = __atomic_fetch_add(&rd->blknum, 1, __ATOMIC_SEQ_CST);
		if (blk >= blkcnt)
			break;

		rand8_init(&rngstate, (blk + 13) * 1571, rd->samples * 23);

		// Block start coordinates
		unsigned int bx = (blk % blkcntx) * blkszx;
		unsigned int by = (blk / blkcntx) * blkszy;

		// Slots 0-7 are packets according to coherence mask
		// Slot 8 are incoherent packets to be traced individually
		unsigned int pmap[9 * blkpcktcnt];
		unsigned int pmapcnt[9] = {0};

		unsigned int pofs = 0; // Potential ofs
		unsigned char pcmask = 0; // Previous coherence mask

		for (unsigned int j = 0; j < blkpcktcnt; j++) {
			make_camray8(
			  &ox8[pofs], &oy8[pofs], &oz8[pofs],
			  &dx8[pofs], &dy8[pofs], &dz8[pofs],
			  bx + mortx[j] * PCKT_W, by + morty[j] * PCKT_H,
			  &rd->cam, &rngstate);

			unsigned char cmask = 0;
			cmask = iscohval3(&cmask, dx8[pofs], dy8[pofs],
			  dz8[pofs]) ? cmask : 8;
			//assert(cmask < 9);

			unsigned int cnt = pmapcnt[cmask]++;
			unsigned int ofs = cmask * blkpcktcnt + cnt;
			pmap[ofs] = j;
			if (pcmask != cmask) { // pofs was wrong, adjust data
				ox8[ofs] = ox8[pofs];
				oy8[ofs] = oy8[pofs];
				oz8[ofs] = oz8[pofs];
				dx8[ofs] = dx8[pofs];
				dy8[ofs] = dy8[pofs];
				dz8[ofs] = dz8[pofs];
			}

			pcmask = cmask;

			// Potential ofs for next slot based on current cmask
			pofs = cmask * blkpcktcnt + cnt + 1;
		}

		// Trace coherent packets
		for (unsigned char j = 0; j < 8; j++) {
			unsigned int cnt = pmapcnt[j];
			unsigned int ofs = j * blkpcktcnt;
			unsigned int i = 0;
			while (i < cnt) {
				unsigned char step = min(MAX_PCKTS, cnt - i);
				trace_pckts(col,
				  &ox8[ofs], &oy8[ofs], &oz8[ofs],
				  &dx8[ofs], &dy8[ofs], &dz8[ofs],
				  step, rd, &rays);

				accum_pckts(rd->acc, rd->buf, col,
				  &pmap[ofs], step, bx, by, w, invspp);

				i += step;
				ofs += step;
			}
		}

		// Trace incoherent packets with individual rays
		unsigned int cnt = pmapcnt[8];
		unsigned int ofs = 8 * blkpcktcnt;
		for (unsigned int j = ofs; j < ofs + cnt; j++) {
			trace_pckt(col, ox8[j], oy8[j], oz8[j],
			  dx8[j], dy8[j], dz8[j], rd, &rays);
			accum_pckts(rd->acc, rd->buf, col, &pmap[j], 1,
			  bx, by, w, invspp);
		}
	}

	__atomic_fetch_add(&rd->rays, rays, __ATOMIC_SEQ_CST);

	return 0;
}

#endif

#ifdef NO_PCKTS

// TODO Use normal trace w/o packets but with Z-curve

#endif

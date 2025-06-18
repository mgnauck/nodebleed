#ifndef REND_H
#define REND_H

#include <immintrin.h>
#include "vec3.h"

struct b2node { // Bvh node, 2-wide, 64 bytes
	struct vec3   lmin;
	unsigned int  l;
	struct vec3   lmax;
	unsigned int  start; // Start index of tri or inst
	struct vec3   rmin;
	unsigned int  r;
	struct vec3   rmax;
	unsigned int  cnt; // Tri or inst cnt
};

#define MBVH_CHILD_CNT  8
struct bmnode { // Mbvh node with M child nodes, used to build b8node
	struct vec3   min;
	unsigned int  start; // Start index of tri or inst
	struct vec3   max;
	unsigned int  cnt; // Tri or inst cnt
	unsigned int  children[MBVH_CHILD_CNT]; // Child node ids
	unsigned int  childcnt;
};

// Node flags
#define NODE_LEAF   0x40000000u
#define NODE_EMPTY  0x80000000u

#define NODEID_MASK  0x3fffffffu
#define TRIID_MASK   0x0fffffffu

// Fuetterling et al., Accelerated Single Ray Tracing for Wide Vector Units
// 8 children, max 4 tris per leaf
struct b8node { // Bvh node, 8-wide, 256 bytes
	__m256       minx; // Aabbs of 8 child nodes
	__m256        maxx;
	__m256        miny;
	__m256        maxy;
	__m256        minz;
	__m256        maxz;
	// Interior node: node flags << 30 | child node id
	// Leaf node: node flags << 30 | (tri cnt - 1) << 28 | tri start
	__m256i       children;
	// Ordered traversal permutation
	// 8 children * 8 quadrants * 3 bit
	__m256i         perm;
};

struct aabb { // 32 bytes
	struct vec3  min;
	float        pad0;
	struct vec3  max;
	float        pad1;
};

struct rmtl { // 32 bytes
	struct vec3   col;
	float         metallic;
	float         roughness;
	float         ior;
	unsigned int  flags;
	unsigned int  pad0;
};

struct rtri { // 48 bytes
	struct vec3  v0;
	float        pad0;
	struct vec3  v1;
	float        pad1;
	struct vec3  v2;
	float        pad2;
};

struct rnrm { // 48 bytes
	struct vec3   n0;
	unsigned int  mtlid;
	struct vec3   n1;
	unsigned int  pad0;
	struct vec3   n2;
	unsigned int  pad1;
};

struct rinst { // 64 bytes
	float         globinv[12]; // Inverse transform 3x4
	unsigned int  flags; // Disabled, emissive, no shadow etc.
	unsigned int  triofs;
	unsigned int  tricnt;
	unsigned int  pad0;
};

struct rcam {
	struct vec3  eye;
	float        vfov;
	struct vec3  ri;
	float        focdist;
	struct vec3  up;
	float        focangle;
};

struct rview {
	struct vec3   dx; // Pixel delta x
	unsigned int  w;
	struct vec3   dy; // Pixel delta y
	unsigned int  h;
	struct vec3   tl; // Pixel top left
};

struct rdata {
	struct rmtl   *mtls;

	struct rtri   *tris; // Tris of all meshes
	struct rnrm   *nrms;

	unsigned int  instcnt;
	struct rinst  *insts;
	struct aabb   *aabbs; // World space instance aabbs

	unsigned int  *imap; // Indices mapping tris/insts
	struct b2node *nodes; // Bvh nodes all blas and tlas
	unsigned int  tlasofs;

/// TEMP TEMP
	struct bmnode *bmnodes;
	struct b8node *b8nodes;
///

	unsigned int  bvhcnt; // Total number of bvhs (blas + tlas)
	unsigned int  *nodecnts; // Node cnt per blas and tlas

	struct rcam   cam;
	struct rview  view;

	unsigned int  blksz; // Size of a block being rendered
	int           blknum; // Block number, accessed atomically

	struct vec3   bgcol;

	struct vec3   *acc; // Accumulator
	unsigned int  *buf; // Color buffer

	unsigned int  samplecnt;
};

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts);
void  rend_release(struct rdata *rd);

void  rend_prepstatic(struct rdata *rd);
void  rend_prepdynamic(struct rdata *rd);
int   rend_render(void *rd); // Expected argument is struct rdata *

#endif

#ifndef REND_H
#define REND_H

#include <immintrin.h>
#include "vec3.h"

// For bmnode and b8node bvhs
#define BRANCH_MAX     8
#define BLAS_LEAF_MAX  4
#define TLAS_LEAF_MAX  1

// Node flags
#define NODE_LEAF   0x80000000u // Bit 31 set indicates leaf node

// Wald et al, 2008, Getting Rid of Packets
// Converted from bnode bvh or build directly by top down splitting
struct bmnode { // Multi branching bvh with M child nodes, used to build b8node
	struct vec3   min;
	unsigned int  start; // Start index of tri or inst
	struct vec3   max;
	unsigned int  cnt; // Tri or inst cnt
	unsigned int  children[BRANCH_MAX]; // Child node ids
	unsigned int  childcnt;
};

// Fuetterling et al., Accelerated Single Ray Tracing for Wide Vector Units
// 8 children, max 4 tris per leaf
// Converted from bmnode bvh
struct b8node { // Bvh node, 8-wide, 256 bytes
	__m256   minx; // Aabbs of 8 child nodes
	__m256   maxx;
	__m256   miny;
	__m256   maxy;
	__m256   minz;
	__m256   maxz;
	// Leaf node flag << 31 | offset to child node in bytes or leaf data
	// Leaf data: ofs to embedded leaf4 in bytes (blas) or inst id (tlas)
	__m256i  children;
	// Ordered traversal permutation
	// 8 children * 8 quadrants * 3 bit
	__m256i  perm;
};

struct leaf4 { // Leaf data of 4 tris, 192 bytes
	__m128        v0x; // 4x vertex 0
	__m128        v0y;
	__m128        v0z;
	__m128        e0x; // 4x edge v1 - v0
	__m128        e0y;
	__m128        e0z;
	__m128        e1x; // 4x edge v2 - v0
	__m128        e1y;
	__m128        e1z;
	unsigned int  id[4]; // 4x tri id
	unsigned int  pad[8];
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
	struct rmtl    *mtls;

	struct rtri    *tris; // Tris of all meshes
	struct rnrm    *nrms;

	unsigned int   instcnt;
	struct rinst   *insts;
	struct aabb    *aabbs; // World space instance aabbs

	unsigned int   *imap; // Indices mapping tris/insts
	unsigned int   tlasofs;
	struct bmnode  *bmnodes;
	struct b8node  *b8nodes;

	struct rcam    cam;
	struct rview   view;

	unsigned int   blksz; // Size of a block being rendered
	int            blknum; // Block number, accessed atomically

	struct vec3    bgcol;

	struct vec3    *acc; // Accumulator
	unsigned int   *buf; // Color buffer

	unsigned int   samplecnt;
};

void  rend_init_compresslut(void);

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts);
void  rend_release(struct rdata *rd);

void  rend_prepstatic(struct rdata *rd);
void  rend_prepdynamic(struct rdata *rd);
int   rend_render(void *rd); // Expected argument is struct rdata *

#endif

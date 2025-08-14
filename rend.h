#ifndef REND_H
#define REND_H

#include <immintrin.h>
#include "vec3.h"

// SPP taken in one step, currently only 1, 2 or 8 due to pckts
#define SPP 1

#define BRANCH_MAX     8 // Child nodes
#define BLAS_LEAF_MAX  4 // Tris
#define TLAS_LEAF_MAX  1 // Instances

#define NODE_LEAF  0x80000000 // Bit 31 set indicates leaf node

// Fuetterling et al., Accelerated Single Ray Tracing for Wide Vector Units
// 8 children, max 4 tris per leaf
struct bnode8 { // Bvh node, 8-wide, 256 bytes
	__m256   minx8; // Aabbs of 8 child nodes
	__m256   maxx8;
	__m256   miny8;
	__m256   maxy8;
	__m256   minz8;
	__m256   maxz8;
	// Leaf node flag << 31 | offset to child node in bytes or leaf data
	// Leaf data: ofs to embedded leaf4 in bytes (blas) or inst id (tlas)
	__m256i  children8;
	// Ordered traversal permutation
	// 8 children * 8 quadrants * 3 bit
	__m256i  perm8;
};

struct leaf4 { // Leaf data of 4 tris, 192 bytes
	__m128        v0x4; // 4x vertex 0
	__m128        v0y4;
	__m128        v0z4;
	__m128        e0x4; // 4x edge v1 - v0
	__m128        e0y4;
	__m128        e0z4;
	__m128        e1x4; // 4x edge v2 - v0
	__m128        e1y4;
	__m128        e1z4;
	__m128i       id4; // 4x tri id
	unsigned int  tricnt; // For packet traversal
	unsigned int  pad[7];
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
	float        tanvfov; // Half tan vfov in rad
	struct vec3  ri;
	float        focdist; // In rad
	struct vec3  up;
	float        tanfangle; // Halt tan foc angle in rad
	struct vec3  fwd;
	float        aspect;
	__m256       eyex8;
	__m256       eyey8;
	__m256       eyez8;
	__m256       rix8;
	__m256       riy8;
	__m256       riz8;
	__m256       upx8;
	__m256       upy8;
	__m256       upz8;
	__m256       fwdx8;
	__m256       fwdy8;
	__m256       fwdz8;
	__m256       rw8; // 1 / width
	__m256       rh8; // 1 / height
	__m256       aspect8;
	__m256       fdist8; // Focus distance
	__m256       fovfdist8; // 2 * tan(0.5 * vfov * PI / 180) * focdist
	__m256       focangle8; // tan(0.5 * focangle * PI / 180)
	__m256       focrad8; // tan(0.5 * focangle * PI / 180) * focdist
};

struct rdata {
	struct rmtl    *mtls;

	struct rtri    *tris; // Tris of all meshes
	struct rnrm    *nrms;

	unsigned int   instcnt;
	struct rinst   *insts;
	struct aabb    *aabbs; // World space instance aabbs

	struct bnode8  *bnodes; // All blas + tlas
	unsigned int   tlasofs;

	struct rcam    cam;

	struct vec3    bgcol;

	unsigned int   width;
	unsigned int   height;
	struct vec3    *acc; // Accumulator (managed by renderer)
	unsigned int   *buf; // Color buffer (managed by application)

	int            blknum; // Block number, accessed atomically

	unsigned int   rays;
	unsigned int   spp;
	unsigned int   samples; // Total samples taken so far
};

void  rend_staticinit(unsigned int blkszx, unsigned int blkszy);
void  rend_staticrelease(void);

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts, unsigned int spp);
void  rend_release(struct rdata *rd);

void  rend_prepstatic(struct rdata *rd);
void  rend_prepdynamic(struct rdata *rd);

void  rend_resaccum(struct rdata *rd, unsigned int w, unsigned int h);
void  rend_clraccum(struct rdata *rd);

int   rend_render(void *rd); // Expected argument is struct rdata *

#endif

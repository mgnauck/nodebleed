#ifndef REND_H
#define REND_H

#include <stdint.h>
#include "vec3.h"

struct bnode { // bvh node, 32 bytes wide
	struct vec3  min; // Aabb min of this node
	uint32_t     sid; // Start index or left child node id
	struct vec3  max; // Aabb max of this node
	uint32_t     cnt; // Tri or inst cnt
};

struct b2node { // bvh node for gpu rendering, 64 byte wide
	struct vec3  lmin; // Aabb min of left child node
	uint32_t     l; // Left child index
	struct vec3  lmax; // Aabb max of left child node
	uint32_t     r; // Right child index
	struct vec3  rmin; // Aabb min of right child node
	uint32_t     start; // Start index of tri or inst
	struct vec3  rmax; // Aabb max of right child node
	uint32_t     cnt; // Tri or inst cnt
};

struct aabb { // 32 bytes
	struct vec3  min;
	float        pad0;
	struct vec3  max;
	float        pad1;
};

struct rmtl { // 32 bytes
	struct vec3  col;
	float        metallic;
	float        roughness;
	float        ior;
	uint32_t     flags;
	uint32_t     pad0;
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
	struct vec3  n0;
	uint32_t     mtlid;
	struct vec3  n1;
	uint32_t     pad0;
	struct vec3  n2;
	uint32_t     pad1;
};

struct rinst { // 64 bytes
	float     globinv[12]; // Inverse transform 3x4
	uint32_t  flags; // Disabled, emissive, no shadow etc.
	uint32_t  triofs;
	uint32_t  tricnt;
	uint32_t  pad0;
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
	struct bnode  *nodes; // All blas and one tlas
	struct b2node *nodes2; // Nodes for GPU rendering
	unsigned int  tlasofs;

	struct rcam   cam;
	struct rview  view;

	struct vec3   bgcol;
};

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts);
void  rend_release(struct rdata *rd);

void  rend_prepstatic(struct rdata *rd);
void  rend_prepdynamic(struct rdata *rd);
void  rend_render(void *dst, struct rdata *rd);

#endif

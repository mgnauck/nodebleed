#ifndef REND_H
#define REND_H

#include <stdint.h>
#include "vec3.h"

struct bnode { // blas
	struct vec3  min;
	uint32_t     sid; // Start index or node id
	struct vec3  max;
	uint32_t     cnt; // Tri cnt
};

struct rmtl {
	struct vec3  col;
	float        metallic;
	float        roughness;
	float        ior;
	uint32_t     flags;
	uint32_t     pad0;
};

struct rtri {
	struct vec3  v0;
	float        pad0;
	struct vec3  v1;
	float        pad1;
	struct vec3  v2;
	float        pad2;
};

struct rnrm {
	struct vec3  n0;
	uint32_t     mtlid;
	struct vec3  n1;
	uint32_t     pad0;
	struct vec3  n2;
	uint32_t     pad1;
};

struct rinst {
	float     globinv[12]; // Inverse transform 3x4
	uint32_t  id; // TODO Check if instance id is required
	uint32_t  flags; // Invisible, no shadow, etc.
	uint32_t  triofs;
	uint32_t  tricnt;
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
	unsigned int  *imap; // Triangle indices mapping (blas)
	struct rinst  *insts;
	struct aabb   *aabbs; // Instance aabbs 
	unsigned int  instcnt;
	struct bnode  *blas; // One blas per mesh (tri buffer)
	struct rcam   cam;
	struct rview  view;
	struct vec3   bgcol;
};

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts);
void  rend_release(struct rdata *rd);

void  rend_prepstatic(struct rdata *rd);
void  rend_render(void *dst, struct rdata *rd);

#endif

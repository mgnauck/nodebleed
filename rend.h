#ifndef REND_H
#define REND_H

#include <stdint.h>
#include "vec3.h"

struct rmtl {
	struct vec3  col;
	float        metallic;
	float        roughness;
	float        ior;
	uint32_t     flags;
	uint32_t     pad0;
};

struct rcam {
	struct vec3  eye;
	float        vfov;
	struct vec3  right;
	float        focdist;
	struct vec3  up;
	float        focangle;
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
	uint32_t  ofs; // Buffer ofs tri + bvh
	uint32_t  flags; // Invisible, no shadow, etc.
	uint32_t  pad0;
};

struct rdata {
	struct rmtl   *mtls;
	struct rcam   cam;
	struct rtri   *tris;
	struct rnrm   *nrms;
	struct rinst  *insts;
};

void  rend_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
                unsigned int maxinsts);
void  rend_release(struct rdata *rd);

void  rcam_set(struct rcam *c, struct vec3 eye, struct vec3 fwd);

void  rend_render(void *dst, unsigned int w, unsigned int h, struct rdata *rd);

#endif

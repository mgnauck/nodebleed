#ifndef RDATA_H
#define RDATA_H

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
	float     itransform[12];
	uint32_t  id; // Instance id
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

void  rd_init(struct rdata *rd, unsigned int maxmtls,
              unsigned int maxtris, unsigned int maxinsts);
void  rd_release(struct rdata *rd);

#endif

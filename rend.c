#include <stdlib.h>
#include "rend.h"
#include "util.h"

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = malloc(maxmtls * sizeof(*rd->mtls));
	rd->tris = malloc(maxtris * sizeof(*rd->tris));
	rd->nrms = malloc(maxtris * sizeof(*rd->nrms));
	rd->insts = malloc(maxinsts * sizeof(*rd->insts));
}

void rend_release(struct rdata *rd)
{
	free(rd->insts);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
}

void rcam_set(struct rcam *c, struct vec3 eye, struct vec3 fwd)
{
	c->eye = eye;
	fwd = vec3_unit(fwd);

	c->right = vec3_unit(vec3_cross((struct vec3){0.0f, 1.0f, 0.0f}, fwd));
	c->up = vec3_unit(vec3_cross(fwd, c->right));
}

void rend_render(void *dst, unsigned int w, unsigned int h, struct rdata *rd)
{
	uint32_t *buf = dst;
	for (unsigned int j = 0; j < h; j++) {
		unsigned int ofs = w * j;
		for (unsigned int i = 0; i < w; i++) {
			buf[ofs + i] = 0xff << 24 | 0xff << 16 | 0xff << 8 | 0xff;
		}
	}
}

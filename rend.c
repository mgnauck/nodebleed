#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ray.h"
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

struct ray create_prim_ray(struct rcam *c, struct rview *v, float x, float y)
{
	struct vec3 pix = vec3_add(v->pix_topleft,
	  vec3_add(vec3_scale(v->pix_dx, x), vec3_scale(v->pix_dy, y)));

	// TODO Jitter pixel sample dx/y * (rand() - 0.5) for AA
	// TODO Jitter eye for DOF (sample disk with radius depending on foc angle/dist)

	return ray_create(c->eye, vec3_unit(vec3_sub(pix, c->eye)));
}

void rend_render(void *dst, struct rdata *rd)
{
	uint32_t *buf = dst;
	for (unsigned int j = 0; j < rd->view.h; j++) {
		unsigned int ofs = rd->view.w * j;
		for (unsigned int i = 0; i < rd->view.w; i++) {
			struct ray r = create_prim_ray(
			  &rd->cam, &rd->view, (float)i, (float)j);
#if 0
			struct vec3 c = r.dir;
			unsigned int cr = min(255, (unsigned int)(255 * c.x));
			unsigned int cg = min(255, (unsigned int)(255 * c.y));
			unsigned int cb = min(255, (unsigned int)(255 * c.z));
			buf[ofs + i] = 0xff << 24 | cr << 16 | cg << 8 | cb;
#endif
			buf[ofs + i] = 0xff << 24 | 0xff << 16 | 0xff << 8 | 0xff;
		}
	}
}

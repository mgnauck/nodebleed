#include <stdlib.h>
#include "rend.h"

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

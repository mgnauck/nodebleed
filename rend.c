#include <stdlib.h>

#include "rend.h"
#include "scene.h"

struct rdata *rend_init(const struct scene *s)
{
	// TODO Count
	unsigned int trimax = 10, instmax = 10;

	struct rdata *rd = malloc(sizeof(*rd));
	rd->mtls = malloc(s->mtlmax * sizeof(*rd->mtls));
	rd->tris = malloc(trimax * sizeof(*rd->tris));
	rd->nrms = malloc(trimax * sizeof(*rd->nrms));
	rd->insts = malloc(instmax * sizeof(*rd->insts));

	return rd;
}

void rend_release(struct rdata *rd)
{
	free(rd->insts);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
	free(rd);
}

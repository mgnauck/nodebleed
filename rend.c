#include <stdio.h>
#include <stdlib.h>

#include "rend.h"
#include "scene.h"

struct rdata *rend_init(const struct scene *s)
{
	// TODO maybe we need to keep dyn/static versions of tris
	unsigned int trimax = 0;
	for (unsigned int i = 0; i < s->meshcnt; i++)
		trimax += s->meshes[i].icnt / 3;

	unsigned int instmax = 0;
	// TODO count instances (static/dyn)

	struct rdata *rd = malloc(sizeof(*rd));
	rd->mtls = malloc(s->mtlmax * sizeof(*rd->mtls));
	rd->tris = malloc(trimax * sizeof(*rd->tris));
	rd->nrms = malloc(trimax * sizeof(*rd->nrms));
	rd->insts = malloc(instmax * sizeof(*rd->insts));

	printf("trimax: %d\n", trimax);

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

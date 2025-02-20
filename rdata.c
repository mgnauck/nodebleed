#include <stdlib.h>
#include "rdata.h"

void rd_init(struct rdata *rd, unsigned int maxmtls, unsigned int maxtris,
             unsigned int maxinsts)
{
	rd->mtls = malloc(maxmtls * sizeof(*rd->mtls));
	rd->tris = malloc(maxtris * sizeof(*rd->tris));
	rd->nrms = malloc(maxtris * sizeof(*rd->nrms));
	rd->insts = malloc(maxinsts * sizeof(*rd->insts));
}

void rd_release(struct rdata *rd)
{
	free(rd->insts);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
}

#include <math.h>
#include <stdio.h>
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

void rend_render(void *dst, struct rdata *rd)
{
	uint32_t *buf = dst;
	for (unsigned int j = 0; j < rd->view.h; j++) {
		unsigned int ofs = rd->view.w * j;
		for (unsigned int i = 0; i < rd->view.w; i++) {
			buf[ofs + i] = 0xff << 24 | 0xff << 16 | 0xff << 8 | 0xff;
		}
	}
}

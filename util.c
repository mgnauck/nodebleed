#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "util.h"

static char *progname = NULL; // Won't be freed

typedef struct pcg_state_setseq_64 {
	uint64_t  state;
	uint64_t  inc;
} pcg32_random_t;

static pcg32_random_t pcg32_global = {
  0x853c49e6748fea9bULL, 0xda3e39cb94b95bdbULL};

/*void *aligned_alloc(size_t alignment, size_t n)
{
	size_t ofs = alignment - 1 + sizeof(void *);
	void *p = malloc(n + ofs);
	void **a = (void **)(((size_t)p + ofs) & ~(alignment - 1));
	a[-1] = p;
	return a;
}

void aligned_free(void *ptr)
{
	free(((void **)ptr)[-1]);
}*/

// https://www.pcg-random.org
uint32_t pcg32_random_r(pcg32_random_t *rng)
{
	uint64_t oldstate = rng->state;
	rng->state = oldstate * 6364136223846793005ULL + rng->inc;
	uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
	uint32_t rot = oldstate >> 59u;
	return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

void pcg32_srandom_r(pcg32_random_t* rng, uint64_t initstate,
                     uint64_t initseq)
{
	rng->state = 0U;
	rng->inc = (initseq << 1u) | 1u;
	pcg32_random_r(rng);
	rng->state += initstate;
	pcg32_random_r(rng);
}

void pcg_srand(uint64_t seed, uint64_t seq)
{
	pcg32_srandom_r(&pcg32_global, seed, seq);
}

uint32_t pcg_rand(void)
{
	return pcg32_random_r(&pcg32_global);
}

float pcg_randf(void)
{
	return ldexp(pcg_rand(), -32);
	//return pcg_rand() / (double)UINT32_MAX;
}

float pcg_randfrng(float start, float end)
{
	return start + pcg_randf() * (end - start);
}

void setflags(unsigned int *state, unsigned int flags)
{
	*state |= flags;
}

void clrflags(unsigned int *state, unsigned int flags)
{
	*state &= ~flags;
}

bool hasflags(unsigned int state, unsigned int flags)
{
	return (state & flags) == flags;
}

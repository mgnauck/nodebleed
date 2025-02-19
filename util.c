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

static pcg32_random_t pcg32_global = { 0x853c49e6748fea9bULL, 0xda3e39cb94b95bdbULL };

// Kernighan, Pike: The Practive of Programming
void eprintf(const char *fmt, ...)
{
	fflush(stdout);

	if (getprogname() != NULL)
		fprintf(stderr, "%s: ", getprogname());

	va_list args;

	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);

	if (fmt[0] != '\0' && fmt[strlen(fmt) - 1] == ':')
		fprintf(stderr, " %s", strerror(errno));
	fprintf(stderr, "\n");

	exit(2);
}

char *estrdup(const char *s)
{
	char *t = malloc(strlen(s) + 1);
	if (t == NULL)
		eprintf("estrdup(\"%.20s\") failed:", s);
	strcpy(t, s);
	return t;
}

void *emalloc(size_t n)
{
	void *p = malloc(n);
	if (p == NULL)
		eprintf("malloc of %u bytes failed:", n);
	return p;
}

void setprogname(const char *name)
{
	progname = estrdup(name);
}

char *getprogname(void)
{
	return progname;
}

// https://www.pcg-random.org
uint32_t pcg32_random_r(pcg32_random_t *rng)
{
	uint64_t oldstate = rng->state;
	rng->state = oldstate * 6364136223846793005ULL + rng->inc;
	uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
	uint32_t rot = oldstate >> 59u;
	return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

void pcg32_srandom_r(pcg32_random_t* rng, uint64_t initstate, uint64_t initseq)
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

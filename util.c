#include <math.h>

#ifdef NOSTDLIB
#include "platform.h"
#endif
#include "util.h"

typedef struct pcg_state_setseq_64 {
	uint64_t  state;
	uint64_t  inc;
} pcg32_random_t;

static pcg32_random_t pcg32_global = {
  0x853c49e6748fea9bULL, 0xda3e39cb94b95bdbULL};

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

#ifdef NOSTDLIB
// From man futex
void fwait(unsigned int *fut)
{
	while (1) {
		// Check if futex is available (compare to 1)
		unsigned int one = 1; // On false, will become value of *futexp
		if (__atomic_compare_exchange_n(fut, &one, 0, false,
		  __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST))
			break;
		// Futex unavailable, so wait
		int s = futex(fut, FUTEX_WAIT, 0, NULL, NULL, 0);
		if (s == -1 && s != -11 /* -EAGAIN */)
			exit(1);
	}
}

void fpost(unsigned int *fut)
{
	unsigned int zero = 0;
	if (__atomic_compare_exchange_n(fut, &zero, 1, false,
	  __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
		// Wake any futex waiters
		int s = futex(fut, FUTEX_WAKE, 1, NULL, NULL, 0);
		if (s < 0 && s >= -4095)
			exit(1);
	}
}

void create_stacks(void **stacks, unsigned int stackscnt, unsigned int stacksz)
{
	for (unsigned int i = 0; i < stackscnt; i++)
		stacks[i] = create_stack(stacksz);
}

void release_stacks(void **stacks, unsigned int stackscnt, unsigned int stacksz)
{
	for (unsigned int i = 0; i < stackscnt; i++)
		release_stack(stacks[i], stacksz);
}

void create_threads(void (*fn)(void *), void *data, void **stacks,
  unsigned int thrdcnt)
{
	for (unsigned int i = 0; i < thrdcnt; i++)
		create_thread(fn, data, stacks[i]);
}
#endif

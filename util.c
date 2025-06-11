#include "util.h"

unsigned int xorshift32(unsigned int *seed)
{
	unsigned int x = *seed;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return *seed = x;
}

float randf(unsigned int *seed)
{
	return xorshift32(seed) * 2.3283064365387e-10f;
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

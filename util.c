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
	union {
		unsigned int  u32;
		float         f32;
	// Keep mantissa (between 1.0 and 2.0) and null exponent bits
	} u = {.u32 = xorshift32(seed) >> 9 | 127 << 23};

	return u.f32 - 1.0f;
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

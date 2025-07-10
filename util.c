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

bool anyflags(unsigned int state, unsigned int flags)
{
	return (state & flags);
}

// https://stackoverflow.com/questions/13219146/how-to-sum-m256-horizontally
float min8(__m256 x8)
{
	// hiquad = x7, x6, x5, x4
	__m128 hiquad = _mm256_extractf128_ps(x8, 1);
	// loquad = x3, x2, x1, x0
	__m128 loquad = _mm256_castps256_ps128(x8);
	// minquad = x3|x7, x2|x6, x1|x5, x0|x4
	__m128 minquad = _mm_min_ps(loquad, hiquad);
	// lodual = -, -, x1|x5, x0|x4
	__m128 lodual = minquad;
	// hidual = -, -, x3|x7, x2|x6
	__m128 hidual = _mm_movehl_ps(minquad, minquad);
	// mindual = -, -, x1|x3|x5|x7, x0|x2|x4|x6
	__m128 mindual = _mm_min_ps(lodual, hidual);
	// lo = -, -, -, x0|x2|x4|x6
	__m128 lo = mindual;
	// hi = -, -, -, x1|x3|x5|x7
	__m128 hi = _mm_shuffle_ps(mindual, mindual, 0x1);
	// mi = -, -, -, x0|x1|x2|x3|x4|x5|x6|x7
	__m128 mi = _mm_min_ss(lo, hi);

	return _mm_cvtss_f32(mi);
}

float max8(__m256 x8)
{
	// hiquad = x7, x6, x5, x4
	__m128 hiquad = _mm256_extractf128_ps(x8, 1);
	// loquad = x3, x2, x1, x0
	__m128 loquad = _mm256_castps256_ps128(x8);
	// maxquad = x3|x7, x2|x6, x1|x5, x0|x4
	__m128 maxquad = _mm_max_ps(loquad, hiquad);
	// lodual = -, -, x1|x5, x0|x4
	__m128 lodual = maxquad;
	// hidual = -, -, x3|x7, x2|x6
	__m128 hidual = _mm_movehl_ps(maxquad, maxquad);
	// maxdual = -, -, x1|x3|x5|x7, x0|x2|x4|x6
	__m128 maxdual = _mm_max_ps(lodual, hidual);
	// lo = -, -, -, x0|x2|x4|x6
	__m128 lo = maxdual;
	// hi = -, -, -, x1|x3|x5|x7
	__m128 hi = _mm_shuffle_ps(maxdual, maxdual, 0x1);
	// ma = -, -, -, x0|x1|x2|x3|x4|x5|x6|x7
	__m128 ma = _mm_max_ss(lo, hi);

	return _mm_cvtss_f32(ma);
}

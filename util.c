#include "util.h"

unsigned int xorshift32(unsigned int *s)
{
	unsigned int x = *s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return *s = x;
}

unsigned int randu(unsigned int *s)
{
	return xorshift32(s);
}

float randf(unsigned int *s)
{
	union {
		unsigned int  u;
		float         f;
	// Keep mantissa (between 1.0 and 2.0) and null exponent bits
	} val = {.u = xorshift32(s) >> 9 | 127 << 23};
	// Substract 1 to be in 0-1 range
	return val.f - 1.0f;
}

// Vigna, Further scramblings of Marsaglia’s xorshift generators
// https://vigna.di.unimi.it/ftp/papers/xorshiftplus.pdf
// https://xoroshiro.di.unimi.it/xorshift128plus.c
unsigned long long xorshift128plus(unsigned long long *state0,
                                   unsigned long long *state1)
{
	unsigned long long s1 = *state0;
	unsigned long long s0 = *state1;
	unsigned long long res = s0 + s1;
	*state0 = s0;
	s1 ^= s1 << 23;
	*state1 = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5);
	return res;
}

void jump(unsigned long long *state0, unsigned long long *state1,
          unsigned long long instate0, unsigned long long instate1)
{
	static unsigned long long JUMP[] = {
	  0x8a5cd789635d2dff, 0x121fd2155c472f96};
	unsigned long long s0 = 0;
	unsigned long long s1 = 0;
	for (unsigned int i = 0; i < sizeof(JUMP) / sizeof(*JUMP); i++) {
		for (unsigned int b = 0; b < 64; b++) {
			if (JUMP[i] & 1ull << b) {
				s0 ^= instate0;
				s1 ^= instate1;
			}
			xorshift128plus(&instate0, &instate1);
		}
	}
	*state0 = s0;
	*state1 = s1;
}

void rand8_init(struct rngstate8 *s,
                unsigned long long instate0, unsigned long long instate1)
{
	_Alignas(64) unsigned long long s0[4];
	_Alignas(64) unsigned long long s1[4];
	s0[0] = instate0;
	s1[0] = instate1;
	jump(s0 + 1, s1 + 1, *s0, *s1);
	jump(s0 + 2, s1 + 2, *(s0 + 1), *(s1 + 1));
	jump(s0 + 3, s1 + 3, *(s0 + 2), *(s1 + 2));
	s->state0 = _mm256_load_si256((__m256i *)s0);
	s->state1 = _mm256_load_si256((__m256i *)s1);
}

__m256i xorshift128plus8(struct rngstate8 *s)
{
	__m256i s1 = s->state0;
	__m256i s0 = s->state1;
	__m256i res = _mm256_add_epi64(s0, s1);
	s->state0 = s0;
	s1 = _mm256_xor_si256(s1, _mm256_slli_epi64(s1, 23));
	__m256i s1new = _mm256_xor_si256(_mm256_xor_si256(
	  _mm256_xor_si256(s1, s0), _mm256_srli_epi64(s1, 18)),
	  _mm256_srli_epi64(s0, 5));
	s->state1 = s1new;
	return res;
}

__m256i randu8(struct rngstate8 *s)
{
	return xorshift128plus8(s);
}

void randuv8(struct rngstate8 *s, unsigned int v[8])
{
	_mm256_store_si256((__m256i *)v, xorshift128plus8(s));
}

__m256 randf8(struct rngstate8 *s)
{
	__m256i e = _mm256_set1_epi32(127 << 23);
	__m256i v = xorshift128plus8(s);
	// Keep mantissa (between 1.0 and 2.0) and null exponent bits
	__m256i f = _mm256_or_si256(_mm256_srli_epi32(v, 9), e);
	// Substract 1 to be in 0-1 range
	return _mm256_sub_ps(_mm256_castsi256_ps(f),
	  _mm256_castsi256_ps(e));
}

void randfv8(struct rngstate8 *s, float v[8])
{
	_mm256_store_ps(v, randf8(s));
}

// http://www-graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
// Interleave lower 16 bits with zeros
unsigned int intleave(unsigned int v)
{
	v &= 0x0000ffff;
	v = (v ^ (v << 8)) & 0x00ff00ff;
	v = (v ^ (v << 4)) & 0x0f0f0f0f;
	v = (v ^ (v << 2)) & 0x33333333;
	return (v ^ (v << 1)) & 0x55555555;
}

unsigned int mortenc(unsigned int x, unsigned int y)
{
	return intleave(y) << 1 | intleave(x);
}

unsigned int mortdecx(unsigned int v)
{
	v &= 0x55555555;
	v = (v ^ (v >> 1)) & 0x33333333;
	v = (v ^ (v >> 2)) & 0x0f0f0f0f;
	v = (v ^ (v >> 4)) & 0x00ff00ff;
	return (v ^ (v >> 8)) & 0x0000ffff;
}

unsigned int mortdecy(unsigned int v)
{
	return mortdecx(v >> 1);
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
	// hiquad = 7 6 5 4
	__m128 hiquad = _mm256_extractf128_ps(x8, 1);
	// loquad = 3 2 1 0
	__m128 loquad = _mm256_castps256_ps128(x8);
	// minquad = 3|7 2|6 1|5 0|4
	__m128 minquad = _mm_min_ps(loquad, hiquad);
	// lodual = - - 1|5 0|4
	__m128 lodual = minquad;
	// hidual = - - 3|7 2|6
	__m128 hidual = _mm_movehl_ps(minquad, minquad);
	// mindual = - - 1|3|5|7 0|2|4|6
	__m128 mindual = _mm_min_ps(lodual, hidual);
	// lo = - - - 0|2|4|6
	__m128 lo = mindual;
	// hi = - - - 1|3|5|7
	__m128 hi = _mm_shuffle_ps(mindual, mindual, 0x1);
	// mi = - - - 0|1|2|3|4|5|6|7
	__m128 mi = _mm_min_ss(lo, hi);

	return _mm_cvtss_f32(mi);
}

float max8(__m256 x8)
{
	// hiquad = 7 6 5 4
	__m128 hiquad = _mm256_extractf128_ps(x8, 1);
	// loquad = 3 2 1 0
	__m128 loquad = _mm256_castps256_ps128(x8);
	// maxquad = 3|7 2|6 1|5 0|4
	__m128 maxquad = _mm_max_ps(loquad, hiquad);
	// lodual = - - 1|5 0|4
	__m128 lodual = maxquad;
	// hidual = - - 3|7 2|6
	__m128 hidual = _mm_movehl_ps(maxquad, maxquad);
	// maxdual = - - 1|3|5|7 0|2|4|6
	__m128 maxdual = _mm_max_ps(lodual, hidual);
	// lo = - - - 0|2|4|6
	__m128 lo = maxdual;
	// hi = - - - 1|3|5|7
	__m128 hi = _mm_shuffle_ps(maxdual, maxdual, 0x1);
	// ma = - - - 0|1|2|3|4|5|6|7
	__m128 ma = _mm_max_ss(lo, hi);

	return _mm_cvtss_f32(ma);
}

// https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision
__m128 rcp4(__m128 a4)
{
	//return _mm_div_ps(one4, a4);

	// Rcp with 1 NR iteration
	__m128 r4 = _mm_rcp_ps(a4);
	__m128 m4 = _mm_mul_ps(a4, _mm_mul_ps(r4, r4));
	return _mm_sub_ps(_mm_add_ps(r4, r4), m4);
}

__m256 rcp8(__m256 a8)
{
	//return _mm256_div_ps(one8, a8);

	// Rcp with 1 NR iteration
	__m256 r8 = _mm256_rcp_ps(a8);
	__m256 m8 = _mm256_mul_ps(a8, _mm256_mul_ps(r8, r8));
	return _mm256_sub_ps(_mm256_add_ps(r8, r8), m8);
}

__m256 rsqrt8(__m256 a8)
{
	__m256 three8 = _mm256_set1_ps(3.0f);
	__m256 half8 = _mm256_set1_ps(0.5f);
	__m256 r8 = _mm256_rsqrt_ps(a8);
	__m256 m8 = _mm256_mul_ps(_mm256_mul_ps(a8, r8), r8);
	return _mm256_mul_ps(_mm256_mul_ps(half8, r8),
	  _mm256_sub_ps(three8, m8));
}

__m256 rsqrt8_(__m256 a8, __m256 three8, __m256 half8)
{
	__m256 r8 = _mm256_rsqrt_ps(a8);
	__m256 m8 = _mm256_mul_ps(_mm256_mul_ps(a8, r8), r8);
	return _mm256_mul_ps(_mm256_mul_ps(half8, r8),
	  _mm256_sub_ps(three8, m8));
}

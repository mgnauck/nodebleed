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

// TODO Can be optimized for 8 bit or use LUT
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

__m256 bcmin8(__m256 a8)
{
	// a8 = 7654 3210

	// p0 = 5476 1032
	__m256 p0 = _mm256_permute_ps(a8, 0x4e); // 0100 1110

	// m0 = 5454 1010
	__m256 m0 = _mm256_min_ps(a8, p0);

	// p1 = 4545 0101
	__m256 p1 = _mm256_permute_ps(m0, 0xb1); // 1011 0001

	// m1 = 4444 0000
	__m256 m1 = _mm256_min_ps(m0, p1);

	// p2 = 0000 4444
	__m256 p2 = _mm256_permute2f128_ps(m1, m1, 0x01);

	return _mm256_min_ps(m1, p2);
}

__m256 bcmax8(__m256 a8)
{
	// a8 = 7654 3210

	// p0 = 5476 1032
	__m256 p0 = _mm256_permute_ps(a8, 0x4e); // 0100 1110

	// m0 = 7676 3232
	__m256 m0 = _mm256_max_ps(a8, p0);

	// p1 = 6767 2323
	__m256 p1 = _mm256_permute_ps(m0, 0xb1); // 1011 0001

	// m1 = 7777 3333
	__m256 m1 = _mm256_max_ps(m0, p1);

	// p2 = 3333 7777
	__m256 p2 = _mm256_permute2f128_ps(m1, m1, 0x01);

	return _mm256_max_ps(m1, p2);
}

/*
__m256 bcmin8(__m256 a8)
{
	// a8 = 7654 3210

	// s0 = 6547 2103
	__m256 s0 = _mm256_shuffle_ps(a8, a8, _MM_SHUFFLE(2, 1, 0, 3));

	// m0 = 6544 2100
	__m256 m0 = _mm256_min_ps(a8, s0);

	// s1 = 4465 0021
	__m256 s1 = _mm256_shuffle_ps(m0, m0, _MM_SHUFFLE(1, 0, 3, 2));

	// m1 = 4444 0000
	__m256 m1 = _mm256_min_ps(m0, s1);

	// swp = 0000 4444
	__m256 swp = _mm256_permute2f128_ps(m1, m1, 0x1);

	return _mm256_min_ps(m1, swp);
}

__m256 bcmax8(__m256 a8)
{
	// a8 = 7654 3210

	// s0 = 6547 2103
	__m256 s0 = _mm256_shuffle_ps(a8, a8, _MM_SHUFFLE(2, 1, 0, 3));

	// m0 = 7657 3213
	__m256 m0 = _mm256_max_ps(a8, s0);

	// s1 = 5776 1332
	__m256 s1 = _mm256_shuffle_ps(m0, m0, _MM_SHUFFLE(1, 0, 3, 2));

	// m1 = 7777 3333
	__m256 m1 = _mm256_max_ps(m0, s1);

	// swp = 3333 7777
	__m256 swp = _mm256_permute2f128_ps(m1, m1, 0x1);

	return _mm256_max_ps(m1, swp);
}
*/

// https://stackoverflow.com/questions/31555260/fast-vectorized-rsqrt-and-reciprocal-with-sse-avx-depending-on-precision
__m128 rcp4(__m128 a4)
{
#ifndef NORCP
	__m128 r4 = _mm_rcp_ps(a4);
	__m128 m4 = _mm_mul_ps(a4, _mm_mul_ps(r4, r4));
	return _mm_sub_ps(_mm_add_ps(r4, r4), m4);
#else
	return _mm_div_ps(one4, a4);
#endif
}

__m256 rcp8(__m256 a8)
{
#ifndef NORCP
	__m256 r8 = _mm256_rcp_ps(a8);
	__m256 m8 = _mm256_mul_ps(a8, _mm256_mul_ps(r8, r8));
	return _mm256_sub_ps(_mm256_add_ps(r8, r8), m8);
#else
	return _mm256_div_ps(one8, a8);
#endif
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

__m256 bcl4to8(__m128 a4, unsigned char lane)
{
	__m256 r8 = _mm256_zextps128_ps256(a4);
	return _mm256_permutevar8x32_ps(r8, _mm256_set1_epi32(lane));
}

__m256i bcl4ito8i(__m128i a4, unsigned char lane)
{
	__m256i r8 = _mm256_zextsi128_si256(a4);
	return _mm256_permutevar8x32_epi32(r8, _mm256_set1_epi32(lane));
}

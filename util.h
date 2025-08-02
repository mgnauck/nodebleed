#ifndef UTIL_H
#define UTIL_H

#include <immintrin.h>
#include <stdbool.h>

#define EPS     1e-5
#define EPS2    1e-3
#define PI      3.14159265
#define TWO_PI  6.28318531
#define INV_PI  (1.0 / PI)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

unsigned int        randu(unsigned int *s); // xorshift32
float               randf(unsigned int *s);

struct rngstate8 { // m256 xorshift128+
	__m256i state0;
	__m256i state1;
};

void                rand8_init(struct rngstate8 *s, unsigned long long instate0,
                               unsigned long long instate1);
__m256i             randu8(struct rngstate8 *s);
void                randuv8(struct rngstate8 *s, unsigned int v[8]);
__m256              randf8(struct rngstate8 *s);
void                randfv8(struct rngstate8 *s, float v[8]);

// No SVML functions available on clang/gcc
// https://stackoverflow.com/questions/40475140/mathematical-functions-for-simd-registers
__m256              _ZGVdN8v_sinf(__m256 x);
__m256              _ZGVdN8v_cosf(__m256 x);

unsigned int        mortenc(unsigned int x, unsigned int y);
unsigned int        mortdecx(unsigned int v);
unsigned int        mortdecy(unsigned int v);

void                setflags(unsigned int *state, unsigned int flags);
void                clrflags(unsigned int *state, unsigned int flags);
bool                hasflags(unsigned int state, unsigned int flags);
bool                anyflags(unsigned int state, unsigned int flags);

// Broadcasted max m256
static inline __m256 bcmin8(__m256 a8)
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

static inline __m256 bcmax8(__m256 a8)
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

// Broadcast m128 lane to m256
static inline __m256 bcl4to8(__m128 a4, unsigned char lane)
{
	__m256 r8 = _mm256_zextps128_ps256(a4);
	return _mm256_permutevar8x32_ps(r8, _mm256_set1_epi32(lane));
}

static inline __m256i bcl4ito8i(__m128i a4, unsigned char lane)
{
	__m256i r8 = _mm256_zextsi128_si256(a4);
	return _mm256_permutevar8x32_epi32(r8, _mm256_set1_epi32(lane));
}

#endif

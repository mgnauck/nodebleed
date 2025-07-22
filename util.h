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

void                setflags(unsigned int *state, unsigned int flags);
void                clrflags(unsigned int *state, unsigned int flags);
bool                hasflags(unsigned int state, unsigned int flags);
bool                anyflags(unsigned int state, unsigned int flags);

float               min8(__m256 x8);
float               max8(__m256 x8);

// Broadcasted max m256
__m256              bcmax8(__m256 x8);

__m128              rcp4(__m128 a4);
__m256              rcp8(__m256 a8);

__m256              rsqrt8(__m256 a8);
__m256              rsqrt8_(__m256 a8, __m256 three8, __m256 half8);

// Broadcast m128 lane to m256
__m256              bcl4to8(__m128 a4, unsigned char lane);
__m256i             bcl4ito8i(__m128i a4, unsigned char lane);

#endif

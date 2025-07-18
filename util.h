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

float   randf(unsigned int *seed);

void    setflags(unsigned int *state, unsigned int flags);
void    clrflags(unsigned int *state, unsigned int flags);
bool    hasflags(unsigned int state, unsigned int flags);
bool    anyflags(unsigned int state, unsigned int flags);

float   min8(__m256 x8);
float   max8(__m256 x8);

__m256  bcmax8(__m256 x8); // Broadcasted max8

__m128  rcp4(__m128 a4);
__m256  rcp8(__m256 a8);

#endif

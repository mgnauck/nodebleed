#ifndef UTIL_H
#define UTIL_H

#include <stdbool.h>

#define EPS     1e-6
#define PI      3.14159265
#define TWO_PI  6.28318531
#define INV_PI  (1.0 / PI)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define arrsz(a) (sizeof(a) / sizeof((a)[0]))

float  randf(unsigned int *seed);

void   setflags(unsigned int *state, unsigned int flags);
void   clrflags(unsigned int *state, unsigned int flags);
bool   hasflags(unsigned int state, unsigned int flags);

#endif

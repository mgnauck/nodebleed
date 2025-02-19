#ifndef UTIL_H
#define UTIL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EPS     1e-6
#define PI      3.14159265
#define TWO_PI  6.28318531

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define arrsz(a) (sizeof(a)/sizeof((a)[0]))

void      eprintf(const char *fmt, ...);
char      *estrdup(const char *s);
void      *emalloc(size_t n);
void      setprogname(const char *name);
char      *getprogname(void);

void      pcg_srand(uint64_t seed, uint64_t seq);
uint32_t  pcg_rand(void);
float     pcg_randf(void);
float     pcg_randfrng(float start, float end);

void      setflags(unsigned int *state, unsigned int flags);
void      clrflags(unsigned int *state, unsigned int flags);
bool      hasflags(unsigned int state, unsigned int flags);

#endif

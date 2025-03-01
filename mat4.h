#ifndef MAT4_H
#define MAT4_H

#include "vec3.h"

// Standard math (row first) order:
//
// 0 1 2 3       1 0 0 x
// 4 5 6 7  and  0 1 0 y
// 8 9 0 a       0 0 1 z
// b c d e       0 0 0 1
//
// No side effects if src == dst.

void         mat4_cpy(float dst[restrict 16], const float src[16]);

void         mat4_identity(float m[16]);
void         mat4_transpose(float dst[16], const float src[16]);

void         mat4_trans(float dst[16], const struct vec3 v);

void         mat4_rotx(float dst[16], float radians);
void         mat4_roty(float dst[16], float radians);
void         mat4_rotz(float dst[16], float radians);

void         mat4_scale(float dst[16], struct vec3 s);
void         mat4_scaleu(float dst[16], float s);

int          mat4_inv(float dst[16], const float src[16]);

void         mat4_mul(float dst[16], const float a[16], const float b[16]);

struct vec3  mat4_mulpos(const float m[16], const struct vec3 v);
struct vec3  mat4_muldir(const float m[16], const struct vec3 v);

struct vec3  mat4_gettrans(const float m[16]);

void         mat4_fromquat(float dst[16], float x, float y, float z, float w);
void         mat4_from3x4(float dst[restrict 16], float src[12]);

#endif

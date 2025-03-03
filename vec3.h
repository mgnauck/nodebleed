#ifndef VEC3_H
#define VEC3_H

struct vec3 {
	float  x, y, z;
};

struct vec3  vec3_rand(void);
struct vec3  vec3_randrng(float min, float max);

struct vec3  vec3_rand2disk(void);

struct vec3  vec3_add(struct vec3 a, struct vec3 b);
struct vec3  vec3_sub(struct vec3 a, struct vec3 b);
struct vec3  vec3_mul(struct vec3 a, struct vec3 b);

struct vec3  vec3_neg(struct vec3 v);
struct vec3  vec3_scale(struct vec3 v, float s);

struct vec3  vec3_cross(struct vec3 a, struct vec3 b);
struct vec3  vec3_unit(struct vec3 v);

float        vec3_dot(struct vec3 a, struct vec3 b);
float        vec3_len(struct vec3 v);

struct vec3  vec3_min(struct vec3 a, struct vec3 b);
struct vec3  vec3_max(struct vec3 a, struct vec3 b);

float        vec3_minc(struct vec3 v); // Component
float        vec3_maxc(struct vec3 v);

float        vec3_getc(struct vec3 v, unsigned char i);

struct vec3  vec3_abs(struct vec3 v);

struct vec3  vec3_spherical(float theta, float phi);

#endif

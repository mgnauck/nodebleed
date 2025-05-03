#include <math.h>

#include "util.h"
#include "vec3.h"

struct vec3 vec3_randunit(void)
{
	float u = 2.0f * pcg_randf() - 1.0f;
	float theta = 2.0f * PI * pcg_randf();
	float r = sqrtf(1.0f - u * u);
	return (struct vec3){r * cosf(theta), r * sinf(theta), u};
	//return (struct vec3){pcg_randf(), pcg_randf(), pcg_randf()};
}

struct vec3 vec3_randhemi(struct vec3 n)
{
	struct vec3 v = vec3_randunit();
	return vec3_dot(n, v) < 0.0f ? vec3_neg(v) : v;
}

struct vec3 vec3_rand2disk(void)
{
	float r = sqrtf(pcg_randf());
	float theta = 2.0f * PI * pcg_randf();
	return (struct vec3){r * cosf(theta), r * sinf(theta), 0.0f};
}

struct vec3 vec3_add(struct vec3 a, struct vec3 b)
{
	return (struct vec3){a.x + b.x, a.y + b.y, a.z + b.z};
}

struct vec3 vec3_sub(struct vec3 a, struct vec3 b)
{
	return (struct vec3){a.x - b.x, a.y - b.y, a.z - b.z};
}

struct vec3 vec3_mul(struct vec3 a, struct vec3 b)
{
	return (struct vec3){a.x * b.x, a.y * b.y, a.z * b.z};
}

struct vec3 vec3_neg(struct vec3 v)
{
	return (struct vec3){-v.x, -v.y, -v.z};
}

struct vec3 vec3_scale(struct vec3 v, float s)
{
	return (struct vec3){v.x * s, v.y * s, v.z * s};
}

struct vec3 vec3_cross(struct vec3 a, struct vec3 b)
{
	return (struct vec3){
	  a.y * b.z - a.z * b.y,
	  a.z * b.x - a.x * b.z,
	  a.x * b.y - a.y * b.x};
}

struct vec3 vec3_unit(struct vec3 v)
{
	float il = 1.0f / vec3_len(v);
	return (struct vec3){v.x * il, v.y * il, v.z * il};
}

float vec3_dot(struct vec3 a, struct vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vec3_len(struct vec3 v)
{
	return sqrtf(vec3_dot(v, v));
}

struct vec3 vec3_min(struct vec3 a, struct vec3 b)
{
	return (struct vec3){min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)};
}

struct vec3 vec3_max(struct vec3 a, struct vec3 b)
{
	return (struct vec3){max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)};
}

float vec3_minc(struct vec3 v)
{
	return min(min(v.x, v.y), v.z);
}

float vec3_maxc(struct vec3 v)
{
	return max(max(v.x, v.y), v.z);
}

float vec3_getc(struct vec3 v, unsigned char i)
{
	return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

struct vec3 vec3_abs(struct vec3 v)
{
	return (struct vec3){fabsf(v.x), fabsf(v.y), fabsf(v.z)};
}

struct vec3 vec3_spherical(float theta, float phi)
{
	return (struct vec3){
	  -cosf(phi) * sinf(theta),
	  -cosf(theta),
	  sinf(phi) * sinf(theta)};
}

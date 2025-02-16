#include <float.h>
#include <math.h>

#include "aabb.h"
#include "util.h"

void aabb_init(struct aabb *a)
{
	*a = (struct aabb){
	  (struct vec3){FLT_MAX, FLT_MAX, FLT_MAX},
	  (struct vec3){-FLT_MAX, -FLT_MAX, -FLT_MAX}};
}

void aabb_combine(struct aabb * restrict dst,
                  const struct aabb * a,
                  const struct aabb * b)
{
	*dst = (struct aabb){
	  vec3_min(a->min, b->min), vec3_max(a->max, b->max)};
}

void aabb_grow(struct aabb *a, struct vec3 v)
{
	a->min = vec3_min(a->min, v);
	a->max = vec3_max(a->max, v);
}

void aabb_pad(struct aabb *a)
{
	float he = EPS * 0.5f;
	struct vec3 d = vec3_sub(a->max, a->min);
	a->min.x = fabsf(d.x) < EPS ? a->min.x - he : a->min.x;
	a->max.x = fabsf(d.x) < EPS ? a->max.x + he : a->max.x;
	a->min.y = fabsf(d.y) < EPS ? a->min.y - he : a->min.y;
	a->max.y = fabsf(d.y) < EPS ? a->max.y + he : a->max.y;
	a->min.z = fabsf(d.z) < EPS ? a->min.z - he : a->min.z;
	a->max.z = fabsf(d.z) < EPS ? a->max.z + he : a->max.z;
}

float aabb_calcarea(const struct aabb *a)
{
	struct vec3 d = vec3_sub(a->max, a->min);
	return d.x * d.y + d.y * d.z + d.z * d.x;
}

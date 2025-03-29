#ifndef AABB_H
#define AABB_H

#include "vec3.h"

struct aabb {
	struct vec3  min;
	//float        pad0;
	struct vec3  max;
	//float        pad1;
};

void    aabb_init(struct aabb *a);
void    aabb_combine(struct aabb *dst,
                     const struct aabb * a,
                     const struct aabb * b);
void   aabb_grow(struct aabb *a, struct vec3 v);
void   aabb_pad(struct aabb *a);
float  aabb_calcarea(const struct aabb *a);

#endif

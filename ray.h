#ifndef RAY_H
#define RAY_H

#include "vec3.h"

struct ray {
	struct vec3 ori;
	struct vec3 dir;
	struct vec3 idir;
};

void  ray_create(struct ray *r, const struct vec3 *ori, const struct vec3 *dir);
void  ray_transform(struct ray *dst, const struct ray *src, float m[16]);

#endif

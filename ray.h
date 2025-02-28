#ifndef RAY_H
#define RAY_H

#include "vec3.h"

struct ray {
	struct vec3 ori;
	struct vec3 dir;
	struct vec3 idir;
};

struct ray  ray_create(struct vec3 ori, struct vec3 dir);

#endif

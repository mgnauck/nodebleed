#include "ray.h"

struct ray ray_create(struct vec3 ori, struct vec3 dir)
{
	return (struct ray){
	  .ori = ori, .dir = dir,
	  .idir = (struct vec3){1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z}};
}

#include "mat4.h"
#include "ray.h"

void ray_create(struct ray *r, const struct vec3 *ori, const struct vec3 *dir)
{
	*r = (struct ray){
	  .ori = *ori, .dir = *dir,
	  .idir = (struct vec3){1.0f / dir->x, 1.0f / dir->y, 1.0f / dir->z}};
}

void ray_transform(struct ray *dst, const struct ray *src, float m[16])
{
	dst->ori = mat4_mulpos(m, src->ori); 
	struct vec3 dir = mat4_muldir(m, src->dir); 
	dst->dir = dir;
	dst->idir = (struct vec3){1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z};
}

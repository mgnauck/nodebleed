#include "mat4.h"
#include "ray.h"

struct ray ray_create(struct vec3 ori, struct vec3 dir)
{
	return (struct ray){
	  .ori = ori, .dir = dir,
	  .idir = (struct vec3){1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z}};
}

struct ray ray_transform(struct ray r, float m[16])
{
	struct ray s;
	s.ori = mat4_mulpos(m, r.ori); 
	s.dir = mat4_muldir(m, r.dir); 
	s.idir = (struct vec3){
	  1.0f / s.dir.x,
	  1.0f / s.dir.y,
	  1.0f / s.dir.z};
	return s;
}

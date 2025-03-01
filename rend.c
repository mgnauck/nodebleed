#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "mat4.h"
#include "ray.h"
#include "rend.h"
#include "util.h"

void rend_init(struct rdata *rd, unsigned int maxmtls,
               unsigned int maxtris, unsigned int maxinsts) 
{
	rd->mtls = malloc(maxmtls * sizeof(*rd->mtls));
	rd->tris = malloc(maxtris * sizeof(*rd->tris));
	rd->nrms = malloc(maxtris * sizeof(*rd->nrms));
	rd->insts = malloc(maxinsts * sizeof(*rd->insts));
}

void rend_release(struct rdata *rd)
{
	free(rd->insts);
	free(rd->nrms);
	free(rd->tris);
	free(rd->mtls);
}

struct hit {
	float     t;
	float     u;
	float     v;
	uint32_t  e;
};

void intersect_tri(struct hit *h, const struct ray *r, const struct vec3 *v0,
                   const struct vec3 *v1, const struct vec3 *v2, uint32_t id)
{
	// Vectors of two edges sharing v0
	const struct vec3 e0 = vec3_sub(*v1, *v0);
	const struct vec3 e1 = vec3_sub(*v2, *v0);

	// Calculate determinat and u param later on
	const struct vec3 pv = vec3_cross(r->dir, e1);
	float det = vec3_dot(e0, pv);

	if (fabsf(det) < EPS)
		// Ray in plane of triangle
		return;

	float idet = 1.0f / det;

	// Distance v0 to origin
	const struct vec3 tv = vec3_sub(r->ori, *v0);

	// Calculate param u and test bounds
	float u = vec3_dot(tv, pv) * idet;
	if (u < 0.0f || u > 1.0f)
		return;

	// Prepare to test for v
	const struct vec3 qv = vec3_cross(tv, e0);

	// Calculate param v and test bounds
	float v = vec3_dot(r->dir, qv) * idet;
	if (v < 0.0f || u + v > 1.0f)
		return;

	// Calculate dist
	float dist = vec3_dot(e1, qv) * idet;
	if (dist > EPS && dist < h->t) {
		h->t = dist;
		h->u = u;
		h->v = v;
		h->e = id;
	}
}

void intersect_insts(struct hit *h, const struct ray *r, const struct rdata *rd)
{
	for (unsigned int j = 0; j < rd->instcnt; j++) {
		const struct rinst *ri= &rd->insts[j];
		float inv[16];
		mat4_from3x4(inv, ri->globinv);
		struct ray ros;
		ray_transform(&ros, r, inv);
		const struct rtri *t = &rd->tris[ri->triofs];
		for (unsigned int i = 0; i < ri->tricnt; i++) {
			intersect_tri(h, &ros, &t->v0, &t->v1, &t->v2, i << 16 | j);
			t++;
		}
	}
}

void rend_render(void *dst, struct rdata *rd)
{
	struct vec3 eye = rd->cam.eye;
	struct vec3 dx = rd->view.dx;
	struct vec3 dy = rd->view.dy;
	struct vec3 le = rd->view.tl;

	uint32_t *buf = dst;
	unsigned int ofs = 0;
	for (unsigned int j = 0; j < rd->view.h; j++) {
		struct vec3 p = le;
		for (unsigned int i = 0; i < rd->view.w; i++) {
			struct ray r;
			ray_create(&r, &eye, &vec3_unit(vec3_sub(p, eye)));
			struct hit h = (struct hit){ .t = FLT_MAX };
			intersect_insts(&h, &r, rd);

			struct vec3 c = rd->bgcol;
			if (h.t < FLT_MAX) {
				unsigned int instid = h.e & 0xffff;
				unsigned int triid = h.e >> 16;
				struct rinst *ri = &rd->insts[instid];
				unsigned int mtlid = rd->nrms[ri->triofs + triid].mtlid;
				c = rd->mtls[mtlid].col;
			}

			unsigned int cr = min(255, (unsigned int)(255 * c.x));
			unsigned int cg = min(255, (unsigned int)(255 * c.y));
			unsigned int cb = min(255, (unsigned int)(255 * c.z));
			buf[ofs + i] = 0xff << 24 | cr << 16 | cg << 8 | cb;

			p = vec3_add(p, dx);
		}

		le = vec3_add(le, dy);
		ofs += rd->view.w;
	}
}

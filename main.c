#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <threads.h>
#include <unistd.h>

#include "SDL.h"

#include <math.h>
#include <float.h>

#include "import.h"
#include "mat4.h"
#include "rend.h"
#include "scene.h"
#include "util.h"

#define WIDTH    1920
#define HEIGHT   1080

#define CAMMOV   0.800
#define CAMLOOK  0.025

#ifndef NDEBUG
#define dprintf printf
#else
#define dprintf(...) {}
#endif

long long  start;
bool       converge = true;
bool       cntrlcam = false;

unsigned int get_max_tris(struct mesh *meshes, unsigned int meshcnt)
{
	unsigned int maxtris = 0;
	for (unsigned int i = 0; i < meshcnt; i++)
		maxtris += meshes[i].icnt / 3;
	return maxtris;
}

unsigned int get_max_insts(struct obj *objs, unsigned int objcnt)
{
	unsigned int maxinsts = 0;
	for (unsigned int i = 0; i < objcnt; i++)
		if (hasflags(objs[i].flags, MESH))
			maxinsts++;
	return maxinsts;
}

void cpy_rdata(struct rdata *rd, struct scene *s)
{
	unsigned int triofs[s->meshcnt];
	unsigned int tricnt[s->meshcnt];
	unsigned int ofs = 0;

	struct rtri *rt = rd->tris;
	struct rnrm *rn = rd->nrms;

	// Copy meshes
	for (unsigned int k = 0; k < s->meshcnt; k++) {
		struct mesh *m = &s->meshes[k];
		unsigned int *ip = m->inds;
		for (unsigned int j = 0; j < m->mcnt; j++) {
			struct mtlref *mr = &m->mtls[j];
			for (unsigned int i = 0; i < mr->tricnt; i++) {
				memcpy(&rt->v0, &m->vrts[*(ip + 0)],
				  sizeof(rt->v0));
				memcpy(&rt->v1, &m->vrts[*(ip + 1)],
				  sizeof(rt->v1));
				memcpy(&rt->v2, &m->vrts[*(ip + 2)],
				  sizeof(rt->v2));
				memcpy(&rn->n0, &m->nrms[*(ip + 0)],
				  sizeof(rn->n0));
				memcpy(&rn->n1, &m->nrms[*(ip + 1)],
				  sizeof(rn->n1));
				memcpy(&rn->n2, &m->nrms[*(ip + 2)],
				  sizeof(rn->n2));
				rn->mtlid = mr->mtlid;
				ip += 3;
				rt++;
				rn++;
			}
		}
		triofs[k] = ofs; // Ofs into tri buffer
		tricnt[k] = m->icnt / 3; // Tri cnt per mesh
		ofs += m->icnt / 3;
	}

	// Copy mtls
	for (unsigned int i = 0; i < s->mtlcnt; i++)
		memcpy(&rd->mtls[i], &s->mtls[i], sizeof(*rd->mtls));

	// Create instances from mesh nodes
	unsigned int cnt = 0;
	for (unsigned int i = 0; i < s->nodecnt; i++) {
		struct obj *o = &s->objs[i];
		if (hasflags(o->flags, MESH)) {
			rd->insts[cnt] = (struct rinst){
			  .flags = o->flags,
			  .triofs = triofs[o->objid],
			  .tricnt = tricnt[o->objid]};
			o->instid = cnt++;
		}
	}
	rd->instcnt = cnt;

	// Other data
	rd->bgcol = s->bgcol;
}

void upd_rinsts(struct rdata *rd, struct scene *s)
{
	for (unsigned int j = 0; j < s->nodecnt; j++) {
		struct obj *o = &s->objs[j];
		if (!hasflags(o->flags, MESH))
			continue;

		// Update transforms
		struct tfmat *t = &s->tfmats[j];
		struct rinst *ri = &rd->insts[o->instid];
		float inv[16];
		mat4_inv(inv, t->glob);
		memcpy(ri->globinv, inv, sizeof(ri->globinv)); // 3x4

		// Update instance aabbs by transforming blas root to world
		float *m = t->glob;
		struct bnode8 *n = &rd->bnodes[ri->triofs << 1];

		struct vec3 nmi =
		  {min8(n->minx8), min8(n->miny8), min8(n->minz8)};
		struct vec3 nma =
		  {max8(n->maxx8), max8(n->maxy8), max8(n->maxz8)};

		struct vec3 mi = {FLT_MAX, FLT_MAX, FLT_MAX};
		struct vec3 ma = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
		for (unsigned char i = 0; i < 8; i++) {
			struct vec3 v = mat4_mulpos(m, (struct vec3){
			  i & 1 ? nmi.x : nma.x,
			  i & 2 ? nmi.y : nma.y,
			  i & 4 ? nmi.z : nma.z});
			mi = vec3_min(mi, v);
			ma = vec3_max(ma, v);
		}

		rd->aabbs[o->instid] = (struct aabb){.min = mi, .max = ma};
	}
}

void upd_rcam(struct rcam *rc, struct cam *c)
{
	*rc = (struct rcam){
	  .eye = c->eye,
	  .tanfov = tanf(0.5f * c->vfov * PI / 180.0f),
	  .ri = c->ri,
	  .focdist = c->focdist,
	  .up = c->up,
	  .focangle = c->focangle * PI / 180.0f,
	  .fwd = c->fwd,
	  .aspect = WIDTH / (float)HEIGHT};
}

void cam_calcbase(struct cam *c)
{
	c->ri = vec3_unit(vec3_cross(c->fwd, (struct vec3){0.0f, 1.0f, 0.0f}));
	c->up = vec3_unit(vec3_cross(c->ri, c->fwd));
}

void cam_setdir(struct cam *c, struct vec3 dir)
{
	c->fwd = vec3_unit(dir);
	cam_calcbase(c);
}

void init(struct scene *s, struct rdata *rd)
{
	//import_file_gltf(s, "../data/animcube.gltf");
	//import_file_gltf(s, "../data/suzy.gltf");
	//import_file_gltf(s, "../data/sponza.gltf");
	//import_file_gltf(s, "../data/toycar.gltf");
	import_file_gltf(s, "../raynin/data/good_7.gltf");

	assert(s);

	dprintf("Imported scene with %d meshes, %d mtls, %d cams, %d nodes, %d tracks, %d samplers\n",
	  s->meshcnt, s->mtlcnt, s->camcnt, s->nodecnt, s->trackcnt,
	  s->samplercnt);

	s->bgcol = (struct vec3){0.4f, 0.4f, 0.4};

	unsigned int trimax = get_max_tris(s->meshes, s->meshcnt);
	unsigned int instmax = get_max_insts(s->objs, s->nodecnt);

	rend_init_compresslut();
	rend_init(rd, s->mtlmax, trimax, instmax);
	rend_resaccum(rd, WIDTH, HEIGHT);

	cpy_rdata(rd, s);

	dprintf("Created render data with %d mtls, %d tris, %d insts\n",
	  s->mtlmax, trimax, instmax);

	long long last = SDL_GetTicks64();
	rend_prepstatic(rd);
	dprintf("Created all blas in %llu ms\n", SDL_GetTicks64() - last);

	scene_updanims(s, 0.0f);
	scene_updtransforms(s);
	scene_updcams(s);
}

void update(struct rdata *rd, struct scene *s, float time)
{
	scene_updanims(s, time);

	if (hasflags(s->dirty, TRANSFORM)) {
		scene_updtransforms(s);

		upd_rinsts(rd, s);

		long long last = SDL_GetTicks64();
		rend_prepdynamic(rd);
		dprintf("Tlas update in %llu ms\n", SDL_GetTicks64() - last);
	}

	if (hasflags(s->dirty, CAM)) {
		if (!cntrlcam)
			scene_updcams(s);
		upd_rcam(&rd->cam, &s->cams[s->currcam]);
	}

	if (!converge || anyflags(s->dirty, TRANSFORM | CAM)) {
		rend_clraccum(rd);
		clrflags(&s->dirty, TRANSFORM | CAM);
	}
}

void keydown(struct scene *s, int key)
{
	if (cntrlcam) {
		struct cam *c = &s->cams[s->currcam];

		switch (key) {
		case 'a':
			c->eye = vec3_add(c->eye, vec3_scale(c->ri, -CAMMOV));
			setflags(&s->dirty, CAM);
			break;
		case 'd':
			c->eye = vec3_add(c->eye, vec3_scale(c->ri, CAMMOV));
			setflags(&s->dirty, CAM);
			break;
		case 'w':
			c->eye = vec3_add(c->eye, vec3_scale(c->fwd, CAMMOV));
			setflags(&s->dirty, CAM);
			break;
		case 's':
			c->eye = vec3_add(c->eye, vec3_scale(c->fwd, -CAMMOV));
			setflags(&s->dirty, CAM);
			break;
		}
	}

	switch(key) {
	case 'r':
		start = SDL_GetTicks64();
		scene_updcams(s);
		setflags(&s->dirty, CAM);
		break;
	case 'c':
		cntrlcam = !cntrlcam;
		break;
	case ' ':
		converge = !converge;
		break;
	}
}

void mousemove(struct scene *s, int dx, int dy)
{
	if (!cntrlcam)
		return;

	struct cam *c = &s->cams[s->currcam];

	float theta = min(max(acosf(c->fwd.y) + CAMLOOK * (float)dy,
	  0.05f), 0.95f * PI);
	float phi = fmodf(atan2f(c->fwd.z, c->fwd.x) + CAMLOOK *
	  (float)dx, TWO_PI);

	cam_setdir(c, vec3_spherical(theta, phi));

	setflags(&s->dirty, CAM);
}

int main(void)
{
	// TODO Test bvh refit with bnode8
	// TODO Import from blender custom bin export
	// TODO Subdiv surfaces
	// TODO Move code from main into some subsys
	// TODO Build step that outputs compressed version

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
		exit(1);

	SDL_Window *win = SDL_CreateWindow("unik", SDL_WINDOWPOS_CENTERED,
	  SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
	if (!win) {
		SDL_Quit();
		exit(1);
	}

	SDL_Surface *scr = SDL_GetWindowSurface(win);
	if (!scr) {
		SDL_DestroyWindow(win);
		SDL_Quit();
		exit(1);
	}

	SDL_SetRelativeMouseMode(SDL_TRUE);

	struct scene s = { 0 };
	struct rdata rd = { 0 };
	init(&s, &rd);

	rd.buf = scr->pixels;
	rd.blksz = 20;

	unsigned int thrdcnt = (int)sysconf(_SC_NPROCESSORS_ONLN);
	thrd_t thrds[thrdcnt];

	bool quit = false;
	start = SDL_GetTicks64();
	while (!quit) {
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			if (ev.type == SDL_QUIT) {
				quit = true;
			} else if (ev.type == SDL_KEYDOWN) {
				if (ev.key.keysym.sym == SDLK_ESCAPE)
					quit = true;
				else
					keydown(&s, ev.key.keysym.sym);
			} else if (ev.type == SDL_MOUSEMOTION) {
				mousemove(&s, ev.motion.xrel, ev.motion.yrel);
			}
		}

		long long last = SDL_GetTicks64();
		update(&rd, &s, (last - start) / 1000.0f);

		for (unsigned int i = 0; i < thrdcnt; i++)
			thrd_create(&thrds[i], rend_render, &rd);
		for (unsigned int i = 0; i < thrdcnt; i++)
			thrd_join(thrds[i], NULL);

		SDL_UpdateWindowSurface(win);

#ifndef NDEBUG
		char title[64];
		long long dur = SDL_GetTicks() - last;
		snprintf(title, 64, "%llu ms, %6.3f Mrays/s, %d samples",
		  dur, rd.rays / (float)(dur * 1000), rd.samples);
		SDL_SetWindowTitle(win, title);
#endif

		rd.blknum = 0;
		rd.rays = 0;
		rd.samples++;
	}

	rend_release(&rd);
	scene_release(&s);

	SDL_DestroyWindow(win);
	SDL_Quit();

	return 0;
}

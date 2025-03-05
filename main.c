#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <SDL.h>

#include "import.h"
#include "mat4.h"
#include "rend.h"
#include "scene.h"
#include "util.h"

#define WIDTH   1024
#define HEIGHT   768

void print_type_sizes(void)
{
	printf("sizeof(char): %ld\n", sizeof(char));
	printf("sizeof(short int): %ld\n", sizeof(short int));
	printf("sizeof(int): %ld\n", sizeof(int));
	printf("sizeof(long int): %ld\n", sizeof(long int));
	printf("sizeof(long long int): %ld\n", sizeof(long long int));
	printf("sizeof(void *): %ld\n", sizeof(void *));
	printf("sizeof(float): %ld\n", sizeof(float));
	printf("sizeof(double): %ld\n", sizeof(double));
	printf("\n");
}

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
		struct mesh *m = scene_getmesh(s, k);
		unsigned int *ip = m->inds;
		for (unsigned int j = 0; j < m->mcnt; j++) {
			struct mtlref *mr = &m->mtls[j];
			for (unsigned int i = 0; i < mr->tricnt; i++) {
				memcpy(&rt->v0, &m->vrts[*(ip + 0)], sizeof(rt->v0));
				memcpy(&rt->v1, &m->vrts[*(ip + 1)], sizeof(rt->v1));
				memcpy(&rt->v2, &m->vrts[*(ip + 2)], sizeof(rt->v2));
				memcpy(&rn->n0, &m->nrms[*(ip + 0)], sizeof(rn->n0));
				memcpy(&rn->n1, &m->nrms[*(ip + 1)], sizeof(rn->n1));
				memcpy(&rn->n2, &m->nrms[*(ip + 2)], sizeof(rn->n2));
				rn->mtlid = mr->mtlid;
				ip += 3;
				rt++;
				rn++;
			}
		}
		triofs[k] = ofs;
		tricnt[k] = m->icnt / 3;
		ofs += m->icnt / 3;
	}

	// Copy mtls
	for (unsigned int i = 0; i < s->mtlcnt; i++)
		memcpy(&rd->mtls[i], scene_getmtl(s, i), sizeof(*rd->mtls));

	// Create instances from mesh nodes
	unsigned int cnt = 0;
	for (unsigned int i = 0; i < s->nodecnt; i++) {
		struct obj *o = scene_getobj(s, i);
		if (hasflags(o->flags, MESH)) {
			rd->insts[cnt] = (struct rinst){
			  .id = cnt,
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

void set_inst_transforms(struct rdata *rd, struct scene *s)
{
	for (unsigned int i = 0; i < s->nodecnt; i++) {
		struct obj *o = scene_getobj(s, i);
		if (hasflags(o->flags, MESH)) {
			struct transform *t = scene_gettransform(s, i);
			struct rinst *ri = &rd->insts[o->instid];
			float inv[16];
			mat4_inv(inv, t->glob);
			// globinv is only 3x4
			memcpy(&ri->globinv, inv, sizeof(ri->globinv));
		}
	}
}

void set_rcam(struct rcam *rc, struct cam *c)
{
	*rc = (struct rcam){
	  .eye = c->eye,
	  .vfov = c->vfov * PI / 180.0f,
	  .ri = c->ri,
	  .focangle = c->focangle * PI / 180.0f,
	  .up = c->up,
	  .focdist = c->focdist};
}

void calc_view(struct rview *v, uint32_t width, uint32_t height, struct cam *c)
{
	float v_height = 2.0f * tanf(0.5f * c->vfov * PI / 180.0f) * c->focdist;
	float v_width = v_height * (float)width / (float)height;

	struct vec3 v_right = vec3_scale(c->ri, v_width);
	struct vec3 v_down = vec3_scale(c->up, -v_height);

	// Pixel delta x/y
	v->dx = vec3_scale(v_right, 1.0f / width);
	v->dy = vec3_scale(v_down, 1.0f / height);

	// view_topleft = eye - focdist * fwd - 0.5 * (view_right + view_down)
	struct vec3 v_topleft = vec3_add(c->eye, vec3_add(
	  vec3_scale(c->fwd, -c->focdist),
	  vec3_scale(vec3_add(v_right, v_down), -0.5f)));

	// pix_topleft = view_topleft + 0.5 * (pix_dx + pix_dy)
	v->tl = vec3_add(v_topleft,
	  vec3_scale(vec3_add(v->dx, v->dy), 0.5f));

	v->w = width;
	v->h = height;
}

void init(struct scene *s, struct rdata *rd)
{
	if (import_gltf(s, "../data/suzy.gltf", "../data/suzy.bin") != 0)
		printf("Failed to import gltf\n");

	printf("imported scene with %d meshes, %d mtls, %d cams, %d roots, %d nodes\n",
	  s->meshcnt, s->mtlcnt, s->camcnt, s->rootcnt, s->nodecnt);

	unsigned int trimax = get_max_tris(s->meshes, s->meshcnt);
	unsigned int instmax = get_max_insts(s->objs, s->nodecnt);

	rend_init(rd, s->mtlmax, trimax, instmax);
	cpy_rdata(rd, s);

	printf("created render data with %d mtls, %d tris, %d insts\n", s->mtlmax, trimax, instmax);

	long last = SDL_GetTicks();
	rend_prepstatic(rd);
	printf("created bvhs in %ld ms\n", SDL_GetTicks() - last);
}

void update(struct rdata *rd, struct scene *s)
{
	scene_updtransforms(s);
	scene_updcams(s);

	set_inst_transforms(rd, s);

	struct cam *c = scene_getcam(s, s->currcam);
	set_rcam(&rd->cam, c);

	calc_view(&rd->view, WIDTH, HEIGHT, c);
}

int main(int argc, char *argv[])
{
	// TODO TLAS
	// TODO Move code from main into some subsys
	// TODO Animation test
	// TODO Static/dynamic separation of meshes in the node tree (incl. premul)

	assert(sizeof(uint32_t) == sizeof(unsigned int));
	assert(sizeof(uint16_t) == sizeof(unsigned short int));
	//print_type_sizes();

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
		return 1;

	SDL_Window *win = SDL_CreateWindow("unik",
	  SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
	if (!win) {
		SDL_Quit();
		return 1;
	}

	SDL_Surface *scr = SDL_GetWindowSurface(win);
	if (!scr) {
		SDL_DestroyWindow(win);
		return 1;
	}

	struct scene s = { 0 };
	struct rdata rd = { 0 };
	init(&s, &rd);

	bool quit = false;
	long last = SDL_GetTicks64();
	while (!quit) {
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT ||
			    (event.type == SDL_KEYDOWN &&
			     event.key.keysym.sym == SDLK_ESCAPE))
				quit = true;
		}

		char title[64];
		snprintf(title, 64, "%ld ms", SDL_GetTicks64() - last);
		SDL_SetWindowTitle(win, title);
		last = SDL_GetTicks64();

		update(&rd, &s);
		rend_render(scr->pixels, &rd);

		SDL_UpdateWindowSurface(win);
	}

	rend_release(&rd);
	scene_release(&s);

	SDL_DestroyWindow(win);
	SDL_Quit();

	return 0;
}

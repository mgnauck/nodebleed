#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "import.h"
#include "mat4.h"
#include "rend.h"
#include "scene.h"
#include "util.h"

#define WIDTH  1024
#define HEIGHT  768

void print_typesz(void)
{
	printf("sizeof(void *): %ld\n", sizeof(void *));
	printf("sizeof(float): %ld\n", sizeof(float));
	printf("sizeof(double): %ld\n", sizeof(double));
	printf("sizeof(char): %ld\n", sizeof(char));
	printf("sizeof(short int): %ld\n", sizeof(short int));
	printf("sizeof(int): %ld\n", sizeof(int));
	printf("sizeof(long int): %ld\n", sizeof(long int));
	printf("sizeof(long long int): %ld\n", sizeof(long long int));
	printf("\n\n");
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
		ofs += m->icnt / 3;
	}

	// Copy mtls
	for (unsigned int i = 0; i < s->mtlcnt; i++)
		memcpy(&rd->mtls[i], scene_getmtl(s, i), sizeof(*s->mtls));

	// Create instances from mesh nodes
	unsigned int instid = 0;
	for (unsigned int i = 0; i < s->nodecnt; i++) {
		struct obj *o = scene_getobj(s, i);
		if (hasflags(o->flags, MESH)) {
			rd->insts[instid] = (struct rinst){
			  .id = instid, .ofs = triofs[o->objid], .flags = o->flags};
			o->instid = instid++;
		}
	}
}

void upd_inst_transforms(struct rdata *rd, struct scene *s)
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

void set_rcam(struct rcam *rc, struct cam *c, float transform[16])
{
	*rc = (struct rcam){
	  .vfov = c->vertfov, .focangle = c->focangle, .focdist = c->focdist};

	rcam_set(rc, mat4_gettrans(transform),
	  mat4_muldir(transform, (struct vec3){0.0f, 0.0f, 1.0f}));
}

int main(int argc, char *argv[])
{
	assert(sizeof(uint32_t) == sizeof(unsigned int));
	assert(sizeof(uint16_t) == sizeof(unsigned short int));

	struct scene s = { 0 };
	if (import_gltf(&s, "../data/test.gltf", "../data/test.bin") != 0)
		printf("Failed to import gltf\n");

	printf("imported scene with %d meshes, %d mtls, %d cams, %d roots, %d nodes\n",
	  s.meshcnt, s.mtlcnt, s.camcnt, s.rootcnt, s.nodecnt);

	unsigned int trimax = get_max_tris(s.meshes, s.meshcnt);
	unsigned int instmax = get_max_insts(s.objs, s.nodecnt);

	struct rdata rd = { 0 };
	rend_init(&rd, s.mtlmax, trimax, instmax);

	cpy_rdata(&rd, &s);

	scene_updtransforms(&s);
	upd_inst_transforms(&rd, &s);

	struct cam *c = scene_getcam(&s, s.currcam);
	set_rcam(&rd.cam, c, scene_gettransform(&s, c->nodeid)->glob);

	// TODO Visualize
	// TODO Animation test
	// TODO Static/dynamic separation of meshes in the node tree (incl. premul)

	rend_release(&rd);
	scene_release(&s);

	return 0;
}

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "gltf.h"
#include "import.h"
#include "mat4.h"
#include "scene.h"
#include "util.h"

#define dprintf printf
//#define dprintf(...)

unsigned int getmtlflags(struct gltfmtl *gm)
{
	unsigned int flags = 0;
	setflags(&flags, gm->refractive ? REFRACTIVE : 0);
	setflags(&flags,
	  gm->emission[0] + gm->emission[1] + gm->emission[2] > 0.0f ?
	  EMISSIVE : 0);
	return flags;
}

void import_cam(struct scene *s, struct gltfcam *gc)
{
	scene_initcam(s, scene_acquirecam(s), gc->name,
	  gc->vfov * 180.0f / PI, 10.0f, 0.0f);
}

void import_mtl(struct scene *s, struct gltfmtl *gm)
{
	struct mtl *m = scene_initmtl(s, scene_acquiremtl(s), gm->name,
	  (struct vec3){gm->col[0], gm->col[1], gm->col[2]});
	m->metallic = gm->metallic;
	m->roughness = gm->roughness;
	m->flags = getmtlflags(gm);
	if (hasflags(m->flags, EMISSIVE))
		m->col = (struct vec3){
		  gm->emission[0], gm->emission[1], gm->emission[2]};
}

void import_mesh(struct scene *s, struct gltfmesh *gm, struct gltf *g,
                 const unsigned char *bin)
{
	// Sum number of indices and vertices of each primitive
	unsigned int icnt = 0, vcnt = 0;
	for (unsigned int j = 0; j < gm->primcnt; j++) {
		struct gltfprim *p = &gm->prims[j];
		struct gltfaccessor *a = &g->accessors[p->indid];
		if (a->datatype != DT_SCALAR)
			dprintf("Expected indices accessor with scalar data type. Ignoring primitive.\n");
		icnt += a->cnt;
		a = &g->accessors[p->posid];
		if (a->datatype != DT_VEC3)
			dprintf("Expected vertices accessor with vec3 data type. Ignoring primitive.\n");
		vcnt += a->cnt;
	}

	struct mesh *m = scene_initmesh(s, scene_acquiremesh(s),
	  vcnt, icnt, gm->primcnt);

	for (unsigned int j = 0; j < gm->primcnt; j++) {
		struct gltfprim *p = &gm->prims[j];

		// Indices
		struct gltfaccessor *iacc = &g->accessors[p->indid];
		if (iacc->datatype != DT_SCALAR) {
			dprintf("Expected indices accessor with scalar data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *ibv = &g->bufviews[iacc->bufview];
		unsigned int *ip = m->inds + m->icnt;
		const unsigned char *bi = bin + ibv->byteofs;
		if (iacc->comptype == 5121) {
			uint8_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, bi, sizeof(v));
				bi += sizeof(v);
				*ip++ = v;
			}
		} else if (iacc->comptype == 5123) {
			uint16_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, bi, sizeof(v));
				bi += sizeof(v);
				*ip++ = v;
			}
		} else if(iacc->comptype == 5125) {
			uint32_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, bi, sizeof(v));
				bi += sizeof(v);
				*ip++ = v;
			}
		} else {
			dprintf("Expected index buffer with byte, short or int components. Ignoring primitive.\n");
			continue;
		}
		m->icnt += ip - m->inds + m->icnt;

		// Vertices
		struct gltfaccessor *vacc = &g->accessors[p->posid];
		if (vacc->datatype != DT_VEC3) {
			dprintf("Expected vertices accessor with vec3 data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *vbv = &g->bufviews[vacc->bufview];
		if (vacc->comptype != 5126) { // 5126 = float
			dprintf("Expected vertex buffer with float components. Ignoring primitive.\n");
			continue;
		}

		// Normals
		struct gltfaccessor *nacc = &g->accessors[p->nrmid];
		if (nacc->datatype != DT_VEC3) {
			dprintf("Expected normals accessor with vec3 data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *nbv = &g->bufviews[nacc->bufview];
		if (nacc->comptype != 5126) { // 5126 = float
			dprintf("Expected normals buffer with float components. Ignoring primitive.\n");
			continue;
		}

		// Vertex cnt == normal cnt in primitive
		assert(vacc->cnt == nacc->cnt);

		struct vec3 *vp = m->vrts + m->vcnt;
		assert(sizeof(vp->x) == 4);
		struct vec3 *np = m->nrms + m->vcnt;
		assert(sizeof(np->x) == 4);
		const unsigned char *bv = bin + vbv->byteofs;
		const unsigned char *bn = bin + nbv->byteofs;
		for (unsigned int i = 0; i < vacc->cnt; i++) {
			memcpy(&vp->x, bv, sizeof(vp->x));
			bv += sizeof(vp->x);
			memcpy(&vp->y, bv, sizeof(vp->y));
			bv += sizeof(vp->y);
			memcpy(&vp->z, bv, sizeof(vp->z));
			bv += sizeof(vp->z);
			vp++;

			memcpy(&np->x, bn, sizeof(np->x));
			bn += sizeof(np->x);
			memcpy(&np->y, bn, sizeof(np->y));
			bn += sizeof(np->y);
			memcpy(&np->z, bn, sizeof(np->z));
			bn += sizeof(np->z);
			np++;

			m->vcnt++;
		}

		if (p->mtlid < 0) {
			dprintf("Found primitive without mtl. Switching to mtl 0.\n");
			p->mtlid = 0;
		}

		// Track consolidated mtl flags at mesh
		setflags(&m->flags, s->mtls[p->mtlid].flags); 

		// Primitive's material
		m->mtls[j] = (struct mtlref){p->mtlid, iacc->cnt / 3};
		m->mcnt++;
	}
}

void import_nodes(struct scene *s, struct gltfnode *nodes, unsigned int rootid)
{
#define STACK_SIZE 64
	unsigned int gnids[STACK_SIZE];
	unsigned int snids[STACK_SIZE];

	int id = scene_acquirenode(s, true);
	if (id < 0)
		eprintf("failed to acquire node: %s", nodes[rootid].name);
	
	gnids[0] = rootid;
	snids[0] = id;

	unsigned int spos = 1;

	while (spos > 0) {
		// Pop next gltf and scene node
		unsigned int gnid = gnids[--spos];
		unsigned int snid = snids[spos];

		struct gltfnode *n = &nodes[gnid];

		// Obj reference and flags
		int objid = n->camid < 0 ? n->meshid : n->camid;

		unsigned int flags = 0;
		if (n->camid >= 0) {
			setflags(&flags, CAM);
			// Cam references its node for transform
			s->cams[objid].nodeid = snid;
		} else if (n->meshid >= 0) {
			// Track mesh flags at mesh node
			setflags(&flags, MESH | s->meshes[n->meshid].flags);
		}

		// If any children, acquire and push to stack in reverse order
		// Direct descendants will be next to each other in mem
		unsigned int snofs = s->nodecnt;
		for (unsigned int i = 0; i < n->childcnt; i++) {
			if (scene_acquirenode(s, false) < 0)
				eprintf("failed to acquire node: %s", n->name);
			assert(spos < STACK_SIZE);
			unsigned int child = n->childcnt - i - 1;
			gnids[spos] = n->children[child];
			snids[spos++] = snofs + child; 
		}

		// Calc local node transform
		float loc[16], scale[16], rot[16], trans[16];
		mat4_scale(scale, (struct vec3){
		  n->scale[0], n->scale[1], n->scale[2]});
		mat4_fromquat(rot, n->rot[0], n->rot[1], n->rot[2], n->rot[3]);
		mat4_trans(trans, (struct vec3){
		  n->trans[0], n->trans[1], n->trans[2]});
		mat4_mul(loc, rot, scale);
		mat4_mul(loc, trans, loc);

		// Set all data of scene node
		scene_initnode(s, snid, n->name, objid, flags,
		  loc, n->childcnt > 0 ? snofs : 0, n->childcnt);
	}
}

int import_data(struct scene *s,
                const char *gltfbuf, const unsigned char *binbuf)
{
	struct gltf g = { 0 };
	if (gltf_init(&g, gltfbuf) != 0) {
		gltf_release(&g);
		return 1;
	}

	assert(g.meshcnt > 0 && g.rootcnt > 0 && g.nodecnt > 0);

	scene_init(s, g.meshcnt, max(1, g.mtlcnt), max(1, g.camcnt),
	  g.rootcnt, g.nodecnt);

	for (unsigned int i = 0; i < g.mtlcnt; i++)
		import_mtl(s, &g.mtls[i]);

	if (g.mtlcnt == 0) {
		dprintf("Found no materials. Creating default material.\n");
		scene_initmtl(s, scene_acquiremtl(s), "FALLBACK", (struct vec3){
		  1.0f, 0.0f, 0.0f});
	}

	for (unsigned int i = 0; i < g.camcnt; i++)
		import_cam(s, &g.cams[i]);

	if (g.camcnt == 0) {
		dprintf("Found no cameras. Creating default cam using transform of node 0.\n");
		scene_initcam(s, scene_acquirecam(s), "FALLBACK",
		  60.0f * 180.0f / PI, 10.0f, 0.0f)->nodeid = 0;
	}

	for (unsigned int i = 0; i < g.meshcnt; i++)
		import_mesh(s, &g.meshes[i], &g, binbuf);

	for (unsigned int i = 0; i < g.rootcnt; i++)
		import_nodes(s, g.nodes, g.roots[i]);

	gltf_release(&g);

	return 0;
}

int import_gltf(struct scene *s, const char *gltfname, const char *binname)
{
	FILE *f = fopen(gltfname, "rt");
	if (f == NULL)
		eprintf("failed to open %s:", gltfname);

	fseek(f, 0L, SEEK_END);
	size_t gltfsz = ftell(f);
	fseek(f, 0L, SEEK_SET);

	char *gltf = emalloc(gltfsz + 1);
	
 	if (fread(gltf, sizeof(*gltf), gltfsz, f) != gltfsz) {
 		fclose(f);
		free(gltf);
		eprintf("failed to read %u bytes:", gltfsz);
 	}
	gltf[gltfsz] = '\0';

 	fclose(f);

 	f = fopen(binname, "rb");
	if (f == NULL)
		eprintf("failed to open %s:", binname);

	fseek(f, 0L, SEEK_END);
	size_t binsz = ftell(f);
	fseek(f, 0L, SEEK_SET);

	unsigned char *bin = emalloc(binsz);

	if (fread(bin, sizeof(*bin), binsz, f) != binsz) {
		fclose(f);
		free(bin);
		free(gltf);
		eprintf("failed to read %u bytes:", binsz);
	}

	fclose(f);

	int err = import_data(s, gltf, bin);

	free(bin);
	free(gltf);

	return err;
}

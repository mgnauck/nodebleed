#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "gltf.h"
#include "import.h"
#include "scene.h"
#include "util.h"

void import_mesh(struct mesh *m, struct gltfmesh *gm, struct gltf *g,
                 const unsigned char *bin)
{
	// Sum number of indices and vertices of each primitive
	unsigned int icnt = 0, vcnt = 0;
	for (unsigned int j = 0; j < gm->primcnt; j++) {
		struct gltfprim *p = &gm->prims[j];
		struct gltfaccessor *a = &g->accessors[p->indid];
		if (a->datatype != DT_SCALAR)
			printf("Expected indices accessor with scalar data type. Ignoring primitive.\n");
		icnt += a->cnt;
		a = &g->accessors[p->posid];
		if (a->datatype != DT_VEC3)
			printf("Expected vertices accessor with vec3 data type. Ignoring primitive.\n");
		vcnt += a->cnt;
	}

	mesh_init(m, vcnt, icnt, gm->primcnt);

	unsigned int mtlofs = 0;
	for (unsigned int j = 0; j < gm->primcnt; j++) {
		struct gltfprim *p = &gm->prims[j];

		// Indices
		struct gltfaccessor *iacc = &g->accessors[p->indid];
		if (iacc->datatype != DT_SCALAR) {
			printf("Expected indices accessor with scalar data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *ibv = &g->bufviews[iacc->bufview];
		unsigned int *ip = m->inds;
		const unsigned char *b = bin + ibv->byteofs;
		if (iacc->comptype == 5121) {
			uint8_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, b, sizeof(v));
				b += sizeof(v);
				*ip++ = v;
			}
		} else if (iacc->comptype == 5123) {
			uint16_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, b, sizeof(v));
				b += sizeof(v);
				*ip++ = v;
			}
		} else if(iacc->comptype == 5125) {
			uint32_t v;
			for (unsigned int i = 0; i < iacc->cnt; i++) {
				memcpy(&v, b, sizeof(v));
				b += sizeof(v);
				*ip++ = v;
			}
		} else {
		    printf("Expected index buffer with byte, short or int components. Ignoring primitive.\n");
		    continue;
		} 

		// Vertices
		struct gltfaccessor *vacc = &g->accessors[p->posid];
		if (vacc->datatype != DT_VEC3) {
			printf("Expected vertices accessor with vec3 data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *vbv = &g->bufviews[vacc->bufview];
		if (vacc->comptype != 5126) { // 5126 = float
		    printf("Expected vertex buffer with float components. Ignoring primitive.\n");
		    continue;
		}

		struct vec3 *vp = m->vrts;
		b = bin + vbv->byteofs;
		assert(sizeof(vp->x) == 4);
		for (unsigned int i = 0; i < vacc->cnt; i++) {
			memcpy(&vp->x, b, sizeof(vp->x));
			b += sizeof(vp->x);
			memcpy(&vp->y, b, sizeof(vp->y));
			b += sizeof(vp->y);
			memcpy(&vp->z, b, sizeof(vp->z));
			b += sizeof(vp->z);
			vp++;
		}

		// Normals
		struct gltfaccessor *nacc = &g->accessors[p->nrmid];
		if (nacc->datatype != DT_VEC3) {
			printf("Expected normals accessor with vec3 data type. Ignoring primitive.\n");
			continue;
		}

		struct gltfbufview *nbv = &g->bufviews[nacc->bufview];
		if (nacc->comptype != 5126) { // 5126 = float
		    printf("Expected normals buffer with float components. Ignoring primitive.\n");
		    continue;
		}

		struct vec3 *np = m->nrms;
		b = bin + nbv->byteofs;
		assert(sizeof(np->x) == 4);
		for (unsigned int i = 0; i < nacc->cnt; i++) {
			memcpy(&np->x, b, sizeof(np->x));
			b += sizeof(np->x);
			memcpy(&np->y, b, sizeof(np->y));
			b += sizeof(np->y);
			memcpy(&np->z, b, sizeof(np->z));
			b += sizeof(np->z);
			np++;
		}

		// Primitive's material
		m->mtls[j] = (struct mtlinf){p->mtlid, mtlofs, iacc->cnt / 3};
		mtlofs += iacc->cnt / 3;
	}
}

void import_cam(struct cam *c, struct gltfcam *gc)
{
	cam_init(c, gc->name, gc->vertfov * 180.0f / PI, 10.0f, 0.0f);
}

void import_mtl(struct mtl *m, struct gltfmtl *gm)
{
	mtl_init(m, gm->name,
	  (struct vec3){gm->col[0], gm->col[1], gm->col[2]});
	m->metallic = gm->metallic;
	m->roughness = gm->roughness;
	if (gm->refractive > 0.99f)
		setflags(&m->flags, MF_REFRACTIVE);
	if (gm->emission[0] + gm->emission[1] + gm->emission[2] > 0.0f) {
		m->col = (struct vec3){
		  gm->emission[0], gm->emission[1], gm->emission[2]};
		setflags(&m->flags, MF_EMISSIVE);
	}
}

int import_data(struct scene *s,
                const char *gltfbuf, const unsigned char *binbuf)
{
	struct gltf g;
	if (gltf_init(&g, gltfbuf) != 0) {
		gltf_release(&g);
		return 1;
	}

	scene_init(s, g.meshcnt, g.mtlcnt, g.camcnt, g.rootcnt, g.nodecnt);

	for (unsigned int i = 0; i < g.rootcnt; i++)
		scene_initrnode(s, g.roots[i]);

	for (unsigned int i = 0; i < g.meshcnt; i++)
		import_mesh(
		  scene_getmesh(s, scene_acquiremesh(s)),
		  &g.meshes[i], &g, binbuf);

	for (unsigned int i = 0; i < g.camcnt; i++)
		import_cam(
		  scene_getcam(s, scene_acquirecam(s)), &g.cams[i]);

	for (unsigned int i = 0; i < g.mtlcnt; i++)
		import_mtl(
		  scene_getmtl(s, scene_acquiremtl(s)), &g.mtls[i]);

	// TODO Create nodes

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

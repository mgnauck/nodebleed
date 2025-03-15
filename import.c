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

void import_nodes(struct scene *s, unsigned int nmap[], struct gltfnode *nodes,
                  unsigned int rootid)
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

		nmap[gnid] = snid; // Store for later

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

		// Set all data of scene node
		scene_initnode(s, snid, n->name, objid, flags,
		  &(struct vec3){n->trans[0], n->trans[1], n->trans[2]},
		  n->rot,
		  &(struct vec3){n->scale[0], n->scale[1], n->scale[2]},
		  n->childcnt > 0 ? snofs : 0, n->childcnt);
	}
}

void import_anim(struct scene *s, struct gltfanim *a, struct gltf *g,
                 unsigned int nmap[], unsigned int accmap[])
{
	unsigned int sofs = s->samplercnt; // Sampler offset

	// Copy channels and translate data target enum
	for (unsigned int i = 0; i < a->channelcnt; i++) {
		struct gltfchan *c = &a->channels[i];
		enum tgttype tgt;
		switch (c->target.path) {
		case PA_TRANSLATION:
			tgt = TGT_TRANS;
			break;
		case PA_SCALE:
			tgt = TGT_SCALE;
			break;
		case PA_ROTATION:
			tgt = TGT_ROT;
			break;
		case PA_WEIGHTS: // Not yet supported
		default:
			eprintf("unknown animation target");
			tgt = TGT_TRANS;
		};
		scene_inittrack(s, scene_acquiretrack(s), sofs + c->sampler,
		  nmap[c->target.node], tgt);
	}

	// Copy samplers and referenced data
	for (unsigned int i = 0; i < a->samplercnt; i++) {
		struct gltfsampler *sa = &a->samplers[i];
		struct gltfaccessor *ia = &g->accessors[sa->input];
		enum interpmode interp;
		switch (sa->interp) {
		case IN_STEP:
			interp = IM_STEP;
			break;
		case IN_LINEAR:
			interp = IM_LINEAR;
			break;
		case IN_CUBIC:
			interp = IM_CUBIC;
			break;
		default:
			eprintf("unknown keyframe interpolation mode");
			interp = IM_STEP;
		};
		// Find data len via keyframe cnt, track's tgt and interp mode,
		// accmap already contains correct ofs into anim data buffer
		scene_initsampler(s, scene_acquiresampler(s), ia->cnt,
		  accmap[sa->input] / sizeof(float),
		  accmap[sa->output] / sizeof(float), interp);
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

	unsigned int animacc[g.accessorcnt]; // Anim data accessors
	memset(animacc, 0, g.accessorcnt * sizeof(unsigned int));

	unsigned int scnt = 0, tcnt = 0;
	for (unsigned int j = 0; j < g.animcnt; j++) {
		struct gltfanim *a = &g.anims[j];
		scnt += a->samplercnt;
		tcnt += a->channelcnt;
		// Create a list of accessors that reference animation data
		for (unsigned int i = 0; i < a->samplercnt; i++) {
			struct gltfsampler *s = &a->samplers[i];
			animacc[s->input] = 1;
			animacc[s->output] = 1;
		}
	}

	// Count bytes the animation data will need
	unsigned int animbytes = 0;
	for (unsigned int i = 0; i < g.accessorcnt; i++)
		if (animacc[i] > 0)
			animbytes += g.bufviews[g.accessors[i].bufview].bytelen;

	scene_init(s, g.meshcnt, max(1, g.mtlcnt), max(1, g.camcnt),
	  g.rootcnt, g.nodecnt, tcnt, scnt, animbytes);

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

	unsigned int nodemap[g.nodecnt];
	for (unsigned int i = 0; i < g.rootcnt; i++)
		import_nodes(s, nodemap, g.nodes, g.roots[i]);

	// Copy raw animation data and map anim accessor to data ofs
	unsigned int ofs = 0; // In bytes
	for (unsigned int i = 0; i < g.accessorcnt; i++)
		if (animacc[i] > 0) {
			struct gltfbufview *bv =
			  &g.bufviews[g.accessors[i].bufview];
			memcpy((unsigned char *)s->animdata + ofs,
			  binbuf + bv->byteofs, bv->bytelen);
			animacc[i] = ofs; // Map accessor id to anim data byte ofs
			ofs += bv->bytelen;
		}

	for (unsigned int i = 0; i < g.animcnt; i++)
		import_anim(s, &g.anims[i], &g, nodemap, animacc);

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

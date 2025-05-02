#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "jsmn.h"
#include "gltf.h"

// Output trace
//#define tprintf printf
#define tprintf(...) {}

#ifndef NDEBUG
#define eprintf printf
#else
#define eprintf(...) {}
#endif

// Fixed buffer for temporary string storage
#define SBUF_LEN 1024
char sbuf[SBUF_LEN];

char *strncpyl(char * restrict dst, const char * src,
               unsigned int srclen, unsigned int dstlen)
{
	unsigned int len = srclen < dstlen - 1 ? srclen : dstlen - 1;
	for (unsigned int i = 0; i < len; i++)
		dst[i] = *src++;
	dst[len] = '\0';
	return dst;
}

char *toktostr(const char *s, jsmntok_t *t)
{
	if (t->type == JSMN_STRING || t->type == JSMN_PRIMITIVE)
		return strncpyl(sbuf, s + t->start, t->end - t->start,
		  SBUF_LEN);
	eprintf("Expected token with type string\n");
	return NULL;
}

int jsoneq(const char *json, jsmntok_t *tok, const char *s)
{
	if (tok->type == JSMN_STRING &&
	    (int)strlen(s) == tok->end - tok->start &&
	    strncmp(json + tok->start, s, tok->end - tok->start) == 0)
		return 0;
	return -1;
}

unsigned int dump(const char *s, jsmntok_t *t)
{
	if (t->type == JSMN_PRIMITIVE || t->type == JSMN_STRING) {
		// Print ignored items
		//tprintf("// %.*s\n", t->end - t->start, s + t->start);
		return 1;
	}

	if (t->type == JSMN_OBJECT) {
		unsigned int j = 1;
		for (int i = 0; i < t->size; i++) {
			jsmntok_t *key = t + j;
			j += dump(s, key);
			if (key->size > 0)
				j += dump(s, t + j);
		}
		return j;
	}

	if (t->type == JSMN_ARRAY) {
		unsigned int j = 1;
		for (int i = 0; i < t->size; i++)
			j += dump(s, t + j);
		return j;
	}

	return 0;
}

unsigned int ignore(const char *s, jsmntok_t *t)
{
	unsigned int j = dump(s, t);
	if (t->size > 0)
		j += dump(s, t + j);
	return j;
}

unsigned int read_mtl_extensions(struct gltfmtl *m, const char *s,
                                 jsmntok_t *t, float *emissivestrength)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "KHR_materials_emissive_strength") == 0) {
			if (jsoneq(s, t + j + 2, "emissiveStrength") == 0) {
				*emissivestrength =
				  atof(toktostr(s, t + j + 3));
				tprintf("emissiveStrength: %f\n",
				  *emissivestrength);
				j += 4;
				continue;
			}
		}

		if (jsoneq(s, key, "KHR_materials_transmission") == 0) {
			if (jsoneq(s, t + j + 2, "transmissionFactor") == 0) {
				m->refractive = atof(toktostr(s, t + j + 3));
				tprintf("transmissionFactor (refractive): %f\n",
				  m->refractive);
				j += 4;
				continue;
			}
		}

		if (jsoneq(s, key, "KHR_materials_ior") == 0) {
			if (jsoneq(s, t + j + 2, "ior") == 0) {
				m->ior = atof(toktostr(s, t + j + 3));
				tprintf("ior: %f\n", m->ior);
				j += 4;
				continue;
			}
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_pbr_metallic_roughness(struct gltfmtl *m, const char *s,
                                         jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "baseColorFactor") == 0) {
			if (t[j + 1].type == JSMN_ARRAY && t[j + 1].size == 4) {
				// Just read rgb, ignore alpha (= j + 5)
				m->col[0] = atof(toktostr(s, &t[j + 2]));
				m->col[1] = atof(toktostr(s, &t[j + 3]));
				m->col[2] = atof(toktostr(s, &t[j + 4]));
				j += 6;
				continue;
			} else {
				eprintf("Failed to read baseColorFactor\n");
			}
		}

		if (jsoneq(s, key, "metallicFactor") == 0) {
			if (t[j + 1].type == JSMN_PRIMITIVE) {
				m->metallic = atof(toktostr(s, &t[j + 1]));
				tprintf("metallicFactor: %f\n", m->metallic);
				j += 2;
				continue;
			} else {
				eprintf("Failed to read metallicFactor\n");
			}
		}

		if (jsoneq(s, key, "roughnessFactor") == 0) {
			if (t[j + 1].type == JSMN_PRIMITIVE) {
				m->roughness = atof(toktostr(s, &t[j + 1]));
				tprintf("roughness: %f\n", m->roughness);
				j += 2;
				continue;
			} else {
				eprintf("Failed to read roughness\n");
			}

		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_mtl(struct gltfmtl *m, const char *s, jsmntok_t *t)
{
	m->col[0] = m->col[1] = m->col[2] = 1.0f;
	m->metallic = 0.0f;
	m->roughness = 0.5f;
	m->ior = 1.5f;
	m->refractive = 0.0f;

	// Temporary store only, will use to calc material emission
	float emissivestrength = 0.0f;
	float emissivefac[3] = {1.0f, 1.0f, 1.0f};

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "name") == 0) {
			char *name = toktostr(s, &t[j + 1]);
			strncpyl(m->name, name, t[j + 1].end - t[j + 1].start,
			  NAME_MAX_LEN);
			tprintf("name: %s\n", m->name);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "emissiveFactor") == 0) {
			if (t[j + 1].type == JSMN_ARRAY && t[j + 1].size == 3) {
				emissivefac[0] = atof(toktostr(s, &t[j + 2]));
				emissivefac[1] = atof(toktostr(s, &t[j + 3]));
				emissivefac[2] = atof(toktostr(s, &t[j + 4]));
				j += 5;
				continue;
			} else {
				eprintf("Failed to read emissiveFactor\n");
			}
		}

		if (jsoneq(s, key, "extensions") == 0) {
			j += 1 + read_mtl_extensions(m, s, t + j + 1,
			  &emissivestrength);
			continue;
		}

		if (jsoneq(s, key, "pbrMetallicRoughness") == 0) {
			j += 1 + read_pbr_metallic_roughness(m, s, t + j + 1);
			continue;
		}

		j += ignore(s, key);
	}

	m->emission[0] = emissivefac[0] * emissivestrength;
	m->emission[1] = emissivefac[1] * emissivestrength;
	m->emission[2] = emissivefac[2] * emissivestrength;

	return j;
}

unsigned int read_mtls(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> mtls\n");

	g->mtlcnt = t->size;
	g->mtls = malloc(g->mtlcnt * sizeof(*g->mtls));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> mtl %i\n", i);
		j += read_mtl(&g->mtls[cnt++], s, t + j);
		tprintf("< mtl %i\n", i);
	}

	tprintf("< mtls (total: %i)\n", cnt);

	return j;
}

unsigned int read_node(struct gltfnode *n, const char *s, jsmntok_t *t)
{
	n->meshid = -1;
	n->camid = -1;
	n->scale[0] = n->scale[1] = n->scale[2] = 1.0f;
	n->rot[0] = n->rot[1] = n->rot[2] = 0.0f;
	n->rot[3] = 1.0f;
	n->trans[0] = n->trans[1] = n->trans[2] = 0.0f;
	n->children = NULL;
	n->childcnt = 0;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "name") == 0) {
			char *name = toktostr(s, &t[j + 1]);
			strncpyl(n->name, name, t[j + 1].end - t[j + 1].start,
			  NAME_MAX_LEN);
			tprintf("name: %s\n", name);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "mesh") == 0) {
			n->meshid = atoi(toktostr(s, &t[j + 1]));
			tprintf("mesh: %i\n", n->meshid);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "camera") == 0) {
			n->camid = atoi(toktostr(s, &t[j + 1]));
			tprintf("camera: %i\n", n->camid);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "translation") == 0) {
			if (t[j + 1].type == JSMN_ARRAY && t[j + 1].size == 3) {
				n->trans[0] = atof(toktostr(s, &t[j + 2]));
				n->trans[1] = atof(toktostr(s, &t[j + 3]));
				n->trans[2] = atof(toktostr(s, &t[j + 4]));
				tprintf("translation: %f, %f, %f\n",
				  n->trans[0], n->trans[1], n->trans[2]);
				j += 5;
				continue;
			} else {
				eprintf("Failed to read translation. Expected 3 floats.\n");
			}
		}

		if (jsoneq(s, key, "scale") == 0) {
			if (t[j + 1].type == JSMN_ARRAY && t[j + 1].size == 3) {
				n->scale[0] = atof(toktostr(s, &t[j + 2]));
				n->scale[1] = atof(toktostr(s, &t[j + 3]));
				n->scale[2] = atof(toktostr(s, &t[j + 4]));
				tprintf("scale: %f, %f, %f\n",
				  n->scale[0], n->scale[1], n->scale[2]);
				j += 5;
				continue;
			} else {
				eprintf("Failed to read scale. Expected 3 floats.\n");
			}
		}

		if (jsoneq(s, key, "rotation") == 0) {
			if (t[j + 1].type == JSMN_ARRAY && t[j + 1].size == 4) {
				n->rot[0] = atof(toktostr(s, &t[j + 2]));
				n->rot[1] = atof(toktostr(s, &t[j + 3]));
				n->rot[2] = atof(toktostr(s, &t[j + 4]));
				n->rot[3] = atof(toktostr(s, &t[j + 5]));
				tprintf("rotation: %f, %f, %f, %f\n",
				  n->rot[0], n->rot[1], n->rot[2], n->rot[3]);
				j += 6;
				continue;
			} else {
				eprintf("Failed to read rotation. Expected quaternion (xyzw).\n");
			}
		}

		if (jsoneq(s, key, "children") == 0) {
			if (t[j + 1].type == JSMN_ARRAY) {
				tprintf("children: ");
				n->childcnt = t[j + 1].size;
				n->children =
				  malloc(n->childcnt * sizeof(*n->children));
				for (unsigned int k = 0; k < n->childcnt; k++) {
					n->children[k] = atoi(
					  toktostr(s, &t[j + 2 + k]));
					tprintf("%d ", n->children[k]);
				}
				tprintf("\n");
				j += 2 + n->childcnt;
				continue;
			} else {
				eprintf("Failed to read children.\n");
			}
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_nodes(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> nodes\n");

	g->nodecnt = t->size;
	g->nodes = malloc(g->nodecnt * sizeof(*g->nodes));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> node %i\n", i);

		struct gltfnode *n = &g->nodes[cnt++];
		j += read_node(n, s, t + j);

		if (n->camid >= 0)
			g->camnodecnt++;

		tprintf("< node %i\n", i);
	}

	tprintf("< nodes (total: %i)\n", cnt);

	return j;
}

unsigned int read_cam_perspective(struct gltfcam *c, const char *s,
                                  jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "yfov") == 0) {
			c->vfov = atof(toktostr(s, &t[j + 1]));
			tprintf("yfov: %f\n", c->vfov);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_cam(struct gltfcam *c, const char *s, jsmntok_t *t)
{
	c->vfov = 0.5f;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "name") == 0) {
			char *name = toktostr(s, &t[j + 1]);
			strncpyl(c->name, name, t[j + 1].end - t[j + 1].start,
			  NAME_MAX_LEN);
			tprintf("name: %s\n", c->name);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "perspective") == 0) {
			j += 1 + read_cam_perspective(c, s, t + j + 1);
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_cams(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> cams\n");

	g->camcnt = t->size;
	g->cams = malloc(g->camcnt * sizeof(*g->cams));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> cam %i\n", i);
		j += read_cam(&g->cams[cnt++], s, t + j);
		tprintf("< cam %i\n", i);
	}

	tprintf("< cams (total: %i)\n", cnt);

	return j;
}

unsigned int read_attributes(struct gltfprim *p, const char *s, jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "POSITION") == 0) {
			p->posid = atoi(toktostr(s, &t[j + 1]));
			tprintf("position: %i\n", p->posid);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "NORMAL") == 0) {
			p->nrmid = atoi(toktostr(s, &t[j + 1]));
			tprintf("normal: %i\n", p->nrmid);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_primitive(struct gltfprim *p, const char *s, jsmntok_t *t)
{
	p->indid = -1;
	p->mtlid = -1;
	p->posid = -1;
	p->nrmid = -1;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "attributes") == 0) {
			j += 1 + read_attributes(p, s, t + j + 1);
			continue;
		}

		if (jsoneq(s, key, "indices") == 0) {
			p->indid = atoi(toktostr(s, &t[j + 1]));
			tprintf("indices: %i\n", p->indid);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "material") == 0) {
			p->mtlid = atoi(toktostr(s, &t[j + 1]));
			tprintf("material: %i\n", p->mtlid);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_primitives(struct gltfmesh *m, const char *s, jsmntok_t *t)
{
	tprintf("> primitives\n");

	m->primcnt = t->size;
	m->prims = malloc(m->primcnt * sizeof(*m->prims));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> primitive %i\n", i);
		j += read_primitive(&m->prims[cnt++], s, &t[j]);
		tprintf("< primitive %i\n", i);
	}

	tprintf("< primitives (total: %i)\n", cnt);

	return j;
}

unsigned int read_mesh(struct gltfmesh *m, const char *s, jsmntok_t *t)
{
	m->prims = NULL;
	m->primcnt = 0;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "name") == 0) {
			char *name = toktostr(s, &t[j + 1]);
			strncpyl(m->name, name,
			  t[j + 1].end - t[j + 1].start, NAME_MAX_LEN);
			tprintf("name: %s\n", name);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "primitives") == 0) {
			if (t[j + 1].type == JSMN_ARRAY) {
				j += 1 + read_primitives(m, s, &t[j + 1]);
				continue;
			} else {
				eprintf("Failed to read primitives\n");
			}
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_meshes(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> meshes\n");

	g->meshcnt = t->size;
	g->meshes = malloc(g->meshcnt * sizeof(*g->meshes));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> mesh %i\n", i);
		j += read_mesh(&g->meshes[cnt++], s, t + j);
		tprintf("< mesh %i\n", i);
	}

	tprintf("< meshes (total: %i)\n", cnt);

	return j;
}

unsigned int read_target(struct gltftarget *ta, const char *s, jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "node") == 0) {
			ta->node = atoi(toktostr(s, &t[j + 1]));
			tprintf("node: %i\n", ta->node);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "path") == 0) {
			char *path = toktostr(s, &t[j + 1]);
			if (strstr(path, "translation"))
				ta->path = PA_TRANSLATION;
			else if (strstr(path, "rotation"))
				ta->path = PA_ROTATION;
			else if (strstr(path, "scale"))
				ta->path = PA_SCALE;
			else if (strstr(path, "weights"))
				ta->path = PA_WEIGHTS;
			else
				eprintf("Anim path of unknown type: %s\n",
				  path);
			tprintf("path: %i (%s)\n", ta->path, path);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_channel(struct gltfchan *c, const char *s, jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "target") == 0) {
			j += 1 + read_target(&c->target, s, t + j + 1);
			continue;
		}

		if (jsoneq(s, key, "sampler") == 0) {
			c->sampler = atoi(toktostr(s, &t[j + 1]));
			tprintf("sampler: %i\n", c->sampler);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_channels(struct gltfanim *a, const char *s, jsmntok_t *t)
{
	tprintf("> channels\n");

	a->channelcnt = t->size;
	a->channels = malloc(a->channelcnt * sizeof(*a->channels));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> channel %i\n", i);
		j += read_channel(&a->channels[cnt++], s, &t[j]);
		tprintf("< channel %i\n", i);
	}

	tprintf("< channels (total: %i)\n", cnt);

	return j;
}

unsigned int read_sampler(struct gltfsampler *sa, const char *s, jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "input") == 0) {
			sa->input = atoi(toktostr(s, &t[j + 1]));
			tprintf("input: %i\n", sa->input);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "interpolation") == 0) {
			char *interp = toktostr(s, &t[j + 1]);
			if (strstr(interp, "STEP"))
				sa->interp = IN_STEP;
			else if (strstr(interp, "LINEAR"))
				sa->interp = IN_LINEAR;
			else if (strstr(interp, "CUBICSPLINE"))
				sa->interp = IN_CUBIC;
			else
				eprintf("Interpolation mode of unknown type: %s\n",
				  interp);
			tprintf("interp: %i (%s)\n", sa->interp, interp);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "output") == 0) {
			sa->output = atoi(toktostr(s, &t[j + 1]));
			tprintf("output: %i\n", sa->output);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_samplers(struct gltfanim *a, const char *s, jsmntok_t *t)
{
	tprintf("> samplers\n");

	a->samplercnt = t->size;
	a->samplers = malloc(a->samplercnt * sizeof(*a->samplers));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> sampler %i\n", i);
		j += read_sampler(&a->samplers[cnt++], s, &t[j]);
		tprintf("< sampler %i\n", i);
	}

	tprintf("< samplers (total: %i)\n", cnt);

	return j;
}

unsigned int read_anim(struct gltfanim *a, const char *s, jsmntok_t *t)
{
	a->channels = NULL;
	a->channelcnt = 0;
	a->samplers = NULL;
	a->samplercnt =0;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "name") == 0) {
			char *name = toktostr(s, &t[j + 1]);
			strncpyl(a->name, name,
			  t[j + 1].end - t[j + 1].start, NAME_MAX_LEN);
			tprintf("name: %s\n", name);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "channels") == 0) {
			if (t[j + 1].type == JSMN_ARRAY) {
				j += 1 + read_channels(a, s, &t[j + 1]);
				continue;
			} else {
				eprintf("Failed to read channels\n");
			}
		}

		if (jsoneq(s, key, "samplers") == 0) {
			if (t[j + 1].type == JSMN_ARRAY) {
				j += 1 + read_samplers(a, s, &t[j + 1]);
				continue;
			} else {
				eprintf("Failed to read samplers\n");
			}
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_anims(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> anims\n");

	g->animcnt = t->size;
	g->anims = malloc(g->animcnt * sizeof(*g->anims));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> anim %i\n", i);
		j += read_anim(&g->anims[cnt++], s, t + j);
		tprintf("< anim %i\n", i);
	}

	tprintf("< anims (total: %i)\n", cnt);

	return j;
}

unsigned int read_accessor(struct gltfaccessor *a, const char *s, jsmntok_t *t)
{
	a->bufview = -1;
	a->byteofs = 0;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "bufferView") == 0) {
			a->bufview = atoi(toktostr(s, &t[j + 1]));
			tprintf("bufferView: %i\n", a->bufview);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "count") == 0) {
			a->cnt = atoi(toktostr(s, &t[j + 1]));
			tprintf("count: %i\n", a->cnt);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "byteOffset") == 0) {
			a->byteofs = atoi(toktostr(s, &t[j + 1]));
			tprintf("byteOffset: %i\n", a->byteofs);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "componentType") == 0) {
			a->comptype = atoi(toktostr(s, &t[j + 1]));
			tprintf("componentType: %i\n", a->comptype);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "type") == 0) {
			char *type = toktostr(s, &t[j + 1]);
			if (strstr(type, "VEC3")) {
				a->datatype = DT_VEC3;
			} else if (strstr(type, "VEC4")) {
				a->datatype = DT_VEC4;
			} else if (strstr(type, "SCALAR")) {
				a->datatype = DT_SCALAR;
			} else {
				a->datatype = DT_UNKNOWN;
				eprintf("Accessor with unknown data type: %s\n", type);
			}
			tprintf("type: %i (%s)\n", a->datatype, type);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	if (a->bufview < 0)
		eprintf("Undefined buffer view found. This is not supported.\n");

	return j;
}

unsigned int read_accessors(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> accessors\n");

	g->accessorcnt = t->size;
	g->accessors = malloc(g->accessorcnt * sizeof(*g->accessors));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> accessor %i\n", i);
		j += read_accessor(&g->accessors[cnt++], s, t + j);
		tprintf("< accessor %i\n", i);
	}

	tprintf("< accessors (total: %i)\n", cnt);

	return j;
}

unsigned int read_bufview(struct gltfbufview *b, const char *s, jsmntok_t *t)
{
	b->bytestride = 0;
	b->byteofs = 0;

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "buffer") == 0) {
			b->buf = atoi(toktostr(s, &t[j + 1]));
			tprintf("buffer: %i\n", b->buf);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "byteLength") == 0) {
			b->bytelen = atoi(toktostr(s, &t[j + 1]));
			tprintf("byteLength: %i\n", b->bytelen);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "byteOffset") == 0) {
			b->byteofs = atoi(toktostr(s, &t[j + 1]));
			tprintf("byteOffset: %i\n", b->byteofs);
			j += 2;
			continue;
		}

		if (jsoneq(s, key, "byteStride") == 0) {
			b->bytestride = atoi(toktostr(s, &t[j + 1]));
			tprintf("byteStride: %i\n", b->bytestride);
			j += 2;
			continue;
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_bufviews(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> bufviews\n");

	g->bufviewcnt = t->size;
	g->bufviews = malloc(g->bufviewcnt * sizeof(*g->bufviews));

	unsigned int cnt = 0;
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> bufview %i\n", i);
		j += read_bufview(&g->bufviews[cnt++], s, t + j);
		tprintf("< bufview %i\n", i);
	}

	tprintf("< bufviews (total: %i)\n", cnt);

	return j;
}

unsigned int read_scene(struct gltf *g, const char *s, jsmntok_t *t)
{
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(s, key, "nodes") == 0) {
			if (t[j + 1].type == JSMN_ARRAY) {
				tprintf("root nodes: ");
				g->rootcnt = t[j + 1].size;
				g->roots =
				  malloc(g->rootcnt * sizeof(*g->roots));
				for (unsigned int k = 0; k < g->rootcnt; k++) {
					g->roots[k] = atoi(
					  toktostr(s, &t[j + 2 + k]));
					tprintf("%d ", g->roots[k]);
				}
				tprintf("\n");
				j += 2 + g->rootcnt;
				continue;
			} else {
				eprintf("Failed to read root nodes.\n");
			}
		}

		j += ignore(s, key);
	}

	return j;
}

unsigned int read_scenes(struct gltf *g, const char *s, jsmntok_t *t)
{
	tprintf("> scenes\n");

	if (t->size > 1)
		eprintf("Found %i scenes. Will process only the first and skip all others.\n",
		  t->size);

	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		tprintf("> scene %i\n", i);

		// For now we process the first scene only
		if (i > 0)
			j += ignore(s, t + j);
		else
			j += read_scene(g, s, t + j);

		tprintf("< scene %i\n", i);
	}

	tprintf("< scenes (total: %i)\n", t->size);

	return j;
}

int gltf_init(struct gltf *g, const char *buf)
{
	jsmn_parser parser;
	jsmn_init(&parser);

	// Retrieve number of tokens
	unsigned long long sz = strlen(buf);
	int cnt = jsmn_parse(&parser, buf, sz, NULL, 0);
	if (cnt < 0) {
		eprintf("Something went wrong parsing the token count of the gltf: %i\n", cnt);
		return 1;
	}

	// Parse all tokens
	jsmntok_t *t = malloc(cnt * sizeof(*t));
	jsmn_init(&parser);
	cnt = jsmn_parse(&parser, buf, sz, t, cnt);
	if (cnt < 0) {
		eprintf("Something went wrong parsing the gltf: %i\n", cnt);
		free(t);
		return 1;
	}

	// First token should always be an object
	if (cnt < 1 || t[0].type != JSMN_OBJECT) {
		eprintf("Expected json object as root token in gltf\n");
		free(t);
		return 1;
	}

	// Read token/data
	unsigned int j = 1;
	for (int i = 0; i < t->size; i++) {
		jsmntok_t *key = t + j;

		if (jsoneq(buf, key, "scenes") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_scenes(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "materials") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_mtls(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "nodes") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_nodes(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "cameras") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_cams(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "meshes") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_meshes(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "animations") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_anims(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "accessors") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_accessors(g, buf, t + j);
			continue;
		}

		if (jsoneq(buf, key, "bufferViews") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size > 0) {
			j++;
			j += read_bufviews(g, buf, t + j);
			continue;
		}

		// Buffers. Check that we have a single buffer with mesh data.
		// Something else is not supported at the moment.
		if (jsoneq(buf, key, "buffers") == 0 &&
		  t[j + 1].type == JSMN_ARRAY && t[j + 1].size != 1) {
			eprintf("Expected gltf with one buffer only. Can not process file further.\n");
			free(t);
			return 1;
		}

		j += ignore(buf, key);
	}

	return 0;
}

void gltf_release(struct gltf *g)
{
	for (unsigned int i = 0; i < g->animcnt; i++) {
		free(g->anims[i].channels);
		free(g->anims[i].samplers);
	}

	for (unsigned int i = 0; i < g->meshcnt; i++)
		free(g->meshes[i].prims);

	for (unsigned int i = 0; i < g->nodecnt; i++)
		free(g->nodes[i].children);

	free(g->roots);
	free(g->nodes);
	free(g->mtls);
	free(g->meshes);
	free(g->cams);
	free(g->anims);
	free(g->accessors);
	free(g->bufviews);
}

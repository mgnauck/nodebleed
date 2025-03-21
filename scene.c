#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mat4.h"
#include "scene.h"
#include "util.h"

int findid(struct scene *s, const char *name,
           unsigned int ofs, unsigned int cnt)
{
	const char *p = s->names + (ofs * NAME_MAX_LEN);
	for (unsigned int i=0; i < cnt; i++)
		if (strcmp(p, name) == 0)
			return i;
		else
			p += NAME_MAX_LEN;
	return -1;
}

void setname(struct scene *s, const char *name, unsigned int ofs)
{
	strcpy(s->names + ofs * NAME_MAX_LEN, name);
}

void scene_init(struct scene *s, unsigned int maxmeshes,
                unsigned int maxmtls, unsigned int maxcams,
                unsigned int maxroots, unsigned int maxnodes,
                unsigned int maxtracks, unsigned int maxsamplers,
                unsigned int animdatabytes)
{
	s->meshmax = maxmeshes;
	s->meshcnt = 0;
	s->meshes = emalloc(maxmeshes * sizeof(*s->meshes));

	s->mtlmax = maxmtls;
	s->mtlcnt = 0;
	s->mtls = emalloc(maxmtls * sizeof(*s->mtls));

	s->cammax = maxcams;
	s->camcnt = 0;
	s->cams = emalloc(maxcams * sizeof(*s->cams));

	s->rootmax = maxroots;
	s->rootcnt = 0;
	s->roots = emalloc(maxroots * sizeof(*s->roots));

	s->nodemax = maxnodes;
	s->nodecnt = 0;
	s->nodes = emalloc(maxnodes * sizeof(*s->nodes));
	s->objs = emalloc(maxnodes * sizeof(*s->objs));
	s->transforms = emalloc(maxnodes * sizeof(*s->transforms));

	s->trackmax = maxtracks;
	s->trackcnt = 0;
	s->tracks = emalloc(maxtracks * sizeof(*s->tracks));

	s->samplermax = maxsamplers;
	s->samplercnt = 0;
	s->samplers = emalloc(maxsamplers * sizeof(*s->samplers));

	s->animdata = emalloc(animdatabytes);

	s->names = emalloc((maxmtls + maxcams + maxnodes) *
	                   NAME_MAX_LEN * sizeof(*s->names));
	
	s->currcam = 0;

	s->bgcol = (struct vec3){ 0.0f, 0.0f, 0.0f };

	s->dirty = MESH | MTL | CAM;
}

void scene_release(struct scene *s)
{
	for (unsigned int i=0; i < s->meshcnt; i++) {
		struct mesh *m = &s->meshes[i];
		free(m->mtls);
		free(m->inds);
		free(m->nrms);
		free(m->vrts);
	}

	free(s->names);

	free(s->animdata);
	free(s->samplers);
	s->samplercnt = 0;
	free(s->tracks);
	s->trackcnt = 0;

	free(s->transforms);
	free(s->objs);
	free(s->nodes);
	s->nodecnt = s->nodemax = 0;
	
	free(s->roots);
	s->rootcnt = s->rootmax = 0;

	free(s->cams);
	s->camcnt = s->cammax = 0;

	free(s->mtls);
	s->mtlcnt = s->mtlmax = 0;

	free(s->meshes);
	s->meshcnt = s->meshmax = 0;
}

int scene_acquiremtl(struct scene *s)
{
	return s->mtlcnt < s->mtlmax ? (int)s->mtlcnt++ : -1;
}

struct mtl *scene_initmtl(struct scene *s, unsigned int id,
                  const char *name, struct vec3 col)
{
	assert(strlen(name) < NAME_MAX_LEN);

	struct mtl *m = scene_getmtl(s, id);
	if (m) {
		*m = (struct mtl){
		  .col = col,
		  .metallic = 0.0f,
	 	 .roughness = 0.5f,
	 	 .ior = 1.5f,
		 .flags = 0};

		setname(s, name, /* ofs */ 0);
	}

	return m;
}

struct mtl *scene_getmtl(struct scene *s, unsigned int id)
{
	return id < s->mtlcnt ? &s->mtls[id] : NULL;
}

int scene_findmtl(struct scene *s, const char *name)
{
	return findid(s, name, 0, s->mtlcnt);
}

int scene_acquirecam(struct scene *s)
{
	return s->camcnt < s->cammax ? (int)s->camcnt++ : -1;
}

struct cam *scene_initcam(struct scene *s, unsigned int id,
                  const char *name, float vfov,
                  float focdist, float focangle)
{
	assert(strlen(name) < NAME_MAX_LEN);

	struct cam *c = scene_getcam(s, id);
	if (c) {
		*c = (struct cam){
		  .vfov = vfov,
		  .focdist = focdist,
		  .focangle = focangle};

		setname(s, name, /* ofs */ s->mtlmax);
	}

	return c;
}

struct cam *scene_getcam(struct scene *s, unsigned int id)
{
	return id < s->camcnt ? &s->cams[id] : NULL;
}

int scene_findcam(struct scene *s, const char *name)
{
	return findid(s, name, s->mtlmax, s->camcnt);
}

int scene_acquiremesh(struct scene *s)
{
	return s->meshcnt < s->meshmax ? (int)s->meshcnt++ : -1;
}

struct mesh *scene_initmesh(struct scene *s, unsigned int id,
                   unsigned int vcnt, unsigned int icnt, unsigned int mcnt)
{
	struct mesh *m = scene_getmesh(s, id);
	if (m) {
		m->vrts = emalloc(vcnt * sizeof(*m->vrts));
		m->nrms = emalloc(vcnt * sizeof(*m->nrms));
		m->inds = emalloc(icnt * sizeof(*m->inds));
		m->mtls = emalloc(mcnt * sizeof(*m->mtls));

		m->vcnt = 0; // Nothing added yet
		m->icnt = 0;
		m->mcnt = 0;

		m->flags = 0;
	}

	return m;
}

struct mesh *scene_getmesh(struct scene *s, unsigned int id)
{
	return id < s->meshcnt ? &s->meshes[id] : NULL;
}

int scene_acquirenode(struct scene *s, bool isroot)
{
	if (isroot && s->rootcnt >= s->rootmax)
		return -1;

	int id = s->nodecnt < s->nodemax ? (int)s->nodecnt++ : -1;

	if (isroot && id >= 0)
		s->roots[s->rootcnt++] = id;

	return id;
}

struct node *scene_initnode(struct scene *s, unsigned int id,
                    const char *name, int objid,
                    unsigned int flags, struct vec3 *trans,
                    float rot[4], struct vec3 *scale,
                    unsigned int cofs, unsigned int ccnt)
{
	assert(strlen(name) < NAME_MAX_LEN);

	struct node *n = scene_getnode(s, id);
	if (n) {
		*n = (struct node){.cofs = cofs, .ccnt = ccnt};

		struct obj *o = scene_getobj(s, id);
		*o = (struct obj){.objid = objid, .instid = -1, .flags = flags};

		struct transform *t = scene_gettransform(s, id);
		t->trans = *trans;
		memcpy(t->rot, rot, sizeof(t->rot));
		t->scale = *scale;
		combine_transform(t->loc, trans, rot, scale);
		// Calc global later during node update

		setname(s, name, /* ofs */ s->mtlmax + s->cammax + id);
	}

	return n;
}

struct node *scene_getnode(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ? &s->nodes[id] : NULL;
}

int scene_findnode(struct scene *s, const char *name)
{
	return findid(s, name, s->mtlmax + s->cammax, s->nodecnt);
}

struct obj *scene_getobj(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ? &s->objs[id] : NULL;
}

struct transform *scene_gettransform(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ? &s->transforms[id] : NULL;
}

const char *scene_getnodename(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ?
	  &s->names[(s->mtlmax + s->cammax + id) * NAME_MAX_LEN] : NULL;
}

int scene_acquiretrack(struct scene *s)
{
	return s->trackcnt < s->trackmax ? (int)s->trackcnt++ : -1;
}

struct track *scene_inittrack(struct scene *s, unsigned int id,
                              unsigned int sid, unsigned int nid,
                              enum tgttype tgt)
{
	struct track *t = scene_gettrack(s, id);
	if (t)
		*t = (struct track){.sid = sid, .nid = nid, .tgt = tgt};
	return t;
}

struct track *scene_gettrack(struct scene *s, unsigned int id)
{
	return id < s->trackcnt ? &s->tracks[id] : NULL;
}

int scene_acquiresampler(struct scene *s)
{
	return s->samplercnt < s->samplermax ? (int)s->samplercnt++ : -1;
}

struct sampler *scene_initsampler(struct scene *s, unsigned int id,
                                  unsigned int kcnt, unsigned int kofs,
                                  unsigned int dofs, enum interpmode interp)
{
	struct sampler *sa = scene_getsampler(s, id);
	if (sa)
		*sa = (struct sampler){.kcnt = kcnt, .kofs = kofs,
		  .dofs = dofs, .interp = interp};
	return sa;
}

struct sampler *scene_getsampler(struct scene *s, unsigned int id)
{
	return id < s->samplercnt ? &s->samplers[id] : NULL;
}

unsigned int find_key(float *keys, unsigned int last, unsigned int cnt, float t)
{
	for (int i = last; i < (int)cnt - 1; i++)
		if (t <= keys[i])
			return max(0, i - 1);
	return cnt - 2;
}

void lerp(float *dst, float *s0, float *s1, float t)
{
	dst[0] = (1.0f - t) * s0[0] + t * s1[0];
	dst[1] = (1.0f - t) * s0[1] + t * s1[1];
	dst[2] = (1.0f - t) * s0[2] + t * s1[2];
}

void spher_lerp(float *dst, float *s0, float *s1, float t)
{
	// Rotations only
	float d = s0[0] * s1[0] + s0[1] * s1[1] + s0[2] * s1[2] + s0[3] * s1[3];
	float a = acosf(d);
	float isina = 1.0f / sinf(a);
	float e = sinf(a * (1.0f - t)) * isina;
	// Mul by dot prod sign ensures interpolation along shortest path of
	// great circle of the two quats
	float f = (d / fabsf(d)) * sinf(a * t) * isina;
	dst[0] = e * s0[0] + f * s1[0];
	dst[1] = e * s0[1] + f * s1[1];
	dst[2] = e * s0[2] + f * s1[2];
	dst[3] = e * s0[3] + f * s1[3];
}

void cubic(float *dst, float *s0, float *s1, float t, float dur, unsigned char cnt)
{
	float t2 = t * t;
	float t3 = t2 * t;
	float c0 = 2 * t3 - 3 * t2 + 1;
	float c1 = dur * (t3 - 2 * t2 + t);
	float c2 = -2.0f * t3 + 3 * t2;
	float c3 = dur * (t3 - t2);

	// s0/s1 consists of in-tangent, keyframe and out-tangent
	float *k0 = &s0[cnt]; // key 0
	float *out = &s0[cnt + cnt]; // out-tangent key 0
	float *k1 = &s1[cnt]; // key 1
	float *in = s1; // in-tangent key 1
	for (unsigned char i = 0; i < cnt; i++)
		dst[i] = c0 * k0[i] + c1 * out[i] + c2 * k1[i] + c3 * in[i];
}

void scene_updanims(struct scene *s, float time)
{
	for (unsigned int j = 0; j < s->trackcnt; j++) {
		struct track *tr = scene_gettrack(s, j);
		struct sampler *sa = scene_getsampler(s, tr->sid);
		assert(sa != NULL);

		float *keys = &s->animdata[sa->kofs];
		unsigned int n = find_key(keys, sa->klast, sa->kcnt, time);
		sa->klast = n;

		float t0 = keys[n];
		float t1 = keys[n + 1];
		float tc = max(min(time, t1), t0); // Clamp to key time range
		float du = t1 - t0; // Duration
		float t = (tc - t0) / du;

		unsigned char comp = tr->tgt == TGT_ROT ? /* Quat */ 4 : 3;
		unsigned char elem = sa->interp == IM_CUBIC ? /* Tangents */ 3 : 1;
		float *v0 = &s->animdata[sa->dofs + elem * comp * n];
		float *v1 = &s->animdata[sa->dofs + elem * comp * (n + 1)];
		float v[comp];

		switch (sa->interp) {
		case IM_STEP:
			for (unsigned char i = 0; i < comp; i++)
				v[i] = v0[i];
			break;
		case IM_LINEAR:
			if (comp == 4)
				spher_lerp(v, v0, v1, t);
			else
				lerp(v, v0, v1, t);
			break;
		case IM_CUBIC:
			cubic(v, v0, v1, t, du, comp);
			if (comp == 4) {
				// Normalize in case of rotation
				float len = 1.0f / sqrtf(v[0] * v[0] +
				  v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
				v[0] *= len;
				v[1] *= len;
				v[2] *= len;
				v[3] *= len;
			}
			break;
		}

		struct transform *tf = scene_gettransform(s, tr->nid);
		assert(tf != NULL);

		float *dst;
		switch (tr->tgt) {
		case TGT_TRANS:
			dst = &tf->trans.x;
			break;
		case TGT_ROT:
			dst = tf->rot;
			break;
		case TGT_SCALE:
			dst = &tf->scale.x;
			break;
		}

		for (unsigned int i = 0; i < comp; i++)
			dst[i] = v[i];

		// Skip local transform update if next track targets same node
		if (j < s->trackcnt - 1 &&
		  scene_gettrack(s, j + 1)->nid == tr->nid)
			continue;

		combine_transform(tf->loc, &tf->trans, tf->rot, &tf->scale);
	}
}

void scene_updtransforms(struct scene *s)
{
#define STACK_SIZE  64
	unsigned int stack[STACK_SIZE];
	unsigned int spos;
	struct node *nodes = s->nodes;
	struct transform *transforms = s->transforms;
	for (unsigned int j = 0; j < s->rootcnt; j++) {
		unsigned int rid = s->roots[j];
		stack[0] = rid;
		spos = 1;
		mat4_cpy(transforms[rid].glob, transforms[rid].loc);
		// TODO Profile traversal, each node is fetched twice
		while (spos > 0) {
			unsigned int nid = stack[--spos];
			struct node *n = &nodes[nid];
			struct transform *nt = &transforms[nid];
			unsigned int cid = n->cofs;
			for (unsigned int i = 0; i < n->ccnt; i++) {
				struct node *c = &nodes[cid];
				struct transform *ct = &transforms[cid];
				mat4_mul(ct->glob, nt->glob, ct->loc);
				assert(spos < STACK_SIZE);
				stack[spos++] = cid++;
			}
		}
	}
}

void calc_cam(struct cam *c, float trans[16])
{
	c->eye = mat4_gettrans(trans);
	c->fwd = vec3_unit(mat4_muldir(trans, (struct vec3){0.0f, 0.0f, 1.0f}));
	c->ri = vec3_unit(vec3_cross((struct vec3){0.0f, 1.0f, 0.0f}, c->fwd));
	c->up = vec3_unit(vec3_cross(c->fwd, c->ri));
}

void scene_updcams(struct scene *s)
{
	for (unsigned int i = 0; i < s->camcnt; i++) {
		struct cam *c = scene_getcam(s, i);
		calc_cam(c, scene_gettransform(s, c->nodeid)->glob);
	}
}

void combine_transform(float dst[16], struct vec3 *trans, float rot[4],
                       struct vec3 *scale)
{
	float mscale[16], mrot[16], mtrans[16];
	mat4_scale(mscale, *scale);
	mat4_fromquat(mrot, rot[0], rot[1], rot[2], rot[3]);
	mat4_trans(mtrans, *trans);
	mat4_mul(dst, mrot, mscale);
	mat4_mul(dst, mtrans, dst);
}

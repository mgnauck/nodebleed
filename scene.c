#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
                unsigned int maxroots, unsigned int maxnodes)
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
	s->objdata = emalloc(maxnodes * sizeof(*s->objdata));
	s->transforms = emalloc(maxnodes * sizeof(*s->transforms));

	unsigned int maxnames = maxmtls + maxcams + maxnodes; 
	s->names = emalloc(maxnames * NAME_MAX_LEN * sizeof(*s->names));
	
	s->currcam = 0;

	s->bgcol = (struct vec3){ 0.0f, 0.0f, 0.0f };

	s->dirty = DF_MESH | DF_MTL | DF_CAM;
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

	free(s->transforms);
	free(s->objdata);
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
	assert(id < s->mtlcnt);
	assert(strlen(name) < NAME_MAX_LEN);

	struct mtl *m = &s->mtls[id];
	*m = (struct mtl){
	  .col = col,
	  .metallic = 0.0f,
	  .roughness = 0.5f,
	  .ior = 1.5f,
	  .flags = 0x0};

	setname(s, name, 0);

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
                  const char *name, float vertfov,
                  float focdist, float focangle)
{
	assert(id < s->camcnt);
	assert(strlen(name) < NAME_MAX_LEN);

	struct cam *c = &s->cams[id];
	*c = (struct cam){
	  .vertfov = vertfov,
	  .focdist = focdist,
	  .focangle = focangle};

	setname(s, name, s->mtlmax);

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
	assert(id < s->meshcnt);

	struct mesh *m = &s->meshes[id];

	m->vrts = emalloc(vcnt * sizeof(*m->vrts));
	m->nrms = emalloc(vcnt * sizeof(*m->nrms));
	m->inds = emalloc(icnt * sizeof(*m->inds));
	m->mtls = emalloc(mcnt * sizeof(*m->mtls));

	m->vcnt = 0; // Nothing added yet
	m->icnt = 0;
	m->mcnt = 0;

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

void scene_initnode(struct scene *s, unsigned int id,
                    const char *name, int objid,
                    unsigned int flags, float local[16],
                    unsigned int cofs, unsigned int ccnt)
{
	assert(id < s->nodecnt);
	assert(strlen(name) < NAME_MAX_LEN);

	s->nodes[id] = (struct node){
	  .id = id, .cofs = cofs, .ccnt = ccnt};

	s->objdata[id] = (struct objdata){
	  .objid = objid, .flags = flags};

	struct transform *t = &s->transforms[id];
	memcpy(t->loc, local, sizeof(t->loc));

	setname(s, name, s->mtlmax + s->cammax);
}

int scene_findnode(struct scene *s, const char *name)
{
	return findid(s, name, s->mtlmax + s->cammax, s->nodecnt);
}

struct objdata *scene_getobjdata(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ? &s->objdata[id] : NULL;
}

struct transform *scene_gettransform(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ? &s->transforms[id] : NULL;
}

const char *scene_getnodename(struct scene *s, unsigned int id)
{
	return id < s->nodecnt ?
	  &s->names[(s->mtlmax + s->cammax + id) * NAME_MAX_LEN] :
	  NULL;
}

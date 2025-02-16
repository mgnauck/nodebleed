#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "scene.h"
#include "util.h"

void mtl_init(struct mtl *m, const char *name, struct vec3 c)
{
	*m = (struct mtl){
	  .col = c,
	  .metallic = 0.0f,
	  .roughness = 0.5f,
	  .ior = 1.5f,
	  .flags = 0x0};

	strcpy(m->name, name);
}

void cam_init(struct cam *c, const char *name, float vertfov,
              float focdist, float focangle)
{
	*c = (struct cam){
	  .vertfov = vertfov,
	  .focdist = focdist,
	  .focangle = focangle};

	strcpy(c->name, name);
}

void mesh_init(struct mesh *m,
               unsigned int vcnt, unsigned int icnt, unsigned int mcnt)
{
	m->vrts = emalloc(vcnt * sizeof(*m->vrts));
	m->nrms = emalloc(vcnt * sizeof(*m->nrms));
	m->inds = emalloc(icnt * sizeof(*m->inds));
	m->mtls = emalloc(mcnt * sizeof(*m->mtls));

	m->vcnt = 0; // Nothing added yet
	m->icnt = 0;
	m->mcnt = 0;
}

void mesh_release(struct mesh *m)
{
	free(m->mtls);
	free(m->inds);
	free(m->nrms);
	free(m->vrts);

	m->vcnt = 0;
	m->icnt = 0;
	m->mcnt = 0;
}

void scene_init(struct scene *s, unsigned int maxmeshes,
                unsigned int maxmtls, unsigned int maxcams,
                unsigned int maxrnodes, unsigned int maxsnodes)
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

	s->rnodemax = maxrnodes;
	s->rnodecnt = 0;
	s->rnodes = emalloc(maxrnodes * sizeof(*s->rnodes));

	s->snodemax = maxsnodes;
	s->snodecnt = 0;
	s->snodes = emalloc(maxsnodes * sizeof(*s->snodes));
	s->nodenames = emalloc(maxsnodes * NAME_MAX_LEN * sizeof(*s->nodenames));
	s->objdata = emalloc(maxsnodes * sizeof(*s->objdata));
	s->transforms = emalloc(maxsnodes * sizeof(*s->transforms));
	
	s->currcam = 0;

	s->bgcol = (struct vec3){ 0.0f, 0.0f, 0.0f };

	s->dirty = DF_MESH | DF_MTL | DF_CAM;
}

void scene_release(struct scene *s)
{
	free(s->transforms);
	free(s->objdata);
	free(s->nodenames);
	free(s->snodes);
	s->snodecnt = s->snodemax = 0;
	
	free(s->rnodes);
	s->rnodecnt = s->rnodemax = 0;

	free(s->cams);
	s->camcnt = s->cammax = 0;

	free(s->mtls);
	s->mtlcnt = s->mtlmax = 0;

	free(s->meshes);
	s->meshcnt = s->meshmax = 0;
}

int scene_acquiremesh(struct scene *s)
{
	return s->meshcnt < s->meshmax ? (int)s->meshcnt++ : -1;
}

struct mesh *scene_getmesh(struct scene *s, unsigned int id)
{
	return id < s->meshcnt ? &s->meshes[id] : NULL;
}

int scene_acquiremtl(struct scene *s)
{
	return s->mtlcnt < s->mtlmax ? (int)s->mtlcnt++ : -1;
}

struct mtl *scene_getmtl(struct scene *s, unsigned int id)
{
	return id < s->mtlcnt ? &s->mtls[id] : NULL;
}

struct mtl *scene_findmtl(struct scene *s, const char *name)
{
	struct mtl *m = s->mtls;
	for (unsigned int i = 0; i < s->mtlcnt; i++)
		if (strcmp(m->name, name) == 0)
			return m;
		else
			m++;
	return NULL;
}

int scene_acquirecam(struct scene *s)
{
	return s->camcnt < s->cammax ? (int)s->camcnt++ : -1;
}

struct cam *scene_getcam(struct scene *s, unsigned int id)
{
	return id < s->camcnt ? &s->cams[id] : NULL;
}

struct cam *scene_findcam(struct scene *s, const char *name)
{
	struct cam *c = s->cams;
	for (unsigned int i = 0; i < s->camcnt; i++)
		if (strcmp(c->name, name) == 0)
			return c;
		else
			c++;
	return NULL;
}

int scene_initrnode(struct scene *s, unsigned int snodeid)
{
	if (s->rnodecnt < s->rnodemax) {
		s->rnodes[s->rnodecnt] = snodeid;
		return s->rnodecnt++;
	}

	return -1;
}

int scene_acquiresnode(struct scene *s)
{
	return s->snodecnt < s->snodemax ? (int)s->snodecnt++ : -1;
}

void scene_initsnode(struct scene *s, unsigned int snodeid,
                     const char *name, int objid, unsigned int flags,
                     float local[16], unsigned int cofs, unsigned int ccnt)
{
	assert(snodeid < s->snodecnt);
	assert(strlen(name) < NAME_MAX_LEN);

	s->snodes[snodeid] =
	  (struct snode){.id = snodeid, .cofs = cofs, .ccnt = ccnt};

	memcpy(&s->nodenames[snodeid * NAME_MAX_LEN], name, strlen(name));

	s->objdata[snodeid] = (struct objdata){.objid = objid, .flags = flags};

	struct transform *t = &s->transforms[snodeid];
	memcpy(t->loc, local, sizeof(t->loc));
}

int scene_findsnode(struct scene *s, const char *name)
{
	char *p = s->nodenames;
	for (unsigned int i = 0; i < s->snodecnt; i++) {
		if (strcmp(p, name) == 0)
			return i;
		p += NAME_MAX_LEN;
	}
	return -1;
}

struct objdata *scene_getobjdata(struct scene *s, unsigned int snodeid)
{
	return snodeid < s->snodecnt ? &s->objdata[snodeid] : NULL;
}

struct transform *scene_gettransform(struct scene *s, unsigned int snodeid)
{
	return snodeid < s->snodecnt ? &s->transforms[snodeid] : NULL;
}

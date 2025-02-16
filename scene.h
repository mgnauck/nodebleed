#ifndef SCENE_H
#define SCENE_H

#include "vec3.h"

// TODO
// mesh primitive generation
// mesh subdivision
// scene animation data/handling
// scene convert scene data to device data

#define NAME_MAX_LEN  128

enum dflags {
	DF_MESH = 0x1,
	DF_CAM  = 0x2,
	DF_MTL  = 0x4,
};

enum mtlflags {
	MF_REFRACTIVE = 0x1,
	MF_EMISSIVE   = 0x2,
};

enum nflags {
	// Obj types
	SN_MESH       = 0x01,
	SN_CAM        = 0x02,
	// States
	SN_DISABLED   = 0x0f,
	SN_STATIC     = 0x1f,
	SN_INVISIBLE  = 0x2f,
	SN_NO_SHADOW  = 0x3f,
	SN_EMISSIVE   = 0x4f,
};

struct mtl {
	struct vec3   col; // Diffuse col of non-metallic/specular col of metallic
	float         metallic; // Appearance range from dielectric to conductor (0 - 1)
	float         roughness; // Perfect reflection to completely diffuse diffuse (0 - 1)
	float         ior; // Index of refraction
	unsigned int  flags; // As per mtlflags
	char          name[NAME_MAX_LEN];
};

struct cam {
	float         vertfov;
	float         focdist;
	float         focangle;
	unsigned int  nodeid;
	char          name[NAME_MAX_LEN];
};

struct mtlinf {
	unsigned int  mtlid; // Material id
	unsigned int  triofs; // Triangle offset this mtl becomes active
	unsigned int  tricnt; // Triangle count this mtl is used
};

struct mesh {
	unsigned int   vcnt;
	struct vec3    *vrts;
	struct vec3    *nrms;

	unsigned int   icnt;
	unsigned int   *inds;

	unsigned int   mcnt;
	struct mtlinf  *mtls;
};

struct snode {
	unsigned int  id; // Index into transforms/objdata/nodenames
	unsigned int  cofs; // Child ofs
	unsigned int  ccnt; // Child cnt
};

struct transform {
	float  loc[16];
	float  glob[16];
};

struct objdata {
	int           objid;
	unsigned int  flags;
};

struct scene {
	unsigned int      meshmax;
	unsigned int      meshcnt;
	struct mesh       *meshes;

	unsigned int      mtlmax;
	unsigned int      mtlcnt;
	struct mtl        *mtls;

	unsigned int      cammax;
	unsigned int      camcnt;
	struct cam        *cams;
	
	unsigned int      rnodemax;
	unsigned int      rnodecnt;
	unsigned int      *rnodes;
	
	unsigned int      snodemax;
	unsigned int      snodecnt;
	struct snode      *snodes;
	struct transform  *transforms;
	struct objdata    *objdata;
	char              *nodenames;

	unsigned int      currcam;

	struct vec3       bgcol;

	unsigned int      dirty;
};

void              mtl_init(struct mtl *m, const char *name, struct vec3 col);

void              cam_init(struct cam *c, const char *name, float vertfov,
                           float focdist, float focangle);
              
void              mesh_init(struct mesh *m,
                            unsigned int vcnt, unsigned int icnt,
                            unsigned int mcnt);
void              mesh_release(struct mesh *m);

void              scene_init(struct scene *s, unsigned int maxmeshes,
                             unsigned int maxmtls, unsigned int maxcams,
                             unsigned int maxrnodes, unsigned int maxsnodes);
void              scene_release(struct scene *s);

int               scene_acquiremesh(struct scene *s);
struct mesh       *scene_getmesh(struct scene *s, unsigned int id);

int               scene_acquiremtl(struct scene *s);
struct mtl        *scene_getmtl(struct scene *s, unsigned int id);
struct mtl        *scene_findmtl(struct scene *s, const char *name);

int               scene_acquirecam(struct scene *s);
struct cam        *scene_getcam(struct scene *s, unsigned int id);
struct cam        *scene_findcam(struct scene *s, const char *name);

int               scene_initrnode(struct scene *s, unsigned int snodeid);

int               scene_acquiresnode(struct scene *s);
void              scene_initsnode(struct scene *s, unsigned int snodeid, 
                                  const char *name, int objid,
                                  unsigned int flags, float local[16],
                                  unsigned int cofs, unsigned int ccnt);
int               scene_findsnode(struct scene *s, const char *name);
struct objdata    *scene_getobjdata(struct scene *s, unsigned int snodeid);
struct transform  *scene_gettransform(struct scene *s, unsigned int snodeid);

#endif

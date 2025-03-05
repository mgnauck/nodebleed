#ifndef SCENE_H
#define SCENE_H

#include <stdbool.h>
#include "vec3.h"

#define NAME_MAX_LEN  128

enum flags {
	// Types
	MESH         = 0x0001,
	CAM          = 0x0002,
	MTL          = 0x0004,

	// State/behaviour
	DISABLED    = 0x0100,
	DYNAMIC     = 0x0200,
	INVISIBLE   = 0x0400,
	NOSHADOW    = 0x0800,
	EMISSIVE    = 0x1000,
	REFRACTIVE  = 0x2000,
};

struct mtl {
	struct vec3   col; // Diff col of non-metallic/specular col of metallic
	float         metallic; // Range from dielectric to conductor (0 - 1)
	float         roughness; // Perfect refl to completely diffuse (0 - 1)
	float         ior; // Index of refraction
	unsigned int  flags;
};

struct cam {
	struct vec3   eye;
	float         vfov; // In degree
	struct vec3   fwd;
	float         focdist;
	struct vec3   ri;
	float         focangle;
	struct vec3   up;
	unsigned int  nodeid;
};

struct mtlref {
	unsigned int  mtlid;
	unsigned int  tricnt; // Triangle count this mtl is used
};

struct mesh {
	unsigned int   vcnt;
	struct vec3    *vrts;
	struct vec3    *nrms;

	unsigned int   icnt;
	unsigned int   *inds;

	unsigned int   mcnt;
	struct mtlref  *mtls;

	unsigned int   flags;
};

struct node {
	unsigned int  id; // TODO Remove id to self? 
	unsigned int  cofs; // Child ofs
	unsigned int  ccnt; // Child cnt
};

struct transform {
	float  loc[16];
	float  glob[16];
};

struct obj {
	int           objid;
	int           instid; // TODO Remove?
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
	
	unsigned int      rootmax;
	unsigned int      rootcnt;
	unsigned int      *roots;
	
	unsigned int      nodemax;
	unsigned int      nodecnt;
	struct node       *nodes;
	struct transform  *transforms;
	struct obj        *objs;

	char              *names;

	unsigned int      currcam;

	struct vec3       bgcol;

	unsigned int      dirty;
};

void              scene_init(struct scene *s, unsigned int maxmeshes,
                             unsigned int maxmtls, unsigned int maxcams,
                             unsigned int maxrnodes, unsigned int maxsnodes);
void              scene_release(struct scene *s);

int               scene_acquiremtl(struct scene *s);
struct mtl        *scene_initmtl(struct scene *s, unsigned int id,
                                 const char *name, struct vec3 col);
struct mtl        *scene_getmtl(struct scene *s, unsigned int id);
int               scene_findmtl(struct scene *s, const char *name);

int               scene_acquirecam(struct scene *s);
struct cam        *scene_initcam(struct scene *s, unsigned int id,
                                 const char *name, float vfov, float focdist,
                                 float focangle);
struct cam        *scene_getcam(struct scene *s, unsigned int id);
int               scene_findcam(struct scene *s, const char *name);

int               scene_acquiremesh(struct scene *s);
struct mesh       *scene_initmesh(struct scene *s, unsigned int id,
                                  unsigned int vcnt, unsigned int icnt,
                                  unsigned int mcnt);
struct mesh       *scene_getmesh(struct scene *s, unsigned int id);

int               scene_acquirenode(struct scene *s, bool isroot);
struct node       *scene_initnode(struct scene *s, unsigned int id,
                                 const char *name, int objid,
                                 unsigned int flags, float local[16],
                                 unsigned int cofs, unsigned int ccnt);
struct node       *scene_getnode(struct scene *s, unsigned int id);
int               scene_findnode(struct scene *s, const char *name);
struct obj        *scene_getobj(struct scene *s, unsigned int id);
struct transform  *scene_gettransform(struct scene *s, unsigned int id);
const char        *scene_getnodename(struct scene *s, unsigned int id);

void              scene_updtransforms(struct scene *s);
void              scene_updcams(struct scene *s);

#endif

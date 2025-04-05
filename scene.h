#ifndef SCENE_H
#define SCENE_H

#include <stdbool.h>
#include "types.h"
#include "vec3.h"

#define NAME_MAX_LEN  128

struct mtl {
	struct vec3   col; // Non-metalic diff or metallic spec col
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
	unsigned int  nid; // Node id
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

struct tfcomp { // Transform components, 48 byte
	float        rot[4]; // Quat
	struct vec3  trans;
	float        pad0;
	struct vec3  scale;
	float        pad1;
};

struct tfmat { // Transform matrices, 128 byte
	float        loc[16];
	float        glob[16];
};

struct obj { // 16 byte
	int           objid; // Ref to mesh/cam
	int           instid; // Ref to rinst
	unsigned int  flags;
	unsigned int  pad0;
};

enum tgttype { // Target type
	TGT_TRANS,
	TGT_ROT, // 4 floats (quat)
	TGT_SCALE
};

struct track {
	unsigned int  sid; // Sampler id
	unsigned int  nid; // Node id
	unsigned int  key; // Last used key
	enum tgttype  tgt;
};

enum interpmode {
	IM_STEP,
	IM_LINEAR,
	IM_CUBIC // Additional in+out tangent
};

struct sampler {
	unsigned int     kcnt; // Keyframe count
	unsigned int     kofs; // Offset to keyframes (timestamps)
	unsigned int     dofs; // Offset to keyframe data
	enum interpmode  interp;
};

struct scene {
	unsigned int         meshmax;
	unsigned int         meshcnt;
	struct mesh          *meshes;

	unsigned int         mtlmax;
	unsigned int         mtlcnt;
	struct mtl           *mtls;

	unsigned int         cammax;
	unsigned int         camcnt;
	struct cam           *cams;
	
	unsigned int         nodemax;
	unsigned int         nodecnt;
	int                  *prnts;
	struct tfcomp        *tfcomps;
	struct tfmat         *tfmats;
	struct obj           *objs;

	unsigned int         trackmax;
	unsigned int         trackcnt;
	struct track         *tracks;

	unsigned int         samplermax;
	unsigned int         samplercnt;
	struct sampler       *samplers;

	float                *animdata;

	char                 *names;

	unsigned int         currcam;

	struct vec3          bgcol;

	unsigned int         dirty;
};

void        scene_init(struct scene *s, unsigned int maxmeshes,
                       unsigned int maxmtls, unsigned int maxcams,
                       unsigned int maxnodes, unsigned int maxtracks,
                       unsigned int maxsamplers, unsigned int animdatabytes);
void        scene_release(struct scene *s);

int         scene_initmtl(struct scene *s, const char *name, struct vec3 col);
int         scene_findmtl(struct scene *s, const char *name);

int         scene_initcam(struct scene *s, const char *name, float vfov,
                          float focdist, float focangle, int nodeid);
int         scene_findcam(struct scene *s, const char *name);

int         scene_initmesh(struct scene *s, unsigned int vcnt, unsigned int icnt,
                           unsigned int mcnt);

int         scene_initnode(struct scene *s, const char *name,
                           int prntid, int objid, unsigned int flags,
                           struct vec3 *trans, float rot[4],
                           struct vec3 *scale);
int         scene_findnode(struct scene *s, const char *name);
const char  *scene_getnodename(struct scene *s, unsigned int id);

int         scene_inittrack(struct scene *s, unsigned int sid, unsigned int nid,
                            enum tgttype tgt);

int         scene_initsampler(struct scene *s, unsigned int kcnt,
                              unsigned int kofs, unsigned int dofs,
                              enum interpmode interp);

void        scene_updanims(struct scene *s, float time);
void        scene_updtransforms(struct scene *s);
void        scene_updcams(struct scene *s);

#endif

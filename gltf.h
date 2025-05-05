#ifndef GLTF_H
#define GLTF_H

// Limitations:
// - Ignoring asset and extensions information
// - Reading only the first scene
// - Ignoring skins and morph targets
// - Ignoring texture coords and textures
// - Not handling sparse accessors
// - Expecting only one external (i.e. not embedded) binary buffer
// - Supporting only a limited set of buffer data types

#define NAME_MAX_LEN  128

enum gltfdatatype {
	DT_SCALAR,
	DT_VEC2,
	DT_VEC3,
	DT_VEC4,
	DT_MAT2,
	DT_MAT3,
	DT_MAT4
};

struct gltfmtl {
	float  col[3];
	float  metallic;
	float  roughness;
	float  ior;
	float  refractive;
	float  emission[3];
	char   name[NAME_MAX_LEN];
	// Ignoring everthing else
};

struct gltfcam {
	float  vfov; // In radian
	char   name[NAME_MAX_LEN];
	// Ignoring everthing else
};

struct gltfprim {
	int  posid;
	int  nrmid;
	int  indid;
	int  mtlid;
};

struct gltfmesh {
	struct gltfprim  *prims;
	unsigned int     primcnt;
	char             name[NAME_MAX_LEN];
};

enum gltfpath {
	PA_TRANSLATION,
	PA_ROTATION,
	PA_SCALE,
	PA_WEIGHTS
};

struct gltftarget {
	unsigned int   node;
	enum gltfpath  path;
};

struct gltfchan {
	unsigned int       sampler;
	struct gltftarget  target;
};

enum gltfinterp {
	IN_STEP,
	IN_LINEAR,
	IN_CUBIC
};

struct gltfsampler {
	unsigned int     input; // Accessor to time values
	enum gltfinterp  interp;
	unsigned int     output; // Accessor to keyframes
};

struct gltfanim {
	struct gltfchan     *channels;
	unsigned int        channelcnt;
	struct gltfsampler  *samplers;
	unsigned int        samplercnt;
	char                name[NAME_MAX_LEN];
};

struct gltfaccessor {
	int                bufview; // TODO: When undefined, data is 0
	unsigned int       cnt;
	unsigned int       byteofs;
	unsigned int       comptype;
	enum gltfdatatype  datatype;
};

struct gltfbufview {
	unsigned int  buf;
	unsigned int  bytelen;
	unsigned int  byteofs;
	unsigned int  bytestride;
};

struct gltfnode {
	int           meshid;
	int           camid;
	float         scale[3];
	float         rot[4]; // Quaternion xyzw
	float         trans[3];
	unsigned int  *children;
	unsigned int  childcnt;
	char          name[NAME_MAX_LEN];
};

struct gltf {
	unsigned int         *roots;
	unsigned int         rootcnt;
	struct gltfnode      *nodes;
	unsigned int         nodecnt;
	unsigned int         camnodecnt;
	struct gltfmtl       *mtls;
	unsigned int         mtlcnt;
	struct gltfmesh      *meshes;
	unsigned int         meshcnt;
	struct gltfcam       *cams;
	unsigned int         camcnt;
	struct gltfanim      *anims;
	unsigned int         animcnt;
	struct gltfaccessor  *accessors;
	unsigned int         accessorcnt;
	struct gltfbufview   *bufviews;
	unsigned int         bufviewcnt;
};

int   gltf_init(struct gltf *g, const char *buf);
void  gltf_release(struct gltf *g);

#endif

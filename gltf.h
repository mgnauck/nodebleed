#ifndef GLTF_H
#define GLTF_H

// Limitations:
// - Ignoring all assets and extensions
// - Expecting only one scene
// - Ignoring animation and skinning data
// - Ignoring texture coords and textures
// - Expecting only one external binary buffer (i.e. not embedded)
// - Only a small set of buffer data types are supported

#define NAME_MAX_LEN  128

enum gltfdatatype {
	DT_SCALAR,
	DT_VEC3,
	DT_UNKNOWN // All the unsupported ones
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

struct gltfaccessor {
	int                bufview; // TODO: When undefined, data should be 0
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
	unsigned int  ccnt;
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
	struct gltfaccessor  *accessors;
	unsigned int         accessorcnt;
	struct gltfbufview   *bufviews;
	unsigned int         bufviewcnt;
};

int   gltf_init(struct gltf *g, const char *buf);
void  gltf_release(struct gltf *g);

#endif

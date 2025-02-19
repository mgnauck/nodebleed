#ifndef RENDERDATA_H
#define RENDERDATA_H

#include <stdint.h>
#include "vec3.h"

struct rd_tpos { // Triangle positions
	struct vec3  v0;
	float        pad0;
	struct vec3  v1;
	float        pad1;
	struct vec3  v2;
	float        pad2;
};

struct rd_tnrm { // Triangle normals
	struct vec3  n0;
	uint32_t     mtlid;
	struct vec3  n1;
	uint32_t     ltriid;
	struct vec3  n2;
	float        pad0;
};

// TODO struct rd_ltri

struct rd_inst {
};



#endif

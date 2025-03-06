#ifndef TYPES_H
#define TYPES_H

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

#endif

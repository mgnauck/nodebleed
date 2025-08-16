#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "binimp.h"
#include "mat4.h"
#include "scene.h"
#include "util.h"

#define MAX_STR_LEN 256

#ifndef NDEBUG
#define dprintf printf
#else
#define dprintf(...) {}
#endif

#ifndef NDEBUG
#define abort(...) { printf(__VA_ARGS__); exit(1); }
#else
#define abort(...) exit(1);
#endif

static void *mmread(char *relpathname, unsigned long long *sz)
{
	int fd = open(relpathname, O_RDONLY);
	if (fd < 0)
		abort("Failed to open %s\n", relpathname);

	struct stat st = {0};
	if (fstat(fd, &st) < 0)
		abort("Failed to retrieve file stat %s\n", relpathname);

	//dprintf("file: %s, sz: %ld\n", relpathname, st.st_size);

	void *buf = mmap(NULL, st.st_size, PROT_READ | PROT_WRITE, MAP_PRIVATE,
	  fd, 0);
	if ((long long)buf < 0)
		abort("Failed to map %ld bytes\n", st.st_size);

	if (close(fd) < 0)
		abort("Close %s failed\n", relpathname);

	*sz = st.st_size;
	return buf;
}

/*
static unsigned char *rduchar(unsigned char *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

static unsigned char *rdushort(unsigned short *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}
*/

static unsigned char *rduint(unsigned int *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

/*
static unsigned char *rdint(int *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}
*/

static unsigned char *rdfloat(float *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

/*
static unsigned char *rdvec3(struct vec3 *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}
*/

static unsigned char *rdstr(char *s, unsigned char *b)
{
	unsigned char *p = b;
	while (*p && p - b < MAX_STR_LEN - 1)
		*s++ = *(char *)p++;
	*s = '\0';
	if (*p)
		abort("Max string len reached without reading \\0");
	return ++p;
}

unsigned char *rdnode(struct scene *s, unsigned int *nodeid, unsigned char *b)
{
	printf("- node ----\n");

	unsigned int dummy;
	b = rduint(&dummy, b); // node id
	printf("nodeid: %d\n", dummy);

	unsigned int pid;
	b = rduint(&pid, b);
	printf("prntid: %d\n", pid);

	b = rduint(&dummy, b);
	printf("prntofs: %d\n", dummy);

	char name[MAX_STR_LEN];
	b = rdstr(name, b);
	printf("name: %s\n", name);

	unsigned int type;
	b = rduint(&type, b);
	printf("type: %d\n", type);

	unsigned int objid;
	b = rduint(&objid, b);
	printf("objid: %d\n", objid);

	b = rduint(&dummy, b); // obj ofs
	printf("obj ofs: %d\n", dummy);

	unsigned int transtid;
	b = rduint(&transtid, b); 
	printf("trans tid: %d\n", transtid);

	b = rduint(&dummy, b); // trans track ofs
	printf("trans tofs: %d\n", dummy);

	unsigned int rottid;
	b = rduint(&rottid, b); 
	printf("rot tid: %d\n", rottid);

	b = rduint(&dummy, b); // rot track ofs
	printf("rot tofs: %d\n", dummy);

	unsigned int scaletid;
	b = rduint(&scaletid, b); 
	printf("scale tid: %d\n", scaletid);

	b = rduint(&dummy, b); // scale track ofs
	printf("scale tofs: %d\n", dummy);

	unsigned int flags = 0;

	if (type == 3)
		setflags(&flags, CAM);
	else if (type == 1)
		setflags(&flags, MESH /* | s->meshes[objid].flags */);
	else if (type == 0)
		; // Root or unused?
	else
		abort("Unknown node type");

	// TEMP
	struct vec3 trans, scale;
	float rot[4];

	int nid = scene_initnode(s, name, (int)pid - 1, objid, flags,
	  &trans, rot, &scale);
	if (nid < 0)
		abort("Failed to create node %s\n", name);

	// Cam references its node
	//if (hasflags(flags, CAM))
	//	s->cams[objid].nid = nid;

	// Map file node id to scene node id
	*nodeid = nid;

	printf("- edon ----\n");

	return b;
}

void imp_bin(struct scene *s, unsigned char *bin)
{
	printf("- bin ----\n");

	unsigned char *b = bin;
	unsigned int dummy;

	unsigned int nodecnt;
	b = rduint(&nodecnt, b);
	printf("nodecnt: %d\n", nodecnt);

	unsigned int meshcnt;
	b = rduint(&meshcnt, b);
	printf("meshcnt: %d\n", meshcnt);

	unsigned int camcnt;
	b = rduint(&camcnt, b);
	printf("camcnt: %d\n", camcnt);

	b = rduint(&dummy, b);
	printf("lightcnt: %d\n", dummy);

	unsigned int mtlcnt;
	b = rduint(&mtlcnt, b);
	printf("mtlcnt: %d\n", mtlcnt);

	unsigned int facegcnt;
	b = rduint(&facegcnt, b);
	printf("facegrpcnt: %d\n", facegcnt);

	unsigned int vertgcnt;
	b = rduint(&vertgcnt, b);
	printf("vertexgrpcnt: %d\n", vertgcnt);

	unsigned int nrmgcnt;
	b = rduint(&nrmgcnt, b);
	printf("nrmgrpcnt: %d\n", nrmgcnt);

	unsigned int trackcnt;
	b = rduint(&trackcnt, b);
	printf("trackcnt: %d\n", trackcnt);

	unsigned int animstartframe;
	b = rduint(&animstartframe, b);
	printf("animstartframe: %d\n", animstartframe);

	unsigned int animendframe;
	b = rduint(&animendframe, b);
	printf("animendframe: %d\n", animendframe);

	unsigned int animsmplstep;
	b = rduint(&animsmplstep, b);
	printf("animsmplstep: %d\n", animsmplstep);

	float animscenefps;
	b = rdfloat(&animscenefps, b);
	printf("animscenefps: %6.3f\n", animscenefps);

	float animfps;
	b = rdfloat(&animfps, b);
	printf("animfps: %6.3f\n", animfps);

	unsigned int bgcoltid;
	b = rduint(&bgcoltid, b);
	printf("bgcol tid: %d\n", bgcoltid);

	unsigned int bgcoltofs;
	b = rduint(&bgcoltofs, b);
	printf("bgcol tofs: %d\n", bgcoltofs);

	// Temporary
	trackcnt = 0;
	unsigned int samplercnt = 0;
	unsigned int animbytes = 0;

	scene_init(s, meshcnt, max(1, mtlcnt), max(1, camcnt),
	  nodecnt, trackcnt, samplercnt, animbytes);

	unsigned int nodemap[nodecnt];
	for (unsigned int i = 0; i < nodecnt; i++) {
		b = rdnode(s, &nodemap[i], b);
	}

	printf("- nib ----\n");
}

void imp_bin_f(struct scene *s, char *name)
{
	unsigned long long sz;
	unsigned char *bin = mmread(name, &sz);

	imp_bin(s, bin);

	munmap(bin, sz);
}

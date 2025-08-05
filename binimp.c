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

static unsigned char *rduint(unsigned int *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

static unsigned char *rdint(int *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

static unsigned char *rdfloat(float *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
}

static unsigned char *rdvec3(struct vec3 *v, unsigned char *b)
{
	unsigned long long sz = sizeof(*v);
	memcpy(v, b, sz);
	return b + sz;
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

	b = rduint(&dummy, b); // Light cnt

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

	// Temporary
	trackcnt = 0;
	unsigned int samplercnt = 0;
	unsigned int animbytes = 0;

	scene_init(s, meshcnt, max(1, mtlcnt), max(1, camcnt),
	  1 + nodecnt, trackcnt, samplercnt, animbytes);

	printf("- nib ----\n");
}

void imp_bin_f(struct scene *s, char *name)
{
	unsigned long long sz;
	unsigned char *bin = mmread(name, &sz);

	imp_bin(s, bin);

	munmap(bin, sz);
}

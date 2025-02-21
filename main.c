#include <stdio.h>

#include "import.h"
#include "rend.h"
#include "scene.h"
#include "util.h"

void print_type_sizes(void)
{
	printf("sizeof(void *): %ld\n", sizeof(void *));
	printf("sizeof(float): %ld\n", sizeof(float));
	printf("sizeof(double): %ld\n", sizeof(double));
	printf("sizeof(char): %ld\n", sizeof(char));
	printf("sizeof(short int): %ld\n", sizeof(short int));
	printf("sizeof(int): %ld\n", sizeof(int));
	printf("sizeof(long int): %ld\n", sizeof(long int));
	printf("sizeof(long long int): %ld\n", sizeof(long long int));
	printf("\n\n");
}

int main(int argc, char *argv[])
{
	struct scene s = { 0 };
	if (import_gltf(&s, "../data/test.gltf", "../data/test.bin") != 0)
		printf("Failed to import gltf\n");

	printf("imported scene with %d meshes, %d mtls, %d cams, %d roots, %d nodes\n",
	  s.meshcnt, s.mtlcnt, s.camcnt, s.rootcnt, s.nodecnt);

	struct rdata *rd = rend_init(&s);

	rend_release(rd);

	scene_release(&s);

	return 0;
}

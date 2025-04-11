#ifndef IMPORT_H
#define IMPORT_H

struct scene;

void import_gltf(struct scene *s, const char *gltfname, const char *binname);

#endif

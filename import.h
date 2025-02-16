#ifndef IMPORT_H
#define IMPORT_H

struct scene;

int import_gltf(struct scene *s, const char *gltfname, const char *binname);

#endif

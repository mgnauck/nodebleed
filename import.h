#ifndef IMPORT_H
#define IMPORT_H

struct scene;

void import_file_gltf(struct scene *s, char *name);
void import_file_bkse(struct scene *s, char *name);

#endif

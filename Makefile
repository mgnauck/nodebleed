.POSIX:
OBJ = gltf.o import.o main.o mat4.o rend.o scene.o util.o vec3.o
OUT = a.out
CC = clang
CFLAGS = -std=c99 -march=native -flto -O3 -Wall -Wextra -Wpedantic -Wshadow
CFLAGS += $(shell sdl2-config --cflags)
#CFLAGS += -g
#CFLAGS += -fsanitize=undefined
#CFLAGS += -DNDEBUG
LDLIBS = -lm $(shell sdl2-config --libs)
#LDFLAGS += -fsanitize=undefined
#LDFLAGS = -flto -O3 -s
LDFLAGS += -fuse-ld=lld
LDFLAGS += -s

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

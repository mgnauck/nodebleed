.POSIX:
OBJ = gltf.o import.o main.o mat4.o rend.o scene.o util.o vec3.o
OUT = a.out
CC = clang
CFLAGS = -std=c11 -flto -O3 -Wall -Wextra -Wpedantic -march=native
CFLAGS += $(shell sdl2-config --cflags)
CFLAGS += -g -Werror -Wshadow
#CFLAGS += -fsanitize=undefined
#CFLAGS += -DNDEBUG
LDLIBS = -lm $(shell sdl2-config --libs)
#LDFLAGS += -fsanitize=undefined
#LDFLAGS = -flto -O3 -s
LDFLAGS += -fuse-ld=lld -s

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)
.c.o:
	$(CC) $(CFLAGS) -c $<
.s.o:
	$(CC) -c $<

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

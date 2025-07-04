.POSIX:
OBJ = gltf.o import.o main.o mat4.o rend.o scene.o util.o vec3.o
OUT = a.out
CC = clang
CFLAGS = -std=c11 -march=native -O3 -Wall -Wextra -Wpedantic -Wshadow
CFLAGS += -flto
CFLAGS += -fno-plt
CFLAGS += -fno-stack-protector -fno-stack-check
CFLAGS += -fno-unwind-tables -fno-asynchronous-unwind-tables -fno-exceptions
CFLAGS += -funsafe-math-optimizations -ffast-math
CFLAGS += -fomit-frame-pointer
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fmerge-all-constants
CFLAGS += -fno-PIC -fno-PIE
CFLAGS += $(shell sdl2-config --cflags)
#CFLAGS += -g
#CFLAGS += -fsanitize=undefined
#CFLAGS += -DNDEBUG
LDLIBS = -lm
LDLIBS += $(shell sdl2-config --libs)
#LDFLAGS += -fsanitize=undefined
#LDFLAGS = -flto -O3
LDFLAGS += -fuse-ld=lld
LDFLAGS += -s
#strip -R .comment a.out

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

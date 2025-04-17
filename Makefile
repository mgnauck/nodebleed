.POSIX:
SRC = $(shell find -L . -type f -name '*.c')
OBJ = $(SRC:.c=.o)
OUT = a.out
CC = clang
CFLAGS = -std=c99 -flto -O3 -Wall -Wextra -Wpedantic
CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function
CFLAGS += $(shell sdl2-config --cflags)
CFLAGS += -g -Werror -Wshadow
#CFLAGS += -DNDEBUG
LDLIBS = -lm $(shell sdl2-config --libs)
#LDFLAGS = -flto -O3 -s
LDFLAGS += -fuse-ld=lld -s

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

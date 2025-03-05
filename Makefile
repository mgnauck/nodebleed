SRC = $(shell find -L . -type f -name '*.c')
OBJ = $(SRC:.c=.o)
OUT = a.out

CC = tcc
CPPFLAGS =
CFLAGS = -std=c99 -O2 -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable
#CFLAGS += -flto
CFLAGS += $(shell sdl2-config --cflags)
#CFLAGS += -Wunsupported
#DBGFLAGS = -g
DBGFLAGS =
LIBS = -lm
LIBS += $(shell sdl2-config --libs)
LDFLAGS = -s
#LDFLAGS += -fuse-ld=lld

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DBGFLAGS) -c $<

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

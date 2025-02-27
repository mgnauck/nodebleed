SRC = $(shell find -L . -type f -name '*.c')
OBJ = $(SRC:.c=.o)
OUT = a.out

CC = tcc
CPPFLAGS =
CFLAGS = -std=c99 -Os -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable
#CFLAGS += -Wunsupported
CFLAGS += $(shell sdl2-config --cflags)
DBGFLAGS = -g
LIBS = -lm
LIBS += $(shell sdl2-config --libs)
LDFLAGS = -s

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DBGFLAGS) -c $<

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

SRC = $(shell find -L . -type f -name '*.c')
OBJ = $(SRC:.c=.o)
OUT = a.out

CC = tcc
CPPFLAGS =
CFLAGS = -std=c99 -Os -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-variable
CFLAGS += -Wunsupported
DBGFLAGS = -g
LIBS = -lm
LDFLAGS = -s

all: $(OUT)

$(OUT): $(OBJ)
	$(CC) $^ -o $@ $(LDFLAGS) $(LIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DBGFLAGS) -c $<

clean:
	rm -rf $(OUT) $(OBJ)

.PHONY: clean

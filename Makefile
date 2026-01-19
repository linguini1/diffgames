CC = gcc
OUT = diffgame

### WARNINGS ###
WARNINGS += -Wall -Wextra

### COMPILER FLAGS ###
CFLAGS += $(WARNINGS)
CFLAGS += -I include
CFLAGS += $(shell sdl2-config --cflags --libs)
CFLAGS += -lSDL2_ttf
CFLAGS += -lm

### SOURCE FILES ###
SRCDIR = src
SRC_FILES = $(wildcard $(SRCDIR)/*.c)
OBJ_FILES = $(patsubst %.c,%.o,$(SRC_FILES))

### BINARIES ###
BINDIR = bin
EXDIR = examples
EXAMPLES = $(patsubst $(EXDIR)/%,%,$(wildcard $(EXDIR)/*))

.PHONY: $(EXAMPLES)

all: $(EXAMPLES)

$(EXAMPLES): $(OBJ_FILES)
	$(MAKE) --silent -C $(EXDIR)/$@
	$(CC) $(CFLAGS) $(OBJ_FILES) $(EXDIR)/$@/main.c -o $(BINDIR)/$@

$(BINDIR)/%: $(EXDIR)/%.c $(OBJ_FILES)
	$(CC) $(CFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	@$(RM) $(OBJ_FILES)
	@$(RM) $(EXAMPLE_BINS)

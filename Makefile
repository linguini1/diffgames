CC = gcc
OUT = diffgame

### WARNINGS ###
WARNINGS += -Wall -Wextra

### COMPILER FLAGS ###
CFLAGS += $(WARNINGS)
CFLAGS += -I include
CFLAGS += -lm

ifneq ($(DEBUG),)
CFLAGS += -DCONFIG_DEBUG
endif

ifeq ($(OS), Windows_NT)
SDL_PATH = C:/MinGW/SDL2-2.32.10/i686-w64-mingw32
SDLTTF_PATH = C:/MinGW/SDL2_ttf-2.24.0/i686-w64-mingw32
CFLAGS += -I $(SDL_PATH)/include
CFLAGS += -I $(SDL_PATH)/include/SDL2
CFLAGS += -L $(SDL_PATH)/lib
CFLAGS += -I $(SDLTTF_PATH)/include
CFLAGS += -I $(SDLTTF_PATH)/include/SDL2
CFLAGS += -L $(SDLTTF_PATH)/lib
CFLAGS += -lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf
CFLAGS += -lm
# CFLAGS += -lgsl -lgslcblas -lm
else
CFLAGS += $(shell sdl2-config --cflags --libs)
CFLAGS += -lSDL2_ttf # SDL2 TTF library
CFLAGS += -lgsl # GNU Scientific Library
endif

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
	$(CC) $(OBJ_FILES) $(EXDIR)/$@/main.c $(CFLAGS) -o $(BINDIR)/$@

$(BINDIR)/%: $(EXDIR)/%.c $(OBJ_FILES)
	$(CC) $^ $(CFLAGS) -o $@

%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

clean:
	@$(RM) $(OBJ_FILES)
	@$(RM) $(EXAMPLE_BINS)

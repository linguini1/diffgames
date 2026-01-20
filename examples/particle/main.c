#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

#define TIMESTEP (0.01)

static const char window_name[] = "Particle";

static double scale = 5.0;
static double p_vel = 50.0;

struct game {
  vec2d_t ppos;
  double heading;
  int mousex;
  int mousey;
};

static void particle_f(void *x, double dt);
static void particle_u(void *x, double dt);

int main(int argc, char **argv) {
  SDL_Event event;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  struct game game_x;
  bool running = true;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:v:")) != -1) {
    switch (c) {
    case 'h':
      puts(HELP_TEXT);
      exit(EXIT_SUCCESS);
      break;
    case 'x':
      dm.w = strtoul(optarg, NULL, 10);
      break;
    case 'y':
      dm.h = strtoul(optarg, NULL, 10);
      break;
    case 's':
      scale = strtod(optarg, NULL);
      break;
    case 'v':
      p_vel = strtod(optarg, NULL);
      if (p_vel <= 0.0) {
        fprintf(stderr, "Invalid velocity.\n");
      }
      break;
    case '?':
      fprintf(stderr, "Unknown option -%c\n", optopt);
      exit(EXIT_FAILURE);
      break;
    }
  }

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("Could not initialize SDL: %s\n", SDL_GetError());
  }

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;

  SDL_Rect fullscreen = {.x = 0, .y = 0, .w = dm.w, .h = dm.h};

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Set up particle with random initial conditons */

  game_x.ppos.x = randval(0, dm.w / scale);
  game_x.ppos.y = randval(0, dm.h / scale);
  game_x.mousex = 0;
  game_x.mousey = 0;
  dynsys_t game = DYNSYS_SINIT(&game_x, particle_f, particle_u, NULL, NULL);

  while (running) {

    /* Check for input events */

    while (SDL_PollEvent(&event)) {

      switch (event.type) {

      case SDL_QUIT:
        running = false;
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {

        case SDLK_ESCAPE:
        case SDLK_q:
          running = false;
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Clear screen to black with opacity so trail is visible */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 10);
    SDL_RenderFillRect(renderer, &fullscreen);

    /* Set to foreground colour (white) */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    /* Draw agents */

    render_vec2d(renderer, &game_x.ppos);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation */

    SDL_GetMouseState(&game_x.mousex, &game_x.mousey);
    dynsys_step(&game, TIMESTEP);
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

/* Particle dynamics. */

static void particle_f(void *x, double dt) {
  struct game *game = (struct game *)x;
  game->ppos.x += p_vel * dt * cos(game->heading);
  game->ppos.y += p_vel * dt * sin(game->heading);
}

/* Particle control function. Particle will always try to follow the cursor. */

static void particle_u(void *x, double dt) {
  unused(dt);
  struct game *game = (struct game *)x;

  /* Compute a heading which moves towards the mouse position */
  double dx = ((double)game->mousex / scale) - game->ppos.x;
  double dy = ((double)game->mousey / scale) - game->ppos.y;
  game->heading = atan2(dy, dx);
}

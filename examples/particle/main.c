#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "helptext.h"
#include "utils.h"

#define TIMESTEP (0.01)

static const char window_name[] = "Particle";

static int mouse_x;
static int mouse_y;
static double scale = 5.0;
static double p_vel = 50.0;

enum state_e {
  P_X, /* Cartesian x position */
  P_Y, /* Cartesian y position */
  P_H, /* Velocity in units/s */
};

static void particle_f(double *x, size_t n, double dt);
static void particle_u(double *x, size_t n, double dt);

int main(int argc, char **argv) {
  SDL_Event event;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
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

  double x[] = {
      [P_X] = randval(0, dm.w / scale), /* X pos */
      [P_Y] = randval(0, dm.h / scale), /* Y pos */
      [P_H] = 0,                        /* Heading */
  };
  dynsys_t particle = DYNSYS_SINIT(x, particle_f, particle_u, NULL, NULL);

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

    SDL_RenderDrawPoint(renderer, x[P_X], x[P_Y]);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation */

    SDL_GetMouseState(&mouse_x, &mouse_y);
    dynsys_step(&particle, TIMESTEP);
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

/* Particle dynamics. */

static void particle_f(double *x, size_t n, double dt) {
  unused(n);
  x[P_X] += p_vel * dt * cos(x[P_H]);
  x[P_Y] += p_vel * dt * sin(x[P_H]);
}

/* Particle control function. Particle will always try to follow the cursor. */

static void particle_u(double *x, size_t n, double dt) {
  unused(n);
  unused(dt);

  /* Compute a heading which moves towards the mouse position */
  double dx = ((double)mouse_x / scale) - x[P_X];
  double dy = ((double)mouse_y / scale) - x[P_Y];
  x[P_H] = atan2(dy, dx);
}

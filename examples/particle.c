#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "dynsys.h"

#define TIMESTEP (0.01)

const char window_name[] = "Mouse-following Particle";
const int width = 2048;
const int height = 1024;
const double scale = 4.0;

static int mouse_x;
static int mouse_y;

/* Represents a mass-less particle */

struct particle_state {
  double x;       /* Cartesian x position */
  double y;       /* Cartesian y position */
  double vel;     /* Velocity in units/s */
  double heading; /* Heading in radians */
};

/* Particle dynamics. */

static void particle_f(double *x, size_t n, double dt) {
  (void)n;
  struct particle_state *p = (struct particle_state *)x;
  p->x += p->vel * dt * cos(p->heading);
  p->y += p->vel * dt * sin(p->heading);
}

/* Particle control function. Particle will always try to follow the cursor. */

static void particle_u(double *x, size_t n, double dt) {
  (void)n;
  (void)dt;
  struct particle_state *p = (struct particle_state *)x;

  /* Compute a heading which moves towards the mouse position */

  double dx = ((double)mouse_x / scale) - p->x;
  double dy = ((double)mouse_y / scale) - p->y;
  p->heading = atan2(dy, dx);
}

int main(void) {

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("Could not initialize SDL: %s\n", SDL_GetError());
  }

  /* Create window */

  SDL_Window *window = SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, width, height,
                                        SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Set up particle with initial conditons */

  double x[4] = {
      (width / (2 * scale)),  /* X pos */
      (height / (2 * scale)), /* Y pos */
      20.0,                   /* Velocity */
      0,                      /* Heading */
  };
  dynsys_t particle = DYNSYS_SINIT(x, particle_f, particle_u, NULL, NULL);

  /* Render simulation */

  bool running = true;
  SDL_Event event;

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

    /* Clear screen to black */

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    /* Set to foreground colour (white) */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    /* Draw agents */

    SDL_RenderDrawPoint(renderer, x[0], x[1]);

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

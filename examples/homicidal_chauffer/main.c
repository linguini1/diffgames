#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "utils.h"

#define TIMESTEP (0.01)

/* State variable indexes */

#define P_X (0)
#define P_Y (1)
#define P_HEAD (2)
#define E_X (3)
#define E_Y (4)
#define E_HEAD (6)

/* Game constants */

#define GAME_SPEED (20.0)
#define PURSUER_VEL (1.0 * GAME_SPEED)
#define EVADER_VEL (0.5 * GAME_SPEED)
#define PURSUER_R (1.0) /* Turning radius */
#define CAPTURE_RADIUS (2.5)

const char window_name[] = "Homicidal Chauffer";
const int width = 2048;
const int height = 1024;
const double scale = 4.0;

/* Game dynamics */

static void game_f(double *x, size_t n, double dt) {
  unused(n);
  /* Isaacs p. 28 */
  x[P_X] += dt * PURSUER_VEL * sin(x[P_HEAD]);
  x[P_Y] += dt * PURSUER_VEL * cos(x[P_HEAD]);
  x[E_X] += dt * EVADER_VEL * sin(x[E_HEAD]);
  x[E_Y] += dt * EVADER_VEL * cos(x[E_HEAD]);
}

static void game_u(double *x, size_t n, double dt) {
  unused(n);
  double phi = -1; /* TODO: calc */
  x[P_HEAD] += dt * (PURSUER_VEL / PURSUER_R) * phi;
  x[E_HEAD] = M_PI_4; /* TODO */
}

/* Running cost of the game is time to capture */
static double game_g(const double *x, size_t n, double dt) {
  unused(n);
  unused(x);
  return dt;
}

/* Distance between the pursuer and evader */
static double distance(const double *x) {
  return sqrt(pow((x[P_X] - x[E_X]), 2) + pow((x[P_Y] - x[E_Y]), 2));
}

int main(int argc, char **argv) {
  unused(argc);
  unused(argv);

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

  /* Set up game with initial conditons */

  double game_x[] = {
      [P_X] = (width / (2 * scale)),  /* Pursuer x */
      [P_Y] = (height / (2 * scale)), /* Pursuer y */
      [P_HEAD] = 0,                   /* Pursuer heading */
      [E_X] = 0,                      /* Evader x */
      [E_Y] = 0,                      /* Evader y */
      [E_HEAD] = 0,                   /* Evader heading */
  };
  dynsys_t game = DYNSYS_SINIT(game_x, game_f, game_u, game_g, NULL);

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

    /* Clear screen to black with some transparency so we can see the movement
     * trails of the players.
     * TODO: figure out alpha
     */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    /* Draw agents. Pursuer is red, evader is green. */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawPoint(renderer, game_x[P_X], game_x[P_Y]); /* P */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderDrawPoint(renderer, game_x[E_X], game_x[E_Y]); /* E */

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation. If the distance between the pursuer and the evader is
     * within the capture range, stop the game and just render a frozen frame of
     * the capture.
     */

    double curdist = distance(game_x);
    if (!(curdist <= CAPTURE_RADIUS)) {
      dynsys_step(&game, TIMESTEP);
    }
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

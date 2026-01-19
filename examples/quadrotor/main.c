#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "utils.h"

#define TIMESTEP (0.01)

static const char window_name[] = "Quadrotor Dynamics";
static const int width = 2048;
static const int height = 1024;
static const double scale = 8.0;

/* State variable indices */

enum state_e {
  Q_F1,  /* Motor 1 thrust */
  Q_F2,  /* Motor 2 thrust */
  Q_F3,  /* Motor 3 thrust */
  Q_F4,  /* Motor 4 thrust */
  Q_X,   /* X position */
  Q_Y,   /* Y position */
  Q_Z,   /* Z position */
  Q_R,   /* Roll */
  Q_P,   /* Pitch */
  Q_YW,  /* Yaw */
  Q_VX,  /* X velocity */
  Q_VY,  /* Y velocity */
  Q_VZ,  /* Z velocity */
  Q_VR,  /* Roll velocity */
  Q_VP,  /* Pitch velocity */
  Q_VYW, /* Yaw velocity */
};

/* Parameters */

#define G (9.81)          /* m/s^2 */
#define QUAD_MASS (4.0)   /* kg */
#define ROTOR_LEN (0.315) /* m */
#define QUAD_J1 (0.05)    /* kgm^2 */
#define QUAD_J2 (0.05)    /* kgm^2 */
#define QUAD_J3 (0.10)    /* kgm^2 */

static void quad_f(double *x, size_t n, double dt);
static void quad_u(double *x, size_t n, double dt);

int main(int argc, char **argv) {
  unused(argc);
  unused(argv);
  SDL_Rect fullscreen = {.x = 0, .y = 0, .w = width, .h = height};

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

  /* Set up particle with initial conditions */

  double quad_x[] = {
      [Q_F1] = 0.0,
      [Q_F2] = 0.0,
      [Q_F3] = 0.0,
      [Q_F4] = 0.0,
      [Q_X] = width / (2 * scale),
      [Q_Y] = height / (2 * scale),
      [Q_Z] = height / (2 * scale),
      [Q_R] = 0.0,
      [Q_P] = 0.0,
      [Q_YW] = 0.0,
      [Q_VX] = 0.0,
      [Q_VY] = 0.0,
      [Q_VZ] = 0.0,
      [Q_VR] = 0.0,
      [Q_VP] = 0.0,
      [Q_VYW] = 0.0,
  };
  dynsys_t quad = DYNSYS_SINIT(quad_x, quad_f, quad_u, NULL, NULL);

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

    /* Clear screen to black with opacity so trail is visible */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 10);
    SDL_RenderFillRect(renderer, &fullscreen);

    /* Set to foreground colour (white) */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    /* Draw
     * TODO: right now the y axis is the quadrotor's z, while the
     * x axis is the real x axis.
     */

    SDL_RenderDrawPoint(renderer, quad_x[Q_X], quad_x[Q_Z]);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation */

    dynsys_step(&quad, TIMESTEP);
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

static void quad_f(double *x, size_t n, double dt) {
  (void)n;

  double v1 = (x[Q_F1] + x[Q_F2] + x[Q_F3] + x[Q_F4]) / QUAD_MASS;
  double v2 = (-x[Q_F1] - x[Q_F2] + x[Q_F3] + x[Q_F4]) / QUAD_J1;
  double v3 = (-x[Q_F1] + x[Q_F2] + x[Q_F3] - x[Q_F4]) / QUAD_J2;
  double v4 = (x[Q_F1] - x[Q_F2] + x[Q_F3] - x[Q_F4]) / QUAD_J3;

  x[Q_X] += dt * x[Q_VX];
  x[Q_Y] += dt * x[Q_VY];
  x[Q_Z] += dt * x[Q_VZ];
  x[Q_R] += dt * x[Q_VR];
  x[Q_P] += dt * x[Q_VP];
  x[Q_YW] += dt * x[Q_VYW];

  x[Q_VX] +=
      dt * v1 *
      (cos(x[Q_P]) * sin(x[Q_R]) * cos(x[Q_YW]) + sin(x[Q_P]) * sin(x[Q_YW]));
  x[Q_VY] +=
      dt * v1 *
      (sin(x[Q_R]) * sin(x[Q_YW]) * cos(x[Q_P]) - cos(x[Q_YW]) * sin(x[Q_P]));
  x[Q_VZ] += dt * (v1 * (cos(x[Q_R]) * cos(x[Q_P])) - G);
  x[Q_VR] += dt * v2 * ROTOR_LEN;
  x[Q_VP] += dt * v3 * ROTOR_LEN;
  x[Q_VYW] += dt * v4;
}

static void quad_u(double *x, size_t n, double dt) {
  (void)n;

  /* For fun, sine wave on the motors. Thrust in Newtons */

  static double t = 0;

  x[Q_F1] = 2.0 * sin(t);
  x[Q_F2] = 2.0 * sin(t);
  x[Q_F3] = 2.0 * sin(t);
  x[Q_F4] = 2.0 * sin(t);
  t += dt;
}

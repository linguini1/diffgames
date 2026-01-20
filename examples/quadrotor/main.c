#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "render.h"
#include "utils.h"

#define TIMESTEP (0.01)

static const char window_name[] = "Quadrotor Dynamics";
static const int width = 2048;
static const int height = 1024;
static const double scale = 8.0;

struct quadrotor {
  vec3d_t pos;     /* Cartesian position */
  vec3d_t rot;     /* Euler angles */
  vec3d_t vel;     /* Cartesian velocity */
  vec3d_t angvel;  /* Angular velocity */
  double force[4]; /* Motor thrusts */
};

/* Parameters */

#define G (9.81)          /* m/s^2 */
#define QUAD_MASS (4.0)   /* kg */
#define ROTOR_LEN (0.315) /* m */
#define QUAD_J1 (0.05)    /* kgm^2 */
#define QUAD_J2 (0.05)    /* kgm^2 */
#define QUAD_J3 (0.10)    /* kgm^2 */

static void quad_f(void *x, double dt);
static void quad_u(void *x, double dt);

int main(int argc, char **argv) {
  unused(argc);
  unused(argv);
  struct quadrotor quad;
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

  memset(&quad, 0, sizeof(quad));
  quad.pos.x = width / (2 * scale);
  quad.pos.y = height / (2 * scale);
  quad.pos.z = height / (2 * scale);
  dynsys_t game = DYNSYS_SINIT(&quad, quad_f, quad_u, NULL, NULL);

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

    render_vec2d(renderer, vec2d_temp(quad.pos.x, quad.pos.z));

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation */

    dynsys_step(&game, TIMESTEP);
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

static void quad_f(void *x, double dt) {
  struct quadrotor *quad = (struct quadrotor *)x;

  double v1 =
      (quad->force[0] + quad->force[1] + quad->force[2] + quad->force[3]) /
      QUAD_MASS;
  double v2 =
      (-quad->force[0] - quad->force[1] + quad->force[2] + quad->force[3]) /
      QUAD_J1;
  double v3 =
      (-quad->force[0] + quad->force[1] + quad->force[2] - quad->force[3]) /
      QUAD_J2;
  double v4 =
      (quad->force[0] - quad->force[1] + quad->force[2] - quad->force[3]) /
      QUAD_J3;

  quad->pos.x += dt * quad->vel.x;
  quad->pos.y += dt * quad->vel.y;
  quad->pos.z += dt * quad->vel.z;
  quad->rot.x += dt * quad->angvel.x;
  quad->rot.y += dt * quad->angvel.y;
  quad->rot.z += dt * quad->angvel.z;

  quad->vel.x += dt * v1 *
                 (cos(quad->rot.y) * sin(quad->rot.x) * cos(quad->rot.z) +
                  sin(quad->rot.y) * sin(quad->rot.z));
  quad->vel.y += dt * v1 *
                 (sin(quad->rot.x) * sin(quad->rot.z) * cos(quad->rot.y) -
                  cos(quad->rot.z) * sin(quad->rot.y));
  quad->vel.z += dt * (v1 * (cos(quad->rot.x) * cos(quad->rot.y)) - G);
  quad->angvel.x += dt * v2 * ROTOR_LEN;
  quad->angvel.y += dt * v3 * ROTOR_LEN;
  quad->angvel.z += dt * v4;
}

static void quad_u(void *x, double dt) {
  struct quadrotor *quad = (struct quadrotor *)x;
  static double t = 0;

  /* For fun, sine wave on the motors. Thrust in Newtons */

  for (unsigned i = 0; i < 4; i++) {
    quad->force[i] = 2.0 * sin(t);
  }

  t += dt;
}

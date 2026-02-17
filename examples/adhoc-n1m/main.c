/* NOTE: for my own sanity, right now this just renders the (x, y) positions
 * of the agents, not considering 3D. I will add 3D rendering later with some
 * actual shapes for the agents, but (x, y) is sufficient to show the behaviour
 * from a bird's eye view.
 */
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

#define TIMESTEP (0.01) /* Fraction of a second */

const char WINDOW_NAME[] = "Ad-hoc Network n = m = 1";

struct pursuer {
  vec3d_t pos;
  vec3d_t vel;
};

struct evader {
  vec3d_t pos;
  double heading;
};

struct game {
  struct pursuer p;
  struct evader *e;
  size_t m;
};

/* Game "constant" parameters */

static double l_max = 45.0; /* Maximum path loss in dB */

#define LAMBDA (0.32764203) /* Wave-length in meters (915MHz) */
#define V_P (20.0)          /* Pursuer velocity in m/s */
#define V_E (10.0)          /* Evader velocity in m/s */

/* Game dynamics */

static void game_f(void *x, double dt);
static void game_u(void *x, double dt);

static void game_seed(struct game *g, const vec3d_t *dim) {
  for (size_t j = 0; j < g->m; j++) {
    g->e[j].pos.x = randval(0.0, dim->x);
    g->e[j].pos.y = randval(0.0, dim->y);
    g->e[j].pos.z = 0.0;
  }

  g->p.pos.x = randval(0.0, dim->x);
  g->p.pos.y = randval(0.0, dim->y);
  g->p.pos.z = randval(0.0, dim->z);
}

int main(int argc, char **argv) {

  double scale = 5.0;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  struct game game_x;
  bool running = true;
  bool show_capture_radius = false;
  bool game_over = false;
  camera3d_t camera;

  /* Default: 1 evader */

  game_x.m = 1;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:m:l:")) != -1) {
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
    case 'l':
      l_max = strtod(optarg, NULL);
      break;
    case 's':
      scale = strtod(optarg, NULL);
      break;
    case 'm':
      game_x.m = strtoul(optarg, NULL, 10);
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
    fprintf(stderr, "Could not initialize SDL: %s\n", SDL_GetError());
  }

  /* Determine window and game space dimensions */

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;
  vec2d_t screen_center =
      VEC2D_SINIT(dm.w / (2.0 * scale), dm.h / (2.0 * scale));
  SDL_Rect fullscreen = {.x = 0, .y = 0, .w = dm.w, .h = dm.h};
  vec3d_t dimensions = VEC3D_SINIT(dm.w / scale, dm.w / scale, dm.h / scale);

  /* Set up camera for rendering
   *
   * Viewing from boundary furthest from the origin.
   * Looking down at 45deg angle to XY plane.
   */

  camera.pos = (vec3d_t)VEC3D_SINIT(dimensions.x / scale, dimensions.y / scale,
                                    dimensions.z / scale);
  camera.ori = (vec3d_t)VEC3D_SINIT(-M_PI_4 / 2, 0, 0);

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Compute radius of adequate connection given L_max */

  double capture_radius = pow(10.0, l_max / 20.0) / (4.0 * M_PI / LAMBDA);

  /* Allocate memory for evader positions */

  game_x.e = malloc(sizeof(struct evader) * game_x.m);
  if (game_x.e == NULL) {
    fprintf(stderr, "Not enough memory.\n");
    return EXIT_FAILURE;
  }

  /* Set up game with random initial conditions */

  srand(time(NULL));
  game_seed(&game_x, &dimensions);

  dynsys_t game = DYNSYS_SINIT(&game_x, game_f, game_u, NULL, NULL);

  /* Simulation loop */

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
        case SDLK_r:
          show_capture_radius = !show_capture_radius;
          break;
        case SDLK_SPACE:
          game_seed(&game_x, &dimensions);
          dynsys_init(&game, &game_x, game_f, game_u, NULL, NULL);
          SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
          SDL_RenderClear(renderer);
          SDL_RenderPresent(renderer);
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Clear screen to black with semi-transparency so trail appears */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 10);
    SDL_RenderFillRect(renderer, &fullscreen);

    /* Draw pursuers in red */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    render_vec2d(renderer, &game_x.p.pos);

    /* Draw evaders in green */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
    for (size_t j = 0; j < game_x.m; j++) {
      render_vec2d(renderer, &game_x.e[j].pos);
    }

    /* Draw radius of L_max in white (this is a sphere) */

    if ((show_capture_radius || game_over) &&
        !f_is_zero(capture_radius, 0.01)) {
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
      render_circle(renderer, (vec2d_t *)&game_x.p.pos, capture_radius, 40);
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Clear pursuer capture radius before next slide */

    if ((show_capture_radius || game_over) &&
        !f_is_zero(capture_radius, 0.01)) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
      render_circle(renderer, (vec2d_t *)&game_x.p.pos, capture_radius, 40);
    }

    /* TODO: Compute terminal condition. Should be equivalent to when evader is
     * T_d time units away from being within the evader radius.
     *
     * NOTE: Current approximation is evader inside distance radius.
     */

    game_over = true;
    for (size_t j = 0; j < game_x.m; j++) {
      if (vec3d_dist_r(&game_x.p.pos, &game_x.e[j].pos) > capture_radius) {
        game_over = false;
        break;
      }
    }
#ifdef CONFIG_DEBUG
    if (game_over) printf("Game over!\n");
    printf("Pursuer 3D " VEC3D_FMT "\n", VEC3D_PRINTF(&game_x.p.pos));
    printf("Evader 3D " VEC3D_FMT "\n", VEC3D_PRINTF(&game_x.e.pos));
#endif

    if (!game_over) {
      dynsys_step(&game, TIMESTEP);
    }
  }

  /* Release resources */

  free(game_x.e);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

/* Dynamics of the evader */
static void evader_f(struct evader *e, double vel, double dt) {
  e->pos.x += dt * vel * cos(e->heading);
  e->pos.y += dt * vel * sin(e->heading);
}

/* Dynamics of the pursuer */
static void pursuer_f(struct pursuer *p, double dt) {
  p->pos.x += dt * p->vel.x;
  p->pos.y += dt * p->vel.y;
  p->pos.z += dt * p->vel.z;
}

/* Dynamics for all players */
static void game_f(void *x, double dt) {
  struct game *game = (struct game *)x;
  pursuer_f(&game->p, dt);
  for (size_t j = 0; j < game->m; j++) {
    evader_f(&game->e[j], V_E, dt);
  }
}

static void game_u(void *x, double dt) {
  unused(dt);
  struct game *game = (struct game *)x;

  /* Co-state computation */

  double c = 20.0 / log(10.0);
  double denom;
  double l_xp = 0;
  double l_yp = 0;
  double l_zp = 0;

  /* Pursuer co-states */

  for (size_t j = 0; j < game->m; j++) {
    denom = pow(game->p.pos.x - game->e[j].pos.x, 2) +
            pow(game->p.pos.y - game->e[j].pos.y, 2) + pow(game->p.pos.z, 2);

    l_xp += (game->p.pos.x - game->e[j].pos.x) / denom;
    l_yp += (game->p.pos.y - game->e[j].pos.y) / denom;
    l_zp += game->p.pos.z / denom;
  }

  l_xp *= c;
  l_yp *= c;
  l_zp *= c;

  /* Pursuer inputs */

  double costate_mag = sqrt(l_xp * l_xp + l_yp * l_yp + l_zp * l_zp);
  game->p.vel.x = -V_P * (l_xp / costate_mag);
  game->p.vel.y = -V_P * (l_yp / costate_mag);
  game->p.vel.z = -V_P * (l_zp / costate_mag);

  /* Evader co-states */

  double l_xj;
  double l_yj;
  for (size_t j = 0; j < game->m; j++) {
    denom = pow(game->p.pos.x - game->e[j].pos.x, 2) +
            pow(game->p.pos.y - game->e[j].pos.y, 2) + pow(game->p.pos.z, 2);
    l_xj = -c * (game->p.pos.x - game->e[j].pos.x) / denom;
    l_yj = -c * (game->p.pos.y - game->e[j].pos.y) / denom;

    /* Evader inputs */

    game->e[j].heading = atan2(l_yj, l_xj);
  }
}

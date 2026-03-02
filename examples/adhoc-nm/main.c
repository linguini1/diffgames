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
#include <string.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

#define TIMESTEP (0.01) /* Fraction of a second */

const char WINDOW_NAME[] = "Ad-hoc Aerial Network";

struct pursuer {
  vec3d_t pos;
  vec3d_t vel;
};

struct evader {
  vec3d_t pos;
  double heading;
};

struct game {
  struct pursuer *p;
  size_t n;
  struct evader *e;
  size_t m;
};

/* Game "constant" parameters */

static bool random_walk = false; /* Evader movement is random */
static double l_max = 45.0;      /* Maximum path loss in dB */

#define LAMBDA (0.32764203)        /* Wave-length in meters (915MHz) */
#define V_P (40.0)                 /* Pursuer velocity in m/s */
#define V_E (20.0)                 /* Evader velocity in m/s */
#define RANDWALK_ERRATICISIM (0.1) /* How erratic the random walk is. */

/* Game dynamics */

static void game_f(void *x, double dt);
static void game_u(void *x, double dt);

static void game_seed(struct game *g, const vec3d_t *dim) {
  for (size_t j = 0; j < g->m; j++) {
    g->e[j].pos.x = randval(0.0, dim->x);
    g->e[j].pos.y = randval(0.0, dim->y);
    g->e[j].pos.z = 0.0;
    g->e[j].heading = randval(0.0, 2.0 * M_PI);
  }

  for (size_t i = 0; i < g->n; i++) {
    g->p[i].pos.x = randval(0.0, dim->x);
    g->p[i].pos.y = randval(0.0, dim->y);
    g->p[i].pos.z = randval(0.0, dim->z);
    memset(&g->p[i].vel, 0, sizeof(g->p[i].vel));
  }
}

int main(int argc, char **argv) {
  double scale = 5.0;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  struct game game_x;
  bool running = true;
  bool show_network = false;
  bool game_over = false;
  camera3d_t camera;

  /* Default: 1 evader, 1 pursuer */

  game_x.m = 1;
  game_x.n = 1;
  size_t z_height = 100;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:z:s:m:n:l:r")) != -1) {
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
    case 'z':
      z_height = strtoul(optarg, NULL, 10);
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
    case 'n':
      game_x.n = strtoul(optarg, NULL, 10);
      break;
    case 'r':
      random_walk = true;
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
  vec3d_t dimensions = VEC3D_SINIT(dm.w / scale, dm.w / scale, z_height);

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

  /* Allocate memory for pursuer positions */

  game_x.p = malloc(sizeof(struct pursuer) * game_x.n);
  if (game_x.p == NULL) {
    fprintf(stderr, "Not enough memory.\n");
    return EXIT_FAILURE;
  }

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
        case SDLK_n:
          show_network = !show_network;
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

    /* Show networking lines */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    for (size_t i = 0; i < game_x.n && show_network; i++) {

      /* Pursuer to evaders */

      for (size_t j = 0; j < game_x.m; j++) {
        if (vec3d_dist_r(&game_x.p[i].pos, &game_x.e[j].pos) <=
            capture_radius) {
          SDL_RenderDrawLine(renderer, game_x.p[i].pos.x, game_x.p[i].pos.y,
                             game_x.e[j].pos.x, game_x.e[j].pos.y);
        }
      }

      /* Pursuer to other pursuers */

      for (size_t k = 0; k < game_x.n; k++) {
        if (i == k) continue;
        if (vec3d_dist_r(&game_x.p[i].pos, &game_x.p[k].pos) <=
            capture_radius) {
          SDL_RenderDrawLine(renderer, game_x.p[i].pos.x, game_x.p[i].pos.y,
                             game_x.p[k].pos.x, game_x.p[k].pos.y);
        }
      }
    }

    /* Draw pursuers in red */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    for (size_t i = 0; i < game_x.n; i++) {
      render_vec2d(renderer, &game_x.p[i].pos);
    }

    /* Draw evaders in green */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
    for (size_t j = 0; j < game_x.m; j++) {
      render_vec2d(renderer, &game_x.e[j].pos);
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Hide networking lines */

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);

    for (size_t i = 0; i < game_x.n && show_network; i++) {

      /* Pursuer to evaders */

      for (size_t j = 0; j < game_x.m; j++) {
        if (vec3d_dist_r(&game_x.p[i].pos, &game_x.e[j].pos) <=
            capture_radius) {
          SDL_RenderDrawLine(renderer, game_x.p[i].pos.x, game_x.p[i].pos.y,
                             game_x.e[j].pos.x, game_x.e[j].pos.y);
        }
      }

      /* Pursuer to other pursuers */

      for (size_t k = 0; k < game_x.n; k++) {
        if (i == k) continue;
        if (vec3d_dist_r(&game_x.p[i].pos, &game_x.p[k].pos) <=
            capture_radius) {
          SDL_RenderDrawLine(renderer, game_x.p[i].pos.x, game_x.p[i].pos.y,
                             game_x.p[k].pos.x, game_x.p[k].pos.y);
        }
      }
    }

    /* TODO: Compute terminal condition based on network fragmentation
     *
     * NOTE: does not include consideration of T_d
     *
     * NOTE: Current approximation is evader inside distance radius.
     */

    if (!game_over) {
      dynsys_step(&game, TIMESTEP);
    }
  }

  /* Release resources */

  free(game_x.p);
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

  for (size_t i = 0; i < game->n; i++) {
    pursuer_f(&game->p[i], dt);
  }

  for (size_t j = 0; j < game->m; j++) {
    evader_f(&game->e[j], V_E, dt);
  }
}

static void game_u(void *x, double dt) {
  unused(dt);
  struct game *game = (struct game *)x;
  double c = 20.0 / log(10.0);

  double denom;
  double l_xi;
  double l_yi;
  double l_zi;
  double costate_mag;

  /* Pursuer co-states */

  for (size_t i = 0; i < game->n; i++) {
    l_xi = 0;
    l_yi = 0;
    l_zi = 0;

    /* First term, sum over evaders */

    for (size_t j = 0; j < game->m; j++) {
      denom = pow(game->p[i].pos.x - game->e[j].pos.x, 2) +
              pow(game->p[i].pos.y - game->e[j].pos.y, 2) +
              pow(game->p[i].pos.z, 2);
      l_xi += (game->p[i].pos.x - game->e[j].pos.x) / denom;
      l_yi += (game->p[i].pos.y - game->e[j].pos.y) / denom;
      l_zi += game->p[i].pos.z / denom;
    }

    /* Multiplicative constant */

    l_xi *= c;
    l_yi *= c;
    l_zi *= c;

    /* Pursuer inputs */

    costate_mag = sqrt(l_xi * l_xi + l_yi * l_yi + l_zi * l_zi);
    game->p[i].vel.x = -V_P * (l_xi / costate_mag);
    game->p[i].vel.y = -V_P * (l_yi / costate_mag);
    game->p[i].vel.z = -V_P * (l_zi / costate_mag);
  }

  /* If the evaders are configured to have a random walk instead of following
   * their optimal controls, do that instead of co-state computation.
   */

  if (random_walk) {
    for (size_t j = 0; j < game->m; j++) {
      game->e[j].heading += RANDWALK_ERRATICISIM * randval(-1.0, 1.0);
    }

    return;
  }

  /* Evader co-states */

  double l_xj;
  double l_yj;

  for (size_t j = 0; j < game->m; j++) {
    l_xj = 0;
    l_yj = 0;

    for (size_t i = 0; i < game->n; i++) {
      denom = pow(game->p[i].pos.x - game->e[j].pos.x, 2) +
              pow(game->p[i].pos.y - game->e[j].pos.y, 2) +
              pow(game->p[i].pos.z, 2);
      l_xj += (game->p[i].pos.x - game->e[j].pos.x) / denom;
      l_yj += (game->p[i].pos.y - game->e[j].pos.y) / denom;
    }

    l_xj *= -c;
    l_yj *= -c;

    /* Evader inputs */

    game->e[j].heading = atan2(l_yj, l_xj);
  }
}

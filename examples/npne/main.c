/* This implementation is based on the paper titled "Multiple Pursuer Multiple
 * Evader Differential Games". Specifically, the N = M PE game
 * described in Section IV-A.
 *
 * @ARTICLE{9122473,
 * author={Garcia, Eloy and Casbeer, David W. and Von Moll, Alexander and
 * Pachter, Meir},
 * journal={IEEE Transactions on Automatic Control},
 * title={Multiple Pursuer Multiple Evader Differential Games},
 * year={2021},
 * volume={66},
 * number={5},
 * pages={2345-2350},
 * keywords={Games;State feedback;Government;Aerospace
 * electronics;Switches;Weapons;Autonomous systems;intelligent control;optimal
 * control},
 * doi={10.1109/TAC.2020.3003840}}
 */

#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "3dtools.h"
#include "dynsys.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

#define TIMESTEP (0.01) /* Fraction of a second */

const char WINDOW_NAME[] = "N Pursuers, M Evaders";

struct pair {
  size_t i; /* Pursuer i */
  size_t j; /* Evader j */
};

struct agent {
  vec2d_t pos;
  double heading;
  double vel;
};

struct game {
  struct agent *agents;      /* All agents */
  struct agent *pursuers;    /* Offset into agent array for pursuers */
  struct agent *evaders;     /* Offset into agent array for evaders */
  struct pair **assignments; /* All possible assignments */
  size_t n_assign;           /* Number of possible assignments */
  size_t n;                  /* Value of N = M */
  double capture_radius;     /* Capture radius of pursuers */
};

/* Game "constant" parameters */

#define CAPTURE_TOLERANCE (0.08)

#define P_VEL_MIN (30.0)
#define P_VEL_MAX (40.0)
#define E_VEL_MIN (10.0)
#define E_VEL_MAX (29.0)

#define CIRCLE_POINTS (10)

/* Used for terminal condition: determine when all pairs have distance of 0 */
static size_t opt_assign = 0;

/* Game dynamics */

#define a(g, i, j) ((g)->evaders[j].vel / (g)->pursuers[i].vel)

static void game_f(void *x, double dt);
static void game_u(void *x, double dt);

static void game_randinit(struct game *g, SDL_DisplayMode *dm, double scale) {
  for (size_t i = 0; i < 2 * g->n; i++) {
    g->agents[i].pos.x = randval(0, dm->w / scale);
    g->agents[i].pos.y = randval(0, dm->h / scale);

    if (i < g->n) {
      g->agents[i].vel = randval(P_VEL_MIN, P_VEL_MAX);
    } else {
      g->agents[i].vel = randval(E_VEL_MIN, E_VEL_MAX);
    }
  }
}

static size_t n_combos(size_t n) {
  if (n == 2) return 2;
  return n * n_combos(n - 1);
}

int main(int argc, char **argv) {
  double scale = 5.0;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  struct game game_x;
  dynsys_t game;
  bool running = true;
  bool show_capture_radius = false;
  bool game_over = false;

  /* Default values */

  game_x.n = 2;
  game_x.capture_radius = 0.0;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:r:n:")) != -1) {
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
    case 'r':
      game_x.capture_radius = strtod(optarg, NULL);
      break;
    case 's':
      scale = strtod(optarg, NULL);
      break;
    case 'n':
      game_x.n = strtoul(optarg, NULL, 10);
      if (game_x.n == 0) {
        fprintf(stderr, "Number of agents cannot be 0.\n");
        exit(EXIT_FAILURE);
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
    fprintf(stderr, "Could not initialize SDL: %s\n", SDL_GetError());
  }

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;
  SDL_Rect fullscreen = {.x = 0, .y = 0, .w = dm.w, .h = dm.h};

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Initialize game */

  game_x.agents = malloc(sizeof(struct agent) * 2 * game_x.n);
  if (game_x.agents == NULL) {
    fprintf(stderr, "Couldn't allocate space for agent states.\n");
    exit(EXIT_FAILURE);
  }

  game_x.pursuers = game_x.agents;
  game_x.evaders = game_x.agents + game_x.n;

  /* Calculate the number of possible assignments */

  game_x.n_assign = n_combos(game_x.n);
  game_x.assignments = malloc(sizeof(struct pair *) * game_x.n_assign);
  if (game_x.assignments == NULL) {
    fprintf(stderr, "Couldn't allocate space for assignments.\n");
    exit(EXIT_FAILURE);
  }

  /* Each assignment must have space for N pairs */

  for (size_t i = 0; i < game_x.n_assign; i++) {
    game_x.assignments[i] = malloc(sizeof(struct pair) * game_x.n);
    if (game_x.assignments[i] == NULL) {
      fprintf(stderr, "Couldn't allocate space for assignments (%u).\n", i);
      exit(EXIT_FAILURE);
    }
  }

  /* Compute all possible assignments to populate array
   * TODO: this is an absolutely garbage method to generate combinations
   */

  size_t e;
  for (size_t a = 0; a < game_x.n_assign; a++) {

  retry_ass:
    for (size_t p = 0; p < game_x.n; p++) {
      /* P is the current pair index, and also always the pursuer index.
       *
       * Now we must choose the evader index such that no evader
       * repeats twice in this assignment, and so that the assignment is
       * unique.
       */
      game_x.assignments[a][p].i = p;
    retry_e:
      e = randval(0, game_x.n);
      for (size_t e1 = 0; e1 < p; e1++) {
        if (game_x.assignments[a][e1].j == e) goto retry_e;
      }
      game_x.assignments[a][p].j = e;
    }

    /* Check that this assignment hasn't been found yet */

    for (size_t a1 = 0; a1 < a; a1++) {
      if (memcmp(game_x.assignments[a], game_x.assignments[a1],
                 sizeof(struct pair) * game_x.n) == 0) {
        goto retry_ass;
      }
    }
  }

#if CONFIG_DEBUG
  /* Print out all pairs for debugging */

  for (size_t a = 0; a < game_x.n_assign; a++) {
    printf("[%u] -> ", a);
    for (size_t p = 0; p < game_x.n; p++) {
      printf("(%u, %u) ", game_x.assignments[a][p].i,
             game_x.assignments[a][p].j);
    }
    printf("\n");
  }
#endif

  /* Random initial conditions for x, y positions of all agents. Heading
   * is not relevant since it can be changed instantaneously. Velocities are
   * random within a range.
   */

  srand(time(NULL));
  game_randinit(&game_x, &dm, scale);
  dynsys_init(&game, &game_x, game_f, game_u, NULL, NULL);

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
          game_randinit(&game_x, &dm, scale);
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

    for (size_t i = 0; i < game_x.n; i++) {
      render_vec2d(renderer, &game_x.pursuers[i].pos);
    }

    /* Draw evaders in green */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);

    for (size_t j = 0; j < game_x.n; j++) {
      render_vec2d(renderer, &game_x.evaders[j].pos);
    }

    /* Draw pursuer capture radius in white */

    if ((show_capture_radius || game_over) &&
        !f_is_zero(game_x.capture_radius, 0.01)) {
      SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

      for (size_t i = 0; i < game_x.n; i++) {
        render_circle(renderer, &game_x.pursuers[i].pos, game_x.capture_radius,
                      CIRCLE_POINTS);
      }
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Clear pursuer capture radius before next slide */

    if ((show_capture_radius || game_over) &&
        !f_is_zero(game_x.capture_radius, 0.01)) {
      SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
      for (size_t i = 0; i < game_x.n; i++) {
        render_circle(renderer, &game_x.pursuers[i].pos, game_x.capture_radius,
                      CIRCLE_POINTS);
      }
    }

    /* Advance simulation until a capture occurs */

    game_over = true;
    for (size_t p = 0; p < game_x.n; p++) {
      struct pair pair = game_x.assignments[opt_assign][p];
      double dist = vec2d_dist_r(&game_x.pursuers[pair.i].pos,
                                 &game_x.evaders[pair.j].pos);
      if (dist > game_x.capture_radius &&
          !f_is_equal(dist, game_x.capture_radius, CAPTURE_TOLERANCE)) {
        game_over = false;
        break;
      }
    }

    if (!game_over) {
      dynsys_step(&game, TIMESTEP);
    }
  }

  /* Release resources */

  for (size_t i = 0; i < game_x.n_assign; i++) {
    free(game_x.assignments[i]);
  }
  free(game_x.assignments);
  free(game_x.agents);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

/* Dynamics for a holonomic agent */

static void agent_f(struct agent *a, double dt) {
  a->pos.x += dt * a->vel * cos(a->heading);
  a->pos.y += dt * a->vel * sin(a->heading);
}

static void game_f(void *x, double dt) {
  struct game *game = (struct game *)x;
  for (size_t i = 0; i < game->n * 2; i++) {
    agent_f(&game->agents[i], dt);
  }
}

static double y_ij(size_t i, size_t j, struct game *g) {
  return (g->evaders[j].pos.y - a(g, i, j) * a(g, i, j) * g->pursuers[i].pos.y -
          a(g, i, j) * vec2d_dist_r(&g->pursuers[i].pos, &g->evaders[j].pos)) /
         (1.0 - (a(g, i, j) * a(g, i, j)));
}

/* NOTE: we assume every pair is feasible */
static double y_s(struct game *g, size_t a) {
  double ys = 0.0;
  for (size_t p = 0; p < g->n; p++) {
    ys += y_ij(g->assignments[a][p].i, g->assignments[a][p].j, g);
  }
  return ys;
}

static void compute_aimpoints(size_t i, size_t j, struct game *g, double *xaim,
                              double *yaim) {
  double aij2 = (a(g, i, j) * a(g, i, j));
  double dij = vec2d_dist_r(&g->pursuers[i].pos, &g->evaders[j].pos);

  *xaim = (g->evaders[j].pos.x - aij2 * g->pursuers[i].pos.x) / (1.0 - aij2);
  *yaim =
      (g->evaders[j].pos.y - aij2 * g->pursuers[i].pos.y - a(g, i, j) * dij) /
      (1.0 - aij2);
}

static void game_u(void *x, double dt) {
  unused(dt);
  struct game *game = (struct game *)x;
  double xaim;
  double yaim;
  double y_s_max = -INFINITY;
  double y_s_cur;
  size_t a_max;
  size_t i;
  size_t j;

  for (size_t a = 0; a < game->n_assign; a++) {
    /* Record the largest value and corresponding assignment set */

    y_s_cur = y_s(game, a);
    if (y_s_cur > y_s_max) {
      y_s_max = y_s_cur;
      a_max = a;
    }
  }

  /* Notify the game termination logic of the current assignment */

  opt_assign = a_max;

  /* Using the best found value and assignment, compute opt controls */

  for (size_t pidx = 0; pidx < game->n; pidx++) {
    i = game->assignments[a_max][pidx].i;
    j = game->assignments[a_max][pidx].j;

    /* Compute optimal headings */

    compute_aimpoints(i, j, game, &xaim, &yaim);
    game->pursuers[i].heading =
        atan2(yaim - game->pursuers[i].pos.y, xaim - game->pursuers[i].pos.x);
    game->evaders[j].heading =
        atan2(yaim - game->evaders[j].pos.y, xaim - game->evaders[j].pos.x);
  }
}

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <getopt.h>
#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_vector_double.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_matrix.h>

#include "3dtools.h"
#include "SDL_render.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WINDOW_NAME "Network Connectivity Visualizer"

#define FONT_PATH "/usr/share/fonts/TTF/JetBrainsMonoNerdFont-Regular.ttf"
#define FONT_SIZE (10)

#define SELECT_RADIUS (4) /* m */
#define SELECT_RES (20)

#define LAMBDA (0.32764203)
#define SIGMOID_K (4.0)

#define AGENT_SIZE (3.0) /* m^2 */
#define UAV_VEL (3.0)    /* m / s */

#define randval(min, max) ((min) + (rand() / (RAND_MAX / ((max) - (min)))))

/* SDL mouse button indexes. TODO: SDL surely has definitions for these
 * somewhere
 */

#define LEFT_CLICK (1)
#define RIGHT_CLICK (3)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
  vec3d_t pos;
} agent_t;

typedef struct {
  agent_t *agents;       /* Agent array */
  unsigned n;            /* Number of UAVs */
  unsigned m;            /* Number of ground units */
  unsigned nagents;      /* Number of agents for convenience */
  double ploss_limit;    /* Path loss limit */
  double z_uav;          /* Fixed z-coordinate of UAVs */
  unsigned sel_idx;      /* UAV selected to move with keyboard */
  double scale;          /* Rendering scale */
  vec2d_t screen;        /* Screen resolution scaled */
  vec2d_t screen_scaled; /* Screen resolution scaled */
  vec2d_t center;        /* Screen center */
} gamestate_t;

typedef struct {
  gsl_matrix *proj;      /* Projection matrix */
  gsl_matrix *net_lpl;   /* Network Laplacian */
  gsl_matrix *proj_lpl;  /* Projected Laplacian */
  gsl_matrix *_proj_int; /* Intermediate matrix for projecting net lapl */
  gsl_eigen_symm_workspace *eigw;
  gsl_vector *eigs;
} math_obj_t;

enum dir_e {
  MOVE_UP,
  MOVE_DOWN,
  MOVE_LEFT,
  MOVE_RIGHT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void debug_print_matrix(gsl_matrix *mat) {
  for (size_t i = 0; i < mat->size1; i++) {
    for (size_t j = 0; j < mat->size2; j++) {
      printf("%.2lf  ", gsl_matrix_get(mat, i, j));
    }
    printf("\n");
  }
}

static double ploss_d(double distance) {
  return 20.0 * log10((4 * M_PI / LAMBDA) * distance);
}

static double ploss(const vec3d_t *p1, const vec3d_t *p2) {
  return ploss_d(vec3d_dist_r(p1, p2));
}

static void game_uav_move(gamestate_t *game, enum dir_e dir) {
  agent_t *uav = &game->agents[game->sel_idx];

  /* Move but allow blocked movement by borders */

  switch (dir) {
  case MOVE_UP:
    uav->pos.y -= (UAV_VEL / game->scale);
    if (uav->pos.y < 0.0) uav->pos.y = 0.0;
    break;
  case MOVE_DOWN:
    uav->pos.y += (UAV_VEL / game->scale);
    if (uav->pos.y > game->screen_scaled.y) uav->pos.y = game->screen_scaled.y;
    break;
  case MOVE_LEFT:
    uav->pos.x -= (UAV_VEL / game->scale);
    if (uav->pos.x < 0.0) uav->pos.x = 0.0;
    break;
  case MOVE_RIGHT:
    uav->pos.x += (UAV_VEL / game->scale);
    if (uav->pos.x > game->screen_scaled.x) uav->pos.x = game->screen_scaled.x;
    break;
  }
}

static void game_uav_teleport(gamestate_t *game, double x, double y) {
  if (x > game->screen_scaled.x) {
    game->agents[game->sel_idx].pos.x = game->screen_scaled.x;
  } else if (x < 0.0) {
    game->agents[game->sel_idx].pos.x = 0.0;
  } else {
    game->agents[game->sel_idx].pos.x = x;
  }

  if (y > game->screen_scaled.y) {
    game->agents[game->sel_idx].pos.y = game->screen_scaled.y;
  } else if (y < 0.0) {
    game->agents[game->sel_idx].pos.y = 0.0;
  } else {
    game->agents[game->sel_idx].pos.y = y;
  }
}

static void game_update_scale(gamestate_t *game, double scale) {
  game->scale = scale;
  game->screen_scaled = vec2d_scale_r(&game->screen, 1.0 / game->scale);
  game->center = (vec2d_t)VEC2D_SINIT(game->screen_scaled.x / 2.0,
                                      game->screen_scaled.y / 2.0);
}

static void game_init(gamestate_t *game, SDL_DisplayMode *screen) {
  game->screen = (vec2d_t)VEC2D_SINIT((double)screen->w, (double)screen->h);
  game_update_scale(game, game->scale);

  /* Randomly initialize agent positions within the screen size, centered
   * around the middle of the screen.
   */

  for (unsigned i = 0; i < game->n; i++) {
    game->agents[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->agents[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->agents[i].pos.z = game->z_uav;
  }

  for (unsigned i = game->n; i < game->n + game->m; i++) {
    game->agents[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->agents[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->agents[i].pos.z = 0;
  }

  /* The first UAV is our selected UAV to move */

  game->sel_idx = 0;
}

static void projmat_init(math_obj_t *math, const gamestate_t *game) {
  for (unsigned i = 0; i < game->nagents; i++) {
    for (unsigned k = 0; k < game->nagents - 1; k++) {
      if (i <= k) {
        gsl_matrix_set(math->proj, i, k, 1.0);
      } else if (i == k + 1) {
        gsl_matrix_set(math->proj, i, k, -((double)k + 1.0));
      } else if (i > k + 1) {
        gsl_matrix_set(math->proj, i, k, 0.0);
      }
    }
  }

  /* Normalize each row */

  gsl_vector_view row;
  double norm;

  for (unsigned i = 0; i < math->proj->size1; i++) {
    row = gsl_matrix_row(math->proj, i);
    norm = gsl_blas_dnrm2(&row.vector);
    if (norm > 0.0) {
      gsl_vector_scale(&row.vector, 1.0 / norm);
    }
  }
}

static double sigm_thresh(double x, double t, double k) {
  return 1.0 / (1.0 + exp(-k * (t - x)));
}

static double corr_sigmoid(const vec3d_t *p1, const vec3d_t *p2, double thresh,
                           double k) {
  double corr = ploss_d(thresh);
  double loss = ploss(p1, p2);
  return sigm_thresh(loss, corr, k);
}

static void net_lpl_init(math_obj_t *math, const gamestate_t *game) {
  bool both_evaders;

  /* Populate all of the matrix elements */

  for (unsigned i = 0; i < game->nagents; i++) {
    for (unsigned j = 0; j < game->nagents; j++) {
      both_evaders = i >= game->n && j >= game->n;
      if (!both_evaders && i != j) {
        gsl_matrix_set(math->net_lpl, i, j,
                       -corr_sigmoid(&game->agents[i].pos, &game->agents[j].pos,
                                     game->ploss_limit, SIGMOID_K));
      } else {
        gsl_matrix_set(math->net_lpl, i, j, 0.0);
      }
    }
  }

  /* Sum up the diagonals */

  gsl_vector_view row;
  for (unsigned i = 0; i < game->nagents; i++) {
    row = gsl_matrix_row(math->net_lpl, i);
    gsl_matrix_set(math->net_lpl, i, i, -gsl_vector_sum(&row.vector));
  }
}

static void project_net_lpl(math_obj_t *math) {
  gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, math->proj, math->net_lpl, 0.0,
                 math->_proj_int); /* P^T L */
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, math->_proj_int, math->proj,
                 0.0, math->proj_lpl); /* P^T L P */
}

static double determinant(math_obj_t *math) {
  double det = 1.0;
  for (size_t i = 0; i < math->eigs->size; i++) {
    det *= gsl_vector_get(math->eigs, i);
  }
  return det;
}

static unsigned components(math_obj_t *math) {
  unsigned comp = 1;
  for (size_t i = 0; i < math->eigs->size; i++) {
    if (gsl_vector_get(math->eigs, i) < 1e-3) {
      comp++;
    }
  }
  return comp;
}

static void math_init(math_obj_t *math, gamestate_t const *game) {
  math->net_lpl = gsl_matrix_alloc(game->nagents, game->nagents);
  math->proj = gsl_matrix_alloc(game->nagents, game->nagents - 1);
  projmat_init(math, game); /* This never changes */
  math->_proj_int = gsl_matrix_alloc(math->proj->size2, math->net_lpl->size1);
  math->proj_lpl = gsl_matrix_alloc(game->nagents - 1, game->nagents - 1);
  math->eigw = gsl_eigen_symm_alloc(math->proj_lpl->size1);
  math->eigs = gsl_vector_alloc(math->proj_lpl->size1);
}

static void math_free(math_obj_t *math) {
  gsl_matrix_free(math->net_lpl);
  gsl_matrix_free(math->proj);
  gsl_matrix_free(math->_proj_int);
  gsl_matrix_free(math->proj_lpl);
  gsl_vector_free(math->eigs);
  gsl_eigen_symm_free(math->eigw);
}

static double math_get_obj_value(math_obj_t *math, const gamestate_t *game) {
  net_lpl_init(math, game);
  project_net_lpl(math);
  gsl_eigen_symm(math->proj_lpl, math->eigs, math->eigw); /* Compute eigs */
  return determinant(math);
}

static void draw_agents(SDL_Renderer *renderer, agent_t *agents, unsigned n) {
  for (unsigned i = 0; i < n; i++) {
    SDL_Rect agentbox = {
        .x = agents[i].pos.x - (AGENT_SIZE / 2),
        .y = agents[i].pos.y - (AGENT_SIZE / 2),
        .w = AGENT_SIZE,
        .h = AGENT_SIZE,
    };
    SDL_RenderFillRect(renderer, &agentbox);
  }
}

static void draw_graph(SDL_Renderer *renderer, gamestate_t *game) {
  for (unsigned i = 0; i < game->n; i++) {
    for (unsigned j = 0; j < game->nagents; j++) {
      if (i == j) continue; /* No same-agent consideration */

      if (ploss(&game->agents[i].pos, &game->agents[j].pos) <
          game->ploss_limit) {
        render_line(renderer, &game->agents[i].pos, &game->agents[j].pos);
      }
    }
  }
}

static void draw_stats(SDL_Renderer *renderer, TTF_Font *font,
                       const gamestate_t *game, double value,
                       unsigned components) {
  static const SDL_Color white = {0xff, 0xff, 0xff, SDL_ALPHA_OPAQUE};
  static char text[64];

  snprintf(text, sizeof(text), "Objval: %.2lf\nComponents: %u", value,
           components);

  SDL_Surface *textsurface =
      TTF_RenderText_Solid_Wrapped(font, text, white, game->screen.x);
  SDL_Texture *message = SDL_CreateTextureFromSurface(renderer, textsurface);
  SDL_Rect textrect = {
      .x = 5,
      .y = 5,
      .w = textsurface->w,
      .h = textsurface->h,
  };

  SDL_RenderCopy(renderer, message, NULL, &textrect);

  SDL_FreeSurface(textsurface);
  SDL_DestroyTexture(message);
}

static void draw_compute_heatmap(SDL_Renderer *renderer, math_obj_t *math,
                                 gamestate_t *game) {
  double obj_val;
  double max_obj_val = log10(pow(game->nagents, game->nagents - 1));
  unsigned pixel;

  /* For each pixel in the background, determine the objective value at that
   * spot. Then, draw it with a colour corresponding to the value.
   */

  for (unsigned x = 0; x < game->screen_scaled.x; x++) {
    for (unsigned y = 0; y < game->screen_scaled.y; y++) {
      game->agents[game->sel_idx].pos.x = x;
      game->agents[game->sel_idx].pos.y = y;

      /* Compute the values
       * TODO: this heat map sucks to look at
       * TODO: we have to decide a better upper max on our objective value than
       * 2x our current value or the theoretical max
       * Maybe we need to do a logarithmic gradient instead of a linear one
       */

      obj_val = math_get_obj_value(math, game);
      pixel = (log10(obj_val + 1.0) / max_obj_val) * 255;
      if (pixel > 255) pixel = 255;
      SDL_SetRenderDrawColor(renderer, pixel, 0, 0xff, SDL_ALPHA_OPAQUE);
      SDL_RenderDrawPoint(renderer, x, y);
    }
  }
}

static void uav_pick_best_spot(math_obj_t *math, gamestate_t *game) {
  double obj_val;
  double best_val;
  unsigned best_x = 0;
  unsigned best_y = 0;

  /* For each pixel in the background, determine the objective value at that
   * spot. Then, draw it with a colour corresponding to the value.
   */

  for (unsigned x = 0; x < game->screen_scaled.x; x++) {
    for (unsigned y = 0; y < game->screen_scaled.y; y++) {
      game->agents[game->sel_idx].pos.x = x;
      game->agents[game->sel_idx].pos.y = y;

      /* Compute the values */

      obj_val = math_get_obj_value(math, game);
      if (obj_val > best_val) {
        best_val = obj_val;
        best_x = x;
        best_y = y;
      }
    }
  }

  game->agents[game->sel_idx].pos.x = best_x;
  game->agents[game->sel_idx].pos.y = best_y;
}

static void draw_selector_circle(SDL_Renderer *renderer, gamestate_t *game) {
  vec3d_t *pos = &game->agents[game->sel_idx].pos;
  render_circle(renderer, (vec2d_t *)pos, SELECT_RADIUS, SELECT_RES);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv) {
  gamestate_t gamestate;
  math_obj_t math;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  int mouse_x;
  int mouse_y;
  int randseed = 0;
  double obj_value;
  unsigned n_comp;
  bool seed_provided = false;
  bool running = true;
  bool show_network = false;
  bool mouse_left_pressed = false;
  bool mouse_right_pressed = false;
  bool draw_heatmap = false;

  /* Initialize game state defaults */

  memset(&gamestate, 0, sizeof(gamestate));
  gamestate.ploss_limit = NAN;
  gamestate.z_uav = 10.0;
  gamestate.scale = 4.0;

  /* Parse input arguments */

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:l:z:r:")) != -1) {
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
      gamestate.scale = strtod(optarg, NULL);
      break;
    case 'l':
      gamestate.ploss_limit = strtold(optarg, NULL);
      break;
    case 'r':
      seed_provided = true;
      randseed = strtoul(optarg, NULL, 10);
      break;
    case '?':
      fprintf(stderr, "Unknown option -%c\n", optopt);
      exit(EXIT_FAILURE);
      break;
    }
  }

  /* Parse positional arguments */

  if (argc <= optind) {
    fprintf(stderr, "Missing argument 'n'.\n");
    exit(EXIT_FAILURE);
  }

  gamestate.n = strtoul(argv[optind++], NULL, 10);
  if (gamestate.n < 1) {
    fprintf(stderr, "Invalid number of UAVs: %u\n", gamestate.n);
    exit(EXIT_FAILURE);
  }

  if (optind >= argc) {
    fprintf(stderr, "Missing argument 'm'.\n");
    exit(EXIT_FAILURE);
  }

  gamestate.m = (unsigned)strtoul(argv[optind++], NULL, 10);
  if (gamestate.m < 1) {
    fprintf(stderr, "Invalid number of ground units: %u\n", gamestate.m);
    exit(EXIT_FAILURE);
  }

  gamestate.nagents = gamestate.n + gamestate.m;

  /* Create text rendering tools */

  if (TTF_Init() < 0) {
    fprintf(stderr, "Could not initialize SDL2TTF: %s\n", TTF_GetError());
    exit(EXIT_FAILURE);
  }

  TTF_Font *font = TTF_OpenFont(FONT_PATH, FONT_SIZE);
  if (font == NULL) {
    fprintf(stderr, "Couldn't open font '" FONT_PATH "': %s\n", TTF_GetError());
    exit(EXIT_FAILURE);
  }

  /* Allocate sufficient number of agents */

  gamestate.agents = malloc(sizeof(*gamestate.agents) * gamestate.nagents);
  if (gamestate.agents == NULL) {
    fprintf(stderr, "Failed to allocate agents: %d\n", errno);
    exit(EXIT_FAILURE);
  }

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    fprintf(stderr, "Could not initialize SDL: %s\n", SDL_GetError());
    exit(EXIT_FAILURE);
  }

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create main window renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, gamestate.scale, gamestate.scale);

  /* Randomly initialize the game */

  if (seed_provided != 0) {
    srand(randseed);
  } else {
    srand(time(NULL));
  }

  game_init(&gamestate, &dm);

  math_init(&math, &gamestate);

  /* Simulation loop */

  while (running) {

    /* Check for input events */

    while (SDL_PollEvent(&event)) {

      switch (event.type) {

      case SDL_QUIT:
        running = false;
        break;

        /* Scaling the render */

      case SDL_MOUSEWHEEL:
        gamestate.scale -= (float)(event.wheel.y / 5.0);
        if (gamestate.scale <= 0.1) gamestate.scale = 0.1; /* Trim */
        SDL_RenderSetScale(renderer, gamestate.scale, gamestate.scale);
        game_update_scale(&gamestate, gamestate.scale);
        break;

      case SDL_MOUSEBUTTONDOWN:
        switch (event.button.button) {
        case LEFT_CLICK:
          mouse_left_pressed = true;
          break;
        case RIGHT_CLICK:
          mouse_right_pressed = true;
          break;
        }
        break;

      case SDL_MOUSEBUTTONUP:
        switch (event.button.button) {
        case LEFT_CLICK:
          mouse_left_pressed = false;
          break;
        case RIGHT_CLICK:
          mouse_right_pressed = false;
          break;
        }
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {

        case SDLK_ESCAPE:
        case SDLK_q:
          running = false; /* Quit game */
          break;
        case SDLK_n:
          show_network = !show_network;
          break;
        case SDLK_h:
          draw_heatmap = !draw_heatmap;
          break;
        case SDLK_c:
          /* Put the selected UAV in its best spot */

          uav_pick_best_spot(&math, &gamestate);
          break;
        case SDLK_SPACE:
          game_init(&gamestate, &dm);
          break;

          /* UAV controls */

        case SDLK_w:
          game_uav_move(&gamestate, MOVE_UP);
          break;
        case SDLK_a:
          game_uav_move(&gamestate, MOVE_LEFT);
          break;
        case SDLK_s:
          game_uav_move(&gamestate, MOVE_DOWN);
          break;
        case SDLK_d:
          game_uav_move(&gamestate, MOVE_RIGHT);
          break;

          /* UAV selection */

        case SDLK_UP:
          gamestate.sel_idx = (gamestate.sel_idx + 1) % gamestate.n;
          break;
        case SDLK_DOWN:
          if (gamestate.sel_idx == 0) {
            gamestate.sel_idx = gamestate.n - 1;
          } else {
            gamestate.sel_idx--;
          }
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Compute objective value */

    obj_value = math_get_obj_value(&math, &gamestate);
    n_comp = components(&math);

    /* Process events */

    /* This allows a kind of "click and drag" effect when we have an agent
     * selected
     */

    if (mouse_left_pressed) {
      SDL_GetMouseState(&mouse_x, &mouse_y);
      game_uav_teleport(&gamestate, mouse_x / gamestate.scale,
                        mouse_y / gamestate.scale);
    }

    /* Do rendering stuff */

    /* Clear screen */

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    /* Draw heat map background */

    if (draw_heatmap) {
      agent_t tmp = gamestate.agents[gamestate.sel_idx];
      draw_compute_heatmap(renderer, &math, &gamestate);
      gamestate.agents[gamestate.sel_idx] = tmp;
    }

    /* Draw graph between agents */

    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, SDL_ALPHA_OPAQUE);

    if (show_network) {
      draw_graph(renderer, &gamestate);
    }

    /* Draw selection bubble around selected UAV */

    draw_selector_circle(renderer, &gamestate);

    /* Draw UAVs */

    SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, SDL_ALPHA_OPAQUE);
    draw_agents(renderer, gamestate.agents, gamestate.n);

    /* Draw ground units */

    SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, SDL_ALPHA_OPAQUE);
    draw_agents(renderer, &gamestate.agents[gamestate.n], gamestate.m);

    /* Draw objective value */

    draw_stats(renderer, font, &gamestate, obj_value, n_comp);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);
  }

  /* Release resources */

  math_free(&math);
  free(gamestate.agents);
  TTF_CloseFont(font);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

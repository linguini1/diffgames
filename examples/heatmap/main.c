/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#include "3dtools.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WINDOW_NAME "Network Connectivity Visualizer"

#define SELECT_RADIUS (4) /* m */
#define SELECT_RES (20)

#define LAMBDA (0.32764203)

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
  agent_t *uavs;         /* UAV array */
  agent_t *ground;       /* Ground unit array */
  unsigned n;            /* Number of UAVs */
  unsigned m;            /* Number of ground units */
  double ploss_limit;    /* Path loss limit */
  double z_uav;          /* Fixed z-coordinate of UAVs */
  unsigned sel_idx;      /* UAV selected to move with keyboard */
  double scale;          /* Rendering scale */
  vec2d_t screen;        /* Screen resolution scaled */
  vec2d_t screen_scaled; /* Screen resolution scaled */
  vec2d_t center;        /* Screen center */
} gamestate_t;

enum dir_e {
  MOVE_UP,
  MOVE_DOWN,
  MOVE_LEFT,
  MOVE_RIGHT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static double ploss(const agent_t *a1, const agent_t *a2) {
  return 20.0 * log10((4 * M_PI / LAMBDA) * vec3d_dist_r(&a1->pos, &a2->pos));
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

  /* Inter-UAV connections */

  for (unsigned i = 0; i < game->n; i++) {
    for (unsigned j = 0; j < game->n; j++) {
      if (i == j) continue;

      if (ploss(&game->uavs[i], &game->uavs[j]) < game->ploss_limit) {
        render_line(renderer, &game->uavs[i].pos, &game->uavs[j].pos);
      }
    }
  }

  /* UAV-ground connections */

  for (unsigned i = 0; i < game->n; i++) {
    for (unsigned j = 0; j < game->m; j++) {
      if (ploss(&game->uavs[i], &game->ground[j]) < game->ploss_limit) {
        render_line(renderer, &game->uavs[i].pos, &game->ground[j].pos);
      }
    }
  }
}

static void draw_selector_circle(SDL_Renderer *renderer, gamestate_t *game) {
  vec3d_t *pos = &game->uavs[game->sel_idx].pos;
  render_circle(renderer, (vec2d_t *)pos, SELECT_RADIUS, SELECT_RES);
}

static void game_uav_move(gamestate_t *game, enum dir_e dir) {
  agent_t *uav = &game->uavs[game->sel_idx];

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
    game->uavs[game->sel_idx].pos.x = game->screen_scaled.x;
  } else if (x < 0.0) {
    game->uavs[game->sel_idx].pos.x = 0.0;
  } else {
    game->uavs[game->sel_idx].pos.x = x;
  }

  if (y > game->screen_scaled.y) {
    game->uavs[game->sel_idx].pos.y = game->screen_scaled.y;
  } else if (y < 0.0) {
    game->uavs[game->sel_idx].pos.y = 0.0;
  } else {
    game->uavs[game->sel_idx].pos.y = y;
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
    game->uavs[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->uavs[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->uavs[i].pos.z = game->z_uav;
  }

  for (unsigned i = 0; i < game->m; i++) {
    game->ground[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->ground[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->ground[i].pos.z = 0;
  }

  /* The first UAV is our selected UAV to move */

  game->sel_idx = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv) {
  gamestate_t gamestate;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  int mouse_x;
  int mouse_y;
  int randseed = 0;
  bool seed_provided = false;
  bool running = true;
  bool show_network = false;
  bool mouse_left_pressed = false;
  bool mouse_right_pressed = false;

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

  gamestate.m = strtoul(argv[optind++], NULL, 10);
  if (gamestate.m < 1) {
    fprintf(stderr, "Invalid number of ground units: %u\n", gamestate.m);
    exit(EXIT_FAILURE);
  }

  /* Allocate sufficient number of agents */

  gamestate.uavs = malloc(sizeof(*gamestate.uavs) * gamestate.n);
  if (gamestate.uavs == NULL) {
    fprintf(stderr, "Failed to allocate agents: %d\n", errno);
    exit(EXIT_FAILURE);
  }

  gamestate.ground = malloc(sizeof(*gamestate.ground) * gamestate.n);
  if (gamestate.ground == NULL) {
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

    /* Draw heat map background TODO: */

    /* Draw graph between agents */

    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, SDL_ALPHA_OPAQUE);

    if (show_network) {
      draw_graph(renderer, &gamestate);
    }

    /* Draw selection bubble around selected UAV */

    draw_selector_circle(renderer, &gamestate);

    /* Draw UAVs */

    SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, SDL_ALPHA_OPAQUE);
    draw_agents(renderer, gamestate.uavs, gamestate.n);

    /* Draw ground units */

    SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, SDL_ALPHA_OPAQUE);
    draw_agents(renderer, gamestate.ground, gamestate.m);

    /* TODO: draw objective value */

    /* Show what was drawn */

    SDL_RenderPresent(renderer);
  }

  /* Release resources */

  free(gamestate.uavs);
  free(gamestate.ground);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

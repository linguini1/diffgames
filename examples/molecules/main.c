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
#include <SDL2/SDL_ttf.h>

#include "3dtools.h"
#include "SDL_render.h"
#include "helptext.h"
#include "list.h"
#include "render.h"
#include "utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WINDOW_NAME "Molecules"

#define FONT_PATH "/usr/share/fonts/TTF/JetBrainsMonoNerdFont-Regular.ttf"
#define FONT_SIZE (10)

#define SELECT_RADIUS (4) /* m */
#define SELECT_RES (20)

#define CONN_RADIUS_RES (30)

#define AGENT_SIZE (3.0)        /* m^2 */
#define AGENT_VEL (3.0)         /* m / s */
#define HEADING_VARIATION (0.2) /* rad / s */
#define DT (0.1)                /* s */

#define randval(min, max) ((min) + (rand() / (RAND_MAX / ((max) - (min)))))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum {
  AKIND_GROUND, /* Ground agent */
  AKIND_UAV,    /* UAV */
} agentkind_e;

typedef struct neighbour {
  struct list_node node; /* Linked list node */
  struct agent *agent;   /* An agent in this neighbourhood */
} neighbour_t;

/* An intersection point of 2 transmission radii */

typedef struct {
  vec2d_t pos; /* 2D position of the intersection */
  unsigned within; /* Transmission radii this falls within. */
} intersect_t;

typedef struct agent {
  agentkind_e kind;       /* Agent type */
  vec3d_t pos;            /* Agent position in 3D space */
  neighbour_t neighbours; /* Linked list of connected neighbours */
  double heading;         /* Optional heading for random walking ground units */
} agent_t;

typedef struct {
  agent_t *agents;       /* Agent array */
  unsigned n;            /* Number of UAVs */
  unsigned m;            /* Number of ground units */
  unsigned nagents;      /* Number of agents for convenience */
  double dist_limit;     /* Distance limit */
  double z_uav;          /* Fixed z-coordinate of UAVs */
  double scale;          /* Rendering scale */
  vec2d_t screen;        /* Screen resolution scaled */
  vec2d_t screen_scaled; /* Screen resolution scaled */
  vec2d_t center;        /* Screen center */
  bool randwalk;         /* Ground units move randomly */
} gamestate_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void game_update_scale(gamestate_t *game, double scale) {
  game->scale = scale;
  game->screen_scaled = vec2d_scale_r(&game->screen, 1.0 / game->scale);
  game->center = (vec2d_t)VEC2D_SINIT(game->screen_scaled.x / 2.0,
                                      game->screen_scaled.y / 2.0);
}

static bool agent_is_neighbour_of(const agent_t *potential,
                                  const agent_t *agent) {
  neighbour_t *entry;

  list_for_every_entry(&agent->neighbours.node, entry, neighbour_t, node) {
    if (entry->agent == potential) {
      return true;
    }
  }

  return false;
}

static void agent_move_towards(agent_t *agent, vec3d_t *to) {
  vec3d_t move_vec;
  double mag;

  vec3d_sub(to, &agent->pos, &move_vec);
  move_vec.z = 0.0; /* Agents are stuck in planes */
  mag = vec3d_norm_r(&move_vec);

  agent->pos.x += (move_vec.x / mag) * AGENT_VEL * DT;
  agent->pos.y += (move_vec.y / mag) * AGENT_VEL * DT;
}

static void game_compute_neighbours(gamestate_t *game) {
  neighbour_t *entry;
  neighbour_t *tmp;

  /* NOTE: the approach taken by this function, of updating everyone's neighbour
   * list, implies the ideal medium approach where messages are instantaneously
   * broadcast and received each time this function is called.
   */

  for (unsigned i = 0; i < game->nagents; i++) {

    /* Check all of the neighbours in the existing list of neighbours. If any
     * of them have left the transmission radius, remove them
     */

    list_for_every_entry_safe(&game->agents[i].neighbours.node, entry, tmp,
                              neighbour_t, node) {
      if (vec3d_dist_r(&game->agents[i].pos, &entry->agent->pos)) {
        list_delete(&entry->node);
        free(entry);
      }
    }

    for (unsigned j = 0; j < game->nagents; j++) {
      if (i == j) continue;

      if (vec3d_dist_r(&game->agents[i].pos, &game->agents[j].pos) <
          game->dist_limit) {

        /* If the agent within range is already in the list, then don't add it
         * a second time.
         */

        if (agent_is_neighbour_of(&game->agents[j], &game->agents[i])) continue;

        /* Not a duplicate, add to the list */

        tmp = malloc(sizeof(neighbour_t));
        if (tmp == NULL) {
          fprintf(stderr, "Buy more RAM: %d!\n", errno);
          return;
        }

        list_initialize(&tmp->node);
        tmp->agent = &game->agents[j];
        list_add_after(&game->agents[i].neighbours.node, &tmp->node);
      }
    }
  }
}

static void agent_move(agent_t *agent) {
  /* Move the agent according to the information it has from its neighbours
   * messages.
   */

  /* First, compute the intersection points of the agent's neighbours,
   * pairwise.
   */

  /* Now, determine the set of intersection points that yield the greatest
   * number of new connections (i.e., the points which are within the greatest
   * number of transmission circles/ranges).
   */

  /* Using the set of the optimal intersection points, choose the navigation
   * point for this agent.
   *
   * TODO: do we move towards the average of the optimal points, or do we pick
   * one to move to?
   */
}

static void game_init(gamestate_t *game, SDL_DisplayMode *screen) {
  game->screen = (vec2d_t)VEC2D_SINIT((double)screen->w, (double)screen->h);
  game_update_scale(game, game->scale);

  /* Randomly initialize agent positions within the screen size, centered
   * around the middle of the screen.
   *
   * We initialize the ground agents first so that it is possible for the user
   * to try multiple different simulations with the same configuration of ground
   * agents but a different number of UAVs.
   */

  for (unsigned i = game->n; i < game->n + game->m; i++) {
    game->agents[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->agents[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->agents[i].pos.z = 0;
    game->agents[i].kind = AKIND_GROUND;
    list_initialize(&game->agents[i].neighbours.node);
    if (game->randwalk) {
      game->agents[i].heading = randval(0.0, M_2_PI);
    }
  }

  for (unsigned i = 0; i < game->n; i++) {
    game->agents[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->agents[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->agents[i].pos.z = game->z_uav;
    game->agents[i].kind = AKIND_UAV;
    list_initialize(&game->agents[i].neighbours.node);
  }

  /* TODO: if graph is not connected, we need to tweak the initialization so
   * that agents start connected.
   */

  /* Compute neighbours after init */

  game_compute_neighbours(game);
}

static void draw_agents(SDL_Renderer *renderer, agent_t *agents, unsigned n) {
  for (unsigned i = 0; i < n; i++) {
    SDL_Rect agentbox = {
        .x = agents[i].pos.x - (AGENT_SIZE / 2),
        .y = agents[i].pos.y - (AGENT_SIZE / 2),
        .w = AGENT_SIZE,
        .h = AGENT_SIZE,
    };

    if (agents[i].kind == AKIND_GROUND) {
      SDL_SetRenderDrawColor(renderer, 0, 0xff, 0, SDL_ALPHA_OPAQUE);
    } else {
      SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, SDL_ALPHA_OPAQUE);
    }

    SDL_RenderFillRect(renderer, &agentbox);
  }
}

static void draw_graph(SDL_Renderer *renderer, gamestate_t *game) {
  neighbour_t *entry;

  /* For each agent, draw its connections to its neighbours from its
   * neighbourhood list
   */

  for (unsigned i = 0; i < game->nagents; i++) {
    list_for_every_entry(&game->agents[i].neighbours.node, entry, neighbour_t,
                         node) {
      render_line(renderer, &game->agents[i].pos, &entry->agent->pos);
    }
  }
}

static void draw_radii(SDL_Renderer *renderer, gamestate_t *game) {
  for (unsigned i = 0; i < game->nagents; i++) {
    if (game->agents[i].kind == AKIND_UAV) {
      /* Pale red */

      SDL_SetRenderDrawColor(renderer, 0xff, 0x7f, 0x7f, SDL_ALPHA_OPAQUE);
    } else {
      /* Pale green */

      SDL_SetRenderDrawColor(renderer, 0x7f, 0xff, 0x7f, SDL_ALPHA_OPAQUE);
    }

    render_circle(renderer, (vec2d_t *)&game->agents[i].pos, game->dist_limit,
                  CONN_RADIUS_RES);
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv) {
  gamestate_t gamestate;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  int randseed = 0;
  bool seed_provided = false;
  bool running = true;
  bool show_network = true;
  bool show_radii = false;

  /* Initialize game state defaults */

  memset(&gamestate, 0, sizeof(gamestate));
  gamestate.dist_limit = 50.0;
  gamestate.z_uav = 10.0;
  gamestate.scale = 4.0;
  gamestate.randwalk = false;

  /* Parse input arguments */

  int c;
  while ((c = getopt(argc, argv, ":hmx:y:s:d:z:r:")) != -1) {
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
    case 'd':
      gamestate.dist_limit = strtold(optarg, NULL);
      break;
    case 'z':
      gamestate.z_uav = strtold(optarg, NULL);
      break;
    case 'm':
      gamestate.randwalk = true;
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

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {

        case SDLK_ESCAPE:
        case SDLK_q:
          running = false; /* Quit game */
          break;
        case SDLK_n:
          show_network = !show_network;
          break;
        case SDLK_r:
          show_radii = !show_radii;
          break;
        case SDLK_SPACE:
          game_init(&gamestate, &dm);
          break;
        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Perform game updates */

    game_compute_neighbours(&gamestate);

    /* Do rendering stuff */

    /* Clear screen */

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);

    /* Draw graph between agents */

    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, SDL_ALPHA_OPAQUE);

    if (show_network) {
      draw_graph(renderer, &gamestate);
    }

    /* Show connection radii */

    if (show_radii) {
      draw_radii(renderer, &gamestate);
    }

    /* Draw all agents */

    draw_agents(renderer, gamestate.agents, gamestate.nagents);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);
  }

  /* Release resources */

  free(gamestate.agents);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

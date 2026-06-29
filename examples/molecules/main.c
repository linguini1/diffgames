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

#define LAMBDA (0.32764203)
#define SIGMOID_K (4.0)

#define AGENT_SIZE (3.0) /* m^2 */
#define AGENT_VEL (3.0)  /* m / s */
#define DT (0.1)

#define randval(min, max) ((min) + (rand() / (RAND_MAX / ((max) - (min)))))

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum {
  AKIND_GROUND, /* Ground agent */
  AKIND_UAV,    /* UAV */
} agentkind_e;

/* Statuses with greater numbers have higher priority */

typedef enum {
  STAT_ISOLATED = 0, /* Not connected to any ground agent */
  STAT_HOP = 1,      /* Connected to a ground agent indirectly */
  STAT_TETHERED = 2, /* Directly connected to a ground agent */
} status_e;

typedef struct neighbour {
  struct list_node node; /* Linked list node */
  struct agent *agent;   /* An agent in this neighbourhood */
} neighbour_t;

typedef struct agent {
  agentkind_e kind;       /* Agent type */
  status_e status;        /* Agent connection status */
  vec3d_t pos;            /* Agent position in 3D space */
  neighbour_t neighbours; /* Linked list of connected neighbours */
} agent_t;

typedef struct {
  agent_t *agents;       /* Agent array */
  unsigned n;            /* Number of UAVs */
  unsigned m;            /* Number of ground units */
  unsigned nagents;      /* Number of agents for convenience */
  double ploss_limit;    /* Path loss limit */
  double z_uav;          /* Fixed z-coordinate of UAVs */
  double scale;          /* Rendering scale */
  vec2d_t screen;        /* Screen resolution scaled */
  vec2d_t screen_scaled; /* Screen resolution scaled */
  vec2d_t center;        /* Screen center */
} gamestate_t;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static double ploss_d(double distance) {
  return 20.0 * log10((4 * M_PI / LAMBDA) * distance);
}

static double inverse_ploss(double ploss) {
  return pow(10.0, ploss / 20.0) / (4 * M_PI / LAMBDA);
}

static double ploss(const vec3d_t *p1, const vec3d_t *p2) {
  return ploss_d(vec3d_dist_r(p1, p2));
}

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

static void agent_move(agent_t *agent, gamestate_t *game) {
  agent_t *best_agent;
  neighbour_t *entry;
  size_t len;
  size_t total_vecs;
  vec3d_t barycenter = VEC3D_SINIT(0.0, 0.0, 0.0);
  vec3d_t scaled_pos;
  double best_dist = INFINITY;
  double dist;

  /* If we're a ground agent, we don't move (yet) */

  if (agent->kind == AKIND_GROUND) return;

  /* Move the UAV according to the algorithm
   *
   * Movement choice is as follows:
   *
   * - If we are TETHERED/HOP and a leaf node, we should move around the radius
   *   of our sole neighbour in the direction of other agents within reach, or
   *   the nearest agent if agents are out of reach. If there are multiple
   *   agents within reach, pick the one with the highest status or who is a
   *   ground unit.
   *
   * - If we are a TETHERED/HOP agent who is not a leaf, we should try to
   *   maintain our connected position.
   *
   * - If we are an ISOLATED agent, we should move towards the nearest
   *   non-isolated or ground agent.
   */

  switch (agent->status) {
  case STAT_ISOLATED:

    /* Find the nearest non-isolated or ground agent */

    for (unsigned i = 0; i < game->nagents; i++) {

      if (&game->agents[i] == agent) continue; /* Don't pick ourselves */

      if (game->agents[i].kind == AKIND_UAV &&
          game->agents[i].status == STAT_ISOLATED)
        continue;

      dist = vec3d_dist_r(&game->agents[i].pos, &agent->pos);

      if (dist < best_dist) {
        best_agent = &game->agents[i];
        best_dist = dist;
      }
    }

    agent_move_towards(agent, &best_agent->pos);
    break;

  case STAT_TETHERED:
  case STAT_HOP:

    /* If we're not a leaf node, move to the barycenter of our neighbours,
     * restricted however to maintaining any existing connection to a ground
     * agent.
     */

    len = list_length(&agent->neighbours.node);
    if (len > 1) {

      /* Compute the barycenter, giving ground agents significantly more weight.
       */

      total_vecs = 0;
      list_for_every_entry(&agent->neighbours.node, entry, neighbour_t, node) {
        if (entry->agent->kind == AKIND_GROUND) {
          vec3d_scale(&entry->agent->pos, len, &scaled_pos);
          total_vecs += len;
        } else {
          scaled_pos = entry->agent->pos;
          total_vecs++;
        }

        vec3d_add(&barycenter, &scaled_pos, &barycenter);
      }

      barycenter = vec3d_scale_r(&barycenter, 1.0 / total_vecs);
      agent_move_towards(agent, &barycenter);
      break;
    }

    /* If we are a leaf node, let's stay within our one neighbour's radius and
     * try to move towards the nearest ground agent.
     *
     * NOTE: originally, this rule was to move towards the nearest agent who is
     * not us or our one neighbour. However, moving towards only ground agents
     * should improve the number of ground agents we manage to connect and
     * should be comparable in performance to the original rule (since other
     * agents would likely be intercepted on the way to a ground agent).
     */

    for (unsigned i = 0; i < game->nagents; i++) {

      /* Don't pick ourselves or our single neighbour */

      if (game->agents[i].kind != AKIND_GROUND) continue;

      dist = vec3d_dist_r(&game->agents[i].pos, &agent->pos);

      if (dist < best_dist) {
        best_agent = &game->agents[i];
        best_dist = dist;
      }
    }

    agent_move_towards(agent, &best_agent->pos);
    break;
  }
}

static void game_compute_neighbours(gamestate_t *game) {
  neighbour_t *entry;
  neighbour_t *tmp;

  for (unsigned i = 0; i < game->nagents; i++) {

    /* Check all of the neighbours in the existing list of neighbours. If any
     * of them have left the transmission radius, remove them
     */

    list_for_every_entry_safe(&game->agents[i].neighbours.node, entry, tmp,
                              neighbour_t, node) {
      if (ploss(&game->agents[i].pos, &entry->agent->pos)) {
        list_delete(&entry->node);
        free(entry);
      }
    }

    for (unsigned j = 0; j < game->nagents; j++) {
      if (i == j) continue;

      if (ploss(&game->agents[i].pos, &game->agents[j].pos) <
          game->ploss_limit) {

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

static void game_compute_statuses(gamestate_t *game) {
  neighbour_t *entry;
  bool all_isolated;

  for (unsigned i = 0; i < game->nagents; i++) {

    /* If this agent has no neighbours, it is trivially isolated */

    if (list_is_empty(&game->agents[i].neighbours.node)) {
      game->agents[i].status = STAT_ISOLATED;
      continue;
    }

    all_isolated = true;

    list_for_every_entry(&game->agents[i].neighbours.node, entry, neighbour_t,
                         node) {

      /* Neighbour is ground unit, this agent is tethered */

      if (entry->agent->kind == AKIND_GROUND) {
        game->agents[i].status = STAT_TETHERED;
        all_isolated = false;

        /* NOTE: We do not inform neighbours that we are tethered. On the next
         * message passing iteration, they will realize our status was updated.
         */

        break;
      }

      /* Neighbour is TETHERED or HOP, this agent is HOP */

      if (entry->agent->status == STAT_TETHERED ||
          entry->agent->status == STAT_HOP) {
        switch (game->agents[i].status) {
        case STAT_ISOLATED:
        case STAT_HOP:
          game->agents[i].status = STAT_HOP;
          all_isolated = false;
          break;
        default:
          break; /* Tethered overrules hop */
        }
      }
    }

    /* If all neighbours are isolated, this agent is isolated */

    if (all_isolated) {
      game->agents[i].status = STAT_ISOLATED;
      /* TODO: we need to broadcast when we lose a neighbour that contributes to
       * our state so our neighbours can update their state accordingly.
       *
       * I.e. if we were previously HOP, but we lose connection with our
       * neighbour that was our connection to a ground agent, we should no
       * longer be HOP but ISOLATED. However, we may get confused and believe
       * we're still hop because our other neighbour who is a leaf node was
       * recorded as HOP because of us.
       */
    }
  }
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
    game->agents[i].status = STAT_ISOLATED;
    list_initialize(&game->agents[i].neighbours.node);
  }

  for (unsigned i = 0; i < game->n; i++) {
    game->agents[i].pos.x = randval(0.5 * game->center.x, 1.5 * game->center.x);
    game->agents[i].pos.y = randval(0.5 * game->center.y, 1.5 * game->center.y);
    game->agents[i].pos.z = game->z_uav;
    game->agents[i].kind = AKIND_UAV;
    game->agents[i].status = STAT_ISOLATED;
    list_initialize(&game->agents[i].neighbours.node);
  }

  /* Compute neighbours after init */

  game_compute_neighbours(game);
  game_compute_statuses(game);
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
      switch (agents[i].status) {
      case STAT_ISOLATED:
        SDL_SetRenderDrawColor(renderer, 0xff, 0, 0, SDL_ALPHA_OPAQUE);
        break;
      case STAT_HOP:
        SDL_SetRenderDrawColor(renderer, 0xff, 0, 0xff, SDL_ALPHA_OPAQUE);
        break;
      case STAT_TETHERED:
        SDL_SetRenderDrawColor(renderer, 0, 0, 0xff, SDL_ALPHA_OPAQUE);
        break;
      }
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
  double radius = inverse_ploss(game->ploss_limit);
  for (unsigned i = 0; i < game->n; i++) {
    render_circle(renderer, (vec2d_t *)&game->agents[i].pos, radius,
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
  gamestate.ploss_limit = 65.0;
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

    for (unsigned i = 0; i < gamestate.nagents; i++) {
      agent_move(&gamestate.agents[i], &gamestate);
    }

    game_compute_neighbours(&gamestate);
    game_compute_statuses(&gamestate);

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

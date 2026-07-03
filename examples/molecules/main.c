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

/* An intersection point of 2 transmission radii.
 *
 * NOTE: we are dealing with intersections of spheres. When we check their
 * intersection, we need to look at the 2D projection of the sphere into the
 * moving agent's z-plane. Otherwise, we will consider intersection points that
 * are not accessible. Therefore, the `pos` member of this struct is the 2D
 * position of the intersection point considering the moving agent's z-plane.
 */

typedef struct {
  vec3d_t pos;     /* 2D position of the intersection, where z coordinate is the
                      plane the intersection was considered on. */
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
  double trans_radius;   /* Distance limit */
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

/* Determine the radius of the cross-section of a sphere of radius `r` centered
 * at plane `z` when 'sliced' by plane `new_z`.
 */

static double projected_radius(double r, double old_z, double new_z) {
  double height = fabs(old_z - new_z);
  return sqrt((r * r) - (height * height));
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

static void game_compute_neighbours(gamestate_t *game) {
  neighbour_t *entry;
  neighbour_t *tmp;

  /* NOTE: the approach taken by this function, of updating everyone's neighbour
   * list, implies the ideal medium approach where messages are instantaneously
   * broadcast and received each time this function is called.
   *
   */

  for (unsigned i = 0; i < game->nagents; i++) {

    /* Clear list
     * NOTE: Since we have to check this agent against all other agents to
     * determine what's in the transmission range, we might as well redo the
     * neighbour list from scratch each time.
     */

    list_for_every_entry_safe(&game->agents[i].neighbours.node, entry, tmp,
                              neighbour_t, node) {
      if (vec3d_dist_r(&game->agents[i].pos, &entry->agent->pos) >
          game->trans_radius) {
        list_delete(&entry->node);
        free(entry);
      }
    }

    list_initialize(&game->agents[i].neighbours.node);

    /* Check all of the neighbours in the existing list of neighbours. If any
     * of them have left the transmission radius, remove them
     */

    for (unsigned j = 0; j < game->nagents; j++) {
      if (i == j) continue;

      if (vec3d_dist_r(&game->agents[i].pos, &game->agents[j].pos) <
          game->trans_radius) {

        /* Within range and cannot be a duplicate due to the way we've iterated;
         * add to the list.
         */

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

static int intersect_sorter(const void *a, const void *b) {
  return ((intersect_t *)(b))->within - ((intersect_t *)(a))->within;
}

/*
 * Returns false if there is no intersection. If there is an intersection, the
 * points are returned in the `points` array.
 *
 * One point of intersection will cause both points to have the same value.
 */
static bool circle_intersection(vec2d_t *p1, vec2d_t *p2, double r1, double r2,
                                vec2d_t points[2]) {
  double dist;
  double a;
  double h;
  double x3;
  double y3;

  vec2d_dist(p1, p2, &dist);

  if (dist > r1 + r2) return false; /* No intersection */

  /* Adapted from:
   * https://stackoverflow.com/questions/3349125/circle-circle-intersection-points
   */

  a = (r1 * r1 - r2 * r2 + dist * dist) / (2 * dist);
  h = sqrt(r1 * r1 - a * a);

  x3 = p1->x + a * (p2->x - p1->x) / dist;
  y3 = p1->y + a * (p2->y - p1->y) / dist;

  points[0].x = x3 + h * (p2->y - p1->y) / dist;
  points[1].x = x3 - h * (p2->y - p1->y) / dist;

  points[0].y = y3 - h * (p2->x - p1->x) / dist;
  points[1].y = y3 + h * (p2->x - p1->x) / dist;

  return true;
}

/* Move the agent according to the information it has from its neighbours
 * messages. `trans_radius` is the transmission distance set by the game
 * globally.
 *
 * NOTE: this function assumes that any agent with at least one neighbour has
 * intersection points to move towards. This is WRONG. It is possible for one
 * agent with a smaller radius (i.e. ground agent due to projection) to have
 * its transmission radius fully contained within the agent of interest's
 * transmission radius, therefore yielding no intersection points. We need to
 * decide what to do in this case.
 *
 * NOTE: this function does not handle two unique agents who occupy the same
 * point in space. This results in infinitely many intersection points. This
 * case should be explicitly handled.
 */

static void agent_move(agent_t *agent, double trans_radius) {
  neighbour_t *n1;
  neighbour_t *n2;
  double r1;
  double r2;
  vec3d_t toward;
  vec2d_t points[2];
  unsigned nneighbours;

  trans_radius *= 0.98; /* Help overshooting */

  /* We'll assume 32 is enough for now; I don't wanna deal with dynamic arrays
   * yet.
   */

  static intersect_t intersections[64];
  unsigned count = 0; /* Number of intersection points found so far */

  if (agent->kind == AKIND_GROUND) return; /* These agents use random walk */

  nneighbours = list_length(&agent->neighbours.node);

  /* If this agent has no neighbours, then we should just move somewhere
   * randomly.
   */

  if (nneighbours == 0) {
    return; /* TODO: randomly move */
  }

  /* If we only have one neighbour, we should repel away from it (but within the
   * transmission radius) to maximize area covered by both.
   *
   * We get the vector pointing towards the neighbour and our distance from the
   * neighbour, then scale the vector in the reverse direction toward the circle
   * extremity. We then move to that point.
   */

  if (nneighbours == 1) {
    n1 = list_first_entry(&agent->neighbours.node, neighbour_t, node);
    vec2d_sub(&agent->pos, &n1->agent->pos, &toward);
    r1 = projected_radius(trans_radius, n1->agent->pos.z, agent->pos.z);
    r2 = vec2d_norm_r((vec2d_t *)&toward); /* r2 holds distance to neighbour */

    /* Scale by the remaining radius and then offset by neighbour position to
     * make it once again an absolute position instead of a relative vector.
     */

    vec2d_scale(&toward, r1 / r2, &toward);
    vec2d_add(&toward, &n1->agent->pos, &toward);
    toward.z = agent->pos.z; /* Stay within the plane */

    /* If we are already on the radius, we should now rotate around our
     * neighbour by following the tangent vector.
     */

    if (vec2d_dist_r((vec2d_t *)&toward, (vec2d_t *)&agent->pos) <= 0.4) {
      vec2d_sub(&agent->pos, &n1->agent->pos, &toward);
      r2 = atan2(toward.y, toward.x);
      r2 += 0.1; /* Increase by 0.1 radians to spin */
      toward.x = n1->agent->pos.x + r1 * cos(r2);
      toward.y = n1->agent->pos.y + r1 * sin(r2);
    }

    agent_move_towards(agent, &toward);

    return;
  }

  /* Next, compute the intersection points of the agent's neighbours,
   * pairwise.
   */

  list_for_every_entry(&agent->neighbours.node, n1, neighbour_t, node) {
    list_for_every_entry(&agent->neighbours.node, n2, neighbour_t, node) {
      if (n1->agent == n2->agent) continue; /* Skip agents who are the same */

      /* Determine intersection point(s) of the transmission radii in this
       * agent's z-plane using the position of the neighbours.
       */

      /* Project transmission radii to the plane of the moving agent */

      r1 = projected_radius(trans_radius, n1->agent->pos.z, agent->pos.z);
      r2 = projected_radius(trans_radius, n2->agent->pos.z, agent->pos.z);

      if (!circle_intersection((vec2d_t *)&n1->agent->pos,
                               (vec2d_t *)&n2->agent->pos, r1, r2, points)) {
        continue; /* No intersection */
      }

      /* Append the first point */

      assert(count < array_len(intersections));
      intersections[count].pos.x = points[0].x;
      intersections[count].pos.y = points[0].y;
      intersections[count].within = 0;
      count++;

      if (fabs(points[0].x - points[1].x) <= 1e-5 &&
          fabs(points[0].y - points[1].y) <= 1e-5) {
        continue; /* If points are identical, we're done */
      }

      /* Otherwise, add the second unique point */

      assert(count < array_len(intersections));
      intersections[count].pos.x = points[1].x;
      intersections[count].pos.y = points[1].y;
      intersections[count].within = 0;
      count++;
    }
  }

  /* TODO: if there are no intersection points (possible, all neighbouring
   * agents' transmission radii are subsets of this agent's transmission
   * radius), we need to decide what to do.
   *
   * Should we move in a direction such that our furthest neighbour is pushed
   * towards the extremity of our radius?
   *
   * For now, we will just move towards the first neighbour. TODO: change this.
   */

  if (count == 0) {
    n1 = list_first_entry(&agent->neighbours.node, neighbour_t, node);
    agent_move_towards(agent, &n1->agent->pos);
    return;
  }

  /* Now, determine the set of intersection points that yield the greatest
   * number of new connections (i.e., the points which are within the greatest
   * number of transmission circles/ranges).
   */

  for (unsigned i = 0; i < count; i++) {
    list_for_every_entry(&agent->neighbours.node, n1, neighbour_t, node) {
      /* Check if this point is within this neighbour's transmission radius */

      r1 = projected_radius(trans_radius, n1->agent->pos.z, agent->pos.z);
      if (vec2d_dist_r((vec2d_t *)&agent->pos,
                       (vec2d_t *)&intersections[i].pos) <= r1) {
        intersections[i].within++;
      }
    }
  }

  /* Sort the intersection points in descending order by `within` field */

  qsort(intersections, array_len(intersections), sizeof(intersections[0]),
        intersect_sorter);

  /* Using the set of the optimal intersection points, choose the navigation
   * point for this agent.
   *
   * TODO: do we move towards the average of the optimal points, or do we pick
   * one to move to?
   *
   * FOR NOW: pick the first one
   */

  agent_move_towards(agent, &intersections[0].pos);
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
  /* We render the radii of ground agents from the perspective of the UAV
   * plane (bird's eye view). From this plane, the transmission radius of the
   * of the ground agents will appear smaller since it must travel some
   * non-zero z-height upwards. (i.e. we are looking at a cross-section of the
   * sphere).
   */

  double ground_radius = projected_radius(game->trans_radius, 0.0, game->z_uav);

  for (unsigned i = 0; i < game->nagents; i++) {
    if (game->agents[i].kind == AKIND_UAV) {
      SDL_SetRenderDrawColor(renderer, 0xff, 0x7f, 0x7f, SDL_ALPHA_OPAQUE);
      render_circle(renderer, (vec2d_t *)&game->agents[i].pos,
                    game->trans_radius, CONN_RADIUS_RES);
    } else {
      SDL_SetRenderDrawColor(renderer, 0x7f, 0xff, 0x7f, SDL_ALPHA_OPAQUE);
      render_circle(renderer, (vec2d_t *)&game->agents[i].pos, ground_radius,
                    CONN_RADIUS_RES);
    }
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
  gamestate.trans_radius = 50.0;
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
      gamestate.trans_radius = strtold(optarg, NULL);
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

    for (unsigned i = 0; i < gamestate.nagents; i++) {
      agent_move(&gamestate.agents[i], gamestate.trans_radius);
    }

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

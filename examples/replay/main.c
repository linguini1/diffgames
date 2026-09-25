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

const char WINDOW_NAME[] = "Replay";
const SDL_Colour BLACK = {0, 0, 0, SDL_ALPHA_OPAQUE};
const SDL_Colour WHITE = {255, 255, 255, SDL_ALPHA_OPAQUE};

static SDL_Colour bgcol;
static SDL_Colour fgcol;

#define LAMBDA (0.32764203)

/* Represents a single agent with a position in space */

typedef struct {
  size_t id;
  vec3d_t pos;
} agent_t;

static double ploss(const agent_t *a1, const agent_t *a2) {
  return 20.0 * log10((4 * M_PI / LAMBDA) * vec3d_dist_r(&a1->pos, &a2->pos));
}

static double inverse_ploss(double ploss) {
  return pow(10.0, ploss / 20.0) / (4 * M_PI / LAMBDA);
}

static char *parse_param(char *line) {
  char *token = strtok(line, "=");
  if (token == NULL) return token;
  return strtok(NULL, "=");
}

void render_agents(SDL_Renderer *renderer, agent_t *agents, size_t n, size_t m,
                   vec2d_t *screen_offset, double scale);
void render_graph(SDL_Renderer *renderer, agent_t *agents, size_t n, size_t m,
                  vec2d_t *screen_offset, double scale, double ploss_limit);

static void tikz_render(FILE *sink, const char *fpath, const agent_t *agents,
                        size_t n, size_t m, double ploss_limit, double r_max,
                        double scale);

static int parse_record(char *buf, agent_t *agent);

int main(int argc, char **argv) {
  double ploss_limit = NAN;
  double scale = 5.0;
  double r_max = NAN;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  const char *filename = NULL;
  char buf[BUFSIZ];
  size_t n = 0;
  size_t m = 0;
  size_t timestep = 0;
  bool running = true;
  bool game_over = false;
  bool exit_on_completion = false;
  bool show_network = false;
  bool mouse_pressed = false;
  bool paused = false;
  vec2d_t mouse_start;
  vec2d_t mouse_end;
  vec2d_t mouse_offset = VEC2D_SINIT(0.0, 0.0);
  vec2d_t perm_offset = VEC2D_SINIT(0.0, 0.0);
  vec2d_t comb_offset;
  int mouse_x;
  int mouse_y;

  bgcol = BLACK;
  fgcol = WHITE;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:f:t:l:e")) != -1) {
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
      scale = strtod(optarg, NULL);
      break;
    case 'f':
      filename = optarg;
      break;
    case 't':
      timestep = strtoul(optarg, NULL, 10);
      break;
    case 'l':
      ploss_limit = strtold(optarg, NULL);
      break;
    case 'e':
      exit_on_completion = true;
      break;
    case '?':
      fprintf(stderr, "Unknown option -%c\n", optopt);
      exit(EXIT_FAILURE);
      break;
    }
  }

  /* Check if file was valid and open the file */

  if (filename == NULL) {
    fprintf(stderr, "No replay file selected.\n");
    return EXIT_FAILURE;
  }

  FILE *file = fopen(filename, "r");
  if (file == NULL) {
    fprintf(stderr, "Could not open file: %d\n", errno);
    return EXIT_FAILURE;
  }

  /* Get simulation parameters needed for rendering from the simulation file:
   * TODO: no error handling,
   */

  char *token;

  fgets(buf, sizeof(buf), file);
  token = parse_param(buf);
  if (token == NULL) {
    fprintf(stderr, "Expected token for 'n', got nothing.\n");
    return EXIT_FAILURE;
  }
  n = strtoul(token, NULL, 10);

  fgets(buf, sizeof(buf), file);
  token = parse_param(buf);
  if (token == NULL) {
    fprintf(stderr, "Expected token for 'm', got nothing.\n");
    return EXIT_FAILURE;
  }
  m = strtoul(token, NULL, 10);

  fgets(buf, sizeof(buf), file);
  token = parse_param(buf);
  if (token == NULL) {
    fprintf(stderr,
            "Expected token for r_max for this simulation, skipping render.\n");
    return EXIT_FAILURE;
  }
  r_max = strtold(token, NULL);

  /* Skip z_ground, z_uav */

  fgets(buf, sizeof(buf), file);
  fgets(buf, sizeof(buf), file);

  fgets(buf, sizeof(buf), file);
  token = parse_param(buf);
  if (isnan(ploss_limit) && token != NULL) {
    ploss_limit = strtold(token, NULL);
  }

  /* Skip dt, weight, kron */

  fgets(buf, sizeof(buf), file);
  fgets(buf, sizeof(buf), file);
  fgets(buf, sizeof(buf), file);

  if (n <= 0 || m <= 0) {
    fprintf(stderr, "Invalid premise, n or m is <= 0\n");
    fclose(file);
    return EXIT_FAILURE;
  }

  /* Allocate arrays for agents */

  agent_t *agents = malloc(sizeof(agent_t) * (n + m));
  if (agents == NULL) {
    fprintf(stderr, "Could not allocate memory for agents.\n");
    fclose(file);
    return EXIT_FAILURE;
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

  /* Create main window renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Create texture for particle trailing effect */

  SDL_Texture *agent_txtr = SDL_CreateTexture(
      renderer, tempdm.format, SDL_TEXTUREACCESS_TARGET, dm.w, dm.h);

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
        scale -= (float)(event.wheel.y / 5.0);
        if (scale <= 0.1) scale = 0.1; /* Trim */
        break;

      case SDL_MOUSEBUTTONDOWN:
        if (!mouse_pressed) {
          SDL_GetMouseState(&mouse_x, &mouse_y);
          vec2d_init(&mouse_start, mouse_x, mouse_y);
          vec2d_init(&mouse_end, mouse_x, mouse_y);
        }
        mouse_pressed = true;
        break;

      case SDL_MOUSEBUTTONUP:
        /* Store the current mouse offset as the permanent offset, reset the
         * mouse offset for next time.
         */

        perm_offset = comb_offset;
        vec2d_init(&mouse_offset, 0, 0);
        mouse_pressed = false;
        break;

      case SDL_KEYDOWN:
        switch (event.key.keysym.sym) {

        case SDLK_ESCAPE:
        case SDLK_q:
          running = false;
          break;
        case SDLK_p:
          paused = !paused;
          break;
        case SDLK_i:
          if (bgcol.r == 0) {
            bgcol = WHITE;
            fgcol = BLACK;
          } else {
            bgcol = BLACK;
            fgcol = WHITE;
          }
          break;
        case SDLK_n:
          show_network = !show_network;
          break;
        case SDLK_SPACE:
          fseek(file, 0, SEEK_SET);
          fgets(buf, sizeof(buf), file); /* Skip headers */
          game_over = false;
          SDL_SetRenderDrawColor(renderer, bgcol.r, bgcol.g, bgcol.b, bgcol.a);
          SDL_SetRenderTarget(renderer, agent_txtr); /* Clear agents */
          SDL_RenderClear(renderer);
          SDL_SetRenderTarget(renderer, NULL); /* Clear window */
          SDL_RenderClear(renderer);
          SDL_RenderPresent(renderer);
          break;

        case SDLK_r:
          /* Render the current game state as a tikz figure */

          tikz_render(stdout, filename, agents, n, m, ploss_limit, r_max,
                      scale);
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    /* Modify offset of the map using user set mouse location */

    if (mouse_pressed) {
      SDL_GetMouseState(&mouse_x, &mouse_y);
      vec2d_init(&mouse_end, mouse_x, mouse_y);
      vec2d_sub(&mouse_end, &mouse_start, &mouse_offset); /* Update offset */
    }

    /* Compute current offset */

    vec2d_add(&perm_offset, &mouse_offset, &comb_offset);

    /* Populate agents with current time step */

    for (size_t i = 0; i < n + m && !game_over && !paused; i++) {
      fgets(buf, sizeof(buf), file);
      if (feof(file)) {
        game_over = true;
        break;
      }
      parse_record(buf, &agents[i]);
    }

    if (!paused) {

      /* Render agents with trails */

      SDL_SetRenderTarget(renderer, agent_txtr); /* Switch to agent texture */

      SDL_SetRenderDrawColor(renderer, bgcol.r, bgcol.g, bgcol.b, 10);
      SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
      SDL_RenderFillRect(renderer, &fullscreen);

      render_agents(renderer, agents, n, m, &comb_offset, scale);
      SDL_SetRenderTarget(renderer, NULL); /* Switch back to window */

      /* Clear window renderer entirely with black */

      SDL_SetRenderTarget(renderer, NULL);
      SDL_SetRenderDrawColor(renderer, bgcol.r, bgcol.g, bgcol.b, bgcol.a);
      SDL_RenderClear(renderer);

      /* Copy over agent render on top of the graph render */

      SDL_RenderCopy(renderer, agent_txtr, NULL, &fullscreen);

      /* Draw scale for 100m */

      SDL_SetRenderDrawColor(renderer, fgcol.r, fgcol.g, fgcol.b, fgcol.a);
      SDL_RenderDrawLine(renderer, 3, 3, 3 + 100 / scale, 3);

      /* Draw the evader radius if this simulation has a bounded play region. */

      if (!isnan(r_max)) {
        vec2d_t center = comb_offset;
        center.x /= scale;
        center.y /= scale;
        render_circle(renderer, &center, r_max / scale, 50);
      }

      /* Render network graph if selected to show the network */

      if (show_network) {
        render_graph(renderer, agents, n, m, &comb_offset, scale, ploss_limit);

        /* We also draw just the most recent agent positions over top */
        render_agents(renderer, agents, n, m, &comb_offset, scale);
      }
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation until a capture occurs */

    usleep(timestep);

    /* Check if game is over and exit */

    if (exit_on_completion && game_over) break;
  }

  /* Release resources */

  fclose(file);
  free(agents);
  SDL_DestroyTexture(agent_txtr);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

static int parse_record(char *buf, agent_t *agent) {
  char *token;

  token = strtok(buf, ",");
  if (token == NULL) return -1;

  agent->id = strtoul(token, NULL, 10);

  for (int i = 0; i < 3; i++) {
    token = strtok(NULL, ",");
    if (token == NULL) return -1;
    ((double *)(&agent->pos))[i] = strtold(token, NULL);
  }

  return 0;
}

void render_agents(SDL_Renderer *renderer, agent_t *agents, size_t n, size_t m,
                   vec2d_t *screen_offset, double scale) {
  vec2d_t ai;
  vec2d_t aj;

  /* Draw pursuers in red on agent texture */

  SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);

  for (size_t i = 0; i < n; i++) {
    vec2d_add(&agents[i].pos, screen_offset, &ai);
    ai.x /= scale;
    ai.y /= scale;
    render_vec2d(renderer, &ai);
  }

  /* Draw evaders in green on agent texture */

  SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);

  for (size_t j = n; j < n + m; j++) {
    vec2d_add(&agents[j].pos, screen_offset, &aj);
    aj.x /= scale;
    aj.y /= scale;
    render_vec2d(renderer, &aj);
  }
}

void render_graph(SDL_Renderer *renderer, agent_t *agents, size_t n, size_t m,
                  vec2d_t *screen_offset, double scale, double ploss_limit) {
  vec2d_t ai;
  vec2d_t aj;

  /* Draw graph connections in white to the window renderer */

  SDL_SetRenderDrawColor(renderer, fgcol.r, fgcol.g, fgcol.b, fgcol.a);

  /* NOTE: When rendering, we assume that each agent can be connected to each
   * other agent (i.e., evader-evader pairings are now allowed)
   */

  for (size_t i = 0; i < n + m; i++) {
    for (size_t j = 0; j < n + m; j++) {
      if (i == j) continue; /* No self-self considerations */

      /* Modify render points */

      vec2d_add(&agents[i].pos, screen_offset, &ai);
      vec2d_add(&agents[j].pos, screen_offset, &aj);
      ai.x /= scale;
      ai.y /= scale;
      aj.x /= scale;
      aj.y /= scale;

      /* Calculate path loss and draw a line if within limit */

      if (ploss(&agents[i], &agents[j]) <= ploss_limit) {
        render_line(renderer, &ai, &aj);
      }
    }
  }
}

static void tikz_agent_ref(FILE *sink, const agent_t *agent, bool pursuer) {
  if (pursuer) {
    fprintf(sink, "(P%zu)", agent->id + 1);
  } else {
    fprintf(sink, "(E%zu)", agent->id + 1);
  }
}

static void tikz_agent_label(FILE *sink, const agent_t *agent, bool pursuer) {
  if (pursuer) {
    fprintf(sink, "P_{%zu}", agent->id + 1);
  } else {
    fprintf(sink, "E_{%zu}", agent->id + 1);
  }
}

static void tikz_agent_definition(FILE *sink, const agent_t *agent,
                                  bool pursuer, double scale) {
  fprintf(sink, "\\coordinate [label=\\textcolor{black}{$");
  tikz_agent_label(sink, agent, pursuer);
  fprintf(sink, "$}] ");
  tikz_agent_ref(sink, agent, pursuer);
  fprintf(sink, " at (%.2f, %.2f);\n", agent->pos.x / scale,
          agent->pos.y / scale);
}

static void tikz_edge(FILE *sink, const agent_t *a1, const agent_t *a2,
                      bool a1p, bool a2p) {
  fprintf(sink, "\\draw ");
  tikz_agent_ref(sink, a1, a1p);
  fprintf(sink, " -- ");
  tikz_agent_ref(sink, a2, a2p);
  fprintf(sink, ";\n");
}

static void tikz_agent_dot(FILE *sink, const agent_t *agent, bool pursuer) {
  fprintf(sink, "\\fill[%s,opacity=0.5] ", pursuer ? "red" : "green");
  tikz_agent_ref(sink, agent, pursuer);
  fprintf(sink, " circle [radius=\\agentdot];\n");
}

static void tikz_agent_radius(FILE *sink, const agent_t *agent, bool pursuer,
                              double ploss_limit, double scale) {
  double radius = inverse_ploss(ploss_limit) / scale;
  fprintf(sink, "\\draw[line width=\\agentrad,loosely dotted] ");
  tikz_agent_ref(sink, agent, pursuer);
  fprintf(sink, " circle [radius=%.2f];\n", radius);
}

static void tikz_draw_radius(FILE *sink, const vec2d_t *center, float radius,
                             double scale) {
  fprintf(sink, "\\draw (%.2f, %.2f) circle [radius=%.2f];\n",
          center->x / scale, center->y / scale, radius / scale);
}

static void tikz_render(FILE *sink, const char *fpath, const agent_t *agents,
                        size_t n, size_t m, double ploss_limit, double r_max,
                        double scale) {
  fprintf(sink,
          "\\begin{tikzpicture}[x=1pt, y=1pt] %% Tweak units for scale\n");
  fprintf(sink, "%% Replay of '%s'\n\n", fpath);
  fprintf(sink, "\\def\\agentdot{2pt} %% Change agent dot size\n");
  fprintf(sink, "\\def\\agentrad{2pt} %% Change agent radius line thickness\n");
  fprintf(sink, "\\def\\drawagentradii{1 < 0} %% Draw agent connection radii "
                "(false)\n\n");
  fprintf(sink, "%% Agent coordinate locations\n\n");

  /* Begin by defining agents with coordinates */

  for (size_t i = 0; i < n + m; i++) {
    tikz_agent_definition(sink, &agents[i], i < n, scale);
  }

  fprintf(sink, "\n%% Agent connection edges\n\n");

  /* For every existing agent connection, let's draw an edge */

  for (size_t i = 0; i < n + m; i++) {
    for (size_t j = 0; j < n + m; j++) {
      if (i == j) continue; /* No self-self considerations */

      /* Calculate path loss and draw a line if within limit */

      if (ploss(&agents[i], &agents[j]) <= ploss_limit) {
        tikz_edge(sink, &agents[i], &agents[j], i < n, j < n);
      }
    }
  }

  /* Give each agent a coloured dot to represent its type */

  fprintf(sink, "\n%% Agent dots\n\n");

  for (size_t i = 0; i < n + m; i++) {
    tikz_agent_dot(sink, &agents[i], i < n);
  }

  /* Draw agent connection radius */

  fprintf(sink, "\n%% Agent connection radius\n\n");

  fprintf(sink, "\\ifthenelse{\\drawagentradii}{\n");
  for (size_t i = 0; i < n + m; i++) {
    tikz_agent_radius(sink, &agents[i], i < n, ploss_limit, scale);
  }
  fprintf(sink, "}{}\n");

  /* Draw overall radius */

  fprintf(sink, "\n%% Bounded region\n\n");
  tikz_draw_radius(sink, &(vec2d_t)VEC2D_SINIT(0, 0), r_max, scale);

  fprintf(sink, "\\end{tikzpicture}\n");
}

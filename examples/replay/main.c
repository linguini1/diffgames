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
#include "SDL_render.h"
#include "helptext.h"
#include "render.h"
#include "utils.h"

const char WINDOW_NAME[] = "Replay";

#define LAMBDA (0.32764203)

/* Represents a single agent with a position in space */

typedef struct {
  size_t id;
  vec3d_t pos;
} agent_t;

static double ploss(const agent_t *a1, const agent_t *a2) {
  return 20.0 * log10((4 * M_PI / LAMBDA) * vec3d_dist_r(&a1->pos, &a2->pos));
}

static int parse_record(char *buf, agent_t *agent);

int main(int argc, char **argv) {
  double scale = 5.0;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  bool running = true;
  char *filename = NULL;
  char buf[BUFSIZ];
  size_t n = 0;
  size_t m = 0;
  size_t timestep = 0;
  bool game_over = false;
  bool exit_on_completion = false;
  bool show_network = false;
  double ploss_limit = 75.0;

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

  /* Get N and M TODO: no error handling */

  fgets(buf, sizeof(buf), file);
  char *token = strtok(buf, ",");
  n = strtoul(token, NULL, 10);
  token = strtok(NULL, ",");
  m = strtoul(token, NULL, 10);

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

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

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
          fseek(file, 0, SEEK_SET);
          fgets(buf, sizeof(buf), file); /* Skip headers */
          game_over = false;
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

    /* Populate agents with current time step */

    for (size_t i = 0; i < n + m && !game_over; i++) {
      fgets(buf, sizeof(buf), file);
      if (feof(file)) {
        game_over = true;
        break;
      }
      parse_record(buf, &agents[i]);
      agents[i].pos.x /= scale;
      agents[i].pos.y /= scale;
      agents[i].pos.z /= scale;
    }

    /* Clear screen to black with semi-transparency so trail appears */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 10);
    SDL_RenderFillRect(renderer, &fullscreen);

    /* Draw graph connections */

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

    for (size_t i = 0; i < n && show_network; i++) {
      for (size_t j = 0; j < n + m; j++) {
        if (i == j) continue; /* No self-self considerations */

        /* Calculate path loss and draw a line if within limit */
        if (ploss(&agents[i], &agents[j]) <= ploss_limit) {
          render_line(renderer, &agents[i].pos, &agents[j].pos);
        }
      }
    }

    /* Draw pursuers in red */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);

    for (size_t i = 0; i < n; i++) {
      render_vec2d(renderer, &agents[i].pos);
    }

    /* Draw evaders in green */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);

    for (size_t j = n; j < n + m; j++) {
      render_vec2d(renderer, &agents[j].pos);
    }

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Clear graph connections */

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);

    for (size_t i = 0; i < n && show_network; i++) {
      for (size_t j = 0; j < n + m; j++) {
        if (i == j) continue; /* No self-self considerations */

        /* Calculate path loss and draw a line if within limit */
        if (ploss(&agents[i], &agents[j]) <= ploss_limit) {
          render_line(renderer, &agents[i].pos, &agents[j].pos);
        }
      }
    }

    /* Advance simulation until a capture occurs */

    usleep(timestep);

    /* Check if game is over and exit */

    if (exit_on_completion && game_over) break;
  }

  /* Release resources */

  fclose(file);
  free(agents);
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

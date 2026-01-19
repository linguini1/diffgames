#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "helptext.h"
#include "utils.h"

const char window_name[] = "Homicidal Chauffer";

#define TIMESTEP (0.01)

/* State variable indexes */

enum state_e {
  P_X,    /* Evader X */
  P_Y,    /* Evader Y */
  P_HEAD, /* Evader heading */
  E_X,    /* Evader X */
  E_Y,    /* Evader Y */
  E_HEAD, /* Evader heading */
};

/* Game constants */

static double chauffeur_vel = 50.0;
static double pedestrian_vel = 25.0;
static double capture_radius = 0.0;
static double turn_radius = 5.0;

/* Game dynamics */

static void game_f(double *x, size_t n, double dt);
static void game_u(double *x, size_t n, double dt);
static double game_g(const double *x, size_t n, double dt);

/* Distance between the pursuer and evader */

static double distance(const double *x) {
  return sqrt(pow((0.0 - x[E_X]), 2) + pow((0.0 - x[E_Y]), 2));
}

int main(int argc, char **argv) {
  double scale = 5.0;
  SDL_DisplayMode dm = {0};
  SDL_DisplayMode tempdm;
  SDL_Event event;
  bool running = true;
  bool game_over = false;

  int c;
  while ((c = getopt(argc, argv, ":hx:y:s:v:r:t:e:")) != -1) {
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
    case 'v':
      chauffeur_vel = strtod(optarg, NULL);
      if (chauffeur_vel < 0) {
        fprintf(stderr, "Chauffeur velocity must be >= 0.\n");
        exit(EXIT_FAILURE);
      }
      break;
    case 'e':
      pedestrian_vel = strtod(optarg, NULL);
      if (pedestrian_vel < 0) {
        fprintf(stderr, "Pedestrian velocity must be >= 0.\n");
        exit(EXIT_FAILURE);
      }
      break;
    case 'r':
      capture_radius = strtod(optarg, NULL);
      if (capture_radius < 0) {
        fprintf(stderr, "Capture radius must be >= 0.\n");
        exit(EXIT_FAILURE);
      }
      break;
    case 't':
      turn_radius = strtod(optarg, NULL);
      if (turn_radius < 0) {
        fprintf(stderr, "Turning radius must be >= 0.\n");
        exit(EXIT_FAILURE);
      }
      break;
    case '?':
      fprintf(stderr, "Unknown option -%c\n", optopt);
      exit(EXIT_FAILURE);
      break;
    }
  }

  if (pedestrian_vel >= chauffeur_vel) {
    fprintf(stderr,
            "Pedestrian velocity must be less than the chauffeur velocity.\n");
    exit(EXIT_FAILURE);
  }

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    printf("Could not initialize SDL: %s\n", SDL_GetError());
  }

  SDL_GetDesktopDisplayMode(0, &tempdm);
  if (dm.w == 0) dm.w = tempdm.w / 2;
  if (dm.h == 0) dm.h = tempdm.h / 2;
  SDL_Rect fullscreen = {.x = 0, .y = 0, .w = dm.w, .h = dm.h};

  /* Create window */

  SDL_Window *window =
      SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, dm.w, dm.h, SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Set up game with initial conditons */

  double game_x[] = {
      [P_X] = randval(0, dm.w / scale), [P_Y] = randval(0, dm.h / scale),
      [P_HEAD] = randval(0, 2 * M_PI),  [E_X] = randval(0, dm.w / scale),
      [E_Y] = randval(0, dm.h / scale), [E_HEAD] = 0,
  };
  dynsys_t game = DYNSYS_SINIT(game_x, game_f, game_u, game_g, NULL);

  /* Render simulation */

  running = true;
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
        case SDLK_SPACE:
          game_x[P_X] = randval(0, dm.w / scale);
          game_x[P_Y] = randval(0, dm.h / scale),
          game_x[P_HEAD] = randval(0, 2 * M_PI);
          game_x[E_X] = randval(0, dm.w / scale),
          game_x[E_Y] = randval(0, dm.h / scale);
          dynsys_init(&game, game_x, sizeof(game_x) / sizeof(double), game_f,
                      game_u, game_g, NULL);
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

    /* Clear screen to black with some transparency so we can see the movement
     * trails of the players.
     */

    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 10);
    SDL_RenderFillRect(renderer, &fullscreen);

    /* Draw agents. Pursuer is red, evader is green. */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawPoint(renderer, game_x[P_X], game_x[P_Y]);

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    SDL_RenderDrawPoint(renderer, game_x[E_X], game_x[E_Y]);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation. If the distance between the pursuer and the evader is
     * within the capture range, stop the game and just render a frozen frame of
     * the capture.
     */

    double curdist = distance(game_x);
    game_over = curdist <= capture_radius;

    if (!game_over) {
      dynsys_step(&game, TIMESTEP);
    }
  }

  /* Release resources */

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();

  return EXIT_SUCCESS;
}

static void game_f(double *x, size_t n, double dt) {
  unused(n);
  x[P_X] += dt * chauffeur_vel * sin(x[P_HEAD]);
  x[P_Y] += dt * chauffeur_vel * cos(x[P_HEAD]);
  x[E_X] += dt * pedestrian_vel * sin(x[E_HEAD]);
  x[E_Y] += dt * pedestrian_vel * cos(x[E_HEAD]);
}

static void game_u(double *x, size_t n, double dt) {
  unused(n);
  static double t = 0;
  double phi;
  double rho = 0;
  double sw = cos(rho + t) - cos(rho);

  /* TODO: optimal chauffeur strategy */
  if (f_is_zero(sw, 0.05)) {
    phi = 0;
  } else if (sw < 0.0) {
    phi = -1;
  } else {
    phi = 1;
  }

  /* TODO: optimal evader strategy */
  x[E_HEAD] = rho + t;
  x[P_HEAD] += dt * (chauffeur_vel / turn_radius) * phi;
  t += dt;
}

/* Running cost of the game is time to capture */
static double game_g(const double *x, size_t n, double dt) {
  unused(n);
  unused(x);
  return dt;
}

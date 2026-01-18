/* This implementation is based on the paper titled "Multiple Pursuer Multiple
 * Evader Differential Games". Specifically, the 2 pursuer, 2 evader game
 * described in Section III.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "dynsys.h"
#include "utils.h"

#define TIMESTEP (0.01)

const char window_name[] = "2 Pursuers vs 2 Evaders";
const int width = 2048;
const int height = 1024;
const double scale = 8.0;

/* State variable indexes */

enum var_e {
  P1_X, /* Pursuer 1 X */
  P1_Y, /* Pursuer 1 Y */
  P1_H, /* Pursuer 1 heading */
  P2_X, /* Pursuer 2 X */
  P2_Y, /* Pursuer 2 Y */
  P2_H, /* Pursuer 2 heading */
  E1_X, /* Evader 1 X */
  E1_Y, /* Evader 1 Y */
  E1_H, /* Evader 1 heading */
  E2_X, /* Evader 2 X */
  E2_Y, /* Evader 2 Y */
  E2_H, /* Evader 2 heading */
};

/* Player velocities */

#define P1_VEL (45.0)
#define P2_VEL (40.0)
#define E1_VEL (25.0)
#define E2_VEL (20.0)

static const double RATIOS[2][2] = {
    {E1_VEL / P1_VEL, E2_VEL / P1_VEL},
    {E1_VEL / P2_VEL, E2_VEL / P2_VEL},
};

#define CAPTURE_RADIUS (1.0)

/* Game dynamics */

static void game_f(double *x, size_t n, double dt);
static void game_u(double *x, size_t n, double dt);

static double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

const SDL_Rect FULLSCREEN = {.x = 0, .y = 0, .w = width, .h = height};

int main(void) {

  /* Set up OpenGL parameters */

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);

  /* Start SDL */

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    fprintf(stderr, "Could not initialize SDL: %s\n", SDL_GetError());
  }

  /* Create window */

  SDL_Window *window = SDL_CreateWindow(window_name, SDL_WINDOWPOS_UNDEFINED,
                                        SDL_WINDOWPOS_UNDEFINED, width, height,
                                        SDL_WINDOW_OPENGL);

  /* Create renderer */

  SDL_Renderer *renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  SDL_RenderSetScale(renderer, scale, scale);

  /* Set up game with random initial conditions */

  srand(time(NULL));
  double game_x[] = {
      [P1_X] = randval(0.0, width / scale),
      [P1_Y] = randval(0.0, height / scale),
      [P1_H] = 0.0,
      [P2_X] = randval(0.0, width / scale),
      [P2_Y] = randval(0.0, height / scale),
      [P2_H] = 0.0,
      [E1_X] = randval(0.0, width / scale),
      [E1_Y] = randval(0.0, height / scale),
      [E1_H] = 0.0,
      [E2_X] = randval(0.0, width / scale),
      [E2_Y] = randval(0.0, height / scale),
      [E2_H] = 0.0,
  };
  dynsys_t game = DYNSYS_SINIT(game_x, game_f, game_u, NULL, NULL);

  /* Render simulation */

  bool running = true;
  SDL_Event event;

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
    SDL_RenderFillRect(renderer, &FULLSCREEN);

    /* Draw pursuers in red */

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, game_x[P1_X], game_x[P1_Y]);
    SDL_RenderDrawPoint(renderer, game_x[P2_X], game_x[P2_Y]);

    /* Draw evaders in green */

    SDL_SetRenderDrawColor(renderer, 0, 255, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderDrawPoint(renderer, game_x[E1_X], game_x[E1_Y]);
    SDL_RenderDrawPoint(renderer, game_x[E2_X], game_x[E2_Y]);

    /* Show what was drawn */

    SDL_RenderPresent(renderer);

    /* Advance simulation until a capture occurs */

    bool game_over =
        ((dist(game_x[E1_X], game_x[E1_Y], game_x[P1_X], game_x[P1_Y]) <=
          CAPTURE_RADIUS) ||
         (dist(game_x[E1_X], game_x[E1_Y], game_x[P2_X], game_x[P2_Y]) <=
          CAPTURE_RADIUS) ||
         (dist(game_x[E2_X], game_x[E2_Y], game_x[P1_X], game_x[P1_Y]) <=
          CAPTURE_RADIUS) ||
         (dist(game_x[E2_X], game_x[E2_Y], game_x[P2_X], game_x[P2_Y]) <=
          CAPTURE_RADIUS));

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

/* Dynamics of a single "simple" agent (holonomic) */
static void player_f(double *x, double *y, double head, double vel, double dt) {
  *x += dt * vel * cos(head);
  *y += dt * vel * sin(head);
}

/* Dynamics for all players */
static void game_f(double *x, size_t n, double dt) {
  unused(n);
  player_f(&x[P1_X], &x[P1_Y], x[P1_H], P1_VEL, dt);
  player_f(&x[P2_X], &x[P2_Y], x[P2_H], P2_VEL, dt);
  player_f(&x[E1_X], &x[E1_Y], x[E1_H], E1_VEL, dt);
  player_f(&x[E2_X], &x[E2_Y], x[E2_H], E2_VEL, dt);
}

static double y_ij(unsigned i, unsigned j, double *px, double *ex, double *py,
                   double *ey) {
  double aij = RATIOS[i][j];
  return (ey[j] - (aij * aij) * py[i] -
          aij * dist(px[i], py[i], ex[j], ey[j])) /
         (1.0 - (aij * aij));
}

static void opt_aimpoints(double *game_x, double *ex1, double *ey1, double *ex2,
                          double *ey2, double *px1, double *py1, double *px2,
                          double *py2) {
  double px[] = {game_x[P1_X], game_x[P2_X]};
  double ex[] = {game_x[E1_X], game_x[E2_X]};
  double py[] = {game_x[P1_Y], game_x[P2_Y]};
  double ey[] = {game_x[E1_Y], game_x[E2_Y]};

  double ys1 = y_ij(0, 0, px, ex, py, ey) + y_ij(1, 1, px, ex, py, ey);
  double ys2 = y_ij(0, 1, px, ex, py, ey) + y_ij(1, 0, px, ex, py, ey);

  if (ys1 > ys2) {
    /* Equation 10 */
    *ex1 =
        (ex[0] - pow(RATIOS[0][0], 2) * px[0]) / (1.0 - pow(RATIOS[0][0], 2));
    *ey1 = (ey[0] - pow(RATIOS[0][0], 2) * py[0] -
            pow(RATIOS[0][0], 2) * dist(px[0], py[0], ex[0], ex[0])) /
           (1.0 - pow(RATIOS[0][0], 2));
    *ex2 =
        (ex[1] - pow(RATIOS[1][1], 2) * px[1]) / (1.0 - pow(RATIOS[1][1], 2));
    *ey2 = (ey[1] - pow(RATIOS[1][1], 2) * py[1] -
            pow(RATIOS[1][1], 2) * dist(px[1], py[1], ex[1], ex[1])) /
           (1.0 - pow(RATIOS[1][1], 2));
    *px1 = *ex1;
    *py1 = *ey1;
    *px2 = *ex2;
    *py2 = *ey2;
  } else {
    /* Equation 11 */
    *ex1 =
        (ex[0] - pow(RATIOS[1][0], 2) * px[1]) / (1.0 - pow(RATIOS[1][0], 2));
    *ey1 = (ey[0] - pow(RATIOS[1][0], 2) * py[1] -
            pow(RATIOS[1][0], 2) * dist(px[1], py[1], ex[0], ex[0])) /
           (1.0 - pow(RATIOS[1][0], 2));
    *ex2 =
        (ex[1] - pow(RATIOS[0][1], 2) * px[0]) / (1.0 - pow(RATIOS[0][1], 2));
    *ey2 = (ey[1] - pow(RATIOS[0][1], 2) * py[0] -
            pow(RATIOS[0][1], 2) * dist(px[0], py[0], ex[1], ex[1])) /
           (1.0 - pow(RATIOS[0][1], 2));
    *px2 = *ex1;
    *py2 = *ey1;
    *px1 = *ex2;
    *py1 = *ey2;
  }
}

static void game_u(double *x, size_t n, double dt) {
  unused(n);
  unused(dt);

  double ex1;
  double px1;
  double ex2;
  double px2;
  double ey1;
  double py1;
  double ey2;
  double py2;

  /* Calculate the optimal aim points for each agent */

  opt_aimpoints(x, &ex1, &ey1, &ex2, &ey2, &px1, &py1, &px2, &py2);

  /* From the optimal aim points, we can compute the optimal headings. Taken
   * from Equation 9.
   */

  x[E1_H] = atan2(ey1 - x[E1_Y], ex1 - x[E1_X]);
  x[E2_H] = atan2(ey2 - x[E2_Y], ex2 - x[E2_X]);
  x[P1_H] = atan2(py1 - x[P1_Y], px1 - x[P1_X]);
  x[P2_H] = atan2(py2 - x[P2_Y], px2 - x[P2_X]);
  // printf("E1: %lf\n", x[E1_H] * (180.0 / M_PI));
  // printf("E2: %lf\n", x[E2_H] * (180.0 / M_PI));
  // printf("P1: %lf\n", x[P1_H] * (180.0 / M_PI));
  // printf("P2: %lf\n", x[P2_H] * (180.0 / M_PI));
}

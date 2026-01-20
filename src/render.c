#include <math.h>

#include "render.h"

void render_circle(SDL_Renderer *renderer, int cx, int cy, unsigned radius,
                   unsigned res) {

  SDL_Point a;
  SDL_Point b;

  for (unsigned i = 0; i < res; i++) {
    a.x = cx + radius * cos((i * 2 * M_PI) / res);
    a.y = cy + radius * sin((i * 2 * M_PI) / res);
    b.x = cx + radius * cos(((i + 1) * 2 * M_PI) / res);
    b.y = cy + radius * sin(((i + 1) * 2 * M_PI) / res);

    SDL_RenderDrawLine(renderer, a.x, a.y, b.x, b.y);
  }
}

void render_quadrotor2d(SDL_Renderer *renderer, int cx, int cy, double heading,
                        double rotorlen) {
  double rotorx;
  double rotory;

  /* We will make the assumption that the front of the quadrotor is between
   * two rotor arms, at a 45 degree angle.
   */

  heading += M_PI_4 / 2;

  /* Draw rotor arms as an X with circles at the rotor tips */

  for (unsigned i = 0; i < 2; i++) {
    rotorx = rotorlen * cos(heading);
    rotory = rotorlen * sin(heading);

    SDL_RenderDrawLine(renderer, cx - rotorx, cy - rotory, cx + rotorx,
                       cy + rotory);
    render_circle(renderer, cx - rotorx, cy - rotory, rotorlen / 4, 10);
    render_circle(renderer, cx + rotorx, cy + rotory, rotorlen / 4, 10);
    heading += M_PI_2;
  }
}

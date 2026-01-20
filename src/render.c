#include <math.h>

#include "render.h"

void render_circle(SDL_Renderer *renderer, const vec2d_t *c, unsigned radius,
                   unsigned res) {
  vec2d_t a;
  vec2d_t b;

  for (unsigned i = 0; i < res; i++) {
    a.x = c->x + radius * cos((i * 2 * M_PI) / res);
    a.y = c->y + radius * sin((i * 2 * M_PI) / res);
    b.x = c->x + radius * cos(((i + 1) * 2 * M_PI) / res);
    b.y = c->y + radius * sin(((i + 1) * 2 * M_PI) / res);

    render_line(renderer, &a, &b);
  }
}

void render_quadrotor2d(SDL_Renderer *renderer, const vec2d_t *c,
                        double heading, double rotorlen) {
  vec2d_t rotor;

  /* We will make the assumption that the front of the quadrotor is between
   * two rotor arms, at a 45 degree angle.
   */

  heading += M_PI_4 / 2;

  /* Draw rotor arms as an X with circles at the rotor tips */

  for (unsigned i = 0; i < 2; i++) {
    rotor.x = rotorlen * cos(heading);
    rotor.y = rotorlen * sin(heading);

    SDL_RenderDrawLine(renderer, c->x - rotor.x, c->y - rotor.y, c->x + rotor.x,
                       c->y + rotor.y);
    render_circle(renderer, vec2d_temp(c->x - rotor.x, c->y - rotor.y),
                  rotorlen / 4, 10);
    render_circle(renderer, vec2d_temp(c->x + rotor.x, c->y + rotor.y),
                  rotorlen / 4, 10);
    heading += M_PI_2;
  }
}

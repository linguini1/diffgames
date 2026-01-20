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

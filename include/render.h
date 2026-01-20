#ifndef DIFFGAMES_RENDER_H
#define DIFFGAMES_RENDER_H

#include <SDL2/SDL.h>

#include "3dtools.h"

/* Represents a camera whose POV is used to render 3D scenes */

typedef struct {
  vec3d_t pos;
} camera3d_t;

/* Short-hands for rendering vector-defined things */

#define render_vec2d(renderer, v) SDL_RenderDrawPoint(renderer, (v)->x, (v)->y)
#define render_line(renderer, vs, ve)                                          \
  SDL_RenderDrawLine(renderer, (vs)->x, (vs)->y, (ve)->x, (ve)->y)

/* Draw a 3D
 *
 * Parameters:
 * - renderer The SDL renderer used to draw
 * - c The coordinates of the circle's center
 * - radius The radius of the circle
 * - res The resolution of the circle (how many points to approximate it)
 */
void render_circle(SDL_Renderer *renderer, const vec2d_t *c, unsigned radius,
                   unsigned res);

/* Draw a quadrotor frame in 2D.
 *
 * Parameters:
 *
 * - renderer The SDL renderer used to draw
 * - c The coordinates of the quad's center
 * - heading The quad's heading angle
 * - rotorlen The length of the rotors from the quadrotor body.
 */
void render_quadrotor2d(SDL_Renderer *renderer, const vec2d_t *c,
                        double heading, double rotorlen);

#endif // DIFFGAMES_RENDER_H

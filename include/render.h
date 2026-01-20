#ifndef DIFFGAMES_RENDER_H
#define DIFFGAMES_RENDER_H

#include <SDL2/SDL.h>

#include "3dtools.h"

/* Represents a camera whose POV is used to render 3D scenes */

typedef struct {
  vec3d_t pos;
} camera_t;

/* Draw a 3D
 *
 * Parameters:
 * - renderer The SDL renderer used to draw
 * - cx The circle's center x coordinate
 * - cy The circle's center y coordinate
 * - radius The radius of the circle
 * - res The resolution of the circle (how many points to approximate it)
 */
void render_circle(SDL_Renderer *renderer, int cx, int cy, unsigned radius,
                   unsigned res);

/* Draw a circle.
 *
 * Parameters:
 * - renderer The SDL renderer used to draw
 * - cx The circle's center x coordinate
 * - cy The circle's center y coordinate
 * - radius The radius of the circle
 * - res The resolution of the circle (how many points to approximate it)
 */
void render_circle(SDL_Renderer *renderer, int cx, int cy, unsigned radius,
                   unsigned res);

/* Draw a quadrotor frame in 2D.
 *
 * Parameters:
 *
 * - renderer The SDL renderer used to draw
 * - cx The quad's center x coordinate
 * - cy The quad's center y coordinate
 * - heading The quad's heading angle
 * - rotorlen The length of the rotors from the quadrotor body.
 */
void render_quadrotor2d(SDL_Renderer *renderer, int cx, int cy, double heading,
                        double rotorlen);

#endif // DIFFGAMES_RENDER_H

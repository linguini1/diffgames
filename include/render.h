#ifndef DIFFGAMES_RENDER_H
#define DIFFGAMES_RENDER_H

#include <SDL2/SDL.h>

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

#endif // DIFFGAMES_RENDER_H

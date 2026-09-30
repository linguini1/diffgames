#ifndef _TIKZ_H_
#define _TIKZ_H_

/* Module for rendering visualizations to Tikz figures.
 *
 * Meant to be somewhat extensible/modular to allow creation
 * of custom rendering functions and use in multiple visualization examples.
 */

#include <stdio.h>

#include "3dtools.h"

void tikz_picture_start(FILE *sink, const char *xscale, const char *yscale);
void tikz_picture_end(FILE *sink);

void tikz_ref(FILE *sink, const char *id);

void tikz_edge_ref(FILE *sink, const char *a, const char *b);
void tikz_edge_2dcoord(FILE *sink, const vec2d_t *a, const vec2d_t *b);
void tikz_edge_3dcoord(FILE *sink, const vec3d_t *a, const vec3d_t *b);

void tikz_circle(FILE *sink, const vec2d_t *center, float radius);

void tikz_dot_ref(FILE *sink, const char *ref, const char *colour,
                  float opacity, float radius);
void tikz_dot_2d(FILE *sink, const vec2d_t *pos, const char *colour,
                 float opacity, float radius);
void tikz_dot_3d(FILE *sink, const vec3d_t *pos, const char *colour,
                 float opacity, float radius);

#endif /* _TIKZ_H_ */

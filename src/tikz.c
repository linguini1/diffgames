#include "tikz.h"

void tikz_picture_start(FILE *sink, const char *xscale, const char *yscale) {
  fprintf(sink, "\\begin{tikzpicture}[x=%s,y=%s] %% Tweak units for scale\n",
          xscale == NULL ? "1pt" : xscale, yscale == NULL ? "1pt" : yscale);
}

void tikz_picture_end(FILE *sink) { fprintf(sink, "\\end{tikzpicture}\n"); }

void tikz_ref(FILE *sink, const char *id) { fprintf(sink, "(%s)", id); }

void tikz_edge_ref(FILE *sink, const char *a, const char *b) {
  fprintf(sink, "\\draw (%s) -- (%s);\n", a, b);
}

void tikz_edge_2dcoord(FILE *sink, const vec2d_t *a, const vec2d_t *b) {
  fprintf(sink, "\\draw (" VEC2D_FMT ") -- (" VEC2D_FMT ");\n", VEC2D_PRINTF(a),
          VEC2D_PRINTF(b));
}

void tikz_edge_3dcoord(FILE *sink, const vec3d_t *a, const vec3d_t *b) {
  fprintf(sink, "\\draw (" VEC3D_FMT ") -- (" VEC3D_FMT ");\n", VEC3D_PRINTF(a),
          VEC3D_PRINTF(b));
}

void tikz_circle(FILE *sink, const vec2d_t *center, float radius) {
  fprintf(sink, "\\draw (" VEC2D_FMT ") circle [radius=%f];\n",
          VEC2D_PRINTF(center), radius);
}

void tikz_dot_ref(FILE *sink, const char *ref, const char *colour,
                  float opacity, float radius) {
  fprintf(sink, "\\fill[%s, opacity=%f] %s circle [radius=%f];\n", colour,
          opacity, ref, radius);
}

void tikz_dot_2d(FILE *sink, const vec2d_t *pos, const char *colour,
                 float opacity, float radius) {
  fprintf(sink, "\\fill[%s, opacity=%f] " VEC2D_FMT " circle [radius=%f];\n",
          colour, opacity, VEC2D_PRINTF(pos), radius);
}

void tikz_dot_3d(FILE *sink, const vec3d_t *pos, const char *colour,
                 float opacity, float radius) {
  fprintf(sink, "\\fill[%s, opacity=%f] " VEC3D_FMT " circle [radius=%f];\n",
          colour, opacity, VEC3D_PRINTF(pos), radius);
}

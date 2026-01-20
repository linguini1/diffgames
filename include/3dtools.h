#ifndef DIFFGAMES_3DTOOLS_H
#define DIFFGAMES_3DTOOLS_H

/* Axis numbering */

enum axis_e {
  AXIS_X = 0,
  AXIS_Y = 1,
  AXIS_Z = 2,
};

/* Vector in 3 dimensions */

typedef struct {
  double x;
  double y;
  double z;
} vec3d_t;

#define VEC3D_FMT "(%lf, %lf, %lf)"
#define VEC3D_PRINTF(v) (v)->x, (v)->y, (v)->z

#define VEC3D_SINIT(vx, vy, vz) {.x = (vx), .y = (vy), .z = (vz)}
#define vec3d_temp(vx, vy) &((vec3d_t)VEC3D_SINIT(vx, vy))
#define vec3d_get_axis(v, a) (((double *)(v))[(a)])

void vec3d_init(vec3d_t *v, double x, double y, double z);
vec3d_t vec3d_init_r(double x, double y, double z);

#define vec3d_add(v1, v2, res)                                                 \
  do {                                                                         \
    (res)->x = (v1)->x + (v2)->x;                                              \
    (res)->y = (v1)->y + (v2)->y;                                              \
    (res)->z = (v1)->z + (v2)->z;                                              \
  } while (0)

vec3d_t vec3d_add_r(vec3d_t *v1, vec3d_t *v2);

#define vec3d_scale(v, alpha, res)                                             \
  do {                                                                         \
    (res)->x = (alpha) * (v)->x;                                               \
    (res)->y = (alpha) * (v)->y;                                               \
    (res)->z = (alpha) * (v)->z;                                               \
  } while (0)

vec3d_t vec3d_scale_r(vec3d_t *v, double alpha);

#define vec3d_dot(v1, v2, res)                                                 \
  do {                                                                         \
    (res) = (v1)->x * (v2->x) + (v1)->y * (v2->y) + (v1)->z * (v2->z);         \
  } while (0)

double vec3d_dot_r(vec3d_t *v1, vec3d_t *v2);

void vec3d_rotate(vec3d_t *v, double angle, enum axis_e axis, vec3d_t *res);
vec3d_t vec3d_rotate_r(vec3d_t *v, double angle, enum axis_e axis);

void vec3d_norm(vec3d_t *v1, vec3d_t *v2, double *res);
double vec3d_norm_r(vec3d_t *v1, vec3d_t *v2);

/* Vector in 2 dimensions */

typedef struct {
  double x;
  double y;
} vec2d_t;

#define VEC2D_FMT "(%lf, %lf)"
#define VEC2D_PRINTF(v) (v)->x, (v)->y

#define VEC2D_SINIT(vx, vy) {.x = (vx), .y = (vy)}
#define vec2d_temp(vx, vy) &((vec2d_t)VEC2D_SINIT(vx, vy))
#define vec2d_get_axis(v, a) (((double *)(v))[(a)])

void vec3d_project(vec3d_t *v, double camdist, vec2d_t *res);
vec2d_t vec3d_project_r(vec3d_t *v, double camdist);

vec2d_t vec2d_init_r(double x, double y);
void vec2d_init(vec2d_t *v, double x, double y);

#define vec2d_add(v1, v2, res)                                                 \
  do {                                                                         \
    (res)->x = (v1)->x + (v2)->x;                                              \
    (res)->y = (v1)->y + (v2)->y;                                              \
  } while (0)

vec2d_t vec2d_add_r(vec2d_t *v1, vec2d_t *v2);

#define vec2d_scale(v1, alpha, res)                                            \
  do {                                                                         \
    (res)->x = (alpha) * (v1)->x;                                              \
    (res)->y = (alpha) * (v1)->y;                                              \
  } while (0)

vec2d_t vec2d_scale_r(vec2d_t *v, double alpha);

#define vec2d_dot(v1, v2, res)                                                 \
  do {                                                                         \
    (res) = (v1)->x * (v2)->x + (v1)->y * (v2)->y;                             \
  } while (0)

double vec2d_dot_r(vec2d_t *v1, vec2d_t *v2);

void vec2d_norm(vec2d_t *v1, vec2d_t *v2, double *res);
double vec2d_norm_r(vec2d_t *v1, vec2d_t *v2);

#endif // DIFFGAMES_3DTOOLS_H

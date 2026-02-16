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

vec3d_t vec3d_add_r(const vec3d_t *v1, const vec3d_t *v2);

#define vec3d_sub(v1, v2, res)                                                 \
  do {                                                                         \
    (res)->x = (v1)->x - (v2)->x;                                              \
    (res)->y = (v1)->y - (v2)->y;                                              \
    (res)->z = (v1)->z - (v2)->z;                                              \
  } while (0)

vec3d_t vec3d_sub_r(const vec3d_t *v1, const vec3d_t *v2);

#define vec3d_scale(v, alpha, res)                                             \
  do {                                                                         \
    (res)->x = (alpha) * (v)->x;                                               \
    (res)->y = (alpha) * (v)->y;                                               \
    (res)->z = (alpha) * (v)->z;                                               \
  } while (0)

vec3d_t vec3d_scale_r(const vec3d_t *v, double alpha);

#define vec3d_dot(v1, v2, res)                                                 \
  do {                                                                         \
    (res) = (v1)->x * (v2->x) + (v1)->y * (v2->y) + (v1)->z * (v2->z);         \
  } while (0)

double vec3d_dot_r(const vec3d_t *v1, const vec3d_t *v2);

void vec3d_rotate(const vec3d_t *v, double angle, enum axis_e axis,
                  vec3d_t *res);
vec3d_t vec3d_rotate_r(const vec3d_t *v, double angle, enum axis_e axis);

void vec3d_norm(const vec3d_t *v, double *res);
double vec3d_norm_r(const vec3d_t *v);

void vec3d_dist(const vec3d_t *v1, const vec3d_t *v2, double *res);
double vec3d_dist_r(const vec3d_t *v1, const vec3d_t *v2);

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

vec2d_t vec2d_init_r(double x, double y);
void vec2d_init(vec2d_t *v, double x, double y);

#define vec2d_add(v1, v2, res)                                                 \
  do {                                                                         \
    (res)->x = (v1)->x + (v2)->x;                                              \
    (res)->y = (v1)->y + (v2)->y;                                              \
  } while (0)

vec2d_t vec2d_add_r(const vec2d_t *v1, const vec2d_t *v2);

#define vec2d_sub(v1, v2, res)                                                 \
  do {                                                                         \
    (res)->x = (v1)->x - (v2)->x;                                              \
    (res)->y = (v1)->y - (v2)->y;                                              \
  } while (0)

vec2d_t vec2d_sub_r(const vec2d_t *v1, const vec2d_t *v2);

#define vec2d_scale(v1, alpha, res)                                            \
  do {                                                                         \
    (res)->x = (alpha) * (v1)->x;                                              \
    (res)->y = (alpha) * (v1)->y;                                              \
  } while (0)

vec2d_t vec2d_scale_r(const vec2d_t *v, double alpha);

#define vec2d_dot(v1, v2, res)                                                 \
  do {                                                                         \
    (res) = (v1)->x * (v2)->x + (v1)->y * (v2)->y;                             \
  } while (0)

double vec2d_dot_r(const vec2d_t *v1, const vec2d_t *v2);

void vec2d_norm(const vec2d_t *v, double *res);
double vec2d_norm_r(const vec2d_t *v);

void vec2d_dist(const vec2d_t *v1, const vec2d_t *v2, double *res);
double vec2d_dist_r(const vec2d_t *v1, const vec2d_t *v2);

/* Represents a camera whose POV is used for projection */

typedef struct {
  vec3d_t pos;
  vec3d_t ori;
} camera3d_t;

void camera_trans(const vec3d_t *v, const camera3d_t *cam, vec3d_t *res);
vec3d_t camera_trans_r(const vec3d_t *v, const camera3d_t *cam);

void camera_proj(const vec3d_t *v, const camera3d_t *cam, vec2d_t *res);
vec2d_t camera_proj_r(const vec3d_t *v, const camera3d_t *cam);

#endif // DIFFGAMES_3DTOOLS_H

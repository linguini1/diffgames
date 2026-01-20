#include <math.h>

#include "3dtools.h"
#include "utils.h"

vec3d_t vec3d_init_r(double x, double y, double z) {
  return (vec3d_t)VEC3D_SINIT(x, y, z);
}

void vec3d_init(vec3d_t *v, double x, double y, double z) {
  v->x = x;
  v->y = y;
  v->z = z;
}

vec3d_t vec3d_add_r(vec3d_t *v1, vec3d_t *v2) {
  vec3d_t res;
  vec3d_add(v1, v2, &res);
  return res;
}

vec3d_t vec3d_scale_r(vec3d_t *v, double alpha) {
  vec3d_t res;
  vec3d_scale(v, alpha, &res);
  return res;
}

double vec3d_dot_r(vec3d_t *v1, vec3d_t *v2) {
  double res;
  vec3d_dot(v1, v2, res);
  return res;
}

void vec3d_rotate(vec3d_t *v, double angle, enum axis_e axis, vec3d_t *res) {
  switch (axis) {
  case AXIS_X:
    res->x = v->x;
    res->y = v->y * cos(angle) - v->z * sin(angle);
    res->z = v->y * sin(angle) + v->z * cos(angle);
    break;
  case AXIS_Y:
    res->x = v->x * cos(angle) + v->z * sin(angle);
    res->y = v->y;
    res->z = v->x * -sin(angle) + v->z * cos(angle);
    break;
  case AXIS_Z:
    res->x = v->x * cos(angle) - v->y * sin(angle);
    res->y = v->x * sin(angle) + v->y * cos(angle);
    res->z = v->z;
    break;
  default:
    unreachable("No such axis.");
    break;
  }
}

void vec3d_norm(vec3d_t *v1, vec3d_t *v2, double *res) {
  *res = sqrt((v2->x - v1->x) * (v2->x - v1->x) +
              (v2->y - v1->y) * (v2->y - v1->y) +
              (v2->z - v1->z) * (v2->z - v1->z));
}

double vec3d_norm_r(vec3d_t *v1, vec3d_t *v2) {
  double res;
  vec3d_norm(v1, v2, &res);
  return res;
}

vec3d_t vec3d_rotate_r(vec3d_t *v, double angle, enum axis_e axis) {
  vec3d_t res;
  vec3d_rotate(v, angle, axis, &res);
  return res;
}

void vec3d_project(vec3d_t *v, double camdist, vec2d_t *res) {
  /* TODO: verify */
  res->x = v->x / (v->z / camdist);
  res->y = v->y / (v->z / camdist);
}

vec2d_t vec3d_project_r(vec3d_t *v, double camdist) {
  vec2d_t res;
  vec3d_project(v, camdist, &res);
  return res;
}

vec2d_t vec2d_init_r(double x, double y) { return (vec2d_t)VEC2D_SINIT(x, y); }

void vec2d_init(vec2d_t *v, double x, double y) {
  v->x = x;
  v->y = y;
}

vec2d_t vec2d_add_r(vec2d_t *v1, vec2d_t *v2) {
  vec2d_t res;
  vec2d_add(v1, v2, &res);
  return res;
}

vec2d_t vec2d_scale_r(vec2d_t *v, double alpha) {
  vec2d_t res;
  vec2d_scale(v, alpha, &res);
  return res;
}

double vec2d_dot_r(vec2d_t *v1, vec2d_t *v2) {
  double res;
  vec2d_dot(v1, v2, res);
  return res;
}

void vec2d_norm(vec2d_t *v1, vec2d_t *v2, double *res) {
  *res = sqrt((v2->x - v1->x) * (v2->x - v1->x) +
              (v2->y - v1->y) * (v2->y - v1->y));
}

double vec2d_norm_r(vec2d_t *v1, vec2d_t *v2) {
  double res;
  vec2d_norm(v1, v2, &res);
  return res;
}

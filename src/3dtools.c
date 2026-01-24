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

vec3d_t vec3d_add_r(const vec3d_t *v1, const vec3d_t *v2) {
  vec3d_t res;
  vec3d_add(v1, v2, &res);
  return res;
}

vec3d_t vec3d_sub_r(const vec3d_t *v1, const vec3d_t *v2) {
  vec3d_t res;
  vec3d_sub(v1, v2, &res);
  return res;
}

vec3d_t vec3d_scale_r(const vec3d_t *v, double alpha) {
  vec3d_t res;
  vec3d_scale(v, alpha, &res);
  return res;
}

double vec3d_dot_r(const vec3d_t *v1, const vec3d_t *v2) {
  double res;
  vec3d_dot(v1, v2, res);
  return res;
}

void vec3d_rotate(const vec3d_t *v, double angle, enum axis_e axis,
                  vec3d_t *res) {
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

void vec3d_norm(const vec3d_t *v, double *res) {
  *res = sqrt((v->x * v->x) + (v->y * v->y) + (v->z * v->z));
}

double vec3d_norm_r(const vec3d_t *v) {
  double res;
  vec3d_norm(v, &res);
  return res;
}

void vec3d_dist(const vec3d_t *v1, const vec3d_t *v2, double *res) {
  vec3d_t diff;
  vec3d_sub(v1, v2, &diff);
  vec3d_norm(&diff, res);
}

double vec3d_dist_r(const vec3d_t *v1, const vec3d_t *v2) {
  double res;
  vec3d_dist(v1, v2, &res);
  return res;
}

vec3d_t vec3d_rotate_r(const vec3d_t *v, double angle, enum axis_e axis) {
  vec3d_t res;
  vec3d_rotate(v, angle, axis, &res);
  return res;
}

vec2d_t vec2d_init_r(double x, double y) { return (vec2d_t)VEC2D_SINIT(x, y); }

void vec2d_init(vec2d_t *v, double x, double y) {
  v->x = x;
  v->y = y;
}

vec2d_t vec2d_add_r(const vec2d_t *v1, const vec2d_t *v2) {
  vec2d_t res;
  vec2d_add(v1, v2, &res);
  return res;
}

vec2d_t vec2d_scale_r(const vec2d_t *v, double alpha) {
  vec2d_t res;
  vec2d_scale(v, alpha, &res);
  return res;
}

double vec2d_dot_r(const vec2d_t *v1, const vec2d_t *v2) {
  double res;
  vec2d_dot(v1, v2, res);
  return res;
}

void vec2d_norm(const vec2d_t *v, double *res) {
  *res = sqrt(v->x * v->x + v->y * v->y);
}

double vec2d_norm_r(const vec2d_t *v) {
  double res;
  vec2d_norm(v, &res);
  return res;
}

void vec2d_dist(const vec2d_t *v1, const vec2d_t *v2, double *res) {
  vec2d_t diff;
  vec2d_sub(v1, v2, &diff);
  vec2d_norm(&diff, res);
}

double vec2d_dist_r(const vec2d_t *v1, const vec2d_t *v2) {
  double res;
  vec2d_dist(v1, v2, &res);
  return res;
}

void camera_trans(const vec3d_t *v, const camera3d_t *cam, vec3d_t *res) {
  /* https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula */
  vec3d_t diff;
  vec3d_sub(v, &cam->pos, &diff);

  double large_term =
      diff.z * cos(cam->ori.y) +
      sin(cam->ori.y) * (diff.y * sin(cam->ori.z) + diff.x * cos(cam->ori.z));

  res->x =
      cos(cam->ori.y) * (diff.y * sin(cam->ori.z) + diff.x * cos(cam->ori.z)) -
      diff.z * sin(cam->ori.y);
  res->y =
      sin(cam->ori.x) * large_term +
      cos(cam->ori.x) * (diff.y * cos(cam->ori.z) - diff.x * sin(cam->ori.z));
  res->z =
      cos(cam->ori.x) * large_term -
      sin(cam->ori.x) * (diff.y * cos(cam->ori.z) - diff.x * sin(cam->ori.z));
}

vec3d_t camera_trans_r(const vec3d_t *v, const camera3d_t *cam) {
  vec3d_t res;
  camera_trans(v, cam, &res);
  return res;
}

void camera_proj(const vec3d_t *v, const camera3d_t *cam, vec2d_t *res) {
  /* https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
   * Uses x/y projection plane.
   * We use display surface at 0,0, so e = c
   */
  vec3d_t d;
  camera_trans(v, cam, &d);
  res->x = (cam->pos.z * d.x) / d.z + cam->pos.x;
  res->y = (cam->pos.z * d.y) / d.z + cam->pos.y;
}

vec2d_t camera_proj_r(const vec3d_t *v, const camera3d_t *cam) {
  vec2d_t res;
  camera_proj(v, cam, &res);
  return res;
}

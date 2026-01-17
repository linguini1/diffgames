/* Included files */

#include <assert.h>

#include "dynsys.h"

void dynsys_init(dynsys_t *s, double *x, size_t n, dynamics_f f, control_f u,
                 run_cost_f g, term_cost_f q) {
  assert(s != NULL || (s == NULL && s == 0));
  assert(f != NULL);
  s->c = 0.0; /* No cost at start of game */
  s->x = x;
  s->n = n;
  s->f = f;
  s->u = u;
  s->g = g;
  s->q = q;
}

void dynsys_step(dynsys_t *s, double dt) {
  assert(s->f != NULL);
  if (s->g != NULL) s->c += s->g(s->x, s->n, dt); /* Update running cost */
  s->f(s->x, s->n, dt); /* Apply system dynamics to initial state */
  if (s->u != NULL) s->u(s->x, s->n, dt); /* Update control variables */
}

double dynsys_cost(const dynsys_t *s) {
  if (s->q == NULL) return s->c;
  return s->c + s->q(s->x, s->n);
}

#ifndef DIFFGAMES_DYNSYS_H
#define DIFFGAMES_DYNSYS_H

/* Included files */

#include <stdlib.h>

struct dynsys_t; /* Forward definition */

/* Dynamics function f(x, t)
 *
 * The dynamics function is responsible for applying the system's dynamics
 * equations to the state variables to move the state forward in time. The
 * dynamics function should only update those state variables which are not
 * control variables.
 *
 * Parameters:
 * - state: The private state (containing state variables) of the dynamic system
 * - dt: The amount of time passed since the last time-step
 */
typedef void (*dynamics_f)(void *state, double dt);

/* Control input function u(t)
 *
 * The control input function is responsible for applying a control input to the
 * system's control variables, which influences the system state via the
 * dynamics in future time steps. The control function should only update those
 * state variables which correspond to the control variables for the system.
 *
 * Parameters:
 * - state: The private state (containing state variables) of the dynamic system
 * - dt: The amount of time passed since the last time-step
 */
typedef void (*control_f)(void *state, double dt);

/* Running cost function g(x, t)
 *
 * This function is used to calculate the running cost over the entire system's
 * run-time.
 *
 * Parameters:
 * - state: The private state (containing state variables) of the dynamic system
 * - dt: The amount of time passed since the last time-step
 *
 * Returns: The running cost incurred for the time-step of duration `dt`.
 */
typedef double (*run_cost_f)(const void *state, double dt);

/* Terminal cost function q(x)
 *
 * This function is used to calculate the terminal cost at the end of the
 * system's run time using the state variables.
 *
 * Parameters:
 * - state: The private state (containing state variables) of the dynamic system
 *
 * Returns: The terminal cost for the system's state `x`.
 */
typedef double (*term_cost_f)(const void *x);

/* Representation of a generic dynamic system */

typedef struct dynsys_t {
  void *x;       /* State vars */
  double c;      /* Game cost tally */
  dynamics_f f;  /* Dynamics function f(x, t) */
  control_f u;   /* Control function u(t) */
  run_cost_f g;  /* Running cost function l(x, t) */
  term_cost_f q; /* Terminal cost function q(x) */
} dynsys_t;

#define DYNSYS_SINIT(d_x, d_f, d_u, d_g, d_q)                                  \
  {                                                                            \
      .x = (d_x),                                                              \
      .c = 0.0,                                                                \
      .f = (d_f),                                                              \
      .u = (d_u),                                                              \
      .g = (d_g),                                                              \
      .q = (d_q),                                                              \
  }

/* dynsys_step
 *
 * Initialize a dynamic system.
 *
 * Parameters:
 * - s: The dynamic system to initialize
 * - x: The private system state, including state variables
 * - f: The dynamics equations of the system
 * - u: The control input equation for the system. If NULL, the system runs
 *      from its initial conditions with no control.
 * - g: The running cost equation. If NULL, the running cost is assumed to be 0
 * - q: The terminal cost equation. If NULL, the terminal cost is assumed to be
 *      0
 */
void dynsys_init(dynsys_t *s, void *x, dynamics_f f, control_f u, run_cost_f g,
                 term_cost_f q);

/* dynsys_step
 *
 * Steps the system dynamics forward in time by:
 * 1) Tallying the running cost
 * 2) Applying the system dynamics function
 * 3) Applying the control input function
 *
 * Parameters:
 * - s: The dynamic system to step forward in time
 * - dt: How far forward in time to advance the system
 */
void dynsys_step(dynsys_t *s, double dt);

/* dynsys_cost
 *
 * This function calculates the total cost incurred so far, assuming this
 * function was called at the terminal time (running cost + terminal cost)
 *
 * Parameters:
 * - s: The dynamic system to get the cost of
 *
 * Returns: The total system cost
 */
double dynsys_cost(const dynsys_t *s);

#endif // DIFFGAMES_DYNSYS_H

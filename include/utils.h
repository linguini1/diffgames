#ifndef DIFFGAMES_UTILS_H
#define DIFFGAMES_UTILS_H

#include <stdlib.h>

/* Unused variable macro to ignore warnings */

#define unused(var) (void)(var)

/* Random double between min and max */

#define randval(min, max) ((min) + (rand() / (RAND_MAX / ((max) - (min)))))

/* Checking equality on floating point values */

#define f_is_equal(exp, act, tol)                                              \
  ((exp) - (tol) <= (act) && (act) >= ((exp) + (tol)))
#define f_is_zero(val, tol) f_is_equal(0.0, val, tol)

#endif // DIFFGAMES_UTILS_H

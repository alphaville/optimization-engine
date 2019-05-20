#define OPEN_NUM_DECISION_VARIABLES 6
#define OPEN_NUM_STATIC_PARAMETERS 2

#ifndef _OPEN_GUARD_
#define _OPEN_GUARD_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Opaque wrapper around PANOCCache, needed for cbindgen to generate a struct
 */
typedef struct PanocInstance PanocInstance;

/**
 * C version of SolverStatus
 */
typedef struct {
  /**
   * number of iterations for convergence
   */
  unsigned long num_iter;
  /**
   * time it took to solve
   */
  unsigned long long solve_time_ns;
  /**
   * norm of the fixed-point residual (FPR)
   */
  double fpr_norm;
  /**
   * cost value at the candidate solution
   */
  double cost_value;
} SolverStatus;

/**
 * Deallocate the solver's memory
 */
void panoc_free(PanocInstance *instance);

/**
 * Allocate memory and setup the solver
 * Note that `max_solve_time_ns` may be set to 0 if an infinite time in desired
 */
PanocInstance *panoc_new(unsigned long lbfgs_memory_size,
                         double tolerance,
                         unsigned long long max_solve_time_ns,
                         unsigned long max_iterations);

/**
 * Run the solver on the input and parameters without any constraints
 */
SolverStatus panoc_solve_no_constraints(PanocInstance *instance, double *u, const double *params);

/**
 * Run the solver on the input and parameters without constraints
 * The `center` may be set to `NULL`, then the origin is assumed as center
 */
SolverStatus panoc_solve_with_ball2_constraints(PanocInstance *instance,
                                                double *u,
                                                const double *params,
                                                const double *center,
                                                double radius);

/**
 * Run the solver on the input and parameters with rectangle constraints
 * `xmin` or `xmax` may be set to `NULL` to indicate that the corresponding constraint is not
 * active, however not both may be `NULL` at the same time
 */
SolverStatus panoc_solve_with_rectangle_constraints(PanocInstance *instance,
                                                    double *u,
                                                    const double *params,
                                                    const double *xmin,
                                                    const double *xmax);

#endif /* _OPEN_GUARD_ */

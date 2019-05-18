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
  unsigned long long num_iter;
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
 * Allocate memory for the solver
 */
PanocInstance *panoc_new(void);

/**
 * Run the solver on the input and parameters
 */
SolverStatus panoc_solve(PanocInstance *instance, double *u_ptr);

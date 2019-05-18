#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct PanocInstance PanocInstance;

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

void panoc_free(PanocInstance *instance);

PanocInstance *panoc_new(void);

SolverStatus panoc_solve(PanocInstance *instance, double *u_ptr);

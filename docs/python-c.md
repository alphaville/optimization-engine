---
id: python-c
title: C/C++ Bindings
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

When using any of the tools to auto-generate a solver it is directly supported to also generate C/C++ bindings for integrating the solver from any language which supports a C or C++ application binary interface ([ABI]). This is a powerful feature when packaging a solver for distribution, or including it into a larger project.

[ABI]: https://en.wikipedia.org/wiki/Application_binary_interface

## Generating bindings

Generating bindings is as adding `with_build_c_bindings()` to your build configuration. 
Here is a full example configuration for creating a solver. We will create an optimizer 
for the Rosenbrock function with equality and bound constraints on the decision variables:

```python
import casadi.casadi as cs
import opengen as og

#
# Problem definition
#

# Decision variabels
u = cs.SX.sym("u", 5)

# Parameters
p = cs.SX.sym("p", 2)

# Cost function
phi = og.functions.rosenbrock(u, p)

# Equality constraints: 1.5 * u[0] = u[1] and u[2] = u[3]
# Can also be inequality constratins using max{c(u, p), 0}, where c(u, p) < 0.
c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])

# Bounds constraints on u
umin = [-2.0, -2.0, -2.0, -2.0, -2.0]
umax = [ 2.0,  2.0,  2.0,  2.0,  2.0]

#
# Set up the solver
#

# Bounds on u: (umin <= u <= umax)
bounds = og.constraints.Rectangle(umin, umax)

# Define the problem
problem = og.builder.Problem(u, p, phi)                 \
    .with_penalty_constraints(c)                        \
    .with_constraints(bounds)

# Meta information for the solver
meta = og.config.OptimizerMeta()                        \
    .with_version("1.0.0")                              \
    .with_authors(["P. Sopasakis", "E. Fresk"])         \
    .with_licence("CC4.0-By")                           \
    .with_optimizer_name("the_optimizer")

# Lets build in release mode with C bindings
build_config = og.config.BuildConfiguration()           \
    .with_rebuild(True)                                 \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <- The important setting

# Solver settings
solver_config = og.config.SolverConfiguration()         \
    .with_tolerance(1e-5)                               \
    .with_constraints_tolerance(1e-4)                   \
    .with_max_outer_iterations(15)                      \
    .with_penalty_weight_update_factor(8.0)             \
    .with_initial_penalty_weights([20.0, 5.0])

# Create the solver!
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)

builder.build()
```

The generated C/C++ bindings are in the auto-generated solver library.
In particular

* The header files are at `the_optimizer/the_optimizer_bindings.{h,hpp}` 
* The static and dynamical library files are located in `the_optimizer/target/{debug,release}` (depending on whether it was a [*debug*] or [*release*] build) 

Note that `the_optimizer` is the name given to the optimizer in the Python codegen above.

[*debug*]: /optimization-engine/docs/python-advanced#build-options
[*release*]: /optimization-engine/docs/python-advanced#build-options


## Bindings API

The generated API has the following functions available (for complete definitions see the generated header files):

```c
/* Contents of */
#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define {SOLVER-NAME}_N1 0
#define {SOLVER-NAME}_N2 2
#define {SOLVER-NAME}_NUM_DECISION_VARIABLES 5
#define {SOLVER-NAME}_NUM_PARAMETERS 2

/**
 * {solver_name} version of ExitStatus
 * Structure: `{solver_name}ExitStatus`
 */
typedef enum {
  /**
   * The algorithm has converged
   * All termination criteria are satisfied and the algorithm
   * converged within the available time and number of iterations
   */
  {solver_name}Converged,
  /**
   * Failed to converge: the maximum number of iterations was reached
   */
  {solver_name}NotConvergedIterations,
  /**
   * Failed to converge: the maximum execution time was reached
   */
  {solver_name}NotConvergedOutOfTime,
  /**
   * If the gradient or cost function cannot be evaluated internally
   */
  {solver_name}NotConvergedCost,
  /**
   * Computation failed and NaN/Infinite value was obtained
   */
  {solver_name}NotConvergedNotFiniteComputation,
} {solver_name}ExitStatus;

/**
 * Solver cache (structure `{solver_name}Cache`)
 */
typedef struct {solver_name}Cache {solver_name}Cache;

/**
 * {solver_name} version of AlmOptimizerStatus
 * Structure: `{solver_name}SolverStatus`
 */
typedef struct {
  /**
   * Exit status
   */
  {solver_name}ExitStatus exit_status;
  /**
   * Number of outer iterations
   */
  unsigned long num_outer_iterations;
  /**
   * Total number of inner iterations
   * This is the sum of the numbers of iterations of
   * inner solvers
   */
  unsigned long num_inner_iterations;
  /**
   * Norm of the fixed-point residual of the the problem
   */
  double last_problem_norm_fpr;
  /**
   * Total solve time
   */
  unsigned long long solve_time_ns;
  /**
   * Penalty value
   */
  double penalty;
  /**
   * Norm of delta y divided by the penalty parameter
   */
  double delta_y_norm_over_c;
  /**
   * Norm of F2(u)
   */
  double f2_norm;
  /**
   * Lagrange multipliers
   */
  const double *lagrange;
} {solver_name}SolverStatus;

/**
 * Deallocate the solver's memory, which has been previously allocated
 * using `{solver_name}_new`
 */
void {solver_name}_free({solver_name}Cache *instance);

/**
 * Allocate memory and setup the solver
 */
{solver_name}Cache *{solver_name}_new(void);

/**
 * Solve the parametric optimization problem for a given parameter
 * 
 * 
 * Arguments:
 * - `instance`: re-useable instance of AlmCache, which should 
 *   be created using `{solver_name}_new` (and should be destroyed once 
 *   it is not needed using `{solver_name}_free`
 * - `u`: (on entry) initial guess of solution, (on exit) solution
 *   (length: `{SOLVER-NAME}_NUM_DECISION_VARIABLES`)
 * - `params`:  static parameters of the optimizer
 *   (length: `{SOLVER-NAME}_NUM_PARAMETERS`)
 * - `y0`: Initial guess of Lagrange multipliers (if `0`, the default will
 *   be used; length: `{SOLVER-NAME}_N1`)
 * - `c0`: Initial penalty parameter (provide `0` to use the default initial
 *   penalty parameter
 * 
 * 
 * Returns:
 * Instance of `{solver_name}SolverStatus`, with the solver status
 * (e.g., number of inner/outer iterations, measures of accuracy, 
 * solver time, and the array of Lagrange multipliers at the solution).
 */
{solver_name}SolverStatus {solver_name}_solve({solver_name}Cache *instance,
                                  double *u,
                                  const double *params,
                                  const double *y0,
                                  const double *c0);
```

This is designed to follow a new-use-free pattern. 

Function `{optimizer-name}_new` will allocate memory and setup a new solver instance and can be used to create as many solvers as necessary. Each solver instance can be used with `{optimizer-name}_solve` to solve the corresponding problem as many times as needed. 

Parameter `u` is the starting guess and also the return of the decision variables and `params` is the array of static parameters. The size of `u` and `params` are `{optimizer-name}_NUM_DECISION_VARIABLES` and `{optimizer-name}_NUM_PARAMETERS` respectively. 

Finally, when done with the solver, use `{optimizer-name}_free` to release the memory allocated by `{optimizer-name}_new`.


## Using the bindings in an app

To use the generated solver in C, the following C code can directly be used to interface to the generated solver:

```c
// File: the_optimizer/optimizer.c

#include <stdio.h>
#include "the_optimizer_bindings.h"

int main() {
	double p[THE_OPTIMIZER_NUM_PARAMETERS] = {1.0, 10.0};  // parameter
	double u[THE_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};  // initial guess

	the_optimizerCache *cache = the_optimizer_new();
	the_optimizerSolverStatus status = the_optimizer_solve(cache, u, p);
	the_optimizer_free(cache);

	for (int i = 0; i < THE_OPTIMIZER_NUM_DECISION_VARIABLES; ++i) {
		printf("u[%d] = %g\n", i, u[i]);
	}

	printf("exit status = %d\n", status.exit_status);
	printf("iterations = %lu\n", status.num_inner_iterations);
	printf("outer iterations = %lu\n", status.num_outer_iterations);
	printf("solve time = %f ms\n", (double)status.solve_time_ns / 1000000.0);

	return 0;
}
```

### Compiling and linking

When building and running an application based on the generated libraries it is good to know that GCC is a bit temperamental when linking libraries, so when linking the static library the following can be used as reference:

```console
gcc optimizer.c -l:libthe_optimizer.a -L./target/release -pthread -lm -ldl -o optimizer
```

Or using direct linking (this is the only thing that works with clang):

```console
gcc optimizer.c ./target/release/libthe_optimizer.a -pthread -lm -ldl -o optimizer
```

While the following can be used when using the shared library:

```console
gcc optimizer.c -lthe_optimizer -L./target/release -pthread -lm -ldl -o optimizer
```

### Running

Which will solve the problem and output the following when run:

```console
# When linking with static lib
./optimizer

# When linking with dynamic lib
LD_LIBRARY_PATH=./target/release ./optimizer

u[0] = 0.654738
u[1] = 0.982045
u[2] = 0.98416
u[3] = 0.984188
u[4] = 0.969986
exit status = 0
iterations = 69
outer iterations = 5
solve time = 0.140401 ms
```

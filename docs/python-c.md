---
id: python-c
title: C/C++ Bindings
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

When using any of the tools to auto-generate a solver it is directly supported to also generate C/C++ bindings for integrating the solver into any language which has C ABI support. This is a powerful feature when packaging a solver for distribution, or including it into a larger project.

## Generating bindings

To generate the bindings, it is as simple as to enable the generation of bindings, and here is a full example configuration for creating a solver. We will create an optimizer for the Rosenbrock function with equality and bound constraints on the decision variables:

```python
#
# Problem definition
#

# Decision variabels
u = cs.SX.sym("u", 5)

# Parameters
p = cs.SX.sym("p", 2)

# Cost function
phi = og.functions.rosenbrock(u, p)

# Equality constraints (can also be inequality constratins)
c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])

# Bounds constraints on u
umin = [-2.0, -2.0, -2.0, -2.0, -2.0]
umax = [ 2.0,  2.0,  2.0,  2.0,  2.0]

#
# Set up the solver
#

# Bounds on u
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
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <- The important setting

# Solver settings
solver_config = og.config.SolverConfiguration()         \
    .with_lfbgs_memory(15)                              \
    .with_tolerance(1e-5)                               \
    .with_max_inner_iterations(155)                     \
    .with_constraints_tolerance(1e-4)                   \
    .with_max_outer_iterations(15)                      \
    .with_penalty_weight_update_factor(8.0)             \
    .with_initial_penalty_weights([20.0, 5.0])

# Create the solver!
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config) \
    .with_generate_not_build_flag(False).with_verbosity_level(0)

builder.build()
```

The generated bindings are in the auto-generated solver library as `the_optimizer/the_optimizer_bindings.{h,hpp}` together with the libraries in the `the_optimizer/target/{debug,release}` folders depending on if it was a debug or release build. Note that `the_optimizer` is the name given to the optimizer in the Python codegen above.

**Matlab generation will come soon.**

## Bindings API

The generated API has the following functions available (for complete definitions see the generated files):

```c
// More definitions ...

#define {optimizer-name}_NUM_DECISION_VARIABLES 5
#define {optimizer-name}_NUM_PARAMETERS 2

// Allocate memory and setup the solver
{optimizer-name}Cache *{optimizer-name}_new(void);

// Deallocate the solver's memory
void {optimizer-name}_free({optimizer-name}Cache *instance);

// Run the solver on the input and parameters
{optimizer-name}Status {optimizer-name}_solve({optimizer-name}Cache *instance,
                                              double *u,
                                              const double *params);
```

Which is designed to follow a new, use, free pattern. The `{optimizer-name}_new` will allocate memory and setup a new solver instance, and can be used to create as many solvers as necessary, where each instance can be used with `{optimizer-name}_solve` to solve the specific problem as many times as needed. The parameter `u` is the starting guess and also the return of the decision variables, and `params` are static parameters. The size of `u` and `params` are `{optimizer-name}_NUM_DECISION_VARIABLES` and `{optimizer-name}_NUM_PARAMETERS` respectively. Finally, when done with the solver, use `{optimizer-name}_free` to release the memory allocated by `{optimizer-name}_new`.

## Using the bindings in an app

To try the generated solver from earlier, the following C code can directly be used to interface to the generated solver:

```c
// optimizer.c

#include <stdio.h>
#include "the_optimizer_bindings.h"

int main() {
	double p[THE_OPTIMIZER_NUM_PARAMETERS] = {1.0, 10.0};
	double u[THE_OPTIMIZER_NUM_DECISION_VARIABLES] = {0};

	the_optimizerCache *cache = the_optimizer_new();
	the_optimizerSolverStatus status = the_optimizer_solve(cache, u, p);
	the_optimizer_free(cache);

	for (int i = 0; i < THE_OPTIMIZER_NUM_DECISION_VARIABLES; ++i) {
		printf("u[%d] = %g\n", i, u[i]);
	}

	printf("exit status = %d\n", status.exit_status);
	printf("iterations = %lu\n", status.num_inner_iterations);
	printf("outer iterations = %lu\n", status.num_outer_iterations);

	return 0;
}
```

### Compiling and linking

When building and running the libraries GCC is a bit temperamental when linking libraries, so when linking the static library the following can be used as reference:

```console
gcc optimizer.c -l:libthe_optimizer.a -L./target/release -pthread -lm -ldl -o optimizer
```

Or using direct linking (this is the only thing that works with clang):

```console
gcc optimizer.c -l./target/release/libthe_optimizer.a -pthread -lm -ldl -o optimizer
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

u[0] = 0.895032
u[1] = 0.818222
u[2] = 0.691895
u[3] = 0.495584
u[4] = 0.234752
exit status = 0
iterations = 22
outer iterations = 1
```

---
id: python-c
title: C/C++ Bindings
description: How to call OpEn from C/C++ or from Robot Operating System (ROS)
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

When using any of the tools to auto-generate a solver it is directly supported to also generate C/C++ bindings for integrating the solver from any language which supports a C or C++ application binary interface ([ABI]). This is a powerful feature when packaging a solver for distribution, or including it into a larger project.

[ABI]: https://en.wikipedia.org/wiki/Application_binary_interface

## Generating bindings

Generating bindings is as easy as adding `with_build_c_bindings()` to your build configuration. 
Here is a full example configuration for creating a solver. We will create an optimizer 
for the [Rosenbrock function](example_rosenbrock_py) with equality and bound constraints on the decision variables:

```python
import casadi.casadi as cs
import opengen as og

u = cs.SX.sym("u", 5)  # Decision variabels
p = cs.SX.sym("p", 2)  # Parameters

phi = og.functions.rosenbrock(u, p)  # Cost function

# Equality constraints: 1.5 * u[0] = u[1] and u[2] = u[3]
# Can also be inequality constratins using max{c(u, p), 0}, where c(u, p) < 0.
c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])

# Bounds constraints on u
umin = [-2.0] * 5  # shorthand notation
umax = [ 2.0] * 5

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
    .with_optimizer_name("the_optimizer")

# Lets build in release mode with C bindings
build_config = og.config.BuildConfiguration()           \
    .with_build_mode("release")                         \
    .with_build_c_bindings()            # <--- The important setting

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

**Matlab generation will come soon.**

## Bindings API

The generated API has the following functions available 
(for complete definitions see the generated header files) for 
a solver entitled "example":

```c
/* Contents of header file: example_bindings.h */

#define EXAMPLE_N1 0
#define EXAMPLE_N2 2
#define EXAMPLE_NUM_DECISION_VARIABLES 5
#define EXAMPLE_NUM_PARAMETERS 2

typedef enum {
  exampleConverged,
  exampleNotConvergedIterations,
  exampleNotConvergedOutOfTime,
  exampleNotConvergedCost,
  exampleNotConvergedNotFiniteComputation,
} exampleExitStatus;

typedef struct exampleCache exampleCache;

typedef struct {
  exampleExitStatus exit_status;
  unsigned long num_outer_iterations;
  unsigned long num_inner_iterations;
  double last_problem_norm_fpr;  
  unsigned long long solve_time_ns;
  double penalty;
  double delta_y_norm_over_c;
  double f2_norm;
  const double *lagrange;
} exampleSolverStatus;

void example_free(exampleCache *instance);

exampleCache *example_new(void);

exampleSolverStatus example_solve(exampleCache *instance,
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

Suppose that you have compiled your optimizer using the option
`with_build_c_bindings()` and you have generated C/C++ bindings.
OpEn will generate an example C file that you can use as a starting point
to build your application. This example file is stored in the directory
containing all other files of your optimizer and is called `example_optimizer.c`.
The auto-generated example has the following form:

```c
/* File: the_optimizer/example_optimizer.c  */

#include <stdio.h>
#include "example_bindings.h"

int main() {
	double p[EXAMPLE_NUM_PARAMETERS] = {1.0, 10.0};  // parameter
	double u[EXAMPLE_NUM_DECISION_VARIABLES] = {0};  // initial guess

	exampleCache *cache = example_new();
	exampleSolverStatus status = example_solve(cache, u, p);
	example_free(cache);

	for (int i = 0; i < EXAMPLE_NUM_DECISION_VARIABLES; ++i) {
		printf("u[%d] = %g\n", i, u[i]);
	}

	printf("exit status = %d\n", status.exit_status);
	printf("iterations = %lu\n", status.num_inner_iterations);
	printf("outer iterations = %lu\n", status.num_outer_iterations);
	printf("solve time = %f ms\n", (double)status.solve_time_ns / 1e6);

	return 0;
}
```



### Compiling and linking
#### Using cmake 
To compile your C program you need to link to the auto-generated
C bindings (see [next section](#compile-your-own-code)). 
However, OpEn generates automatically a `CMakeLists.txt` file
to facilitate the compilation/linking procedure. To build the
auto-generated example run

```console
cmake .
make
```

once you build your optimizer you can run the executable (`optimizer`)
with 

```console
make run
```

#### Compile your own code 
When building and running an application based on the generated libraries 
it is good to know that GCC is a bit temperamental when linking libraries,
so when linking the static library the following can be used as reference:

```console
gcc example_optimizer.c -l:libthe_optimizer.a \
  -L./target/release -pthread -lm -ldl -o optimizer
```

Or using direct linking (this is the only thing that works with clang):

```console
gcc example_optimizer.c ./target/release/libthe_optimizer.a \
  -pthread -lm -ldl -o optimizer
```

While the following can be used when using the shared library:

```console
gcc example_optimizer.c -lthe_optimizer \
  -L./target/release -pthread -lm -ldl -o optimizer
```

### Running

Which will solve the problem and output the following when run:

```console
# When linking with static lib
./optimizer
```

```console
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

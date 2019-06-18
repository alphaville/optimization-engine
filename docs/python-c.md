---
id: python-c
title: C/C++ Bindings
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

When using any of the tools to auto-generate a solver it is directly supported to also generate C/C++ bindings for integrating the solver into any language which has C ABI support. This is a powerful feature when packaging a solver for distribution, or including it into a larger project.

## Generating bindings

To generate the bindings, it is as simple as to enable the generation of bindings as:

```python
# Python generation

build_config = og.config.BuildConfiguration()   \
    # ...
    .with_build_c_bindings()
```

The generated bindings are in the auto-generated solver library as `{optimizer-name}/{optimizer-name}_bindings.{h,hpp}` together with the libraries in the `{optimizer-name}/target/{debug,release}` folders depending on if it was a debug or release build. Note that `{optimizer-name}` is the name given to the optimizer in the Python codegen.

Matlab generation will come soon.

## Bindings API

The generated API has the following functions available (for complete definitions see the generated file):

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

Which is designed to follow a create, use, free pattern. The `{optimizer-name}_new` will allocate memory and setup a new solver instance, and can be used to create as many solvers as necessary, where each instance can be used with `{optimizer-name}_solve` to solve the specific problem as many times as needed. The parameter `u` is the starting guess and also the return of the decision variables, and `params` are static parameters. The size of `u` and `params` are `{optimizer-name}_NUM_DECISION_VARIABLES` and `{optimizer-name}_NUM_PARAMETERS` respectively. Finally, when done with the solver, use `{optimizer-name}_free` to release the memory allocated by `{optimizer-name}_new`.

## Using the bindings in an app

Bellow is an example of using the bindings in a simple C program, lets assume the solver was named `the_optimizer`, i.e. `{optimizer-name}_optimizer` = `the_optimizer`

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

Which will output the following when run:

```console
u[0] = 0.895032
u[1] = 0.818222
u[2] = 0.691895
u[3] = 0.495584
u[4] = 0.234752
exit status = 0
iterations = 22
outer iterations = 1
```

When building and running the libraries GCC is a bit temperamental when linking libraries, so when linking the static library the following can be used as reference:

```console
gcc optimizer.c -l:libthe_optimizer.a -L./target/debug -pthread -lm -ldl -o optimizer
./optimizer
```

While the following can be used when using the shared library:

```console
gcc optimizer.c -lthe_optimizer -L./target/debug -pthread -lm -ldl -o optimizer
env LD_LIBRARY_PATH=./target/debug ./optimizer
```


---
id: python-c
title: C/C++ Bindings
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

When using any of the tools to auto-generate a solver it is directly supported to also generate C/C++ bindings for integrating the solver into any language which has C ABI support.

## Generating bindings

To generate the bindings, it is as simple as to enable the generation of bindings.

### Generate bindings in Python

```python
build_config = og.config.BuildConfiguration()   \
    # ...
    .with_build_c_bindings()
```

### Generate bindings in MATLAB

Matlab support is on the way, but not available yet.

## Bindings API

The generated API has the following functions available:

```c
// More definitions ...

#define NUM_DECISION_VARIABLES 5
#define NUM_PARAMETERS 2

// Allocate memory and setup the solver
HomotopyCache *solver_new(void);

// Deallocate the solver's memory
void solver_free(HomotopyCache *instance);

// Run the solver on the input and parameters
CHomotopySolverStatus solver_solve(HomotopyCache *instance, double *u, const double *params);
```

Which is designed to follow a create, use, free pattern. The `solver_new` will allocate memory and setup a new solver instance, and can be used to create as many solvers as necessary, where each instance can be used with `solver_solve` to solve the specific problem as many times as needed. The parameter `u` is the starting guess and also the return of the decision variables, and `params` are static parameters. The size of `u` and `params` are `NUM_DECISION_VARIABLES` and `NUM_PARAMETERS` respectively. Finally, when done with the solver, use `solver_free` to release the memory allocated by `solver_new`.



## Using the bindings in an app



### Linking Static library

### Linking Shared library


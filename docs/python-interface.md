---
id: python-interface
title: Python
---

The Python interface is under development

Sample:


```python
from casadi import SX, Function
from opengen import *

u = SX.sym("u", 5)
p = SX.sym("p", 2)

# cost function
phi = rosenbrock(u, p)

# c(u; p)
c = vertcat(norm_2(u) - 1.,
            u[0] + p[0] * u[1] - 3.,
            (p[0] + p[1]) * (u[0] + u[1]) - 2.)

# Problem statement
xmin = [-1.0, -2.0, -1.0, -1.0, -3.0]
xmax = [2.0, 1.0, 3.0, 4.0, 1.0]
bounds = Rectangle(xmin, xmax)

problem = Problem(u, p, phi) \
    .with_penalty_constraints(c) \
    .with_constraints(bounds)

# Metadata
meta = OptimizerMeta() \
    .with_version("0.0.2") \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By") \
    .with_optimizer_name("funky_optimizer")

# Build configuration
build_config = BuildConfiguration() \
    .with_rebuild(True) \
    .with_build_mode("debug")

# Solver configuration
solver_config = SolverConfiguration() \
    .with_lfbgs_memory(15) \
    .with_tolerance(1e-5) \
    .with_max_iterations(135)

# Auto-generate code and build project
builder = OpEnOptimizerBuilder(problem,
                               meta=meta,
                               build_config=build_config,
                               solver_config=solver_config) \
    .with_generate_not_build_flag(True)

builder.build()
```


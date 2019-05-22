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
problem = Problem(u, p, phi) \
    .with_penalty_constraints(c)

# Metadata
meta = OptimizerMeta(). \
    with_version("0.0.2"). \
    with_authors(["P. Sopasakis", "E. Fresk"]). \
    with_licence("CC4.0-By"). \
    with_optimizer_name("funky_optimizer")

# Build configuration
build_config = BuildConfiguration()
build_config.with_rebuild(False)

# Auto-generate code and build project
builder = OpEnOptimizerBuilder(problem,
                               meta=meta,
                               build_config=build_config)
builder.build()

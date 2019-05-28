import casadi.casadi as cs
import opengen as og

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)

# cost function
phi = og.functions.rosenbrock(u, p)

# c(u; p)
c = 1.5*u[0] - u[1]

# Constraints
xmin = [-1.0, -2.0, -1.0, -1.0, -3.0]
xmax = [2.0, 1.0, 3.0, 4.0, 1.0]
bounds = og.constraints.Rectangle(xmin, xmax)

# Problem statement: the oracle
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c) \
    .with_constraints(bounds)

# Metadata of auto-generated Rust package
meta = og.config.OptimizerMeta() \
    .with_version("0.0.2") \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By") \
    .with_optimizer_name("wow_optimizer")

# Build configuration
# - build mode (debug/release), where to store it, which version
#   of optimization_engine to use (must be on crates.io)
build_config = og.config.BuildConfiguration() \
    .with_rebuild(False) \
    .with_build_mode("debug") \
    .with_build_directory("yyy") \
    .with_open_version("0.3.2")

# Solver configuration (tolerance, L-BFGS memory, etc)
solver_config = og.config.SolverConfiguration() \
    .with_lfbgs_memory(15) \
    .with_tolerance(1e-5) \
    .with_max_iterations(155)

# Auto-generate code and build project
# - We can generate the code, but not build it
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config) \
    .with_generate_not_build_flag(False) \
    .build()

print('DONE :-)')


import opengen as og
import casadi.casadi as cs

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)        \
    .with_constraints(bounds)
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("my_optimizers")     \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()               \
    .with_build_c_bindings()                   \
    .with_ros()
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("rosenbrock")             \
    .with_version('0.1.1')                         \
    .with_licence('LGPLv3')
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1e3)                     \
    .with_penalty_weight_update_factor(5)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
    .with_generate_not_build_flag(True)
builder.build()

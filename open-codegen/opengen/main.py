import opengen as og
import casadi.casadi as cs


u = cs.MX.sym("u", 5)                 # decision variable (nu = 5)
p = cs.MX.sym("p", 2)                 # parameter (np = 2)


f1 = cs.vertcat(1 - u[1],
                2*cs.sin(u[2]) - 0.1)
f2 = u[3]

set_c = og.constraints.BallInf([0.1, 0.2], 0.6)
set_y = og.constraints.Rectangle([-1e12, -1e12], [1e12, 1e12])
phi = og.functions.rosenbrock(u, p)       # cost function
bounds = og.constraints.Ball2(None, 1.5)  # ball centered at origin
problem = og.builder.Problem(u, p, phi)\
    .with_constraints(bounds)\
    .with_aug_lagrangian_constraints(f1, set_c, set_y) \
    .with_penalty_constraints(f2)
meta = og.config.OptimizerMeta()                \
    .with_version("0.1.0")                      \
    .with_authors(["P. Sopasakis"])             \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("marietta")
build_config = og.config.BuildConfiguration()   \
    .with_build_directory("python_build")       \
    .with_build_mode("debug")                   \
    .with_tcp_interface_config()
solver_config = og.config.SolverConfiguration()         \
            .with_lfbgs_memory(15)                      \
            .with_tolerance(1e-6)                       \
            .with_initial_tolerance(1e-3)               \
            .with_delta_tolerance(1e-3)                 \
            .with_initial_penalty(15.0)                 \
            .with_penalty_weight_update_factor(10.0)    \
            .with_max_inner_iterations(155)             \
            .with_max_duration_micros(1e8)              \
            .with_max_outer_iterations(50)              \
            .with_sufficient_decrease_coefficient(0.05) \
            .with_cbfgs_parameters(1.5, 1e-10, 1e-12)
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)
builder.build()


mng = og.tcp.OptimizerTcpManager('python_build/marietta')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
print(mng.call(p=[1.0, 2.0]))
mng.kill()

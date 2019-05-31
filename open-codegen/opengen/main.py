import casadi.casadi as cs
import opengen as og

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5*u[0] - u[1],
               u[2] - u[3],
               cs.cos(u[4]))
xmin = [-2., -1., -1., -1., -4.]
xmax = [1., 3., 1., 1., 4.]

bounds = og.constraints.Rectangle(xmin, xmax)

problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c).with_constraints(bounds)


meta = og.config.OptimizerMeta() \
    .with_version("0.0.2") \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By") \
    .with_optimizer_name("test_optimizer")

build_config = og.config.BuildConfiguration() \
    .with_rebuild(False) \
    .with_build_mode("debug") \
    .with_build_directory(".python_codegen_builds") \
    .with_open_version("0.3.2")

solver_config = og.config.SolverConfiguration() \
    .with_lfbgs_memory(15) \
    .with_tolerance(1e-5) \
    .with_max_inner_iterations(1000) \
    .with_constraints_tolerance(1e-4) \
    .with_max_outer_iterations(25) \
    .with_penalty_weight_update_factor(5) \
    .with_initial_penalty_weights(2.0) \
    .with_max_duration_micros(3000000)

og.builder.OpEnOptimizerBuilder(problem,
                                metadata=meta,
                                build_configuration=build_config,
                                solver_configuration=solver_config) \
    .with_generate_not_build_flag(False) \
    .build()

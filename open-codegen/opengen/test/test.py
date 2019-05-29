import unittest
import casadi.casadi as cs
import opengen as og


class RustBuildTestCase(unittest.TestCase):

    def test_rust_build_with_penalty(self):
        u = cs.SX.sym("u", 15)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5 * u[0] - u[1], u[2] - u[3])
        bounds = og.constraints.Ball2(None, 1.5)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(c) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(".yyy") \
            .with_build_mode("debug")
        og.builder.OpEnOptimizerBuilder(problem, build_configuration=build_config) \
            .with_generate_not_build_flag(False).build()

    def test_rust_build_without_penalty(self):
        u = cs.SX.sym("u", 15)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        bounds = og.constraints.Ball2(None, 1.5)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(None) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(".yyy") \
            .with_build_mode("debug")
        og.builder.OpEnOptimizerBuilder(problem, build_configuration=build_config) \
            .with_generate_not_build_flag(False).build()
    
    def test_fully_featured_release_mode(self):
        u = cs.SX.sym("u", 5)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])
        xmin = [-2.0, -2.0, -2.0, -2.0, -2.0]
        xmax = [2.0, 2.0, 2.0, 2.0, 2.0]
        bounds = og.constraints.Rectangle(xmin, xmax)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(c) \
            .with_constraints(bounds)
        meta = og.config.OptimizerMeta() \
            .with_version("0.0.2") \
            .with_authors(["P. Sopasakis", "E. Fresk"]) \
            .with_licence("CC4.0-By") \
            .with_optimizer_name("wow_optimizer")
        build_config = og.config.BuildConfiguration() \
            .with_rebuild(False) \
            .with_build_mode("debug") \
            .with_build_directory(".yyy") \
            .with_open_version("0.3.2")
        solver_config = og.config.SolverConfiguration() \
            .with_lfbgs_memory(15) \
            .with_tolerance(1e-5) \
            .with_max_inner_iterations(155) \
            .with_constraints_tolerance(1e-4) \
            .with_max_outer_iterations(15) \
            .with_penalty_weight_update_factor(8.0) \
            .with_initial_penalty_weights([20.0, 5.0])
        og.builder.OpEnOptimizerBuilder(problem,
                                        metadata=meta,
                                        build_configuration=build_config,
                                        solver_configuration=solver_config) \
            .with_generate_not_build_flag(False) \
            .build()


if __name__ == '__main__':
    unittest.main()

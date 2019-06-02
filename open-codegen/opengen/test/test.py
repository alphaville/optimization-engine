import unittest
import casadi.casadi as cs
import opengen as og


class RustBuildTestCase(unittest.TestCase):

    TEST_DIR = ".python_test_build"
    OPEN_VERSION = "0.4.0-alpha"

    def test_rectangle_empty(self):
        xmin = [-1, 2]
        xmax = [-2, 4]
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(xmin, xmax)

    def test_rectangle_incompatible_dimensions(self):
        xmin = [-1, -1, 1]
        xmax = [1, 1]
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(xmin, xmax)

    def test_rectangle_both_none(self):
        with self.assertRaises(Exception) as __context:
            og.constraints.Rectangle(None, None)

    def test_ball_negative_radius(self):
        with self.assertRaises(Exception) as __context:
            og.constraints.Ball2(None, -1)

    def test_solver_config_wrong_max_duration(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_duration_micros(0)

    def test_solver_config_wrong_update_factor(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_penalty_weight_update_factor(0.5)

    def test_solver_config_wrong_outer_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_outer_iterations(0)

    def test_solver_config_wrong_inner_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_inner_iterations(0)

    def test_solver_config_wrong_constraints_tolerance(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_constraints_tolerance(0)

    def test_solver_config_wrong_inner_tolerance(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_tolerance(0)

    def test_solver_config_wrong_lbfgs_memory(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_lfbgs_memory(1)

    def test_solver_config_wrong_max_inner_iterations(self):
        with self.assertRaises(Exception) as __context:
            og.config.SolverConfiguration().with_max_inner_iterations()

    def test_build_incompatible_dim_(self):
        u = cs.SX.sym("u", 5)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(u[0], u[1],  u[2] - u[3])
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(c)
        build_config = og.config.BuildConfiguration()
        solver_config = og.config.SolverConfiguration() \
            .with_initial_penalty_weights([1.0, 2.0]) # should have dim = 3
        with self.assertRaises(Exception) as __context:
            og.builder.OpEnOptimizerBuilder(problem,
                                            build_configuration=build_config,
                                            solver_configuration=solver_config) \
                .with_generate_not_build_flag(True).build()

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
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode("debug").with_open_version(self.OPEN_VERSION)
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
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode("debug").with_open_version(self.OPEN_VERSION)
        og.builder.OpEnOptimizerBuilder(problem, build_configuration=build_config) \
            .with_generate_not_build_flag(False).build()

    def test_rust_build_with_tcp_server(self):
        u = cs.SX.sym("u", 5)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        bounds = og.constraints.Ball2(None, 1.5)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(None) \
            .with_constraints(bounds)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_build_mode("debug").with_open_version(self.OPEN_VERSION)
        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("tcp_enabled_optimizer")
        builder = og.builder.OpEnOptimizerBuilder(problem,
                                                  metadata=meta,
                                                  build_configuration=build_config) \
            .with_generate_not_build_flag(False).with_verbosity_level(1)
        builder.enable_tcp_interface()
        builder.build()

    def test_fully_featured_release_mode(self):
        u = cs.SX.sym("u", 5)
        p = cs.SX.sym("p", 2)
        phi = og.functions.rosenbrock(u, p)
        c = cs.vertcat(1.5*u[0] - u[1], u[2] - u[3])
        xmin = [-2.0, -2.0, -2.0, -2.0, -2.0]
        xmax = [ 2.0,  2.0,  2.0,  2.0,  2.0]
        bounds = og.constraints.Rectangle(xmin, xmax)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(c)        \
            .with_constraints(bounds)
        meta = og.config.OptimizerMeta()                \
            .with_version("0.0.0")                      \
            .with_authors(["P. Sopasakis", "E. Fresk"]) \
            .with_licence("CC4.0-By")                   \
            .with_optimizer_name("the_optimizer")
        build_config = og.config.BuildConfiguration()         \
            .with_rebuild(False)                              \
            .with_build_mode("debug")                         \
            .with_build_directory(RustBuildTestCase.TEST_DIR) \
            .with_open_version(self.OPEN_VERSION)
        solver_config = og.config.SolverConfiguration()   \
            .with_lfbgs_memory(15)                        \
            .with_tolerance(1e-5)                         \
            .with_max_inner_iterations(155)               \
            .with_constraints_tolerance(1e-4)             \
            .with_max_outer_iterations(15)                \
            .with_penalty_weight_update_factor(8.0)       \
            .with_initial_penalty_weights([20.0, 5.0])
        builder = og.builder.OpEnOptimizerBuilder(problem,
                                                  metadata=meta,
                                                  build_configuration=build_config,
                                                  solver_configuration=solver_config) \
            .with_generate_not_build_flag(False).with_verbosity_level(0)
        builder.build()

    def test_tcp_server(self):
        tcp_manager = og.tcp.OptimizerTcpManager(
            RustBuildTestCase.TEST_DIR + '/tcp_enabled_optimizer')
        tcp_manager.start()

        for i in range(100):
            tcp_manager.ping()

        for i in range(100):
            result = tcp_manager.call(p=[1.0, 10.0+i],
                                      initial_guess=[1.0, 1.0, 2.0 + 0.5*i, 3.0, 4.0])
            _exit_status = result['exit_status']

        tcp_manager.kill()


if __name__ == '__main__':
    unittest.main()

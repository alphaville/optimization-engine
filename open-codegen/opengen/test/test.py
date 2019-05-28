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
            .with_build_directory("yyy")
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
            .with_build_directory("yyy")
        og.builder.OpEnOptimizerBuilder(problem, build_configuration=build_config) \
            .with_generate_not_build_flag(False).build()


if __name__ == '__main__':
    unittest.main()

import opengen as og
import unittest
import casadi.casadi as cs
import numpy as np
import math


class RaspberryPiTest(unittest.TestCase):

    # -----------------------------------------------------------------------
    # Cross-compile to Raspberry Pi
    # -----------------------------------------------------------------------
    def test_compile_rpi(self):
        optimizers_dir = "my_optimizers"
        optimizer_name = "rosenbrock"
        nu, np = 5, 2
        u = cs.SX.sym("u", nu)  # decision variable (nu = 5)
        p = cs.SX.sym("p", np)  # parameter (np = 2)
        phi = og.functions.rosenbrock(u, p) + 1500*cs.sum1(u)
        c_f2 = cs.vertcat(0.2 + 1.5 * u[0] - u[1], u[2] - u[3] - 0.1)
        bounds = og.constraints.Ball2(None, 1.5)
        problem = og.builder.Problem(u, p, phi) \
            .with_penalty_constraints(c_f2)\
            .with_constraints(bounds)
        meta = og.config.OptimizerMeta()\
            .with_optimizer_name(optimizer_name)
        build_config = og.config.BuildConfiguration() \
            .with_build_directory(optimizers_dir) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config() \
            .with_target_system("rpi")
        solver_cfg = og.config.SolverConfiguration() \
            .with_penalty_weight_update_factor(1.5) \
            .with_preconditioning(True)
        builder = og.builder.OpEnOptimizerBuilder(problem,
                                                  meta, 
                                                  build_config,
                                                  solver_cfg)
        builder.build()


if __name__ == '__main__':
    unittest.main()

import os
import unittest

import casadi.casadi as cs
import opengen as og


class DummySolverStatus:
    def __init__(self, solution):
        self.solution = solution
        self.cost = 1.23
        self.exit_status = "Converged"
        self.solve_time_ms = 0.7
        self.lagrange_multipliers = []


class DummyDirectSolver:
    def __init__(self, solution):
        self.solution = solution
        self.last_call = None

    def run(self, p, initial_guess=None, initial_lagrange_multipliers=None, initial_penalty=None):
        self.last_call = {
            "p": p,
            "initial_guess": initial_guess,
            "initial_lagrange_multipliers": initial_lagrange_multipliers,
            "initial_penalty": initial_penalty,
        }
        return DummySolverStatus(self.solution)


class OcpTestCase(unittest.TestCase):
    TEST_DIR = ".python_test_build_ocp"

    @staticmethod
    def get_open_local_absolute_path():
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

    def make_ocp(self, shooting=og.ocp.ShootingMethod.SINGLE):
        ocp = og.ocp.OptimalControlProblem(nx=2, nu=1, horizon=3, shooting=shooting)
        ocp.add_parameter("x0", 2)
        ocp.add_parameter("xref", 2, default=[1.0, -1.0])

        def dynamics(x, u, param):
            return cs.vertcat(x[0] + u[0], x[1] - u[0])

        def stage_cost(x, u, param, _t):
            err = x - param["xref"]
            return cs.dot(err, err) + 0.5 * cs.dot(u, u)

        def terminal_cost(x, param):
            err = x - param["xref"]
            return 2.0 * cs.dot(err, err)

        ocp.with_dynamics(dynamics)
        ocp.with_stage_cost(stage_cost)
        ocp.with_terminal_cost(terminal_cost)
        ocp.with_input_constraints(og.constraints.Rectangle([-2.0], [2.0]))
        ocp.with_path_constraint(lambda x, u, param, _t: x[0] + u[0] - param["xref"][0])
        return ocp

    def test_parameter_defaults_are_packed(self):
        ocp = self.make_ocp()
        packed = ocp.parameters.pack({"x0": [0.5, -0.25]})
        self.assertEqual(packed, [0.5, -0.25, 1.0, -1.0])

    def test_symbolic_lowering_builds_penalty_mapping_and_cartesian_constraints(self):
        ocp = self.make_ocp()
        builder = og.ocp.OCPBuilder(
            ocp,
            metadata=og.config.OptimizerMeta().with_optimizer_name("ocp_test"),
        )
        low_level = builder.build_problem()

        self.assertEqual(low_level.dim_decision_variables(), 3)
        self.assertEqual(low_level.dim_parameters(), 4)
        self.assertEqual(low_level.dim_constraints_penalty(), 3)
        self.assertIsInstance(low_level.constraints, og.constraints.CartesianProduct)
        self.assertEqual(low_level.constraints.segments, [0, 1, 2])

    def test_generated_optimizer_uses_named_parameters_and_defaults(self):
        ocp = self.make_ocp()
        backend = DummyDirectSolver(solution=[0.1, 0.2, 0.3])
        optimizer = og.ocp.GeneratedOptimizer(
            ocp=ocp,
            optimizer_name="dummy",
            target_dir=".",
            backend=backend,
            backend_kind="direct",
        )

        result = optimizer.solve(x0=[0.0, 0.0])
        print(result)

        self.assertEqual(backend.last_call["p"], [0.0, 0.0, 1.0, -1.0])
        self.assertEqual(result.inputs, [[0.1], [0.2], [0.3]])
        self.assertEqual(result.states[0], [0.0, 0.0])
        self.assertEqual(result.states[-1], [0.6000000000000001, -0.6000000000000001])

    def test_multiple_shooting_builds_decision_vector_and_defect_constraints(self):
        ocp = self.make_ocp(shooting=og.ocp.ShootingMethod.MULTIPLE)
        builder = og.ocp.OCPBuilder(
            ocp,
            metadata=og.config.OptimizerMeta().with_optimizer_name("ocp_ms_test"),
        )
        low_level = builder.build_problem()

        self.assertEqual(low_level.dim_decision_variables(), 9)
        self.assertEqual(low_level.dim_parameters(), 4)
        self.assertEqual(low_level.dim_constraints_penalty(), 9)
        self.assertIsInstance(low_level.constraints, og.constraints.CartesianProduct)
        self.assertEqual(low_level.constraints.segments, [0, 2, 3, 5, 6, 8])

    def test_multiple_shooting_solution_extracts_states_from_decision_vector(self):
        ocp = self.make_ocp(shooting=og.ocp.ShootingMethod.MULTIPLE)
        backend = DummyDirectSolver(solution=[
            0.1, 0.1, -0.1,
            0.2, 0.3, -0.3,
            0.4, 0.7, -0.7,
        ])
        optimizer = og.ocp.GeneratedOptimizer(
            ocp=ocp,
            optimizer_name="dummy_ms",
            target_dir=".",
            backend=backend,
            backend_kind="direct",
        )

        result = optimizer.solve(x0=[0.0, 0.0])

        self.assertEqual(result.inputs, [[0.1], [0.2], [0.4]])
        self.assertEqual(result.states, [[0.0, 0.0], [0.1, -0.1], [0.3, -0.3], [0.7, -0.7]])

    def test_ocp_generates_rust_solver_and_calls_tcp_interface(self):
        optimizer_name = "ocp_tcp_smoke"
        port = 3391

        ocp = og.ocp.OptimalControlProblem(nx=1, nu=1, horizon=2)
        ocp.add_parameter("x0", 1)
        ocp.add_parameter("xref", 1, default=[0.0])
        ocp.with_dynamics(lambda x, u, param: x + u)
        ocp.with_stage_cost(
            lambda x, u, param, _t: cs.dot(x - param["xref"], x - param["xref"]) + 0.1 * cs.dot(u, u)
        )
        ocp.with_terminal_cost(
            lambda x, param: 2.0 * cs.dot(x - param["xref"], x - param["xref"])
        )
        ocp.with_input_constraints(og.constraints.Rectangle([-1.0], [1.0]))

        meta = og.config.OptimizerMeta().with_optimizer_name(optimizer_name)
        build_config = og.config.BuildConfiguration() \
            .with_open_version(local_path=OcpTestCase.get_open_local_absolute_path()) \
            .with_build_directory(OcpTestCase.TEST_DIR) \
            .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
            .with_tcp_interface_config(
                tcp_interface_config=og.config.TcpServerConfiguration(bind_port=port)
            )
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(1e-5) \
            .with_initial_tolerance(1e-5) \
            .with_delta_tolerance(1e-5) \
            .with_initial_penalty(10.0) \
            .with_penalty_weight_update_factor(5.0) \
            .with_max_inner_iterations(50) \
            .with_max_outer_iterations(10)

        builder = og.ocp.OCPBuilder(
            ocp,
            metadata=meta,
            build_configuration=build_config,
            solver_configuration=solver_config,
        )
        builder.build()

        mng = og.tcp.OptimizerTcpManager(
            optimizer_path=os.path.join(OcpTestCase.TEST_DIR, optimizer_name)
        )
        mng.start()

        try:
            pong = mng.ping()
            self.assertEqual(1, pong["Pong"])

            response = mng.call(p=[1.0, 0.0])
            self.assertTrue(response.is_ok())
            status = response.get()
            self.assertEqual("Converged", status.exit_status)
            self.assertEqual(2, len(status.solution))
        finally:
            mng.kill()


if __name__ == "__main__":
    unittest.main()

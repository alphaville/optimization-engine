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
    def make_ocp(self):
        ocp = og.ocp.OptimalControlProblem(nx=2, nu=1, horizon=3)
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

        self.assertEqual(backend.last_call["p"], [0.0, 0.0, 1.0, -1.0])
        self.assertEqual(result.inputs, [[0.1], [0.2], [0.3]])
        self.assertEqual(result.states[0], [0.0, 0.0])
        self.assertEqual(result.states[-1], [0.6000000000000001, -0.6000000000000001])


if __name__ == "__main__":
    unittest.main()

import importlib
import os
import sys
import casadi.casadi as cs

from opengen.builder.optimizer_builder import OpEnOptimizerBuilder
from opengen.builder.problem import Problem
from opengen.config.build_config import BuildConfiguration
from opengen.config.solver_config import SolverConfiguration
from opengen.constraints.cartesian import CartesianProduct
from opengen.constraints.no_constraints import NoConstraints
from opengen.tcp.optimizer_tcp_manager import OptimizerTcpManager

from .problem import ShootingMethod
from .solution import OcpSolution


class GeneratedOptimizer:
    """Wrapper around generated OpEn optimizers using named parameters."""

    def __init__(self, ocp, optimizer_name, target_dir, backend, backend_kind):
        self.__ocp = ocp
        self.__optimizer_name = optimizer_name
        self.__target_dir = target_dir
        self.__backend = backend
        self.__backend_kind = backend_kind
        self.__started = False
        self.__symbolic_model = self.__ocp.build_symbolic_model()
        self.__rollout_function = self.__make_rollout_function()

    @property
    def target_dir(self):
        return self.__target_dir

    @property
    def backend_kind(self):
        return self.__backend_kind

    def start(self):
        if self.__backend_kind == "tcp" and not self.__started:
            self.__backend.start()
            self.__started = True
        return self

    def kill(self):
        if self.__backend_kind == "tcp" and self.__started:
            self.__backend.kill()
            self.__started = False

    def __make_rollout_function(self):
        if self.__ocp.shooting == ShootingMethod.MULTIPLE:
            return None
        states = cs.horzcat(*self.__symbolic_model["state_trajectory"])
        return cs.Function("ocp_rollout", [self.__symbolic_model["u"], self.__symbolic_model["p"]], [states])

    def __pack_parameters(self, solve_kwargs):
        return self.__ocp.parameters.pack(solve_kwargs)

    def __extract_inputs(self, flat_solution):
        if self.__ocp.shooting == ShootingMethod.SINGLE:
            nu = self.__ocp.nu
            return [
                flat_solution[stage_idx * nu:(stage_idx + 1) * nu]
                for stage_idx in range(self.__ocp.horizon)
            ]

        return [
            flat_solution[start:stop]
            for start, stop in self.__symbolic_model["input_slices"]
        ]

    def __extract_states(self, flat_solution, packed_parameters):
        if self.__ocp.shooting == ShootingMethod.MULTIPLE:
            x0_start, x0_stop = self.__ocp.parameters.slices()["x0"]
            x0 = packed_parameters[x0_start:x0_stop]
            states = [x0]
            states.extend(
                flat_solution[start:stop]
                for start, stop in self.__symbolic_model["state_slices"]
            )
            return states

        state_matrix = self.__rollout_function(flat_solution, packed_parameters).full()
        return [state_matrix[:, idx].reshape((-1,)).tolist() for idx in range(state_matrix.shape[1])]

    def solve(
        self,
        initial_guess=None,
        initial_lagrange_multipliers=None,
        initial_penalty=None,
        **parameter_values,
    ):
        packed_parameters = self.__pack_parameters(parameter_values)

        if self.__backend_kind == "direct":
            raw = self.__backend.run(
                p=packed_parameters,
                initial_guess=initial_guess,
                initial_lagrange_multipliers=initial_lagrange_multipliers,
                initial_penalty=initial_penalty,
            )
            if raw is None:
                raise RuntimeError("solver failed")
        elif self.__backend_kind == "tcp":
            self.start()
            response = self.__backend.call(
                packed_parameters,
                initial_guess=initial_guess,
                initial_y=initial_lagrange_multipliers,
                initial_penalty=initial_penalty,
            )
            if not response.is_ok():
                raise RuntimeError(str(response.get()))
            raw = response.get()
        else:
            raise RuntimeError("optimizer backend is not available")

        inputs = self.__extract_inputs(raw.solution)
        states = self.__extract_states(raw.solution, packed_parameters)
        return OcpSolution(raw=raw, inputs=inputs, states=states)


class OCPBuilder:
    """Builder that lowers an OCP to the existing OpEn Problem abstraction."""

    def __init__(
        self,
        problem,
        metadata,
        build_configuration=BuildConfiguration(),
        solver_configuration=None,
    ):
        self.__ocp = problem
        self.__metadata = metadata
        self.__build_configuration = build_configuration
        self.__solver_configuration = (
            solver_configuration
            if solver_configuration is not None
            else SolverConfiguration()
        )
        self.__generate_not_build = False

    def with_generate_not_build_flag(self, flag):
        self.__generate_not_build = flag
        return self

    def __make_input_constraints(self):
        stage_constraints = self.__ocp.input_constraints
        if stage_constraints is None:
            return NoConstraints()

        if self.__ocp.horizon == 1:
            return stage_constraints

        segments = [((idx + 1) * self.__ocp.nu) - 1 for idx in range(self.__ocp.horizon)]
        constraints = [stage_constraints] * self.__ocp.horizon
        return CartesianProduct(segments, constraints)

    def __make_multiple_shooting_constraints(self):
        stage_constraints = self.__ocp.input_constraints
        if stage_constraints is None:
            stage_constraints = NoConstraints()

        hard_stage_constraints = self.__ocp.hard_stage_state_input_constraints
        hard_terminal_constraints = self.__ocp.hard_terminal_state_constraints

        if hard_stage_constraints is not None and self.__ocp.input_constraints is not None:
            raise ValueError(
                "cannot combine with_input_constraints with "
                "with_hard_stage_state_input_constraints automatically; "
                "encode the input bounds directly in the stage state-input set"
            )

        if hard_stage_constraints is not None and hard_terminal_constraints is not None:
            raise ValueError(
                "cannot combine with_hard_stage_state_input_constraints with "
                "with_hard_terminal_state_constraints automatically; "
                "their intersection on the terminal stage is not supported"
            )

        if hard_stage_constraints is not None:
            segments = [
                ((idx + 1) * (self.__ocp.nu + self.__ocp.nx)) - 1
                for idx in range(self.__ocp.horizon)
            ]
            constraints = [hard_stage_constraints] * self.__ocp.horizon
            return CartesianProduct(segments, constraints)

        if hard_terminal_constraints is not None:
            segments = []
            constraints = []
            offset = -1

            for _ in range(self.__ocp.horizon - 1):
                offset += self.__ocp.nu
                segments.append(offset)
                constraints.append(stage_constraints)

                offset += self.__ocp.nx
                segments.append(offset)
                constraints.append(NoConstraints())

            offset += self.__ocp.nu
            segments.append(offset)
            constraints.append(stage_constraints)

            offset += self.__ocp.nx
            segments.append(offset)
            constraints.append(hard_terminal_constraints)

            return CartesianProduct(segments, constraints)

        segments = []
        constraints = []
        offset = -1

        for _ in range(self.__ocp.horizon):
            offset += self.__ocp.nu
            segments.append(offset)
            constraints.append(stage_constraints)

            offset += self.__ocp.nx
            segments.append(offset)
            constraints.append(NoConstraints())

        return CartesianProduct(segments, constraints)

    def build_problem(self):
        model = self.__ocp.build_symbolic_model()
        low_level_problem = Problem(model["u"], model["p"], model["cost"])

        if self.__ocp.shooting == ShootingMethod.SINGLE:
            constraints = self.__make_input_constraints()
        elif self.__ocp.shooting == ShootingMethod.MULTIPLE:
            constraints = self.__make_multiple_shooting_constraints()
        else:
            raise NotImplementedError("unsupported shooting method")

        low_level_problem = low_level_problem.with_constraints(constraints)

        if model["alm_mapping"] is not None:
            low_level_problem = low_level_problem.with_aug_lagrangian_constraints(
                model["alm_mapping"],
                model["alm_set_c"],
                model["alm_set_y"],
            )

        if model["penalty_mapping"] is not None:
            low_level_problem = low_level_problem.with_penalty_constraints(
                model["penalty_mapping"]
            )

        return low_level_problem

    def __make_backend(self, target_dir):
        optimizer_name = self.__metadata.optimizer_name

        if self.__build_configuration.build_python_bindings:
            if target_dir not in sys.path:
                sys.path.insert(0, target_dir)
            module = importlib.import_module(optimizer_name)
            return module.solver(), "direct"

        if self.__build_configuration.tcp_interface_config is not None:
            return OptimizerTcpManager(target_dir), "tcp"

        return None, "none"

    def build(self):
        low_level_problem = self.build_problem()
        builder = OpEnOptimizerBuilder(
            low_level_problem,
            metadata=self.__metadata,
            build_configuration=self.__build_configuration,
            solver_configuration=self.__solver_configuration,
        )
        if self.__generate_not_build:
            builder.with_generate_not_build_flag(True)
        info = builder.build()
        target_dir = os.path.abspath(info["paths"]["target"])
        backend, backend_kind = self.__make_backend(target_dir)
        return GeneratedOptimizer(
            ocp=self.__ocp,
            optimizer_name=self.__metadata.optimizer_name,
            target_dir=target_dir,
            backend=backend,
            backend_kind=backend_kind,
        )

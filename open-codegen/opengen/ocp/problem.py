from enum import Enum

import casadi.casadi as cs

from .parameter import ParameterPack


class ShootingMethod(Enum):
    SINGLE = "single"
    MULTIPLE = "multiple"


class OptimalControlProblem:
    """High-level optimal control specification."""

    def __init__(self, nx, nu, horizon, shooting=ShootingMethod.SINGLE):
        if not isinstance(nx, int) or nx <= 0:
            raise ValueError("nx must be a positive integer")
        if not isinstance(nu, int) or nu <= 0:
            raise ValueError("nu must be a positive integer")
        if not isinstance(horizon, int) or horizon <= 0:
            raise ValueError("horizon must be a positive integer")

        if isinstance(shooting, str):
            shooting = ShootingMethod(shooting)

        self.__nx = nx
        self.__nu = nu
        self.__horizon = horizon
        self.__shooting = shooting
        self.__symbol_type = cs.SX.sym
        self.__parameters = ParameterPack(symbol_type=self.__symbol_type)

        self.__dynamics = None
        self.__stage_cost = None
        self.__terminal_cost = None
        self.__input_constraints = None
        self.__path_constraints = []

    @property
    def nx(self):
        return self.__nx

    @property
    def nu(self):
        return self.__nu

    @property
    def horizon(self):
        return self.__horizon

    @property
    def shooting(self):
        return self.__shooting

    @property
    def symbol_type(self):
        return self.__symbol_type

    @property
    def parameters(self):
        return self.__parameters

    @property
    def dynamics(self):
        return self.__dynamics

    @property
    def input_constraints(self):
        return self.__input_constraints

    @property
    def path_constraints(self):
        return list(self.__path_constraints)

    def add_parameter(self, name, size, default=None):
        self.__parameters.add(name, size, default=default)
        return self

    def with_dynamics(self, dynamics):
        self.__dynamics = dynamics
        return self

    def with_stage_cost(self, stage_cost):
        self.__stage_cost = stage_cost
        return self

    def with_terminal_cost(self, terminal_cost):
        self.__terminal_cost = terminal_cost
        return self

    def with_input_constraints(self, constraints):
        self.__input_constraints = constraints
        return self

    def with_path_constraint(self, constraint, kind="penalty"):
        if kind != "penalty":
            raise NotImplementedError("only penalty path constraints are supported")
        self.__path_constraints.append((kind, constraint))
        return self

    def validate(self):
        if self.__dynamics is None:
            raise ValueError("dynamics must be specified")
        if self.__stage_cost is None:
            raise ValueError("stage cost must be specified")
        return self

    def __build_single_shooting_model(self):
        u = self.__symbol_type("u", self.__nu * self.__horizon)
        p = self.__parameters.symbol()
        param = self.__parameters.view(p)

        x0 = param.get("x0")
        if x0 is None:
            raise ValueError("parameter 'x0' must be declared for OCP problems")
        if x0.size1() * x0.size2() != self.__nx:
            raise ValueError("parameter 'x0' has incompatible dimension")

        cost = 0
        x = x0
        states = [x0]
        penalty_terms = []

        for stage_idx in range(self.__horizon):
            start = stage_idx * self.__nu
            stop = start + self.__nu
            u_t = u[start:stop]
            cost += self.__stage_cost(x, u_t, param, stage_idx)

            for kind, path_constraint in self.__path_constraints:
                if kind == "penalty":
                    penalty_terms.append(path_constraint(x, u_t, param, stage_idx))

            x = self.__dynamics(x, u_t, param)
            states.append(x)

        if self.__terminal_cost is not None:
            cost += self.__terminal_cost(x, param)

        penalty_mapping = None
        if penalty_terms:
            penalty_mapping = cs.vertcat(*penalty_terms)

        return {
            "shooting": self.__shooting.value,
            "u": u,
            "p": p,
            "param": param,
            "cost": cost,
            "penalty_mapping": penalty_mapping,
            "state_trajectory": states,
        }

    def __build_multiple_shooting_model(self):
        p = self.__parameters.symbol()
        param = self.__parameters.view(p)

        x0 = param.get("x0")
        if x0 is None:
            raise ValueError("parameter 'x0' must be declared for OCP problems")
        if x0.size1() * x0.size2() != self.__nx:
            raise ValueError("parameter 'x0' has incompatible dimension")

        decision_blocks = []
        input_slices = []
        state_slices = []
        states = [x0]
        cost = 0
        penalty_terms = []

        x_current = x0
        offset = 0

        for stage_idx in range(self.__horizon):
            u_t = self.__symbol_type(f"u_{stage_idx}", self.__nu)
            x_next = self.__symbol_type(f"x_{stage_idx + 1}", self.__nx)

            decision_blocks.extend([u_t, x_next])
            input_slices.append((offset, offset + self.__nu))
            offset += self.__nu
            state_slices.append((offset, offset + self.__nx))
            offset += self.__nx

            cost += self.__stage_cost(x_current, u_t, param, stage_idx)

            for kind, path_constraint in self.__path_constraints:
                if kind == "penalty":
                    penalty_terms.append(path_constraint(x_current, u_t, param, stage_idx))

            penalty_terms.append(x_next - self.__dynamics(x_current, u_t, param))

            x_current = x_next
            states.append(x_current)

        if self.__terminal_cost is not None:
            cost += self.__terminal_cost(x_current, param)

        u = cs.vertcat(*decision_blocks)
        penalty_mapping = cs.vertcat(*penalty_terms) if penalty_terms else None

        return {
            "shooting": self.__shooting.value,
            "u": u,
            "p": p,
            "param": param,
            "cost": cost,
            "penalty_mapping": penalty_mapping,
            "state_trajectory": states,
            "input_slices": input_slices,
            "state_slices": state_slices,
        }

    def build_symbolic_model(self):
        self.validate()
        if self.__shooting == ShootingMethod.SINGLE:
            return self.__build_single_shooting_model()
        if self.__shooting == ShootingMethod.MULTIPLE:
            return self.__build_multiple_shooting_model()
        raise NotImplementedError("unsupported shooting method")

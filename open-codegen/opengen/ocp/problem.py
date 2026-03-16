from enum import Enum

import casadi.casadi as cs

from opengen.constraints.cartesian import CartesianProduct
from opengen.constraints.zero import Zero

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
        self.__dynamics_constraint_kind = "penalty"
        self.__stage_cost = None
        self.__terminal_cost = None
        self.__input_constraints = None
        self.__hard_stage_state_input_constraints = None
        self.__hard_terminal_state_constraints = None
        self.__path_constraints = []
        self.__terminal_constraints = []

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
    def hard_stage_state_input_constraints(self):
        return self.__hard_stage_state_input_constraints

    @property
    def hard_terminal_state_constraints(self):
        return self.__hard_terminal_state_constraints

    @property
    def path_constraints(self):
        return list(self.__path_constraints)

    @property
    def terminal_constraints(self):
        return list(self.__terminal_constraints)

    def add_parameter(self, name, size, default=None):
        self.__parameters.add(name, size, default=default)
        return self

    def with_dynamics(self, dynamics):
        self.__dynamics = dynamics
        return self

    def with_dynamics_constraints(self, kind="penalty"):
        if kind not in ("penalty", "alm"):
            raise ValueError("dynamics constraint kind must be either 'penalty' or 'alm'")
        self.__dynamics_constraint_kind = kind
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

    def with_hard_stage_state_input_constraints(self, constraints):
        self.__hard_stage_state_input_constraints = constraints
        return self

    def with_hard_terminal_state_constraints(self, constraints):
        self.__hard_terminal_state_constraints = constraints
        return self

    @staticmethod
    def __constraint_dimension(mapping):
        return mapping.size1() * mapping.size2()

    @staticmethod
    def __append_constraint_definition(container, kind, constraint, set_c=None, set_y=None):
        if kind not in ("penalty", "alm"):
            raise ValueError("constraint kind must be either 'penalty' or 'alm'")
        if kind == "alm" and set_c is None:
            raise ValueError("set_c must be provided for ALM constraints")
        container.append({
            "kind": kind,
            "constraint": constraint,
            "set_c": set_c,
            "set_y": set_y,
        })

    @staticmethod
    def __assemble_constraint_set(blocks, key):
        if not blocks:
            return None
        if len(blocks) == 1:
            return blocks[0][key]

        segments = []
        constraints = []
        offset = -1
        for block in blocks:
            offset += block["dimension"]
            segments.append(offset)
            constraints.append(block[key])
        return CartesianProduct(segments, constraints)

    def with_path_constraint(self, constraint, kind="penalty", set_c=None, set_y=None):
        self.__append_constraint_definition(
            self.__path_constraints,
            kind,
            constraint,
            set_c=set_c,
            set_y=set_y,
        )
        return self

    def with_terminal_constraint(self, constraint, kind="penalty", set_c=None, set_y=None):
        self.__append_constraint_definition(
            self.__terminal_constraints,
            kind,
            constraint,
            set_c=set_c,
            set_y=set_y,
        )
        return self

    def validate(self):
        if self.__dynamics is None:
            raise ValueError("dynamics must be specified")
        if self.__stage_cost is None:
            raise ValueError("stage cost must be specified")
        if self.__shooting == ShootingMethod.SINGLE:
            if self.__hard_stage_state_input_constraints is not None:
                raise ValueError(
                    "with_hard_stage_state_input_constraints is only available in multiple shooting"
                )
            if self.__hard_terminal_state_constraints is not None:
                raise ValueError(
                    "with_hard_terminal_state_constraints is only available in multiple shooting"
                )
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
        alm_blocks = []

        for stage_idx in range(self.__horizon):
            start = stage_idx * self.__nu
            stop = start + self.__nu
            u_t = u[start:stop]
            cost += self.__stage_cost(x, u_t, param, stage_idx)

            for definition in self.__path_constraints:
                mapping = definition["constraint"](x, u_t, param, stage_idx)
                if definition["kind"] == "penalty":
                    penalty_terms.append(mapping)
                else:
                    alm_blocks.append({
                        "mapping": mapping,
                        "dimension": self.__constraint_dimension(mapping),
                        "set_c": definition["set_c"],
                        "set_y": definition["set_y"],
                    })

            x = self.__dynamics(x, u_t, param)
            states.append(x)

        if self.__terminal_cost is not None:
            cost += self.__terminal_cost(x, param)

        for definition in self.__terminal_constraints:
            mapping = definition["constraint"](x, param)
            if definition["kind"] == "penalty":
                penalty_terms.append(mapping)
            else:
                alm_blocks.append({
                    "mapping": mapping,
                    "dimension": self.__constraint_dimension(mapping),
                    "set_c": definition["set_c"],
                    "set_y": definition["set_y"],
                })

        penalty_mapping = None
        if penalty_terms:
            penalty_mapping = cs.vertcat(*penalty_terms)

        alm_mapping = None
        alm_set_c = None
        alm_set_y = None
        if alm_blocks:
            alm_mapping = cs.vertcat(*[block["mapping"] for block in alm_blocks])
            alm_set_c = self.__assemble_constraint_set(alm_blocks, "set_c")
            if all(block["set_y"] is not None for block in alm_blocks):
                alm_set_y = self.__assemble_constraint_set(alm_blocks, "set_y")

        return {
            "shooting": self.__shooting.value,
            "u": u,
            "p": p,
            "param": param,
            "cost": cost,
            "alm_mapping": alm_mapping,
            "alm_set_c": alm_set_c,
            "alm_set_y": alm_set_y,
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
        alm_blocks = []
        dynamics_alm_terms = []

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

            for definition in self.__path_constraints:
                mapping = definition["constraint"](x_current, u_t, param, stage_idx)
                if definition["kind"] == "penalty":
                    penalty_terms.append(mapping)
                else:
                    alm_blocks.append({
                        "mapping": mapping,
                        "dimension": self.__constraint_dimension(mapping),
                        "set_c": definition["set_c"],
                        "set_y": definition["set_y"],
                    })

            dynamics_defect = x_next - self.__dynamics(x_current, u_t, param)
            if self.__dynamics_constraint_kind == "penalty":
                penalty_terms.append(dynamics_defect)
            else:
                dynamics_alm_terms.append(dynamics_defect)

            x_current = x_next
            states.append(x_current)

        if self.__terminal_cost is not None:
            cost += self.__terminal_cost(x_current, param)

        if dynamics_alm_terms:
            dynamics_mapping = cs.vertcat(*dynamics_alm_terms)
            alm_blocks.append({
                "mapping": dynamics_mapping,
                "dimension": self.__constraint_dimension(dynamics_mapping),
                "set_c": Zero(),
                "set_y": None,
            })

        for definition in self.__terminal_constraints:
            mapping = definition["constraint"](x_current, param)
            if definition["kind"] == "penalty":
                penalty_terms.append(mapping)
            else:
                alm_blocks.append({
                    "mapping": mapping,
                    "dimension": self.__constraint_dimension(mapping),
                    "set_c": definition["set_c"],
                    "set_y": definition["set_y"],
                })

        u = cs.vertcat(*decision_blocks)
        penalty_mapping = cs.vertcat(*penalty_terms) if penalty_terms else None
        alm_mapping = None
        alm_set_c = None
        alm_set_y = None
        if alm_blocks:
            alm_mapping = cs.vertcat(*[block["mapping"] for block in alm_blocks])
            alm_set_c = self.__assemble_constraint_set(alm_blocks, "set_c")
            if all(block["set_y"] is not None for block in alm_blocks):
                alm_set_y = self.__assemble_constraint_set(alm_blocks, "set_y")

        return {
            "shooting": self.__shooting.value,
            "u": u,
            "p": p,
            "param": param,
            "cost": cost,
            "alm_mapping": alm_mapping,
            "alm_set_c": alm_set_c,
            "alm_set_y": alm_set_y,
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

import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
from enum import Enum
from opengen.constraints import Constraint
from .optimizer_formulations import FormulationType, single_shooting_formulation, multiple_shooting_formulation
from .ocp_build_interface import OcpInterfaceType


class ConstraintMethod(Enum):
    ALM = 1
    PM = 2


class OptimalControlProblem:
    def __init__(self, nx, nu, sys_dyn_fn, stage_cost_fn, terminal_cost_fn):
        self.__nx = nx
        self.__nu = nu
        self.__horizon = 15
        self.__x_set = og.constraints.NoConstraints()
        self.__u_set = og.constraints.NoConstraints()
        self.__formulation_type = FormulationType.MULTIPLE_SHOOTING
        self.__ocp_build_interface = OcpInterfaceType.TCP
        self.__exclusion_set = None
        self.__problem = None
        self.sys_dyn_fn = sys_dyn_fn
        self.stage_cost_fn = stage_cost_fn
        self.terminal_cost_fn = terminal_cost_fn
        self.problem_formulation = multiple_shooting_formulation

    def with_horizon(self, horizon):
        self.__horizon = horizon
        return self

    def with_state_constraint(self, x_set):
        self.__x_set = x_set
        return self

    def with_input_constraint(self, u_set):
        self.__u_set = u_set
        return self

    def with_formulation_type(self, formulation):
        if formulation is FormulationType.MULTIPLE_SHOOTING:
            self.problem_formulation = multiple_shooting_formulation
        elif formulation is FormulationType.SINGLE_SHOOTING:
            self.problem_formulation = single_shooting_formulation
        else:
            raise Exception("Formulation type not Supported")
        self.__formulation_type = formulation
        return self

    def with_exclusion_set(self, exclusion_set):
        self.__exclusion_set = exclusion_set
        return self

    def with_build_interface(self, ocp_build_interface):
        if ocp_build_interface in (OcpInterfaceType.TCP, OcpInterfaceType.DIRECT):
            self.__ocp_build_interface = ocp_build_interface
        else:
            raise Exception("Build Interface type not Supported")
        return self

    def save_OCP(self, problem):
        if isinstance(problem, og.builder.Problem):
            self.__problem = problem
        else:
            raise Exception("Problem must belong to class opengen.builder.Problem")

    def get_nx(self):
        return self.__nx

    def get_nu(self):
        return self.__nu

    def get_horizon(self):
        return self.__horizon

    def get_state_constraint(self):
        return self.__x_set

    def get_input_constraint(self):
        return self.__u_set

    def get_formulation_type(self):
        return self.__formulation_type

    def get_exclusion_set(self):
        return self.__exclusion_set

    def get_ocp_build_interface(self):
        return self.__ocp_build_interface

    def get_OCP(self):
        return self.__problem


class BallExclusionSet(Constraint):
    def __init__(self, set_type, center, radius, state_indices=None, method=ConstraintMethod.PM):

        supported_constraints_list = (og.constraints.Ball1, og.constraints.Ball2, og.constraints.BallInf)
        if not isinstance(set_type, supported_constraints_list):
            raise Exception("Constraint Not Supported")
        self.__set_type = set_type

        self.__center = center
        self.__radius = radius

        if state_indices is None:
            state_indices = [i for i in range(len(center))]
        else:
            if state_indices[0] < 0:
                raise ValueError("the first element of segment must be a positive integer")
            if any([state_indices[i] >= state_indices[i + 1] for i in range(len(state_indices) - 1)]):
                raise ValueError("segments should be a list of integers in strictly ascending order")

        self.__state_indices = state_indices

        constraint_method_list = (ConstraintMethod.PM, ConstraintMethod.ALM)
        if not method in constraint_method_list:
            raise Exception("Constraint Method not Supported")
        self.__method = method

    def append_exclusion_set(self, set_exclusion_fn, x, nx, N):
        radius = self.__radius
        centre = self.__center
        if isinstance(self.__set_type, og.constraints.Ball2):
            for iteration in range(N):
                x_current = x[iteration*nx:(iteration+1)*nx]
                x_trunc = [x_current[i] for i in self.__state_indices]
                val = radius**2
                for index in range(len(x_trunc)):
                    val -= (x_trunc[index] - centre[index])**2
                set_exclusion_fn = cs.vertcat(set_exclusion_fn, fn.fmax(0.0, val))

        return set_exclusion_fn

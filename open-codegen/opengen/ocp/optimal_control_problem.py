import opengen as og
from .type_enums import *


class OptimalControlProblem:
    def __init__(self, p_symb, nx, nu, sys_dyn_fn, stage_cost_fn, terminal_cost_fn):
        self.__p_symb = p_symb
        self.__nx = nx
        self.__nu = nu
        self.__horizon = 15
        self.__x_set = og.constraints.NoConstraints()
        self.__u_set = og.constraints.NoConstraints()
        self.__formulation_type = FormulationType.MULTIPLE_SHOOTING
        self.__exclusion_set = None
        self.__problem = None
        self.sys_dyn_fn = sys_dyn_fn
        self.stage_cost_fn = stage_cost_fn
        self.terminal_cost_fn = terminal_cost_fn

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
        self.__formulation_type = formulation
        return self

    def with_exclusion_set(self, exclusion_set):
        self.__exclusion_set = exclusion_set
        return self

    def save_OCP(self, problem):
        if isinstance(problem, og.builder.Problem):
            self.__problem = problem
        else:
            raise Exception("Problem must belong to class opengen.builder.Problem")

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
    def p_symb(self):
        return self.__p_symb

    @property
    def x_set(self):
        return self.__x_set

    @property
    def u_set(self):
        return self.__u_set

    @property
    def formulation_type(self):
        return self.__formulation_type

    @property
    def exclusion_set(self):
        return self.__exclusion_set

    @property
    def problem(self):
        return self.__problem
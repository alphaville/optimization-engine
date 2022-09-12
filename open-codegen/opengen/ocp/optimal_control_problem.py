import opengen as og
from .set_exclusion import ExclusionSet
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
        if isinstance(x_set, list):
            # case with N constraints with each of dimension: nx
            if not len(x_set) == self.__horizon:
                raise Exception("Length of list must be equal to the Horizon")
            dim = x_set[0].dimension()
        elif x_set.dimension() == self.__nx * self.__horizon:
            # case with single constraint with dimension: nx*N
            dim = x_set.dimension()/self.__horizon
        else:
            # case with single constraint with dimension: nx
            dim = x_set.dimension()

        if (dim == self.__nx):
            self.__x_set = x_set
        else:
            raise Exception("Size mismatch for state bounds")   # Check size of x_set and how handeled in single shooting
        return self

    def with_input_constraint(self, u_set):
        if isinstance(u_set, list):
            # case with N constraints with each of dimension: nu
            if not len(u_set) == self.__horizon:
                raise Exception("Length of list must be equal to the Horizon")
            dim = u_set[0].dimension()
        elif u_set.dimension() == self.__nu * self.__horizon:
            # case with single constraint with dimension: nu*N
            dim = u_set.dimension() / self.__horizon
        else:
            # case with single constraint with dimension: nu
            dim = u_set.dimension()

        if (dim == self.__nu):
            self.__u_set = u_set
        else:
            raise Exception("Size mismatch for input bounds")
        return self

    def with_formulation_type(self, formulation):
        if not isinstance(formulation, FormulationType):
            raise Exception("Formulation type must be instance of FormulationType")
        self.__formulation_type = formulation
        return self

    def with_exclusion_set(self, exclusion_set):
        if not all(isinstance(x, ExclusionSet) for x in exclusion_set):
            raise Exception("Elements of exclusion set must be of class constraint")
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
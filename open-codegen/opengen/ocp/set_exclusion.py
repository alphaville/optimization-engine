from constraints import Constraint
from .type_enums import ConstraintMethod
import casadi.casadi as cs


class ExclusionSet(Constraint):
    def __init__(self, constraint, state_idx=None, mode=ConstraintMethod.PM):
        self.__constraint = constraint
        self.__state_idx = state_idx
        self.__mode = mode

    def get_exclusion_mode(self):
        return self.__mode

    def exclude_set(self, x, nx, horizon):
        exclude_set = []

        if self.__state_idx is None:
            self.__state_idx = [i for i in range(nx)]
        else:
            if self.__state_idx[0] < 0:
                raise ValueError("the first element of segment must be a positive integer")
            if any([self.__state_idx[i] >= self.__state_idx[i + 1] for i in range(len(self.__state_idx) - 1)]):
                raise ValueError("segments should be a list of integers in strictly ascending order")

        for iteration in range(horizon):
            x_current = x[iteration * nx:(iteration + 1) * nx]
            x_trunc = []
            for i in self.__state_idx:
                x_trunc = cs.vertcat(x_trunc, x_current[i])
            exclude_set = cs.vertcat(exclude_set, self.__constraint.sup_level_set(x_trunc))

        return exclude_set




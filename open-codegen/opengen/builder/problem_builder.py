import casadi.casadi as cs
from enum import Enum
import opengen as og

class ProblemBuilder:

    class Op(Enum):
        LT = 1
        IN = 2

    def __init__(self):
        self.__decision_variables = set()
        self.__parameter_variables = set()
        self.__cost_function = None
        self.__aug_lagrangian_constraints = None
        self.__penalty_constraints = None

    def add_decision_variable(self, *decision_vars):
        set_of_decision_vars = set(decision_vars)
        # If the list of decision variables is not empty, make sure the new
        # arrivals have a compatible type
        if len(self.__decision_variables) > 0:
            existing_type = next(iter(self.__decision_variables)).symbol_type
            if not all([x.symbol_type == existing_type for x in set_of_decision_vars]):
                raise Exception("Incompatible type")
        if not all([isinstance(var, og.sym.Symbol) for var in decision_vars]):
            raise Exception("Incompatible type: only SX and MX are allowed")
        self.__decision_variables = self.__decision_variables.union(set_of_decision_vars)

    def add_parameter_variable(self, *var):
        pass

    def set_cost_function(self, cost):
        self.__cost_function = cost

    @property
    def decision_variables(self):
        return self.__decision_variables

    @property
    def parameter_variables(self):
        return self.__parameter_variables

    @property
    def aug_lagrangian_constraints(self):
        return self.__aug_lagrangian_constraints


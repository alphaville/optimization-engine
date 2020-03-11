import casadi.casadi as cs
from enum import Enum

class ProblemBuilder:

    class Op(Enum):
        LT = 1
        IN = 2

    def __init__(self):
        self.__decision_variables = set()
        self.__parameter_variables = []
        self.__cost_function = None
        self.__aug_lagrangian_constraints = None
        self.__penalty_constraints = None

    def add_decision_variable(self, *var):
        set_decision_vars = set(var)
        # If the list of decision variables is not empty, make sure the new
        # arrivals have a compatible type
        if len(self.__decision_variables) > 0:
            existing_type = type(next(iter(self.__decision_variables)))
            if not all(isinstance(x, existing_type) for x in set_decision_vars):
                raise Exception("Incompatible type")
        else:
            if not all(isinstance(x, (cs.SX, cs.MX)) for x in set_decision_vars):
                raise Exception("Incompatible type: only SX and MX are allowed")
        self.__decision_variables = self.__decision_variables.union(set_decision_vars)

    def add_parameter_variable(self, *var):
        list_parameter_vars = list(var)
        # If the list of decision variables is not empty, make sure the new
        # arrivals have a compatible type
        if len(self.__parameter_variables) > 0:
            existing_type = type(self.__parameter_variables[0])
            if not all(isinstance(x, existing_type) for x in list_parameter_vars):
                raise Exception("Incompatible type")
        else:
            if not all(isinstance(x, (cs.SX, cs.MX)) for x in list_parameter_vars):
                raise Exception("Incompatible type: only SX and MX are allowed")
        self.__parameter_variables += list_parameter_vars

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

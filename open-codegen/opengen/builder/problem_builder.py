from enum import Enum
import opengen as og
import casadi.casadi as cs


class ProblemBuilder:

    def __init__(self):
        self.__decision_variables = set()
        self.__parameter_variables = set()
        self.__cost_function = None
        self.__aug_lagrangian_constraints = set()
        self.__penalty_constraints = set()
        self.__constraints = {}

    def add_decision_variable(self, *decision_vars):
        # The following prevents the user from using both SX
        # and MX-type variables
        for decision_variable in decision_vars:
            self.__decision_variables.update(cs.SX.elements(decision_variable))

    def add_parameter_variable(self, *parameter_vars):
        # The following prevents the user from using both SX
        # and MX-type variables
        for parameter_variable in parameter_vars:
            self.__parameter_variables.update(cs.SX.elements(parameter_variable))

    def set_cost_function(self, cost):
        self.__cost_function = cost

    def add_constraint(self, var, constraint_set):
        self.__constraints[constraint_set] = var

    def add_aug_lagrangian_constraint(self, symbolic_function, set_c):

        pass

    @property
    def decision_variables(self):
        return self.__decision_variables

    @property
    def parameter_variables(self):
        return self.__parameter_variables

    @property
    def aug_lagrangian_constraints(self):
        return self.__aug_lagrangian_constraints

    @property
    def constraints(self):
        return self.__constraints

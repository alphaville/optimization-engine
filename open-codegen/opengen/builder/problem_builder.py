import casadi.casadi as cs
import collections
from .problem import Problem


class OrderedSet(collections.MutableSet):
    # OrderedSet implementation
    # Credit: http://code.activestate.com/recipes/576694/

    def __init__(self, iterable=None):
        self.end = end = []
        end += [None, end, end]         # sentinel node for doubly linked list
        self.map = {}                   # key --> [key, prev, next]
        if iterable is not None:
            self |= iterable

    def __len__(self):
        return len(self.map)

    def __contains__(self, key):
        return key in self.map

    def add(self, key):
        if key not in self.map:
            end = self.end
            curr = end[1]
            curr[2] = end[1] = self.map[key] = [key, curr, end]

    def discard(self, key):
        if key in self.map:
            key, prev, next = self.map.pop(key)
            prev[2] = next
            next[1] = prev

    def __iter__(self):
        end = self.end
        curr = end[2]
        while curr is not end:
            yield curr[0]
            curr = curr[2]

    def __reversed__(self):
        end = self.end
        curr = end[1]
        while curr is not end:
            yield curr[0]
            curr = curr[1]

    def pop(self, last=True):
        if not self:
            raise KeyError('set is empty')
        key = self.end[1][0] if last else self.end[2][0]
        self.discard(key)
        return key

    def __repr__(self):
        if not self:
            return '%s()' % (self.__class__.__name__,)
        return '%s(%r)' % (self.__class__.__name__, list(self))

    def __eq__(self, other):
        if isinstance(other, OrderedSet):
            return len(self) == len(other) and list(self) == list(other)
        return set(self) == set(other)


class ProblemBuilder:

    def __init__(self):
        """
        Construct new instance of ProblemBuilder
        """
        # Ordered set of decision variables
        self.__decision_variables = OrderedSet()
        # Ordered set of parameter variables
        self.__parameter_variables = OrderedSet()
        # Cost function
        self.__cost_function = None
        # Dictionary of augmented Lagrangian constraitns
        self.__aug_lagrangian_constraints = {}
        # Penalty constraints (symbolic functions)
        self.__penalty_constraints = None
        # Simple (projectable) constraints
        self.__constraints = {}

    def add_decision_variable(self, *decision_vars):
        """
        Add decision variable(s)
        :param decision_vars:
        :return:
        """
        # The following prevents the user from using both SX
        # and MX-type variables
        for decision_variable in decision_vars:
            if isinstance(decision_variable, cs.SX):
                for dv in cs.SX.elements(decision_variable):
                    if not (dv.is_symbolic() and dv.is_valid_input()):
                        raise ValueError("Decision variable is not a primitive variable!")
                    self.__decision_variables.add(dv)
            elif isinstance(decision_variable, cs.MX):
                self.__decision_variables.add(decision_variable)
            else:
                raise ValueError("Only SX/MX decision variables are allowed")

    def add_parameter_variable(self, *parameter_vars):
        """
        Add parameter variable(s)
        :param parameter_vars:
        :return:
        """
        # The following prevents the user from using both SX
        # and MX-type variables
        for parameter_variable in parameter_vars:
            if isinstance(parameter_variable, cs.SX):
                for pv in cs.SX.elements(parameter_variable):
                    if not (pv.is_symbolic() and pv.is_valid_input()):
                        raise ValueError("Parameter variable is not a primitive variable!")
                    self.__parameter_variables.add(pv)
            elif isinstance(parameter_variable, cs.MX):
                self.__parameter_variables.add(parameter_variable)
            else:
                raise ValueError("Only SX/MX parameter variables are allowed")

    def set_cost_function(self, cost):
        """
        Define the cost function
        :param cost:
        :return:
        """
        if not isinstance(cost, (cs.SX, cs.MX)):
            raise ValueError("cost must be of type SX or MX")
        if cost.shape != (1, 1):
            raise ValueError("cost function must be scalar-valued")
        self.__cost_function = cost

    def add_constraint(self, var, constraint_set):
        """
        Add a simple projectable constraint on some of the decision
        variables
        :param var:
        :param constraint_set:
        :return:
        """
        if not (var.is_symbolic() and var.is_valid_input()):
            raise ValueError("Only primitive variables can be constrained using this method")
        self.__constraints[constraint_set] = var

    def add_aug_lagrangian_constraint(self, symbolic_function, set_c):
        pass

    def add_penalty_constraints(self, penalty_function):
        """
        Add a penalty constraint
        :param penalty_function:
        :return:
        """
        if not isinstance(penalty_function, (cs.SX, cs.MX)):
            raise ValueError("penalty_function must be of type SX or MX")
        if penalty_function.shape[1] != 1:
            raise ValueError("penalty_function must be scalar- or vector-valued")
        if self.__penalty_constraints is None:
            self.__penalty_constraints = penalty_function
        else:
            self.__penalty_constraints = \
                cs.vertcat(self.__penalty_constraints, penalty_function)

    @staticmethod
    def __set_to_symbol(set_symbols):
        us = []
        for ui in set_symbols:
            us = cs.vertcat(us, ui)
        return us
    pass

    def __check_before_build(self):
        pass

    def __construct_decision_variable(self):
        return ProblemBuilder.__set_to_symbol(self.__decision_variables)

    def __construct_parameter_variable(self):
        return ProblemBuilder.__set_to_symbol(self.__parameter_variables)

    def __construct_constraints(self):
        pass

    def build(self):
        self.__check_before_build()
        u = self.__construct_decision_variable()
        p = self.__construct_parameter_variable()
        cost = self.__cost_function
        problem = Problem(u, p, cost)
        return problem

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

    @property
    def penalty_constraints(self):
        return self.__penalty_constraints
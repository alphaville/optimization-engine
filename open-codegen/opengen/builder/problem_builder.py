import casadi.casadi as cs
import collections


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
        # The following prevents the user from using both SX
        # and MX-type variables
        for decision_variable in decision_vars:
            for dv in cs.SX.elements(decision_variable):
                self.__decision_variables.add(dv)

    def add_parameter_variable(self, *parameter_vars):
        # The following prevents the user from using both SX
        # and MX-type variables
        for parameter_variable in parameter_vars:
            for pv in cs.SX.elements(parameter_variable):
                self.__parameter_variables.add(pv)

    def set_cost_function(self, cost):
        self.__cost_function = cost

    def add_constraint(self, var, constraint_set):
        self.__constraints[constraint_set] = var

    def add_aug_lagrangian_constraint(self, symbolic_function, set_c):
        pass

    def add_penalty_constraints(self, penalty_function):
        if penalty_function.shape[1] != 1:
            raise Exception("penalty function must be real-valued")
        if self.__penalty_constraints is None:
            self.__penalty_constraints = penalty_function
        else:
            self.__penalty_constraints = \
                cs.vertcat(self.__penalty_constraints, penalty_function)

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

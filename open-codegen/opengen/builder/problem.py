class Problem:

    def __init__(self, u, p, cost):
        self._u = u
        self._p = p
        self._cost = cost
        self._u_constraints = None
        self._penalty_constraints = None
        self._al_constraints = None

    # ---------- SETTERS -----------------------------------------------

    def with_constraints(self, u_constraints):
        self._u_constraints = u_constraints
        return self

    def with_penalty_constraints(self, penalty_constraints):
        self._penalty_constraints = penalty_constraints
        return self

    def with_aug_lagrangian_constraints(self, al_constraints):
        self._al_constraints = al_constraints
        return self

    # ---------- DIMENSIONS --------------------------------------------

    def dim_decision_variables(self):
        return self._u.size(1)

    def dim_parameters(self):
        return self._p.size(1)

    def dim_constraints_penalty(self):
        return 0 if self._penalty_constraints is None \
            else self._penalty_constraints.size(1)

    def dim_constraints_at(self):
        return 0 if self._al_constraints is None \
            else self._al_constraints.size(1)

    # ---------- GETTERS -----------------------------------------------

    def decision_variables(self):
        return self._u

    def parameter_variables(self):
        return self._p

    def cost_function(self):
        return self._cost

    def penalty_constraints(self):
        return self._penalty_constraints

class Problem:

    def __init__(self, u, p, cost):
        self.u = u
        self.p = p
        self.cost = cost
        self.u_constraints = None
        self.penalty_constraints = None
        self.al_constraints = None

    def with_constraints(self, u_constraints):
        self.u_constraints = u_constraints
        return self

    def with_penalty_constraints(self, penalty_constraints):
        self.penalty_constraints = penalty_constraints
        return self

    def with_aug_lagrangian_constraints(self, al_constraints):
        self.al_constraints = al_constraints
        return self

    def dim_decision_variables(self):
        return self.u.size(1)

    def dim_parameters(self):
        return self.p.size(1)

    def dim_constraints_penalty(self):
        return 0 if self.penalty_constraints is None else self.penalty_constraints.size(1)

    def dim_constraints_at(self):
        return 0 if self.al_constraints is None else self.al_constraints.size(1)


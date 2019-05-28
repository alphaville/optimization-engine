class SolverConfiguration:

    def __init__(self):
        self._tolerance = 1e-4
        self._lbfgs_memory = 10
        self._max_iterations = 500
        self._max_time_millis = 20
        self._max_outer_iterations = 10
        self._constraints_tolerance = 1e-4
        self._penalty_weight_update_factor = 5.0

    # --------- GETTERS -----------------------------
    def tolerance(self):
        return self._tolerance

    def lbfgs_memory(self):
        return self._lbfgs_memory

    def max_iterations(self):
        return self._max_iterations

    def max_time_millis(self):
        return self._max_time_millis

    def constraints_tolerance(self):
        return self._constraints_tolerance

    def max_outer_iterations(self):
        return self._max_outer_iterations

    def penalty_weight_update_factor(self):
        return self._penalty_weight_update_factor

    # --------- SETTERS -----------------------------
    def with_tolerance(self, tolerance):
        self._tolerance = tolerance
        return self

    def with_lfbgs_memory(self, lbfgs_memory):
        self._lbfgs_memory = lbfgs_memory
        return self

    def with_max_iterations(self, max_iters):
        self._max_iterations = max_iters
        return self

    def with_constraints_tolerance(self, constraints_tolerance):
        self._constraints_tolerance = constraints_tolerance
        return self

    def with_max_outer_iterations(self, max_outer_iterations):
        self._max_outer_iterations = max_outer_iterations
        return self

    def with_penalty_weight_update_factor(self, penalty_weight_update_factor):
        self._penalty_weight_update_factor = penalty_weight_update_factor
        return self

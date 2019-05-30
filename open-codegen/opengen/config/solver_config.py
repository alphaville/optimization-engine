class SolverConfiguration:
    """Configuration of solver parameters

    """

    def __init__(self):
        """Construct an instance of solver configuration parameters

        Args:
            None

        Returns:
            New instance of SolverConfiguration

        """
        self._tolerance = 1e-4
        self._lbfgs_memory = 10
        self._max_inner_iterations = 500
        self._max_time_millis = 20
        self._max_outer_iterations = 10
        self._constraints_tolerance = 1e-4
        self._penalty_weight_update_factor = 5.0
        self._initial_weights = 10.0
        self._max_duration_micros = 5000000

    # --------- GETTERS -----------------------------
    @property
    def tolerance(self):
        return self._tolerance

    @property
    def lbfgs_memory(self):
        return self._lbfgs_memory

    @property
    def max_inner_iterations(self):
        return self._max_inner_iterations

    @property
    def max_time_millis(self):
        return self._max_time_millis

    @property
    def constraints_tolerance(self):
        return self._constraints_tolerance

    @property
    def max_outer_iterations(self):
        return self._max_outer_iterations

    @property
    def penalty_weight_update_factor(self):
        return self._penalty_weight_update_factor

    @property
    def initial_penalty_weights(self):
        if isinstance(self._initial_weights, list):
            return self._initial_weights
        else:
            return [self._initial_weights]

    @property
    def max_duration_micros(self):
        return self._max_duration_micros

    # --------- SETTERS -----------------------------
    def with_tolerance(self, tolerance):
        """Specify tolerance"""
        assert tolerance > 0
        self._tolerance = tolerance
        return self

    def with_lfbgs_memory(self, lbfgs_memory):
        """Specify L-BFGS memory

        Assertions:
            It is required that the L-BFGS memory is larger than or
            equal to 2

        Returns:
            Returns the current instance of SolverConfiguration
        """
        assert lbfgs_memory >= 2
        self._lbfgs_memory = lbfgs_memory
        return self

    def with_max_inner_iterations(self, max_iters):
        """Maximum number of inner iterations"""
        assert max_iters > 1
        self._max_inner_iterations = max_iters
        return self

    def with_constraints_tolerance(self, constraints_tolerance):
        """Tolerance on constraint violation"""
        assert constraints_tolerance > 0
        self._constraints_tolerance = constraints_tolerance
        return self

    def with_max_outer_iterations(self, max_outer_iterations):
        """Maximum number of outer iterations"""
        assert max_outer_iterations > 1
        self._max_outer_iterations = max_outer_iterations
        return self

    def with_penalty_weight_update_factor(self, penalty_weight_update_factor):
        assert penalty_weight_update_factor > 1.0
        self._penalty_weight_update_factor = penalty_weight_update_factor
        return self

    def with_initial_penalty_weights(self, initial_weights):
        self._initial_weights = initial_weights
        return self

    def with_max_duration_micros(self, max_duration_micros):
        assert(max_duration_micros >= 1)
        self._max_duration_micros = max_duration_micros
        return self

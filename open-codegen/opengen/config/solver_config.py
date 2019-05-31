class SolverConfiguration:
    """Configuration of solver parameters

    """

    def __init__(self):
        """Construct an instance of solver configuration parameters

        Returns:
            New instance of SolverConfiguration

        """
        self.__tolerance = 1e-4
        self.__lbfgs_memory = 10
        self.__max_inner_iterations = 500
        self.__max_outer_iterations = 10
        self.__constraints_tolerance = 1e-4
        self.__penalty_weight_update_factor = 5.0
        self.__initial_weights = 10.0
        self.__max_duration_micros = 5000000

    # --------- GETTERS -----------------------------
    @property
    def tolerance(self):
        """Tolerance of inner solver"""
        return self.__tolerance

    @property
    def lbfgs_memory(self):
        """LBFGS memory for the inner solver"""
        return self.__lbfgs_memory

    @property
    def max_inner_iterations(self):
        """Maximum number of iterations for the inner solver"""
        return self.__max_inner_iterations

    @property
    def constraints_tolerance(self):
        """Tolerance on the satisfaction of the constraints"""
        return self.__constraints_tolerance

    @property
    def max_outer_iterations(self):
        """Maximum number of iterations for the outer solver"""
        return self.__max_outer_iterations

    @property
    def penalty_weight_update_factor(self):
        """Multiplicative factor for the update of the penalty weights"""
        return self.__penalty_weight_update_factor

    @property
    def initial_penalty_weights(self):
        """Initial penalty weights"""
        if isinstance(self.__initial_weights, list):
            return self.__initial_weights
        else:
            return [self.__initial_weights]

    @property
    def max_duration_micros(self):
        """Maximum execution time in microseconds

        Returns:
            Integer value
        """
        return self.__max_duration_micros

    # --------- SETTERS -----------------------------
    def with_tolerance(self, tolerance):
        """Specify tolerance

        Raises:
            Exception: if tolerance <= 0

        Returns:
            The current object
        """
        if tolerance <= 0:
            raise Exception("The tolerance must be >0")
        self.__tolerance = float(tolerance)
        return self

    def with_lfbgs_memory(self, lbfgs_memory):
        """Specify L-BFGS memory

        Assertions:
            It is required that the L-BFGS memory is larger than or
            equal to 2

        Returns:
            Returns the current instance of SolverConfiguration
        """
        if lbfgs_memory < 2:
            raise Exception("The L-BFGS memory must be at least equal to 2")
        self.__lbfgs_memory = int(lbfgs_memory)
        return self

    def with_max_inner_iterations(self, max_iters):
        """Maximum number of inner iterations

        Returns:
            The current object
        """
        if max_iters < 1:
            raise Exception("The maximum number of inner iterations must be at least equal to 1")
        self.__max_inner_iterations = int(max_iters)
        return self

    def with_constraints_tolerance(self, constraints_tolerance):
        """Tolerance on constraint violation

        Returns:
            The current object
        """
        if constraints_tolerance <= 0:
            raise Exception("The constraints tolerance must be strictly positive")
        self.__constraints_tolerance = float(constraints_tolerance)
        return self

    def with_max_outer_iterations(self, max_outer_iterations):
        """Maximum number of outer iterations

        Returns:
            The current object
        """
        if max_outer_iterations < 1:
            raise Exception("The maximum number of outer iterations must be at least equal to 1")
        self.__max_outer_iterations = int(max_outer_iterations)
        return self

    def with_penalty_weight_update_factor(self, penalty_weight_update_factor):
        """Penalty update factor

        At every outer iteration of the penalty method, the weights are
        multiplied by this factor.

        Args:
            penalty_weight_update_factor: penalty weight update factor

        Raises:
            Exception: if the update factor is less than 1.0

        Returns:
            The current object
        """
        if penalty_weight_update_factor < 1.0:
            raise Exception("The penalty update factor needs to be >= 1")
        self.__penalty_weight_update_factor = float(penalty_weight_update_factor)
        return self

    def with_initial_penalty_weights(self, initial_weights):
        """Specify the initial penalty weights

        Returns:
            The current object
        """
        self.__initial_weights = initial_weights
        return self

    def with_max_duration_micros(self, max_duration_micros):
        """Specify the maximum duration in microseconds (must be an integer)

        Args:
            max_duration_micros: maximum execution duration in microseconds (integer)

        Raises:
            Exception: if <code>max_duration_micros</code> is less than 1

        Returns:
            The current object
        """
        if max_duration_micros < 1:
            raise Exception("The maximum duration (in microseconds) must be >= 1")
        self.__max_duration_micros = int(max_duration_micros)
        return self

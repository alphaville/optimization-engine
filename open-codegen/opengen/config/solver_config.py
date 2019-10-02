class SolverConfiguration:
    """Configuration of solver parameters

    """

    def __init__(self):
        """Construct an instance of solver configuration parameters

        Returns:
            New instance of SolverConfiguration

        """
        self.__tolerance = 1e-4
        self.__initial_tolerance = 1e-4
        self.__lbfgs_memory = 10
        self.__max_inner_iterations = 500
        self.__max_outer_iterations = 10
        self.__constraints_tolerance = 1e-4
        self.__initial_penalty = 1.0
        self.__penalty_weight_update_factor = 5.0
        self.__max_duration_micros = 5000000
        self.__inner_tolerance_update_factor = 0.1
        self.__sufficient_decrease_coefficient = 0.1
        self.__cbfgs_alpha = None
        self.__cbfgs_epsilon = None
        self.__cbfgs_sy_epsilon = None

    # --------- GETTERS -----------------------------

    @property
    def sufficient_decrease_coefficient(self):
        """Sufficient decrease coefficient"""
        return self.__sufficient_decrease_coefficient

    @property
    def initial_penalty(self):
        """Initial penalty"""
        return self.__initial_penalty

    @property
    def cbfgs_alpha(self):
        return self.__cbfgs_alpha

    @property
    def cbfgs_epsilon(self):
        return self.__cbfgs_epsilon

    @property
    def cbfgs_sy_epsilon(self):
        return self.__cbfgs_sy_epsilon

    @property
    def tolerance(self):
        """Tolerance of inner solver"""
        return self.__tolerance

    @property
    def initial_tolerance(self):
        """Initial tolerance of inner solver"""
        return self.__initial_tolerance

    @property
    def inner_tolerance_update_factor(self):
        """"Update factor for inner tolerance"""
        return self.__inner_tolerance_update_factor

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
    def max_duration_micros(self):
        """Maximum execution time in microseconds

        Returns:
            Integer value
        """
        return self.__max_duration_micros

    # --------- SETTERS -----------------------------

    def with_sufficient_decrease_coefficient(self, sufficient_decrease_coefficient):
        """
        Specify the sufficient decrease coefficient of the algorithm

            :param sufficient_decrease_coefficient: sufficient decrease coefficient

            :returns: The current object
        """
        if sufficient_decrease_coefficient <= 0.0 or sufficient_decrease_coefficient >= 1.0:
            raise Exception("sufficient decrease coeff must be in (0,1)")
        self.__sufficient_decrease_coefficient = sufficient_decrease_coefficient
        return self

    def with_initial_penalty(self, initial_penalty):
        """Initial penalty

        :param initial_penalty: initial value of penalty

        :returns: The current object
        """
        if initial_penalty <= 0:
            raise Exception("Initial penalty must be >0")

        self.__initial_penalty = float(initial_penalty)
        return self

    def with_tolerance(self, tolerance):
        """Specify tolerance

        :raises: Exception: if tolerance <= 0

        :returns: The current object
        """
        if tolerance <= 0:
            raise Exception("The tolerance must be >0")
        self.__tolerance = float(tolerance)
        return self

    def with_initial_tolerance(self, initial_tolerance):
        """
        Specify the initial tolerance

        :param initial_tolerance: initial tolerance

        :returns: The current object
        """
        if initial_tolerance <= 0:
            raise Exception("The initial tolerance must be >0")
        self.__initial_tolerance = float(initial_tolerance)
        return self

    def with_inner_tolerance_update_factor(self, inner_tol_update_factor):
        if inner_tol_update_factor <= 0 or inner_tol_update_factor > 1:
            raise Exception("The tolerance update factor must be in [0, 1)")

        self.__inner_tolerance_update_factor = float(inner_tol_update_factor)
        return self

    def with_lfbgs_memory(self, lbfgs_memory):
        """Specify L-BFGS memory

        :raises: It is required that the L-BFGS memory is larger than or
        equal to 2, otherwise an Exception is raised

        :returns: Returns the current instance of SolverConfiguration
        """
        if lbfgs_memory < 2:
            raise Exception("The L-BFGS memory must be at least equal to 2")
        self.__lbfgs_memory = int(lbfgs_memory)
        return self

    def with_max_inner_iterations(self, max_iters):
        """Maximum number of inner iterations

        :param max_iters: maximum number of iterations

        :returns: The current object
        """
        if max_iters < 1:
            raise Exception("The maximum number of inner iterations must be at least equal to 1")
        self.__max_inner_iterations = int(max_iters)
        return self

    def with_delta_tolerance(self, constraints_tolerance):
        """Tolerance on constraint violation

        :param constraints_tolerance: tolerance delta (related to constraint
        violation)

        :return: the current object
        """
        if constraints_tolerance <= 0:
            raise Exception("The constraints tolerance must be strictly positive")
        self.__constraints_tolerance = float(constraints_tolerance)
        return self

    def with_max_outer_iterations(self, max_outer_iterations):
        """Maximum number of outer iterations

        :return: the current object
        """
        if max_outer_iterations < 1:
            raise Exception("The maximum number of outer iterations must be at least equal to 1")
        self.__max_outer_iterations = int(max_outer_iterations)
        return self

    def with_penalty_weight_update_factor(self, penalty_weight_update_factor):
        """Penalty update factor

        At every outer iteration of the penalty method, the weights are
        multiplied by this factor.

        :param penalty_weight_update_factor: penalty weight update factor

        :raises: Exception: if the update factor is less than 1.0

        :return: the current object
        """
        if penalty_weight_update_factor < 1.0:
            raise Exception("The penalty update factor needs to be >= 1")
        self.__penalty_weight_update_factor = float(penalty_weight_update_factor)
        return self

    def with_max_duration_micros(self, max_duration_micros):
        """Specify the maximum duration in microseconds (must be an integer)

        :param max_duration_micros: maximum execution duration in microseconds (integer)

        :raises: Exception: if <code>max_duration_micros</code> is less than 1

        :returns: The current object
        """
        if max_duration_micros < 1:
            raise Exception("The maximum duration (in microseconds) must be >= 1")
        self.__max_duration_micros = int(max_duration_micros)
        return self

    def with_cbfgs_parameters(self, alpha, epsilon, sy_epsilon):
        """Specify the CBFGS parameters alpha and epsilon

        :param alpha: CBFGS parameter alpha
        :param epsilon: CBFGS parameter epsilon
        :param sy_epsilon: Tolerance on the s-y inner product

        :returns: the current object
        """
        if epsilon < 0.0:
            raise Exception("CBFGS parameter epsilon must be positive")

        if alpha < 0.0:
            raise Exception("CBFGS parameter alpha must be positive")

        self.__cbfgs_epsilon = epsilon
        self.__cbfgs_alpha = alpha
        self.__cbfgs_sy_epsilon = sy_epsilon
        return self

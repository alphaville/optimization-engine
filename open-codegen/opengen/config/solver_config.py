class SolverConfiguration:
    """Configuration of solver parameters

    """

    def __init__(self):
        """Construct an instance of solver configuration parameters

        Returns:
            :return: New instance of SolverConfiguration

        """
        self.__tolerance = 1e-4
        self.__initial_tolerance = 1e-4
        self.__lbfgs_memory = 10
        self.__max_inner_iterations = 500
        self.__max_outer_iterations = 10
        self.__constraints_tolerance = 1e-4
        # For the initial penalty, None means that the actual value will be computed
        # in Rust (see templates/optimizer.rs)
        self.__initial_penalty = None
        self.__penalty_weight_update_factor = 5.0
        self.__max_duration_micros = 5000000
        self.__inner_tolerance_update_factor = 0.1
        self.__sufficient_decrease_coefficient = 0.1
        self.__cbfgs_alpha = None
        self.__cbfgs_epsilon = None
        self.__cbfgs_sy_epsilon = None
        self.__do_preconditioning = False  # alpha version of preconditioning: optional

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
            :return: Integer value
        """
        return self.__max_duration_micros

    @property
    def preconditioning(self):
        """Whether an automatic preconditioning should be applied

        Returns:        
            :return: True iff preconditioning is active
        """
        return self.__do_preconditioning

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

        If preconditioning is activated, then the initial penalty is computed internally
        following the recommendations of the book of Brigin and Martinez (Chapter 12).
        If you enable the preconditioning and you use this method, then you will be
        overriding the value of the initial penalty.

        If preconditioning is not enabled, you can set the initial penalty using this
        method; if you don't do so, the solver will use the default value (which is 1.0).

        :param initial_penalty: initial value of penalty

        :returns: The current object
        """
        if initial_penalty is None:
            self.__initial_penalty = None
            return

        if initial_penalty <= 0:
            raise Exception("Initial penalty must be >0")

        self.__initial_penalty = float(initial_penalty)
        return self

    def with_tolerance(self, tolerance):
        """Specify tolerance

        :param tolerance: tolerance

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
        """Tolerance update factor

        The tolerance is initially given by 
        :py:meth:`~opengen.config.solver_config.SolverConfiguration.with_initial_tolerance`
        and it is then updated by this update factor until the target tolerance
        which is given by 
        :py:meth:`~opengen.config.solver_config.SolverConfiguration.with_tolerance`
        """
        if inner_tol_update_factor <= 0 or inner_tol_update_factor > 1:
            raise Exception("The tolerance update factor must be in [0, 1)")

        self.__inner_tolerance_update_factor = float(inner_tol_update_factor)
        return self

    def with_lbfgs_memory(self, lbfgs_memory):
        """Specify L-BFGS memory

        :param lbfgs_memory: LBFGS memory

        :raises: It is required that the L-BFGS memory is larger than or
            equal to 2, otherwise an Exception is raised

        :return: Returns the current instance of SolverConfiguration
        """
        if lbfgs_memory < 2:
            raise Exception("The L-BFGS memory must be at least equal to 2")
        self.__lbfgs_memory = int(lbfgs_memory)
        return self

    def with_max_inner_iterations(self, max_iters):
        """Maximum number of inner iterations

        :param max_iters: maximum number of iterations

        :return: The current object
        """
        if max_iters < 1:
            raise Exception(
                "The maximum number of inner iterations must be at least equal to 1")
        self.__max_inner_iterations = int(max_iters)
        return self

    def with_delta_tolerance(self, constraints_tolerance):
        """Tolerance on constraint violation

        :param constraints_tolerance: tolerance delta (related to constraint violation)

        :return: the current object
        """
        if constraints_tolerance <= 0:
            raise Exception(
                "The constraints tolerance must be strictly positive")
        self.__constraints_tolerance = float(constraints_tolerance)
        return self

    def with_max_outer_iterations(self, max_outer_iterations):
        """Maximum number of outer iterations

        :param max_outer_iterations: maximum number of outer iterations

        :return: the current object
        """
        if max_outer_iterations < 1:
            raise Exception(
                "The maximum number of outer iterations must be at least equal to 1")
        self.__max_outer_iterations = int(max_outer_iterations)
        return self

    def with_penalty_weight_update_factor(self, penalty_weight_update_factor):
        """Penalty update factor

        At every outer iteration of the penalty method, the weights are
        multiplied by this factor.

        :param penalty_weight_update_factor: penalty weight update factor

        :raises: Exception, if the update factor is less than 1.0

        :return: the current object
        """
        if penalty_weight_update_factor < 1.0:
            raise Exception("The penalty update factor needs to be >= 1")
        self.__penalty_weight_update_factor = float(
            penalty_weight_update_factor)
        return self

    def with_max_duration_micros(self, max_duration_micros):
        """Specify the maximum duration in microseconds (must be an integer)

        The solver will interrupt the computation after this time limit and 
        will return the current iterate.

        :param max_duration_micros: maximum execution duration in microseconds (integer)

        :raises: Exception: if `max_duration_micros` is less than 1

        :return: The current object
        """
        if max_duration_micros < 1:
            raise Exception(
                "The maximum duration (in microseconds) must be >= 1")
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

    def with_preconditioning(self, do_preconditioning=True):
        """Whether to apply preconditioning using the approach of [1]

        Note that this overrides the computation of the initial penalty

        Note also that unless this method is called, no preconditioning is
        applied (this may change in a future release; we may make enable
        preconditioning by default)

        [1] E.G. Birgin and J.M. Martinez, Practical Augmented Lagrangian
        Methods for Constrained Optimization, SIAM, 2014

        :param do_preconditioning: whether to precondition

        :returns: the current object
        """
        self.__do_preconditioning = do_preconditioning
        return self

    def to_dict(self):
        return {
            "tolerance": self.__tolerance,
            "initial_tolerance": self.__initial_tolerance,
            "lbfgs_memory": self.__lbfgs_memory,
            "max_inner_iterations": self.__max_inner_iterations,
            "max_outer_iterations": self.__max_outer_iterations,
            "constraints_tolerance": self.__constraints_tolerance,
            "initial_penalty": self.__initial_penalty,
            "penalty_weight_update_factor": self.__penalty_weight_update_factor,
            "max_duration_micros": self.__max_duration_micros,
            "inner_tolerance_update_factor": self.__inner_tolerance_update_factor,
            "sufficient_decrease_coefficient": self.__sufficient_decrease_coefficient,
            "cbfgs_alpha": self.__cbfgs_alpha,
            "cbfgs_epsilon": self.__cbfgs_epsilon,
            "cbfgs_sy_epsilon": self.__cbfgs_sy_epsilon,
            "do_preconditioning": self.__do_preconditioning
        }

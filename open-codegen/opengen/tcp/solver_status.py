class SolverStatus:
    """Class for storing the status of the solver."""
    def __init__(self, status):
        """Constructs instance of <code>SolverStatus</code>

        Args:
            status: dictionary containing solver status attributes

        Returns:
            New instance of <code>SolverStatus</code>
        """

        for k, v in status.items():
            attribute_name = "__{}".format(k)
            setattr(self, attribute_name, v)

    @property
    def exit_status(self):
        """
        Returns:
        Converged if the solver has converged

        NotConvergedIterations if the maximum number of outer or total inner
        iterations was reached

        NotConvergedOutOfTime if the solver did not have enough time to
        converge

        :return: The exit status of the solver
        """
        return self.__dict__["__exit_status"]

    @property
    def num_outer_iterations(self):
        """
        Returns the number of outer (ALM/PM) iterations of the algorithm
        :return: The number of outer iterations
        """
        return self.__dict__["__num_outer_iterations"]

    @property
    def num_inner_iterations(self):
        """
        Returns the total number of inner iterations for all inner problems
        :return: The total number of inner iterations
        """
        return self.__dict__["__num_inner_iterations"]

    @property
    def last_problem_norm_fpr(self):
        """
        Returns the infinity norm of the fixed-point residual of the last
        inner optimization problem
        :return: inf-norm of the FPR of the last inner problem
        """
        return self.__dict__["__last_problem_norm_fpr"]

    @property
    def f1_infeasibility(self):
        """
        Returns the distance between F1(u,p) and C at the solution, a measure
        of the infeasibility of the constraint F1(u,p) in C
        :return: Infeasibility of the constraint F1(u,p) in C
        """
        return self.__dict__["__delta_y_norm_over_c"]

    @property
    def f2_norm(self):
        """
        :return: Euclidean norm of f2(u,p) at the solution
        """
        return self.__dict__["__f2_norm"]

    @property
    def solve_time_ms(self):
        """
        :return: Total execution time in milliseconds
        """
        return self.__dict__["__solve_time_ms"]

    @property
    def penalty(self):
        """
        :return: Last value of the penalty parameter
        """
        return self.__dict__["__penalty"]

    @property
    def solution(self):
        """
        :return: Solution vector
        """
        return self.__dict__["__solution"]

    @property
    def lagrange_multipliers(self):
        """
        :return: Vector of Lagrange multipliers
        """
        return self.__dict__["__lagrange_multipliers"]


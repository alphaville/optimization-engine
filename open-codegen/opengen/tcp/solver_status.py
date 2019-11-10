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
        Returns value at key __exit_status
        :return: The exit status of the solver
        """
        return self.__dict__["__exit_status"]

    @property
    def num_outer_iterations(self):
        """
        Returns value at key __num_outer_iterations
        :return: The number of outer iterations
        """
        return self.__dict__["__num_outer_iterations"]

    @property
    def num_inner_iterations(self):
        """
        Returns value at key __num_inner_iterations
        :return: The total number of inner iterations
        """
        return self.__dict__["__num_inner_iterations"]

    @property
    def last_problem_norm_fpr(self):
        """
        Returns value at key __last_problem_norm_fpr
        :return: Norm of the fixed-point residual of the last inner problem
        """
        return self.__dict__["__last_problem_norm_fpr"]

    @property
    def delta_y_norm_over_c(self):
        """
        Returns value at key __delta_y_norm_over_c
        :return: Distance between f1(u,p) and C at the solution
        """
        return self.__dict__["__delta_y_norm_over_c"]

    @property
    def f2_norm(self):
        """
        Returns value at key __f2_norm
        :return: Euclidean norm of f2(u,p) at the solution
        """
        return self.__dict__["__f2_norm"]

    @property
    def solve_time_ms(self):
        """
        Returns value at key __solve_time_ms
        :return: Total execution time in milliseconds
        """
        return self.__dict__["__solve_time_ms"]

    @property
    def penalty(self):
        """
        Returns value at key __penalty
        :return: Last value of the penalty parameter
        """
        return self.__dict__["__penalty"]

    @property
    def solution(self):
        """
        Returns value at key __solution
        :return: Solution vector
        """
        return self.__dict__["__solution"]

    @property
    def lagrange_multipliers(self):
        """
        Returns value at key __lagrange_multipliers
        :return: Vector of Lagrange multipliers
        """
        return self.__dict__["__lagrange_multipliers"]


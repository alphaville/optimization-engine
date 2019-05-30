from casadi import SX, Function


class Problem:
    """Definition of an optimization problem

    Provides the cost function, constraints and additional
    penalty-type constraints.
    """

    def __init__(self, u, p, cost):
        """Construct an optimization problem

        Args:
            u: decision variable (CasADi variable)
            p: parameter (CasADi variable)
            cost: cost function (CasADi function of u and p)

        Example:
            >>> import casadi.casadi as cs
            >>> import opengen as og
            >>> # Define u and p
            >>> u = cs.SX.sym('u', 5)
            >>> p = cs.SX.sym('p', 2)
            >>> # Cost function
            >>> phi = og.functions.rosenbrock(u, p)
            >>> # Define optimization problem
            >>> problem = og.builder.Problem(u, p, phi)

        """
        self.__u = u
        self.__p = p
        self.__cost = cost
        self.__u_constraints = None
        self.__penalty_constraints = None
        self.__penalty_function = None
        self.__al_constraints = None

    # ---------- SETTERS -----------------------------------------------

    def with_constraints(self, u_constraints):
        """Specify or update the constraints of the problem

        Args:
            u_constraints: constraints on the decision variable; must
            be a Constraint object (such as
            \link opengen.constraints.ball2.Ball2 Ball2 \endlink
            and
            \link opengen.constraints.rectangle.Rectangle Rectangle \endlink)

        Returns:
            Current object
        """
        self.__u_constraints = u_constraints
        return self

    def with_penalty_constraints(self, penalty_constraints, penalty_function=None):
        """Constraints to for the penalty method

        Specify the constraints to be treated with the penalty method (that is,
        function c(u; p)) and the penalty function, g. If no penalty function
        is specified, the quadratic penalty will be used.

        Parameters:
            penalty_constraints:
            penalty_function:

        Returns:
            self
        """
        if penalty_constraints is None:
            pass

        self.__penalty_constraints = penalty_constraints
        if penalty_function is None:
            # default penalty function: quadratic
            z = SX.sym("z")
            self.__penalty_function = Function('g_penalty_function', [z], [z ** 2])
        else:
            self.__penalty_function = penalty_function
        return self

    def with_aug_lagrangian_constraints(self, al_constraints):
        # TODO This is not supported yet
        self.__al_constraints = al_constraints
        return self

    # ---------- DIMENSIONS --------------------------------------------

    def dim_decision_variables(self):
        return self.__u.size(1)

    def dim_parameters(self):
        return self.__p.size(1)

    def dim_constraints_penalty(self):
        return 0 if self.__penalty_constraints is None \
            else self.__penalty_constraints.size(1)

    def dim_constraints_aug_lagrangian(self):
        return 0 if self.__al_constraints is None \
            else self.__al_constraints.size(1)

    # ---------- OTHER GETTERS -----------------------------------------

    @property
    def cost_function(self):
        return self.__cost

    @property
    def penalty_constraints(self):
        return self.__penalty_constraints

    @property
    def penalty_function(self):
        return self.__penalty_function

    @property
    def constraints(self):
        return self.__u_constraints

    @property
    def constraints_aug_lagrangian(self):
        return self.__al_constraints

    @property
    def decision_variables(self):
        return self.__u

    @property
    def parameter_variables(self):
        return self.__p
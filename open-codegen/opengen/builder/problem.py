import casadi.casadi as cs


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
            and \link opengen.constraints.rectangle.Rectangle Rectangle \endlink)

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

        Args:
            penalty_constraints: a function <code>c(u, p)</code>, of the decision
            variable <code>u</code> and the parameter vector <code>p</code>, which
            corresponds to the constraints <code>c(u, p)</code>
            penalty_function: a function <code>g: R -> R</code>, used to define the
            penalty in the penalty method; the default is <code>g(z) = z^2</code>.
            You typically will not need to change this, but if you very much want to,
            you need to provide an instance of <code>casadi.casadi.Function</code>
            (not a CasADi symbol such as <code>SX</code>).

        Returns:
            self
        """
        if penalty_constraints is None:
            pass

        self.__penalty_constraints = penalty_constraints
        if penalty_function is None:
            # default penalty function: quadratic
            z = cs.SX.sym("z")
            self.__penalty_function = cs.Function('g_penalty_function', [z], [z ** 2])
        else:
            self.__penalty_function = penalty_function
        return self

    def with_aug_lagrangian_constraints(self, al_constraints):
        """Not implemented yet"""
        raise NotImplementedError

    # ---------- DIMENSIONS --------------------------------------------

    def dim_decision_variables(self):
        """Number of decision variables"""
        return self.__u.size(1)

    def dim_parameters(self):
        """Number of parameters"""
        return self.__p.size(1)

    def dim_constraints_penalty(self):
        """Number of penalty-type constraints"""
        return 0 if self.__penalty_constraints is None \
            else self.__penalty_constraints.size(1)

    def dim_constraints_aug_lagrangian(self):
        """Not implemented yet"""
        return 0

    # ---------- OTHER GETTERS -----------------------------------------

    @property
    def cost_function(self):
        """Cost function as a CaADi symbol"""
        return self.__cost

    @property
    def penalty_constraints(self):
        """Penalty constraints as a CasADi symbol (function c)"""
        return self.__penalty_constraints

    @property
    def penalty_function(self):
        """Penalty function, <code>g</code>"""
        return self.__penalty_function

    @property
    def constraints(self):
        """Hard constraints"""
        return self.__u_constraints

    @property
    def constraints_aug_lagrangian(self):
        """Not implemented yet"""
        raise NotImplementedError

    @property
    def decision_variables(self):
        """Decision variables (CasADi symbol)"""
        return self.__u

    @property
    def parameter_variables(self):
        """Parameter variables (CasADi symbol)"""
        return self.__p

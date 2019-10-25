import casadi.casadi as cs
from .set_y_calculator import SetYCalculator
from ..constraints.no_constraints import NoConstraints


class Problem:
    """Definition of an optimization problem

    Provides the cost function, constraints and additional
    ALM/penalty-type constraints.
    """

    def __init__(self, u, p, cost):
        """Construct an optimization problem

            :param u: decision variable (CasADi variable)
            :param p: parameter (CasADi variable)
            :param cost: cost function (CasADi function of u and p)

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

        # Decision variable: u
        self.__u = u
        # Parameter: p
        self.__p = p
        # Cost function: f(u, p)
        self.__cost = cost
        # Constraints on u: u in U
        self.__u_constraints = NoConstraints()
        # ALM-type constraints: mapping F1(u, p)
        self.__alm_mapping_f1 = None
        # ALM-type constraints: set C
        # Corresponds to the constraints: F1(u, p) in C
        self.__alm_set_c = None
        # Set Y of dual variables (compact subset of C*)
        self.__alm_set_y = None
        # Penalty-type constraints: mapping F2(u, p)
        # Constraints: F1(u, p) = 0
        self.__penalty_mapping_f2 = None
        # penalty function - default (and most commonly used): squared 2-norm
        self.__penalty_function = None

    # ---------- SETTERS -----------------------------------------------

    def with_constraints(self, u_constraints):
        """Specify or update the constraints of the problem

        Args:
            u_constraints: constraints on the decision variable; must
                           be a Constraint object (such as
                           opengen.constraints.ball2.Ball2
                           and opengen.constraints.rectangle.Rectangle)

        Returns:
            Current object

        """
        self.__u_constraints = u_constraints
        return self

    def with_penalty_constraints(self, penalty_constraints):
        """Constraints to for the penalty method

        Specify the constraints to be treated with the penalty method (that is,
        function F2(u; p))

        :param penalty_constraints: a function <code>c(u, p)</code>, of the decision
                variable <code>u</code> and the parameter vector <code>p</code>, which
                corresponds to the constraints <code>c(u, p)</code>

        :return: self
        """

        self.__penalty_mapping_f2 = penalty_constraints
        return self

    def with_aug_lagrangian_constraints(self, mapping_f1, set_c, set_y=None):
        """
        Constraints: F1(u, p) in C

        :param mapping_f1: mapping of the form `F1: R^{n} x R^{p} --> R^{n1}`
        :param set_c: a convex closed set C
        :param set_y: a compact subset of C*, the convex conjugate of C

        :return: self
        """
        if not set_c.is_convex():
            raise Exception("Set C must be convex")

        self.__alm_mapping_f1 = mapping_f1
        self.__alm_set_c = set_c
        if set_y is not None:
            self.__alm_set_y = set_y
        else:
            # Y not provided
            c = SetYCalculator(set_c)
            self.__alm_set_y = c.obtain()
        return self

    # ---------- DIMENSIONS --------------------------------------------

    def dim_decision_variables(self):
        """Number of decision variables

        :return: number of decision variables
        """
        return self.__u.size(1)

    def dim_parameters(self):
        """Number of parameters"""
        return self.__p.size(1)

    def dim_constraints_penalty(self):
        """Number of penalty-type constraints"""
        return 0 if self.__penalty_mapping_f2 is None \
            else self.__penalty_mapping_f2.size(1)

    def dim_constraints_aug_lagrangian(self):
        """Not implemented yet"""
        return 0 if self.__alm_mapping_f1 is None \
            else self.__alm_mapping_f1.size(1)

    # ---------- OTHER GETTERS -----------------------------------------

    @property
    def cost_function(self):
        """Cost function as a CaADi symbol"""
        return self.__cost

    @property
    def penalty_mapping_f2(self):
        """Penalty-type mapping F2 as a CasADi symbol"""
        return self.__penalty_mapping_f2

    @property
    def penalty_mapping_f1(self):
        """ALM mapping F1 as a CasADi symbol"""
        return self.__alm_mapping_f1

    @property
    def alm_set_c(self):
        """Set C in the definition of constraints: F1(u, p) in C"""
        return self.__alm_set_c

    @property
    def alm_set_y(self):
        """Set Y for the Lagrange multipliers"""
        return self.__alm_set_y

    @property
    def penalty_function(self):
        """Penalty function, g"""
        return self.__penalty_function

    @property
    def constraints(self):
        """Hard constraints; set U"""
        return self.__u_constraints

    @property
    def decision_variables(self):
        """Decision variables (CasADi symbol)"""
        return self.__u

    @property
    def parameter_variables(self):
        """Parameter variables (CasADi symbol)"""
        return self.__p

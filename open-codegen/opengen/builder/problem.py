import casadi.casadi as cs


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
        self.__u_constraints = None
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
        # @deprecated - to be removed
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

    def with_penalty_constraints(self, penalty_constraints, penalty_function=None):
        """Constraints to for the penalty method

        Specify the constraints to be treated with the penalty method (that is,
        function c(u; p)) and the penalty function, g. If no penalty function
        is specified, the quadratic penalty will be used.


        :param penalty_constraints: a function <code>c(u, p)</code>, of the decision
                variable <code>u</code> and the parameter vector <code>p</code>, which
                corresponds to the constraints <code>c(u, p)</code>

        :param penalty_function: a function <code>g: R -> R</code>, used to define the
                penalty in the penalty method; the default is <code>g(z) = z^2</code>.
                You typically will not need to change this, but if you very much want to,
                you need to provide an instance of <code>casadi.casadi.Function</code>
                (not a CasADi symbol such as <code>SX</code>).

        :return: self
        """
        if penalty_constraints is None:
            pass

        self.__penalty_mapping_f2 = penalty_constraints
        if penalty_function is None:
            # default penalty function: quadratic
            z = cs.SX.sym("z")
            self.__penalty_function = cs.Function('g_penalty_function', [z], [z ** 2])
        else:
            self.__penalty_function = penalty_function
        return self

    def with_aug_lagrangian_constraints(self, mapping_f1, set_c, set_y):
        """
        Constraints F1(u, p) in C

        :param mapping_f1:
        :param set_c:
        :param set_y:

        :return: self
        """
        self.__alm_mapping_f1 = mapping_f1
        self.__alm_set_c = set_c
        self.__alm_set_y = set_y
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
        return 0

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

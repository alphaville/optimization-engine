import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
from opengen.constraints import Constraint
from .optimizer_formulations import single_shooting_formulation, multiple_shooting_formulation
from .ocp_build_interface import tcp_interface, direct_interface
from .type_enums import *



class OptimalControlProblem:
    def __init__(self, p_symb, nx, nu, sys_dyn_fn, stage_cost_fn, terminal_cost_fn):
        self.__p_symb = p_symb
        self.__nx = nx
        self.__nu = nu
        self.__horizon = 15
        self.__x_set = og.constraints.NoConstraints()
        self.__u_set = og.constraints.NoConstraints()
        self.__formulation_type = FormulationType.MULTIPLE_SHOOTING
        self.__ocp_build_interface = OcpInterfaceType.TCP
        self.__exclusion_set = None
        self.__problem = None
        self.sys_dyn_fn = sys_dyn_fn
        self.stage_cost_fn = stage_cost_fn
        self.terminal_cost_fn = terminal_cost_fn

    def with_horizon(self, horizon):
        self.__horizon = horizon
        return self

    def with_state_constraint(self, x_set):
        self.__x_set = x_set
        return self

    def with_input_constraint(self, u_set):
        self.__u_set = u_set
        return self

    def with_formulation_type(self, formulation):
        self.__formulation_type = formulation
        return self

    def with_exclusion_set(self, exclusion_set):
        self.__exclusion_set = exclusion_set
        return self

    def with_build_interface(self, ocp_build_interface):
        if ocp_build_interface in (OcpInterfaceType.TCP, OcpInterfaceType.DIRECT):
            self.__ocp_build_interface = ocp_build_interface
        else:
            raise Exception("Build Interface type not Supported")
        return self

    def save_OCP(self, problem):
        if isinstance(problem, og.builder.Problem):
            self.__problem = problem
        else:
            raise Exception("Problem must belong to class opengen.builder.Problem")

    def get_nx(self):
        return self.__nx

    def get_nu(self):
        return self.__nu

    def get_horizon(self):
        return self.__horizon

    def get_state_constraint(self):
        return self.__x_set

    def get_input_constraint(self):
        return self.__u_set

    def get_formulation_type(self):
        return self.__formulation_type

    def get_exclusion_set(self):
        return self.__exclusion_set

    def get_ocp_build_interface(self):
        return self.__ocp_build_interface

    def get_OCP(self):
        return self.__problem

    def build(self):
        nu = self.__nu
        nx = self.__nx
        N = self.__horizon
        x_init = self.__p_symb[0:nx]
        ocp_build_interface = self.__ocp_build_interface

        u = cs.SX.sym('u', nu * N)
        # x = cs.SX.sym('x', nx * (N + 1))
        x = cs.vertcat(x_init, cs.SX.sym('x', nx * N))

        if self.__formulation_type is FormulationType.MULTIPLE_SHOOTING:
            problem_formulation = multiple_shooting_formulation
        elif self.__formulation_type is FormulationType.SINGLE_SHOOTING:
            problem_formulation = single_shooting_formulation
        else:
            raise Exception("Formulation type not Supported")
        (cost, decision_var, bounds, alm_mapping, alm_set, pm_constraints) = problem_formulation(self, self.__p_symb, u, nu, x, nx, N)

        self.__problem = og.builder.Problem(decision_var, self.__p_symb, cost) \
            .with_constraints(bounds) \

        if fn.is_symbolic(alm_mapping):
            self.__problem.with_aug_lagrangian_constraints(alm_mapping, alm_set)

        if fn.is_symbolic(pm_constraints):
            self.__problem.with_penalty_constraints(pm_constraints)

        build_config = og.config.BuildConfiguration() \
            .with_build_mode("debug") \
            .with_build_python_bindings()

        if ocp_build_interface is OcpInterfaceType.TCP:
            build_config.with_build_directory("my_optimizers_tcp")
            build_config.with_tcp_interface_config()
        else:
            build_config.with_build_directory("my_optimizers")
            # build_config.with_build_c_bindings()

        meta = og.config.OptimizerMeta() \
            .with_optimizer_name("navigation")
        solver_config = og.config.SolverConfiguration() \
            .with_tolerance(1e-3) \
            .with_initial_tolerance(1e-2) \
            .with_initial_penalty(1000) \
            .with_penalty_weight_update_factor(1.25) \
            .with_max_outer_iterations(65) \
            .with_max_inner_iterations(2000) \
            .with_delta_tolerance(1e-3)
        builder = og.builder.OpEnOptimizerBuilder(self.__problem,
                                                  meta,
                                                  build_config,
                                                  solver_config)
        builder.build()

    def solve(self, p_init, print_result):

        if self.__ocp_build_interface is OcpInterfaceType.TCP:
            ocp_build_function = tcp_interface
        else:
            ocp_build_function = direct_interface

        return ocp_build_function(p_init, print_result)


class BallExclusionSet(Constraint):
    def __init__(self, set_type, center, radius, state_indices=None, method=ConstraintMethod.PM):

        supported_constraints_list = (og.constraints.Ball1, og.constraints.Ball2, og.constraints.BallInf)
        if not isinstance(set_type, supported_constraints_list):
            raise Exception("Constraint Not Supported")
        self.__set_type = set_type

        self.__center = center
        self.__radius = radius

        if state_indices is None:
            state_indices = [i for i in range(len(center))]
        else:
            if state_indices[0] < 0:
                raise ValueError("the first element of segment must be a positive integer")
            if any([state_indices[i] >= state_indices[i + 1] for i in range(len(state_indices) - 1)]):
                raise ValueError("segments should be a list of integers in strictly ascending order")

        self.__state_indices = state_indices

        constraint_method_list = (ConstraintMethod.PM, ConstraintMethod.ALM)
        if not method in constraint_method_list:
            raise Exception("Constraint Method not Supported")
        self.__method = method

    def append_exclusion_set(self, set_exclusion_fn, x, nx, N):
        radius = self.__radius
        centre = self.__center
        if isinstance(self.__set_type, og.constraints.Ball2):
            for iteration in range(N):
                x_current = x[iteration*nx:(iteration+1)*nx]
                x_trunc = [x_current[i] for i in self.__state_indices]
                val = radius**2
                for index in range(len(x_trunc)):
                    val -= (x_trunc[index] - centre[index])**2
                set_exclusion_fn = cs.vertcat(set_exclusion_fn, fn.fmax(0.0, val))

        return set_exclusion_fn

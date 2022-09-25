import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
from .optimizer_formulations import single_shooting_formulation, multiple_shooting_formulation
import numpy as np
from .type_enums import *

class OCPBuilder:
    def __init__(self, ocp):
        self.__ocp = ocp
        self.__build_config = None
        self.__meta = None
        self.__solver_config = None
        self.__ocp_build_interface = OcpInterfaceType.TCP
        self.__default_max_outer_iterations = 30

    def with_build_config(self, build_config):
        self.__build_config = build_config
        return self

    def with_meta(self, meta):
        self.__meta = meta
        return self

    def with_solver_config(self, solver_config):
        self.__solver_config = solver_config
        self.__default_max_outer_iterations = solver_config.max_outer_iterations
        return self

    def with_build_interface(self, ocp_build_interface):
        if ocp_build_interface in (OcpInterfaceType.TCP, OcpInterfaceType.DIRECT):
            self.__ocp_build_interface = ocp_build_interface
        else:
            raise Exception("Build Interface type not Supported")
        return self

    def __build_default_config(self):
        ocp_build_interface = self.__ocp_build_interface
        if self.__build_config is None:
            self.__build_config = og.config.BuildConfiguration() \
                .with_build_mode("release") \
                .with_build_python_bindings()

            if ocp_build_interface is OcpInterfaceType.TCP:
                self.__build_config.with_build_directory("my_optimizers_tcp")
                self.__build_config.with_tcp_interface_config()
            else:
                self.__build_config.with_build_directory("my_optimizers")
            # build_config.with_build_c_bindings()

        if self.__meta is None:
            self.__meta = og.config.OptimizerMeta() \
                .with_optimizer_name("navigation")

        if self.__solver_config is None:
            self.__solver_config = og.config.SolverConfiguration() \
                .with_tolerance(1e-4) \
                .with_initial_tolerance(1e-2) \
                .with_initial_penalty(5000) \
                .with_penalty_weight_update_factor(1.25) \
                .with_inner_tolerance_update_factor(0.1) \
                .with_max_outer_iterations(30) \
                .with_max_inner_iterations(2000) \
                .with_delta_tolerance(1e-3) \
                .with_preconditioning(True) \
                .with_optimized_initial_penalty(True)

    def build(self):
        ocp = self.__ocp
        nu = ocp.nu
        nx = ocp.nx
        N = ocp.horizon
        p_symb = ocp.p_symb
        x_init = p_symb[0:nx]

        self.__build_default_config()

        u = cs.SX.sym('u', nu * N)
        x = cs.vertcat(x_init, cs.SX.sym('x', nx * N))

        if ocp.formulation_type is FormulationType.MULTIPLE_SHOOTING:
            problem_formulation = multiple_shooting_formulation
        elif ocp.formulation_type is FormulationType.SINGLE_SHOOTING:
            problem_formulation = single_shooting_formulation
        else:
            raise Exception("Formulation type not Supported")

        (cost, decision_var, bounds, alm_mapping, alm_set, pm_constraints) = problem_formulation(ocp, p_symb, u, nu, x, nx, N)

        problem = og.builder.Problem(decision_var, p_symb, cost) \
            .with_constraints(bounds) \

        if fn.is_symbolic(alm_mapping):
            problem.with_aug_lagrangian_constraints(alm_mapping, alm_set)

        if fn.is_symbolic(pm_constraints):
            problem.with_penalty_constraints(pm_constraints)

        if self.__solver_config.preconditioning is True:
            problem.update_problem_with_preconditioning()

        if self.__solver_config.optimize_initial_penalty is True:
            self.__solver_config.with_max_outer_iterations(1)
            self.__meta.with_optimizer_name(self.__meta.optimizer_name + "_init")

        ocp.save_OCP(problem)

        builder = og.builder.OpEnOptimizerBuilder(self.__ocp.problem,
                                                  self.__meta,
                                                  self.__build_config,
                                                  self.__solver_config)
        builder.build()


    def __calculate_preconditioning_coefficients(self, p_val, u_val = None):
        """
                Computes the values of preconditioning coefficients

                This function computes the values of preconditioning coefficients:
                - w_cost
                - w1
                - w2

                :returns: [[w_cost], w1, w2]
                """
        problem = self.__ocp.problem
        u = problem.decision_variables
        p = problem.parameter_variables
        c_f1 = problem.penalty_mapping_f1
        n1 = problem.dim_constraints_aug_lagrangian()
        c_f2 = problem.penalty_mapping_f2
        n2 = problem.dim_constraints_penalty()
        phi = problem.cost_function
        if u_val is None:
            u_val = [0] * self.__ocp.problem.dim_decision_variables()
        theta_val_raw = p_val + [1] * (1 + n1 + n2)

        w_cost_symb = [1 / cs.fmax(1, cs.norm_inf(cs.jacobian(phi, u)))]
        w_cost_fn = cs.Function('w_cost_fn', [u, p], w_cost_symb)
        w_cost = w_cost_fn(u_val, theta_val_raw)

        theta_val = p_val + [w_cost[0].__float__()]

        w1_symb = []
        if n1 > 0:
            jac_f1 = cs.jacobian(c_f1, u)
            for i in range(n1):
                w1_symb_i = 1 / cs.fmax(1, cs.norm_inf(jac_f1[i, :]))
                w1_symb = cs.vertcat(w1_symb, w1_symb_i)
            w_constraint_f1_fn = cs.Function('w_constraint_f1_fn', [u, p], [w1_symb])
            w1 = w_constraint_f1_fn(u_val, theta_val_raw)
            theta_val = theta_val + [w1[i].__float__() for i in range(w1.size(1))]

        w2_symb = []
        if n2 > 0:
            jac_f2 = cs.jacobian(c_f2, u)
            for i in range(n2):
                w2_symb_i = 1 / cs.fmax(1, cs.norm_inf(jac_f2[i, :]))
                w2_symb = cs.vertcat(w2_symb, w2_symb_i)
            w_constraint_f2_fn = cs.Function('w_constraint_f2_fn', [u, p], [w2_symb])
            w2 = w_constraint_f2_fn(u_val, theta_val_raw)
            theta_val = theta_val + [w2[i].__float__() for i in range(w2.size(1))]

        return theta_val

    def __calculate_initial_penalty(self, u_initial_guess, theta_val):
        """
                Computes the values of initial penalty and max_outer_iterations

                This function computes the values of preconditioning coefficients:
                - initial_penalty
                - max_outer_iterations

                :returns: (initial_penalty, max_outer_iterations)
                """
        problem = self.__ocp.problem
        u = problem.decision_variables
        theta = problem.parameter_variables
        cost_fn = cs.Function('cost_fn', [u, theta], [problem.cost_function])
        cost = cost_fn(u_initial_guess, theta_val)
        cost = cost.__float__()
        infeasibility_psi_fn = cs.Function('infeasibility_psi_fn', [u, theta], [problem.infeasibility_psi])
        infeasibility_psi = infeasibility_psi_fn(u_initial_guess, theta_val)
        infeasibility_psi = infeasibility_psi.__float__()

        init_penalty = 10 * max(1, abs(cost))
        init_penalty /= max(1, abs(infeasibility_psi))
        init_penalty = max(1e3, min(init_penalty, 1e8))

        solver_config = self.__solver_config
        max_penalty = solver_config.max_penalty_allowed
        max_outer_iterations = self.__default_max_outer_iterations
        penalty_weight_update_factor = 10**(np.log10(max_penalty/init_penalty)/max_outer_iterations)

        return init_penalty, penalty_weight_update_factor


    def solve(self, p_init, print_result):

        if self.__solver_config.preconditioning is True:
            p_init = self.__calculate_preconditioning_coefficients(p_val=p_init)

        if self.__ocp_build_interface is OcpInterfaceType.TCP:
            ocp_solve_function = tcp_interface
        else:
            ocp_solve_function = direct_interface

        if self.__solver_config.optimize_initial_penalty is True:
            u_initial_guess = ocp_solve_function(p_init, self.__meta.optimizer_name, print_result=False)
            (init_penalty, penalty_weight_update_factor) = self.__calculate_initial_penalty(u_initial_guess, p_init)
            self.__solver_config.with_initial_penalty(init_penalty)
            self.__solver_config.with_penalty_weight_update_factor(penalty_weight_update_factor)
            self.__solver_config.with_max_outer_iterations(self.__default_max_outer_iterations)
            self.__solver_config.with_optimized_initial_penalty(False)
            self.__meta.with_optimizer_name(self.__meta.optimizer_name[:-5])
            self.__build_config.with_rebuild(False)
            builder = og.builder.OpEnOptimizerBuilder(self.__ocp.problem,
                                                      self.__meta,
                                                      self.__build_config,
                                                      self.__solver_config)
            builder.build()

        return ocp_solve_function(p_init, self.__meta.optimizer_name, print_result)


def tcp_interface(z_initial, optimizer_name, print_result=False):
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers_tcp/' + optimizer_name)
    mng.start()

    mng.ping()
    solution = mng.call(z_initial)
    mng.kill()

    if print_result:
        print('cost: \t%s' % solution["cost"])
        print('exit_status: \t%s' % solution["exit_status"])
        print('f1_infeasibility: \t%s' % solution["f1_infeasibility"])
        print('f2_norm: \t%s' % solution["f2_norm"])
        print('num_inner_iterations: \t%s' % solution["num_inner_iterations"])
        print('num_outer_iterations: \t%s' % solution["num_outer_iterations"])
        print('penalty: \t%s' % solution["penalty"])
        print('solve_time_ms: \t%s \n' % solution["solve_time_ms"])

    return solution['solution']


def direct_interface(z_initial, optimizer_name, print_result=False):
    # Note: to use the direct interface you need to build using
    #       .with_build_python_bindings()
    import sys

    # Use Direct Interface
    # ------------------------------------
    sys.path.insert(1, './my_optimizers/' + optimizer_name)
    optimizer = __import__(optimizer_name)

    solver = optimizer.solver()
    result = solver.run(z_initial)

    if result:
        if print_result:
            print('cost: \t%s' % result.cost)
            print('exit_status: \t%s' % result.exit_status)
            print('f1_infeasibility: \t%s' % result.f1_infeasibility)
            print('f2_norm: \t%s' % result.f2_norm)
            print('num_inner_iterations: \t%s' % result.num_inner_iterations)
            print('num_outer_iterations: \t%s' % result.num_outer_iterations)
            print('penalty: \t%s' % result.penalty)
            print('solve_time_ms: \t%s \n' % result.solve_time_ms)
    else:
        raise AssertionError("No Solution")

    return result.solution


import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
from .optimizer_formulations import single_shooting_formulation, multiple_shooting_formulation
from .type_enums import *

class OCPBuilder:
    def __init__(self, ocp, build_config=None, meta=None, solver_config=None):
        self.__ocp = ocp
        self.__build_config = build_config
        self.__meta = meta
        self.__solver_config = solver_config

    def with_build_config(self, build_config):
        self.__build_config = build_config
        return self

    def with_meta(self, meta):
        self.__meta = meta
        return self

    def with_solver_config(self, solver_config):
        self.__solver_config = solver_config
        return self

    def build(self):
        ocp = self.__ocp
        nu = ocp.nu
        nx = ocp.nx
        N = ocp.horizon
        p_symb = ocp.p_symb
        x_init = p_symb[0:nx]
        ocp_build_interface = ocp.ocp_build_interface

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

        ocp.save_OCP(problem)

        if self.__build_config is None:
            self.__build_config = og.config.BuildConfiguration() \
                .with_build_mode("debug") \
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
                .with_tolerance(1e-3) \
                .with_initial_tolerance(1e-2) \
                .with_initial_penalty(1000) \
                .with_penalty_weight_update_factor(1.25) \
                .with_inner_tolerance_update_factor(0.1) \
                .with_max_outer_iterations(65) \
                .with_max_inner_iterations(2000) \
                .with_delta_tolerance(1e-3)

        builder = og.builder.OpEnOptimizerBuilder(self.__ocp.problem,
                                                  self.__meta,
                                                  self.__build_config,
                                                  self.__solver_config)
        builder.build()

    def solve(self, p_init, print_result):

        if self.__ocp.ocp_build_interface is OcpInterfaceType.TCP:
            ocp_build_function = tcp_interface
        else:
            ocp_build_function = direct_interface

        return ocp_build_function(p_init, print_result)


def tcp_interface(z_initial, print_result=False):
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('my_optimizers_tcp/navigation')
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
        print('solve_time_ms: \t%s' % solution["solve_time_ms"])

    return solution['solution']


def direct_interface(z_initial, print_result=False):
    # Note: to use the direct interface you need to build using
    #       .with_build_python_bindings()
    import sys

    # Use Direct Interface
    # ------------------------------------
    sys.path.insert(1, './my_optimizers/navigation')
    import navigation

    solver = navigation.solver()
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
            print('solve_time_ms: \t%s' % result.solve_time_ms)
    else:
        raise AssertionError("No Solution")

    return result.solution


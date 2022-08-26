import opengen as og
from enum import Enum


class OcpInterfaceType(Enum):
    DIRECT = 1
    TCP = 2


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

    if print_result:
        print('cost: \t%s' % result.cost)
        print('exit_status: \t%s' % result.exit_status)
        print('f1_infeasibility: \t%s' % result.f1_infeasibility)
        print('f2_norm: \t%s' % result.f2_norm)
        print('num_inner_iterations: \t%s' % result.num_inner_iterations)
        print('num_outer_iterations: \t%s' % result.num_outer_iterations)
        print('penalty: \t%s' % result.penalty)
        print('solve_time_ms: \t%s' % result.solve_time_ms)

    return result.solution


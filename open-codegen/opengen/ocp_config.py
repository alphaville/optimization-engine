import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
import numpy as np
from ocp.optimal_control_problem import OptimalControlProblem, BallExclusionSet
from ocp.ocp_build_interface import OcpInterfaceType
from ocp.optimizer_formulations import FormulationType


# # -----------------------------------------------
# x_symb = cs.SX.sym('x', 6)
# u_symb = cs.SX.sym('u', 2)
# x_init = cs.SX.sym('xinit', 6)
# mass = cs.SX.sym('m', 1)
# p_symb = cs.vertcat((x_init, mass))
#
#
# def system_dynamics(x, u):
#     x_next = x + mass*u
#     pass
#
#
# def cost(x, u, p):
#     pass
#
#
# state_constraints = og.constraints.BallInf(radius=1)
# input_constraints = og.constraints.Ball2(radius=5)
# obstacle1 = og.constraints.Ball1(radius=1, center=[5, 5])
# obstacles = [obstacle1]
#
# problem = OptimalControlProblemBuilder(x_symb, u_symb, p_symb).with_cost(cost)\
#     .with_dynamics(x_symb + mass * u_symb)\
#     .with_obstacles(obstacles, state_idx=[2, 5, 9], mode=WITH_ALM)\
#     .with_state_constraints(state_constraints, mode=WITH_PM)\
#     .with_input_constraints(input_constraints)\
#     .with_formulation_type(SINGLE_SHOOTING, dynamics_mode=WITH_ALM)\
#     .build()
#
#
# meta = og.config.OptimizerMeta()
# solver_cfg = og.config.SolverConfiguration()
#
# builder = og.builder.OpEnOptimizerBuilder(problem,
#                                           meta,
#                                           build_config,
#                                           solver_config)

# -----------------------------------------------

(nx, nu, N) = (3, 2, 30)
(xref, yref, thetaref) = (1, 1, 0)

umin = [-3.0, -2.0]
umax = [3.0] * nu

xmin = [-np.Inf, -np.Inf, -0.6]*N
xmax = [np.Inf, np.Inf, 0.6]*N
# xmin = [-1000, -1000, -0.6]*N
# xmax = [1000, 1000, 0.6]*N

u_set = [og.constraints.Rectangle(umin, umax)]*N

x_set = og.constraints.Rectangle(xmin, xmax)
# x_set = og.constraints.NoConstraints()

formulation_type = FormulationType.SINGLE_SHOOTING

c1_centre = [0, 1.5]
c1_radius = 0.25
c1_type = og.constraints.Ball2()
c1_state_indices = [0, 1]
c1_obj = BallExclusionSet(c1_type, c1_centre, c1_radius, c1_state_indices)

exclusion_set = [c1_obj]

ocp_build_interface = OcpInterfaceType.DIRECT

ts = 0.03

def system_dynamics_function(x, u):
    L = 0.5
    x_tp1 = [None]*nx
    if fn.is_symbolic(x) or fn.is_symbolic(u):
        theta_dot = (1 / L) * (u[1] * cs.cos(x[2]) - u[0] * cs.sin(x[2]))
        x_tp1[0] = x[0] + ts * (u[0] + L * cs.sin(x[2]) * theta_dot)
        x_tp1[1] = x[1] + ts * (u[1] - L * cs.cos(x[2]) * theta_dot)
        x_tp1[2] = x[2] + ts * theta_dot
    else:
        theta_dot = (1 / L) * (u[1] * np.cos(x[2]) - u[0] * np.sin(x[2]))
        x_tp1[0] = x[0] + ts * (u[0] + L * np.sin(x[2]) * theta_dot)
        x_tp1[1] = x[1] + ts * (u[1] - L * np.cos(x[2]) * theta_dot)
        x_tp1[2] = x[2] + ts * theta_dot
    return x_tp1


def stage_cost_fn(x, u):
    (q, qtheta, r) = (10, 0.1, 1)
    stage_cost = q * ((x[0] - xref) ** 2 + (x[1] - yref) ** 2) + qtheta * (x[2] - thetaref) ** 2 + r * cs.dot(u, u)
    return stage_cost


def terminal_cost_fn(x, u):
    (qN, qthetaN) = (2000, 2)
    terminal_cost = qN*((x[-3]-xref)**2 + (x[-2]-yref)**2) + qthetaN*(x[-1]-thetaref)**2
    return terminal_cost


def create_OCP_obj():
    user_OCP = OptimalControlProblem(nx, nu, system_dynamics_function, stage_cost_fn, terminal_cost_fn)\
                .with_horizon(N)\
                .with_state_constraint(x_set)\
                .with_input_constraint(u_set)\
                .with_formulation_type(formulation_type)\
                .with_exclusion_set(exclusion_set)\
                .with_build_interface(ocp_build_interface)
    return user_OCP

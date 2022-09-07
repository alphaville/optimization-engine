import constraints
import casadi.casadi as cs
import opengen.functions as fn
import numpy as np
from ocp.optimal_control_problem import OptimalControlProblem
from ocp.ocp_builder import OCPBuilder
import matplotlib.pyplot as plt
from ocp.set_exclusion import ExclusionSet
from ocp.type_enums import *


# Build parametric optimizer
# ------------------------------------
(nx, nu, N) = (3, 2, 30)

umin = [-3.0, -2.0]
umax = [3.0] * nu

# xmin = [-np.Inf, -np.Inf, -0.6]*N
# xmax = [np.Inf, np.Inf, 0.6]*N
xmin = [-1000, -1000, -0.6]*N
xmax = [1000, 1000, 0.6]*N

u_set = [constraints.Rectangle(umin, umax)]*N

x_set = constraints.Rectangle(xmin, xmax)
# x_set = og.constraints.NoConstraints()

c1_centre = [0, 1.5]
c1_radius = 0.25
c1 = ExclusionSet(constraint=constraints.BallInf(center=c1_centre, radius=c1_radius), state_idx=[0, 1], mode=ConstraintMethod.PM)
# c1 = ExclusionSet(constraint=constraints.Rectangle(xmin=[-0.2, 1.3], xmax=[0.2, 1.7]), state_idx=[0, 1], mode=ConstraintMethod.PM)

exclusion_set_list = [c1]

x_init = cs.SX.sym('x_init', nx)
ts = cs.SX.sym('ts', 1)
L = cs.SX.sym('L', 1)
q = cs.SX.sym('q', 1)
qtheta = cs.SX.sym('qtheta', 1)
r = cs.SX.sym('r', 1)
qN = cs.SX.sym('qN', 1)
qthetaN = cs.SX.sym('qthetaN', 1)
x_ref = cs.SX.sym('x_ref', nx)
p_symb = cs.vertcat(x_init, ts, L, q, qtheta, r, qN, qthetaN, x_ref)

p_idx = {
    "x_init": 0,
    "ts": nx,
    "L": nx + 1,
    "q": nx + 2,
    "qtheta": nx + 3,
    "r": nx + 4,
    "qN": nx + 5,
    "qthetaN": nx + 6,
    "x_ref": nx + 7
}

def system_dynamics_fn(x, u, p_symb):
    x_tp1 = [None]*nx
    L = p_symb[p_idx["L"]]
    ts = p_symb[p_idx["ts"]]

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


def stage_cost_fn(x, u, p_symb):
    q = p_symb[p_idx["q"]]
    qtheta = p_symb[p_idx["qtheta"]]
    xref = p_symb[p_idx["x_ref"]]
    yref = p_symb[p_idx["x_ref"] + 1]
    thetaref = p_symb[p_idx["x_ref"] + 2]
    stage_cost = q * ((x[0] - xref) ** 2 + (x[1] - yref) ** 2) + qtheta * (x[2] - thetaref) ** 2 + r * cs.dot(u, u)
    return stage_cost


def terminal_cost_fn(x, p_symb):
    qN = p_symb[p_idx["qN"]]
    qthetaN = p_symb[p_idx["qthetaN"]]
    xref = p_symb[p_idx["x_ref"]]
    yref = p_symb[p_idx["x_ref"] + 1]
    thetaref = p_symb[p_idx["x_ref"] + 2]
    terminal_cost = qN*((x[-3]-xref)**2 + (x[-2]-yref)**2) + qthetaN*(x[-1]-thetaref)**2
    return terminal_cost


user_ocp = OptimalControlProblem(p_symb, nx, nu, system_dynamics_fn, stage_cost_fn, terminal_cost_fn) \
    .with_horizon(N) \
    .with_state_constraint(x_set) \
    .with_input_constraint(u_set) \
    .with_formulation_type(FormulationType.SINGLE_SHOOTING) \
    .with_exclusion_set(exclusion_set_list) \
    .with_build_interface(OcpInterfaceType.DIRECT)

builder = OCPBuilder(user_ocp)

builder.build()


def plot_solution(user_ocp, u_star, p_val):
    nx = user_ocp.nx
    nu = user_ocp.nu
    N = user_ocp.horizon

    ts = p_val[p_idx["ts"]]

    time = np.arange(0, ts * N, ts)
    ux = u_star[0:nu * N:nu]
    uy = u_star[1:nu * N:nu]

    # plt.subplot(211)
    # plt.plot(time, ux, '-o')
    # plt.ylabel('u_x')
    # plt.subplot(212)
    # plt.plot(time, uy, '-o')
    # plt.ylabel('u_y')
    # plt.xlabel('Time')
    # plt.show()

    z_star = [None] * (nx * (N + 1))
    z_star[:nx] = p_val[:nx]

    if user_ocp.formulation_type is FormulationType.MULTIPLE_SHOOTING:
        z_star[nx:] = u_star[nu * N:]
    elif user_ocp.formulation_type is FormulationType.SINGLE_SHOOTING:
        for t in range(N):
            z_star[nx * (t + 1):nx * (t + 2)] = user_ocp.sys_dyn_fn(z_star[nx * t:nx * (t + 1)],
                                                                    u_star[nu * t:nu * (t + 1)], p_val)

    # plot system states x, y and theta
    time = np.arange(0, ts * (N + 1), ts)
    zx = z_star[0:nx * (N + 1):nx]
    zy = z_star[1:nx * (N + 1):nx]
    ztheta = z_star[2:nx * (N + 1):nx]

    # plt.subplot(311)
    # plt.plot(time, zx, '-o')
    # plt.ylabel('z_x')
    # plt.subplot(312)
    # plt.plot(time, zy, '-o')
    # plt.ylabel('z_y')
    # plt.subplot(313)
    # plt.plot(time, ztheta, '-o')
    # plt.ylabel('z_theta')
    # plt.xlabel('Time')
    # plt.show()

    plt.plot(zx, zy, '-o')
    plt.ylabel('y')
    plt.xlabel('x')

    # t = np.linspace(0, 6.3, 120)
    # x_circle = c1_centre[0] + c1_radius * np.sin(t)
    # y_circle = c1_centre[1] + c1_radius * np.cos(t)
    # plt.plot(x_circle, y_circle)

    xp = [c1_centre[0] - c1_radius, c1_centre[0] + c1_radius]
    yp = [c1_centre[1] - c1_radius, c1_centre[1] + c1_radius]
    x = np.concatenate((np.linspace(xp[0], xp[1], 120), np.array([xp[1]] * 120), np.linspace(xp[1], xp[0], 120), np.array([xp[0]] * 120)))
    y = np.concatenate((np.array([yp[1]] * 120), np.linspace(yp[1], yp[0], 120), np.array([yp[0]] * 120), np.linspace(yp[0], yp[1], 120)))
    plt.plot(x, y)

    plt.arrow(zx[0], zy[0], np.cos(ztheta[0]) * 0.1, np.sin(ztheta[0]) * 0.1, head_width=.05, color=(0.8, 0, 0.2))
    plt.arrow(zx[-1], zy[-1], np.cos(ztheta[-1]) * 0.1, np.sin(ztheta[-1]) * 0.1, head_width=.05, color=(0.8, 0, 0.2))

    plt.show()

    # print(f"z_theta minimum:{min(ztheta)}")


x_init = [-1.0, 2.0, 0.0]
ts = 0.03
L = 0.5
(q, qtheta, r) = (10, 0.1, 1)
(qN, qthetaN) = (2000, 200)
(xref, yref, thetaref) = (1, 1, 0)
p_val = x_init + [ts, L, q, qtheta, r, qN, qthetaN, xref, yref, thetaref]

u_star = builder.solve(p_val, print_result=True)
plot_solution(user_ocp, u_star, p_val)


# ts = 0.05
# p_val = x_init + [ts, L, q, qtheta, r, qN, qthetaN, xref, yref, thetaref]
#
# u_star = user_ocp.solve(p_val, print_result=True)
# plot_solution(user_ocp, u_star, p_val)
#
#
# (xref, yref, thetaref) = (1, 0, 0)
# p_val = x_init + [ts, L, q, qtheta, r, qN, qthetaN, xref, yref, thetaref]
#
# u_star = user_ocp.solve(p_val, print_result=True)
# plot_solution(user_ocp, u_star, p_val)


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
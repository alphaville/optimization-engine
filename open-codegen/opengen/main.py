import constraints
import casadi.casadi as cs
import opengen.functions as fn
import numpy as np
from ocp.optimal_control_problem import OptimalControlProblem
from ocp.ocp_builder import OCPBuilder
import matplotlib.pyplot as plt
from ocp.set_exclusion import ExclusionSet
from ocp.type_enums import *
import math


# Build parametric optimizer
# ------------------------------------
(nx, nu, N) = (4, 2, 35)

pi = round(math.pi, 2)

umin = [-3.0, -pi/6]
umax = [3.0, pi/6]

xmin = [-1000, -1000, -2*pi, -5.0]*N
xmax = [1000, 1000, 2*pi, 5.0]*N

u_set = [constraints.Rectangle(umin, umax)]*N

x_set = constraints.Rectangle(xmin, xmax)
# x_set = og.constraints.NoConstraints()

c1_centre = [-1.5, 1.5]
c1_radius = 0.5
c1 = ExclusionSet(constraint=constraints.Ball2(center=c1_centre, radius=c1_radius), state_idx=[0, 1], mode=ConstraintMethod.PM)
# c1 = ExclusionSet(constraint=constraints.BallInf(center=c1_centre, radius=c1_radius), state_idx=[0, 1], mode=ConstraintMethod.PM)
# c1 = ExclusionSet(constraint=constraints.Rectangle(xmin=[-0.2, 1.3], xmax=[0.2, 1.7]), state_idx=[0, 1], mode=ConstraintMethod.PM)

exclusion_set_list = [c1]

x_init = cs.SX.sym('x_init', nx)
ts = cs.SX.sym('ts', 1)
L = cs.SX.sym('L', 1)
alpha = cs.SX.sym('alpha', 1)
qp = cs.SX.sym('qp', 1)
qtheta = cs.SX.sym('qtheta', 1)
qv = cs.SX.sym('qv', 1)
ra = cs.SX.sym('ra', 1)
rdelta = cs.SX.sym('rdelta', 1)
qpN = cs.SX.sym('qpN', 1)
qthetaN = cs.SX.sym('qthetaN', 1)
qvN = cs.SX.sym('qvN', 1)
x_ref = cs.SX.sym('x_ref', nx)
p_symb = cs.vertcat(x_init, ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, x_ref)

p_idx = {
    "x_init": 0,
    "ts": nx,
    "L": nx + 1,
    "alpha": nx + 2,
    "qp": nx + 3,
    "qtheta": nx + 4,
    "qv": nx + 5,
    "ra": nx + 6,
    "rdelta": nx + 7,
    "qpN": nx + 8,
    "qthetaN": nx + 9,
    "qvN": nx + 10,
    "x_ref": nx + 11
}

u_previous = [0]*nu

def system_dynamics_fn(x, u, p_symb):
    x_tp1 = [None]*nx
    L = p_symb[p_idx["L"]]
    ts = p_symb[p_idx["ts"]]
    alpha = p_symb[p_idx["alpha"]]

    if fn.is_symbolic(x) or fn.is_symbolic(u):
        x_tp1[0] = x[0] + ts * (x[3] * cs.cos(x[2]))
        x_tp1[1] = x[1] + ts * (x[3] * cs.sin(x[2]))
        x_tp1[2] = x[2] + ts * ((x[3] * cs.tan(u[1]))/L)
        x_tp1[3] = x[3] + ts * (alpha * (u[0] - x[3]))
    else:
        x_tp1[0] = x[0] + ts * (x[3] * np.cos(x[2]))
        x_tp1[1] = x[1] + ts * (x[3] * np.sin(x[2]))
        x_tp1[2] = x[2] + ts * ((x[3] * np.tan(u[1])) / L)
        x_tp1[3] = x[3] + ts * (alpha * (u[0] - x[3]))
    return x_tp1


def stage_cost_fn(x, u, p_symb):
    qp = p_symb[p_idx["qp"]]
    qtheta = p_symb[p_idx["qtheta"]]
    qv = p_symb[p_idx["qv"]]
    ra = p_symb[p_idx["ra"]]
    rdelta = p_symb[p_idx["rdelta"]]
    xref = p_symb[p_idx["x_ref"]]
    yref = p_symb[p_idx["x_ref"] + 1]
    thetaref = p_symb[p_idx["x_ref"] + 2]
    vref = p_symb[p_idx["x_ref"] + 3]
    global u_previous

    stage_cost = qp * ((x[0] - xref) ** 2 + (x[1] - yref) ** 2) \
                 + qtheta * (x[2] - thetaref) ** 2 \
                 + qv * (x[3] - vref) ** 2 \
                 + ra * (u[0] - u_previous[0]) ** 2 \
                 + rdelta * (u[1] - u_previous[1]) ** 2
    u_previous = u

    return stage_cost


def terminal_cost_fn(x, p_symb):
    qpN = p_symb[p_idx["qpN"]]
    qthetaN = p_symb[p_idx["qthetaN"]]
    qvN = p_symb[p_idx["qvN"]]
    xref = p_symb[p_idx["x_ref"]]
    yref = p_symb[p_idx["x_ref"] + 1]
    thetaref = p_symb[p_idx["x_ref"] + 2]
    vref = p_symb[p_idx["x_ref"] + 3]

    terminal_cost = qpN*((x[0]-xref)**2 + (x[1]-yref)**2) \
                    + qthetaN*(x[2]-thetaref)**2 \
                    + qvN*(x[3]-vref)**2

    return terminal_cost


user_ocp = OptimalControlProblem(p_symb, nx, nu, system_dynamics_fn, stage_cost_fn, terminal_cost_fn) \
    .with_horizon(N)\
    .with_formulation_type(FormulationType.MULTIPLE_SHOOTING)\
    .with_input_constraint(u_set)\
    .with_state_constraint(x_set)\
    .with_exclusion_set(exclusion_set_list)

builder = OCPBuilder(user_ocp)\
            .with_build_interface(OcpInterfaceType.DIRECT)

builder.build()


def plot_solution(user_ocp, u_star, p_val):
    nx = user_ocp.nx
    nu = user_ocp.nu
    N = user_ocp.horizon

    ts = p_val[p_idx["ts"]]
    t_max = round(ts * N, 3)
    time = np.arange(0, t_max, ts)
    ux = u_star[0:nu * N:nu]
    uy = u_star[1:nu * N:nu]

    plt.subplot(211)
    plt.plot(time, ux, '-o')
    plt.ylabel('acceleration')
    plt.subplot(212)
    plt.plot(time, uy, '-o')
    plt.ylabel('delta')
    plt.xlabel('Time')
    plt.show()

    z_star = [None] * (nx * (N + 1))
    z_star[:nx] = p_val[:nx]

    if user_ocp.formulation_type is FormulationType.MULTIPLE_SHOOTING:
        z_star[nx:] = u_star[nu * N:]
    elif user_ocp.formulation_type is FormulationType.SINGLE_SHOOTING:
        for t in range(N):
            z_star[nx * (t + 1):nx * (t + 2)] = user_ocp.sys_dyn_fn(z_star[nx * t:nx * (t + 1)],
                                                                    u_star[nu * t:nu * (t + 1)], p_val)

    # plot system states x, y and theta
    t_max = round(ts * (N + 1), 3)
    time = np.arange(0, t_max, ts)
    zx = z_star[0:nx * (N + 1):nx]
    zy = z_star[1:nx * (N + 1):nx]
    ztheta = z_star[2:nx * (N + 1):nx]
    zv = z_star[3:nx * (N + 1):nx]

    plt.subplot(211)
    plt.plot(time, zv, '-o')
    plt.ylabel('velocity')
    plt.subplot(212)
    plt.plot(time, ztheta, '-o')
    plt.ylabel('theta')
    plt.xlabel('Time')
    plt.show()

    plt.plot(zx, zy, '-o')
    plt.ylabel('y')
    plt.xlabel('x')

    t = np.linspace(0, 2*pi, 120)
    x_circle = c1_centre[0] + c1_radius * np.sin(t)
    y_circle = c1_centre[1] + c1_radius * np.cos(t)
    plt.plot(x_circle, y_circle)

    # xp = [c1_centre[0] - c1_radius, c1_centre[0] + c1_radius]
    # yp = [c1_centre[1] - c1_radius, c1_centre[1] + c1_radius]
    # x = np.concatenate((np.linspace(xp[0], xp[1], 120), np.array([xp[1]] * 120), np.linspace(xp[1], xp[0], 120), np.array([xp[0]] * 120)))
    # y = np.concatenate((np.array([yp[1]] * 120), np.linspace(yp[1], yp[0], 120), np.array([yp[0]] * 120), np.linspace(yp[0], yp[1], 120)))
    # plt.plot(x, y)

    plt.arrow(zx[0], zy[0], np.cos(ztheta[0]) * 0.2, np.sin(ztheta[0]) * 0.1, head_width=.1, color=(0.8, 0, 0.2))
    plt.arrow(zx[-1], zy[-1], np.cos(ztheta[-1]) * 0.2, np.sin(ztheta[-1]) * 0.1, head_width=.1, color=(0.8, 0, 0.2))

    plt.show()

    # print(f"z_theta minimum:{min(ztheta)}")


x_init = [-3.0, 3.0, 0.0, 1.0]
ts = 0.2
L = 2.5
alpha = 0.25
(qp, qtheta, qv, ra, rdelta) = (18, 2, 5, 100, 30)
(qpN, qthetaN, qvN) = (3000, 1000, 100)
(xref, yref, thetaref, vref) = (0, 0, 0, 0)
p_val = x_init + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, xref, yref, thetaref, vref]

# u_star = builder.solve(p_val, print_result=True)
# plot_solution(user_ocp, u_star, p_val)


(qp, qtheta, qv, ra, rdelta) = (30, 2, 10, 30, 30)
(qpN, qthetaN, qvN) = (2000, 1000, 3000)
p_val = x_init + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, xref, yref, thetaref, vref]
u_star = builder.solve(p_val, print_result=True)
plot_solution(user_ocp, u_star, p_val)


(qp, qtheta, qv, ra, rdelta) = (30, 5, 5, 20, 20)
(qpN, qthetaN, qvN) = (1000, 3000, 1000)
(xref, yref, thetaref, vref) = (0, 0, -pi/2, 0)
p_val = x_init + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, xref, yref, thetaref, vref]
u_star = builder.solve(p_val, print_result=True)
plot_solution(user_ocp, u_star, p_val)
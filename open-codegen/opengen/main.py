import opengen.constraints as constraints
import casadi.casadi as cs
import opengen.functions as fn
import opengen.config as cfg
import numpy as np
from ocp.optimal_control_problem import OptimalControlProblem
from ocp.ocp_builder import OCPBuilder
import matplotlib.pyplot as plt
from ocp.set_exclusion import ExclusionSet
from ocp.type_enums import *
import math
import sys

def plot_2D_circle(centre, radius):
    t = np.linspace(0, 2 * pi, 120)
    x_circle = centre[0] + radius * np.sin(t)
    y_circle = centre[1] + radius * np.cos(t)
    plt.plot(x_circle, y_circle)

def plot_2D_ballinf(centre, radius):
    xp = [centre[0] - radius, centre[0] + radius]
    yp = [centre[1] - radius, centre[1] + radius]
    x = np.concatenate((np.linspace(xp[0], xp[1], 120), np.array([xp[1]] * 120), np.linspace(xp[1], xp[0], 120), np.array([xp[0]] * 120)))
    y = np.concatenate((np.array([yp[1]] * 120), np.linspace(yp[1], yp[0], 120), np.array([yp[0]] * 120), np.linspace(yp[0], yp[1], 120)))
    plt.plot(x, y)

def plot_2D_box(xmin, xmax):
    xp = [xmin[0], xmax[0]]
    yp = [xmin[1], xmax[1]]
    x = np.concatenate((np.linspace(xp[0], xp[1], 120), np.array([xp[1]] * 120), np.linspace(xp[1], xp[0], 120), np.array([xp[0]] * 120)))
    y = np.concatenate((np.array([yp[1]] * 120), np.linspace(yp[1], yp[0], 120), np.array([yp[0]] * 120), np.linspace(yp[0], yp[1], 120)))
    plt.plot(x, y)

# Build parametric optimizer
# ------------------------------------
(nx, nu, N) = (4, 2, 30)

pi = round(math.pi, 2)

umin = [-3.0, -pi/6]
umax = [3.0, pi/6]

xmin = [-1000, -1000, -2*pi, -5.0]
xmax = [1000, 1000, 2*pi, 5.0]

u_set = [constraints.Rectangle(umin, umax)]*N

x_set = [constraints.Rectangle(xmin, xmax)]*N

def testcase_input_1():
    x_init_val = [-8.0, 3.0, 0.0, 1.0]
    x_ref_val = [0, 0, -pi/2, 0]

    c1_centre = [-1.5, 1.5]
    c1_radius = 0.5

    b1_centre = [-3.5, 0]
    b1_radius = 0.8

    b2_centre = [3.5, 0]
    b2_radius = 0.8

    return x_init_val, x_ref_val, c1_centre, c1_radius, b1_centre, b1_radius, b2_centre, b2_radius


def testcase_input_2():
    x_init_val = [-8.0, 4.0, 0.0, 1.0]
    x_ref_val = [0, 0, 0, 0]

    c1_centre = [-2.5, 2]
    c1_radius = 1

    b1_xmin=[-7, -1]
    b1_xmax=[-3, 1]

    b2_xmin=[3, -1]
    b2_xmax=[7, 1]

    return x_init_val, x_ref_val, c1_centre, c1_radius, b1_xmin, b1_xmax, b2_xmin, b2_xmax


def testcase_input_3():
    x_init_val = [-8.0, 5.0, 0.0, 1.0]
    x_ref_val = [0, 0, -pi/2, 0]

    c1_centre = [-1, 2]
    c1_radius = 1

    b1_xmin=[-5, -2]
    b1_xmax=[-3, 2]

    b2_xmin=[2, -2]
    b2_xmax=[4, 2]

    return x_init_val, x_ref_val, c1_centre, c1_radius, b1_xmin, b1_xmax, b2_xmin, b2_xmax

formulation = FormulationType.MULTIPLE_SHOOTING

solver_config = cfg.SolverConfiguration() \
                .with_preconditioning(True) \
                .with_optimized_initial_penalty(True)

# [x_init_val, x_ref_val, c1_centre, c1_radius, s1_centre, s1_radius, s2_centre, s2_radius, formulation] = testcase_input_1()
# [x_init_val, x_ref_val, c1_centre, c1_radius, b1_xmin, b1_xmax, b2_xmin, b2_xmax] = testcase_input_2()
[x_init_val, x_ref_val, c1_centre, c1_radius, b1_xmin, b1_xmax, b2_xmin, b2_xmax] = testcase_input_3()

test_name = (".\output\T3_Multiple_shooting_" if formulation is FormulationType.MULTIPLE_SHOOTING else ".\output\T3_Single_shooting_") + \
    "Pre_" + str(solver_config.preconditioning) + "_Init_" +str(solver_config.optimize_initial_penalty)
figure_name = test_name + ".png"
file_name = test_name + ".txt"
# file_name = ".\output\output_log.txt"

c1 = ExclusionSet(constraint=constraints.Ball2(center=c1_centre, radius=c1_radius), state_idx=[0, 1])
# b1 = ExclusionSet(constraint=constraints.BallInf(center=s1_centre, radius=s1_radius), state_idx=[0, 1])
# b2 = ExclusionSet(constraint=constraints.BallInf(center=s2_centre, radius=s2_radius), state_idx=[0, 1])
b1 = ExclusionSet(constraint=constraints.Rectangle(b1_xmin, b1_xmax), state_idx=[0, 1])
b2 = ExclusionSet(constraint=constraints.Rectangle(b2_xmin, b2_xmax), state_idx=[0, 1])

exclusion_set_list = [c1, b1, b2]

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
    .with_formulation_type(formulation)\
    .with_input_constraint(u_set)\
    .with_state_constraint(x_set)\
    .with_exclusion_set(exclusion_set_list)

builder = OCPBuilder(user_ocp)\
            .with_build_interface(OcpInterfaceType.DIRECT) \
            .with_solver_config(solver_config)

builder.build()

def plot_solution(user_ocp, u_star, p_val):
    nx = user_ocp.nx
    nu = user_ocp.nu
    N = user_ocp.horizon

    ts = p_val[p_idx["ts"]]
    t_max = round(ts * (N-1), 3)
    time = np.linspace(0, t_max, N)
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
    t_max = round(ts * N, 3)
    time = np.linspace(0, t_max, (N + 1))
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

    plot_2D_circle(c1_centre, c1_radius)
    # plot_2D_ballinf(s1_centre, s1_radius)
    # plot_2D_ballinf(s2_centre, s2_radius)
    plot_2D_box(b1_xmin, b1_xmax)
    plot_2D_box(b2_xmin, b2_xmax)

    plt.arrow(zx[0], zy[0], np.cos(ztheta[0]) * 0.2, np.sin(ztheta[0]) * 0.2, head_width=.5, color=(0.8, 0, 0.2))
    plt.arrow(zx[-1], zy[-1], np.cos(ztheta[-1]) * 0.2, np.sin(ztheta[-1]) * 0.2, head_width=.5, color=(0.8, 0, 0.2))

    plt.xlim([-8.5,7.5])
    plt.ylim([-3,6])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.savefig(figure_name, bbox_inches='tight')
    plt.show()


ts = 0.5
L = 2.5
alpha = 0.25
# (qp, qtheta, qv, ra, rdelta) = (18, 2, 5, 100, 30)
# (qpN, qthetaN, qvN) = (3000, 1000, 100)
# # (xref, yref, thetaref, vref) = (0, 0, 0, 0)
# # p_val = x_init_val + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, xref, yref, thetaref, vref]

# u_star = builder.solve(p_val, print_result=True)
# plot_solution(user_ocp, u_star, p_val)


(qp, qtheta, qv, ra, rdelta) = (18, 2, 5, 10, 30)
(qpN, qthetaN, qvN) = (3000, 5000, 10)
p_val = x_init_val + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN] + x_ref_val

original_stdout = sys.stdout
with open(file_name, 'a') as f:
    sys.stdout = f

    if x_ref_val[2] == 0:
        print('\nTESTCASE 2')
    elif x_ref_val[2] == -pi/2:
        print('\nTESTCASE 3')
    print('Formulation: Multiple Shooting') if formulation is FormulationType.MULTIPLE_SHOOTING \
                                            else print('Formulation: Single Shooting')
    print('Preconditioning: ', solver_config.preconditioning)
    print('Optimized Initial Penalty: ', solver_config.optimize_initial_penalty)

    u_star = builder.solve(p_val, print_result=True)
    print('\n')
    sys.stdout = original_stdout
plot_solution(user_ocp, u_star, p_val)

#
# (qp, qtheta, qv, ra, rdelta) = (30, 5, 5, 20, 20)
# (qpN, qthetaN, qvN) = (1000, 3000, 1000)
# (xref, yref, thetaref, vref) = (0, 0, -pi/2, 0)
# p_val = x_init + [ts, L, alpha, qp, qtheta, qv, ra, rdelta, qpN, qthetaN, qvN, xref, yref, thetaref, vref]
# u_star = builder.solve(p_val, print_result=True)
# plot_solution(user_ocp, u_star, p_val)
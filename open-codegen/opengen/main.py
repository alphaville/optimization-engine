import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
import numpy as np
from ocp.optimizer_formulations import FormulationType
from ocp.optimal_control_problem import OptimalControlProblem, BallExclusionSet
from ocp.ocp_build_interface import OcpInterfaceType
import matplotlib.pyplot as plt


# Build parametric optimizer
# ------------------------------------
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

x_init = cs.SX.sym('x_init', nx)
ts = cs.SX.sym('ts', 1)
L = cs.SX.sym('L', 1)
q = cs.SX.sym('q', 1)
qtheta = cs.SX.sym('qtheta', 1)
r = cs.SX.sym('r', 1)
qN = cs.SX.sym('qN', 1)
qthetaN = cs.SX.sym('qthetaN', 1)
p_symb = cs.vertcat(x_init, ts, L, q, qtheta, r, qN, qthetaN)


def system_dynamics_function(x, u, p_symb):
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


def stage_cost_fn(x, u, p_symb):
    stage_cost = q * ((x[0] - xref) ** 2 + (x[1] - yref) ** 2) + qtheta * (x[2] - thetaref) ** 2 + r * cs.dot(u, u)
    return stage_cost


def terminal_cost_fn(x, u, p_symb):
    terminal_cost = qN*((x[-3]-xref)**2 + (x[-2]-yref)**2) + qthetaN*(x[-1]-thetaref)**2
    return terminal_cost


user_ocp = OptimalControlProblem(p_symb, nx, nu, system_dynamics_function, stage_cost_fn, terminal_cost_fn) \
    .with_horizon(N) \
    .with_state_constraint(x_set) \
    .with_input_constraint(u_set) \
    .with_formulation_type(formulation_type) \
    .with_exclusion_set(exclusion_set) \
    .with_build_interface(ocp_build_interface)

user_ocp.build()


def plot_solution(user_ocp, u_star, p_val):
    nx = user_ocp.get_nx()
    nu = user_ocp.get_nu()
    N = user_ocp.get_horizon()

    p_idx = {
        "x_init": 0,
        "ts": nx,
        "L": nx + 1,
        "q": nx + 2,
        "qtheta": nx + 4,
        "r": nx + 5,
        "qN": nx + 6,
        "qthetaN": nx + 7
    }

    ts = p_val[p_idx["ts"]]

    time = np.arange(0, ts * N, ts)
    ux = u_star[0:nu * N:nu]
    uy = u_star[1:nu * N:nu]

    plt.subplot(211)
    plt.plot(time, ux, '-o')
    plt.ylabel('u_x')
    plt.subplot(212)
    plt.plot(time, uy, '-o')
    plt.ylabel('u_y')
    plt.xlabel('Time')
    plt.show()

    z_star = [None] * (nx * (N + 1))
    z_star[:nx] = p_val[:nx]

    if user_ocp.get_formulation_type() is FormulationType.MULTIPLE_SHOOTING:
        z_star[nx:] = u_star[nu * N:]
    elif user_ocp.get_formulation_type() is FormulationType.SINGLE_SHOOTING:
        for t in range(N):
            z_star[nx * (t + 1):nx * (t + 2)] = user_ocp.sys_dyn_fn(z_star[nx * t:nx * (t + 1)],
                                                                    u_star[nu * t:nu * (t + 1)], p_val)

    # plot system states x, y and theta
    time = np.arange(0, ts * (N + 1), ts)
    zx = z_star[0:nx * (N + 1):nx]
    zy = z_star[1:nx * (N + 1):nx]
    ztheta = z_star[2:nx * (N + 1):nx]

    plt.subplot(311)
    plt.plot(time, zx, '-o')
    plt.ylabel('z_x')
    plt.subplot(312)
    plt.plot(time, zy, '-o')
    plt.ylabel('z_y')
    plt.subplot(313)
    plt.plot(time, ztheta, '-o')
    plt.ylabel('z_theta')
    plt.xlabel('Time')
    plt.show()

    plt.plot(zx, zy, '-o')
    plt.ylabel('y')
    plt.xlabel('x')

    t = np.linspace(0, 6.3, 120)
    x_circle = c1_centre[0] + c1_radius * np.sin(t)
    y_circle = c1_centre[1] + c1_radius * np.cos(t)
    plt.plot(x_circle, y_circle)
    plt.show()

    # print(f"z_theta minimum:{min(ztheta)}")


x_init = [-1.0, 2.0, 0.0]
ts = 0.03
L = 0.5
(q, qtheta, r) = (10, 0.1, 1)
(qN, qthetaN) = (2000, 2)
p_val = x_init + [ts, L, q, qtheta, r, qN, qthetaN]

u_star = user_ocp.solve(p_val, print_result=True)
plot_solution(user_ocp, u_star, p_val)


ts = 0.05
p_val = x_init + [ts, L, q, qtheta, r, qN, qthetaN]

u_star = user_ocp.solve(p_val, print_result=True)
plot_solution(user_ocp, u_star, p_val)


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
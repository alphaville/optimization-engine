import opengen as og
import casadi.casadi as cs
import opengen.functions as fn
import matplotlib.pyplot as plt
import numpy as np
from ocp.optimizer_formulations import FormulationType
from ocp_config import create_OCP_obj
from ocp.ocp_build_interface import OcpInterfaceType, tcp_interface, direct_interface

# Build parametric optimizer
# ------------------------------------
user_ocp = create_OCP_obj()

nx = user_ocp.get_nx()
nu = user_ocp.get_nu()
N = user_ocp.get_horizon()

u = cs.SX.sym('u', nu * N)
x = cs.SX.sym('x', nx * (N + 1))

ocp_build_interface = user_ocp.get_ocp_build_interface()

(cost, decision_var, bounds, alm_mapping, alm_set, pm_constraints) = user_ocp.problem_formulation(user_ocp, u, nu, x, nx, N)

problem = og.builder.Problem(decision_var, x[0:nx], cost)\
    .with_constraints(bounds)\

if fn.is_symbolic(alm_mapping):
    problem.with_aug_lagrangian_constraints(alm_mapping, alm_set)

if fn.is_symbolic(pm_constraints):
    problem.with_penalty_constraints(pm_constraints)

user_ocp.save_OCP(problem)

build_config = og.config.BuildConfiguration()\
    .with_build_mode("debug")\
    .with_build_python_bindings()

if ocp_build_interface is OcpInterfaceType.TCP:
    build_config.with_build_directory("my_optimizers_tcp")
    build_config.with_tcp_interface_config()
else:
    build_config.with_build_directory("my_optimizers")
    # build_config.with_build_c_bindings()

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("navigation")
solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-3)\
    .with_initial_tolerance(1e-2)\
    .with_initial_penalty(1000)\
    .with_penalty_weight_update_factor(1.25)\
    .with_max_outer_iterations(65)\
    .with_max_inner_iterations(2000)\
    .with_delta_tolerance(1e-3)
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()

z_initial = [-1.0, 2.0, 0.0]

if ocp_build_interface is OcpInterfaceType.TCP:
    ocp_build_function = tcp_interface
else:
    ocp_build_function = direct_interface

u_star = ocp_build_function(z_initial, print_result=True)

# Plot solution
# ------------------------------------
from ocp_config import ts, c1_centre, c1_radius

time = np.arange(0, ts*N, ts)
ux = u_star[0:nu*N:nu]
uy = u_star[1:nu*N:nu]

plt.subplot(211)
plt.plot(time, ux, '-o')
plt.ylabel('u_x')
plt.subplot(212)
plt.plot(time, uy, '-o')
plt.ylabel('u_y')
plt.xlabel('Time')
plt.show()

z_star = [None] * (nx*(N+1))
z_star[0:nx] = z_initial[:]

if user_ocp.get_formulation_type() is FormulationType.MULTIPLE_SHOOTING:
    z_star[nx:] = u_star[nu*N:]
elif user_ocp.get_formulation_type() is FormulationType.SINGLE_SHOOTING:
    for t in range(N):
        z_star[nx*(t+1):nx*(t+2)] = user_ocp.sys_dyn_fn(z_star[nx*t:nx*(t+1)], u_star[nu*t:nu*(t+1)])

# plot system states x, y and theta
time = np.arange(0, ts*(N+1), ts)
zx = z_star[0:nx*(N+1):nx]
zy = z_star[1:nx*(N+1):nx]
ztheta = z_star[2:nx*(N+1):nx]

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

print(f"z_theta minimum:{min(ztheta)}")

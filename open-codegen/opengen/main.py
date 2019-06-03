import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Build parametric optimizer
# ---------------------------------------------------------------------------
(nu, nx, N) = (2, 3, 60)
L = 0.5
ts = 0.05
(xref , yref, thetaref) = (2, 2, 0)
(q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx)

(x, y, theta) = (z0[0], z0[1], z0[2])
cost = 0
c = 0
for t in range(0, N):
    cost += q * ((x - xref)**2 + (y - yref)**2) + qtheta * (theta - thetaref)**2
    u_t = u[t*nu:(t+1)*nu]
    theta_dot = (1/L) * (u_t[1] * cs.cos(theta) - u_t[0] * cs.sin(theta))
    cost += r * cs.dot(u_t, u_t)
    x += ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
    y += ts * (u_t[1] - L * cs.cos(theta) * theta_dot)
    c += cs.fmax(0, 1 - x**2 - y**2)
    theta += ts * theta_dot

cost = cost + qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

umin = [-5.0] * (nu*N)
umax = [5.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost).with_constraints(bounds).with_penalty_constraints(c)
build_config = og.config.BuildConfiguration() \
    .with_build_directory(".python_test_build") \
    .with_build_mode("release")
meta = og.config.OptimizerMeta() \
    .with_optimizer_name("navigation")
solver_config = og.config.SolverConfiguration().with_tolerance(1e-4)\
    .with_constraints_tolerance(1e-2) \
    .with_max_outer_iterations(5)     \
    .with_penalty_weight_update_factor(10.0)\
    .with_initial_penalty_weights(100.0)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
    .with_verbosity_level(1)
builder.enable_tcp_interface()
builder.build()


# ---------------------------------------------------------------------------
# Use TCP server
# ---------------------------------------------------------------------------
mng = og.tcp.OptimizerTcpManager('.python_test_build/navigation')
mng.start()

x_init = [-2.0, -2.0, 0.0]
mng.ping()
solution = mng.call(x_init, initial_guess=[1.0] * (nu*N))
print(solution['exit_status'])
print(solution['solve_time_ms'])
mng.kill()

# ---------------------------------------------------------------------------
# Plot solution
# ---------------------------------------------------------------------------
time = np.arange(0, ts*N, ts)
u_star = solution['solution']
ux = u_star[0:nu*N:2]
uy = u_star[1:nu*N:2]

plt.subplot(211)
plt.plot(time, ux, '-o')
plt.ylabel('u_x')
plt.subplot(212)
plt.plot(time, uy, '-o')
plt.ylabel('u_y')
plt.xlabel('Time')
plt.show()

# ---------------------------------------------------------------------------
# Plot trajectory
# ---------------------------------------------------------------------------
x_states = [0.0] * (nx*(N+2))
x_states[0:nx+1] = x_init
for t in range(0, N):
    u_t = u_star[t*nu:(t+1)*nu]

    x = x_states[t * nx]
    y = x_states[t * nx + 1]
    theta = x_states[t * nx + 2]

    theta_dot = (1/L) * (u_t[1] * np.cos(theta) - u_t[0] * np.sin(theta))

    x_states[(t + 1) * nx] = x + ts * (u_t[0] + L * np.sin(theta) * theta_dot)
    x_states[(t + 1) * nx + 1] = y + ts * (u_t[1] - L * np.cos(theta) * theta_dot)
    x_states[(t + 1) * nx + 2] = theta + ts * theta_dot

xx = x_states[0:nx*N:nx]
xy = x_states[1:nx*N:nx]

print(x_states)
print(xx)

t = np.arange(0, 6.4, 0.1)
x_circ = np.sin(t)
y_circ = np.cos(t)
plt.plot(x_circ, y_circ, 'r-')

plt.plot(xx, xy, '-o')
plt.ylabel('y')
plt.xlabel('x')
plt.show()


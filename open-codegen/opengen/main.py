import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------------
# Build parametric optimizer
# ---------------------------------------------------------------------------
nu = 2
N = 20
nx = 3
L = 0.5
ts = 0.1
xref = 1
yref = 1
thetaref = 0

q = 20
qtheta = 0.1
r = 1
qN = 10*q
qthetaN = 10*qtheta

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx)

x = z0[0]
y = z0[1]
theta = z0[2]
cost = 0

for t in range(0, nu*N, nu):
    cost += q * ((x - xref)**2 + (y - yref)**2) + qtheta * (theta - thetaref)**2
    u_t = u[t:t+nu]
    theta_dot = (1/L) * (u_t[1] * cs.cos(theta) - u_t[0] * cs.sin(theta))
    cost += r * cs.dot(u_t, u_t)
    x += ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
    y += ts * (u_t[1] - L * cs.cos(theta) * theta_dot)
    theta += ts * theta_dot

cost = cost + qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

umin = [-3.0] * (nu*N)
umax = [3.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
build_config = og.config.BuildConfiguration() \
    .with_build_directory(".python_test_build") \
    .with_build_mode("debug")
meta = og.config.OptimizerMeta() \
    .with_optimizer_name("navigation")
solver_config = og.config.SolverConfiguration().with_tolerance(1e-5)
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config) \
    .with_verbosity_level(1)
builder.enable_tcp_interface()
builder.build()


# ---------------------------------------------------------------------------
# Use TCP server
# ---------------------------------------------------------------------------
mng = og.tcp.OptimizerTcpManager('.python_test_build/navigation')
mng.start()

mng.ping()
solution = mng.call([-1.0, 2.0, 0.0], initial_guess=[1.0] * (nu*N))
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


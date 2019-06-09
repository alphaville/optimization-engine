---
id: python-examples
title: Examples
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


## Rosenbrock function
This is the example we presented in the previous section. You can copy
it and experiment. The problem we need to solve is:

<div class="math">
\[
    \begin{align}
    \operatorname*{Minimize}_{\|u\|\leq r}& \sum_{i=1}^{n_u - 1} b (u_{i+1} - u_{i}^2)^2 + (a-u_i)^2
    \\
    \text{subject to: }& 1.5 u_1 - u_2 = 0
    \\
    &u_3 - u_4 + 0.1 \leq 0
    \end{align}
\]</div>

The parameter vector is $p=(a, b)$.

```python
import opengen as og
import casadi.casadi as cs

# Build parametric optimizer
# ------------------------------------
u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)        \
    .with_constraints(bounds)
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_test_build") \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("tcp_enabled_optimizer")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_constraints_tolerance(1e-4)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('python_test_build/tcp_enabled_optimizer')
mng.start()

mng.ping()
solution = mng.call([1.0, 50.0])
print(solution)

mng.kill()
```

## Unobstructed navigation

Here we solve the following optimal control problem

<div class="math">
\[
    \begin{align}
    \mathbb{P}(p){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& \sum_{i=1}^{N - 1} 
        \|z_t-z^{\mathrm{ref}}\|_Q^2 + r \|u_t\|^2 + \|z_N-z^{\mathrm{ref}}\|_{Q_N}^2
    \\
    \text{subject to: }& z_{t+1} = f(z_t, u_t), t=0,\ldots, N-1
    \\
    &u_{\min} \leq u_t \leq u_{\max}, t=0,\ldots, N-1
    \\
    &z_0=p
    \end{align}
\]</div>

where $z = (x,y,\theta)$ is the position and orientation of the vehicle, $f$
describes the vehicle dynamics which is this example is

<div class="math">
\[
    f(z, u) = \begin{bmatrix}
    x + t_s [x +  \sin\theta (u_y \cos\theta - u_x \sin\theta) ]
    \\
    y + t_s [y +  \sin\theta (u_y \cos\theta - u_x \sin\theta) ]
    \\
    \theta + t_s L^{-1} (u_y \cos\theta - u_x \sin\theta)
    \end{bmatrix}
\]</div>

```python
import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

# Build parametric optimizer
# ------------------------------------
(nu, nx, N) = (2, 3, 20)
L = 0.5
ts = 0.1
(xref , yref, thetaref) = (1, 1, 0)
(q, qtheta, r, qN, qthetaN) = (10, 0.1, 1, 200, 2)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx)

(x, y, theta) = (z0[0], z0[1], z0[2])
cost = 0


for t in range(0, nu*N, nu):
    cost += q*((x-xref)**2 + (y-yref)**2) + qtheta*(theta-thetaref)**2
    u_t = u[t:t+2]
    theta_dot = (1/L) * (u_t[1] * cs.cos(theta) - u_t[0] * cs.sin(theta))
    cost += r * cs.dot(u_t, u_t)
    x += ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
    y += ts * (u_t[1] - L * cs.cos(theta) * theta_dot)
    theta += ts * theta_dot

cost += qN*((x-xref)**2 + (y-yref)**2) + qthetaN*(theta-thetaref)**2

umin = [-3.0] * (nu*N)
umax = [3.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost).with_constraints(bounds)
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_test_build") \
    .with_build_mode("debug")
meta = og.config.OptimizerMeta()       \
    .with_optimizer_name("navigation")
solver_config = og.config.SolverConfiguration().with_tolerance(1e-5)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config) \
    .with_verbosity_level(1)
builder.enable_tcp_interface()
builder.build()


# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('python_test_build/navigation')
mng.start()

mng.ping()
solution = mng.call([-1.0, 2.0, 0.0], initial_guess=[1.0] * (nu*N))
mng.kill()


# Plot solution
# ------------------------------------
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
```

This will produce the following plot:

<img src="/optimization-engine/img/unobstructed_navigation_python.png" alt="Matplotlib unobstructed navigation" />

If you would like to plot the (x, y)-trajectories of the vehicle,
run the following code:

```python
# Plot trajectory
# ------------------------------------
x_states = [0.0] * (nx*(N+2))
x_states[0:nx+1] = x_init
for t in range(0, N):
    u_t = u_star[t*nu:(t+1)*nu]

    x = x_states[t * nx]
    y = x_states[t * nx + 1]
    theta = x_states[t * nx + 2]

    theta_dot = (1/L) * (u_t[1] * np.cos(theta) - u_t[0] * np.sin(theta))

    x_states[(t+1)*nx] = x + ts * (u_t[0] + L*np.sin(theta)*theta_dot)
    x_states[(t+1)*nx+1] = y + ts * (u_t[1] - L*np.cos(theta)*theta_dot)
    x_states[(t+1)*nx+2] = theta + ts*theta_dot

xx = x_states[0:nx*N:nx]
xy = x_states[1:nx*N:nx]

print(x_states)
print(xx)
plt.plot(xx, xy, '-o')
plt.show()
``` 

<img src="/optimization-engine/img/unobstructed_navigation_trajectories_python.png" alt="Matplotlib unobstructed navigation" />



## Obstructed navigation

Let us add an obstable: consider a ball centered at the origin
with radius $r=1$, which needs to be avoided. To that end, we
introduce the constraint $c(u; p) = 0$ where

<div class="math">
\[c(u; p) = \sum_{t=0}^{N-1}[1-\|z_t\|^2]_+,\]</div>

where $z_t=(x_t,y_t)$ is the position of the vehicle at time $t$ and 
$[z]_+ = \max\\{0, z\\}$. 

The decision variable, as above, is $u = (u_0, \ldots, u_{N-1})$
and $z_t = z_t(u)$.

In order to define function $c$ in Python, we need to add one 
line in the above code:

```python
c = 0
for t in range(0, N):
    cost += q*((x-xref)**2 + (y-yref)**2) + qtheta*(theta-thetaref)**2
    u_t = u[t*nu:(t+1)*nu]
    theta_dot = (1/L) * (u_t[1] * cs.cos(theta) - u_t[0] * cs.sin(theta))
    cost += r * cs.dot(u_t, u_t)
    x += ts * (u_t[0] + L * cs.sin(theta) * theta_dot)
    y += ts * (u_t[1] - L * cs.cos(theta) * theta_dot)    
    theta += ts * theta_dot
    # ADD THIS LINE:
    c += cs.fmax(0, 1 - x**2 - y**2)
```

It makes sense to customize the solver parameters, especially
the allowed violation of the constraints (`with_constraints_tolerance`),
the update factor for the penalty weights (`with_penalty_weight_update_factor`)
and the initial weights (`with_initial_penalty_weights`):
 
```python
solver_config = og.config.SolverConfiguration()  \
    .with_tolerance(1e-4)                        \
    .with_max_outer_iterations(5)                \
    .with_constraints_tolerance(1e-2)            \
    .with_penalty_weight_update_factor(10.0)     \
    .with_initial_penalty_weights(100.0)
```

Here is a trajectory of the system with $N=60$. 

<img src="/optimization-engine/img/obstructed_navigation_trajectories_python.png" alt="Matplotlib obstructed navigation" />

## Nonlinear Estimation

### Problem statement
Consider a dynamical system, for simplicity, but without loss of generality
without an actuation, with dynamics

<div class="math">
\[
x_{t+1} = f(x_t) + w_t,
\]
</div>

where $w_t$ is an unknown disturbance signal. We are able to measure the system's
output through 

<div class="math">
\[
y_{t} = h(x_t) + v_t,
\]
</div>

where $v_t$ is the measurement error.

Given a set of measurements $Y_N=(y_{0},\ldots,y_{N})$, our objective is to 
determine estimates $\hat{x}_t$ for all $t=0,\ldots,N-1$.

The estimation problem consists in solving the following optimization 
problem

<div class="math">
\[
    \begin{align}
\operatorname*{Minimize}_{\hat{x}_0, \hat{\mathbf{w}}, \hat{\mathbf{v}}} &
    \sum_{t=0}^{N}\|\hat{w}_t\|^2_Q + \|\hat{v}_t\|^2_R
    \\
    \text{subject to: }& \hat{x}_{t+1} = f(\hat{x}_t) + \hat{w}_t
    \\
    & y_t = h(\hat{x}_t) + \hat{v}_t, t=0,\ldots, N-1.
    \end{align}
\]
</div>

By eliminating all $\hat{w}_t$ and $\hat{v}_t$, the problem becomes

<div class="math">
\[
    \mathbb{P}(\mathbf{y}) {}:{}
    \operatorname*{Minimize}_{\hat{\mathbf{x}}}
    \sum_{t=0}^{N}\|\hat{x}_{t+1} - f(\hat{x}_t)\|^2_Q + \|y_t - h(\hat{x}_t)\|^2_R
\]
</div>

The solution of this problem generates an optimal estimate 

<div class="math">
\[
    \hat{\mathbf{x}}^\ast=(\hat{x}_0^*,\ldots,\hat{x}_{N-1}^*).
\]
</div>

Such problems are used in **moving horizon estimation** with zero prior
weighting.


### Data Generation

Consider the Euler-discretization of the Lorenz system given by 

<div class="math">
\[
x_{t+1} = x_{t} + t_s 
\begin{bmatrix}
\sigma (x_{2, t} - x_{1,t})
\\
x_{1, t} (\rho - x_{3, t}) - x_{2, t}
\\
x_{1, t} x_{2, t} - \beta x_{3, t}
\end{bmatrix} + w_t,
\]
</div>

where $t_s$ is the sampling time, $\sigma$, $\rho$ and $\beta$ are constant
parameters, $x_t = (x_{1, t}, x_{2, t}, x_{3, t})$ is the system state and $w_t$
is an unknown disturbance.

The measurements are obtained via the system output $y_t \in \mathbb{R}^3$ 
given by

<div class="math">
\[
y_{t} = 
\begin{bmatrix}
2x_{1, t}
\\
x_{2, t} + x_{3, t}
\\
\tfrac{1}{10}x_{3, t}^2 - x_{1, t}
\end{bmatrix} + v_t,
\]
</div>

where $v_t\in\mathbb{R}^3$ in a measurement noise term.

Let us first generate output data from this system.

```python
# Some necessary imports
# ------------------------------------
import opengen as og
import casadi.casadi as cs
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
```

```python
# Generate data
# ------------------------------------
(rho, sigma, beta, ts) = (28.0, 10.0, 8.0/3.0, 0.02)
nx = 3
Tsim = 100


def lorenz_sys(x):
    y = [sigma * (x[1] - x[0]),
         x[0] * (rho - x[2]) - x[1],
         x[0] * x[1] - beta * x[2]]
    return y


def dynamics(x):
    dx = lorenz_sys(x)
    return [x[idx] + ts*dx[idx] for idx in range(nx)]


def output(x):
    return [2.0*x[0], x[1]+x[2], x[2]**2/10.0-x[0]]

# Produce data
# ------------------------------------
x_data = [0.0] * (nx * Tsim)
y_data = [0.0] * (nx * Tsim)
x_data[0:nx] = [-10.0, -12.0, 27.0]
y_data[0:nx] = output(x_data[0:nx])

for i in range(Tsim-1):
    w = np.random.normal(0, 1, 3)
    v = np.random.normal(0, 1, 3)
    x_data[(i+1)*nx:(i+2)*nx] = dynamics(x_data[i*nx:(i+1)*nx]) + 0.02*w
    y_data[(i+1)*nx:(i+2)*nx] = output(x_data[(i+1)*nx:(i+2)*nx]) + 0.1*v
```    
The output data are plotted below:

<img src="/optimization-engine/img/python_estimation_data.png" alt="System output data">


### Code generation

```python
def lorenz_sys_casadi(x):
    y = cs.vertcat(sigma * (x[1] - x[0]),
         x[0] * (rho - x[2]) - x[1],
         x[0] * x[1] - beta * x[2])
    return y


def dynamics_casadi(x):
    dx = lorenz_sys_casadi(x)
    return x + ts*dx


def output_casadi(x):
    return cs.vertcat(2.0*x[0], x[1]+x[2], x[2]**2/10.0-x[0])

# Problem definition
# ----------------------------------
X_hat = cs.SX.sym('Xhat', nx*(Tsim))
Y = cs.SX.sym('Y', nx*Tsim)
V = 0.0
print(X_hat)
for i in range(Tsim-1):
    w = X_hat[(i+1)*nx:(i+2)*nx] - dynamics_casadi(X_hat[i*nx:(i+1)*nx])
    v = Y[i*nx:(i+1)*nx] - output_casadi(X_hat[i*nx:(i+1)*nx])
    V += 20.0 * cs.dot(w, w) + 1.0 * cs.dot(v, v)

# Code generation
# ------------------------------------
problem = og.builder.Problem(X_hat, Y, V)
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_test_build") \
    .with_build_mode("release")
meta = og.config.OptimizerMeta()               \
    .with_optimizer_name("estimator")
builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config)
builder.enable_tcp_interface()
builder.build()
```

This will generate a parametric optimizer for problem $\mathbb{P}(\mathbf{y})$
shown above that takes the system output, $\mathbf{y}$, and returns an estimate
of the system state.

Such an estimation for the above randomly-generated data is shown below. The 
optimization problem was solved in 2.73 milliseconds (in 65 iterations).

<img src="/optimization-engine/img/python_estimation.png" alt="State estimates">
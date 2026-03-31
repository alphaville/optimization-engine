---
id: example-nav
title: Navigation
description: Nonlinear MPC for navigation with OpEn
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


## Problem statement

### System dynamics
Assuming zero slip of the trailer wheels, the nonlinear kinematics of a ground vehicle, shown in the following figure

<img src="/optimization-engine/img/cart_schematic.jpg" alt="Cart Schematic" width="400"/>

is given by the following equations

<div class="math">
\[\begin{split}
\dot{x}(t) {}={}&amp; u_x(t) + L \sin \theta(t) \dot{\theta}(t)\\
\dot{y}(t) {}={}&amp; u_y(t) - L \cos \theta(t) \dot{\theta}(t)\\
\dot{\theta}(t) {}={}&amp; \tfrac{1}{L}(u_y(t) \cos \theta(t) - u_x(t) \sin \theta(t))
\end{split}\]</div>

where the state vector $z=(x, y, \theta)$ comprises the coordinates $x$ and $y$ of the trailer and the heading angle $\theta$. 

The input $u=(u_x, u_y)$ is a velocity reference which is tracked by a low-level controller. 

The distance between the center of mass of the trailer and the fulcrum connecting to the towing vehicle is $L = 0.5\mathrm{m}$.

The system dynamics can be written concisely as
<div class="math">\[\dot{z}(t) = f(z(t), u(t)).\]</div>

We shall discretise this system using the Euler discretization with sampling time $t_s$, that is
<div class="math">\[z_{t+1} = z_t + t_sf(z_{t}, u_{t}).\]</div>



### Cost functions
Our aim is to determine a sequence of control actions, 
<div class="math">\[u = (u_0, u_1, \ldots, u_{N-1}),\]</div>
so as to create a sequence of states $z_0,\ldots, z_N$, which reach as close as possible to a given target point $(x^{\mathrm{ref}}, y^{\mathrm{ref}})$ and a reference heading $\theta^{\mathrm{ref}}$.

To this end, we define the following stage cost function
<div class="math">\[\ell(z, u) = q [(x-x^{\mathrm{ref}})^2 + (y-y^{\mathrm{ref}})^2] + q_{\theta}(\theta-\theta^{\mathrm{ref}})^2 + r\|u\|^2,\]</div>
where $q$, $q_\theta$ and $r$ are nonnegative weight coefficients. 

Likewise, we define the terminal cost function
<div class="math">\[\ell_N(z) = q_N [(x-x^{\mathrm{ref}})^2 + (y-y^{\mathrm{ref}})^2] + q_{\theta,N}(\theta-\theta^{\mathrm{ref}})^2,\]</div>

We now introduce the *total cost function*

<div class="math">\[V(u; z_0) = \sum_{t=0}^{N-1}\ell(z_t, u_t) + \ell_N(z_N),\]</div>

where the sequence of states, $z_0,\ldots, z_N$ is governed by the discrete-time dynamics stated above.


### Optimal control problem

We formulate the following optimal control problem

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{2N}}&amp;\ \ V(u; z_0)\\
\mathrm{subject\ to} &amp;\ \ u \in U(z_0)\end{split}\tag{4}\]</div>

We assume there are no active constraints on the input variables, that is 
$U(z_0) = \mathbb{R}^{2N}$.

Note that the sequence of states, $z_0, z_1, \ldots, z_N$, does not partipate in the problem definition in *(4)*. This is because $z_t = z_t(z_0, u_0, \ldots, u_{t-1})$ for all $t=1,\ldots, N$. In order words, all $z_t$ are functions of $u$ and $z_0$.




## Code generation in MATLAB

Firstly, we define the total cost function

```matlab
% parameters
L = 0.5; ts = 0.1;

% Prediction horizon
N = 50; 

% target point and bearing
xref=1; yref=1; thetaref = 0;

% weights
q = 10; qtheta = .1; r = 1;
qN = 10*q; qthetaN = 10*qtheta;

nu = 2; nx = 3;
u = casadi.SX.sym('u', nu*N); 
z0 = casadi.SX.sym('z0', nx);

x = z0(1); y = z0(2); theta = z0(3);
cost = 0;
for t = 1:nu:nu*N
    cost = cost + q*((x-xref)^2 + (y-yref)^2) + qtheta*(theta-thetaref)^2;
    u_t = u(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    cost = cost + r*(u_t'*u_t);
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
end
cost = cost + qN*((x-xref)^2 + (y-yref)^2) + qthetaN*(theta-thetaref)^2;

constraints = OpEnConstraints.make_no_constraints();
```

We then build the parametric optimizer:

```matlab
builder = OpEnOptimizerBuilder()...
    .with_problem(u, z0, cost, constraints)...
    .with_build_name('navigation')...
    .with_build_mode('release')...
    .with_fpr_tolerance(1e-4)...
    .with_max_iterations(500);
optimizer = builder.build();
```

## Solution

The solution is presented below (the algorithm converges in 25 iterations after 3.2ms):

<img src="/optimization-engine/img/nav-oc-sol-xyt.jpg" 
    alt="Navigation (x,y) vs time" 
    width="500"/>

<img src="/optimization-engine/img/nav-oc-sol-xy.jpg" 
    alt="Navigation (x,y) profile" 
    width="500"/>

<img src="/optimization-engine/img/nav-oc-sol-theta.jpg" 
    alt="Navigation (x,y) profile" 
    width="500"/>



## Free references
Let's get a little creative!

<p>
    We may define the parameter of the optimization problem to be 
    <div class="math">
    \[p = (z_0, z^{\mathrm{ref}})\]
    </div>
    where $z^{\mathrm{ref}} = (x^{\mathrm{ref}},y^{\mathrm{ref}},\theta^{\mathrm{ref}})$.
    This way, we will have an optimal control module that will allow us to plan 
    trajectories from any initial position and pose to any final position and pose.
    This can then be used as a model predictive controller, where the references
    can be provided either by the user, manually, or by a higher-level system.
</p>

<p>
    We will define the parameter $p$, instead of $z_0$, as follows:
</p>

```matlab
p = casadi.SX.sym('p', 2*nx);
x = p(1); y = p(2); theta = p(3);
xref = p(4); yref = p(5); thetaref = p(6);
```

<p>
    the rest of the problem definition remains the same.
</p>

<p>
    The optimizer should now be called as follows:
</p>

```matlab
z_init = [1.0; -0.3; deg2rad(30)];
z_ref  = [1.5;  0.7; deg2rad(50)];
out = optimizer.consume([z_init; z_ref]);
```

<img src="/optimization-engine/img/nav-oc-sol-refs.jpg" 
    alt="Multiple references" 
    title="Different refernece positions and poses" 
    width="500"/>


## Obstacle avoidance
Consider the problem of determining a minimum-cost trajectory which 
avoids an obstacle, $O$, which is described by 
<div class="math">
    \[O = \{z = (x,y) {}\mid{}  h(z) > 0\},\]
</div>
where $h:\mathbb{R}^2\to\mathbb{R}$ is a $C^{1,1}$-function. The obstacle avoidance constraint
$z\notin O$ is satisfied if and only if 
<div class="math">
    \[[{}h(z){}]_+^2 {}={} 0,\]
</div>
<p>
    where $[x]_+ = \max\{0, x\}$ is the plus operator.
</p>
<p>
    We now define the modified stage cost function
</p>
<div class="math">
    \[\tilde{\ell}(z,u) {}={} \ell(z,u) + \eta [{}h(z){}]_+^2,\]
</div>
<p>
    and the modified terminal cost function
</p>
<div class="math">
    \[\tilde{\ell}_N(z) {}={} \ell_N(z) + \eta_N [{}h(z){}]_+^2,\]
</div>
where $\eta$ and $\eta_N$ are positive weights.
In the following plot we show trajectories for different values of 
$\eta$ and $\eta_N$...


<img src="/optimization-engine/img/nav-oc-sol-xy-obst.jpg" 
    alt="Obstacle avoidance" 
    title="Obstacle avoidance" 
    width="500"/>

For completeness, here is the modified code:

```matlab
nu = 2; nx = 3;
u = casadi.SX.sym('u', nu*N); 
p = casadi.SX.sym('p', 2*nx+1);
x = p(1); y = p(2); theta = p(3);
xref = p(4); yref = p(5); thetaref=p(6);
h_penalty = p(end);
cost = 0;

% Obstacle (disc centered at `zobs` with radius `c`)
c = 0.4; zobs = [0.7; 0.5];

for t = 1:nu:nu*N
    z = [x; y];
    cost = cost + q*norm(z-zref) + qtheta*(theta-thetaref)^2;
    cost = cost + h_penalty * max(c^2 - norm(z-zobs)^2, 0)^2;
    u_t = u(t:t+1);
    theta_dot = (1/L)*(u_t(2)*cos(theta) - u_t(1)*sin(theta));
    cost = cost + r*(u_t'*u_t);
    x = x + ts * (u_t(1) + L * sin(theta) * theta_dot);
    y = y + ts * (u_t(2) - L * cos(theta) * theta_dot);
    theta = theta + ts * theta_dot;
end
cost = cost + qN*((x-xref)^2 + (y-yref)^2) + qthetaN*(theta-thetaref)^2;
cost = cost + h_penalty * max(c^2 - norm(z-zobs)^2, 0)^2;
```


## Experimental validation

A somewhat more involved nonlinear model predictive control (NMPC) formulation, enhanced with obstacle avoidance capabilities, was presented in [ECC '18](https://core.ac.uk/download/pdf/153430972.pdf).

A short footage of our experiment is shown below:

![](/optimization-engine/img/6f6ea4f8d194.gif)

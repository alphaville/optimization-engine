---
id: example-nav
title: Navigation
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


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
We define the following stage cost function
<div class="math">\[\ell(z, u) = q [(x-x^{\mathrm{ref}})^2 + (y-y^{\mathrm{ref}})^2] + q_{\theta}(\theta-\theta^{\mathrm{ref}})^2 + r\|u\|^2,\]</div>
where $q$, $q_\theta$ and $r$ are nonnegative weight coefficients. 

Likewise, we define the terminal cost function
<div class="math">\[\ell_N(z) = q_N [(x-x^{\mathrm{ref}})^2 + (y-y^{\mathrm{ref}})^2] + q_{\theta,N}(\theta-\theta^{\mathrm{ref}})^2,\]</div>

Our aim is to determine a sequence of control actions, 
<div class="math">\[u = (u_0, u_1, \ldots, u_{N-1}),\]</div>

We now define the *total cost function*

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
    cost = cost + q*((x-xref)^2 + (y-yref)^2) + qtheta*(theta-thetaref)^2 ;  
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

<img src="/optimization-engine/img/nav-oc-sol-xyt.jpg" alt="Navigation (x,y) vs time" width="500"/>

<img src="/optimization-engine/img/nav-oc-sol-xy.jpg" alt="Navigation (x,y) profile" width="500"/>

<img src="/optimization-engine/img/nav-oc-sol-theta.jpg" alt="Navigation (x,y) profile" width="500"/>

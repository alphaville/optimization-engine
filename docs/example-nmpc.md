---
id: example-nmpc
title: Control of population dynamics
sidebar_label: Population dynamics
description: Nonlinear model predictive control with OpEn: population dynamics
---
<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


Here, we will give a complete example of designing a nonlinear model predictive controller (NMPC) using **OpEn**.

Let us first give the problem statement. 

## Control of population dynamics

Consider a **Lotka-Volterra-type model**, aka **predator-prey-type model**, with actuation $u_t$, which is described by the following system of equations
<div class="math">
\[\begin{split}
x_{t+1} {}={}&amp; \frac{\alpha x_{t} - \beta x_{t}y_{t}}{1 + \gamma x_{t}} + u_{t}\\
y_{t+1} {}={}&amp; \frac{\delta y_{t} - \eta x_{t}y_{t}}{1+\lambda y_{t}}
\end{split}\tag{1}\]</div>
This is a two-dimensional system with state 
<div class="math">\[z_{t} = (x_{t}, y_{t})\]</div>, where $x$ is the number of prey (e.g., rabbits) and $y$ is the number of a predator (e.g., foxes); $u$ corresponds to an external inflow or outflow of prey.

We are looking for a sequence of inputs
<div class="math">\[u = (u_{0}, \ldots, u_{N-1})\]</div>
which minimizes the total cost function 
<div class="math">\[V(u, z_0) {}={} \sum_{t=0}^{N-1}\ell(z_{t}, u_{t}) + \ell_N(z_N),\tag{2}\]</div>
where the stage cost function is defined as 
<div class="math">\[\ell(z,u)=q\|x-x^{\mathrm{ref}}\|^2 + \mu \max\{0, y^{\mathrm{lim}}-y\} + ru^2,\tag{3a}\]</div>
and the terminal cost function is 
<div class="math">\[\ell_N(z) = q\|x-x^{\mathrm{ref}}\|^2 + \mu \max\{0, y^{\mathrm{lim}}-y\}.\tag{3b}\]</div>

We chose these non-quadratic functions to demonstrate that **OpEn** can easily accommodate any ($C^{1,1}$-smooth) nonlinear cost function.

We therefore need to solve the following parametric optimization problem, with parameter 
$z_0 = (x_0, y_0)$:
<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{N}}&amp;\ \ V(u; z_0)\\
\mathrm{subject\ to} &amp;\ \ u \in U(z_0)\end{split}\tag{4}\]</div>

We assume there are no active constraints on the input variables, that is 
$U(z_0) = \mathbb{R}^N$.

The following parameters are given: 
$\alpha = 0.6$, $\beta = 0.05$,  $\gamma = 0.1$,
$\eta =  0.1$, $\delta = 0.95$ and $\lambda = 0.1$.

Note that the sequence of states, $z_0, z_1, \ldots, z_N$, does not partipate in the problem definition in *(4)*. This is because $z_t = z_t(z_0, u_0, \ldots, u_{t-1})$ for all $t=1,\ldots, N$. In order words, all $z_t$ are functions of $u$ and $z_0$.


## Code generation in MATLAB

We may generate a parametric optimizer for the above optimal control problem as follows:

```matlab
% Specify the parameters of the system:
alpha = 0.6; bet = 0.05; gam = 0.1; 
eta =  0.1; delta = 0.95;  lam = 0.1; 

% Prediction horizon:
N = 20;

% Parameters of the cost functions:
q = 1.5; r = 0.1; qN = 2; mu = 3000;
xref=0.2; ylim=0.05;

u = casadi.SX.sym('u', N);
z0 = casadi.SX.sym('u', 2);

% Construct the cost function
x = z0(1); y = z0(2); cost = 0;
for t = 1:N
    cost = cost + q*(x-xref)^2 + mu*max(ylim-y,0)^2+ r*u(t)^2;    
    x = (alpha*x - bet*x*y)/(1 + gam*x) + u(t);
    y = ( delta*y - eta*x*y)/(1 + lam*y);
end
cost = cost + qN*(x-xref)^2 + mu*max(ylim-y,0)^2;

% Constraints
constraints = OpEnConstraints.make_no_constraints();

% Use OpEnOptimizerBuilder to build an optimizer
builder = OpEnOptimizerBuilder()...
    .with_problem(u, z0, cost, constraints)...
    .with_build_name('lotka_volterra')...
    .with_fpr_tolerance(1e-8);
optimizer = builder.build();
```

## Solution

We first need to start the parametric optimization module and connect to it (read the [documentation](/optimization-engine/docs/matlab-interface) of the MATLAB interface for details)

```matlab
optimizer.run();
optimizer.connect();
```

We may then use the optimizer as follows:

```matlab
z0 = [0.75;0.5];
out = optimizer.consume(z0);
```

Here `z0` is the initial condition, $z_0 = (x_0, y_0)$. 

The optimizer returns the following structure:

```text
     p: [2×1 double]
     u: [20×1 double]
     n: 46
     f: -8.0539
    dt: '4.615292ms'
```

The problem in solved in 4.6ms (46 iterations).

The optimal solution is stored in `out.u`.

The solution is presented in the following plot:

<img src="/optimization-engine/img/lv-oc-sol.jpg" alt="Lotka-Volterra Optimal Solution" width="500"/>

The optimizer object can be reused to solve a problem of the form (4) with a different parameter, $z_0$. For example:

```matlab
z0 = [-1;1];
out = optimizer.consume(z0);
```

produces the following solution:

<img src="/optimization-engine/img/lv-oc-sol-2.jpg" alt="Lotka-Volterra Optimal Solution" width="500"/>

Once the optimizer is no longer needed, we should disconnect from it and kill it:

```matlab
optimizer.run();
optimizer.stop();
```

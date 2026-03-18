---
id: python-ocp
title: Optimal Control
description: Optimal Control with OpEn/opengen
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

<div class="alert alert-warning">
<b>Info:</b> The functionality presented here was introduced in <code>opengen</code> version <code>0.10.0a1</code>.
The API is still young and is likely to change in versions <code>0.10</code> and <code>0.11</code>.
</div>

Opegen now comes with a new module that facilitates the construction 
of optimal control problems. In an intuitive and straightforward way,
the user defines the problem ingredients (stage/terminal costs, 
state/input constraints, dynamics). 

## In a nutshell: quick overview

Suppose you want to solve the optimal control problem 
<div class="math">
\[
    \begin{align}
    \mathbb{P}_N(p){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& 
    \sum_{t=1}^{N - 1} 
        \underbrace{\|x_t-x^{\mathrm{ref}}\|^2 + r \|u_t\|^2}_{\text{stage cost}} 
        + 
        \underbrace{\|x_N-x^{\mathrm{ref}}\|^2}_{\text{terminal cost}}
    \\
    \text{subject to: }& x_{t+1} = F(x_t, u_t), t=0,\ldots, N-1
    \\
    &u_{\min} \leq u_t \leq u_{\max}, t=0,\ldots, N-1
    \\
    &x_t \leq x_{\max}, t=0,\ldots, N
    \\
    &x_0=x
    \end{align}
\]</div>
This can be done as follows:

```python
# Check this out...
ocp = og.ocp.OptimalControlProblem(nx=2, nu=1, horizon=5, 
                                   shooting=og.ocp.ShootingMethod.SINGLE)
ocp.add_parameter("x0", 2)
ocp.add_parameter("xref", 2, default=[0.0, 0.0])

# System dynamics
ocp.with_dynamics(lambda x, u, param: 
                  cs.vertcat(x[0] + u[0], x[1] - u[0]))

# A typical stage cost function
ocp.with_stage_cost(
    lambda x, u, param, _t:
    cs.dot(x - param["xref"], x - param["xref"]) + 0.01 * cs.dot(u, u)
)

# Terminal cost function
ocp.with_terminal_cost(
    lambda x, param:
    2.0 * cs.dot(x - param["xref"], x - param["xref"])
)

# Input constraints
ocp.with_input_constraints(og.constraints.Rectangle([-0.4], [0.4]))

# State/input joint constraints
ocp.with_path_constraint(
    lambda x, u, param, _t: cs.fmax(0.0, x[0] - 1.5)
)
```

Having defined the above OCP, we can build the optimizer...


```python
solver_config = ...
optimizer = og.ocp.OCPBuilder(
            ocp,
            metadata=og.config.OptimizerMeta().with_optimizer_name("ocp_single_tcp"),
            build_configuration=og.config.BuildConfiguration()
                .with_build_directory(".")
                .with_tcp_interface_config(
                    tcp_interface_config=og.config.TcpServerConfiguration(bind_port=3391)
                ),
            solver_configuration=solver_config,
        ).build()
```

The optimizer can then be called as follows:

```python
result = single_optimizer.solve(x0=[1,-1], xref=[0, 0])
```

and note that the parameter `xref=xref` is optional; if not specified,
the default value will be used (the default was set when we constructed the 
OCP).

## Step-by-step documentation
---
id: python-ocp
title: Optimal Control
description: Optimal Control with OpEn/opengen
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<style>
.but{
  border: none;
  color: #348c4f;
  padding: 15px 20px;
  text-align: center;
  text-decoration: none;
  display: inline-block;
  font-size: 16px;
  margin: 0px 0px;
  cursor: pointer;
  width: 250px;
  border-radius: 8px;
}
</style>
<style>
.but1 {
    background-color: #e9e642;
}
</style><style>
.but2 {
    background-color: #008CBA;
}
</style>

<div class="alert alert-warning">
<b>Info:</b> The functionality presented here was introduced in <code>opengen</code> version <code>0.10.0a1</code>.
The API is still young and is likely to change in versions <code>0.10</code> and <code>0.11</code>.
</div>

Opegen now comes with a new module that facilitates the construction 
of optimal control problems. In an intuitive and straightforward way,
the user defines the problem ingredients (stage/terminal costs, 
state/input constraints, dynamics). 

<a href="https://colab.research.google.com/drive/17vbVUbqcah9seIg17aN6bW0-T15FWrBo?usp=sharing" target="_blank"><button class="but but1" ><b>Try this on Google Colab</b></button></a>

## In a nutshell: quick overview

Suppose you want to solve the optimal control problem 
<div class="math">
\[
    \begin{align}
    \mathbb{P}_N(p){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& 
    \sum_{t=1}^{N - 1} 
        \underbrace{q\|x_t-x^{\mathrm{ref}}\|^2 + r \|u_t\|^2}_{\text{stage cost}} 
        + 
        \underbrace{10\|x_N-x^{\mathrm{ref}}\|^2}_{\text{terminal cost}}
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
Suppose the state is two-dimensional and the input is one-dimensional.
The dynamics is 
<div class="math">
\[
    \begin{align}
    F(x, u; a) = 
        \begin{bmatrix}
        0.98 \sin(x_1) + x_2 \\
        0.1x_1^2 - 0.5 x_1 + a x_2 + u
        \end{bmatrix},
    \end{align}
\]</div>
where $a$ is a parameter. Suppose the stage cost
function is
<div class="math">
\[
    \begin{align}
    \ell(x, u; x^{\rm ref}, q, r)
    {}={}
    q\| x - x^{\rm ref}\|^2 + r \|u\|^2,
    \end{align}
\]</div>
where $x^{\rm ref}$, $q$, and $r$ are parameters. The terminal 
cost function is 
<div class="math">
\[
    \begin{align}
    V_f(x; x^{\mathrm{ref}}) = 100\|x_N-x^{\mathrm{ref}}\|^2.
    \end{align}
\]</div>
Lastly, we have the state constraint $x_t \geq x_{\rm min}$, where $x_{\rm min}$ is a parameter,
and the hard input constraints $|u_t| \leq 0.2$.

<br>

This optimal control problem can be constructed as follows:

```python
optimizer_name = "ocp_alm"

# Construct the OCP
ocp = og.ocp.OptimalControlProblem(nx=2, nu=1, horizon=20)

# Define the parameters
ocp.add_parameter("x0", 2)
ocp.add_parameter("xref", 2, default=[0.0, 0.0])
ocp.add_parameter("q", 1, default=1)
ocp.add_parameter("r", 1, default=0.1)
ocp.add_parameter("a", 1, default=1)
ocp.add_parameter("xmin", 1, default=-1)

# System dynamics
ocp.with_dynamics(lambda x, u, param:
                  cs.vertcat(0.98 * cs.sin(x[0]) + x[1],
                             0.1 * x[0]**2 - 0.5 * x[0] + param["a"] * x[1] + u[0]))
# Stage cost
ocp.with_stage_cost(
    lambda x, u, param, _t:
    param["q"] * cs.dot(x - param["xref"], x - param["xref"])
    + param["r"] * cs.dot(u, u)
)

# Terminal cost
ocp.with_terminal_cost(
    lambda x, param: 100 * cs.dot(x - param["xref"], x - param["xref"])
)

# State constraint: x1 <= xmax, imposed with ALM
ocp.with_path_constraint(
    lambda x, u, param, _t: x[1] - param["xmin"],
    kind="alm",
    set_c=og.constraints.Rectangle([0.], [1000.0]),
)

# Input constraints
ocp.with_input_constraints(og.constraints.BallInf(radius=0.2))
```

Having defined the above OCP, we can build the optimizer...

<!--DOCUSAURUS_CODE_TABS-->

<!--Direct intercafe-->
```python
ocp_optimizer = og.ocp.OCPBuilder(
    ocp,
    metadata=og.config.OptimizerMeta().with_optimizer_name(optimizer_name),
    build_configuration=og.config.BuildConfiguration()
        .with_build_python_bindings().with_rebuild(True),
    solver_configuration=og.config.SolverConfiguration()
        .with_tolerance(1e-5)
        .with_delta_tolerance(1e-5)
        .with_preconditioning(True)
        .with_penalty_weight_update_factor(1.8)
        .with_max_inner_iterations(2000)
        .with_max_outer_iterations(40),
).build()
```

<!--TCP socket interface-->
```python
ocp_optimizer = og.ocp.OCPBuilder(
    ocp,
    metadata=og.config.OptimizerMeta().with_optimizer_name(optimizer_name),
    build_configuration=og.config.BuildConfiguration()
        .with_tcp_interface_config(
            tcp_interface_config=og.config.TcpServerConfiguration(bind_port=3391)
        ).with_rebuild(True),
    solver_configuration=og.config.SolverConfiguration()
        .with_tolerance(1e-5)
        .with_delta_tolerance(1e-5)
        .with_preconditioning(True)
        .with_penalty_weight_update_factor(1.8)
        .with_max_inner_iterations(2000)
        .with_max_outer_iterations(40),
).build()
```

<!--END_DOCUSAURUS_CODE_TABS-->

The optimizer can then be called as follows:

```python
result = ocp_optimizer.solve(x0=[0.4, 0.2], q=30, r=1, a=0.8, xmin=-0.2)
```

and note that all parameters except `x0` are optional; if not specified,
their default values will be used (the defaults were set when we constructed the 
OCP). We can now plot the optimal sequence of inputs (`result.inputs`)

<img src="/optimization-engine/img/ocp-inputs.png" alt="Seq. inputs" width="60%">

and the corresponding sequence of states (`result.states`)

<img src="/optimization-engine/img/ocp-states.png" alt="Seq. states" width="60%">

The object `result` contains the above sequences of inputs and states and additional
information about the solution, solver time, Lagrange multipliers, etc.


## Step-by-step documentation
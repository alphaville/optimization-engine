---
id: python-ocp-2
title: Constructing OCPs
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

:::note Info
The functionality presented here was introduced in `opengen` version [`0.10.0a1`](https://pypi.org/project/opengen/#history). The API is still young and is likely to change in version `0.11`.
:::

Here we will look at how we can construct an optimal control problem (OCP)
by defining its state and terminal cost functions, input and state 
constraints, prediction horizon and other options.


Generally, we want to solve problems of the form 
<div class="math">
\[
    \begin{align}
    \mathbb{P}_N(p){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& 
    \sum_{t=0}^{N - 1} \ell_t(x_t, u_t; p) + V_f(x_N; p)
    \\
    \text{subject to: }& x_{t+1} = F(x_t, u_t; p),
    \\
    &u_t \in U, 
    \\
    &(x_t, u_t) \in Z_t(p), \text{ for } t=0,\ldots, N - 1
    \\
    &x_N \in X_N, 
    \\
    &x_0=x
    \end{align}
\]</div>


## OCP Formulations

### Single shooting

There are two main ways to formulate an optimal control 
problem. The **single shooting** approach consists in 
eliminating the sequence of states; the decision variables
are the control actions $u_0, \ldots, u_{N-1}$, i.e., the OCP looks
like (simplified version — additional constraints can be present)

<div class="math">
\[
    \begin{align}
    \mathbb{P}_N(p){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& 
    \sum_{t=0}^{N - 1} \ell_t(x_t, u_t; p) + V_f(x_N; p)
    \\
    \text{subject to: }& u_t \in U, 
    \\
    &x_0=x
    \end{align}
\]</div>

where $x_t$ is a function of the initial state, $x_0$, and 
the sequence of inputs $u_0, \ldots, u_{t-1}$. For example, 
$x_1 = F(x_0, u_0)$, and $x_2 = F(x_1, u_1) = F(F(x_0, u_0), u_1)$.
The single shooting formulation is the default one as we have 
observed that it leads to better performance. To construct a 
single shooting OCP do

```python
import opengen as og
import casadi.casadi as cs

ocp = og.ocp.OptimalControlProblem(
    nx=2, nu=1, 
    horizon=20,
    shooting=og.ocp.ShootingMethod.SINGLE)
```

Since $\mathbf{u} = (u_0, \ldots, u_{N-1})$ is the decision 
variable, we can impose hard constraints of the form 
$u_t \in U$ for all $t$. For example, to impose the constraint 
$\Vert u_t \Vert_\infty \leq 0.2$ we can do 

```python
set_U = og.constraints.BallInf(radius=0.2)
ocp.with_input_constraints(set_U)
```

### Multiple shooting

Alternatively, we can formulate the problem using the 
**multiple shooting** approach, where the sequence of states
is not eliminated. The OCP now has the form (simplified version — additional constraints can be present)

<div class="math">
\[
    \begin{align}
    \mathbb{P}_N(p){}:{}\operatorname*{Minimize}_{\substack{u_0, \ldots, u_{N-1} \\ x_0, \ldots, x_{N}}}& 
    \sum_{t=0}^{N - 1} \ell_t(x_t, u_t; p) + V_f(x_N; p)
    \\
    \text{subject to: }& 
    x_{t+1} = F(x_t, u_t; p),
    \\
    &(x_t, u_t) \in Z, \text{ for } t=0,\ldots, N - 1
    \\
    &x_N \in X_N, 
    \\
    &x_0=x
    \end{align}
\]</div>


A multiple shooting problem can be constructed as follows

<a href="https://colab.research.google.com/drive/1pjJLPBW0KgtAC_5z3zCHhJ4SV9WE7Jhg?usp=sharing" target="_blank"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Try it In Colab"></a>


```python
ocp = og.ocp.OptimalControlProblem(
    nx=2, nu=1, 
    horizon=20,
    shooting=og.ocp.ShootingMethod.MULTIPLE)
```

Here the decision variable is the vector $\mathbf{z} = (u_0, x_0, \ldots, u_{N-1}, x_{N-1}, x_N)$
and the system dynamics become equality constraints, which can be imposed via ALM (default)
or PM; this can be configured using `with_dynamics_constraints` and either `"alm"` or `"penalty"`.

```python
ocp.with_dynamics_constraints("alm")
```

When using the multiple shooting formulation, we can impose constraints of the form $(x_t, u_t)\in Z$
using `with_hard_stage_state_input_constraints`.

```python
set_Z = og.constraints.BallInf(radius=1.0)
ocp.with_hard_stage_state_input_constraints(set_Z)
```

Likewise, since $x_N$ is part of the decision variable vector, $\mathbf{z}$,
we can impose hard constraints. This can be done as follows

```
set_XN = og.constraints.Ball1(radius=0.1)
ocp.with_hard_terminal_state_constraints(set_XN)
```

## Parameters

The OCP API supports **named parameters**. Parameters are packed internally
into the flat vector expected by the low-level OpEn builder, but the user does
not need to keep track of slices manually. Instead, parameters are declared by
name and are later accessed by name inside the callback functions.

Parameters are added using `add_parameter(name, size, default=None)`.

```python
ocp.add_parameter("x0", 2)
ocp.add_parameter("xref", 2, default=[0.0, 0.0])
ocp.add_parameter("q", 1, default=10.0)
```

In the above example:

- `x0` is required because no default value is provided
- `xref` and `q` are optional when calling the optimizer
- scalar defaults such as `10.0` are allowed for one-dimensional parameters

If the same parameter name is declared twice, `add_parameter` raises a
`ValueError`. Additional checks are in place to validate parameter dimensions.

Parameters can be used in callback functions (dynamics, cost functions, 
constraints); more on this later.

:::note
The parameter <code>x0</code> should always be declared because it defines the initial
state of the OCP.
:::



## Dynamics

The system dynamics is specified using `with_dynamics`. The callback must
return the next state, that is, it must implement the map

<div class="math">
\[
    x^+ = F(x, u; p)
\]</div>

where `x` is the current state, `u` is the current input, and `p` is the
parameter view introduced earlier. In other words, the OCP layer expects
**discrete-time** dynamics.

A typical discrete-time model can be provided as follows:

```python
ocp.with_dynamics(
    lambda x, u, p:
    cs.vertcat(
        0.98 * cs.sin(x[0]) + x[1],
        0.1 * x[0]**2 - 0.5 * x[0] + p["a"] * x[1] + u[0]
    )
)
```

Note that the parameter argument behaves like a dictionary of CasADi
symbols, so named parameters can be accessed with expressions such as
`p["a"]`, `p["xref"]`, and so on.

### Continuous-time dynamics

If your model is given in continuous time as

<div class="math">
\[
    \dot{x} = f(x, u; p),
\]</div>

you can discretize it using `og.ocp.DynamicsDiscretizer`. This helper
constructs a discrete-time callback that is directly compatible with
`with_dynamics`.

For example, suppose

<div class="math">
\[
    \dot{x} = \begin{bmatrix}
    x_2 \\
    -x_1 + u
    \end{bmatrix}.
\]</div>

Then we can discretize it as follows:

```python
continuous_dynamics = lambda x, u, p: cs.vertcat(
    x[1],
    -x[0] + u[0]
)

discretizer = og.ocp.DynamicsDiscretizer(
    continuous_dynamics,
    sampling_time=0.1,
)

ocp.with_dynamics(discretizer.rk4())
```

### Available discretization methods

The following discretization methods are currently available:

- `euler()`
- `midpoint()`
- `heun()`
- `rk4()`
- `multistep(method=..., num_steps=...)`

For example, if a higher-accuracy internal integration is needed over one
sampling interval, we can use a multistep method:

```python
ocp.with_dynamics(
    discretizer.multistep(method="rk4", num_steps=4)
)
```

This subdivides the sampling interval into four smaller substeps and applies
RK4 on each one.

### Choosing a discretization

As a rule of thumb:

- `euler()` is the simplest and cheapest option
- `midpoint()` and `heun()` offer better accuracy than Euler at modest cost
- `rk4()` is a good default when accuracy matters
- `multistep(...)` is useful when the controller sampling time is relatively
  large compared to the time scale of the plant

In all cases, the result of the discretizer is a standard discrete-time
callback, so once it is passed to `with_dynamics`, the remainder of the OCP
construction is unchanged.

## Stage cost function

The stage cost is specified using `with_stage_cost`. The callback must accept
`(x, u, p, stage_idx)` and return the scalar quantity $\ell_t(x_t, u_t; p)$.
Here `x` is the current state, $x_t$, `u` is the current input, $u_t$,
`p` is the parameter view, `stage_idx` is the stage index, $t$.

### Typical quadratic stage cost

A common choice in optimal control is a quadratic tracking term plus a control
effort penalty:

<div class="math">
\[
    \ell_t(x_t, u_t; p)
    =
    q \|x_t - x^{\mathrm{ref}}\|^2 + r \|u_t - u^{\rm ref}\|^2.
\]</div>

This can be written as:

```python
def my_stage_cost(x, u, p, _t):
    xref = p["xref"]
    uref = p["uref"]
    ex = x - xref
    eu = u - uref
    return  p["q"] * cs.dot(ex, ex) + p["r"] * cs.dot(eu, eu)

ocp.with_stage_cost(my_stage_cost)
```

The stage index is written as `_t` above because it is not used in this
example.

### Time-varying stage costs

If the cost depends explicitly on the stage, the argument `stage_idx` can be
used directly. For example, a discounted quadratic stage cost can be written
as

$$\ell_t(x_t, u_t; p) = \gamma^t \left( q \Vert x_t - x^{\mathrm{ref}} \Vert^2 + r \Vert u_t - u^{\rm ref} \Vert^2 \right),$$

where $\gamma \in (0, 1]$ is a discount factor. This can be implemented
as follows (this time using a lambda):

```python
ocp.with_stage_cost(
    lambda x, u, p, t:
    (p["gamma"][0] ** t) * (
        p["q"] * cs.dot(x - p["xref"], x - p["xref"])
        + p["r"] * cs.dot(u - p["uref"], u - p["uref"])
    )
)
```

In that case, `gamma`, `q`, and `r` should be declared as parameters, for example:

```python
ocp.add_parameter("gamma", 1, default=0.95)
ocp.add_parameter("q", 1, default=1.0)
ocp.add_parameter("r", 1, default=0.5)
```

More generally, `stage_idx` can be used to encode stage-dependent references,
weights, or economic costs.

Note that the return value must be a scalar CasADi expression. 

## Terminal cost function

The terminal cost is specified using `with_terminal_cost`. The callback must
accept `(x, p)` and return the scalar quantity $V_f(x_N; p)$.
Here `x` is the terminal state $x_N$ and `p` is the parameter view.

A common choice is a quadratic tracking penalty of the form

<div class="math">
\[
    V_f(x_N; p) = \|x_N - x^{\mathrm{ref}}\|_{P}^{2},
\]</div>

where $P$ is a symmetric positive definite matrix. This can be
implemented as follows:

```python
# Terminal cost matrix
P = cs.DM([
    [10.0, 0.0],
    [0.0, 2.0],
])

# Terminal cost function
ocp.with_terminal_cost(
    lambda x, p:
    cs.mtimes([
        (x - p["xref"]).T,
        P,
        (x - p["xref"]),
    ])
)
```

## General symbolic constraints

In addition to hard constraints on the decision variables, the OCP API supports
general **symbolic constraints**. These are constraints defined by CasADi
expressions and handled either by the penalty method (PM) or by the augmented
Lagrangian method (ALM).

There are two variants:

- `with_path_constraint` for stage-wise constraints, evaluated at every
  stage $t = 0, \ldots, N-1$
- `with_terminal_constraint` for terminal constraints, evaluated at $x_N$

### Penalty-type symbolic constraints

Penalty constraints are appended to the mapping $F_2$ (see [PM docs](/optimization-engine/docs/python-interface#penalty-method)). 
Such sonstraints are treated through the penalty method.
For example, to impose the state inequality $x_{2,t} \geq x_{\min}$
we may write

```python
ocp.with_path_constraint(
    lambda x, u, p, _t: cs.fmax(0.0, -x[1] + p["xmin"]),
    kind="penalty",
)
```

Likewise, a terminal penalty constraint can be used to encode an inequality
such as

$$
x_{1,N} + x_{2,N} \leq 0.1.
$$

This can be written as

```python
ocp.with_terminal_constraint(
    lambda x, p: cs.fmax(0.0, x[0] + x[1] - 0.1),
    kind="penalty",
)
```

### ALM-type symbolic constraints

ALM constraints are appended to the mapping $F_1$ and must be combined
with a set :math:`C`, so that the constraint has the form

<div class="math">
\[
    F_1(x_t, u_t; p) \in C
\]</div>

or, for terminal constraints,

<div class="math">
\[
    F_1(x_N; p) \in C.
\]</div>

In that case, $C$ (`set_c`) must be provided.
For example, the inequality

<div class="math">
\[
    x_{2,t} - x_{\min} \geq 0
\]</div>

can be written as an ALM constraint by taking $C = [0, \infty)$, that is,

```python
set_c = og.constraints.Rectangle([0.0], [float("inf")])
ocp.with_path_constraint(
    lambda x, u, p, _t: x[1] - p["xmin"],
    kind="alm",
    set_c=set_c,
)
```

A terminal ALM constraint can be defined similarly:

```python
set_c = set_c=og.constraints.Rectangle([-0.1], [0.1])
ocp.with_terminal_constraint(
    lambda x, p: x[0],
    kind="alm",
    set_c=set_c,
)
```

An optional dual set `set_y` may also be provided, but in many cases it is
best to leave it unspecified and let the lower-level OpEn machinery choose it
(see the [documentation](/optimization-engine/docs/python-interface#augmented-lagrangian-method)).

## Next steps

Once you have defined your optimal control problem, the next step is to 
[build an optimizer](/optimization-engine/docs/python-ocp-3).
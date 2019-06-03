---
id: python-interface
title: Opengen
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>


We present `opengen`: a Python interface to Optimization 
Engine (OpEn), which facilitates the process of code generation
of high-performance parametric optimizers. 

<img src="/optimization-engine/img/open-functionality.jpg" alt="Opengen code generation" width="85%"/>


## About

Opegen is a Python package that facilitates the design of parametric
optimizers and generates Rust code using Optimization Engine. The user
designs the parametric optimization problem entirely in Python.
The auto-generated code is then compiled in Rust can be used in any
of the following ways:

- Directly in Rust (you can include it in you Rust project as a dependency)
- Over a TCP/IP interface based on JSON (which can be accessed easily from any programming language)
- In Python (using the TCP/IP interface in the background)
- In C or C++ (support will be released soon)

<img src="/optimization-engine/img/python-iface-workflow.jpg" alt="Opengen code generation" width="85%"/>

## Installation

In order to install `opengen` you need:

- [Python](https://www.python.org/) (tested against versions 2.7, 3.4 and 3.6)
- [Rust 2018](https://www.rust-lang.org/tools/install) (tested with nightly, beta and stable)

You may install `opengen` using `pip`; just open a terminal
and type in:

```
pip install opengen
```

On some linux systems you may need to prepend `sudo`. We recommend
that you use [virtualenv](https://virtualenv.pypa.io/en/latest/),
but it is not necessary.

## Getting started

### Problem specification 

Here we will demonstrate how to use `opengen` to generate a 
parametric optimizer

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)\\
\mathrm{subject\ to} &amp;\ \ u \in U\end{split}\]</div>

The first step is to import `opengen` and 
[`casadi`](https://web.casadi.org/) in your Python code:

```python
import casadi.casadi as cs
import opengen as og
```

Next, we define the cost function. Suppose that we want to minimize
the [Rosenbrock function](https://en.wikipedia.org/wiki/Rosenbrock_function)
defined as 

<div class="math">
\[f(u) = \sum_{i=1}^{n_u - 1} b (u_{i+1} - u_{i}^2)^2 + (a-u_i)^2\]</div>

where $u=(u_1,\ldots, u_{n_u})$ is the decision variable and 
$p=(a, b)$ is a parameter vector.

Consider the case with $n_u=5$. The cost function is defined 
using CasADi `SX` symbols as follows:

```python
u = cs.SX.sym("u", 5)                 # decision variable (nu = 5)
p = cs.SX.sym("p", 2)                 # parameter (np = 2)
phi = og.functions.rosenbrock(u, p)   # cost function
```

Next, we need to define the constraints. Support that 
$U$ is a Euclidean ball with radius $r=1.5$ centered at 
the origin, $U= \\{u \in \mathbb{R}^5 {}:{} \|u\| \leq r \\}$.
This is,

```python
bounds = og.constraints.Ball2(None, 1.5)  # ball centered at origin
``` 

Alternatively, you may choose `bounds` to be a box of 
the form 

<div class="math">
\[U = \{u \in \mathbb{R}^{n_u} {}:{} u_{\min}
\leq u \leq u_{\max} \}\]</div>
 
using `og.constraints.Rectangle`. Set `U` is a simple set (typically
a ball or a box). Opegen supports more general constraints of the 
form $c(u; p) = 0$ and $c(u; p) \leq 0$ 
(see [next section](#general-constraints)). 

We may now define the optimization problem as follows:

```python
problem = og.builder.Problem(u, p, phi).with_constraints(bounds)
```

### Code generation

In order to generate a parametric optimizer in Rust for the 
above problem we need to specify the following information
which, albeit optional, will need to be configured and customized
in most cases

- Metadata: general metadata about the auto-generated solver;
  The most important piece of information here is the 
  `optimizer_name`. The optimizer will be stored in a folder
  with the same name
- Build configuration: build options such as the path where the 
  generated optimizer should be stored and the build mode
  (debug or release)
- Solver configuration: solver-specific parameters such as the 
  tolerance, L-BFGS memory length and more

Here is an example of metadata:

```python
meta = og.config.OptimizerMeta()                \
    .with_version("0.0.0")                      \
    .with_authors(["P. Sopasakis", "E. Fresk"]) \
    .with_licence("CC4.0-By")                   \
    .with_optimizer_name("the_optimizer")
```

Note that `with_authors` takes a list of strings. The licence
is either the name of a licence or a URL. The optimizer name 
is the name of the auto-generated crate and the name of the 
folder in which all generated files will be stored.

Next, let us create a basic build configuration:

```python
build_config = og.config.BuildConfiguration()  \
    .with_build_directory("python_build")      \
    .with_build_mode("debug") 
```

This will instruct opegen to store the generated optimizer in
`python_build/the_optimizer`. The build mode can be either 
`debug` (fast compilation, suboptimal performance) or 
`release` (slow compilation, best performance). 

There are some additional parameters one might be interested in
such as `with_rebuild(True/False)` (whether to re-build the 
optimizer from scratch - equivalent of clean+build). 

Lastly, one needs to specify certain solver parameters; an 
example is given below:

```python
solver_config = og.config.SolverConfiguration()   \
            .with_lfbgs_memory(15)                \
            .with_tolerance(1e-5)                 \
            .with_max_inner_iterations(155)
```

Method `.with_max_inner_iterations` is used to specify the 
maximum number of iterations. We refer to it as "inner" iterations
to distinguish from "outer" iterations in the [penalty method](https://en.wikipedia.org/wiki/Penalty_method)
(see below).

We are now ready to call the code generator providing 
the above information:

```python
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          metadata=meta,
                                          build_configuration=build_config,
                                          solver_configuration=solver_config)
builder.enable_tcp_interface()
builder.build()
```

Method `.enable_tcp_interface()` will activate the generation of a 
TCP/IP interface with which we will be able to call the solver from 
Python and other languages. You do not need to activate it if you 
plan to use the optimizer on an embedded device. 

*Note.* You may configure the TCP parameters of the server using 
`og.config.TcpServerConfiguration`. By default, the server will 
bind on `127.0.0.1` and will listen for requests at port `4598`.



### Calling the optimizer

Provided you have generated code using `.enable_tcp_interface()`,
you will be able to call it from Python.

All you need to do is create an instance of `OptimizerTcpManager`
providing the path to your optimizer.

The typical steps involved in calling an optimizer over its
TPC/IP interface are:

- Start the server (starts in a separate thread and runs as 
  a sub-process in the background)
- Consume it
- Stop it

The following code snippet should be self explanatory

```python
mng = og.tcp.OptimizerTcpManager('python_build/the_optimizer')
mng.start()

pong = mng.ping()                 # check if the server is alive
print(pong)
solution = mng.call([1.0, 50.0])  # call the solver over TCP
print(solution)

mng.kill()
```

The ping will return the dictionary `{"Pong":1}` and `solution`
is also a dictionary containing the solution and other useful
solver information:

```json
{
  "exit_status": "Converged", 
  "num_outer_iterations": 1, 
  "num_inner_iterations": 28, 
  "last_problem_norm_fpr": 6.415335992359856e-06, 
  "max_constraint_violation": 0.0, 
  "solve_time_ms": 4.39725, 
  "solution": [
    0.9064858741205364, 
    0.8255064578408071, 
    0.6864317421667316, 
    0.4750545892829331, 
    0.22352733342511494 ]
}
```

Alongside the solution vector (`solution`), the solver returns 
other useful information such as the solution status (`Converged`),
the total number of iterations (`num_inner_iterations`), the norm
of the fixed-point residual (`last_problem_norm_fpr`) which 
quantifies the solution quality  and the solution time in milliseconds
(`solve_time_ms`). The parameters `num_outer_iterations` and 
`max_constraint_violation` will be elucidated in the next section.



## General constraints

Opengen can handle more general constraints of the form 
$c(u; p) = 0$ using the [penalty method](https://en.wikipedia.org/wiki/Penalty_method). 
In particular, opengen can solve problems of the form 

<div class="math">
\[\begin{split}
\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}& \ f(u; p)\\
\mathrm{subject\ to}\, &u \in U,
\\ 
&c(u; p) = 0,\end{split}\]</div> 

where $c:\mathbb{R}^{n_u}\times \mathbb{R}^{n_p} \to \mathbb{R}^{n_c}$. 
This can be used to encode either equality or inequality constraints 
of the form $\psi(u; p) \leq 0$ using 

<div class="math">
\[c(u; p) = \max\{0, \psi(u; p)\}.\]</div>

As an example, consider again the problem of minimizing the 
Rosenbrock function subject to the constraints

<div class="math">
\[\begin{split}
1.5u_1 - u_2 {}={}& 0,\\
u_3 - u_4 + 0.1 {}\leq{}& 0.
\end{split}\]</div>

To that end, we define the constraints function

<div class="math">
\[c(u; p) = 
\begin{bmatrix}
  1.5u_1 - u_2
  \\
  \max\{0, u_3 - u_4 + 0.1\}
\end{bmatrix}.\]</div>

It is now very easy to include this in out problem formulation.
We first need to define function $c$:

```python
c = cs.vertcat(1.5 * u[0] - u[1], 
               cs.fmax(0.0, u[2] - u[3] + 0.1))
```

and include it in the problem formulation:

```python
problem = og.builder.Problem(u, p, phi)  \
    .with_penalty_constraints(c)         \
    .with_constraints(bounds)
```

The rest of the code remains the same. The solution will
now be

```json
{
  "exit_status": "Converged", 
  "num_outer_iterations": 6, 
  "num_inner_iterations": 35, 
  "last_problem_norm_fpr": 3.1976300788348913e-06, 
  "max_constraint_violation": 7.659485695667156e-05, 
  "solve_time_ms": 3.842833, 
  "solution": [
    0.02196420476411661, 
    0.032927537058737015, 
    -0.031152544256866314, 
    0.06877086088617702, 
    0.004151858447457505 ]
}
```

Note here that the solver performed 6 outer 
[penalty-type iterations](https://en.wikipedia.org/wiki/Penalty_method)
and 35 inner iterations overall. The infinity norm of the constraint
violations is approximately $7.66\cdot 10^{-5}$ (which is below the 
default tolerance of $10^{-4}$. This means that the solution $u^\star$
satisfies

<div class="math">
\[
\|c(u^\star; p)\|_{\infty}\leq 7.66\cdot 10^{-5}.
\]
</div>

The user may change the maximum constraint violation using 

```python
solver_config.with_constraints_tolerance(1e-6)
``` 



## Advanced options

### Solver options
The user may wish to modify some additional solver parameters.

When using the penalty method to account for general constraints,
the most important parameters which determine the speed of 
convergence are the **initial values** of the penalty weights and the 
**update factor**. These are set using

```python
solver_config.with_penalty_weight_update_factor(8.0)       \
             .with_initial_penalty_weights([20.0, 5.0])
``` 

The number of the initial penalty weights must be equal to 
$n_c$; the number of constraints. If you need to set all 
weights to the same value, use

```python
solver_config.with_initial_penalty_weights(100.0)
```

In embedded applications it is often important that the solver
is forced to terminate within a given time period. The user may
set the **maximum runtime** (in microseconds) using

```python
# Maximum duration: 50ms
solver_config.with_max_duration_micros(50000)
```

Lastly, the maximum number of outer iterations can be set using

```python
solver_config.with_max_outer_iterations(num_out_iters)
```

The number of outer iterations should be kept reasonably small
to avoid too high values of the penalty parameters (which increase
exponentially).


### Build options

During the design phase, one needs to experiment with the problem
formulation and solver parameters. This is way the default build
mode is the "debug" mode, which compiles fast, but it suboptimal.
Building in "release" mode takes slightly longer to compile, but
can lead to a significant speed-up. To do so, use the option

```python
build_config.with_build_mode("debug")
```

*Note.* Coming soon: cross-compilation for different targets
(e.g., a Raspberry Pi).
 
### TCP/IP options

In order to change the IP and port at which the server listens
for requests (e.g., for remote connections), you may crate an 
`TcpServerConfiguration` object as follows

```python
tcp_config = og.config.TcpServerConfiguration('10.8.0.12', 9555)
```

and then provide it to the builder using 

```python
builder.enable_tcp_interface(tcp_config)
```



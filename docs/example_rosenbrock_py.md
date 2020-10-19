---
id: example_rosenbrock_py
title: Rosenbrock Function
description: Minimization of the Rosenbrock function using OpEn
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

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

<!--DOCUSAURUS_CODE_TABS-->

<!--Python-->
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
    .with_build_directory("my_optimizers") \
    .with_build_mode("debug")                  \
    .with_tcp_interface_config()
meta = og.config.OptimizerMeta()                   \
    .with_optimizer_name("rosenbrock")
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(1e3)                     \
    .with_penalty_weight_update_factor(5)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config, solver_config)
builder.build()

# Use TCP server
# ------------------------------------
mng = og.tcp.OptimizerTcpManager('my_optimizers/rosenbrock')
mng.start()

mng.ping()
server_response = mng.call([1.0, 50.0])

if server_response.is_ok():
    solution = server_response.get()
    u_star = solution.solution
    status = solution.exit_status

mng.kill()
```

<!--MATLAB-->
```matlab
% NOTE: The MATLAB API is currently unstable and very likely to change
% Define variables
% ------------------------------------
u = casadi.SX.sym('u', 5);
p = casadi.SX.sym('p', 2);

% Define cost function and constraints
% ------------------------------------
phi = rosenbrock(u, p);
f2 = [1.5*u(1) - u(2);
      max(0, u(3)-u(4)+0.1)];

bounds = OpEnConstraints.make_ball_at_origin(5.0);

opEnBuilder = OpEnOptimizerBuilder()...
    .with_problem(u, p, phi, bounds)...
    .with_build_name('penalty_new')...
    .with_fpr_tolerance(1e-5)...
    .with_constraints_as_penalties(f2);

opEnOptimizer = opEnBuilder.build();

% Consume
% ------------------------------------
clc;
opEnOptimizer.run();
opEnOptimizer.connect();
out = opEnOptimizer.consume([0.5,2])
out = opEnOptimizer.consume([0.5,2])
opEnOptimizer.stop();

```

<!--END_DOCUSAURUS_CODE_TABS-->

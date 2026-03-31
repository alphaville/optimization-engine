---
id: matlab-interface
title: MATLAB
description: OpEn's MATLAB interface
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

<font color="blue"><b>IMPORTANT NOTE:</b> IN OpEn version 0.6.0 we have added support 
for the augmented Lagrangian and penalty methods; this is not available through the 
MATLAB interface at the moment. We would advise that the users use [`opengen`] in Python
instead. The MATLAB interface will be updated soon - [stay tuned]</font>

[`opengen`]: ./python-interface
[stay tuned]: https://twitter.com/isToxic

## About the MATLAB interface
If you haven't done so already, you first need to [install the MATLAB interface](./installation#matlab-interface).

![matlab](/optimization-engine/img/matlab_logo.png)

This inteface allows the designers to prepare their parametric optimizers in MATLAB and then use the code generation tool we provide to build their parametric optimizer in Rust. The generated code can either be used directly in Rust, or, it can be interfaced from **any software**.

![OpEn code generation](/optimization-engine/img/115ba54c2ad0.gif "Quick code generation")

In other words, MATLAB is used to **design** a parametric optimizer, which runs in Rust and can be interfaced from **any** programming language or operating system.

The generated optimzer runs in Rust and exports its functionality using **UDP sockets**.

Let's now walk through the necessary steps to design a parametric optimizer.

## Problem definition

Recall that we need to define a parametric optimization problem of the form

<div class="math">
\[\begin{split}\operatorname*{Minimize}_{u {}\in{} \mathbb{R}^{n_u}}&amp;\ \ f(u; p)\\
\mathrm{subject\ to} &amp;\ \ u \in U(p)\end{split}\]</div>

where $u$ is the decision variable of the problem and $p$ is a parameter.

We, therefore, need to define the cost function and the set of constraints. The gradient of the cost function will be computed automatically (CasADi takes care of that).


### Cost function
Determining the cost function is straightforward. We simply need to create two CasADi symbols, `u` and `p`, for the decision variable and the parameter respectively. These symbols need to have specified dimensions (`nu` and `np` respectively).

```matlab
nu = 6;                        % number of decision variables
np = 2;                        % number of parameters

u = casadi.SX.sym('u', nu);    % decision variables
p = casadi.SX.sym('p', np);    % parameters

phi = rosenbrock(u, p);        % cost function phi(u; p)
```

### Constraints

Constraints fall into two sub-categories: non-parametric constraints and parametric ones (which are under development). Non-parametric constraints are specified during design time and cannot be updated in real time. Parametric constraints depend on the parameter `p` and are updated at every invocation of the solver.

Non-parametric constraints are implemented by `@OpEnConstraints`, a MATLAB class which follows the Factory pattern. Constraint objects are created by calling a static function. For example, to create the constraint

<div class="math">
\[\|u\| {}\leq{} 1\]
</div>

we construct an instance of `OpEnConstraints` as follows:


```matlab
constraints = OpEnConstraints.make_ball_at_origin(1.0);
```

## Building a parametric optimizer

### Code Generation

Code generation is as simple as

```matlab
builder = OpEnOptimizerBuilder().with_problem(u, p, phi, constraints);
optimizer = builder.build();
```

As shown above, we first need to create an instance of **OpEnOptimizerBuilder**; this is a builder that will take care of code generation and will allow us to call the generated parametric optimizer from MATLAB.


The builder needs to know the problem specifications, that is, the cost function and the constraints (which we provide using `with_problem`).

We then call `build()` to create an optimizer -- an instance of `OpEnOptimizer`.



### Using the Rust module
The auto-generated Rust module exposes its functionality over a JSON-based communication protocol over a UDP socket. For more information, read the [**UDP communication documentation page**](./udp-sockets).

The auto-generated Rust module can be used directly from MATLAB; it can be started, used and terminated.

First, we start the module:

```matlab
optimizer.run();
```

Then, we connect to the optimizer to be able to use it:

```matlab
optimizer.connect();
```

We may now use the parametric optimizer:

```matlab
p = [1, 250]; % parameter
out = optimizer.consume(p);
```

This request returns a MATLAB structure like the following one:

```text
     p: [2×1 double]
     u: [6×1 double]
     n: 77
     f: -5.369481020006234
    dt: '19.276626ms'
```

where `p` is the provided parameter, `u` is the solution, `n` is the number of iterations, `f` is the logarithm of the fixed-point residual and `dt` is the elapsed time (with units of measurement).

We may call the server several times. Once we're done, we should close the UDP connection and stop the server.

```matlabf
optimizer.disconnect();
optimizer.stop();
```

## Advanced configuration

Having created a builder (an instance of `OpEnOptimizerBuilder`) we may override some of the default properties using certain methods as explained in what follows.

### Solver parameters
We may specify/update the length of the L-BFGS  buffer, the tolerance and the maximum number of iterations as follows:

```matlab
builder.with_lbfgs_memory(15)...
       .with_max_iterations(2500)...
       .with_fpr_tolerance(1e-8);
```

### Connection properties
By default the UDP interface binds at the local address `127.0.0.1` and the default port `3498`.

We may update the port using:

```matlab
builder.with_local_udp_at_port(4567);
```

or, we may also allow the module to bind on any IP. This can be done if we intend to access the parametric optimizer remotely:

```matlab
builder.with_public_udp_at_port(4567);
```

We may also specify a custom IP at which the optimizer should bind using:

```matlab
builder.with_bind_address('10.8.1.6', 4567);
```

### Metadata and other properties

The generated optimizer will be stored in `{OPTIMIZER_BUILD_PATH}/{OPTIMIZER_NAME}`; we may specify a custom path and a custom name for your optimizer as follows:

```matlab
builder.with_build_path('/path/to/build/directory')...
       .with_build_name('turbo_optimizer');
```
The default path is `{OPTIMIZATION_ENGINE_ROOT}/build/autogenerated_optimizer/`.

You may also choose whether the optimizer should be compiled in `debug` (default) or `release` mode. For best performance, change the build mode to `release` as follows

```matlab
builder.with_build_mode('release');
```

You can choose whether the optimizer will run on your system or whether it should be cross-compiled for some other system (e.g., a Raspberry Pi, or some ARM-based system).

For example, to cross-compile for Raspberry Pi specify the following target:


```matlab
builder.with_target_system('rpi');
```

You may also specify the authors and license of this optimizer using

```matlab
builder.with_authors('P. Sopasakis <p.sopasakis@example.com>', ...
                    'E. Fresk <e.fresk@example.com>')...
       .with_license('MIT and APACHE-v2')
       .with_version('0.1.5');
```


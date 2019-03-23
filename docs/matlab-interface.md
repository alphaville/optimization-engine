---
id: matlab-interface
title: MATLAB
---

## About the MATLAB interface
If you haven't done so already, you first need to [install the MATLAB interface](./installation#matlab-interface).

This inteface allows the designers to prepare their parametric optimizers in MATLAB and then use the code generation tool we provide to build their parametric optimizer in Rust. The generated code can either be used directly in Rust, or, it can be interfaced from **any software**. 

In other words, MATLAB is used to **design** a parametric optimizer, which runs in Rust and can be interfaced from **any** programming language or operating system.

The generated optimzer runs in Rust and exports its functionality using **UDP sockets**.

Let's now walk through the necessary steps to design a parametric optimizer.

## Problem definition

Recall that we need to define a parametric optimization problem of the form

```text
Minimize f(u; p)
subject to: u in U(p)
```

where `u` is the decision variable of the problem and `p` is a parameter.

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

Non-parametric constraints are implemented by `@OpEnConstraints`, a MATLAB class which follows the Factory pattern. Constraint objects are created by calling a static function. For example, to create the constraint `|u| <= 1.0`, we construct an instance of `OpEnConstraints` as follows:


```matlab
constraints = OpEnConstraints.make_ball_at_origin(1.0);
```

## Code Generation

Code generation is as simple as 

```matlab
builder = OpEnOptimizerBuilder().with_problem(u, p, cost, constraints);
optimizer = builder.build();
```

As shown above, we first need to create an instance of **OpEnOptimizerBuilder**; this is a builder that will take care of code generation and will allow us to call the generated parametric optimizer from MATLAB.


The builder needs to know the problem specifications, that is, the cost function and the constraints (which we provide using `with_problem`).

We then call `build()` to create an optimizer -- an instance of `OpEnOptimizer`.



## Using the Rust module
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

## Configuring the Builder

Coming soon
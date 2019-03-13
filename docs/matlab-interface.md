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

### In a nutshell
Code generation is as simple as 

```matlab
build_config = open_build_config();
open_generate_code(build_config, constraints, u, p, phi);
```


### Build configuration
The default build configuration is a structure returned by `build_config = open_build_config();`. This specifies the destination path where the parametric optimizer will be stored, various metadata (authorship, license, etc), solver parameters, and parameters related to the interface. Let us go through these in detail. 

#### Basic configuration
The following properties are metadata:

| Property       | Description                                  |
|----------------|----------------------------------------------|
| `build_name`   | Name of your optimizer (same as folder name) |
| `version`      | Version of your optimizer                    |
| `license`      | String of license                            |
| `authors`      | Cell array with author names                 |


Solver properties:

| Property                | Description                    |
|-------------------------|--------------------------------|
| `solver.lbfgs_mem`      | Length of L-BFGS buffer        |
| `solver.max_iters`      | Maximum number of iterations   |
| `solver.tolerance`      | Tolerance                      |

UDP socket properties:

| Property                       | Description        |
|--------------------------------|--------------------|
| `udp_interface.bind_address`   | Bind address       |
| `udp_interface.port`           | Post (integer)     |



#### Advanced configuration

The following properties are some paths, which you shouldn't need to modify:

| Property                     | Description                                  |
|------------------------------|----------------------------------------------|
| `optimization_engine_path`   | Path of optimization-engine                  |
| `icasadi_path`               | Path of icasadi (casadi Rust interface)      |
| `build_path`                 | Directory where the optimizer will be stored |
| `build_mode`                 | `debug` or `release`                         |
| `target`                     | target hardware (`rpi` for Raspberry Pi)     |






## Using the Rust module
The auto-generated Rust module exposes its functionality over a JSON-based communication protocol over a UDP socket. For more information, read the [**UDP communication documentation page**](./udp-sockets).

The auto-generated Rust module can be used directly from MATLAB; it can be started, used and terminated.

First, we start the module:

```matlab
run_parametric_optimizer(build_config);
```

Then, we open a UDP connection to the target:

```matlab
ip_address = '127.0.0.1';
udp_connection = udp(ip_address, build_config.udp_interface.port);
fopen(udp_connection);
```

We may now use the parametric optimizer:

```matlab
p = [1, 250]; % parameter
[out, json_response] = consume_parametric_optimizer(udp_connection, p);
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

```matlab
fclose(udp_connection);
kill_parametric_optimizer(ip_address, build_config.udp_interface.port);
```
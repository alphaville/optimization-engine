---
id: python-ros
title: Generation of ROS packages
sidebar_label: ROS packages
description: Code generation for ROS packages using OpEn in Python
---

## What is ROS
The [Robot Operating System](https://www.ros.org/) (ROS) is a collection of tools and libraries, as well as a framework that facilitates the data exchange among them. ROS is popular in the robotics community and is used to design and operate modern robotic systems.

In ROS all functionality is implemented in **nodes** which communicate with each other by **messages**. A node outputs data on a **topic** and can subscribe to another node's topic to read its data. For example, a *controller* node can subscribe to a few sensor topics, read their measurements, compute a control action and publish its control action on another topic.

## ROS + OpEn

OpEn (with opengen version `0.5.0` or newer) can generate ready-to-use ROS packages very easily.

### Messages


#### Input Parameters

The input parameters message follows the following specification:

```
float64[]      parameter         # parameter p (mandatory)
float64[]      initial_guess     # u0 (optional/recommended)
float64[]      initial_y         # y0 (optional)
float64        initial_penalty   # initial penalty (optional)
```

An example of such a message is

```yaml
parameter: [15.0, 2.1]
initial_guess: [0.5, 0.5, 0.5, 0.5, 0.5]
initial_penalty: 1000,
initial_y: []
```


#### Result

A result message (`OptimizationResult`) contains the solution of the parametric optimization problem and details about the solution procedure such as the number of inner/outer iterations and the solution time. The result of an auto-generated OpEn node is a message with the following specification:

```
# Constants match the enumeration of status codes
uint8 STATUS_CONVERGED=0
uint8 STATUS_NOT_CONVERGED_ITERATIONS=1
uint8 STATUS_NOT_CONVERGED_OUT_OF_TIME=2
uint8 STATUS_NOT_CONVERGED_COST=3
uint8 STATUS_NOT_CONVERGED_FINITE_COMPUTATION=4

float64[]    solution              # optimizer (solution)
uint8        inner_iterations      # number of inner iterations
uint16       outer_iterations      # number of outer iterations
uint8        status                # status code
float64      cost                  # cost value at solution
float64      norm_fpr              # norm of FPR of last inner problem
float64      penalty               # penalty value
float64[]    lagrange_multipliers  # vector of Lagrange multipliers
float64      infeasibility_f1      # infeasibility wrt F1
float64      infeasibility_f2      # infeasibility wrt F2
float64      solve_time_ms         # solution time in ms
```

An example of such a message is given below:

```yaml
solution: [0.5317, 0.7975, 0.6761, 0.7760, 0.5214]
inner_iterations: 159
outer_iterations: 5
status: 0
norm_fpr: 2.142283848e-06
penalty: 111250.0
lagrange_multipliers: []
infeasibility_f1: 0.0
infeasibility_f2: 2.44131958366e-05
solve_time_ms: 2.665959
```

### Configuration Parameters

You can set certain configuration parameters of your ROS package both when you construct it (see [below](#ros-package-generation-in-python)) as well as after you have generated the code. In particular, you may configure:

- the rate at which your node runs
- the name of the input topic
- the name of the output topic

by modifying the contents of `config/open_params.yaml`. This is an auto-generated configuration file that looks like this:

```yaml
solution_topic: "solution"
params_topic: "parameters"
rate: 35
```

### 

## Code generation

### ROS Package Generation in Python 
To generate a ROS package you can use opengen - OpEn's Python interface. You will have to provide certain configuration parameters, such as the package name, the node name and the rate of your node in Hz.

To do so you need to construct a `RosConfiguration` object:

```py
ros_config = og.config.RosConfiguration()       \
    .with_package_name("parametric_optimizer")  \
    .with_node_name("open_node")                \
    .with_rate(35) 
```

Then apply `.with_ros(ros_config)` on your build configuration.

When you compile with this option, the generation of C/C++ bindings is activated. This is because the auto-generated ROS package calls your optimizer via its C++ interface.


### Example

```py
import opengen as og
import casadi.casadi as cs

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = cs.vertcat(1.5 * u[0] - u[1],
               cs.fmax(0.0, u[2] - u[3] + 0.1))
bounds = og.constraints.Ball2(None, 1.5)
problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c)        \
    .with_constraints(bounds)
meta = og.config.OptimizerMeta()                \
    .with_optimizer_name("rosenbrock")          \
    .with_version('0.1.1')                      \
    .with_licence('LGPLv3')
ros_config = og.config.RosConfiguration()       \
    .with_package_name("parametric_optimizer")  \
    .with_node_name("open_node")                \
    .with_rate(35)                              \
    .with_description("cool ROS node")
build_config = og.config.BuildConfiguration()   \
    .with_build_directory("my_optimizers")      \
    .with_tcp_interface_config()                \
    .with_build_c_bindings()                    \
    .with_ros(ros_config)
solver_config = og.config.SolverConfiguration()    \
    .with_tolerance(1e-5)                          \
    .with_delta_tolerance(1e-4)                    \
    .with_initial_penalty(890)                     \
    .with_penalty_weight_update_factor(5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config) 
builder.build()
```

The above program will generate a parametric optimizer at `my_optimizers/rosenbrock`. The ROS package will be in `my_optimizers/rosenbrock/parametric_optimizer`.

An example of an auto-generated ROS package is available on this [github repository](https://github.com/alphaville/open_ros).

<div class="alert alert-info">
<b>See also:</b> this <a href="example_navigation_ros_codegen">complete example</a> on how to use OpEn to build an NMPC controller on a <em>Husky</em> ground vehicle.</div>


### Use the auto-generated ROS package
OpEn generates a README file that you will find in `my_optimizers/rosenbrock/parametric_optimizer` with detailed instructions on how to build, run and configure your new ROS package. We recommend that you use `catkin` to build it. In brief, you have to:

- Create a soft symbolic link (using `ln -s`) of the auto-generated package into `catkin_ws/src`
- Compile with `catkin_make`
- Run using the auto-generated launch file

Check out the auto-generated README.md file for details.
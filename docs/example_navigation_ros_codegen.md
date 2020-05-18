---
id: example_navigation_ros_codegen
title: Navigation of ground vehicle using ROS codegen
description: Nonlinear model predictive control with OpEn using ROS
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

Example contributed by Guido Sanchez ([Github](https://github.com/gmsanchez/))

## Auto-generated ground vehicle navigation for ROS

We will create a MPC controller for position tracking of a [Husky](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/) ground vehicle. Husky is a medium sized robotic development platform fully supported in ROS with community driven Open Source code and examples.

<img src="/optimization-engine/img/husky.jpg" alt="Husky Ground Vehicle">

The vehicle can be modeled using the differential drive equations:

<div class="math">
\[\begin{split}
\dot{x}(t) {}={}&amp; v(t) \cos \theta(t),\\
\dot{y}(t) {}={}&amp; v(t) \sin \theta(t),\\
\dot{\theta}(t) {}={}&amp; \omega(t).
\end{split}\]
</div>

We want to solve the following optimal control problem

<div class="math">
\[
    \begin{align}
    \mathbb{P}(z){}:{}\operatorname*{Minimize}_{u_0, \ldots, u_{N-1}}& \sum_{i=1}^{N - 1} 
        \|x_t-x^{\mathrm{ref}}\|_Q^2 + \|u_t\|_R^2 + \|x_N-x^{\mathrm{ref}}\|_{Q_N}^2
    \\
    \text{subject to: }& x_{t+1} = f(x_t, u_t), t=0,\ldots, N-1
    \\
    &u_{\min} \leq u_t \leq u_{\max}, t=0,\ldots, N-1
    \\
    &x_0=z
    \end{align}
\]</div>

where $x = (p_x,p_y,\theta)$ is the position and orientation of the vehicle,
$x^{\mathrm{ref}}$ is the target position and orientation, $u = (v, \omega)$ is the linear and angular velocity of the vehicle and $f$ describes 
the vehicle dynamics, which in this example is

<div class="math">
\[
    f(x, u) = \begin{bmatrix}
    p_x + t_s (v \cos\theta)
    \\
    p_y + t_s (v \sin\theta )
    \\
    \theta + t_s \omega
    \end{bmatrix}
\]</div>

The auto generated controller will allow you to move the vehicle to any point of the simulation grid, as shown in the following video

<video width="960" height="540" controls>
  <source src="/optimization-engine/img/husky_video.mp4" type="video/mp4">
</video>

### Code generation for the MPC controller

To generate a ROS package you can use opengen - OpEn's Python interface (with opengen version `0.5.0` or newer). You will have to provide certain configuration parameters, such as the package name, the node name and the rate of your node in Hz.

### Install opengen 0.5.0 in a virtual environment

```bash
cd ~
mkdir ~/open_ros_codegen
cd ~/open_ros_codegen
virtualenv -p python3.6 venvopen
source venvopen/bin/activate
pip install opengen==0.5.0
pip install matplotlib
```

### Run the example code generation program

Let's make a folder for the code generation script

```bash
mkdir -p ~/open_ros_codegen/nmpc_open
cd ~/open_ros_codegen/nmpc_open
```

Create a file named `create_open_solver.py` inside the `nmpc_open` folder with the following content

```python
import casadi as cs
import opengen as og
import numpy as np

N = 10  # The MPC horizon length
NX = 3  # The number of elements in the state vector
NU = 2  # The number of elements in the control vector
sampling_time = 0.1
NSim = 100

Q = cs.DM.eye(NX) * [1.0, 1.0, 0.0001]
R = cs.DM.eye(NU) * [0.1, 50.0]
QN = cs.DM.eye(NX) * [10.0, 10.0, 0.0001]


def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])


def dynamics_dt(x, u):
    dx = dynamics_ct(x, u)
    return cs.vcat([x[i] + sampling_time * dx[i] for i in range(NX)])


# The stage cost for x and u
def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])


# The terminal cost for x
def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QN, dx])


x_0 = cs.MX.sym("x_0", NX)
x_ref = cs.MX.sym("x_ref", NX)
u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]

# Create the cost function
x_t = x_0
total_cost = 0

for t in range(0, N):
    total_cost += stage_cost(x_t, u_k[t], x_ref)  # update cost
    x_t = dynamics_dt(x_t, u_k[t])  # update state

total_cost += terminal_cost(x_t, x_ref)  # terminal cost

optimization_variables = []
optimization_parameters = []

optimization_variables += u_k
optimization_parameters += [x_0]
optimization_parameters += [x_ref]

optimization_variables = cs.vertcat(*optimization_variables)
optimization_parameters = cs.vertcat(*optimization_parameters)

umin = [-2.0, -1.0] * N  # - cs.DM.ones(NU * N) * cs.inf
umax = [2.0, 1.0] * N  # cs.DM.ones(NU * N) * cs.inf

bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(optimization_variables,
                             optimization_parameters,
                             total_cost) \
    .with_constraints(bounds)

ros_config = og.config.RosConfiguration() \
    .with_package_name("open_nmpc_controller") \
    .with_node_name("open_mpc_controller_node") \
    .with_rate((int)(1.0/sampling_time)) \
    .with_description("Cool ROS node.")

build_config = og.config.BuildConfiguration()\
    .with_build_directory("optimization_engine")\
    .with_build_mode("release")\
    .with_build_c_bindings() \
    .with_ros(ros_config)

meta = og.config.OptimizerMeta()\
    .with_optimizer_name("mpc_controller")

solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()
```

The above program will generate a parametric optimizer at `optimization_engine/mpc_controller`. The ROS package will be in `optimization_engine/mpc_controller/open_nmpc_controller`.

## Create a ROS workspace for our new package

We must create a ROS catkin workspace and place our package inside the `src` folder. We will do this by creating a symbolic link. After that, we can build and test our package.

```bash
mkdir -p ~/open_ros_codegen/catkin_ws/src
cd ~/open_ros_codegen/catkin_ws/src
ln -s ~/open_ros_codegen/nmpc_open/optimization_engine/mpc_controller/open_nmpc_controller .
cd ~/open_ros_codegen/catkin_ws
catkin build
source ~/open_ros_codegen/catkin_ws/devel/setup.bash
roslaunch open_nmpc_controller open_optimizer.launch
```

Right now we have a running ROS node with the defualt configuration. Now we must edit it a little bit to drive our vehicle.

## Download and run Husky packages

```bash
cd ~/open_ros_codegen/catkin_ws/src
git clone https://github.com/husky/husky.git
catkin build
```
Now we can run the Husky simulation as follows

``roslaunch husky_gazebo husky_empty_world.launch``

## Edit the auto-generated node

The auto generated ROS package (as stated in the [documentation](https://alphaville.github.io/optimization-engine/docs/python-ros)) creates two topics:
1. A topic that waits for the input parameters of the optimizer.
2. A topic that outputs the result given the input parameters.

We need to modify this behavior in order to drive the vehicle from one point to another. Our node should know where the vehicle is in order to obtain the correct control commands, so we must listen to the topic `/odometry/filtered`. This topic is already configured in the Husky package and publishes the vehicle [Odometry](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) data: position and orientation in 3D.

Given our current position and orientation, we will obtain the NMPC solution and publish it as a [Twist](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) message to the `/husky_velocity_controller/cmd_vel` topic. Once again, the topic is already configured in the Husky package, we just need to publish the desired velocity commands.

Finally, we will add a subscriber that listens on the `/open_nmpc_controller/command/pose` topic and expects a [PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html) message. We will use the `position.x` and `position.y` fields for the x-y reference and the `orientation.w` for the yaw angle reference (in degrees).

In order to accomplish that, we must modify the auto generated `open_optimizer.cpp` file. We need to add a subscriber that listens to the `odometry/filtered` topic and a publisher that publishes data to the `/husky_velocity_controller/cmd_vel` topic. The following code is a modification of the auto generated code that adds the needed functionality.

```c++
/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 * Generated at 2020-05-07 01:22:27.483106.
 */
#include "ros/ros.h"
#include "open_nmpc_controller/OptimizationResult.h"
#include "open_nmpc_controller/OptimizationParameters.h"
#include "mpc_controller_bindings.hpp"
#include "open_optimizer.hpp"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"


namespace open_nmpc_controller {
/**
 * Class open_nmpc_controller::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {

private:
    open_nmpc_controller::OptimizationParameters params;
    open_nmpc_controller::OptimizationResult results;
    double p[MPC_CONTROLLER_NUM_PARAMETERS] = { 0 };
    double u[MPC_CONTROLLER_NUM_DECISION_VARIABLES] = { 0 };
    double *y = NULL;
    
    static const int NX = 3;
    static const int NU = 2;
    
    double current_pos[NX] = {0};
    double current_ref[NX] = {0};

    mpc_controllerCache* cache;
    double init_penalty = ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_MPC_CONTROLLER_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == MPC_CONTROLLER_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < MPC_CONTROLLER_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == MPC_CONTROLLER_N1) {
            for (size_t i = 0; i < MPC_CONTROLLER_N1; ++i)
                y[i] = params.initial_y[i];
		}

    }

    /**
     * Call OpEn to solve the problem
     */
    mpc_controllerSolverStatus solve()
    {
        return mpc_controller_solve(cache, u, p, y, &init_penalty);
    }


public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
	    y = new double[MPC_CONTROLLER_N1];
        cache = mpc_controller_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
		if (y!=NULL) delete[] y;
        mpc_controller_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(mpc_controllerSolverStatus& status)
    {
        std::vector<double> sol(u, u + MPC_CONTROLLER_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + MPC_CONTROLLER_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/open_nmpc_controller/open_params`
     */
    void mpcReceiveRequestCallback(
        const open_nmpc_controller::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        mpc_controllerSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }
    
    void commandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_ref[0] = msg->pose.position.x;
        current_ref[1] = msg->pose.position.y;
        current_ref[2] = msg->pose.orientation.w;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        double roll, pitch, yaw;
        
        tf::Quaternion quat(msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z,
                            msg->pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        
        current_pos[0] = msg->pose.pose.position.x;
        current_pos[1] = msg->pose.pose.position.y;
        current_pos[2] = yaw;
    }
    
    void solveAndPublishCmdVel(ros::Publisher& publisher)
    {
        double current_par [MPC_CONTROLLER_NUM_PARAMETERS] = {0};
        double current_var [MPC_CONTROLLER_NUM_DECISION_VARIABLES] = {0};
        double lin_vel_cmd, ang_vel_cmd = 0;
        
        for (int i=0; i<NX; i++) {
            current_par[i] = current_pos[i];
            current_par[i+NX] = current_ref[i];
        }
        
        /* solve                  */
        mpc_controllerSolverStatus status
            = mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);
        
        lin_vel_cmd = current_var[0];
        ang_vel_cmd = current_var[1];
    
        geometry_msgs::Twist twist;
        twist.linear.x = lin_vel_cmd;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = ang_vel_cmd;
        publisher.publish(twist);
        
        ROS_INFO("x: %f, y: %f, yaw: %f", current_pos[0], current_pos[1], current_pos[2]);
        ROS_INFO("Solve time: %f ms. I will send %f %f \n",
                 (double)status.solve_time_ns / 1000000.0, lin_vel_cmd, ang_vel_cmd);

    }
    
}; /* end of class OptimizationEngineManager */

} /* end of namespace open_nmpc_controller */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'open_nmpc_controller/result'.
 *
 * It obtains inputs from 'open_nmpc_controller/parameters'.
 *
 */
int main(int argc, char** argv)
{

    std::string result_topic, params_topic;  /* parameter and result topics */
    std::string out_twist, in_odometry;
    double rate; /* rate of node (specified by parameter) */

    open_nmpc_controller::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_MPC_CONTROLLER_NODE_NAME);
    ros::NodeHandle nh, private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC));
    private_nh.param("rate", rate,
                     double(ROS_NODE_MPC_CONTROLLER_RATE));
    
    private_nh.param("out_twist_name", out_twist, std::string("/husky_velocity_controller/cmd_vel"));
    private_nh.param("in_odom_name", in_odometry, std::string("/odometry/filtered"));

    ros::Publisher mpc_pub
        = private_nh.advertise<open_nmpc_controller::OptimizationResult>(
            result_topic,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            params_topic,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &open_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Subscriber pos_sub
        = nh.subscribe(in_odometry,
                       1,
                       &open_nmpc_controller::OptimizationEngineManager::odometryCallback,
                       &mng);
    ros::Subscriber command_trajectory_subscriber
        = private_nh.subscribe("command/pose",
                               1,
                               &open_nmpc_controller::OptimizationEngineManager::commandPoseCallback,
                               &mng);
    ros::Publisher pub_twist_cmd = nh.advertise<geometry_msgs::Twist>(out_twist, 1);
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        mng.solveAndPublishCmdVel(pub_twist_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

After that, we need to rebuild and launch our ROS package

```bash
cd ~/open_ros_codegen/catkin_ws/
catkin build
roslaunch open_nmpc_controller open_optimizer.launch 
```

You can command the vehicle to go to the position $x^{\mathrm{ref}} = (10, 10, 0)$ by writing on a new terminal window:

```bash
source ~/open_ros_codegen/catkin_ws/devel/setup.bash
rostopic pub /open_nmpc_controller/command/pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 10.0
    y: 10.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```

The source code of this example is available at [open_ros_codegen](https://github.com/gmsanchez/open_ros_codegen) Github repository.

---
id: example_navigation_ros_codegen
title: Navigation of ground vehicle using ROS codegen
description: Nonlinear model predictive control with OpEn using ROS
---

<script type="text/x-mathjax-config">MathJax.Hub.Config({tex2jax: {inlineMath: [['$','$'], ['\\(','\\)']]}});</script>
<script type="text/javascript" async src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>

## Auto-generated ground vehicle navigation for ROS

Consider the navigation problem for an autonomous ground vehicle that can be modeled using the differential drive equations.

### Code generation

To generate a ROS package you can use opengen - OpEn's Python interface (with opengen version `0.5.0` or newer). You will have to provide certain configuration parameters, such as the package name, the node name and the rate of your node in Hz.

### Install opengen 0.5.0 in a virtual environment

```bash
cd ~
mkdir open_ros_codegen
virtualenv -p python3.6 venvopen
source venvopen/bin/activate
pip install opengen==0.5.0a1
pip install matplotlib
```

### Run the example code generation program

Let's make a folder for the code generation script

```bash
cd ~/open_ros_codegen
mkdir nmpc_open
cd nmpc_open
```

Create a file named `create_open_solver.py` inside the `nmpc_open` folder with the following content

```python
import casadi as cs
import opengen as og
import matplotlib.pyplot as plt
import numpy as np

useSX = True  # Use CasADi SX or MX. Seems quicker with SX
doSim = False  # Run simulation after creating the solver

N = 10
NX = 3
NU = 2
sampling_time = 0.1
NSim = 100

Q = cs.DM.eye(NX) * [1.0, 1.0, 0.0001]
R = cs.DM.eye(NU) * [0.1, 50.0]
QN = cs.DM.eye(NX) * [10.0, 10.0, 0.0001]


def dynamics_ct(_x, _u):
    return cs.vcat([_u[0] * cs.cos(_x[2]),
                    _u[0] * cs.sin(_x[2]),
                    _u[1]])


def dynamics_dt(_x, _u):
    dx = dynamics_ct(_x, _u)
    return cs.vcat([_x[i] + sampling_time * dx[i] for i in range(NX)])


def stage_cost(_x, _u, _x_ref=None, _u_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    if _u_ref is None:
        _u_ref = cs.DM.zeros(_u.shape)
    dx = _x - _x_ref
    du = _u - _u_ref
    return cs.mtimes([dx.T, Q, dx]) + cs.mtimes([du.T, R, du])


def terminal_cost(_x, _x_ref=None):
    if _x_ref is None:
        _x_ref = cs.DM.zeros(_x.shape)
    dx = _x - _x_ref
    return cs.mtimes([dx.T, QN, dx])

if useSX:
    x_0 = cs.SX.sym("x_0", NX)
    x_ref = cs.SX.sym("x_ref", NX)
    u_k = [cs.SX.sym('u_' + str(i), NU) for i in range(N)]
else:
    x_0 = cs.MX.sym("x_0", NX)
    x_ref = cs.MX.sym("x_ref", NX)
    u_k = [cs.MX.sym('u_' + str(i), NU) for i in range(N)]


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

problem = og.builder.Problem(optimization_variables, optimization_parameters, total_cost).with_constraints(bounds)

if doSim:
    build_config = og.config.BuildConfiguration()\
        .with_build_directory("optimization_engine")\
        .with_build_mode("release")\
        .with_tcp_interface_config()
else:
    ros_config = og.config.RosConfiguration() \
        .with_package_name("open_nmpc_controller") \
        .with_node_name("open_mpc_controller_node") \
        .with_rate(10) \
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
                                          solver_config) \
    .with_verbosity_level(1)
builder.build()


if doSim:
    # Use TCP server
    # ------------------------------------
    mng = og.tcp.OptimizerTcpManager('optimization_engine/mpc_controller')
    mng.start()

    current_x_0 = [-1, -1, 0]
    current_x_ref = [2, 2, 0]
    current_u = [0, 0]
    cur_opt_var = [0] * (N * NU)  # current optimization variables
    cur_opt_par = current_x_0 + current_x_ref  # current optimization parameters
    average_solvetime = 0.0

    x_sim = np.zeros((NSim + 1, NX))
    x_sim[0:N + 1] = current_x_0

    for k in range(0, NSim):

        solver_status = mng.call(cur_opt_par)
        cur_opt_var = solver_status['solution']
        solvetime = solver_status['solve_time_ms']
        current_u = cur_opt_var[0:NU]
        current_x_0 = dynamics_dt(current_x_0, current_u).toarray().flatten()
        x_sim[k + 1] = current_x_0

        print(("Step %d/%d took %f [ms]") % (k, NSim, solvetime))
        average_solvetime += solvetime

        cur_opt_par = list(current_x_0) + current_x_ref

    mng.kill()

    print(("Average solvetime %.4f [ms]") % (average_solvetime/NSim))

    plt.figure()
    plt.subplot(3,1,1)
    plt.plot(x_sim[:, 0])
    plt.subplot(3, 1, 2)
    plt.plot(x_sim[:, 1])
    plt.subplot(3, 1, 3)
    plt.plot(x_sim[:, 0], x_sim[:, 1])
    plt.show()
```

The above program will generate a parametric optimizer at `optimization_engine/mpc_controller`. The ROS package will be in `optimization_engine/mpc_controller/open_nmpc_controller`.

## Create a ROS workspace for our new package

We must create a ROS catkin workspace and place our package inside the `src` folder. We will do this by creating a symbolic link. After that, we can build and test our package.

```bash
cd ~/open_nmpc_controller/
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s ../../nmpc_open/optimization_engine/mpc_controller/open_nmpc_controller .
cd ~/open_ros_codegen/catkin_ws
catkin build
source devel/setup.bash
roslaunch open_nmpc_controller open_optimizer.launch
```

Right now we have a running ROS node with the defualt configuration. Now we must edit it a little bit to drive our vehicle.

## Download and run Husky packages

```bash
cd ~/open_nmpc_controller/catkin_ws/src
git clone https://github.com/husky/husky.git
catkin build
```
Now we can run the Husky simulation as follows

``roslaunch husky_gazebo husky_empty_world.launch``

## Edit the auto-generated node

Our node should know where the vehicle is in order to obtain the correct control commands, so we must listen to an Odometry topic that is being published to `/odometry/filtered`

Given our current position and orientation, we will obtain the NMPC solution and publish it as a Twist message to the `/husky_velocity_controller/cmd_vel` topic.

In order to do that, we must modify the auto generated `open_optimizer.cpp` file as follows:


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

    double current_pos[3] = {0};
    double current_ref[3] = {0};

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

    void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        current_ref[0] = msg->pose.position.x;
        current_ref[1] = msg->pose.position.y;
        current_ref[2] = msg->pose.orientation.w;  // we will use this field for the yaw in degrees.
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        double roll, pitch, yaw;

        tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
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

        current_par[0] = current_pos[0];
        current_par[1] = current_pos[1];
        current_par[2] = current_pos[2];
        current_par[3] = current_ref[0];
        current_par[4] = current_ref[1];
        current_par[5] = current_ref[2];

        /* solve                  */
        mpc_controllerSolverStatus status = mpc_controller_solve(cache, current_var, current_par, 0, &init_penalty);

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
        ROS_INFO("Solve time: %f ms. I will send %f %f \n", (double)status.solve_time_ns / 1000000.0, lin_vel_cmd, ang_vel_cmd);

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
    ros::NodeHandle private_nh("~");

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
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC,
            ROS_NODE_MPC_CONTROLLER_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC,
            ROS_NODE_MPC_CONTROLLER_PARAMS_TOPIC_QUEUE_SIZE,
            &open_nmpc_controller::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);

    ros::Subscriber pos_sub
        = private_nh.subscribe(in_odometry, 1, &open_nmpc_controller::OptimizationEngineManager::odometryCallback, &mng);
    ros::Rate loop_rate(ROS_NODE_MPC_CONTROLLER_RATE);
    ros::Subscriber command_trajectory_subscriber
        = private_nh.subscribe("command/pose", 1, &open_nmpc_controller::OptimizationEngineManager::CommandPoseCallback, &mng);
    ros::Publisher pub_twist_cmd = private_nh.advertise<geometry_msgs::Twist>(out_twist, 1);

    while (ros::ok()) {
        // mng.solveAndPublish(mpc_pub);
        mng.solveAndPublishCmdVel(pub_twist_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```
We can rebuild and launch our node.

```bash
cd ~/open_nmpc_controller/catkin_ws/
catkin build
roslaunch open_nmpc_controller open_optimizer.launch
```

Our modified node will listen to the topic `/open_nmpc_controller/command/pose` and set the reference. As an example, you can execute

```bash
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

To send the ground vehicle to the point (10, 10).

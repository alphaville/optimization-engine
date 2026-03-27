# ROS2 Package: {{ros.package_name}}


## Installation and Setup

Move or link the auto-generated ROS2 package (folder `{{ros.package_name}}`) to your workspace source tree (typically `~/ros2_ws/src/`).

From within the folder `{{ros.package_name}}`, compile with:

```bash
colcon build --packages-select {{ros.package_name}}
source install/setup.bash 
# or source install/setup.zsh if you are using zsh
```

If you want to activate logging (recommended), do

```bash
mkdir -p .ros_log
export ROS_LOG_DIR="$PWD/.ros_log"
```


## Launch and Use

Start the optimizer in one terminal. The process stays in the foreground while
the node is running.

```bash
# Terminal 1
source install/setup.bash 
# or: source install/setup.zsh
ros2 run {{ros.package_name}} {{ros.node_name}}
```

If ROS2 cannot write to its default log directory, set an explicit writable log
path:

```bash
mkdir -p .ros_log
export ROS_LOG_DIR="$PWD/.ros_log"
```

If the node starts but does not appear in the ROS2 graph, try forcing Fast DDS
in both terminals before sourcing the generated workspace and running any
`ros2` commands:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

In a second terminal, source the same environment and verify discovery:

```bash
# Terminal 2
source install/setup.bash   
# or: source install/setup.zsh
ros2 node list --no-daemon --spin-time 5
ros2 topic list --no-daemon --spin-time 5
```

You should see the node `/{{ros.node_name}}`, the input topic
`/{{ros.subscriber_subtopic}}`, and the output topic
`/{{ros.publisher_subtopic}}`.

Then publish a request to the configured parameters topic
(default: `/{{ros.subscriber_subtopic}}`):

```bash
ros2 topic pub --once /{{ros.subscriber_subtopic}} {{ros.package_name}}/msg/OptimizationParameters "{parameter: [YOUR_PARAMETER_VECTOR], initial_guess: [INITIAL_GUESS_OPTIONAL], initial_y: [], initial_penalty: 15.0}"
```

The result will be announced on the configured result topic
(default: `/{{ros.publisher_subtopic}}`):

```bash
ros2 topic echo /{{ros.publisher_subtopic}} --once
```

To get the optimal solution you can do:

```bash
ros2 topic echo /{{ros.publisher_subtopic}} --field solution
```

You can also start the node using the generated launch file:

```bash
ros2 launch {{ros.package_name}} open_optimizer.launch.py
```

The launch file loads its runtime parameters from
[`config/open_params.yaml`](config/open_params.yaml).


## Messages

This package involves two messages: `OptimizationParameters`
and `OptimizationResult`, which are used to define the input
and output values to the node. `OptimizationParameters` specifies
the parameter vector, the initial guess (optional), the initial
guess for the vector of Lagrange multipliers and the initial value
of the penalty value. `OptimizationResult` is a message containing
all information related to the solution of the optimization
problem, including the optimal solution, the solver status,
solution time, Lagrange multiplier vector and more.

The message structures are defined in the following msg files:

- [`OptimizationParameters.msg`](msg/OptimizationParameters.msg)
- [`OptimizationResult.msg`](msg/OptimizationResult.msg)


## Configure

You can configure the rate and topic names by editing
[`config/open_params.yaml`](config/open_params.yaml).


## Directory structure and contents

The following auto-generated files are included in your ROS2 package:

```txt
├── CMakeLists.txt
├── config
│   └── open_params.yaml
├── extern_lib
│   └── librosenbrock.a
├── include
│   ├── open_optimizer.hpp
│   └── rosenbrock_bindings.hpp
├── launch
│   └── open_optimizer.launch.py
├── msg
│   ├── OptimizationParameters.msg
│   └── OptimizationResult.msg
├── package.xml
├── README.md
└── src
    └── open_optimizer.cpp
```

# ROS Package {{ros.package_name}}

## Installation and Setup

Move the auto-generated ROS package (folder `{{ros.package_name}}`) to your catkin workspace (typically `~/catkin_ws/`):

Compile with:

```console
cd ~/catkin_ws/
catkin_make
``` 

## Launch and Use

Run with:

```
roslaunch {{ros.package_name}} open_optimizer.launch
```

Once you run the node, you can post your request to `{{ros.package_name}}/parameters` as follows:

```
rostopic pub /{{ros.package_name}}/parameters  \
  {{ros.package_name}}/OptimizationParameters  \
  "{'parameter':[YOUR_PARAMETER_VECTOR],       \
   'initial_guess':[INITIAL_GUESS (OPTIONAL)]}" --once
```

The result will be announced on `/{{ros.package_name}}/solution`:

```
rostopic echo /{{ros.package_name}}/solution
```


## Configure

You can configure the rate and topic names by editing `config/open_params.yaml`.

## Directory structure and contents

The following auto-generated files are included in your ROS package:

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
│   └── open_optimizer.launch
├── msg
│   ├── OptimizationParameters.msg
│   └── OptimizationResult.msg
├── package.xml
├── README.md
└── src
    └── open_optimizer.cpp
```

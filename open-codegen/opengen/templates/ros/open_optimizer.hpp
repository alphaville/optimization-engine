#ifndef ROS_NODE_{{meta.optimizer_name|upper}}_H
#define ROS_NODE_{{meta.optimizer_name|upper}}_H

#define ROS_NODE_{{meta.optimizer_name|upper}}_NODE_NAME "open_optimizer"
#define ROS_NODE_{{meta.optimizer_name|upper}}_BASE_TOPIC "/{{meta.optimizer_name}}"
#define ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC "open_solution"
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC "open_parameters"
#define ROS_NODE_{{meta.optimizer_name|upper}}_RATE 10
#define ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC_QUEUE_SIZE 100
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE 100
#define ROS_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY 1000.0

#endif /* Header Sentinel: MPC_DUMMY_H */

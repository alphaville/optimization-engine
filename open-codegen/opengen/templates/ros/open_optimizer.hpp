#ifndef ROS_NODE_{{meta.optimizer_name|upper}}_H
#define ROS_NODE_{{meta.optimizer_name|upper}}_H

#define ROS_NODE_{{meta.optimizer_name|upper}}_NODE_NAME "{{ros.node_name}}"
#define ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC "{{ros.publisher_subtopic}}"
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC "{{ros.subscriber_subtopic}}"
#define ROS_NODE_{{meta.optimizer_name|upper}}_RATE {{ros.rate}}
#define ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC_QUEUE_SIZE {{ros.solution_topic_queue_size}}
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE {{ros.params_topic_queue_size}}
#define ROS_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY 1000.0

#endif /* Header Sentinel: MPC_DUMMY_H */
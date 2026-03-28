#ifndef ROS_NODE_{{meta.optimizer_name|upper}}_H
#define ROS_NODE_{{meta.optimizer_name|upper}}_H

/**
 * Default node name
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_NODE_NAME "{{ros.node_name}}"

/**
 * Default result (publisher) topic name
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC "{{ros.publisher_subtopic}}"

/**
 * Default parameters (subscriber) topic name
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC "{{ros.subscriber_subtopic}}"

/**
 * Default execution rate (in Hz)
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_RATE {{ros.rate}}

/**
 * Default result topic queue size
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC_QUEUE_SIZE {{ros.result_topic_queue_size}}

/**
 * Default parameters topic queue size
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE {{ros.params_topic_queue_size}}

/**
 * Default initial penalty
 */
#define ROS_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY {{solver_config.initial_penalty}}


#endif /* Header Sentinel: MPC_DUMMY_H */

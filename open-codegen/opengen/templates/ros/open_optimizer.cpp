#include "ros/ros.h"
#include "ros_node_{{meta.optimizer_name}}/OptimizationResult.h"
#include "ros_node_{{meta.optimizer_name}}/OptimizationParameters.h"
#include "rosenbrock_bindings.hpp"
#include "open_optimizer.hpp"

namespace ros_node_{{meta.optimizer_name}} {
/**
 * Class ros_node_{{meta.optimizer_name}}::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {

private:
    ros_node_{{meta.optimizer_name}}::OptimizationParameters params;
    ros_node_{{meta.optimizer_name}}::OptimizationResult results;
    double p[{{meta.optimizer_name|upper}}_NUM_PARAMETERS] = { 0 };
    double u[{{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES] = { 0 };
		double *y = NULL;

    rosenbrockCache* cache;
    double init_penalty = ROS_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY;

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
            : ROS_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < {{meta.optimizer_name|upper}}_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == {{meta.optimizer_name|upper}}_N1) {
            for (size_t i = 0; i < {{meta.optimizer_name|upper}}_N1; ++i)
                y[i] = params.initial_y[i];
		}

    }

    /**
     * Call OpEn to solve the problem
	   */
    rosenbrockSolverStatus solve()
    {
        return rosenbrock_solve(cache, u, p, y, &init_penalty);
    }


public:
    /**
	   * Constructor of OptimizationEngineManager
	   */
    OptimizationEngineManager()
    {
			  y = new double[{{meta.optimizer_name|upper}}_N1];
        cache = rosenbrock_new();
    }

    /**
	   * Destructor of OptimizationEngineManager
	   */
    ~OptimizationEngineManager()
    {
			  if (y!=NULL) delete[] y;
        rosenbrock_free(cache);
    }

    /**
	   * Copies results from `status` to the local field `results`
	   */
    void updateResults(rosenbrockSolverStatus& status)
    {
        std::vector<double> sol(u, u + {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + {{meta.optimizer_name|upper}}_N1);
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
		 * Callback that obtains data from topic `/ros_node_{{meta.optimizer_name}}/open_params`
		 */
    void mpcReceiveRequestCallback(
        const ros_node_{{meta.optimizer_name}}::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        rosenbrockSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }
}; /* end of class OptimizationEngineManager */

} /* end of namespace ros_node_{{meta.optimizer_name}} */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. This
 * is called '{{meta.optimizer_name}}/solution'
 *
 * It obtains inputs from a public/global topic, which is specified
 * by a parameter (see config/open_params.yaml).
 */
int main(int argc, char** argv)
{

    std::string solution_topic, params_topic;  /* parameter and solution topics */
    double rate; /* rate of node (specified by parameter) */

    ros_node_{{meta.optimizer_name}}::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_{{meta.optimizer_name|upper}}_NODE_NAME);
    ros::NodeHandle private_nh("~");

    private_nh.param("solution_topic", solution_topic, std::string(ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC));
    private_nh.param("params_topic", params_topic, std::string(ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC));
    private_nh.param("rate", rate, double(ROS_NODE_{{meta.optimizer_name|upper}}_RATE));

    ros::Publisher mpc_pub
        = private_nh.advertise<ros_node_{{meta.optimizer_name}}::OptimizationResult>(
            ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC,
            ROS_NODE_{{meta.optimizer_name|upper}}_SOLUTION_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC,
            ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE,
            &ros_node_{{meta.optimizer_name}}::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Rate loop_rate(ROS_NODE_{{meta.optimizer_name|upper}}_RATE);

    while (ros::ok()) {
        mng.solveAndPublish(mpc_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

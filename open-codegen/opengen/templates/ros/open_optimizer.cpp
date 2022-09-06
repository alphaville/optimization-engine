/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include "ros/ros.h"
#include "{{ros.package_name}}/OptimizationResult.h"
#include "{{ros.package_name}}/OptimizationParameters.h"
#include "{{meta.optimizer_name}}_bindings.hpp"
#include "open_optimizer.hpp"

namespace {{ros.package_name}} {
/**
 * Class {{ros.package_name}}::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {
/**
 * Private fields and methods
 */
private:
    /**
     * Optimization parameters announced on the corresponding
     * topic ({{ros.package_name}}/parameters)
     */
    {{ros.package_name}}::OptimizationParameters params;
    /**
     * Object containing the result (solution and solver
     * statistics), which will be announced on {{ros.package_name}}/results
     */
    {{ros.package_name}}::OptimizationResult results;
    /**
     * Vector of parameters (provided by the client on
     * {{ros.package_name}}/parameters)
     */
    double p[{{meta.optimizer_name|upper}}_NUM_PARAMETERS] = { 0 };
    /**
     * Solution vector
     */
    double u[{{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES] = { 0 };
    /**
     * Vector of Lagrange multipliers (if any)
     */
    double *y = NULL;
    /**
     * Workspace variable used by the solver - initialised once
     */
    {{meta.optimizer_name}}Cache* cache;
    /**
     * Initial guess for the penalty parameter
     */
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
    {{meta.optimizer_name}}SolverStatus solve()
    {
        return {{meta.optimizer_name}}_solve(cache, u, p, y, &init_penalty);
    }
/**
 * Public fields and methods
 */
public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
	    y = new double[{{meta.optimizer_name|upper}}_N1];
        cache = {{meta.optimizer_name}}_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
		if (y!=NULL) delete[] y;
        {{meta.optimizer_name}}_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults({{meta.optimizer_name}}SolverStatus& status)
    {
        std::vector<double> sol(u, u + {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + {{meta.optimizer_name|upper}}_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.cost = status.cost;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/{{ros.package_name}}/open_params`
     */
    void mpcReceiveRequestCallback(
        const {{ros.package_name}}::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        {{meta.optimizer_name}}SolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }
}; /* end of class OptimizationEngineManager */
} /* end of namespace {{ros.package_name}} */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is '{{ros.package_name}}/{{ros.publisher_subtopic}}'.
 *
 * It obtains inputs from '{{ros.package_name}}/{{ros.subscriber_subtopic}}'.
 *
 */
int main(int argc, char** argv)
{
    std::string result_topic, params_topic;  /* parameter and result topics */
    double rate; /* rate of node (specified by parameter) */

    {{ros.package_name}}::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_{{meta.optimizer_name|upper}}_NODE_NAME);
    ros::NodeHandle private_nh("~");

    /* obtain parameters from config/open_params.yaml file */
    private_nh.param("result_topic", result_topic,
                     std::string(ROS_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC));
    private_nh.param("params_topic", params_topic,
                     std::string(ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC));
    private_nh.param("rate", rate,
                     double(ROS_NODE_{{meta.optimizer_name|upper}}_RATE));

    ros::Publisher mpc_pub
        = private_nh.advertise<{{ros.package_name}}::OptimizationResult>(
            result_topic,
            ROS_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            params_topic,
            ROS_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE,
            &{{ros.package_name}}::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Rate loop_rate(rate);

    while (ros::ok()) {
        mng.solveAndPublish(mpc_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "{{ros.package_name}}/msg/optimization_parameters.hpp"
#include "{{ros.package_name}}/msg/optimization_result.hpp"
#include "{{meta.optimizer_name}}_bindings.hpp"
#include "open_optimizer.hpp"

namespace {{ros.package_name}} {
/**
 * ROS2 node that wraps the generated OpEn solver.
 *
 * The node subscribes to `OptimizationParameters`, validates and copies the
 * incoming request into the native solver buffers, invokes the generated C
 * bindings, and publishes one `OptimizationResult` message for each request.
 */
class OptimizationEngineNode : public rclcpp::Node {
private:
    using OptimizationParametersMsg = {{ros.package_name}}::msg::OptimizationParameters;
    using OptimizationResultMsg = {{ros.package_name}}::msg::OptimizationResult;

    OptimizationParametersMsg params_;
    OptimizationResultMsg results_;
    bool has_received_request_ = false;
    double p_[{{meta.optimizer_name|upper}}_NUM_PARAMETERS] = { 0 };
    double u_[{{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES] = { 0 };
    double* y_ = nullptr;
    {{meta.optimizer_name}}Cache* cache_ = nullptr;
    double init_penalty_ = ROS2_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY;

    rclcpp::Publisher<OptimizationResultMsg>::SharedPtr publisher_;
    rclcpp::Subscription<OptimizationParametersMsg>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    static constexpr uint16_t kInvalidInitialGuessErrorCode = 1600;
    static constexpr uint16_t kInvalidInitialYErrorCode = 1700;
    static constexpr uint16_t kInvalidParameterErrorCode = 3003;
    static constexpr double kInitialPenaltyEpsilon = std::numeric_limits<double>::epsilon();

    /**
     * Convert the configured solver loop rate in Hz to a ROS2 timer period.
     *
     * A non-positive rate falls back to a conservative default of 100 ms.
     */
    static std::chrono::milliseconds rateToPeriod(double rate)
    {
        if (rate <= 0.0) {
            return std::chrono::milliseconds(100);
        }
        int period_ms = static_cast<int>(1000.0 / rate);
        if (period_ms < 1) {
            period_ms = 1;
        }
        return std::chrono::milliseconds(period_ms);
    }

    /**
     * Build a human-readable dimension-mismatch message for invalid requests.
     */
    std::string makeDimensionErrorMessage(
        const char* label,
        size_t provided,
        size_t expected) const
    {
        std::ostringstream oss;
        oss << label << ": provided " << provided << ", expected " << expected;
        return oss.str();
    }

    /**
     * Populate `results_` with a structured error response.
     *
     * This is used both for request-validation failures in the ROS2 wrapper
     * and for solver-side failures propagated through the generated bindings.
     */
    void setErrorResult(
        uint8_t status,
        uint16_t error_code,
        const std::string& error_message)
    {
        results_.solution.clear();
        results_.lagrange_multipliers.clear();
        results_.inner_iterations = 0;
        results_.outer_iterations = 0;
        results_.status = status;
        results_.error_code = error_code;
        results_.error_message = error_message;
        results_.cost = 0.0;
        results_.norm_fpr = 0.0;
        results_.penalty = 0.0;
        results_.infeasibility_f1 = 0.0;
        results_.infeasibility_f2 = 0.0;
        results_.solve_time_ms = 0.0;
    }

    /**
     * Validate the most recent request and copy it into the solver buffers.
     *
     * On success, this method updates `p_`, `u_`, `y_`, and `init_penalty_`
     * and returns `true`. On failure, it prepares an error result in `results_`
     * and returns `false` without invoking the solver.
     */
    bool validateAndUpdateInputData()
    {
        // A missing or too-small penalty falls back to the generated default.
        init_penalty_ = (params_.initial_penalty > kInitialPenaltyEpsilon)
            ? params_.initial_penalty
            : ROS2_NODE_{{meta.optimizer_name|upper}}_DEFAULT_INITIAL_PENALTY;

        if (params_.parameter.size() != {{meta.optimizer_name|upper}}_NUM_PARAMETERS) {
            setErrorResult(
                OptimizationResultMsg::STATUS_INVALID_REQUEST,
                kInvalidParameterErrorCode,
                makeDimensionErrorMessage(
                    "wrong number of parameters",
                    params_.parameter.size(),
                    {{meta.optimizer_name|upper}}_NUM_PARAMETERS));
            return false;
        }
        for (size_t i = 0; i < {{meta.optimizer_name|upper}}_NUM_PARAMETERS; ++i) {
            p_[i] = params_.parameter[i];
        }

        // If no initial guess is provided, keep the previous `u_` as a warm start.
        if (!params_.initial_guess.empty()
            && params_.initial_guess.size() != {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES) {
            setErrorResult(
                OptimizationResultMsg::STATUS_INVALID_REQUEST,
                kInvalidInitialGuessErrorCode,
                makeDimensionErrorMessage(
                    "initial guess has incompatible dimensions",
                    params_.initial_guess.size(),
                    {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES));
            return false;
        }
        if (params_.initial_guess.size() == {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES; ++i) {
                u_[i] = params_.initial_guess[i];
            }
        }

        // Likewise, an empty `initial_y` means "reuse the previous multipliers".
        if (!params_.initial_y.empty() && params_.initial_y.size() != {{meta.optimizer_name|upper}}_N1) {
            setErrorResult(
                OptimizationResultMsg::STATUS_INVALID_REQUEST,
                kInvalidInitialYErrorCode,
                makeDimensionErrorMessage(
                    "wrong dimension of Lagrange multipliers",
                    params_.initial_y.size(),
                    {{meta.optimizer_name|upper}}_N1));
            return false;
        }
        if (params_.initial_y.size() == {{meta.optimizer_name|upper}}_N1) {
            for (size_t i = 0; i < {{meta.optimizer_name|upper}}_N1; ++i) {
                y_[i] = params_.initial_y[i];
            }
        }

        return true;
    }

    /**
     * Invoke the generated C solver interface on the current buffers.
     */
    {{meta.optimizer_name}}SolverStatus solve()
    {
        return {{meta.optimizer_name}}_solve(cache_, u_, p_, y_, &init_penalty_);
    }

    /**
     * Lazily allocate the solver workspace and multiplier buffer.
     */
    void initializeSolverIfNeeded()
    {
        if (y_ == nullptr) {
            y_ = new double[{{meta.optimizer_name|upper}}_N1]();
        }
        if (cache_ == nullptr) {
            cache_ = {{meta.optimizer_name}}_new();
        }
    }

    /**
     * Convert the solver status structure into the ROS2 result message.
     */
    void updateResults({{meta.optimizer_name}}SolverStatus& status)
    {
        results_.solution.clear();
        for (size_t i = 0; i < {{meta.optimizer_name|upper}}_NUM_DECISION_VARIABLES; ++i) {
            results_.solution.push_back(u_[i]);
        }

        results_.lagrange_multipliers.clear();
        for (size_t i = 0; i < {{meta.optimizer_name|upper}}_N1; ++i) {
            results_.lagrange_multipliers.push_back(status.lagrange[i]);
        }

        results_.inner_iterations = status.num_inner_iterations;
        results_.outer_iterations = status.num_outer_iterations;
        results_.norm_fpr = status.last_problem_norm_fpr;
        results_.cost = status.cost;
        results_.penalty = status.penalty;
        results_.status = static_cast<uint8_t>(status.exit_status);
        results_.error_code = static_cast<uint16_t>(status.error_code);
        // The bindings expose a null-terminated C buffer; convert it once here.
        results_.error_message = std::string(status.error_message);
        results_.solve_time_ms = static_cast<double>(status.solve_time_ns) / 1000000.0;
        results_.infeasibility_f2 = status.f2_norm;
        results_.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Store the latest optimization request received on the parameters topic.
     */
    void receiveRequestCallback(const OptimizationParametersMsg::ConstSharedPtr msg)
    {
        params_ = *msg;
        has_received_request_ = true;
    }

    /**
     * Process at most one pending request and publish exactly one response.
     *
     * Repeated timer ticks do not republish stale results: once a request has
     * been handled, `has_received_request_` is cleared until the next message
     * arrives on the input topic.
     */
    void solveAndPublish()
    {
        if (!has_received_request_) {
            return;
        }
        initializeSolverIfNeeded();
        if (!validateAndUpdateInputData()) {
            publisher_->publish(results_);
            // Mark the request as consumed so the timer does not republish stale errors.
            has_received_request_ = false;
            return;
        }
        {{meta.optimizer_name}}SolverStatus status = solve();
        updateResults(status);
        publisher_->publish(results_);
        // Each request should produce exactly one response message.
        has_received_request_ = false;
    }

public:
    /**
     * Construct the ROS2 node, declare runtime parameters, and create the
     * publisher, subscriber, and wall timer used by the generated wrapper.
     */
    OptimizationEngineNode()
        : Node(ROS2_NODE_{{meta.optimizer_name|upper}}_NODE_NAME)
    {
        this->declare_parameter<std::string>(
            "result_topic",
            std::string(ROS2_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC));
        this->declare_parameter<std::string>(
            "params_topic",
            std::string(ROS2_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC));
        this->declare_parameter<double>(
            "rate",
            double(ROS2_NODE_{{meta.optimizer_name|upper}}_RATE));

        std::string result_topic = this->get_parameter("result_topic").as_string();
        std::string params_topic = this->get_parameter("params_topic").as_string();
        double rate = this->get_parameter("rate").as_double();

        publisher_ = this->create_publisher<OptimizationResultMsg>(
            result_topic,
            ROS2_NODE_{{meta.optimizer_name|upper}}_RESULT_TOPIC_QUEUE_SIZE);
        subscriber_ = this->create_subscription<OptimizationParametersMsg>(
            params_topic,
            ROS2_NODE_{{meta.optimizer_name|upper}}_PARAMS_TOPIC_QUEUE_SIZE,
            std::bind(&OptimizationEngineNode::receiveRequestCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            rateToPeriod(rate),
            std::bind(&OptimizationEngineNode::solveAndPublish, this));
    }

    /**
     * Release any lazily allocated solver resources.
     */
    ~OptimizationEngineNode() override
    {
        if (y_ != nullptr) {
            delete[] y_;
        }
        if (cache_ != nullptr) {
            {{meta.optimizer_name}}_free(cache_);
        }
    }
};
} /* end of namespace {{ros.package_name}} */

/**
 * Start the generated ROS2 optimizer node and hand control to the ROS2
 * executor until shutdown is requested.
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<{{ros.package_name}}::OptimizationEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

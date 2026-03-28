/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include <chrono>
#include <cstdint>
#include <functional>
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

    static constexpr int32_t kInvalidInitialGuessErrorCode = 1600;
    static constexpr int32_t kInvalidInitialYErrorCode = 1700;
    static constexpr int32_t kInvalidParameterErrorCode = 3003;

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

    std::string makeDimensionErrorMessage(
        const char* label,
        size_t provided,
        size_t expected) const
    {
        std::ostringstream oss;
        oss << label << ": provided " << provided << ", expected " << expected;
        return oss.str();
    }

    void setErrorResult(
        uint8_t status,
        int32_t error_code,
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

    bool validateAndUpdateInputData()
    {
        init_penalty_ = (params_.initial_penalty > 1.0)
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

    {{meta.optimizer_name}}SolverStatus solve()
    {
        return {{meta.optimizer_name}}_solve(cache_, u_, p_, y_, &init_penalty_);
    }

    void initializeSolverIfNeeded()
    {
        if (y_ == nullptr) {
            y_ = new double[{{meta.optimizer_name|upper}}_N1]();
        }
        if (cache_ == nullptr) {
            cache_ = {{meta.optimizer_name}}_new();
        }
    }

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
        results_.error_code = status.error_code;
        results_.error_message = std::string(status.error_message);
        results_.solve_time_ms = static_cast<double>(status.solve_time_ns) / 1000000.0;
        results_.infeasibility_f2 = status.f2_norm;
        results_.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    void receiveRequestCallback(const OptimizationParametersMsg::ConstSharedPtr msg)
    {
        params_ = *msg;
        has_received_request_ = true;
    }

    void solveAndPublish()
    {
        if (!has_received_request_) {
            return;
        }
        initializeSolverIfNeeded();
        if (!validateAndUpdateInputData()) {
            publisher_->publish(results_);
            has_received_request_ = false;
            return;
        }
        {{meta.optimizer_name}}SolverStatus status = solve();
        updateResults(status);
        publisher_->publish(results_);
        has_received_request_ = false;
    }

public:
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<{{ros.package_name}}::OptimizationEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

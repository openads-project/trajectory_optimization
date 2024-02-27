#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimization::TrajectoryOptimizationNode)


/**
 * @brief Namespace for trajectory_optimization package
 *
 */
namespace trajectory_optimization {


// parameter names

// constants
const std::string TrajectoryOptimizationNode::kInputTopic = "~/output";
const std::string TrajectoryOptimizationNode::kOutputTopic = "~/output";
const std::string TrajectoryOptimizationNode::kParam = "param";


/**
 * @brief Creates a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::TrajectoryOptimizationNode(const rclcpp::NodeOptions& options) : Node("trajectory_optimization_node", options) {

  this->declareParameters();
  this->loadParameters();
  this->setup();
}



/**
 * @brief Declares all parameters that this node uses
 */
void TrajectoryOptimizationNode::declareParameters() {

  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "TODO";

  // set allowed parameter range
  rcl_interfaces::msg::FloatingPointRange param_range;
  param_range.set__from_value(0.1).set__to_value(10.0).set__step(0.1);
  param_desc.floating_point_range = {param_range};

  // declare parameter
  this->declare_parameter(kParam, rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void TrajectoryOptimizationNode::loadParameters() {

  // load parameter
  try {
    param_ = this->get_parameter(kParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kParam.c_str());
    exit(EXIT_FAILURE);
  }
}


/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void TrajectoryOptimizationNode::setup() {

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryOptimizationNode::parametersCallback, this, std::placeholders::_1));

  // create a subscriber for handling incoming messages
  subscriber_ =
    this->create_subscription<std_msgs::msg::Int32>(
      kInputTopic, 10,
      std::bind(&TrajectoryOptimizationNode::topicCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());

  // create a publisher for publishing messages
  publisher_ = this->create_publisher<std_msgs::msg::Int32>(
    kOutputTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());



  // create a timer for repeatedly invoking a callback to publish messages
  publish_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(param_),
                            std::bind(&TrajectoryOptimizationNode::publishTimerCallback,
                            this));
}


/**
 * @brief This callback is invoked when a parameter value has changed
 *
 * @param[in] parameters                                  input
 *
 * @return    rcl_interfaces::msg::SetParametersResult    output
 */
rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {

  // update timer with newly configured period parameter value
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == kParam) {
      param_ = param.as_double();
    }
  }

  // mark parameter change successful
  result.successful = true;
  result.reason = "success";

  return result;
}


/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input
 */
void TrajectoryOptimizationNode::topicCallback(
  const std_msgs::msg::Int32::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
}


/**
 * @brief This callback is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::publishTimerCallback() {
  
  std_msgs::msg::Int32::UniquePtr msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = 10;
  publisher_->publish(std::move(msg));
}


}
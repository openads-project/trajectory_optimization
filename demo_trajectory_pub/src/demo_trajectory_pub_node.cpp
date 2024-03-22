#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <demo_trajectory_pub/demo_trajectory_pub_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(demo_trajectory_pub::DemoTrajectoryPubNode)

/**
 * @brief Namespace for demo_trajectory_pub package
 *
 */
namespace demo_trajectory_pub {

// constants
const std::string DemoTrajectoryPubNode::kTrajectoryTopic = "~/demo_trajectory";
const std::string DemoTrajectoryPubNode::kNStatesParam = "n_states";
const std::string DemoTrajectoryPubNode::kPubFreqParam = "publish_frequency";
const std::string DemoTrajectoryPubNode::kTrajectoryHoizonParam = "trajectory_horizon";

/**
 * @brief Creates a DemoTrajectoryPubNode node
 *
 */
DemoTrajectoryPubNode::DemoTrajectoryPubNode(const rclcpp::NodeOptions& options)
    : Node("demo_trajectory_pub_node", options) {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

/**
 * @brief Declares all parameters that this node uses
 */
void DemoTrajectoryPubNode::declareParameters() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  // declare number of states parameter
  param_desc.description = "Number of trajectory states";
  this->declare_parameter(kNStatesParam, n_states_, param_desc);

  // declare publish frequency parameter
  param_desc.description = "Publish Frequency in Hz";
  this->declare_parameter(kPubFreqParam, pub_freq_, param_desc);

  // declare trajectory horizon parameter
  param_desc.description = "Trajectory Horizon in seconds";
  this->declare_parameter(kTrajectoryHoizonParam, optimization_horizon_, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void DemoTrajectoryPubNode::loadParameters() {
  try {
    n_states_ = this->get_parameter(kNStatesParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kNStatesParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    pub_freq_ = this->get_parameter(kPubFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPubFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    optimization_horizon_ = this->get_parameter(kTrajectoryHoizonParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kTrajectoryHoizonParam.c_str());
    exit(EXIT_FAILURE);
  }
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void DemoTrajectoryPubNode::setup() {
  // set up publisher for output topic
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());

  // create timer for planning cycle
  planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(pub_freq_),
                                            std::bind(&DemoTrajectoryPubNode::pubTrajectory, this));
}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void DemoTrajectoryPubNode::pubTrajectory() {
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      *trajectory, trajectory_planning_msgs::msg::REFERENCE::TYPE_ID, n_states_ + 1);

  trajectory->header.stamp = this->now();
  trajectory->header.frame_id = "map";
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, false);

  for (int i = 0; i < n_states_ + 1; i++) {
    double t = (double)i * optimization_horizon_ / n_states_;
    double v = 5.0;
    // std::vector<double> state = {t, t, 0.0, t};
    std::vector<double> state = {t, t * v, 0.0, v};
    trajectory_planning_msgs::trajectory_access::setState(*trajectory, state, i);
  }

  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_DEBUG(this->get_logger(), "Published trajectory");
}

}  // namespace demo_trajectory_pub
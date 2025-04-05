#include <trajectory_optimization/trajectory_optimization_base_node.hpp>

namespace trajectory_optimization {
/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  ego_data_ = *msg;
  received_ego_data_ = true;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received object list");
  object_list_ = *msg;
  received_object_list_ = true;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input reference trajectory
 */
void TrajectoryOptimizationNode::referenceTrajectoryCallback(
    const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received reference trajectory");
  reference_trajectory_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received route");
  route_ = *msg;
}
}  // namespace trajectory_optimization
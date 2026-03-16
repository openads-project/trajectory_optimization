// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {
/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  ego_data_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  if (consider_objects_ != CONSIDER_OBJECTS::NO_OBJECTS) {
    RCLCPP_DEBUG(this->get_logger(), "Received object list");
    object_list_ = *msg;
  } else {
    // reset object list to empty
    object_list_ = perception_msgs::msg::ObjectList();
  }
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
  if (run_as_callback_) {
    planningCycle();
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  if (consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS) {
    RCLCPP_DEBUG(this->get_logger(), "Received route");
    route_ = *msg;
  } else {
    // reset route to empty
    route_ = route_planning_msgs::msg::Route();
  }
}
}  // namespace trajectory_optimization

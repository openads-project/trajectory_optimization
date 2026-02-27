// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  if (diagnostics_.ego_data_diagnostic) diagnostics_.ego_data_diagnostic->tick(msg->header.stamp);
  ego_data_ = *msg;
}

void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  if (consider_objects_ != CONSIDER_OBJECTS::NO_OBJECTS) {
    RCLCPP_DEBUG(this->get_logger(), "Received object list");
    if (diagnostics_.object_list_diagnostic) diagnostics_.object_list_diagnostic->tick(msg->header.stamp);
    object_list_ = *msg;
  } else {
    // reset object list to empty
    object_list_ = perception_msgs::msg::ObjectList();
  }
}

void TrajectoryOptimizationNode::referenceTrajectoryCallback(
    const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received reference trajectory");
  if (diagnostics_.reference_trajectory_diagnostic) diagnostics_.reference_trajectory_diagnostic->tick(msg->header.stamp);
  reference_trajectory_ = *msg;
  if (run_as_callback_) {
    planningCycle();
  }
}

void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  if (consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS) {
    RCLCPP_DEBUG(this->get_logger(), "Received route");
    if (diagnostics_.route_diagnostic) diagnostics_.route_diagnostic->tick(msg->header.stamp);
    route_ = *msg;
  } else {
    // reset route to empty
    route_ = route_planning_msgs::msg::Route();
  }
}
}  // namespace trajectory_optimization

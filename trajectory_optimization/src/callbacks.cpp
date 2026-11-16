// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  std::lock_guard<std::mutex> lock(input_mutex_);
  buffered_inputs_.ego_data = msg;
}

void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received object list");
  std::lock_guard<std::mutex> lock(input_mutex_);
  buffered_inputs_.object_list = msg;
}

void TrajectoryOptimizationNode::referenceTrajectoryCallback(
    const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received reference trajectory");
  std::lock_guard<std::mutex> lock(input_mutex_);
  buffered_inputs_.reference_trajectory = msg;
  reference_cycle_pending_ = true;
  reference_cycle_timer_->reset();
}

void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received route");
  std::lock_guard<std::mutex> lock(input_mutex_);
  buffered_inputs_.route = msg;
}

void TrajectoryOptimizationNode::takeInputSnapshot() {
  BufferedInputs inputs;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    inputs = buffered_inputs_;
  }

  // only the solver callback group reads or changes active cycle inputs and settings.
  if (inputs.ego_data) {
    ego_data_ = *inputs.ego_data;
  }
  // Reset the object list to empty when objects are disabled or no input has arrived.
  object_list_ = inputs.object_list && consider_objects_ != CONSIDER_OBJECTS::NO_OBJECTS ? *inputs.object_list
                                                                                         : perception_msgs::msg::ObjectList();
  // Reset the route to empty when boundaries are disabled or no input has arrived.
  route_ =
      inputs.route && consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS ? *inputs.route : route_planning_msgs::msg::Route();
  if (inputs.reference_trajectory) reference_trajectory_ = *inputs.reference_trajectory;
}

void TrajectoryOptimizationNode::processReferenceCycle() {
  bool pending = false;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    // serialize cancellation and input-side resets so a wakeup cannot be lost.
    reference_cycle_timer_->cancel();
    pending = reference_cycle_pending_;
    reference_cycle_pending_ = false;
  }
  if (pending && run_as_callback_) planningCycle();
}
}  // namespace trajectory_optimization

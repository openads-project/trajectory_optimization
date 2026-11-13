// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  const auto callback_time = std::chrono::steady_clock::now();
  const auto callback_ros_ns = now().nanoseconds();
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  std::lock_guard<std::mutex> lock(input_mutex_);
  buffered_inputs_.ego_data = msg;
  buffered_inputs_.ego_callback_steady_time = callback_time;
  buffered_inputs_.ego_callback_ros_ns = callback_ros_ns;
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

std::chrono::steady_clock::time_point TrajectoryOptimizationNode::takeInputSnapshot() {
  BufferedInputs inputs;
  std::chrono::steady_clock::time_point snapshot_time;
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    inputs = buffered_inputs_;
    snapshot_time = std::chrono::steady_clock::now();
  }

  // only the solver callback group reads or changes active cycle inputs and settings.
  if (inputs.ego_data) {
    ego_data_ = *inputs.ego_data;
    ego_callback_ros_ns_ = inputs.ego_callback_ros_ns;
    ego_callback_steady_time_ = inputs.ego_callback_steady_time;
  }
  object_list_ = inputs.object_list && consider_objects_ != CONSIDER_OBJECTS::NO_OBJECTS ? *inputs.object_list
                                                                                         : perception_msgs::msg::ObjectList();
  route_ =
      inputs.route && consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS ? *inputs.route : route_planning_msgs::msg::Route();
  if (inputs.reference_trajectory) reference_trajectory_ = *inputs.reference_trajectory;
  return snapshot_time;
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

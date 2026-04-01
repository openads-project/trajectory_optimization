// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {

class TrajectoryOptimizationAckermannNode : public TrajectoryOptimizationNode {
 public:
  explicit TrajectoryOptimizationAckermannNode(const rclcpp::NodeOptions& options);

  ~TrajectoryOptimizationAckermannNode();

 private:
  // init trajectory with correct type
  void initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  // stabilization strageties
  std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) override;
  std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  // convert to trajectory msg
  void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  // parameters
  // model specific bi-level thresholds
  double bi_level_dDelta_ = 5.0;
};

}  // namespace trajectory_optimization

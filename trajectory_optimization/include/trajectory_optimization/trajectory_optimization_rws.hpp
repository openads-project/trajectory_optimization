// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {

class TrajectoryOptimizationRWSNode : public TrajectoryOptimizationNode {
 public:
  explicit TrajectoryOptimizationRWSNode(const rclcpp::NodeOptions& options);

  ~TrajectoryOptimizationRWSNode();

 private:
  // init trajectory with correct type
  void initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  // stabilization strageties
  std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) override;
  std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  // convert to trajectory msg
  void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  // project acceleration vector on velocity vector
  double projectVectorAonV(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& v);

  // print state info
  double computeVehicleSlipAngle(const double& delta_front, const double& delta_rear);

  // parameters
  double distance_front_axle_ = 1.7;
  double distance_rear_axle_ = 1.7;

  // model specific bi-level thresholds
  double bi_level_dDelta_front_ = 5.0;
  double bi_level_dDelta_rear_ = 5.0;
};

}  // namespace trajectory_optimization

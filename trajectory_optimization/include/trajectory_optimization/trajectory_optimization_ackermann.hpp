// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {

class TrajectoryOptimizationAckermannNode : public TrajectoryOptimizationNode {
 public:
  /**
   * @brief Initializes the optimization node for a kinematic bicycle model with Ackermann steering.
   *
   * @param[in] options ROS node options used for construction.
   */
  explicit TrajectoryOptimizationAckermannNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Initializes a drivable trajectory message for a kinematic bicycle model with Ackermann steering.
   *
   * @param[out] trajectory Trajectory message to initialize.
   */
  void initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  /**
   * @brief Computes the initial optimizer state using bi-level stabilizaion.
   *
   * @param[in] ego_data Current EgoData based on the kinematic bicycle model with Ackermann steering.
   * @return Initial state vector for the Ackermann model.
   */
  std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  /**
   * @brief Computes the initial optimizer state using high-level stabilization.
   *
   * @param[in] ego_data Current EgoData based on the kinematic bicycle model with Ackermann steering.
   * @return Initial state vector for the Ackermann model.
   */
  std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  /**
   * @brief Maps the optimized state trajectory into a trajectory message for a kinematic bicycle model with Ackermann steering.
   *
   * @param[in,out] trajectory Trajectory message to populate.
   */
  void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  // parameters
  // model specific bi-level thresholds
  double bi_level_dDelta_ = 5.0;
};

}  // namespace trajectory_optimization

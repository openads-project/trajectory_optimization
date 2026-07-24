// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {

class TrajectoryOptimizationRWSNode : public TrajectoryOptimizationNode {
 public:
  /**
   * @brief Initializes the optimization node for a kinematic bicycle model with rear wheel steering.
   *
   * @param[in] options ROS node options used for construction.
   */
  explicit TrajectoryOptimizationRWSNode(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Initializes a drivable trajectory message for a kinematic bicycle model with rear wheel steering.
   *
   * @param[out] trajectory Trajectory message to initialize.
   */
  void initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  /**
   * @brief Computes the initial optimizer state using bi-level stabilizaion.
   *
   * @param[in] ego_data Current EgoData based on the kinematic bicycle model with rear wheel steering.
   * @return Initial state vector for the RWS model.
   */
  std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  /**
   * @brief Computes the initial optimizer state using high-level stabilization.
   *
   * @param[in] ego_data Current EgoData based on the kinematic bicycle model with rear wheel steering.
   * @return Initial state vector for the RWS model.
   */
  std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

  /**
   * @brief Maps the optimized state trajectory into a trajectory message for a kinematic bicycle model with rear wheel steering.
   *
   * @param[in,out] trajectory Trajectory message to populate.
   */
  void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

  /**
   * @brief Projects the acceleration vector onto the current direction of motion.
   *
   * @param[in] a Acceleration vector.
   * @param[in] v Velocity vector.
   * @return Longitudinal acceleration along the velocity direction.
   */
  static double projectVectorAonV(const geometry_msgs::msg::Vector3& a, const geometry_msgs::msg::Vector3& v);

  /**
   * @brief Computes the kinematic vehicle slip angle from front and rear steering angles.
   *
   * @param[in] delta_front Front steering angle.
   * @param[in] delta_rear Rear steering angle.
   * @return Vehicle slip angle in radians.
   */
  double computeVehicleSlipAngle(const double& delta_front, const double& delta_rear) const;

  // parameters
  double distance_front_axle_ = 1.7;
  double distance_rear_axle_ = 1.7;
};

}  // namespace trajectory_optimization

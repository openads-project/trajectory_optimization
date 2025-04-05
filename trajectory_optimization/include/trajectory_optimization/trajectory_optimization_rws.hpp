#pragma once

#include "trajectory_optimization_base_node.hpp"

namespace trajectory_optimization {

class TrajectoryOptimizationRWSNode : public TrajectoryOptimizationNode {
  public:
    explicit TrajectoryOptimizationRWSNode(const rclcpp::NodeOptions &options);

    ~TrajectoryOptimizationRWSNode();

  private:


    // set trajectory type
    void setTrajectoryType(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

    // stabilization strageties
    std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) override;
    std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) override;

    // convert to trajectory msg
    void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) override;

    // print state info
    double computeVehicleslipAngle(const double& delta_front, const double& delta_rear);
    void printStateInfo(const std::vector<double>& state) override;
    
};


}  // namespace trajectory_optimization

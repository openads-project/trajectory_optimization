#pragma once

#include <rclcpp/rclcpp.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace demo_trajectory_pub {

class DemoTrajectoryPubNode : public rclcpp::Node {
 public:
  explicit DemoTrajectoryPubNode(const rclcpp::NodeOptions &options);

 private:

  // output topics
  static const std::string kTrajectoryTopic;

  // parameter names
  static const std::string kNStatesParam;
  static const std::string kPubFreqParam;
  static const std::string kTrajectoryHoizonParam;

  void declareParameters();
  void loadParameters();
  void setup();
  void pubTrajectory();

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // parameters
  double pub_freq_ = 10.0;
  int n_states_ = 51;
  double optimization_horizon_ = 5.0;

};

}  // namespace demo_trajectory_pub

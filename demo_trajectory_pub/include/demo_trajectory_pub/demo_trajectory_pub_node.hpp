#pragma once

#include <rclcpp/rclcpp.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs_utils/object_access.hpp>

namespace demo_trajectory_pub {

class DemoTrajectoryPubNode : public rclcpp::Node {
 public:
  explicit DemoTrajectoryPubNode(const rclcpp::NodeOptions &options);

 private:

  // output topics
  static const std::string kTrajectoryTopic;
  static const std::string kEgoDataTopic;
  static const std::string kObjectListTopic;

  // parameter names
  static const std::string kNStatesParam;
  static const std::string kPubFreqParam;
  static const std::string kTrajectoryHorizonParam;
  static const std::string kX0Param;
  static const std::string kY0Param;
  static const std::string kV0Param;
  static const std::string kVEgoParam;
  static const std::string kAParam;
  static const std::string kTheta0Param;
  static const std::string kOmegaParam;
  static const std::string kNObjectsParam;
  static const std::string kObjectsDeltaX;
  static const std::string kObjectsDeltaY;

  void declareParameters();
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  void setup();
  void publish();

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<perception_msgs::msg::EgoData>::SharedPtr egodata_pub_;
  rclcpp::Publisher<perception_msgs::msg::ObjectList>::SharedPtr object_list_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // parameters
  double pub_freq_ = 10.0;
  int n_states_ = 51;
  double optimization_horizon_ = 5.0;
  double x0_ = 0.0;
  double y0_ = 0.0;
  double v0_ = 0.0;
  double v_ego_ = 0.0;
  double a_ = 1.0;
  double theta0_ = 0.0;
  double omega_ = 0.0;
  int n_objects_ = 10;
  double objects_delta_x_ = 10.0;
  double objects_delta_y_ = 0.0;
};

}  // namespace demo_trajectory_pub

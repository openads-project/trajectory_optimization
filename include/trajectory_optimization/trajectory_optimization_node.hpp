#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace trajectory_optimization {


class TrajectoryOptimizationNode : public rclcpp::Node {

 public:

  explicit TrajectoryOptimizationNode(const rclcpp::NodeOptions& options);

 private:

  static const std::string kInputTopic;
  static const std::string kOutputTopic;
  static const std::string kParam;

  void declareParameters();
  void loadParameters();

  void setup();

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  void topicCallback(const std_msgs::msg::Int32::ConstSharedPtr msg);

  void publishTimerCallback();


  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  double param_ = 1.0;


  int count_ = 0;
};


}

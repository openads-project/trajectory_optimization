#pragma once

#include <rclcpp/rclcpp.hpp>

#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs_utils/object_access.hpp>

namespace demo_trajectory_pub {

template <typename C> struct is_vector : std::false_type {};
template <typename T,typename A> struct is_vector< std::vector<T,A> > : std::true_type {};
template <typename C> inline constexpr bool is_vector_v = is_vector<C>::value;

class DemoTrajectoryPubNode : public rclcpp::Node {
 public:
  explicit DemoTrajectoryPubNode(const rclcpp::NodeOptions &options);

 private:

  template <typename T>
  void declareAndLoadParameter(const std::string &name,
                               T &param,
                               const std::string &description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  void setup();
  void publish();

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<perception_msgs::msg::EgoData>::SharedPtr egodata_pub_;
  rclcpp::Publisher<perception_msgs::msg::ObjectList>::SharedPtr object_list_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // parameters
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>>>
      auto_reconfigurable_params_;
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

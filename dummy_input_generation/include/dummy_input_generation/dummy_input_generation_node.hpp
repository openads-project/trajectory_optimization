// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

namespace dummy_input_generation {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Publishes configurable dummy ego, object, and reference trajectory inputs.
 *
 * The node is intended for testing and developing the trajectory optimization
 * and periodically publishes synthetic inputs derived from ROS parameters.
 */
class DummyInputGenerationNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   */
  explicit DummyInputGenerationNode(const rclcpp::NodeOptions& options);

  /// @brief Default ego vehicle length used for published EgoData messages in meters.
  static constexpr double EGO_LENGTH = 5.173;
  /// @brief Default ego vehicle width used for published EgoData messages in meters.
  static constexpr double EGO_WIDTH = 1.94;
  /// @brief Default object and ego vehicle height used for published messages in meters.
  static constexpr double OBJECT_HEIGHT = 2.0;

 private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param[in] name name
   * @param[in] param parameter variable to load into
   * @param[in] description description
   * @param[in] add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param[in] is_required whether failure to load parameter will stop node
   * @param[in] read_only set parameter to read-only
   * @param[in] from_value parameter range minimum
   * @param[in] to_value parameter range maximum
   * @param[in] step_value parameter range step
   * @param[in] additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name,
                               T& param,
                               const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false,
                               const bool read_only = false,
                               const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");
  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param[in] parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /// @brief Sets up subscribers, publishers, etc. to configure the node.
  void setup();
  /// @brief Recreate the periodic publish timer based on the configured frequency.
  void createPlanningTimer();
  /// @brief Publish the current dummy ego state, object list, and reference trajectory.
  void publish();

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<perception_msgs::msg::EgoData>::SharedPtr egodata_pub_;
  rclcpp::Publisher<perception_msgs::msg::ObjectList>::SharedPtr object_list_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  // parameters
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  // 1) general params
  double publish_frequency_ = 10.0;
  std::string message_frame_id_ = "map";

  // 2) ego data params
  std::string ego_state_model_ = "ackermann";
  double ego_vel_lon_ = 0.0;
  double ego_acc_lon_ = 0.0;
  double ego_steering_angle_ack_ = 0.0;
  double ego_steering_angle_front_ = 0.0;
  double ego_steering_angle_rear_ = 0.0;
  std::vector<double> ego_translation_to_geometric_center_ = {1.4895, 0.0, 0.420};

  // 3) reference trajectory params
  int reference_n_states_ = 51;
  double reference_trajectory_horizon_ = 5.0;
  bool reference_standstill_ = false;
  double reference_x0_ = 0.0;
  double reference_y0_ = 0.0;
  double reference_v0_ = 0.0;
  double reference_a_ = 1.0;
  double reference_theta0_ = 0.0;
  double reference_omega_ = 0.0;

  // 4) object params
  int object_count_ = 10;
  double object_delta_x_ = 10.0;
  double object_delta_y_ = 0.0;
  double object_length_ = 4.0;
  double object_width_ = 2.0;
  double object_yaw_ = 0.0;
};

}  // namespace dummy_input_generation

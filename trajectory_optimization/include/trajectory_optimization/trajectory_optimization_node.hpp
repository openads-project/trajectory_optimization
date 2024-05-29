#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// definitions
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <route_planning_msgs/msg/route.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// access functions
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_trajectory_planning_msgs/tf2_trajectory_planning_msgs.hpp>

// acados
#include <acados/utils/math.h>
#include <acados/utils/print.h>
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <acados_ocp/acados_solver_trajectory_planning.h>
#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

namespace trajectory_optimization {

class TrajectoryOptimizationNode : public rclcpp::Node {
 public:
  explicit TrajectoryOptimizationNode(const rclcpp::NodeOptions &options);

  ~TrajectoryOptimizationNode();

 private:
  // input topics
  const std::string kEgoDataTopic = "~/ego_data";
  const std::string kObjectListTopic = "~/object_list";
  const std::string kReferenceTrajectoryTopic = "~/reference_trajectory";
  const std::string kRouteTopic = "~/route";

  // output topics
  const std::string kTrajectoryTopic = "~/trajectory";

  template <typename T>
  void declareAndLoadParameter(const std::string &name, T &member_param, const rclcpp::ParameterType &type,
                               const std::string &description, const bool add_to_auto_reconfigurable_params = true,
                               const bool is_required = false, const bool read_only = false,
                               const std::optional<double> &from_value = std::nullopt,
                               const std::optional<double> &to_value = std::nullopt,
                               const std::optional<double> &step_value = std::nullopt,
                               const std::string &additional_constraints = "");
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void setup();
  void setupSolver();
  void freeSolver();

  void printSolution(int status);
  void trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory &trajectory);

  std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData &ego_data);
  std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData &ego_data);
  bool linearInterpolation(const std::vector<double> &X, const std::vector<double> &Y, const double &desired_x,
                           double &output_y);

  void egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg);
  void objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg);
  void referenceTrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg);

  void planningCycle();
  bool updateOcpInputs(const perception_msgs::msg::EgoData &ego_data,
                       const perception_msgs::msg::ObjectList &object_list,
                       const route_planning_msgs::msg::Route &route,
                       const trajectory_planning_msgs::msg::Trajectory &reference_trajectory);

  void setOcpParameters(std::vector<double> &cost_weights,
                        const trajectory_planning_msgs::msg::Trajectory &reference_trajectory);

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr ego_data_sub_;
  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr object_list_sub_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr route_sub_;
  rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr reference_trajectory_sub_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

  rclcpp::TimerBase::SharedPtr planning_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // input data
  perception_msgs::msg::EgoData ego_data_;
  perception_msgs::msg::ObjectList object_list_;
  route_planning_msgs::msg::Route route_;
  trajectory_planning_msgs::msg::Trajectory reference_trajectory_;

  // received data flags
  bool received_ego_data_ = false;
  bool received_object_list_ = false;

  // parameters
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter &)>, std::string>> auto_reconfigurable_params_;
  std::string vehicle_frame_id_ = "base_link";
  std::string trajectory_frame_id_ = "base_link";
  std::string fixed_over_time_frame_id_ = "map";
  double optimization_freq_ = 10.0;
  int n_shots_ = TRAJECTORY_PLANNING_N;
  double optimization_horizon_ = 1.0;
  bool verbose_ = false;
  double wheelbase_ = 2.711;
  double standstill_threshold_ = 0.45;
  bool high_level_stabilization_ = false;

  // bi-level thresholds
  double bi_level_dV_ = 5.0;
  double bi_level_dA_ = 2.0;
  double bi_level_dY_ = 0.1;
  double bi_level_dYaw_ = 5.0;
  double bi_level_dDelta_ = 90.0;

  // latest valid trajectory
  trajectory_planning_msgs::msg::Trajectory latest_valid_trajectory_;

  // cost weights
  std::vector<double> cost_weights_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double dynamic_weight_ = 1.0;

  // ocp parameter vector structure
  std::vector<int64_t> p_cost_weights_shape_ = {10, 1};
  std::vector<int64_t> p_ref_path_shape_ = {100, 4};
  std::vector<int64_t> p_obstacles_shape_ = {10, 3};

  // ocp variables
  trajectory_planning_solver_capsule *acados_ocp_capsule_;
  ocp_nlp_config *nlp_config_;
  ocp_nlp_dims *nlp_dims_;
  ocp_nlp_in *nlp_in_;
  ocp_nlp_out *nlp_out_;
  ocp_nlp_solver *nlp_solver_;
  void *nlp_opts_;

  double *xtraj_;
  double *utraj_;
};

}  // namespace trajectory_optimization

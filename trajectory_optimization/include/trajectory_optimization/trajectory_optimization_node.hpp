#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

// definitions
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <route_planning_msgs/msg/driveable_space.hpp>
#include <route_planning_msgs/msg/route.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// access functions
#include <perception_msgs_utils/object_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

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
  static const std::string kDriveableSpaceTopic;
  static const std::string kEgoDataTopic;
  static const std::string kObjectListTopic;
  static const std::string kReferenceTrajectoryTopic;
  static const std::string kRouteTopic;

  // output topics
  static const std::string kTrajectoryTopic;

  // parameter names
  static const std::string kOptimizationFreqParam;
  static const std::string kNStatesParam;
  static const std::string kOptimizationHoizonParam;
  static const std::string kVerboseParam;
  static const std::string kCostWeightsParam;
  static const std::string kInitAsRefParam;
  static const std::string kPCostWeightsShapeParam;
  static const std::string kPRefPathShapeParam;

  void declareParameters();
  void loadParameters();
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  void setup();
  void setupSolver(const perception_msgs::msg::EgoData &ego_data);
  void freeSolver();

  void printSolution(int status);

  void egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg);
  void driveableSpaceCallback(const route_planning_msgs::msg::DriveableSpace::ConstSharedPtr msg);
  void objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg);
  void referenceTrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg);

  void planningCycle();
  void updateOcpInputs(const perception_msgs::msg::EgoData &ego_data,
                       const perception_msgs::msg::ObjectList &object_list,
                       const route_planning_msgs::msg::DriveableSpace &driveable_space,
                       const route_planning_msgs::msg::Route &route,
                       const trajectory_planning_msgs::msg::Trajectory &reference_trajectory);

  void setOcpParameters(std::vector<double>& cost_weights,
                        const trajectory_planning_msgs::msg::Trajectory& reference_trajectory);

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr ego_data_sub_;
  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr object_list_sub_;
  rclcpp::Subscription<route_planning_msgs::msg::DriveableSpace>::SharedPtr driveable_space_sub_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr route_sub_;
  rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr reference_trajectory_sub_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

  rclcpp::TimerBase::SharedPtr planning_timer_;

  // input data
  perception_msgs::msg::EgoData ego_data_;
  perception_msgs::msg::ObjectList object_list_;
  route_planning_msgs::msg::DriveableSpace driveable_space_;
  route_planning_msgs::msg::Route route_;
  trajectory_planning_msgs::msg::Trajectory reference_trajectory_;

  // received data flags
  bool received_ego_data_ = false;

  // parameters
  double optimization_freq_ = 10.0;
  int n_states_ = TRAJECTORY_PLANNING_N;
  double optimization_horizon_ = 1.0;
  bool verbose_ = false;
  bool init_as_ref_ = false;

  // cost weights
  std::vector<double> cost_weights_ = {1.0, 1.0, 1.0, 1.0, 1.0};

  // ocp parameter vector structure
  std::vector<long int> p_cost_weights_shape_ = {5};
  std::vector<long int> p_ref_path_shape_ = {100, 4};

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

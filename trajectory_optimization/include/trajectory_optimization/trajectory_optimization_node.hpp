// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <tracetools/tracetools.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

// definitions
#include <perception_msgs/msg/ego_data.hpp>
#include <perception_msgs/msg/object_list.hpp>
#include <route_planning_msgs/msg/route.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// access functions
#include <perception_msgs_utils/object_access.hpp>
#include <route_planning_msgs_utils/route_access.hpp>
#include <trajectory_planning_msgs_utils/trajectory_access.hpp>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_perception_msgs/tf2_perception_msgs.hpp>
#include <tf2_route_planning_msgs/tf2_route_planning_msgs.hpp>
#include <tf2_trajectory_planning_msgs/tf2_trajectory_planning_msgs.hpp>

// acados
#include <trajectory_optimization/ocp_model_handler.hpp>
#include <trajectory_optimization/performance_logger.hpp>

namespace trajectory_optimization {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

class TrajectoryOptimizationNode : public rclcpp::Node {
 public:
  /**
   * @brief Initializes the base node and loads the common optimizer configuration.
   *
   * @param[in] node_name Name of the ROS node instance.
   * @param[in] options ROS node options used for construction.
   */
  explicit TrajectoryOptimizationNode(const std::string node_name, const rclcpp::NodeOptions& options);

  /**
   * @brief Releases solver resources owned by the node.
   */
  ~TrajectoryOptimizationNode() override;

  /**
   * @brief Copy construction is disabled because the node owns non-copyable runtime resources.
   */
  TrajectoryOptimizationNode(const TrajectoryOptimizationNode&) = delete;

  /**
   * @brief Copy assignment is disabled because the node owns non-copyable runtime resources.
   *
   * @return Reference to this node. The operator is deleted.
   */
  TrajectoryOptimizationNode& operator=(const TrajectoryOptimizationNode&) = delete;

  /**
   * @brief Move construction is disabled to keep solver and ROS handles bound to a single instance.
   */
  TrajectoryOptimizationNode(TrajectoryOptimizationNode&&) = delete;

  /**
   * @brief Move assignment is disabled to keep solver and ROS handles bound to a single instance.
   *
   * @return Reference to this node. The operator is deleted.
   */
  TrajectoryOptimizationNode& operator=(TrajectoryOptimizationNode&&) = delete;

 protected:
  enum CONSIDER_BOUNDARIES { NO_BOUNDS = 0, SUGGESTED_LANE = 1, INCLUDING_ADJACENT = 2, DRIVABLE_SPACE = 3 };

  enum CONSIDER_OBJECTS { NO_OBJECTS = 0, STATIC_OBJECTS = 1, PREDICTED_OBJECTS = 2 };

  /**
   * @brief Declares a ROS parameter, loads its value and optionally registers it for runtime updates.
   *
   * @tparam T Parameter value type.
   * @param[in] name Parameter name.
   * @param[out] param Member variable to store the parameter value.
   * @param[in] description Human-readable parameter description.
   * @param[in] add_to_auto_reconfigurable_params Whether parameter updates automatically update the member variable.
   * @param[in] is_required Whether the node should fail if the parameter is not set.
   * @param[in] read_only Whether the parameter is exposed as read-only.
   * @param[in] from_value Optional lower bound for numeric parameters.
   * @param[in] to_value Optional upper bound for numeric parameters.
   * @param[in] step_value Optional step size for numeric parameters.
   * @param[in] additional_constraints Additional free-form constraint text for the parameter descriptor.
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
   * @brief Applies parameter updates that can be reconfigured while the node is running.
   *
   * @param[in] parameters Parameters requested by the ROS parameter service.
   * @return Result of the parameter update request.
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Creates ROS interfaces, initializes cached messages and prepares the solver.
   */
  void setup();

  /**
   * @brief Resets the optimizer memory while retaining the generated solver instance.
   */
  void resetSolver();

  /**
   * @brief Builds and sets a dynamically consistent NLP initial guess from the current state and cached controls.
   *
   * @param[in] x_init Hard initial state of the OCP.
   * @param[in] stamp Absolute time corresponding to x_init.
   * @return `true` if the state rollout succeeded.
   */
  bool setInitialGuess(const std::vector<double>& x_init, const rclcpp::Time& stamp);

  /**
   * @brief Creates the acados solver instance and initializes its state buffers.
   */
  void setupSolver();

  /**
   * @brief Frees the acados solver and clears cached optimization results.
   */
  void freeSolver();

  /**
   * @brief Logs solver status and optional debug statistics for the last optimization run.
   *
   * @param[in] metrics Performance metrics collected for the optimization run.
   */
  void printSolution(const PerformanceMetrics& metrics);

  /**
   * @brief Transforms the planned trajectory into the configured output frame (trajectory_frame_id_) if required.
   *
   * @param[in,out] trajectory Trajectory to transform.
   * @return `true` if the trajectory is already in the target frame or the transformation succeeded.
   */
  bool trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory& trajectory);

  /**
   * @brief Wraps an angle into a configured interval.
   *
   * @param[in] angle_rad Angle in radians.
   * @param[in] min_val Lower bound of the target interval.
   * @param[in] max_val Upper bound of the target interval.
   * @return Wrapped angle in radians.
   */
  static double wrap_angle_rad(double angle_rad, double min_val = -M_PI, double max_val = M_PI);

  /**
   * @brief Interpolates a value from sampled data and optionally handles angle wrap-around.
   *
   * @param[in] X Sample positions.
   * @param[in] Y Sample values.
   * @param[in] desired_x Query position.
   * @param[out] output_y Interpolated result.
   * @param[in] wrap_angle Whether angular differences should be wrapped to `[-pi, pi]`.
   * @return `true` if the query lies within the sample range, otherwise `false`.
   */
  bool linearInterpolation(const std::vector<double>& X,
                           const std::vector<double>& Y,
                           const double& desired_x,
                           double& output_y,
                           const bool wrap_angle = false);

  /**
   * @brief Stores the latest ego state used by the optimizer.
   *
   * @param[in] msg Incoming ego vehicle message.
   */
  void egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg);

  /**
   * @brief Stores the current object list when object handling is enabled.
   *
   * @param[in] msg Incoming object list message.
   */
  void objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg);

  /**
   * @brief Updates the reference trajectory and optionally triggers optimization immediately.
   *
   * @param[in] msg Incoming reference trajectory message.
   */
  void referenceTrajectoryCallback(const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  /**
   * @brief Stores the current route used for boundary constraints when enabled.
   *
   * @param[in] msg Incoming route message.
   */
  void routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg);

  /**
   * @brief Runs one full planning cycle from input preparation, over solver execution, to trajectory publication.
   */
  void planningCycle();

  /**
   * @brief Transforms external inputs into the optimizer frame and writes them into the OCP.
   *
   * @param[in] ego_data Current ego state.
   * @param[in] object_list Current object list.
   * @param[in] route Current route data.
   * @param[in] reference_trajectory Current reference trajectory.
   * @param[in] x_init Initial optimizer state.
   * @return `true` if all optimizer inputs were updated successfully.
   */
  bool updateOcpInputs(const perception_msgs::msg::EgoData& ego_data,
                       const perception_msgs::msg::ObjectList& object_list,
                       const route_planning_msgs::msg::Route& route,
                       const trajectory_planning_msgs::msg::Trajectory& reference_trajectory,
                       const std::vector<double>& x_init);

  /**
   * @brief Writes stage-independent data into the OCP.
   *
   * @param[in] cost_weights Configured cost weights.
   * @param[in] reference_trajectory Reference trajectory in optimizer frame.
   * @param[in] route Route data used for boundary distances.
   */
  void setOcpGlobalParameters(const std::vector<double>& cost_weights,
                              const trajectory_planning_msgs::msg::Trajectory& reference_trajectory,
                              const route_planning_msgs::msg::Route& route);

  /**
   * @brief Writes stage-wise obstacle and dynamic weighting parameters into the OCP.
   *
   * @param[in] ego_data Current ego state.
   * @param[in] object_list Object list in optimizer frame.
   */
  void setOcpParameters(const perception_msgs::msg::EgoData& ego_data, const perception_msgs::msg::ObjectList& object_list);

  /**
   * @brief Computes minimum normal distances from the reference path to the active route boundaries.
   *
   * @param[in] reference_trajectory Reference trajectory in optimizer frame.
   * @param[in] route Route data used to derive active boundaries.
   * @return Left and right boundary distances for each reference sample.
   */
  std::vector<std::pair<double, double>> normalBoundaryDistance(
      const trajectory_planning_msgs::msg::Trajectory& reference_trajectory, const route_planning_msgs::msg::Route& route);

  /**
   * @brief Keeps the nearest forward objects and discards the remaining entries.
   *
   * @param[in,out] object_list Object list to filter.
   * @param[in] n_objects Maximum number of objects to retain.
   */
  static void keepNClosestObjects(perception_msgs::msg::ObjectList& object_list, const int n_objects);

  /**
   * @brief Approximates an oriented bounding box with a set of obstacle circles.
   *
   * @param[in] x Bounding-box center x-coordinate.
   * @param[in] y Bounding-box center y-coordinate.
   * @param[in] yaw Bounding-box heading.
   * @param[in] length Bounding-box length.
   * @param[in] width Bounding-box width.
   * @return Flattened circle list in the configured obstacle parameter layout.
   */
  std::vector<double> discretizeBB2Circles(
      const double x, const double y, const double yaw, const double length, const double width);

  /**
   * @brief Publishes visualization markers for the obstacle circles currently used by the optimizer.
   *
   * @param[in] obstacles Flattened obstacle circle list.
   */
  void vizCircles(const std::vector<double>& obstacles);

  /**
   * @brief Publishes the ego-vehicle circle approximation used by the selected OCP model.
   *
   * @param[in] x_trajectory Optimizer state trajectory.
   * @param[in] model_name Name of the active OCP model.
   */
  void vizEgoCircles(const std::vector<double>& x_trajectory, const std::string& model_name);

  /**
   * @brief Publishes the boundary intersections corresponding to the distances passed to the OCP.
   *
   * @param[in] left_boundary_points Points on the left side.
   * @param[in] right_boundary_points Points on the right side.
   */
  void vizBoundaryPoints(const std::vector<Eigen::Vector2d>& left_boundary_points,
                         const std::vector<Eigen::Vector2d>& right_boundary_points);

  // virtual functions need to be implemented in derived classes

  /**
   * @brief Initializes an output trajectory message with the model-specific message type and size.
   *
   * @param[out] trajectory Trajectory message to initialize.
   */
  virtual void initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) = 0;

  /**
   * @brief Computes the initial optimizer state using bi-level stabilizaion based on the model-specific EgoData type.
   * 
   * This function uses bi-level stabilization for initializing the optimization problem.
   * In general the initial state is interpolated from the latest trajectory (-> low-level stabilization).
   * But if the difference between the interpolated state and the ego state is too large, the ego state is used instead (-> high-level stabilization).
   * This combination of low- and high-level stabilization is called bi-level stabilization.
   *
   * @param[in] ego_data Current ego state.
   * @return Initial state vector for the OCP.
   */
  virtual std::vector<double> getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) = 0;

  /**
   * @brief Computes the initial optimizer state using high-level stabilization based on the model-specific EgoData type.
   *
   * This function uses high-level stabilization for initializing the optimization problem.
   * -> initial state = current state of the ego vehicle.
   *
   * @param[in] ego_data Current ego state.
   * @return Initial state vector for the OCP.
   */
  virtual std::vector<double> getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) = 0;

  /**
   * @brief Maps the optimized state trajectory into the model-specific output message fields.
   *
   * @param[in,out] trajectory Output trajectory message.
   */
  virtual void convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) = 0;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  rclcpp::Subscription<perception_msgs::msg::EgoData>::SharedPtr ego_data_sub_;
  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr object_list_sub_;
  rclcpp::Subscription<route_planning_msgs::msg::Route>::SharedPtr route_sub_;
  rclcpp::Subscription<trajectory_planning_msgs::msg::Trajectory>::SharedPtr reference_trajectory_sub_;

  rclcpp::Publisher<trajectory_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr circles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ego_circles_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundary_pub_;

  rclcpp::TimerBase::SharedPtr planning_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // input data
  perception_msgs::msg::EgoData ego_data_;
  perception_msgs::msg::ObjectList object_list_;
  route_planning_msgs::msg::Route route_;
  trajectory_planning_msgs::msg::Trajectory reference_trajectory_;

  // parameters
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;
  std::string vehicle_frame_id_ = "base_link";
  std::string trajectory_frame_id_ = "base_link";
  std::string fixed_over_time_frame_id_ = "map";
  std::string model_name_ = "karl";
  double ego_data_timeout_ = 1.0;
  double optimization_freq_ = 10.0;
  int n_shots_ = 50;
  double optimization_horizon_ = 1.0;
  bool verbose_ = false;
  bool performance_logging_ = false;
  bool debug_viz_ = false;
  double standstill_threshold_ = 0.45;
  bool high_level_stabilization_ = false;
  bool add_x_init_to_ref_ = false;
  uint8_t consider_objects_ = CONSIDER_OBJECTS::PREDICTED_OBJECTS;
  uint8_t consider_boundaries_ = CONSIDER_BOUNDARIES::SUGGESTED_LANE;
  bool run_as_callback_ = false;

  // common bi-level thresholds
  double bi_level_dV_ = 5.0;
  double bi_level_dA_ = 2.0;
  double bi_level_dY_ = 0.1;
  double bi_level_dYaw_ = 5.0;

  // latest valid trajectory
  trajectory_planning_msgs::msg::Trajectory latest_valid_trajectory_;

  // controls of the latest accepted solution; an empty vector denotes that no warm start is available
  std::vector<double> control_guess_;
  rclcpp::Time control_guess_stamp_{0, 0, RCL_ROS_TIME};

  // visualization
  std::vector<double> viz_circles_;

  // cost weights
  std::vector<double> cost_weights_ = std::vector<double>(12, 1.0);
  double dynamic_weight_ = 1.0;
  double thw_ = 2.0;
  double d_min_obstacle_long_ = 5.0;
  double d_min_obstacle_lat_ = 0.5;
  double d_min_boundary_lat_ = 0.0;
  double min_prediction_probability_ = 0.0;

  // ocp parameter vector structure
  // attention: changes here must also be done in the OCP!
  std::vector<int64_t> p_cost_weights_shape_ = {12, 1};      // nWeights x weightDim
  std::vector<int64_t> p_ref_path_shape_ = {51, 6};          // nStates x [psi, x, y, v, d_bound_left, d_bound_right]
  std::vector<int64_t> p_obstacle_circles_shape_ = {30, 3};  // nObstacleCircles x [x, y, radius]

  // ocp variables
  ocp_model_capsule_t ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  std::vector<double> xtraj_;
  std::vector<double> utraj_;
  uint64_t logging_cycle_ = 0;
  std::unique_ptr<PerformanceLogger> performance_logger_;
};

}  // namespace trajectory_optimization

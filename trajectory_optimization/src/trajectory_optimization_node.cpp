#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

/**
 * @brief Namespace for trajectory_optimization package
 *
 */
namespace trajectory_optimization {

/**
 * @brief Creates a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::TrajectoryOptimizationNode(const std::string node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options) {
  // declare and load node parameters
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_,
                                "Frame ID of local vehicle frame (the ocp is defined in this frame)");
  this->declareAndLoadParameter("trajectory_frame_id", trajectory_frame_id_, "Frame ID of output trajectory");
  this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_,
                                "Frame ID of frame that is fixed over time for finding temporal transforms");
  this->declareAndLoadParameter("ego_data_timeout", ego_data_timeout_,
                                "Time after which a received ego vehicle data is considered invalid [s]. Optimization will not be run if ego data is invalid.");
  this->declareAndLoadParameter("model_name", model_name_, "Name of the model to be used for trajectory optimization [karl, shuttle, shuttle_ackermann, taxi]");
  this->declareAndLoadParameter("optimization_frequency", optimization_freq_, "Optimization Frequency in Hz");
  this->declareAndLoadParameter("n_shots", n_shots_, "Number of shooting intervals in optimization horizon");
  this->declareAndLoadParameter("optimization_horizon", optimization_horizon_, "Optimization Horizon in seconds");
  this->declareAndLoadParameter("verbose", verbose_, "Print solver statistics");
  this->declareAndLoadParameter("debug_visualization", debug_viz_, "Publish debug visualization markers (e.g. obstacle circles)");
  this->declareAndLoadParameter("run_as_callback", run_as_callback_, "Run OCP once for each received reference trajectory (true) or on a timer (false)");
  this->declareAndLoadParameter("cost_weights", cost_weights_, "Cost function weights");
  this->declareAndLoadParameter("dynamic_weight", dynamic_weight_, "Dynamic weight alpha");
  this->declareAndLoadParameter("thw", thw_, "Time headway to front vehicle");
  this->declareAndLoadParameter("d_min_obstacle_long", d_min_obstacle_long_,
                                "Minimum distance to keep to obstacle in longitudinal direction [m]");
  this->declareAndLoadParameter("d_min_obstacle_lat", d_min_obstacle_lat_,
                                "Minimum distance to keep to obstacle in lateral direction [m]");
  this->declareAndLoadParameter("d_min_boundary_lat", d_min_boundary_lat_,
                                "Minimum distance to keep to boundary in lateral direction [m]");
  this->declareAndLoadParameter("standstill_threshold", standstill_threshold_,
                                "Threshold for standstill detection [m/s]. If all state velocities are below this "
                                "threshold, publish standstill trajectory");
  this->declareAndLoadParameter("high_level_stabilization", high_level_stabilization_,
                                "Use high-level stabilization strategy for init state (= init with current EgoData)");
  this->declareAndLoadParameter("add_x_init_to_ref", add_x_init_to_ref_,
                                "add initial state of OCP to beginning of reference trajectory if this starts in front of ego vehicle");
  this->declareAndLoadParameter("consider_objects", consider_objects_,
                                "consider objects in optimization: 0 = none, 1 = static (no prediction), 2 = dynamic (with prediction)");
  this->declareAndLoadParameter("consider_boundaries", consider_boundaries_,
                                "consider route boundaries in optimization: 0 = no, 1 = suggested lane, 2 = including adjacent, 3 = drivable space");
  this->declareAndLoadParameter("bi_level_dV", bi_level_dV_,
                                "Threshold for bi-level stabilization: maximum velocity difference [m/s]");
  this->declareAndLoadParameter("bi_level_dA", bi_level_dA_,
                                "Threshold for bi-level stabilization: maximum acceleration difference [m/s^2]");
  this->declareAndLoadParameter("bi_level_dY", bi_level_dY_,
                                "Threshold for bi-level stabilization: maximum y-offset [m]");
  this->declareAndLoadParameter("bi_level_dYaw", bi_level_dYaw_,
                                "Threshold for bi-level stabilization: maximum yaw difference [degree]");
  this->declareAndLoadParameter("init_as_ref", init_as_ref_, "Boolean that enables initialization of trajectory states as reference states under certain set of conditions");
  this->setup();
}

/**
 * @brief Destroys a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::~TrajectoryOptimizationNode() { freeSolver(); }

template <typename T>
void TrajectoryOptimizationNode::declareAndLoadParameter(const std::string& name,
                                                         T& param,
                                                         const std::string& description,
                                                         const bool add_to_auto_reconfigurable_params,
                                                         const bool is_required,
                                                         const bool read_only,
                                                         const std::optional<double>& from_value,
                                                         const std::optional<double>& to_value,
                                                         const std::optional<double>& step_value,
                                                         const std::string& additional_constraints) {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = description;
  param_desc.additional_constraints = additional_constraints;
  param_desc.read_only = read_only;

  auto type = rclcpp::ParameterValue(param).get_type();

  if (from_value.has_value() && to_value.has_value()) {
    if constexpr(std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr(std::is_floating_point_v<T>) {
      rcl_interfaces::msg::FloatingPointRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.floating_point_range = {range};
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter type of parameter '%s' does not support specifying a range", name.c_str());
    }
  }

  this->declare_parameter(name, type, param_desc);

  try {
    param = this->get_parameter(name).get_value<T>();
    std::stringstream ss;
    ss << "Loaded parameter '" << name << "': ";
    if constexpr(is_vector_v<T>) {
      ss << "[";
      for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
      ss << "]";
    } else {
      ss << param;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    if (is_required) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Missing required parameter '" << name << "', exiting");
      exit(EXIT_FAILURE);
    } else {
      std::stringstream ss;
      ss << "Missing parameter '" << name << "', using default value: ";
      if constexpr(is_vector_v<T>) {
        ss << "[";
        for (const auto& element : param) ss << element << (&element != &param.back() ? ", " : "");
        ss << "]";
      } else {
        ss << param;
      }
      RCLCPP_WARN_STREAM(this->get_logger(), ss.str());
      this->set_parameters({rclcpp::Parameter(name, rclcpp::ParameterValue(param))});
    }
  }

  if (add_to_auto_reconfigurable_params) {
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) {
      param = p.get_value<T>();
    };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(), param.value_to_string().c_str());
        break;
      }
    }
    // handle special cases
    if (param.get_name() == "run_as_callback") {
      if (!run_as_callback_ && !planning_timer_) {
        planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / optimization_freq_),
        std::bind(&TrajectoryOptimizationNode::planningCycle, this));
        RCLCPP_WARN(this->get_logger(), "OCP runs now periodically with frequency %f Hz", optimization_freq_);
      } else if (run_as_callback_ && planning_timer_) {
        planning_timer_->cancel();
        planning_timer_.reset();
        RCLCPP_WARN(this->get_logger(), "OCP runs now on reference trajectory callback");
      }
    }
  }
  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void TrajectoryOptimizationNode::setup() {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryOptimizationNode::parametersCallback, this, std::placeholders::_1));

  // set up subscriber for input topics
  ego_data_sub_ = this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 1, std::bind(&TrajectoryOptimizationNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_sub_->get_topic_name());

  object_list_sub_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      kObjectListTopic, 1, std::bind(&TrajectoryOptimizationNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", object_list_sub_->get_topic_name());

  route_sub_ = this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 1, std::bind(&TrajectoryOptimizationNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", route_sub_->get_topic_name());

  reference_trajectory_sub_ = this->create_subscription<trajectory_planning_msgs::msg::Trajectory>(
      kReferenceTrajectoryTopic, 1,
      std::bind(&TrajectoryOptimizationNode::referenceTrajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", reference_trajectory_sub_->get_topic_name());

  // set up publisher for output topics
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(kTrajectoryTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());
  circles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kObjectMarkerTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", circles_pub_->get_topic_name());
  boundary_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kBoundaryMarkerTopic, 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", boundary_pub_->get_topic_name());

  // create timer for planning cycle
  if (run_as_callback_) {
    RCLCPP_INFO(this->get_logger(), "OCP runs on reference trajectory callback");
  } else {
    RCLCPP_INFO(this->get_logger(), "OCP runs continuously with frequency %f Hz", optimization_freq_);
    planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / optimization_freq_),
                                             std::bind(&TrajectoryOptimizationNode::planningCycle, this));
  }

  // init reference trajectory
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      reference_trajectory_, trajectory_planning_msgs::msg::REFERENCE::TYPE_ID, n_shots_ + 1);
  reference_trajectory_.header.frame_id = vehicle_frame_id_;

  // init latest trajectory (doesn't matter which type, only used for standstill detection)
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      latest_valid_trajectory_, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_shots_ + 1);
  latest_valid_trajectory_.header.frame_id = trajectory_frame_id_;

  setupSolver();
}

void TrajectoryOptimizationNode::setupSolver() {
  // setup acados solver
  acados_ocp_capsule_ = trajectory_optimization::acados_create_capsule(model_name_);
  int status = trajectory_optimization::acados_create(acados_ocp_capsule_);
  nlp_dims_ = trajectory_optimization::acados_get_nlp_dims(acados_ocp_capsule_);

  // allocate the array and fill it accordingly
  double* new_time_steps = NULL;
  if (n_shots_ != nlp_dims_->N) {
    new_time_steps = new double(optimization_horizon_ / n_shots_);
    RCLCPP_INFO(this->get_logger(), "new_time_steps = %f", *new_time_steps);
    status = trajectory_optimization::acados_create_with_discretization(acados_ocp_capsule_, n_shots_, new_time_steps);
  }
  delete[] new_time_steps;

  if (status) {
    RCLCPP_INFO(this->get_logger(), "%s_acados_create() returned status %d. Exiting.", model_name_.c_str(), status);
    exit(1);
  }

  nlp_config_ = trajectory_optimization::acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = trajectory_optimization::acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = trajectory_optimization::acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = trajectory_optimization::acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = trajectory_optimization::acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = trajectory_optimization::acados_get_nlp_opts(acados_ocp_capsule_);

  // initialization of state and control values; set all to zero
  std::vector<double> x_init(*nlp_dims_->nx, 0.0);
  std::vector<double> u_init(*nlp_dims_->nu, 0.0);

  // initialize solution
  for (int i = 0; i < n_shots_; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init.data());
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u_init.data());
  }
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, n_shots_, "x", x_init.data());

  xtraj_ = new double[*nlp_dims_->nx * (n_shots_ + 1)];
  utraj_ = new double[*nlp_dims_->nu * n_shots_];
}

/**
 * @brief Deallocates memory and frees the solver used for trajectory optimization.
 *
 * This function deallocates the memory used by `xtraj_` and `utraj_` arrays.
 * It also frees the solver and the solver capsule used for trajectory planning.
 *
 * @return None.
 */
void TrajectoryOptimizationNode::freeSolver() {
  // deallocate memory
  delete[] xtraj_;
  delete[] utraj_;

  int status;
  // free solver
  status = trajectory_optimization::acados_free(acados_ocp_capsule_);
  if (status) {
    printf("%s_acados_free() returned status %d. \n", model_name_.c_str(), status);
  }
  // free solver capsule
  status = trajectory_optimization::acados_free_capsule(acados_ocp_capsule_);
  if (status) {
    printf("%s_acados_free_capsule() returned status %d. \n", model_name_.c_str(), status);
  }
}

/**
 * @brief Resets the solver by freeing the existing solver and setting up a new one.
 *
 * This function first frees the existing solver by calling the `freeSolver` function,
 * and then sets up a new solver by calling the `setupSolver` function.
 */
void TrajectoryOptimizationNode::resetSolver() {
  freeSolver();
  setupSolver();
}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::planningCycle() {
  if (debug_viz_) viz_circles_.clear();
  if (rclcpp::Time(this->now()) - rclcpp::Time(ego_data_.header.stamp) > rclcpp::Duration::from_seconds(ego_data_timeout_)) {
    RCLCPP_WARN(this->get_logger(), "EgoData outdated. Skipping planning cycle.");
    return;
  }
  // init trajectory message and set header
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  initializeTrajectory(*trajectory);

  trajectory->header.frame_id = vehicle_frame_id_;
  trajectory->header.stamp = ego_data_.header.stamp;  // use latest ego_data stamp as trajectory stamp

  // init time-steps of trajectory to ensure increasing time-steps even for standstill trajectories
  double dt = optimization_horizon_ / n_shots_;
  for (int i = 0; i <= n_shots_; ++i) trajectory_planning_msgs::trajectory_access::setT(*trajectory, i * dt, i);

  // check if the reference trajectory is standstill
  if (trajectory_planning_msgs::trajectory_access::getStandstill(reference_trajectory_)) {
    RCLCPP_WARN(this->get_logger(), "Standstill trajectory. Skipping planning cycle. Publish standstill trajectory.");
    // transform trajectory to output frame
    trajectory2outputFrame(*trajectory);
    trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, true);
    trajectory_pub_->publish(std::move(trajectory));
    resetSolver();
    return;
  }

  // set initial state
  std::vector<double> x_init(*nlp_dims_->nx, 0.0);
  if (!trajectory_planning_msgs::trajectory_access::getStandstill(latest_valid_trajectory_)) {
    x_init = high_level_stabilization_ ? getHighLevelX0(ego_data_) : getBiLevelX0(ego_data_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Latest available trajectory is standstill. Using ego data for initial state (high-level initialization).");
    x_init = getHighLevelX0(ego_data_);
  }

  // debug print of initial state
  std::stringstream ss;
  ss << "Initial state: ";
  for (size_t i = 0; i < x_init.size(); ++i)
    ss << "x[" << i << "]: " << x_init[i] << (i != x_init.size() - 1 ? ", " : "");
  RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_init.data());
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_init.data());

  // update inputs to the ocp; skip planning cycle if update fails
  if (!updateOcpInputs(ego_data_, object_list_, route_, reference_trajectory_, x_init)) {
    RCLCPP_WARN(this->get_logger(), "Failed to update inputs. Skipping planning cycle.");
    return;
  }

  // solve the optimization problem
  int status = trajectory_optimization::acados_solve(acados_ocp_capsule_);

  // get solution
  for (int ii = 0; ii <= nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * *nlp_dims_->nx]);
  for (int ii = 0; ii < nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * *nlp_dims_->nu]);

  printSolution(status);
  if (debug_viz_) vizCircles(viz_circles_);

  if (status == 1 || status == 3 || status == 4) {
    RCLCPP_ERROR(this->get_logger(), "Solver failed with status %d.", status);
    resetSolver();
    return;
  }

  // convert output into trajectory message
  convertToTrajectoryMsg(*trajectory);

  bool standstill = true;
  for (int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(*trajectory); ++i) {
    if (trajectory_planning_msgs::trajectory_access::getV(*trajectory, i) > standstill_threshold_) standstill = false;
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, standstill);
  if (standstill) {
    resetSolver();
  }

  // transform trajectory to output frame
  trajectory2outputFrame(*trajectory);

  latest_valid_trajectory_ = *trajectory;
  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_INFO(this->get_logger(), "Published trajectory");
}

/**
 * @brief Updates the inputs for the ocp.
 *
 * @param ego_data
 * @param object_list
 * @param route (currently unused)
 * @param reference_trajectory
 * @return True if the inputs were successfully updated, false otherwise.
 */
bool TrajectoryOptimizationNode::updateOcpInputs(
    const perception_msgs::msg::EgoData& ego_data, const perception_msgs::msg::ObjectList& object_list,
    const route_planning_msgs::msg::Route& route,
    const trajectory_planning_msgs::msg::Trajectory& reference_trajectory,
    const std::vector<double>& x_init) {
  // transform inputs to target base_link frame
  trajectory_planning_msgs::msg::Trajectory tf_reference_trajectory;
  perception_msgs::msg::ObjectList tf_object_list;
  route_planning_msgs::msg::Route tf_route;
  try {
    // reference trajectory
    tf_reference_trajectory =
        tf2_buffer_->transform(reference_trajectory, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    if (add_x_init_to_ref_ && trajectory_planning_msgs::trajectory_access::getX(tf_reference_trajectory, 0) > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Adding x_init to beginning of reference trajectory");
      std::vector<double> x_0_ref = {0.0, x_init[0], x_init[1], x_init[3]};
      tf_reference_trajectory.states.insert(tf_reference_trajectory.states.begin(), x_0_ref.begin(), x_0_ref.end());
    }
    // object list
    if (!object_list.objects.empty() && object_list.header.frame_id != vehicle_frame_id_) {
      tf_object_list = tf2_buffer_->transform(object_list, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                                              fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    } else {
      tf_object_list = object_list;
    }
    keepNClosestObjects(tf_object_list, p_obstacle_circles_shape_[0]);
    // route
    if (!route.route_elements.empty() && route.header.frame_id != vehicle_frame_id_) {
      tf_route = tf2_buffer_->transform(route, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                                        fixed_over_time_frame_id_, tf2::durationFromSec(0.1));
    } else {
      tf_route = route;
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation is not available. Ex: %s", ex.what());
    return false;
  }

  if (init_as_ref_ && trajectory_planning_msgs::trajectory_access::getStandstill(latest_valid_trajectory_)) {
    // set initial guess
    std::vector<double> initial_guess(*nlp_dims_->nx, 0.0);
    for (int i = 0; i <= n_shots_; ++i) {
      int idx = std::min(i, trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_reference_trajectory)-1);
      initial_guess[0] = trajectory_planning_msgs::trajectory_access::getX(tf_reference_trajectory, idx);
      initial_guess[1] = trajectory_planning_msgs::trajectory_access::getY(tf_reference_trajectory, idx);
      initial_guess[3] = trajectory_planning_msgs::trajectory_access::getV(tf_reference_trajectory, idx);
      initial_guess[5] = trajectory_planning_msgs::trajectory_access::getTheta(tf_reference_trajectory, idx);
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", initial_guess.data());
    }
  }

  // update ocp parameters
  try {
    this->setOcpGlobalParameters(cost_weights_, tf_reference_trajectory, tf_route);
    this->setOcpParameters(ego_data, tf_object_list);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while setting OCP parameters: %s", e.what());
    return false;
  }

  return true;
}

void TrajectoryOptimizationNode::setOcpGlobalParameters(const std::vector<double>& cost_weights,
                                                        const trajectory_planning_msgs::msg::Trajectory& reference_trajectory,
                                                        const route_planning_msgs::msg::Route& route) {
    const auto start_time = std::chrono::steady_clock::now();
    std::vector<double> global_params;

    // cost weights
    global_params.insert(global_params.end(), cost_weights.begin(), cost_weights.end());

    // other cost params
    global_params.push_back(thw_);
    global_params.push_back(d_min_obstacle_long_);
    global_params.push_back(d_min_obstacle_lat_);
    global_params.push_back(d_min_boundary_lat_);

    // reference path (including boundaries)
    int n_ref_states = p_ref_path_shape_[0] * p_ref_path_shape_[1];
    std::vector<std::pair<double, double>> boundary_distances = normalBoundaryDistance(reference_trajectory, route);
    // fill ref vector for ocp -> psi, x, y, v, d_bound_left, d_bound_right
    std::vector<double> ref;
    for (int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory); ++i) {
      ref.push_back(trajectory_planning_msgs::trajectory_access::getTheta(reference_trajectory, i));
      ref.push_back(trajectory_planning_msgs::trajectory_access::getX(reference_trajectory, i));
      ref.push_back(trajectory_planning_msgs::trajectory_access::getY(reference_trajectory, i));
      ref.push_back(trajectory_planning_msgs::trajectory_access::getV(reference_trajectory, i));
      ref.push_back(boundary_distances[i].first);   // left boundary distance
      ref.push_back(boundary_distances[i].second);  // right boundary distance
    }
    if (ref.size() >= static_cast<size_t>(n_ref_states)) {
      global_params.insert(global_params.end(), ref.begin(), ref.begin() + n_ref_states);
    } else {
      // TODO: what to do here? Currently just copy the whole reference trajectory and rest is filled with infinity
      global_params.insert(global_params.end(), ref.begin(), ref.end());
      global_params.insert(global_params.end(), n_ref_states - ref.size(), std::numeric_limits<double>::infinity());
    }

    if (global_params.size() != (size_t)nlp_dims_->np_global) {
      RCLCPP_ERROR(this->get_logger(), "Size of global parameters (%ld) does not match expected size (%d).", global_params.size(), nlp_dims_->np_global);
      throw std::runtime_error("Size of global parameters does not match expected size.");
    }
    trajectory_optimization::acados_set_p_global_and_precompute_dependencies(acados_ocp_capsule_, global_params.data(), global_params.size());
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_DEBUG(this->get_logger(), "setOcpGlobalParameters duration: %.3f ms", elapsed_ms);
}

void TrajectoryOptimizationNode::setOcpParameters(const perception_msgs::msg::EgoData& ego_data,
                                                  const perception_msgs::msg::ObjectList& object_list) {
  const auto start_time = std::chrono::steady_clock::now();
  // loop over shooting intervals
  double floating_dynamic_weight = 1.0;
  double dt = optimization_horizon_ / n_shots_;
  for (int i = 0; i <= n_shots_; ++i) {
    int idx, n;

    // dynamic weight
    idx = 0;
    n = 1;
    std::vector<int> idx_dynamic_weight(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_dynamic_weight.begin(), idx_dynamic_weight.end(), idx);
    trajectory_optimization::acados_update_params_sparse(acados_ocp_capsule_, i, idx_dynamic_weight.data(), &floating_dynamic_weight, n);
    floating_dynamic_weight *= dynamic_weight_;

    // obstacles
    idx += n;
    n = p_obstacle_circles_shape_[0] * p_obstacle_circles_shape_[1];
    std::vector<double> circles;  // [x1, y1, r1, x2, y2, r2, ...]

    for (size_t j = 0; j < object_list.objects.size(); ++j) {
      double x_tgt, y_tgt, yaw_tgt;
      std::vector<double> TIME, X, Y, YAW;
      // TODO: should not be done for each shooting interval. Could be improved.
      TIME.push_back(rclcpp::Time(object_list.header.stamp).nanoseconds() / 1e9);
      X.push_back(perception_msgs::object_access::getX(object_list.objects[j]));
      Y.push_back(perception_msgs::object_access::getY(object_list.objects[j]));
      YAW.push_back(perception_msgs::object_access::getYaw(object_list.objects[j]));
      if (consider_objects_ == CONSIDER_OBJECTS::PREDICTED_OBJECTS && object_list.objects[j].state_predictions.size() > 0) {
        for (auto &predicted_state: object_list.objects[j].state_predictions[0].states) {
          TIME.push_back(rclcpp::Time(predicted_state.header.stamp).nanoseconds() / 1e9);
          X.push_back(perception_msgs::object_access::getX(predicted_state));
          Y.push_back(perception_msgs::object_access::getY(predicted_state));
          YAW.push_back(perception_msgs::object_access::getYaw(predicted_state));
        }
        double des_time = rclcpp::Time(ego_data.header.stamp).nanoseconds() / 1e9 + dt * i;
        linearInterpolation(TIME, X, des_time, x_tgt);
        linearInterpolation(TIME, Y, des_time, y_tgt);
        linearInterpolation(TIME, YAW, des_time, yaw_tgt, true);
      } else {
        x_tgt = X.front();
        y_tgt = Y.front();
        yaw_tgt = YAW.front();
      }
      // ensure that x_tgt and y_tgt represent the geometric center of the object
      double alpha = std::atan2(object_list.objects[j].state.reference_point.translation_to_geometric_center.y, object_list.objects[j].state.reference_point.translation_to_geometric_center.x);
      double beta = wrap_angle_rad(yaw_tgt - alpha);
      double a = std::sqrt(std::pow(object_list.objects[j].state.reference_point.translation_to_geometric_center.x, 2) + std::pow(object_list.objects[j].state.reference_point.translation_to_geometric_center.y, 2));
      x_tgt += a * std::cos(beta);
      y_tgt += a * std::sin(beta);

      std::vector<double> obj_circles = discretizeBB2Circles(x_tgt, y_tgt, yaw_tgt,
                                                         perception_msgs::object_access::getLength(object_list.objects[j]),
                                                         perception_msgs::object_access::getWidth(object_list.objects[j]));

      circles.insert(circles.end(), obj_circles.begin(), obj_circles.end());
      if (circles.size() >= (size_t)n) {
        circles.resize(n);
        break;
      }
    }
    // fill up with dummy "ghost" obstacle circles at (10000, 10000) to avoid NaNs in the optimization problem
    // TODO: improve this
    while (circles.size() < (size_t)n) {
      std::vector<double> dummy_circle = {10000.0, 10000.0, 1.0};
      circles.insert(circles.end(), dummy_circle.begin(), dummy_circle.end());
    }
    if ((circles.size() % p_obstacle_circles_shape_[1]) != 0) {
      RCLCPP_WARN(this->get_logger(), "Circles vector size is not a multiple of the circle shape. Resizing.");
      circles.resize(n);
    }
    if (debug_viz_) viz_circles_.insert(viz_circles_.end(), circles.begin(), circles.end());

    std::vector<int> idx_obstacles(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_obstacles.begin(), idx_obstacles.end(), idx);
    trajectory_optimization::acados_update_params_sparse(acados_ocp_capsule_, i, idx_obstacles.data(), circles.data(), n);
  }
  const auto elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
  RCLCPP_DEBUG(this->get_logger(), "setOcpParameters duration: %.3f ms", elapsed_ms);
}

}  // namespace trajectory_optimization

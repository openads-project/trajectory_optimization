// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
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

namespace {
using SteadyClock = std::chrono::steady_clock;

double elapsedMilliseconds(const SteadyClock::time_point& start) {
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - start).count();
}

double elapsedMilliseconds(const SteadyClock::time_point& start, const SteadyClock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

}  // namespace

TrajectoryOptimizationNode::TrajectoryOptimizationNode(const std::string node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options) {
  // declare and load node parameters
  this->declareAndLoadParameter("vehicle_frame_id", vehicle_frame_id_,
                                "Frame ID of local vehicle frame (the ocp is defined in this frame)");
  this->declareAndLoadParameter("trajectory_frame_id", trajectory_frame_id_, "Frame ID of output trajectory");
  this->declareAndLoadParameter("fixed_over_time_frame_id", fixed_over_time_frame_id_,
                                "Frame ID of frame that is fixed over time for finding temporal transforms");
  this->declareAndLoadParameter("ego_data_timeout", ego_data_timeout_,
                                "Time after which a received ego vehicle data is considered invalid [s]. Optimization will not "
                                "be run if ego data is invalid.");
  this->declareAndLoadParameter("model_name", model_name_,
                                "Name of the model to be used for trajectory optimization [karl, shuttle]");
  this->declareAndLoadParameter("collision_geometry", collision_geometry_, "Collision geometry implementation [circles, obb_sat]",
                                false, false, true, std::nullopt, std::nullopt, std::nullopt, "Must be 'circles' or 'obb_sat'.");
  this->declareAndLoadParameter("optimization_frequency", optimization_freq_, "Optimization frequency in Hz");
  this->declareAndLoadParameter("n_shots", n_shots_, "Number of shooting intervals in optimization horizon");
  this->declareAndLoadParameter("optimization_horizon", optimization_horizon_, "Optimization Horizon in seconds");
  this->declareAndLoadParameter("verbose", verbose_, "Print solver statistics");
  this->declareAndLoadParameter("performance_logging", performance_logging_,
                                "Write one CSV record for every scheduled planning tick", false, false, true);
  this->declareAndLoadParameter("debug_visualization", debug_viz_, "Publish debug visualization markers (e.g. obstacle circles)");
  this->declareAndLoadParameter("run_as_callback", run_as_callback_,
                                "Run OCP once for each received reference trajectory (true) or on a timer (false)");
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
                                "Threshold for standstill detection [m/s]. If the velocities of all states are below this "
                                "threshold, publish standstill trajectory");
  this->declareAndLoadParameter("high_level_stabilization", high_level_stabilization_,
                                "Use high-level stabilization strategy for init state (= init with current EgoData)");
  this->declareAndLoadParameter(
      "consider_objects", consider_objects_,
      "consider objects in optimization: 0 = none, 1 = static (no prediction), 2 = dynamic (with prediction)");
  this->declareAndLoadParameter("min_prediction_probability", min_prediction_probability_,
                                "Minimum probability for predicted object states to be considered", true, false, false, 0.0, 1.0);
  this->declareAndLoadParameter(
      "consider_boundaries", consider_boundaries_,
      "consider route boundaries in optimization: 0 = no, 1 = suggested lane, 2 = including adjacent, 3 = drivable space");
  this->declareAndLoadParameter("bi_level_dV", bi_level_dV_,
                                "Threshold for bi-level stabilization: maximum velocity difference [m/s]");
  this->declareAndLoadParameter("bi_level_dY", bi_level_dY_, "Threshold for bi-level stabilization: maximum y-offset [m]");
  this->declareAndLoadParameter("bi_level_dYaw", bi_level_dYaw_,
                                "Threshold for bi-level stabilization: maximum yaw difference [degree]");
  if (collision_geometry_ != "circles" && collision_geometry_ != "obb_sat") {
    throw std::invalid_argument("Invalid collision_geometry: " + collision_geometry_);
  }
  solver_model_name_ = collision_geometry_ == "obb_sat" ? model_name_ + "_obb_sat" : model_name_;
  if (performance_logging_) {
    try {
      performance_logger_ = std::make_unique<PerformanceLogger>(get_name());
      RCLCPP_INFO(get_logger(), "Writing benchmark logs to '%s'.", performance_logger_->path().c_str());
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Could not initialize benchmark logging: %s", error.what());
    }
  }
  this->setup();
}

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
    if constexpr (std::is_integral_v<T>) {
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(static_cast<T>(from_value.value())).set__to_value(static_cast<T>(to_value.value()));
      if (step_value.has_value()) range.set__step(static_cast<T>(step_value.value()));
      param_desc.integer_range = {range};
    } else if constexpr (std::is_floating_point_v<T>) {
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
    if constexpr (is_vector_v<T>) {
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
      if constexpr (is_vector_v<T>) {
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
    std::function<void(const rclcpp::Parameter&)> setter = [&param](const rclcpp::Parameter& p) { param = p.get_value<T>(); };
    auto_reconfigurable_params_.push_back(std::make_tuple(name, setter));
  }
}

rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(),
                    param.value_to_string().c_str());
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

void TrajectoryOptimizationNode::setup() {
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryOptimizationNode::parametersCallback, this, std::placeholders::_1));

  // set up subscriber for input topics
  ego_data_sub_ = this->create_subscription<perception_msgs::msg::EgoData>(
      "~/ego_data", 1, std::bind(&TrajectoryOptimizationNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_sub_->get_topic_name());

  object_list_sub_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      "~/object_list", 1, std::bind(&TrajectoryOptimizationNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", object_list_sub_->get_topic_name());

  route_sub_ = this->create_subscription<route_planning_msgs::msg::Route>(
      "~/route", 1, std::bind(&TrajectoryOptimizationNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", route_sub_->get_topic_name());

  reference_trajectory_sub_ = this->create_subscription<trajectory_planning_msgs::msg::Trajectory>(
      "~/reference_trajectory", 1,
      std::bind(&TrajectoryOptimizationNode::referenceTrajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", reference_trajectory_sub_->get_topic_name());

  // set up publisher for output topics
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>("~/trajectory", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());
  circles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization/object_circles", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", circles_pub_->get_topic_name());
  ego_circles_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization/ego_circles", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", ego_circles_pub_->get_topic_name());
  boundary_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization/boundaries", 1);
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

  // Annotate message links for tracing: Publish trajectory periodically, which depends an all subscribed topics.
  std::vector<const void*> link_subs;
  link_subs.push_back(static_cast<const void*>(ego_data_sub_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void*>(object_list_sub_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void*>(route_sub_->get_subscription_handle().get()));
  link_subs.push_back(static_cast<const void*>(reference_trajectory_sub_->get_subscription_handle().get()));
  std::vector<const void*> link_pubs;
  link_pubs.push_back(static_cast<const void*>(trajectory_pub_->get_publisher_handle().get()));
  RCLCPP_INFO(get_logger(), "Annotating message links for tracing with %zu subscriptions and %zu publications", link_subs.size(),
              link_pubs.size());
  TRACETOOLS_TRACEPOINT(message_link_periodic_async, link_subs.data(), link_subs.size(), link_pubs.data(), link_pubs.size());
}

void TrajectoryOptimizationNode::setupSolver() {
  if (n_shots_ <= 0) {
    RCLCPP_FATAL(this->get_logger(), "n_shots must be > 0, got %d", n_shots_);
    exit(1);
  }

  // Create exactly once. This also supports a runtime horizon that differs from the generated default.
  ocp_capsule_ = trajectory_optimization::acados_create_capsule(solver_model_name_);
  std::vector<double> new_time_steps(n_shots_, optimization_horizon_ / n_shots_);
  RCLCPP_INFO(this->get_logger(), "Create OCP with: horizon = %f, n_shots = %d, dt = %f", optimization_horizon_, n_shots_,
              new_time_steps.front());
  int status = trajectory_optimization::acados_create_with_discretization(ocp_capsule_, n_shots_, new_time_steps.data());

  if (status != ACADOS_SUCCESS) {
    RCLCPP_FATAL(this->get_logger(), "%s_acados_create_with_discretization() returned status %d. Exiting.",
                 solver_model_name_.c_str(), status);
    exit(1);
  }

  nlp_config_ = trajectory_optimization::acados_get_nlp_config(ocp_capsule_);
  nlp_dims_ = trajectory_optimization::acados_get_nlp_dims(ocp_capsule_);
  nlp_in_ = trajectory_optimization::acados_get_nlp_in(ocp_capsule_);
  nlp_out_ = trajectory_optimization::acados_get_nlp_out(ocp_capsule_);
  nlp_solver_ = trajectory_optimization::acados_get_nlp_solver(ocp_capsule_);
  nlp_opts_ = trajectory_optimization::acados_get_nlp_opts(ocp_capsule_);
  if (nlp_dims_->N != n_shots_) {
    RCLCPP_FATAL(this->get_logger(), "Created solver has N=%d, expected n_shots=%d. Exiting.", nlp_dims_->N, n_shots_);
    exit(1);
  }

  xtraj_.resize(*nlp_dims_->nx * (n_shots_ + 1));
  utraj_.resize(*nlp_dims_->nu * n_shots_);
  control_guess_.clear();
}

void TrajectoryOptimizationNode::freeSolver() {
  // free solver
  int status = trajectory_optimization::acados_free(ocp_capsule_);
  if (status != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s_acados_free() returned status %d.", solver_model_name_.c_str(), status);
  }
  // free solver capsule
  status = trajectory_optimization::acados_free_capsule(ocp_capsule_);
  if (status != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s_acados_free_capsule() returned status %d.", solver_model_name_.c_str(), status);
  }
}

void TrajectoryOptimizationNode::resetSolver() {
  const int status = trajectory_optimization::acados_reset(ocp_capsule_, 1, 0, 0, 0);
  if (status != ACADOS_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "%s_acados_reset() returned status %d. Recreating solver.", model_name_.c_str(), status);
    freeSolver();
    setupSolver();
  }
}

bool TrajectoryOptimizationNode::setInitialGuess(const std::vector<double>& x_init, const rclcpp::Time& stamp) {
  constexpr double MAX_CONTROL_GUESS_AGE_FACTOR = 0.5;
  const int nx = *nlp_dims_->nx;
  const int nu = *nlp_dims_->nu;
  const double time_step = optimization_horizon_ / n_shots_;
  const size_t expected_control_size = static_cast<size_t>(nu * n_shots_);
  if (x_init.size() != static_cast<size_t>(nx)) {
    RCLCPP_ERROR(get_logger(), "Initial state has size %zu, expected %d.", x_init.size(), nx);
    return false;
  }

  // Fall back to zero controls if no sufficiently recent solver output is available.
  std::vector<double> controls(expected_control_size, 0.0);
  if (control_guess_.size() == expected_control_size) {
    const double elapsed = (stamp - control_guess_stamp_).seconds();
    if (elapsed >= 0.0 && elapsed < MAX_CONTROL_GUESS_AGE_FACTOR * optimization_horizon_) {
      // Shift the previous controls to the current planning time and interpolate between shooting nodes.
      std::vector<double> lower_controls(nu);
      std::vector<double> upper_controls(nu);
      for (int stage = 0; stage < n_shots_; ++stage) {
        const double previous_stage = (elapsed + stage * time_step) / time_step;
        if (previous_stage >= n_shots_) break;

        const int lower_stage = static_cast<int>(std::floor(previous_stage));
        const double interpolation_factor = previous_stage - lower_stage;
        ocp_nlp_constraints_model_get(nlp_config_, nlp_dims_, nlp_in_, stage, "lbu", lower_controls.data());
        ocp_nlp_constraints_model_get(nlp_config_, nlp_dims_, nlp_in_, stage, "ubu", upper_controls.data());
        for (int control = 0; control < nu; ++control) {
          const double lower_value = control_guess_[lower_stage * nu + control];
          const double upper_value = lower_stage + 1 < n_shots_ ? control_guess_[(lower_stage + 1) * nu + control] : 0.0;
          const double interpolated_control = lower_value + interpolation_factor * (upper_value - lower_value);
          controls[stage * nu + control] = std::clamp(interpolated_control, lower_controls[control], upper_controls[control]);
        }
      }
    }
  }

  ocp_nlp_out_set_values_to_zero(nlp_config_, nlp_dims_, nlp_out_);
  std::vector<double> rollout_state = x_init;
  std::vector<double> intermediate_state(nx);
  std::vector<double> k1(nx), k2(nx), k3(nx), k4(nx);
  const double integration_step = time_step / 2.0;
  for (int stage = 0; stage < n_shots_; ++stage) {
    double* control = &controls[stage * nu];
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "x", rollout_state.data());
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, stage, "u", control);

    // Match sim_method_num_stages=4 and sim_method_num_steps=2 from the generated OCP.
    for (int integration = 0; integration < 2; ++integration) {
      trajectory_optimization::acados_evaluate_dynamics(ocp_capsule_, stage, rollout_state.data(), control, k1.data());
      for (int i = 0; i < nx; ++i) intermediate_state[i] = rollout_state[i] + 0.5 * integration_step * k1[i];
      trajectory_optimization::acados_evaluate_dynamics(ocp_capsule_, stage, intermediate_state.data(), control, k2.data());
      for (int i = 0; i < nx; ++i) intermediate_state[i] = rollout_state[i] + 0.5 * integration_step * k2[i];
      trajectory_optimization::acados_evaluate_dynamics(ocp_capsule_, stage, intermediate_state.data(), control, k3.data());
      for (int i = 0; i < nx; ++i) intermediate_state[i] = rollout_state[i] + integration_step * k3[i];
      trajectory_optimization::acados_evaluate_dynamics(ocp_capsule_, stage, intermediate_state.data(), control, k4.data());
      for (int i = 0; i < nx; ++i) {
        rollout_state[i] += integration_step / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
      }
    }
  }
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, n_shots_, "x", rollout_state.data());
  return true;
}

void TrajectoryOptimizationNode::planningCycle() {
  const auto cycle_start = SteadyClock::now();
  PerformanceMetrics metrics;
  metrics.cycle = ++logging_cycle_;
  metrics.ego_stamp_ns = rclcpp::Time(ego_data_.header.stamp).nanoseconds();
  metrics.reference_stamp_ns = rclcpp::Time(reference_trajectory_.header.stamp).nanoseconds();
  metrics.route_stamp_ns = rclcpp::Time(route_.header.stamp).nanoseconds();
  metrics.reference_points = trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory_);
  metrics.objects = static_cast<int>(object_list_.objects.size());
  metrics.collision_geometry = collision_geometry_;
  auto logCompletedCycle = [&]() {
    metrics.cycle_ms = elapsedMilliseconds(cycle_start);
    metrics.postprocessing_ms = metrics.cycle_ms - metrics.preprocessing_ms - metrics.solve_wall_ms;
    if (performance_logger_) performance_logger_->write(metrics);
  };

  if (debug_viz_) {
    viz_circles_.clear();
    viz_obbs_.clear();
  }
  if (rclcpp::Time(this->now()) - rclcpp::Time(ego_data_.header.stamp) > rclcpp::Duration::from_seconds(ego_data_timeout_)) {
    RCLCPP_WARN(this->get_logger(), "EgoData outdated. Skipping planning cycle.");
    metrics.outcome = "skipped_ego_outdated";
    logCompletedCycle();
    return;
  }
  // init trajectory message and set header
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory = std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
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
    if (!trajectory2outputFrame(*trajectory)) {
      metrics.outcome = "skipped_standstill_transform";
      logCompletedCycle();
      return;
    }
    trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, true);
    trajectory_pub_->publish(std::move(trajectory));
    metrics.published = true;
    metrics.outcome = "published_standstill";
    // Invalidate the warm start and reset the solver once when entering standstill.
    if (!control_guess_.empty()) {
      control_guess_.clear();
      resetSolver();
    }
    logCompletedCycle();
    return;
  }

  // set initial state
  std::vector<double> x_init(*nlp_dims_->nx, 0.0);
  if (!trajectory_planning_msgs::trajectory_access::getStandstill(latest_valid_trajectory_)) {
    x_init = high_level_stabilization_ ? getHighLevelX0(ego_data_) : getBiLevelX0(ego_data_);
  } else {
    RCLCPP_WARN(this->get_logger(),
                "Latest available trajectory is standstill. Using ego data for initial state (high-level initialization).");
    x_init = getHighLevelX0(ego_data_);
  }

  // debug print of initial state
  std::stringstream ss;
  ss << "Initial state: ";
  for (size_t i = 0; i < x_init.size(); ++i) ss << "x[" << i << "]: " << x_init[i] << (i != x_init.size() - 1 ? ", " : "");
  RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_init.data());
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_init.data());

  // update inputs to the ocp; skip planning cycle if update fails
  if (!updateOcpInputs(ego_data_, object_list_, route_, reference_trajectory_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to update inputs. Skipping planning cycle.");
    metrics.outcome = "skipped_input_update";
    logCompletedCycle();
    return;
  }
  metrics.obstacle_hypotheses = obstacle_hypotheses_;
  metrics.dropped_obstacle_hypotheses = dropped_obstacle_hypotheses_;
  metrics.parameter_update_ms = parameter_update_ms_;

  if (!setInitialGuess(x_init, rclcpp::Time(ego_data_.header.stamp))) {
    control_guess_.clear();
    metrics.outcome = "skipped_initial_guess";
    logCompletedCycle();
    return;
  }

  // solve the optimization problem
  const auto solve_start = SteadyClock::now();
  metrics.preprocessing_ms = elapsedMilliseconds(cycle_start, solve_start);
  metrics.solver_ran = true;
  metrics.status = trajectory_optimization::acados_solve(ocp_capsule_);
  const auto solve_end = SteadyClock::now();
  metrics.solve_wall_ms = elapsedMilliseconds(solve_start, solve_end);

  PerformanceLogger::collectSolverStatistics(metrics, nlp_solver_, nlp_config_, nlp_dims_, nlp_in_, nlp_out_,
                                             performance_logger_ != nullptr || verbose_);

  const bool usable_status =
      metrics.status == ACADOS_SUCCESS || metrics.status == ACADOS_MAXITER || metrics.status == ACADOS_TIMEOUT;
  if (usable_status) {
    for (int ii = 0; ii <= nlp_dims_->N; ++ii) {
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * *nlp_dims_->nx]);
    }
    for (int ii = 0; ii < nlp_dims_->N; ++ii) {
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * *nlp_dims_->nu]);
    }
  }

  const auto* solver_opts = trajectory_optimization::acados_get_common_nlp_opts(nlp_config_, nlp_opts_);
  const bool finite_solution = usable_status && std::isfinite(metrics.cost_value) && std::isfinite(metrics.res_eq) &&
                               std::isfinite(metrics.res_ineq) &&
                               std::all_of(xtraj_.begin(), xtraj_.end(), [](double value) { return std::isfinite(value); }) &&
                               std::all_of(utraj_.begin(), utraj_.end(), [](double value) { return std::isfinite(value); });
  const bool primal_feasible =
      finite_solution && metrics.res_eq <= solver_opts->tol_eq && metrics.res_ineq <= solver_opts->tol_ineq;

  if (!primal_feasible) {
    printSolution(metrics);
    RCLCPP_WARN(this->get_logger(),
                "Rejecting solver output: status=%d finite=%d primal residuals=[eq=%e, ineq=%e] tolerances=[eq=%e, ineq=%e].",
                metrics.status, finite_solution, metrics.res_eq, metrics.res_ineq, solver_opts->tol_eq, solver_opts->tol_ineq);
    if (finite_solution && performance_logger_) {
      const auto geometry = vehicleGeometry(model_name_);
      const int ego_primitives = collision_geometry_ == "obb_sat" ? 1 : geometry.circle_count;
      const int obstacle_primitives = collision_geometry_ == "obb_sat" ? static_cast<int>(p_obstacle_obbs_shape_[0])
                                                                       : static_cast<int>(p_obstacle_circles_shape_[0]);
      PerformanceLogger::collectConstraintDiagnostics(metrics, nlp_solver_, nlp_dims_, 2 * ego_primitives,
                                                      obstacle_primitives * ego_primitives);
    }
    if (finite_solution && (metrics.status == ACADOS_MAXITER || metrics.status == ACADOS_TIMEOUT)) {
      // Preserve progress from a recoverable solve; the states are rolled out again from the next x_init.
      control_guess_ = utraj_;
      control_guess_stamp_ = rclcpp::Time(ego_data_.header.stamp);
    } else {
      control_guess_.clear();
    }
    resetSolver();
    metrics.outcome = "rejected_solver";
    logCompletedCycle();
    return;
  }

  control_guess_ = utraj_;
  control_guess_stamp_ = rclcpp::Time(ego_data_.header.stamp);
  collectGeometryValidation(metrics);

  printSolution(metrics);
  if (debug_viz_) {
    if (collision_geometry_ == "obb_sat") {
      vizObbs(viz_obbs_);
      vizEgoObbs(xtraj_, model_name_);
    } else {
      vizCircles(viz_circles_);
      vizEgoCircles(xtraj_, model_name_);
    }
  }

  // convert output into trajectory message
  convertToTrajectoryMsg(*trajectory);

  bool standstill = true;
  for (int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(*trajectory); ++i) {
    if (trajectory_planning_msgs::trajectory_access::getV(*trajectory, i) > standstill_threshold_) standstill = false;
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, standstill);

  // transform trajectory to output frame
  if (!trajectory2outputFrame(*trajectory)) {
    metrics.outcome = "skipped_output_transform";
    logCompletedCycle();
    return;
  }

  latest_valid_trajectory_ = *trajectory;
  trajectory_pub_->publish(std::move(trajectory));
  metrics.published = true;
  metrics.outcome = "published_optimized";
  logCompletedCycle();
  const char* cycle_time_color = metrics.cycle_ms <= 100.0 ? "\x1b[32m" : "\x1b[31m";
  RCLCPP_INFO(this->get_logger(), "Published trajectory (cycle: %s%.2f ms\x1b[0m)", cycle_time_color, metrics.cycle_ms);
}

void TrajectoryOptimizationNode::collectGeometryValidation(PerformanceMetrics& metrics) const {
  metrics.geometry_validated = true;
  const auto geometry = vehicleGeometry(model_name_);
  const int state_dimension = *nlp_dims_->nx;

  auto ego_box_at_stage = [&](int stage) {
    const size_t offset = static_cast<size_t>(stage) * static_cast<size_t>(state_dimension);
    return orientedBoxFromReference(xtraj_[offset], xtraj_[offset + 1], xtraj_[offset + 5], geometry.length, geometry.width,
                                    geometry.center_offset_long, geometry.center_offset_lat);
  };
  auto boundary_penetration = [&](const OrientedBox& ego_box) {
    return std::max(boxBoundaryViolationDepth(ego_box, validation_left_boundary_, true),
                    boxBoundaryViolationDepth(ego_box, validation_right_boundary_, false));
  };

  for (int stage = 0; stage <= n_shots_; ++stage) {
    const auto ego_box = ego_box_at_stage(stage);
    const double boundary_depth = boundary_penetration(ego_box);
    if (boundary_depth > 1e-9) ++metrics.node_boundary_violations;
    if (std::isfinite(boundary_depth)) {
      metrics.max_node_boundary_penetration_m = std::max(metrics.max_node_boundary_penetration_m, boundary_depth);
    }
    for (size_t hypothesis = 0; hypothesis < validation_obstacle_boxes_.size(); ++hypothesis) {
      const auto& boxes = validation_obstacle_boxes_[hypothesis];
      if (static_cast<size_t>(stage) >= boxes.size()) continue;
      if (exactSatSeparationMargin(ego_box, boxes[stage]) <= 0.0) {
        ++metrics.node_object_collisions;
        if (hypothesis >= validation_selected_hypotheses_.size() || !validation_selected_hypotheses_[hypothesis]) {
          ++metrics.dropped_hypothesis_collisions;
        }
      }
    }
  }

  constexpr int INTERSAMPLES_PER_INTERVAL = 10;
  for (int stage = 0; stage < n_shots_; ++stage) {
    const auto first_ego_box = ego_box_at_stage(stage);
    const auto second_ego_box = ego_box_at_stage(stage + 1);
    for (int substep = 1; substep <= INTERSAMPLES_PER_INTERVAL; ++substep) {
      const double factor = static_cast<double>(substep) / static_cast<double>(INTERSAMPLES_PER_INTERVAL + 1);
      const auto ego_box = interpolateBox(first_ego_box, second_ego_box, factor);
      const double boundary_depth = boundary_penetration(ego_box);
      if (boundary_depth > 1e-9) ++metrics.intersample_boundary_violations;
      if (std::isfinite(boundary_depth)) {
        metrics.max_intersample_boundary_penetration_m = std::max(metrics.max_intersample_boundary_penetration_m, boundary_depth);
      }
      for (const auto& boxes : validation_obstacle_boxes_) {
        if (static_cast<size_t>(stage + 1) >= boxes.size()) continue;
        if (exactSatSeparationMargin(ego_box, interpolateBox(boxes[stage], boxes[stage + 1], factor)) <= 0.0) {
          ++metrics.intersample_object_collisions;
        }
      }
    }
  }

  if (metrics.node_object_collisions > 0 || metrics.node_boundary_violations > 0) {
    RCLCPP_ERROR(get_logger(),
                 "Exact geometry validator (%s) found node violations: objects=%d boundaries=%d "
                 "max_boundary_penetration=%.3f m dropped_hypotheses=%d.",
                 collision_geometry_.c_str(), metrics.node_object_collisions, metrics.node_boundary_violations,
                 metrics.max_node_boundary_penetration_m, metrics.dropped_hypothesis_collisions);
  }
}

bool TrajectoryOptimizationNode::updateOcpInputs(const perception_msgs::msg::EgoData& ego_data,
                                                 const perception_msgs::msg::ObjectList& object_list,
                                                 const route_planning_msgs::msg::Route& route,
                                                 const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  // transform inputs to target base_link frame
  trajectory_planning_msgs::msg::Trajectory tf_reference_trajectory;
  perception_msgs::msg::ObjectList tf_object_list;
  route_planning_msgs::msg::Route tf_route;
  try {
    // reference trajectory
    tf_reference_trajectory =
        tf2_buffer_->transform(reference_trajectory, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    // object list
    if (!object_list.objects.empty() && object_list.header.frame_id != vehicle_frame_id_) {
      tf_object_list = tf2_buffer_->transform(object_list, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                                              fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    } else {
      tf_object_list = object_list;
    }
    if (collision_geometry_ == "circles") {
      keepNClosestObjects(tf_object_list, static_cast<int>(p_obstacle_circles_shape_[0]));
    }
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

  if (trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_reference_trajectory) <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Reference trajectory contains no sample points.");
    return false;
  }

  validation_left_boundary_.clear();
  validation_right_boundary_.clear();
  for (const auto& route_element : tf_route.route_elements) {
    if (!route_element.is_enriched) continue;
    if (consider_boundaries_ == CONSIDER_BOUNDARIES::SUGGESTED_LANE && !route_element.lane_elements.empty()) {
      const size_t suggested_index =
          std::min(static_cast<size_t>(route_element.suggested_lane_idx), route_element.lane_elements.size() - 1);
      const auto& lane = route_element.lane_elements[suggested_index];
      validation_left_boundary_.push_back({lane.left_boundary.point.x, lane.left_boundary.point.y});
      validation_right_boundary_.push_back({lane.right_boundary.point.x, lane.right_boundary.point.y});
    } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::INCLUDING_ADJACENT && !route_element.lane_elements.empty()) {
      validation_left_boundary_.push_back(
          {route_element.lane_elements.front().left_boundary.point.x, route_element.lane_elements.front().left_boundary.point.y});
      validation_right_boundary_.push_back(
          {route_element.lane_elements.back().right_boundary.point.x, route_element.lane_elements.back().right_boundary.point.y});
    } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::DRIVABLE_SPACE) {
      validation_left_boundary_.push_back({route_element.left_boundary.x, route_element.left_boundary.y});
      validation_right_boundary_.push_back({route_element.right_boundary.x, route_element.right_boundary.y});
    }
  }

  // update ocp parameters
  try {
    this->setOcpGlobalParameters(cost_weights_, tf_reference_trajectory, tf_route);
    this->setOcpParameters(ego_data, tf_object_list, tf_reference_trajectory);
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
  const auto expected_cost_weights_size = static_cast<size_t>(p_cost_weights_shape_[0] * p_cost_weights_shape_[1]);
  if (cost_weights.size() != expected_cost_weights_size) {
    RCLCPP_ERROR(this->get_logger(), "Size of cost_weights (%zu) does not match expected size (%zu).", cost_weights.size(),
                 expected_cost_weights_size);
    throw std::runtime_error("Size of cost_weights does not match expected size.");
  }
  global_params.insert(global_params.end(), cost_weights.begin(), cost_weights.end());

  // other cost params
  global_params.push_back(thw_);
  global_params.push_back(d_min_obstacle_long_);
  global_params.push_back(d_min_obstacle_lat_);
  global_params.push_back(d_min_boundary_lat_);

  // reference path (including boundaries)
  const size_t n_ref_states = static_cast<size_t>(p_ref_path_shape_[0] * p_ref_path_shape_[1]);
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
  if (ref.size() >= n_ref_states) {
    global_params.insert(global_params.end(), ref.begin(),
                         ref.begin() + static_cast<std::vector<double>::difference_type>(n_ref_states));
  } else {
    // Repeat the final valid state. Infinity padding can propagate NaNs through closest-point calculations.
    global_params.insert(global_params.end(), ref.begin(), ref.end());
    const size_t state_width = static_cast<size_t>(p_ref_path_shape_[1]);
    while (global_params.size() < expected_cost_weights_size + 4 + n_ref_states) {
      global_params.insert(global_params.end(), ref.end() - static_cast<std::vector<double>::difference_type>(state_width),
                           ref.end());
    }
  }

  if (global_params.size() != static_cast<size_t>(nlp_dims_->np_global)) {
    RCLCPP_ERROR(this->get_logger(), "Size of global parameters (%zu) does not match expected size (%d).", global_params.size(),
                 nlp_dims_->np_global);
    throw std::runtime_error("Size of global parameters does not match expected size.");
  }
  const int status = trajectory_optimization::acados_set_p_global_and_precompute_dependencies(
      ocp_capsule_, global_params.data(), static_cast<int>(global_params.size()));
  if (status != ACADOS_SUCCESS) {
    throw std::runtime_error("acados global parameter update failed with status " + std::to_string(status));
  }
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
  RCLCPP_DEBUG(this->get_logger(), "setOcpGlobalParameters duration: %.3f ms", elapsed_ms);
}

void TrajectoryOptimizationNode::setOcpParameters(const perception_msgs::msg::EgoData& ego_data,
                                                  const perception_msgs::msg::ObjectList& object_list,
                                                  const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  const auto start_time = std::chrono::steady_clock::now();

  // The exact validator always sees every input hypothesis, independently of the
  // approximation and capacity used by the selected OCP.
  if (collision_geometry_ == "circles") {
    constexpr double MIN_HALF_EXTENT = 0.05;
    const int stage_count = n_shots_ + 1;
    const double dt = optimization_horizon_ / n_shots_;
    const double ego_stamp = static_cast<double>(rclcpp::Time(ego_data.header.stamp).nanoseconds()) / 1e9;
    const double object_stamp = static_cast<double>(rclcpp::Time(object_list.header.stamp).nanoseconds()) / 1e9;
    validation_obstacle_boxes_.clear();
    validation_selected_hypotheses_.clear();

    for (const auto& object : object_list.objects) {
      auto append_validation_hypothesis = [&](const perception_msgs::msg::ObjectStatePrediction* prediction) {
        std::vector<double> times{object_stamp};
        std::vector<double> x{perception_msgs::object_access::getX(object)};
        std::vector<double> y{perception_msgs::object_access::getY(object)};
        std::vector<double> yaw{perception_msgs::object_access::getYaw(object)};
        if (prediction != nullptr) {
          for (const auto& state : prediction->states) {
            times.push_back(static_cast<double>(rclcpp::Time(state.header.stamp).nanoseconds()) / 1e9);
            x.push_back(perception_msgs::object_access::getX(state));
            y.push_back(perception_msgs::object_access::getY(state));
            yaw.push_back(perception_msgs::object_access::getYaw(state));
          }
        }

        std::vector<OrientedBox> boxes;
        boxes.reserve(stage_count);
        const double offset_long = object.state.reference_point.translation_to_geometric_center.x;
        const double offset_lat = object.state.reference_point.translation_to_geometric_center.y;
        const double length = std::max(2.0 * MIN_HALF_EXTENT, perception_msgs::object_access::getLength(object));
        const double width = std::max(2.0 * MIN_HALF_EXTENT, perception_msgs::object_access::getWidth(object));
        for (int stage = 0; stage < stage_count; ++stage) {
          const double desired_time = ego_stamp + dt * stage;
          boxes.push_back(orientedBoxFromReference(
              interpolateSample(times, x, desired_time), interpolateSample(times, y, desired_time),
              interpolateSample(times, yaw, desired_time, true), length, width, offset_long, offset_lat));
        }
        validation_obstacle_boxes_.push_back(std::move(boxes));
        validation_selected_hypotheses_.push_back(true);
      };

      bool prediction_added = false;
      if (consider_objects_ == CONSIDER_OBJECTS::PREDICTED_OBJECTS) {
        for (const auto& prediction : object.state_predictions) {
          if (prediction.probability > min_prediction_probability_) {
            append_validation_hypothesis(&prediction);
            prediction_added = true;
          }
        }
      }
      if (!prediction_added) append_validation_hypothesis(nullptr);
    }
    obstacle_hypotheses_ = static_cast<int>(validation_obstacle_boxes_.size());
    dropped_obstacle_hypotheses_ = 0;
  }

  if (collision_geometry_ == "obb_sat") {
    struct ObstacleHypothesis {
      uint64_t object_id;
      int prediction_index;
      double probability;
      uint8_t classification;
      std::vector<OrientedBox> boxes;
      double minimum_reference_gap;
      int earliest_minimum_stage;
    };

    constexpr double MIN_HALF_EXTENT = 0.05;
    const int stage_count = n_shots_ + 1;
    const double dt = optimization_horizon_ / n_shots_;
    const double ego_stamp = static_cast<double>(rclcpp::Time(ego_data.header.stamp).nanoseconds()) / 1e9;
    const double object_stamp = static_cast<double>(rclcpp::Time(object_list.header.stamp).nanoseconds()) / 1e9;
    const auto ego_geometry = vehicleGeometry(model_name_);
    std::vector<ObstacleHypothesis> hypotheses;

    for (const auto& object : object_list.objects) {
      uint8_t classification = 0;
      double classification_probability = -1.0;
      for (const auto& candidate : object.state.classifications) {
        if (candidate.probability > classification_probability) {
          classification = candidate.type;
          classification_probability = candidate.probability;
        }
      }

      auto append_hypothesis = [&](const perception_msgs::msg::ObjectStatePrediction* prediction, int prediction_index) {
        std::vector<double> times{object_stamp};
        std::vector<double> x{perception_msgs::object_access::getX(object)};
        std::vector<double> y{perception_msgs::object_access::getY(object)};
        std::vector<double> yaw{perception_msgs::object_access::getYaw(object)};
        if (prediction != nullptr) {
          for (const auto& state : prediction->states) {
            times.push_back(static_cast<double>(rclcpp::Time(state.header.stamp).nanoseconds()) / 1e9);
            x.push_back(perception_msgs::object_access::getX(state));
            y.push_back(perception_msgs::object_access::getY(state));
            yaw.push_back(perception_msgs::object_access::getYaw(state));
          }
        }

        ObstacleHypothesis hypothesis{object.id,
                                      prediction_index,
                                      prediction != nullptr ? prediction->probability : 1.0,
                                      classification,
                                      {},
                                      std::numeric_limits<double>::infinity(),
                                      0};
        hypothesis.boxes.reserve(stage_count);
        const double offset_long = object.state.reference_point.translation_to_geometric_center.x;
        const double offset_lat = object.state.reference_point.translation_to_geometric_center.y;
        const double length = std::max(2.0 * MIN_HALF_EXTENT, perception_msgs::object_access::getLength(object));
        const double width = std::max(2.0 * MIN_HALF_EXTENT, perception_msgs::object_access::getWidth(object));
        const int reference_points = trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory);

        for (int stage = 0; stage < stage_count; ++stage) {
          const double desired_time = ego_stamp + dt * stage;
          const auto box =
              orientedBoxFromReference(interpolateSample(times, x, desired_time), interpolateSample(times, y, desired_time),
                                       interpolateSample(times, yaw, desired_time, true), length, width, offset_long, offset_lat);
          hypothesis.boxes.push_back(box);

          const int reference_index = std::min(stage, reference_points - 1);
          const double reference_yaw =
              trajectory_planning_msgs::trajectory_access::getTheta(reference_trajectory, reference_index);
          auto reference_ego = orientedBoxFromReference(
              trajectory_planning_msgs::trajectory_access::getX(reference_trajectory, reference_index),
              trajectory_planning_msgs::trajectory_access::getY(reference_trajectory, reference_index), reference_yaw,
              ego_geometry.length, ego_geometry.width, ego_geometry.center_offset_long, ego_geometry.center_offset_lat);
          const double reference_velocity =
              std::max(0.0, trajectory_planning_msgs::trajectory_access::getV(reference_trajectory, reference_index));
          const double rear_margin = std::max(0.0, d_min_obstacle_long_);
          const double front_margin = std::max(rear_margin, thw_ * reference_velocity);
          reference_ego = expandBoxForward(reference_ego, front_margin, rear_margin, d_min_obstacle_lat_);
          const double gap = exactSatSeparationMargin(reference_ego, box);
          if (gap < hypothesis.minimum_reference_gap) {
            hypothesis.minimum_reference_gap = gap;
            hypothesis.earliest_minimum_stage = stage;
          }
        }
        hypotheses.push_back(std::move(hypothesis));
      };

      bool prediction_added = false;
      if (consider_objects_ == CONSIDER_OBJECTS::PREDICTED_OBJECTS) {
        for (size_t prediction_index = 0; prediction_index < object.state_predictions.size(); ++prediction_index) {
          const auto& prediction = object.state_predictions[prediction_index];
          if (prediction.probability > min_prediction_probability_) {
            append_hypothesis(&prediction, static_cast<int>(prediction_index));
            prediction_added = true;
          }
        }
      }
      if (!prediction_added) append_hypothesis(nullptr, -1);
    }

    std::vector<HypothesisPriority> priorities;
    priorities.reserve(hypotheses.size());
    for (const auto& hypothesis : hypotheses) {
      priorities.push_back({hypothesis.minimum_reference_gap, hypothesis.earliest_minimum_stage, hypothesis.object_id,
                            hypothesis.prediction_index});
    }
    std::vector<ObstacleHypothesis> ranked_hypotheses;
    ranked_hypotheses.reserve(hypotheses.size());
    for (const size_t index : rankHypotheses(priorities)) ranked_hypotheses.push_back(std::move(hypotheses[index]));
    hypotheses = std::move(ranked_hypotheses);

    obstacle_hypotheses_ = static_cast<int>(hypotheses.size());
    const size_t capacity = static_cast<size_t>(p_obstacle_obbs_shape_[0]);
    validation_obstacle_boxes_.clear();
    validation_selected_hypotheses_.clear();
    validation_obstacle_boxes_.reserve(hypotheses.size());
    validation_selected_hypotheses_.reserve(hypotheses.size());
    selected_hypothesis_metadata_.clear();
    for (size_t index = 0; index < hypotheses.size(); ++index) {
      validation_obstacle_boxes_.push_back(hypotheses[index].boxes);
      validation_selected_hypotheses_.push_back(index < capacity);
      if (index < capacity) {
        selected_hypothesis_metadata_.push_back({hypotheses[index].object_id, hypotheses[index].prediction_index,
                                                 hypotheses[index].probability, hypotheses[index].classification});
      }
    }
    dropped_obstacle_hypotheses_ = static_cast<int>(hypotheses.size() > capacity ? hypotheses.size() - capacity : 0);
    if (hypotheses.size() > capacity) {
      std::ostringstream dropped;
      const size_t report_end = std::min(hypotheses.size(), capacity + 20);
      for (size_t index = capacity; index < report_end; ++index) {
        if (index != capacity) dropped << ',';
        dropped << hypotheses[index].object_id << ':' << hypotheses[index].prediction_index << ':'
                << static_cast<int>(hypotheses[index].classification);
      }
      RCLCPP_WARN(get_logger(), "Dropped %d of %d OBB hypotheses after deterministic risk ranking: [%s%s]",
                  dropped_obstacle_hypotheses_, obstacle_hypotheses_, dropped.str().c_str(),
                  hypotheses.size() > report_end ? ",..." : "");
      hypotheses.resize(capacity);
    }

    double floating_dynamic_weight = 1.0;
    for (int stage = 0; stage < stage_count; ++stage) {
      int parameter_index = 0;
      int parameter_count = 1;
      std::vector<int> dynamic_weight_indices{parameter_index};
      int status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, stage, dynamic_weight_indices.data(),
                                                                        &floating_dynamic_weight, parameter_count);
      if (status != ACADOS_SUCCESS) {
        throw std::runtime_error("acados dynamic-weight update failed with status " + std::to_string(status));
      }
      floating_dynamic_weight *= dynamic_weight_;

      parameter_index += parameter_count;
      parameter_count = static_cast<int>(p_obstacle_obbs_shape_[0] * p_obstacle_obbs_shape_[1]);
      std::vector<double> obbs;
      obbs.reserve(parameter_count);
      for (const auto& hypothesis : hypotheses) {
        const auto& box = hypothesis.boxes[stage];
        obbs.insert(obbs.end(), {box.x, box.y, box.yaw, box.half_length, box.half_width});
      }
      while (obbs.size() < static_cast<size_t>(parameter_count)) {
        obbs.insert(obbs.end(), {10000.0, 10000.0, 0.0, MIN_HALF_EXTENT, MIN_HALF_EXTENT});
      }
      if (debug_viz_) viz_obbs_.insert(viz_obbs_.end(), obbs.begin(), obbs.end());

      std::vector<int> obstacle_indices(parameter_count);
      std::iota(obstacle_indices.begin(), obstacle_indices.end(), parameter_index);
      status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, stage, obstacle_indices.data(), obbs.data(),
                                                                    parameter_count);
      if (status != ACADOS_SUCCESS) {
        throw std::runtime_error("acados OBB update failed with status " + std::to_string(status));
      }
    }
    parameter_update_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_DEBUG(get_logger(), "setOcpParameters OBB duration: %.3f ms", parameter_update_ms_);
    return;
  }

  selected_hypothesis_metadata_.clear();
  struct PredictionData {
    std::vector<double> time;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    size_t index;
    double probability;
  };
  std::vector<std::vector<PredictionData>> object_predictions(object_list.objects.size());
  const double object_stamp = static_cast<double>(rclcpp::Time(object_list.header.stamp).nanoseconds()) / 1e9;
  for (size_t j = 0; j < object_list.objects.size(); ++j) {
    const auto& object = object_list.objects[j];
    if (consider_objects_ != CONSIDER_OBJECTS::PREDICTED_OBJECTS || object.state_predictions.empty()) continue;

    auto append_prediction = [&](const auto& state_prediction, size_t prediction_idx) {
      PredictionData prediction{{object_stamp},
                                {perception_msgs::object_access::getX(object)},
                                {perception_msgs::object_access::getY(object)},
                                {perception_msgs::object_access::getYaw(object)},
                                prediction_idx,
                                state_prediction.probability};
      for (const auto& predicted_state : state_prediction.states) {
        prediction.time.push_back(static_cast<double>(rclcpp::Time(predicted_state.header.stamp).nanoseconds()) / 1e9);
        prediction.x.push_back(perception_msgs::object_access::getX(predicted_state));
        prediction.y.push_back(perception_msgs::object_access::getY(predicted_state));
        prediction.yaw.push_back(perception_msgs::object_access::getYaw(predicted_state));
      }
      object_predictions[j].push_back(std::move(prediction));
    };

    for (size_t prediction_idx = 0; prediction_idx < object.state_predictions.size(); ++prediction_idx) {
      const auto& state_prediction = object.state_predictions[prediction_idx];
      if (state_prediction.probability > min_prediction_probability_) {
        append_prediction(state_prediction, prediction_idx);
      }
    }
    if (object_predictions[j].empty()) append_prediction(object.state_predictions[0], 0);
  }

  // loop over shooting intervals
  double floating_dynamic_weight = 1.0;
  double dt = optimization_horizon_ / n_shots_;
  for (int i = 0; i <= n_shots_; ++i) {
    // dynamic weight
    int idx = 0;
    int n = 1;
    std::vector<int> idx_dynamic_weight(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_dynamic_weight.begin(), idx_dynamic_weight.end(), idx);
    int status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, i, idx_dynamic_weight.data(),
                                                                      &floating_dynamic_weight, n);
    if (status != ACADOS_SUCCESS) {
      throw std::runtime_error("acados dynamic-weight update failed with status " + std::to_string(status));
    }
    floating_dynamic_weight *= dynamic_weight_;

    // obstacles
    idx += n;
    n = static_cast<int>(p_obstacle_circles_shape_[0] * p_obstacle_circles_shape_[1]);
    std::vector<double> circles;  // [x1, y1, r1, x2, y2, r2, ...]

    for (size_t j = 0; j < object_list.objects.size(); ++j) {
      std::vector<std::tuple<double, double, double>> target_states;
      if (!object_predictions[j].empty()) {
        for (const auto& prediction : object_predictions[j]) {
          double x_tgt = 0.0, y_tgt = 0.0, yaw_tgt = 0.0;
          double des_time = static_cast<double>(rclcpp::Time(ego_data.header.stamp).nanoseconds()) / 1e9 + dt * i;
          if (des_time > prediction.time.back()) {
            const double relative_des_time = des_time - prediction.time.front();
            const double relative_max_time = prediction.time.back() - prediction.time.front();
            RCLCPP_WARN(this->get_logger(),
                        "Prediction horizon shorter than requested interpolation time. "
                        "object=%zu prediction=%zu probability=%.3f desired_rel=%.3f s max_rel=%.3f s n_states=%zu. "
                        "Using last prediction state.",
                        j, prediction.index, prediction.probability, relative_des_time, relative_max_time,
                        prediction.time.size() - 1);
            x_tgt = prediction.x.back();
            y_tgt = prediction.y.back();
            yaw_tgt = prediction.yaw.back();
          } else {
            linearInterpolation(prediction.time, prediction.x, des_time, x_tgt);
            linearInterpolation(prediction.time, prediction.y, des_time, y_tgt);
            linearInterpolation(prediction.time, prediction.yaw, des_time, yaw_tgt, true);
          }
          target_states.emplace_back(x_tgt, y_tgt, yaw_tgt);
        }
      } else {
        target_states.emplace_back(perception_msgs::object_access::getX(object_list.objects[j]),
                                   perception_msgs::object_access::getY(object_list.objects[j]),
                                   perception_msgs::object_access::getYaw(object_list.objects[j]));
      }

      double alpha = std::atan2(object_list.objects[j].state.reference_point.translation_to_geometric_center.y,
                                object_list.objects[j].state.reference_point.translation_to_geometric_center.x);
      double a = std::sqrt(std::pow(object_list.objects[j].state.reference_point.translation_to_geometric_center.x, 2) +
                           std::pow(object_list.objects[j].state.reference_point.translation_to_geometric_center.y, 2));
      for (auto& [x_tgt, y_tgt, yaw_tgt] : target_states) {
        // ensure that x_tgt and y_tgt represent the geometric center of the object
        double beta = wrap_angle_rad(yaw_tgt - alpha);
        x_tgt += a * std::cos(beta);
        y_tgt += a * std::sin(beta);

        std::vector<double> obj_circles =
            discretizeBB2Circles(x_tgt, y_tgt, yaw_tgt, perception_msgs::object_access::getLength(object_list.objects[j]),
                                 perception_msgs::object_access::getWidth(object_list.objects[j]));

        circles.insert(circles.end(), obj_circles.begin(), obj_circles.end());
        if (circles.size() >= static_cast<size_t>(n)) {
          circles.resize(n);
          break;
        }
      }
      if (circles.size() >= static_cast<size_t>(n)) {
        break;
      }
    }
    // fill up with dummy "ghost" obstacle circles at (10000, 10000) to avoid NaNs in the optimization problem
    // TODO: improve this  // NOLINT(google-readability-todo)
    while (circles.size() < static_cast<size_t>(n)) {
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
    status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, i, idx_obstacles.data(), circles.data(), n);
    if (status != ACADOS_SUCCESS) {
      throw std::runtime_error("acados obstacle update failed with status " + std::to_string(status));
    }
  }
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
  parameter_update_ms_ = elapsed_ms;
  RCLCPP_DEBUG(this->get_logger(), "setOcpParameters duration: %.3f ms", elapsed_ms);
}

}  // namespace trajectory_optimization

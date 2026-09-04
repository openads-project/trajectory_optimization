// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <tuple>

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
  this->declareAndLoadParameter("optimization_frequency", optimization_freq_, "Optimization frequency in Hz");
  this->declareAndLoadParameter("n_shots", n_shots_, "Number of shooting intervals in optimization horizon");
  this->declareAndLoadParameter("optimization_horizon", optimization_horizon_, "Optimization Horizon in seconds");
  this->declareAndLoadParameter("two_stage_optimization", two_stage_optimization_,
                                "Initialize the constrained OCP with a solve without object and boundary constraints");
  this->declareAndLoadParameter("relaxed_solve_timeout_ms", relaxed_solve_timeout_ms_,
                                "ACADOS timeout for the relaxed initialization solve [ms]; 0 disables the timeout");
  this->declareAndLoadParameter("constrained_solve_timeout_ms", constrained_solve_timeout_ms_,
                                "ACADOS timeout for the fully constrained solve [ms]; 0 disables the timeout");
  this->declareAndLoadParameter("verbose", verbose_, "Print solver statistics");
  this->declareAndLoadParameter("performance_logging", performance_logging_,
                                "Write one CSV record for every completed solver run");
  this->declareAndLoadParameter("debug_visualization", debug_viz_,
                                "Publish debug visualization markers for ego, objects and boundaries");
  this->declareAndLoadParameter("run_as_callback", run_as_callback_,
                                "Run OCP once for each received reference trajectory (true) or on a timer (false)");
  this->declareAndLoadParameter("cost_weights", cost_weights_, "Cost function weights");
  this->declareAndLoadParameter("dynamic_weight", dynamic_weight_, "Dynamic weight alpha");
  this->declareAndLoadParameter("thw", thw_, "Time headway to front vehicle");
  this->declareAndLoadParameter("d_min_object_long", d_min_object_long_,
                                "Minimum distance to keep to objects in longitudinal direction [m]");
  this->declareAndLoadParameter("d_min_object_lat", d_min_object_lat_,
                                "Minimum distance to keep to objects in lateral direction [m]");
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
  object_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization/objects", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", object_marker_pub_->get_topic_name());
  ego_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/visualization/ego", 1);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", ego_marker_pub_->get_topic_name());
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
  ocp_capsule_ = trajectory_optimization::acados_create_capsule(model_name_);
  std::vector<double> new_time_steps(n_shots_, optimization_horizon_ / n_shots_);
  RCLCPP_INFO(this->get_logger(), "Create OCP with: horizon = %f, n_shots = %d, dt = %f", optimization_horizon_, n_shots_,
              new_time_steps.front());
  int status = trajectory_optimization::acados_create_with_discretization(ocp_capsule_, n_shots_, new_time_steps.data());

  if (status != ACADOS_SUCCESS) {
    RCLCPP_FATAL(this->get_logger(), "%s_acados_create_with_discretization() returned status %d. Exiting.", model_name_.c_str(),
                 status);
    exit(1);
  }

  nlp_config_ = trajectory_optimization::acados_get_nlp_config(ocp_capsule_);
  nlp_dims_ = trajectory_optimization::acados_get_nlp_dims(ocp_capsule_);
  nlp_in_ = trajectory_optimization::acados_get_nlp_in(ocp_capsule_);
  nlp_out_ = trajectory_optimization::acados_get_nlp_out(ocp_capsule_);
  nlp_solver_ = trajectory_optimization::acados_get_nlp_solver(ocp_capsule_);
  nlp_opts_ = trajectory_optimization::acados_get_nlp_opts(ocp_capsule_);
  setSolverTimeout(constrained_solve_timeout_ms_);
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
    RCLCPP_ERROR(this->get_logger(), "%s_acados_free() returned status %d.", model_name_.c_str(), status);
  }
  // free solver capsule
  status = trajectory_optimization::acados_free_capsule(ocp_capsule_);
  if (status != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s_acados_free_capsule() returned status %d.", model_name_.c_str(), status);
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
  if (debug_viz_) viz_object_boxes_.clear();
  if (rclcpp::Time(this->now()) - rclcpp::Time(ego_data_.header.stamp) > rclcpp::Duration::from_seconds(ego_data_timeout_)) {
    RCLCPP_WARN(this->get_logger(), "EgoData outdated. Skipping planning cycle.");
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
      return;
    }
    trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, true);
    trajectory_pub_->publish(std::move(trajectory));
    // Invalidate the warm start and reset the solver once when entering standstill.
    if (!control_guess_.empty()) {
      control_guess_.clear();
      resetSolver();
    }
    return;
  }

  PerformanceMetrics metrics;
  metrics.cycle = ++logging_cycle_;
  metrics.ego_stamp_ns = rclcpp::Time(ego_data_.header.stamp).nanoseconds();
  metrics.reference_stamp_ns = rclcpp::Time(reference_trajectory_.header.stamp).nanoseconds();
  metrics.route_stamp_ns = rclcpp::Time(route_.header.stamp).nanoseconds();
  metrics.reference_points = trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory_);
  metrics.objects = static_cast<int>(object_list_.objects.size());
  auto logCompletedCycle = [&]() {
    metrics.cycle_ms = elapsedMilliseconds(cycle_start);
    metrics.postprocessing_ms = metrics.cycle_ms - metrics.preprocessing_ms - metrics.solve_wall_ms;
    if (performance_logger_) {
      performance_logger_->write(metrics);
    }
  };

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
    return;
  }

  if (!setInitialGuess(x_init, rclcpp::Time(ego_data_.header.stamp))) {
    control_guess_.clear();
    return;
  }

  // start solving the OCP
  // first solve the relaxed OCP (if configured and necessary)
  // then solve the constrained OCP using either the relaxed solution or the regular warm start
  const auto solve_start = SteadyClock::now();
  metrics.preprocessing_ms = elapsedMilliseconds(cycle_start, solve_start);
  const auto* solver_opts = trajectory_optimization::acados_get_common_nlp_opts(nlp_config_, nlp_opts_);
  bool finite_solution = false;
  bool primal_feasible = false;

  // solve the relaxed OCP (if configured and necessary)
  if (two_stage_optimization_ && (consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS || object_constraints_active_)) {
    metrics.relaxed_attempted = true;
    if (!setSafetyConstraintActivation(false)) return;

    setSolverTimeout(relaxed_solve_timeout_ms_);
    const auto relaxed_solve_start = SteadyClock::now();
    const bool relaxed_primal_feasible = solveAndValidate(metrics, finite_solution);
    metrics.relaxed_solve_wall_ms = elapsedMilliseconds(relaxed_solve_start);
    metrics.relaxed_status = metrics.status;

    // evaluate the relaxed residuals before restoration, but retain its nlp_out_
    if (!setSafetyConstraintActivation(true)) return;

    if (!relaxed_primal_feasible) {
      metrics.failure_phase = "relaxed";
      metrics.solve_wall_ms = elapsedMilliseconds(solve_start);
      printSolution(metrics);
      RCLCPP_WARN(get_logger(),
                  "Rejecting relaxed initialization: status=%d finite=%d primal residuals=[eq=%e, ineq=%e] "
                  "tolerances=[eq=%e, ineq=%e].",
                  metrics.status, finite_solution, metrics.res_eq, metrics.res_ineq, solver_opts->tol_eq, solver_opts->tol_ineq);
      resetSolver();
      logCompletedCycle();
      return;
    }
  }

  // solve the constrained OCP using either the relaxed solution or the regular warm start
  setSolverTimeout(constrained_solve_timeout_ms_);
  const auto constrained_solve_start = SteadyClock::now();
  primal_feasible = solveAndValidate(metrics, finite_solution);
  metrics.constrained_solve_wall_ms = elapsedMilliseconds(constrained_solve_start);
  metrics.solve_wall_ms = elapsedMilliseconds(solve_start);

  if (!primal_feasible) {
    metrics.failure_phase = "constrained";
    printSolution(metrics);
    RCLCPP_WARN(this->get_logger(),
                "Rejecting solver output: status=%d finite=%d primal residuals=[eq=%e, ineq=%e] tolerances=[eq=%e, ineq=%e].",
                metrics.status, finite_solution, metrics.res_eq, metrics.res_ineq, solver_opts->tol_eq, solver_opts->tol_ineq);
    if (finite_solution && performance_logger_) {
      PerformanceLogger::collectConstraintDiagnostics(metrics, nlp_solver_, nlp_dims_, static_cast<int>(p_objects_shape_[0]));
    }
    // keep the last published solution as warm-start source
    resetSolver();
    logCompletedCycle();
    return;
  }

  control_guess_ = utraj_;
  control_guess_stamp_ = rclcpp::Time(ego_data_.header.stamp);

  printSolution(metrics);
  if (debug_viz_) {
    vizObjectBoxes(viz_object_boxes_);
    vizEgoBoxes(xtraj_, model_name_);
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
    logCompletedCycle();
    return;
  }

  latest_valid_trajectory_ = *trajectory;
  trajectory_pub_->publish(std::move(trajectory));
  metrics.published = true;
  logCompletedCycle();
  const char* cycle_time_color = metrics.cycle_ms <= 100.0 ? "\x1b[32m" : "\x1b[31m";
  if (metrics.cycle_ms > 100.0) {
    if (metrics.relaxed_attempted) {
      RCLCPP_INFO(this->get_logger(),
                  "Published trajectory (cycle: %s%.2f ms\x1b[0m; relaxed solve: %.2f ms; constrained solve: %.2f ms)",
                  cycle_time_color, metrics.cycle_ms, metrics.relaxed_solve_wall_ms, metrics.constrained_solve_wall_ms);
    } else {
      RCLCPP_INFO(this->get_logger(), "Published trajectory (cycle: %s%.2f ms\x1b[0m; constrained solve: %.2f ms)",
                  cycle_time_color, metrics.cycle_ms, metrics.constrained_solve_wall_ms);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Published trajectory (cycle: %s%.2f ms\x1b[0m)", cycle_time_color, metrics.cycle_ms);
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
    keepNClosestObjects(tf_object_list, static_cast<int>(p_objects_shape_[0]));
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
  const auto expected_cost_weights_size = static_cast<size_t>(p_cost_weights_shape_[0] * p_cost_weights_shape_[1]);
  if (cost_weights.size() != expected_cost_weights_size) {
    RCLCPP_ERROR(this->get_logger(), "Size of cost_weights (%zu) does not match expected size (%zu).", cost_weights.size(),
                 expected_cost_weights_size);
    throw std::runtime_error("Size of cost_weights does not match expected size.");
  }
  global_params.insert(global_params.end(), cost_weights.begin(), cost_weights.end());

  // other cost params
  global_params.push_back(thw_);
  global_params.push_back(d_min_object_long_);
  global_params.push_back(d_min_object_lat_);
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
                                                  const perception_msgs::msg::ObjectList& object_list) {
  const auto start_time = std::chrono::steady_clock::now();

  // prepare object predictions
  struct PredictionData {
    std::vector<double> time;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
  };

  std::vector<std::vector<PredictionData>> object_predictions(object_list.objects.size());
  const double object_stamp = static_cast<double>(rclcpp::Time(object_list.header.stamp).nanoseconds()) / 1e9;
  if (consider_objects_ == CONSIDER_OBJECTS::PREDICTED_OBJECTS) {
    for (size_t object_index = 0; object_index < object_list.objects.size(); ++object_index) {
      const auto& object = object_list.objects[object_index];
      auto append_prediction = [&](const auto& prediction) {
        PredictionData prediction_data{{object_stamp},
                                       {perception_msgs::object_access::getX(object)},
                                       {perception_msgs::object_access::getY(object)},
                                       {perception_msgs::object_access::getYaw(object)}};
        for (const auto& state : prediction.states) {
          prediction_data.time.push_back(static_cast<double>(rclcpp::Time(state.header.stamp).nanoseconds()) / 1e9);
          prediction_data.x.push_back(perception_msgs::object_access::getX(state));
          prediction_data.y.push_back(perception_msgs::object_access::getY(state));
          prediction_data.yaw.push_back(perception_msgs::object_access::getYaw(state));
        }
        object_predictions[object_index].push_back(std::move(prediction_data));
      };

      // keep all predictions above the configured probability threshold
      for (const auto& prediction : object.state_predictions) {
        if (prediction.probability > min_prediction_probability_) append_prediction(prediction);
      }

      // fall back to the first prediction if none satisfies the threshold
      if (object_predictions[object_index].empty() && !object.state_predictions.empty()) {
        append_prediction(object.state_predictions[0]);
      }
    }
  }

  object_constraints_active_ = consider_objects_ != CONSIDER_OBJECTS::NO_OBJECTS && !object_list.objects.empty();
  constexpr double MIN_HALF_EXTENT = 0.05;

  // loop over shooting intervals
  const double dt = optimization_horizon_ / n_shots_;
  double floating_dynamic_weight = 1.0;
  for (int stage = 0; stage <= n_shots_; ++stage) {
    // dynamic weight
    int idx = 0;
    int parameter_count = static_cast<int>(p_dynamic_weight_shape_[0] * p_dynamic_weight_shape_[1]);
    std::vector<int> dynamic_weight_indices{idx};
    int status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, stage, dynamic_weight_indices.data(),
                                                                      &floating_dynamic_weight, parameter_count);
    if (status != ACADOS_SUCCESS) {
      throw std::runtime_error("acados dynamic-weight update failed with status " + std::to_string(status));
    }
    floating_dynamic_weight *= dynamic_weight_;

    // constraint activation
    idx += parameter_count;
    parameter_count = static_cast<int>(p_constraint_activation_shape_[0] * p_constraint_activation_shape_[1]);
    std::array<int, 2> constraint_activation_indices{idx, idx + 1};
    std::array<double, 2> constraint_activation{object_constraints_active_ ? 1.0 : 0.0,
                                                consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS ? 1.0 : 0.0};
    status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, stage, constraint_activation_indices.data(),
                                                                  constraint_activation.data(), parameter_count);
    if (status != ACADOS_SUCCESS) {
      throw std::runtime_error("acados constraint-activation update failed with status " + std::to_string(status));
    }

    // objects
    idx += parameter_count;
    parameter_count = static_cast<int>(p_objects_shape_[0] * p_objects_shape_[1]);
    std::vector<double> object_boxes;
    object_boxes.reserve(parameter_count);
    for (size_t object_index = 0; object_index < object_list.objects.size(); ++object_index) {
      const auto& object = object_list.objects[object_index];
      std::vector<std::array<double, 3>> target_states;
      if (!object_predictions[object_index].empty()) {
        for (const auto& prediction : object_predictions[object_index]) {
          const double desired_time = static_cast<double>(rclcpp::Time(ego_data.header.stamp).nanoseconds()) / 1e9 + dt * stage;
          std::array<double, 3> state{};
          linearInterpolation(prediction.time, prediction.x, desired_time, state[0]);
          linearInterpolation(prediction.time, prediction.y, desired_time, state[1]);
          linearInterpolation(prediction.time, prediction.yaw, desired_time, state[2], true);
          target_states.push_back(state);
        }
      } else {
        // treat objects without a selected prediction as static
        target_states.push_back({perception_msgs::object_access::getX(object), perception_msgs::object_access::getY(object),
                                 perception_msgs::object_access::getYaw(object)});
      }

      const double offset_long = object.state.reference_point.translation_to_geometric_center.x;
      const double offset_lat = object.state.reference_point.translation_to_geometric_center.y;
      const double half_length = std::max(MIN_HALF_EXTENT, 0.5 * perception_msgs::object_access::getLength(object));
      const double half_width = std::max(MIN_HALF_EXTENT, 0.5 * perception_msgs::object_access::getWidth(object));
      for (const auto& state : target_states) {
        // ensure that the object position represents its geometric center
        const double cos_yaw = std::cos(state[2]);
        const double sin_yaw = std::sin(state[2]);
        const double center_x = state[0] + offset_long * cos_yaw - offset_lat * sin_yaw;
        const double center_y = state[1] + offset_long * sin_yaw + offset_lat * cos_yaw;
        object_boxes.insert(object_boxes.end(), {center_x, center_y, state[2], half_length, half_width, 1.0});
        if (object_boxes.size() >= static_cast<size_t>(parameter_count)) {
          object_boxes.resize(parameter_count);
          break;
        }
      }
      if (object_boxes.size() >= static_cast<size_t>(parameter_count)) break;
    }
    // fill remaining object slots with inactive boxes
    while (object_boxes.size() < static_cast<size_t>(parameter_count)) {
      object_boxes.insert(object_boxes.end(), {0.0, 0.0, 0.0, MIN_HALF_EXTENT, MIN_HALF_EXTENT, 0.0});
    }
    if (debug_viz_) viz_object_boxes_.insert(viz_object_boxes_.end(), object_boxes.begin(), object_boxes.end());

    std::vector<int> object_indices(parameter_count);
    std::iota(object_indices.begin(), object_indices.end(), idx);
    status = trajectory_optimization::acados_update_params_sparse(ocp_capsule_, stage, object_indices.data(), object_boxes.data(),
                                                                  parameter_count);
    if (status != ACADOS_SUCCESS) {
      throw std::runtime_error("acados object-box update failed with status " + std::to_string(status));
    }
  }
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time).count();
  RCLCPP_DEBUG(get_logger(), "setOcpParameters object-box duration: %.3f ms", elapsed_ms);
}

}  // namespace trajectory_optimization

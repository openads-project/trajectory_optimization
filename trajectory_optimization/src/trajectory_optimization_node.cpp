#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimization::TrajectoryOptimizationNode)

/**
 * @brief Namespace for trajectory_optimization package
 *
 */
namespace trajectory_optimization {

// parameter names

// constants
const std::string TrajectoryOptimizationNode::kDriveableSpaceTopic = "~/driveable_space";
const std::string TrajectoryOptimizationNode::kEgoDataTopic = "~/ego_data";
const std::string TrajectoryOptimizationNode::kObjectListTopic = "~/object_list";
const std::string TrajectoryOptimizationNode::kReferenceTrajectoryTopic = "~/reference_trajectory";
const std::string TrajectoryOptimizationNode::kRouteTopic = "~/route";

const std::string TrajectoryOptimizationNode::kTrajectoryTopic = "~/trajectory";

const std::string TrajectoryOptimizationNode::kOptimizationFreqParam = "optimization_frequency";
const std::string TrajectoryOptimizationNode::kNStatesParam = "n_shots";
const std::string TrajectoryOptimizationNode::kOptimizationHoizonParam = "optimization_horizon";
const std::string TrajectoryOptimizationNode::kVerboseParam = "verbose";
const std::string TrajectoryOptimizationNode::kCostWeightsParam = "cost_weights";
const std::string TrajectoryOptimizationNode::kInitAsRefParam = "init_as_ref";

const std::string TrajectoryOptimizationNode::kPCostWeightsShapeParam = "p_cost_weights_shape";
const std::string TrajectoryOptimizationNode::kPRefPathShapeParam = "p_ref_path_shape";

/**
 * @brief Creates a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::TrajectoryOptimizationNode(const rclcpp::NodeOptions& options)
    : Node("trajectory_optimization_node", options) {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

/**
 * @brief Destroys a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::~TrajectoryOptimizationNode() {
  freeSolver();
}

/**
 * @brief Declares all parameters that this node uses
 */
void TrajectoryOptimizationNode::declareParameters() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "Optimization Frequency in Hz";
  this->declare_parameter(kOptimizationFreqParam, optimization_freq_, param_desc);

  param_desc.description = "Number of states in the optimization problem";
  this->declare_parameter(kNStatesParam, n_states_, param_desc);

  param_desc.description = "Optimization Horizon in seconds";
  this->declare_parameter(kOptimizationHoizonParam, optimization_horizon_, param_desc);

  param_desc.description = "Print solver statistics";
  this->declare_parameter(kVerboseParam, verbose_, param_desc);

  param_desc.description = "Cost function weights";
  this->declare_parameter(kCostWeightsParam, cost_weights_, param_desc);

  param_desc.description = "Initialize as reference trajectory";
  this->declare_parameter(kInitAsRefParam, init_as_ref_, param_desc);

  param_desc.description = "OCP parameter vector shape for cost weights";
  this->declare_parameter(kPCostWeightsShapeParam, p_cost_weights_shape_, param_desc);

  param_desc.description = "OCP parameter vector shape for reference path";
  this->declare_parameter(kPRefPathShapeParam, p_ref_path_shape_, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void TrajectoryOptimizationNode::loadParameters() {
  try {
    optimization_freq_ = this->get_parameter(kOptimizationFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kOptimizationFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    n_states_ = this->get_parameter(kNStatesParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kNStatesParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    optimization_horizon_ = this->get_parameter(kOptimizationHoizonParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kOptimizationHoizonParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    verbose_ = this->get_parameter(kVerboseParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%i'", kVerboseParam.c_str(), verbose_);
  }
  try {
    cost_weights_ = this->get_parameter(kCostWeightsParam).as_double_array();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kCostWeightsParam.c_str());
  }
  try {
    init_as_ref_ = this->get_parameter(kInitAsRefParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kInitAsRefParam.c_str());
  }
  try {
    p_cost_weights_shape_ = this->get_parameter(kPCostWeightsShapeParam).as_integer_array();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kPCostWeightsShapeParam.c_str());
  }
  try {
    p_ref_path_shape_ = this->get_parameter(kPRefPathShapeParam).as_integer_array();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kPRefPathShapeParam.c_str());
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  // update timer with newly configured period parameter value
  for (const auto& param : parameters) {
    if (param.get_name() == kVerboseParam) {
      verbose_ = param.as_bool();
    } else if (param.get_name() == kCostWeightsParam) {
      cost_weights_ = param.as_double_array();
    } else if (param.get_name() == kInitAsRefParam) {
      init_as_ref_ = param.as_bool();
    } else if (param.get_name() == kPCostWeightsShapeParam) {
      p_cost_weights_shape_ = param.as_integer_array();
    } else if (param.get_name() == kPRefPathShapeParam) {
      p_ref_path_shape_ = param.as_integer_array();
    }
  }

  // mark parameter change successful
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

/**
 * @brief Sets up subscribers, publishers, and more.
 *
 */
void TrajectoryOptimizationNode::setup() {
  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&TrajectoryOptimizationNode::parametersCallback, this, std::placeholders::_1));

  // set up subscriber for input topics
  ego_data_sub_ = this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10, std::bind(&TrajectoryOptimizationNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_sub_->get_topic_name());

  driveable_space_sub_ = this->create_subscription<route_planning_msgs::msg::DriveableSpace>(
      kDriveableSpaceTopic, 10,
      std::bind(&TrajectoryOptimizationNode::driveableSpaceCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", driveable_space_sub_->get_topic_name());

  object_list_sub_ = this->create_subscription<perception_msgs::msg::ObjectList>(
      kObjectListTopic, 10, std::bind(&TrajectoryOptimizationNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", object_list_sub_->get_topic_name());

  route_sub_ = this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10, std::bind(&TrajectoryOptimizationNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", route_sub_->get_topic_name());

  reference_trajectory_sub_ = this->create_subscription<trajectory_planning_msgs::msg::Trajectory>(
      kReferenceTrajectoryTopic, 10,
      std::bind(&TrajectoryOptimizationNode::referenceTrajectoryCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", reference_trajectory_sub_->get_topic_name());

  // set up publisher for output topic
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());

  // create timer for planning cycle
  planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / optimization_freq_),
                                            std::bind(&TrajectoryOptimizationNode::planningCycle, this));

  // init reference trajectory
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      reference_trajectory_, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_states_ + 1);

  // setupSolver();
}

void TrajectoryOptimizationNode::setupSolver(const perception_msgs::msg::EgoData& ego_data) {
  // setup acados solver
  acados_ocp_capsule_ = trajectory_planning_acados_create_capsule();

  // allocate the array and fill it accordingly
  double* new_time_steps = NULL;
  if (n_states_ != TRAJECTORY_PLANNING_N) {
    new_time_steps = new double(optimization_horizon_ / n_states_);
    RCLCPP_INFO(this->get_logger(), "new_time_steps = %f", *new_time_steps);
  }
  int status = trajectory_planning_acados_create_with_discretization(acados_ocp_capsule_, n_states_, new_time_steps);
  delete[] new_time_steps;

  if (status) {
    RCLCPP_INFO(this->get_logger(), "trajectory_planning_acados_create() returned status %d. Exiting.", status);
    exit(1);
  }

  nlp_config_ = trajectory_planning_acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = trajectory_planning_acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = trajectory_planning_acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = trajectory_planning_acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = trajectory_planning_acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = trajectory_planning_acados_get_nlp_opts(acados_ocp_capsule_);

  // initialization for state values
  double x_init[TRAJECTORY_PLANNING_NX];
  for (int i = 0; i < TRAJECTORY_PLANNING_NX; ++i) {
    x_init[i] = 0.0;
  }

  // initial value for control input
  double u0[TRAJECTORY_PLANNING_NU];
  for (int i = 0; i < TRAJECTORY_PLANNING_NU; ++i) {
    u0[i] = 0.0;
  }

  // initialize solution
  int rti_phase = 0;
  for (int i = 0; i < n_states_; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, n_states_, "x", x_init);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);

  // if ego data revceived set initial state
  if (received_ego_data_) {
    x_init[3] = perception_msgs::object_access::getVelLon(ego_data);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x_init);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x_init);
  } else {
    RCLCPP_WARN(this->get_logger(), "Ego data not received. Using default initial state.");
  }

  xtraj_ = new double[TRAJECTORY_PLANNING_NX * (n_states_ + 1)];
  utraj_ = new double[TRAJECTORY_PLANNING_NU * n_states_];
}

void TrajectoryOptimizationNode::freeSolver() {
  // deallocate memory
  delete[] xtraj_;
  delete[] utraj_;

  int status;
  // free solver
  status = trajectory_planning_acados_free(acados_ocp_capsule_);
  if (status) {
    printf("trajectory_planning_acados_free() returned status %d. \n", status);
  }
  // free solver capsule
  status = trajectory_planning_acados_free_capsule(acados_ocp_capsule_);
  if (status) {
    printf("trajectory_planning_acados_free_capsule() returned status %d. \n", status);
  }
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received ego data");
  received_ego_data_ = true;
  ego_data_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input drivable space
 */
void TrajectoryOptimizationNode::driveableSpaceCallback(
    const route_planning_msgs::msg::DriveableSpace::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received driveable space");
  driveable_space_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received object list");
  object_list_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input reference trajectory
 */
void TrajectoryOptimizationNode::referenceTrajectoryCallback(
    const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received reference trajectory");
  reference_trajectory_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received route");
  route_ = *msg;
}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::planningCycle() {
  // check if the reference trajectory is standstill
  if (trajectory_planning_msgs::trajectory_access::getStandstill(reference_trajectory_)) {
    RCLCPP_WARN(this->get_logger(), "Standstill trajectory. Skipping planning cycle.");
    return;
  }
  setupSolver(ego_data_);

  // update inputs to the ocp
  updateOcpInputs(ego_data_, object_list_, driveable_space_, route_, reference_trajectory_);

  // init trajectory message and set header
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      *trajectory, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_states_ + 1);
  trajectory->header.stamp = this->now();
  trajectory->header.frame_id = reference_trajectory_.header.frame_id;

  int status = trajectory_planning_acados_solve(acados_ocp_capsule_);

  // get solution
  for (int ii = 0; ii <= nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * TRAJECTORY_PLANNING_NX]);
  for (int ii = 0; ii < nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * TRAJECTORY_PLANNING_NU]);

  if (verbose_) printSolution(status);
  if (status == 3) {
    RCLCPP_WARN(this->get_logger(), "Solver failed. Skipping trajectory publication.");
    return;
  }

  // convert output into trajectory message
  double dt = optimization_horizon_ / n_states_;
  for (int i = 0; i <= n_states_; ++i) {
    trajectory_planning_msgs::trajectory_access::setT(*trajectory, i * dt, i);
    trajectory_planning_msgs::trajectory_access::setX(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 0], i);
    trajectory_planning_msgs::trajectory_access::setY(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 1], i);
    trajectory_planning_msgs::trajectory_access::setS(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 2], i);
    trajectory_planning_msgs::trajectory_access::setV(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 3], i);
    trajectory_planning_msgs::trajectory_access::setA(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 4], i);
    trajectory_planning_msgs::trajectory_access::setTheta(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 5], i);
    // TODO: Kappa and dKappa
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, false);  // TODO: check if standstill

  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_INFO(this->get_logger(), "Published trajectory");

  freeSolver();
}

/**
 * @brief This function updates the inputs to the ocp
 *
 */
void TrajectoryOptimizationNode::updateOcpInputs(
    const perception_msgs::msg::EgoData& ego_data, const perception_msgs::msg::ObjectList& object_list,
    const route_planning_msgs::msg::DriveableSpace& driveable_space, const route_planning_msgs::msg::Route& route,
    const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  if (init_as_ref_) {
    // set initial guess
    double x_init[TRAJECTORY_PLANNING_NX];
    for (int i = 0; i < TRAJECTORY_PLANNING_NX; ++i) {
      x_init[i] = 0.0;
    }
    for (int i = 0; i <= n_states_; ++i) {
      x_init[0] = trajectory_planning_msgs::trajectory_access::getX(reference_trajectory, i);
      x_init[1] = trajectory_planning_msgs::trajectory_access::getY(reference_trajectory, i);
      x_init[3] = trajectory_planning_msgs::trajectory_access::getV(reference_trajectory, i);
      if (i == 0) {
        // TODO: get from ego_data (high-level stabilization) or last valid trajectory (low-level stabilization)
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x_init);
        ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x_init);
      }
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init);
    }
  }

  // update ocp parameters
  this->setOcpParameters(cost_weights_, reference_trajectory);
}

void TrajectoryOptimizationNode::setOcpParameters(
    std::vector<double>& cost_weights, const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  // loop over shooting intervals
  for (int i = 0; i <= n_states_; ++i) {
    int idx, n;

    // cost weights
    idx = 0;
    n = p_cost_weights_shape_[0];
    std::vector<int> idx_cost_weights(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_cost_weights.begin(), idx_cost_weights.end(), idx);
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_cost_weights.data(),
                                                    cost_weights.data(), n);

    // ref path
    idx += n;
    n = p_ref_path_shape_[0] * p_ref_path_shape_[1];
    std::vector<int> idx_ref_path(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_ref_path.begin(), idx_ref_path.end(), idx);
    // fill ref_path vector with values from reference_trajectory
    std::vector<double> ref_path(n, std::numeric_limits<double>::infinity());
    if (reference_trajectory.states.size() >= n) {
      std::copy(reference_trajectory.states.begin(), reference_trajectory.states.begin() + n, ref_path.begin());
    } else {
      // TODO: what to do here? Currently just copy the whole reference trajectory and rest is filled with infinity
    std::copy(reference_trajectory.states.begin(), reference_trajectory.states.end(), ref_path.begin());
    }
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_ref_path.data(), ref_path.data(), n);
  }
}

/**
 * @brief This function prints the solution of the ocp
 *
 */
void TrajectoryOptimizationNode::printSolution(int status) {
  // get statistics
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;
  ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);

  printf("\n--- xtraj ---\n");
  d_print_exp_tran_mat(TRAJECTORY_PLANNING_NX, n_states_ + 1, xtraj_, TRAJECTORY_PLANNING_NX);
  printf("\n--- utraj ---\n");
  d_print_exp_tran_mat(TRAJECTORY_PLANNING_NU, n_states_, utraj_, TRAJECTORY_PLANNING_NU);
  // ocp_nlp_out_print(nlp_solver_->dims, nlp_out_);

  printf("\nsolved ocp %d times, solution printed above\n\n", 1);

  if (status == ACADOS_SUCCESS) {
    printf("trajectory_planning_acados_solve(): SUCCESS!\n");
  } else {
    printf("trajectory_planning_acados_solve() failed with status %d.\n", status);
  }

  trajectory_planning_acados_print_stats(acados_ocp_capsule_);

  printf("\nSolver info:\n");
  printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, 1, elapsed_time * 1000,
         kkt_norm_inf);
}

}  // namespace trajectory_optimization
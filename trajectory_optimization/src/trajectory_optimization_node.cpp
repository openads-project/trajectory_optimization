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

const std::string TrajectoryOptimizationNode::kVehicleFrameIdParam = "vehicle_frame_id";
const std::string TrajectoryOptimizationNode::kTrajectoryFrameIdParam = "trajectory_frame_id";
const std::string TrajectoryOptimizationNode::kFixedOverTimeFrameIdParam = "fixed_over_time_frame_id";
const std::string TrajectoryOptimizationNode::kOptimizationFreqParam = "optimization_frequency";
const std::string TrajectoryOptimizationNode::kNShotsParam = "n_shots";
const std::string TrajectoryOptimizationNode::kOptimizationHoizonParam = "optimization_horizon";
const std::string TrajectoryOptimizationNode::kVerboseParam = "verbose";
const std::string TrajectoryOptimizationNode::kWheelBaseParam = "wheelbase";
const std::string TrajectoryOptimizationNode::kCostWeightsParam = "cost_weights";
const std::string TrajectoryOptimizationNode::kDynamicWeightParam = "dynamic_weight";
const std::string TrajectoryOptimizationNode::kStandstillTresholdParam = "standstill_threshold";
const std::string TrajectoryOptimizationNode::kHighLevelStabilizationParam = "high_level_stabilization";

const std::string TrajectoryOptimizationNode::kPCostWeightsShapeParam = "p_cost_weights_shape";
const std::string TrajectoryOptimizationNode::kPRefPathShapeParam = "p_ref_path_shape";
const std::string TrajectoryOptimizationNode::kPObstaclesShapeParam = "p_obstacles_shape";

const std::string TrajectoryOptimizationNode::kBiLevelThresholdVParam = "bi_level_dV";
const std::string TrajectoryOptimizationNode::kBiLevelThresholdAParam = "bi_level_dA";
const std::string TrajectoryOptimizationNode::kBiLevelThresholdYParam = "bi_level_dY";
const std::string TrajectoryOptimizationNode::kBiLevelThresholdYawParam = "bi_level_dYaw";
const std::string TrajectoryOptimizationNode::kBiLevelThresholdDeltaParam = "bi_level_dDelta";

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
TrajectoryOptimizationNode::~TrajectoryOptimizationNode() { freeSolver(); }

/**
 * @brief Declares all parameters that this node uses
 */
void TrajectoryOptimizationNode::declareParameters() {
  rcl_interfaces::msg::ParameterDescriptor param_desc;

  param_desc.description = "Frame ID of local vehicle frame (the ocp is defined in this frame)";
  this->declare_parameter(kVehicleFrameIdParam, vehicle_frame_id_, param_desc);

  param_desc.description = "Frame ID of output trajectory";
  this->declare_parameter(kTrajectoryFrameIdParam, trajectory_frame_id_, param_desc);

  param_desc.description = "Frame ID of frame that is fixed over time for finding temporal transforms";
  this->declare_parameter(kFixedOverTimeFrameIdParam, fixed_over_time_frame_id_, param_desc);

  param_desc.description = "Optimization Frequency in Hz";
  this->declare_parameter(kOptimizationFreqParam, optimization_freq_, param_desc);

  param_desc.description = "Number of shooting intervals in optimization horizon";
  this->declare_parameter(kNShotsParam, n_shots_, param_desc);

  param_desc.description = "Optimization Horizon in seconds";
  this->declare_parameter(kOptimizationHoizonParam, optimization_horizon_, param_desc);

  param_desc.description = "Print solver statistics";
  this->declare_parameter(kVerboseParam, verbose_, param_desc);

  param_desc.description = "Wheelbase of the vehicle [m]";
  this->declare_parameter(kWheelBaseParam, wheelbase_, param_desc);

  param_desc.description = "Cost function weights";
  this->declare_parameter(kCostWeightsParam, cost_weights_, param_desc);

  param_desc.description = "Dynamic weight alpha";
  this->declare_parameter(kDynamicWeightParam, dynamic_weight_, param_desc);

  param_desc.description = "Threshold for standstill detection [m/s]. If all state velocities are below this threshold, publish standstill trajectory";
  this->declare_parameter(kStandstillTresholdParam, standstill_threshold_, param_desc);

  param_desc.description = "Use high-level stabilization strategy for init state (= init with current EgoData)";
  this->declare_parameter(kHighLevelStabilizationParam, high_level_stabilization_, param_desc);

  param_desc.description = "OCP parameter vector shape for cost weights";
  this->declare_parameter(kPCostWeightsShapeParam, p_cost_weights_shape_, param_desc);

  param_desc.description = "OCP parameter vector shape for reference path";
  this->declare_parameter(kPRefPathShapeParam, p_ref_path_shape_, param_desc);

  param_desc.description = "OCP parameter vector shape for obstacles";
  this->declare_parameter(kPObstaclesShapeParam, p_obstacles_shape_, param_desc);

  param_desc.description = "Threshold for bi-level stabilization: maximum velocity difference [m/s]";
  this->declare_parameter(kBiLevelThresholdVParam, bi_level_dV_, param_desc);

  param_desc.description = "Threshold for bi-level stabilization: maximum acceleration difference [m/s^2]";
  this->declare_parameter(kBiLevelThresholdAParam, bi_level_dA_, param_desc);

  param_desc.description = "Threshold for bi-level stabilization: maximum y-offset [m]";
  this->declare_parameter(kBiLevelThresholdYParam, bi_level_dY_, param_desc);

  param_desc.description = "Threshold for bi-level stabilization: maximum yaw difference [degree]";
  this->declare_parameter(kBiLevelThresholdYawParam, bi_level_dYaw_, param_desc);

  param_desc.description = "Threshold for bi-level stabilization: maximum steering angle difference [degree]";
  this->declare_parameter(kBiLevelThresholdDeltaParam, bi_level_dDelta_, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void TrajectoryOptimizationNode::loadParameters() {
  try {
    vehicle_frame_id_ = this->get_parameter(kVehicleFrameIdParam).as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%s'", kVerboseParam.c_str(),
                vehicle_frame_id_.c_str());
  }
  try {
    trajectory_frame_id_ = this->get_parameter(kTrajectoryFrameIdParam).as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%s'", kTrajectoryFrameIdParam.c_str(),
                trajectory_frame_id_.c_str());
  }
  try {
    fixed_over_time_frame_id_ = this->get_parameter(kFixedOverTimeFrameIdParam).as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting to '%s'", kFixedOverTimeFrameIdParam.c_str(),
                fixed_over_time_frame_id_.c_str());
  }
  try {
    optimization_freq_ = this->get_parameter(kOptimizationFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kOptimizationFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    n_shots_ = this->get_parameter(kNShotsParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kNShotsParam.c_str());
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
    wheelbase_ = this->get_parameter(kWheelBaseParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kWheelBaseParam.c_str());
  }
  try {
    cost_weights_ = this->get_parameter(kCostWeightsParam).as_double_array();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kCostWeightsParam.c_str());
  }
  try {
    dynamic_weight_ = this->get_parameter(kDynamicWeightParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kDynamicWeightParam.c_str());
  }
  try {
    standstill_threshold_ = this->get_parameter(kStandstillTresholdParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kStandstillTresholdParam.c_str());
  }
  try {
    high_level_stabilization_ = this->get_parameter(kHighLevelStabilizationParam).as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kHighLevelStabilizationParam.c_str());
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
  try {
    p_obstacles_shape_ = this->get_parameter(kPObstaclesShapeParam).as_integer_array();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kPObstaclesShapeParam.c_str());
  }
  try {
    bi_level_dV_ = this->get_parameter(kBiLevelThresholdVParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kBiLevelThresholdVParam.c_str());
  }
  try {
    bi_level_dA_ = this->get_parameter(kBiLevelThresholdAParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kBiLevelThresholdAParam.c_str());
  }
  try {
    bi_level_dY_ = this->get_parameter(kBiLevelThresholdYParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kBiLevelThresholdYParam.c_str());
  }
  try {
    bi_level_dYaw_ = this->get_parameter(kBiLevelThresholdYawParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kBiLevelThresholdYawParam.c_str());
  }
  try {
    bi_level_dDelta_ = this->get_parameter(kBiLevelThresholdDeltaParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not set, defaulting", kBiLevelThresholdDeltaParam.c_str());
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
  for (const auto& param : parameters) {
    if (param.get_name() == kOptimizationFreqParam) {
      optimization_freq_ = param.as_double();
    } else if (param.get_name() == kNShotsParam) {
      n_shots_ = param.as_int();
    } else if (param.get_name() == kOptimizationHoizonParam) {
      optimization_horizon_ = param.as_double();
    } else if (param.get_name() == kVerboseParam) {
      verbose_ = param.as_bool();
    } else if (param.get_name() == kWheelBaseParam) {
      wheelbase_ = param.as_double();
    } else if (param.get_name() == kCostWeightsParam) {
      cost_weights_ = param.as_double_array();
    } else if (param.get_name() == kDynamicWeightParam) {
      dynamic_weight_ = param.as_double();
    } else if (param.get_name() == kStandstillTresholdParam) {
      standstill_threshold_ = param.as_double();
    } else if (param.get_name() == kHighLevelStabilizationParam) {
      high_level_stabilization_ = param.as_bool();
    } else if (param.get_name() == kPCostWeightsShapeParam) {
      p_cost_weights_shape_ = param.as_integer_array();
    } else if (param.get_name() == kPRefPathShapeParam) {
      p_ref_path_shape_ = param.as_integer_array();
    } else if (param.get_name() == kPObstaclesShapeParam) {
      p_obstacles_shape_ = param.as_integer_array();
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
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

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
      reference_trajectory_, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_shots_ + 1);
  reference_trajectory_.header.frame_id = vehicle_frame_id_;

  // init latest trajectory
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      latest_valid_trajectory_, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_shots_ + 1);
  latest_valid_trajectory_.header.frame_id = trajectory_frame_id_;

  setupSolver();
}

void TrajectoryOptimizationNode::setupSolver() {
  // setup acados solver
  acados_ocp_capsule_ = trajectory_planning_acados_create_capsule();

  // allocate the array and fill it accordingly
  double* new_time_steps = NULL;
  if (n_shots_ != TRAJECTORY_PLANNING_N) {
    new_time_steps = new double(optimization_horizon_ / n_shots_);
    RCLCPP_INFO(this->get_logger(), "new_time_steps = %f", *new_time_steps);
  }
  int status = trajectory_planning_acados_create_with_discretization(acados_ocp_capsule_, n_shots_, new_time_steps);
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

  // initialization of state and control values; set all to zero
  double x_init[TRAJECTORY_PLANNING_NX] = {0.0};
  double u0[TRAJECTORY_PLANNING_NU] = {0.0};

  // initialize solution
  int rti_phase = 0;
  for (int i = 0; i < n_shots_; ++i) {
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_init);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u0);
  }
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, n_shots_, "x", x_init);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);

  xtraj_ = new double[TRAJECTORY_PLANNING_NX * (n_shots_ + 1)];
  utraj_ = new double[TRAJECTORY_PLANNING_NU * n_shots_];
}

/**
 * @brief Calculates and returns the initial state vector for the ocp using bi-level stabilization.
 *
 * This function uses bi-level stabilization for initializing the optimization problem.
 * In general the initial state is interpolated from the latest trajectory (-> low-level stabilization).
 * But if the difference between the interpolated state and the ego state is too large, the ego state is used instead (-> high-level stabilization).
 * This combination of low- and high-level stabilization is called bi-level stabilization.
 *
 * @param ego_data EgoData message.
 * @return Initial state for the optimization problem.
 */
std::vector<double> TrajectoryOptimizationNode::getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) {
  // transform latest trajectory to current base_link frame
  trajectory_planning_msgs::msg::Trajectory tf_trajectory;
  try {
    tf_trajectory =
        tf2_buffer_->transform(latest_valid_trajectory_, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation is not available. Init high-level instead. Ex: %s", ex.what());
    return getHighLevelX0(ego_data);
  }

  // fill vectors with state values from the transformed trajectory
  std::vector<double> TIME, V, Y, A, THETA, DELTA;
  for (int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(tf_trajectory); i++) {
    TIME.push_back(trajectory_planning_msgs::trajectory_access::getT(tf_trajectory, i));
    Y.push_back(trajectory_planning_msgs::trajectory_access::getY(tf_trajectory, i));
    V.push_back(trajectory_planning_msgs::trajectory_access::getV(tf_trajectory, i));
    A.push_back(trajectory_planning_msgs::trajectory_access::getA(tf_trajectory, i));
    THETA.push_back(trajectory_planning_msgs::trajectory_access::getTheta(tf_trajectory, i));
    double delta = atan(wheelbase_ * trajectory_planning_msgs::trajectory_access::getKappa(
                                         tf_trajectory, i));  // export to trajectory_access?
    DELTA.push_back(delta);
  }

  // interpolate target states by time from the extracted vectors; if not successful, set to ego state (high-level initialization)
  double v_tgt, y_tgt, a_tgt, theta_tgt, delta_tgt;
  double des_time =
      (rclcpp::Time(ego_data.header.stamp) - rclcpp::Time(latest_valid_trajectory_.header.stamp)).seconds();
  RCLCPP_INFO(this->get_logger(), "Desired time: %f", des_time);
  if (!linearInterpolation(TIME, Y, des_time, y_tgt)) y_tgt = 0.0;
  if (!linearInterpolation(TIME, V, des_time, v_tgt)) v_tgt = perception_msgs::object_access::getVelLon(ego_data);
  if (!linearInterpolation(TIME, A, des_time, a_tgt)) a_tgt = perception_msgs::object_access::getAccLon(ego_data);
  if (!linearInterpolation(TIME, THETA, des_time, theta_tgt)) theta_tgt = 0.0;
  if (!linearInterpolation(TIME, DELTA, des_time, delta_tgt))
    delta_tgt = perception_msgs::object_access::getSteeringAngleAck(ego_data);

  RCLCPP_DEBUG(this->get_logger(), "y_tgt: %f, v_tgt: %f, a_tgt: %f, theta_tgt: %f, delta_tgt: %f", y_tgt, v_tgt, a_tgt,
               theta_tgt, delta_tgt);

  // handle thresholds for bi-level stabilization (which means, using ego state as initial state for the optimization)
  // longitudinal reinits
  if (fabs(v_tgt - perception_msgs::object_access::getVelLon(ego_data)) > bi_level_dV_ ||
      fabs(a_tgt - perception_msgs::object_access::getAccLon(ego_data)) > bi_level_dA_) {
    v_tgt = perception_msgs::object_access::getVelLon(ego_data);
    a_tgt = perception_msgs::object_access::getAccLon(ego_data);
  }
  // lateral reinits
  if (fabs(y_tgt) > bi_level_dY_ || fabs(theta_tgt) > bi_level_dYaw_ * M_PI / 180.0) {
    y_tgt = 0.0;
    theta_tgt = 0.0;
    delta_tgt = perception_msgs::object_access::getSteeringAngleAck(ego_data);
  } else if (fabs(delta_tgt - perception_msgs::object_access::getSteeringAngleAck(ego_data)) > bi_level_dDelta_) {
    delta_tgt = perception_msgs::object_access::getSteeringAngleAck(ego_data);
  }

  std::vector<double> x_init(TRAJECTORY_PLANNING_NX, 0.0);
  x_init[0] = 0.0;
  x_init[1] = y_tgt;
  x_init[2] = 0.0;
  x_init[3] = v_tgt;
  x_init[4] = a_tgt;
  x_init[5] = theta_tgt;
  x_init[6] = delta_tgt;
  return x_init;
}

/**
 * @brief Returns the initial state vector for the ocp using higl-level stabilization.
 *
 * This function uses high-level stabilization for initializing the optimization problem.
 * -> initial state = current state of the ego vehicle.
 *
 * @param ego_data EgoData message.
 * @return Initial state for the optimization problem.
 */
std::vector<double> TrajectoryOptimizationNode::getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) {
  std::vector<double> x_init(TRAJECTORY_PLANNING_NX, 0.0);
  x_init[3] = perception_msgs::object_access::getVelLon(ego_data);
  // x_init[4] = perception_msgs::object_access::getAccLon(ego_data);
  x_init[4] = 0.0;
  x_init[6] = perception_msgs::object_access::getSteeringAngleAck(ego_data);
  return x_init;
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
 * @brief This function is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::planningCycle() {
  if (!received_ego_data_) {
    RCLCPP_WARN(this->get_logger(), "No EgoData received. Skipping planning cycle.");
    return;
  }
  // init trajectory message and set header
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      *trajectory, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_shots_ + 1);
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
    freeSolver();
    setupSolver();
    return;
  }

  // set initial state
  std::vector<double> x_init(TRAJECTORY_PLANNING_NX, 0.0);
  if (!trajectory_planning_msgs::trajectory_access::getStandstill(latest_valid_trajectory_)) {
    x_init = high_level_stabilization_ ? getHighLevelX0(ego_data_) : getBiLevelX0(ego_data_);
  } else {
    RCLCPP_WARN(this->get_logger(), "No latest trajectory available. Using default initial state. (0)");
  }
  RCLCPP_DEBUG(this->get_logger(), "Initial state: x: %f, y: %f, s: %f v: %f, a: %f, theta: %f, delta: %f ", x_init[0],
               x_init[1], x_init[2], x_init[3], x_init[4], x_init[5], x_init[6]);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", x_init.data());
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", x_init.data());

  // update inputs to the ocp; skip planning cycle if update fails
  if (!updateOcpInputs(ego_data_, object_list_, driveable_space_, route_, reference_trajectory_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to update inputs. Skipping planning cycle.");
    return;
  }

  // solve the optimization problem
  int status = trajectory_planning_acados_solve(acados_ocp_capsule_);

  // get solution
  for (int ii = 0; ii <= nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj_[ii * TRAJECTORY_PLANNING_NX]);
  for (int ii = 0; ii < nlp_dims_->N; ++ii)
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj_[ii * TRAJECTORY_PLANNING_NU]);

  if (verbose_) printSolution(status);
  if (status == 1 || status == 3 || status == 4) {
    RCLCPP_ERROR(this->get_logger(), "Solver failed with status %d. Publishing latest valid trajectory.", status);
    trajectory_pub_->publish(latest_valid_trajectory_);
    // reset control problem
    freeSolver();
    setupSolver();
    return;
  }

  // convert output into trajectory message
  for (int i = 0; i <= n_shots_; ++i) {
    trajectory_planning_msgs::trajectory_access::setX(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 0], i);
    trajectory_planning_msgs::trajectory_access::setY(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 1], i);
    trajectory_planning_msgs::trajectory_access::setS(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 2], i);
    trajectory_planning_msgs::trajectory_access::setV(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 3], i);
    trajectory_planning_msgs::trajectory_access::setA(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 4], i);
    trajectory_planning_msgs::trajectory_access::setTheta(*trajectory, xtraj_[i * TRAJECTORY_PLANNING_NX + 5], i);
    double kappa = tan(xtraj_[i * TRAJECTORY_PLANNING_NX + 6]) / wheelbase_;
    trajectory_planning_msgs::trajectory_access::setKappa(*trajectory, kappa, i);
    // TODO: dKappa
  }

  bool standstill = true;
  for (int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(*trajectory); ++i) {
    if (trajectory_planning_msgs::trajectory_access::getV(*trajectory, i) > standstill_threshold_) standstill = false;
  }
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, standstill);
  if (standstill) {
    freeSolver();
    setupSolver();
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
 * @param driveable_space (currently unused)
 * @param route (currently unused)
 * @param reference_trajectory
 * @return True if the inputs were successfully updated, false otherwise.
 */
bool TrajectoryOptimizationNode::updateOcpInputs(
    const perception_msgs::msg::EgoData& ego_data, const perception_msgs::msg::ObjectList& object_list,
    const route_planning_msgs::msg::DriveableSpace& driveable_space, const route_planning_msgs::msg::Route& route,
    const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  // transform inputs to target base_link frame
  trajectory_planning_msgs::msg::Trajectory tf_reference_trajectory;
  perception_msgs::msg::ObjectList tf_object_list;
  try {
    tf_reference_trajectory =
        tf2_buffer_->transform(reference_trajectory, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                               fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    if (!object_list.objects.empty()) {
      tf_object_list = tf2_buffer_->transform(object_list, vehicle_frame_id_, tf2_ros::fromMsg(ego_data.header.stamp),
                                              fixed_over_time_frame_id_, tf2::durationFromSec(0.01));
    }
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Transformation is not available. Ex: %s", ex.what());
    return false;
  }

  // update ocp parameters
  this->setOcpParameters(cost_weights_, tf_reference_trajectory);

  return true;
}

void TrajectoryOptimizationNode::setOcpParameters(
    std::vector<double>& cost_weights, const trajectory_planning_msgs::msg::Trajectory& reference_trajectory) {
  // loop over shooting intervals
  double floating_dynamic_weight = 1.0;
  for (int i = 0; i <= n_shots_; ++i) {
    int idx, n;

    // cost weights
    idx = 0;
    n = p_cost_weights_shape_[0] * p_cost_weights_shape_[1];
    std::vector<int> idx_cost_weights(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_cost_weights.begin(), idx_cost_weights.end(), idx);
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_cost_weights.data(),
                                                    cost_weights.data(), n);

    // dynamic weight
    idx += n;
    n = 1;
    std::vector<int> idx_dynamic_weight(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_dynamic_weight.begin(), idx_dynamic_weight.end(), idx);
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_dynamic_weight.data(),
                                                    &floating_dynamic_weight, n);
    floating_dynamic_weight *= dynamic_weight_;

    // ref path
    idx += n;
    n = p_ref_path_shape_[0] * p_ref_path_shape_[1];
    std::vector<int> idx_ref_path(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_ref_path.begin(), idx_ref_path.end(), idx);
    
    // replace all t values with theta since we don't need t in the ocp
    trajectory_planning_msgs::msg::Trajectory ref = reference_trajectory;
    for (unsigned int i = 0; i < trajectory_planning_msgs::trajectory_access::getSamplePointSize(ref); ++i) {
      trajectory_planning_msgs::trajectory_access::setT(ref, trajectory_planning_msgs::trajectory_access::getTheta(reference_trajectory, i), i);
    }

    // fill ref_path vector with values from ref
    std::vector<double> ref_path(n, std::numeric_limits<double>::infinity());
    if (ref.states.size() >= n) {
      std::copy(ref.states.begin(), ref.states.begin() + n, ref_path.begin());
    } else {
      // TODO: what to do here? Currently just copy the whole reference trajectory and rest is filled with infinity
      std::copy(ref.states.begin(), ref.states.end(), ref_path.begin());
    }
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_ref_path.data(), ref_path.data(), n);

    // obstacles
    // TODO: get obstacles from object list predictions
    double x_obs = 5.0;
    double y_obs = 0.0;
    double r_obs = 0.5;
    std::vector<double> obstacles = {x_obs, y_obs, r_obs};
    idx += n;
    n = p_obstacles_shape_[0] * p_obstacles_shape_[1];
    std::vector<int> idx_obstacles(n);
    // fill vector with values from idx to idx + n
    std::iota(idx_obstacles.begin(), idx_obstacles.end(), idx);
    trajectory_planning_acados_update_params_sparse(acados_ocp_capsule_, i, idx_obstacles.data(), obstacles.data(), n);
  }
}

}  // namespace trajectory_optimization
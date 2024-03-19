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
const std::string TrajectoryOptimizationNode::kRouteTopic = "~/route";

const std::string TrajectoryOptimizationNode::kTrajectoryTopic = "~/trajectory";

const std::string TrajectoryOptimizationNode::kPlanningFreqParam = "planning_frequency";


/**
 * @brief Creates a TrajectoryOptimizationNode node
 *
 */
TrajectoryOptimizationNode::TrajectoryOptimizationNode(const rclcpp::NodeOptions& options) : Node("trajectory_optimization_node", options) {

  this->declareParameters();
  this->loadParameters();
  this->setup();
}



/**
 * @brief Declares all parameters that this node uses
 */
void TrajectoryOptimizationNode::declareParameters() {

  // set parameter description
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Planning Frequency in Hz";

  // declare parameter
  this->declare_parameter(kPlanningFreqParam, rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void TrajectoryOptimizationNode::loadParameters() {

  // load parameter
  try {
    planning_freq_ = this->get_parameter(kPlanningFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPlanningFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
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
  ego_data_sub_ =
    this->create_subscription<perception_msgs::msg::EgoData>(
      kEgoDataTopic, 10,
      std::bind(&TrajectoryOptimizationNode::egoDataCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", ego_data_sub_->get_topic_name());

  driveable_space_sub_ =
    this->create_subscription<route_planning_msgs::msg::DriveableSpace>(
      kDriveableSpaceTopic, 10,
      std::bind(&TrajectoryOptimizationNode::driveableSpaceCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", driveable_space_sub_->get_topic_name());

  object_list_sub_ =
    this->create_subscription<perception_msgs::msg::ObjectList>(
      kObjectListTopic, 10,
      std::bind(&TrajectoryOptimizationNode::objectListCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", object_list_sub_->get_topic_name());

  route_sub_ =
    this->create_subscription<route_planning_msgs::msg::Route>(
      kRouteTopic, 10,
      std::bind(&TrajectoryOptimizationNode::routeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", route_sub_->get_topic_name());

  // set up publisher for output topic
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(
    kTrajectoryTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", trajectory_pub_->get_topic_name());

  // create timer for planning cycle
  planning_timer_ =
    this->create_wall_timer(std::chrono::duration<double>(planning_freq_),
                            std::bind(&TrajectoryOptimizationNode::planningCycle,
                            this));

  setupSolver();
}

void TrajectoryOptimizationNode::setupSolver() {
  // setup acados solver
  trajectory_planning_solver_capsule *acados_ocp_capsule = trajectory_planning_acados_create_capsule();

  // there is an opportunity to change the number of shooting intervals in C without new code generation
  int N = TRAJECTORY_PLANNING_N; // TODO: param
  // allocate the array and fill it accordingly
  double* new_time_steps = NULL;
  int status = trajectory_planning_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

  if (status) {
    RCLCPP_INFO(this->get_logger(), "trajectory_planning_acados_create() returned status %d. Exiting.", status);
    exit(1);
  }

  ocp_nlp_config *nlp_config = trajectory_planning_acados_get_nlp_config(acados_ocp_capsule);
  ocp_nlp_dims *nlp_dims = trajectory_planning_acados_get_nlp_dims(acados_ocp_capsule);
  ocp_nlp_in *nlp_in = trajectory_planning_acados_get_nlp_in(acados_ocp_capsule);
  ocp_nlp_out *nlp_out = trajectory_planning_acados_get_nlp_out(acados_ocp_capsule);
  ocp_nlp_solver *nlp_solver = trajectory_planning_acados_get_nlp_solver(acados_ocp_capsule);
  void *nlp_opts = trajectory_planning_acados_get_nlp_opts(acados_ocp_capsule);
}


/**
 * @brief This callback is invoked when a parameter value has changed
 *
 * @param[in] parameters                                  input
 *
 * @return    rcl_interfaces::msg::SetParametersResult    output
 */
rcl_interfaces::msg::SetParametersResult TrajectoryOptimizationNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {

  // update timer with newly configured period parameter value
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == kPlanningFreqParam) {
      planning_freq_ = param.as_double();
    }
  }

  // mark parameter change successful
  result.successful = true;
  result.reason = "success";

  return result;
}


/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(
  const perception_msgs::msg::EgoData::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received ego data");
  // TODO: process ego data
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input drivable space
 */
void TrajectoryOptimizationNode::driveableSpaceCallback(
  const route_planning_msgs::msg::DriveableSpace::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received driveable space");
  // TODO: process driveable space
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(
  const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received object list");
  // TODO: process object list
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(
  const route_planning_msgs::msg::Route::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received route");
  // TODO: process route
}


/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void TrajectoryOptimizationNode::planningCycle() {
  
  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory = std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(*trajectory, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, TRAJECTORY_PLANNING_N);  

  trajectory_pub_->publish(std::move(trajectory));
}


}
#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <demo_trajectory_pub/demo_trajectory_pub_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(demo_trajectory_pub::DemoTrajectoryPubNode)

/**
 * @brief Namespace for demo_trajectory_pub package
 *
 */
namespace demo_trajectory_pub {

// constants
const std::string DemoTrajectoryPubNode::kTrajectoryTopic = "~/demo_trajectory";
const std::string DemoTrajectoryPubNode::kEgoDataTopic = "~/ego_data";
const std::string DemoTrajectoryPubNode::kObjectListTopic = "~/demo_object_list";
const std::string DemoTrajectoryPubNode::kNStatesParam = "n_states";
const std::string DemoTrajectoryPubNode::kPubFreqParam = "publish_frequency";
const std::string DemoTrajectoryPubNode::kTrajectoryHorizonParam = "trajectory_horizon";
const std::string DemoTrajectoryPubNode::kX0Param = "x0";
const std::string DemoTrajectoryPubNode::kY0Param = "y0";
const std::string DemoTrajectoryPubNode::kV0Param = "v0";
const std::string DemoTrajectoryPubNode::kAParam = "a";
const std::string DemoTrajectoryPubNode::kTheta0Param = "theta0";
const std::string DemoTrajectoryPubNode::kOmegaParam = "omega";
const std::string DemoTrajectoryPubNode::kNObjectsParam = "n_objects";
const std::string DemoTrajectoryPubNode::kObjectsDeltaX = "objects_delta_x";
const std::string DemoTrajectoryPubNode::kObjectsDeltaY = "objects_delta_y";

/**
 * @brief Creates a DemoTrajectoryPubNode node
 *
 */
DemoTrajectoryPubNode::DemoTrajectoryPubNode(const rclcpp::NodeOptions& options)
    : Node("demo_trajectory_pub_node", options) {
  this->declareParameters();
  this->loadParameters();
  this->setup();
}

/**
 * @brief Declares all parameters that this node uses
 */
void DemoTrajectoryPubNode::declareParameters() {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::FloatingPointRange param_range;

  param_desc.description = "Number of trajectory states";
  this->declare_parameter(kNStatesParam, n_states_, param_desc);

  param_desc.description = "Publish Frequency in Hz";
  this->declare_parameter(kPubFreqParam, pub_freq_, param_desc);

  param_desc.description = "Trajectory Horizon in seconds";
  this->declare_parameter(kTrajectoryHorizonParam, optimization_horizon_, param_desc);

  param_desc.description = "Initial x position";
  param_range.set__from_value(-5.0).set__to_value(5.0).set__step(0.5);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kX0Param, x0_, param_desc);

  param_desc.description = "Initial y position";
  param_range.set__from_value(-5.0).set__to_value(5.0).set__step(0.5);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kY0Param, y0_, param_desc);

  param_desc.description = "Initial velocity";
  param_range.set__from_value(-10.0).set__to_value(10.0).set__step(0.5);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kV0Param, v0_, param_desc);

  param_desc.description = "Acceleration";
  param_range.set__from_value(-5.0).set__to_value(5.0).set__step(0.5);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kAParam, a_, param_desc);

  param_desc.description = "Initial heading angle";
  param_range.set__from_value(-180.0).set__to_value(180.0).set__step(10.0);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kTheta0Param, theta0_, param_desc);

  param_desc.description = "Initial angular velocity";
  param_range.set__from_value(-45.0).set__to_value(45.0).set__step(5.0);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kOmegaParam, omega_, param_desc);

  param_desc = rcl_interfaces::msg::ParameterDescriptor();
  param_desc.description = "Number of objects in object list";
  this->declare_parameter(kNObjectsParam, n_objects_, param_desc);

  param_desc.description = "Delta x between objects";
  param_range.set__from_value(0.0).set__to_value(20.0).set__step(1.0);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kObjectsDeltaX, objects_delta_x_, param_desc);

  param_desc.description = "Delta y between objects";
  param_range.set__from_value(-5.0).set__to_value(5.0).set__step(1.0);
  param_desc.floating_point_range = {param_range};
  this->declare_parameter(kObjectsDeltaY, objects_delta_y_, param_desc);
}

/**
 * @brief Loads ROS parameters used in the node.
 *
 */
void DemoTrajectoryPubNode::loadParameters() {
  try {
    n_states_ = this->get_parameter(kNStatesParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kNStatesParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    pub_freq_ = this->get_parameter(kPubFreqParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kPubFreqParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    optimization_horizon_ = this->get_parameter(kTrajectoryHorizonParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is required", kTrajectoryHorizonParam.c_str());
    exit(EXIT_FAILURE);
  }
  try {
    x0_ = this->get_parameter(kX0Param).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kX0Param.c_str());
  }
  try {
    y0_ = this->get_parameter(kY0Param).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kY0Param.c_str());
  }
  try {
    v0_ = this->get_parameter(kV0Param).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kV0Param.c_str());
  }
  try {
    a_ = this->get_parameter(kAParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kAParam.c_str());
  }
  try {
    theta0_ = this->get_parameter(kTheta0Param).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kTheta0Param.c_str());
  }
  try {
    omega_ = this->get_parameter(kOmegaParam).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kOmegaParam.c_str());
  }
  try {
    n_objects_ = this->get_parameter(kNObjectsParam).as_int();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kNObjectsParam.c_str());
  }
  try {
    objects_delta_x_ = this->get_parameter(kObjectsDeltaX).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kObjectsDeltaX.c_str());
  }
  try {
    objects_delta_y_ = this->get_parameter(kObjectsDeltaY).as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_WARN(this->get_logger(), "Parameter '%s' is not specified, defaulting", kObjectsDeltaY.c_str());
  }
}

/**
 * @brief Handles reconfiguration when a parameter value is changed
 *
 * @param parameters parameters
 * @return parameter change result
 */
rcl_interfaces::msg::SetParametersResult DemoTrajectoryPubNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    if (param.get_name() == kX0Param) {
      x0_ = param.as_double();
    } else if (param.get_name() == kY0Param) {
      y0_ = param.as_double();
    } else if (param.get_name() == kV0Param) {
      v0_ = param.as_double();
    } else if (param.get_name() == kAParam) {
      a_ = param.as_double();
    } else if (param.get_name() == kTheta0Param) {
      theta0_ = param.as_double();
    } else if (param.get_name() == kOmegaParam) {
      omega_ = param.as_double();
    } else if (param.get_name() == kNObjectsParam) {
      n_objects_ = param.as_int();
    } else if (param.get_name() == kObjectsDeltaX) {
      objects_delta_x_ = param.as_double();
    } else if (param.get_name() == kObjectsDeltaY) {
      objects_delta_y_ = param.as_double();
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
void DemoTrajectoryPubNode::setup() {

  // set up publishers
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing Trajectories to '%s'", trajectory_pub_->get_topic_name());

  egodata_pub_ = this->create_publisher<perception_msgs::msg::EgoData>(kEgoDataTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing EgoData to '%s'", egodata_pub_->get_topic_name());

  object_list_pub_ = this->create_publisher<perception_msgs::msg::ObjectList>(kObjectListTopic, 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", object_list_pub_->get_topic_name());

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&DemoTrajectoryPubNode::parametersCallback, this, std::placeholders::_1));

  // create timer for planning cycle
  planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(pub_freq_),
                                            std::bind(&DemoTrajectoryPubNode::publish, this));
}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void DemoTrajectoryPubNode::publish() {

  // --- publish ego data ---

  perception_msgs::msg::EgoData::UniquePtr egodata = std::make_unique<perception_msgs::msg::EgoData>();
  perception_msgs::object_access::initializeState(*egodata, perception_msgs::msg::EGO::MODEL_ID);
  egodata->header.stamp = this->now();
  egodata->header.frame_id = "map";
  egodata_pub_->publish(std::move(egodata));

  // --- publish trajectory ---

  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      *trajectory, trajectory_planning_msgs::msg::REFERENCE::TYPE_ID, n_states_);

  trajectory->header.stamp = this->now();
  trajectory->header.frame_id = "map";
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, false);

  double t = 0.0;
  double dt = optimization_horizon_ / (n_states_ - 1);
  double x = x0_;
  double y = y0_;
  double theta = theta0_;
  double v = v0_;
  std::vector<double> state0 = {t, x, y, v};
  trajectory_planning_msgs::trajectory_access::setState(*trajectory, state0, 0);

  for (int i = 1; i < n_states_; i++) {

    double theta_rad = theta * M_PI / 180.0;
    double vx = v * std::cos(theta_rad);
    double vy = v * std::sin(theta_rad);
    double ax = a_ * std::cos(theta_rad);
    double ay = a_ * std::sin(theta_rad);

    t += dt;
    x = x + vx * dt + 0.5 * ax * dt * dt;
    y = y + vy * dt + 0.5 * ay * dt * dt;
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    v = std::sqrt(vx * vx + vy * vy);
    theta_rad = theta_rad + omega_ * M_PI / 180.0 * dt;
    theta = theta_rad * 180.0 / M_PI;

    std::vector<double> state = {t, x, y, v};
    trajectory_planning_msgs::trajectory_access::setState(*trajectory, state, i);
  }

  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_DEBUG(this->get_logger(), "Published trajectory");

  // --- publish object list ---

  perception_msgs::msg::ObjectList::UniquePtr object_list = std::make_unique<perception_msgs::msg::ObjectList>();
  object_list->header.stamp = this->now();
  object_list->header.frame_id = "map";

  for (int i = 0; i < n_objects_; i++) {
    perception_msgs::msg::Object obj;
    perception_msgs::object_access::initializeState(obj, perception_msgs::msg::ISCACTR::MODEL_ID);
    double l = 4.0;
    double w = 2.0;
    double h = 2.0;
    double x = objects_delta_x_ * (i + 1);
    double y = objects_delta_y_ * (i + 1);
    double z = h / 2;
    perception_msgs::object_access::setX(obj, x);
    perception_msgs::object_access::setY(obj, y);
    perception_msgs::object_access::setZ(obj, z);
    perception_msgs::object_access::setLength(obj, l);
    perception_msgs::object_access::setWidth(obj, w);
    perception_msgs::object_access::setHeight(obj, h);
    object_list->objects.push_back(obj);
  }

  object_list_pub_->publish(std::move(object_list));
}

}  // namespace demo_trajectory_pub
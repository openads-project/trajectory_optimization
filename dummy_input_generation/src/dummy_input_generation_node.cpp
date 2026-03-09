#include <math.h>

#include <chrono>
#include <functional>
#include <thread>

#include <dummy_input_generation/dummy_input_generation_node.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(dummy_input_generation::DummyInputGenerationNode)

/**
 * @brief Namespace for dummy_input_generation package
 *
 */
namespace dummy_input_generation {

/**
 * @brief Creates a DummyInputGenerationNode node
 *
 */
DummyInputGenerationNode::DummyInputGenerationNode(const rclcpp::NodeOptions& options)
    : Node("dummy_input_generation_node", options) {
  // declare and load node parameters; setup node
  this->declareAndLoadParameter("publish_frequency", publish_frequency_, "Publish frequency in Hz", true, true, false, 0.01, 100.0, 0.01);
  this->declareAndLoadParameter("message_frame_id", message_frame_id_, "Common frame ID for all published messages");

  this->declareAndLoadParameter("ego_state_model", ego_state_model_, "Ego state model used for published EgoData", true, false, false, std::nullopt, std::nullopt, std::nullopt, "Valid values: ackermann, rws");
  this->declareAndLoadParameter("ego_vel_lon", ego_vel_lon_, "Ego longitudinal velocity [m/s]", true, false, false, -20.0, 40.0, 0.1);
  this->declareAndLoadParameter("ego_acc_lon", ego_acc_lon_, "Ego longitudinal acceleration [m/s^2]", true, false, false, -10.0, 10.0, 0.1);
  this->declareAndLoadParameter("ego_steering_angle_ack", ego_steering_angle_ack_, "Ackermann steering angle [rad]", true, false, false, -M_PI/2, M_PI/2);
  this->declareAndLoadParameter("ego_steering_angle_front", ego_steering_angle_front_, "Front steering angle for RWS [rad]", true, false, false, -M_PI/2, M_PI/2);
  this->declareAndLoadParameter("ego_steering_angle_rear", ego_steering_angle_rear_, "Rear steering angle for RWS [rad]", true, false, false, -M_PI/2, M_PI/2);

  this->declareAndLoadParameter("reference_n_states", reference_n_states_, "Number of reference trajectory states", true, true, false, 2.0, 500.0, 1.0);
  this->declareAndLoadParameter("reference_trajectory_horizon", reference_trajectory_horizon_, "Reference trajectory horizon in seconds", true, true, false, 0.1, 60.0, 0.1);
  this->declareAndLoadParameter("reference_standstill", reference_standstill_, "Publish reference trajectory with standstill flag");
  this->declareAndLoadParameter("reference_x0", reference_x0_, "Initial x position of reference trajectory", true, false, false, -5.0, 5.0, 0.5);
  this->declareAndLoadParameter("reference_y0", reference_y0_, "Initial y position of reference trajectory", true, false, false, -5.0, 5.0, 0.5);
  this->declareAndLoadParameter("reference_v0", reference_v0_, "Initial velocity of reference trajectory", true, false, false, -10.0, 10.0, 0.5);
  this->declareAndLoadParameter("reference_a", reference_a_, "Acceleration of reference trajectory", true, false, false, -5.0, 5.0, 0.5);
  this->declareAndLoadParameter("reference_theta0", reference_theta0_, "Initial heading angle of reference trajectory [deg]", true, false, false, -180.0, 180.0, 10.0);
  this->declareAndLoadParameter("reference_omega", reference_omega_, "Angular velocity of reference trajectory [deg/s]", true, false, false, -45.0, 45.0, 5.0);

  this->declareAndLoadParameter("object_count", object_count_, "Number of objects in object list", true, false, false, 0.0, 100.0, 1.0);
  this->declareAndLoadParameter("object_delta_x", object_delta_x_, "Delta x between objects");
  this->declareAndLoadParameter("object_delta_y", object_delta_y_, "Delta y between objects");
  this->declareAndLoadParameter("object_length", object_length_, "Object length [m]", true, false, false, 0.1, 20.0, 0.1);
  this->declareAndLoadParameter("object_width", object_width_, "Object width [m]", true, false, false, 0.1, 10.0, 0.1);
  this->declareAndLoadParameter("object_yaw", object_yaw_, "Object yaw [rad]", true, false, false, -M_PI, M_PI);

  this->setup();
}

template <typename T>
void DummyInputGenerationNode::declareAndLoadParameter(const std::string& name,
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
rcl_interfaces::msg::SetParametersResult DummyInputGenerationNode::parametersCallback(const std::vector<rclcpp::Parameter>& parameters) {
  for (const auto& param : parameters) {
    for (auto& auto_reconfigurable_param : auto_reconfigurable_params_) {
      if (param.get_name() == std::get<0>(auto_reconfigurable_param)) {
        std::get<1>(auto_reconfigurable_param)(param);
        RCLCPP_INFO(this->get_logger(), "Reconfigured parameter '%s' to: %s", param.get_name().c_str(), param.value_to_string().c_str());
        break;
      }
    }
    if (param.get_name() == "publish_frequency") {
      createPlanningTimer();
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
void DummyInputGenerationNode::setup() {

  // create a callback for dynamic parameter configuration
  parameters_callback_ = this->add_on_set_parameters_callback(
      std::bind(&DummyInputGenerationNode::parametersCallback, this, std::placeholders::_1));

  // set up publishers
  trajectory_pub_ = this->create_publisher<trajectory_planning_msgs::msg::Trajectory>("~/reference_trajectory", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing Trajectories to '%s'", trajectory_pub_->get_topic_name());

  egodata_pub_ = this->create_publisher<perception_msgs::msg::EgoData>("~/ego_data", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing EgoData to '%s'", egodata_pub_->get_topic_name());

  object_list_pub_ = this->create_publisher<perception_msgs::msg::ObjectList>("~/object_list", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", object_list_pub_->get_topic_name());

  createPlanningTimer();
}

void DummyInputGenerationNode::createPlanningTimer() {
  if (planning_timer_) {
    planning_timer_->cancel();
    planning_timer_.reset();
  }
  planning_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / publish_frequency_),
                                            std::bind(&DummyInputGenerationNode::publish, this));
  RCLCPP_INFO(this->get_logger(), "Planning timer updated to %.3f Hz.", publish_frequency_);
}

/**
 * @brief This function is invoked every period seconds by the timer
 *
 */
void DummyInputGenerationNode::publish() {

  // --- publish ego data ---

  perception_msgs::msg::EgoData::UniquePtr egodata = std::make_unique<perception_msgs::msg::EgoData>();
  if (ego_state_model_ == "ackermann") {
    perception_msgs::object_access::initializeState(*egodata, perception_msgs::msg::EGO::MODEL_ID);
    perception_msgs::object_access::setVelLon(*egodata, ego_vel_lon_);
    perception_msgs::object_access::setAccLon(*egodata, ego_acc_lon_);
    perception_msgs::object_access::setSteeringAngleAck(*egodata, ego_steering_angle_ack_);
  } else if (ego_state_model_ == "rws") {
    perception_msgs::object_access::initializeState(*egodata, perception_msgs::msg::EGORWS::MODEL_ID);
    perception_msgs::object_access::setVelLon(*egodata, ego_vel_lon_);
    perception_msgs::object_access::setVelLat(*egodata, 0.0);
    perception_msgs::object_access::setAccLon(*egodata, ego_acc_lon_);
    perception_msgs::object_access::setAccLat(*egodata, 0.0);
    perception_msgs::object_access::setSteeringAngleFront(*egodata, ego_steering_angle_front_);
    perception_msgs::object_access::setSteeringAngleRear(*egodata, ego_steering_angle_rear_);
  } else {
    RCLCPP_FATAL(this->get_logger(), "Invalid ego_state_model '%s'. Valid values are 'ackermann' and 'rws'.", ego_state_model_.c_str());
    exit(EXIT_FAILURE);
  }
  egodata->length = EGO_LENGTH;
  egodata->width = EGO_WIDTH;
  egodata->height = OBJECT_HEIGHT;
  egodata->header.stamp = this->now();
  egodata->header.frame_id = message_frame_id_;
  egodata_pub_->publish(std::move(egodata));

  // --- publish trajectory ---

  trajectory_planning_msgs::msg::Trajectory::UniquePtr trajectory =
      std::make_unique<trajectory_planning_msgs::msg::Trajectory>();
  trajectory_planning_msgs::trajectory_access::initializeTrajectory(
      *trajectory, trajectory_planning_msgs::msg::REFERENCE::TYPE_ID, reference_n_states_);

  trajectory->header.stamp = this->now();
  trajectory->header.frame_id = message_frame_id_;
  trajectory_planning_msgs::trajectory_access::setStandstill(*trajectory, reference_standstill_);

  double t = 0.0;
  double dt = reference_trajectory_horizon_ / (reference_n_states_ - 1);
  double x = reference_x0_;
  double y = reference_y0_;
  double theta = reference_theta0_;
  double v = reference_v0_;
  std::vector<double> state0 = {t, x, y, v};
  trajectory_planning_msgs::trajectory_access::setState(*trajectory, state0, 0);

  for (int i = 1; i < reference_n_states_; i++) {

    double theta_rad = theta * M_PI / 180.0;
    double vx = v * std::cos(theta_rad);
    double vy = v * std::sin(theta_rad);
    double ax = reference_a_ * std::cos(theta_rad);
    double ay = reference_a_ * std::sin(theta_rad);

    t += dt;
    x = x + vx * dt + 0.5 * ax * dt * dt;
    y = y + vy * dt + 0.5 * ay * dt * dt;
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    v = std::sqrt(vx * vx + vy * vy);
    theta_rad = theta_rad + reference_omega_ * M_PI / 180.0 * dt;
    theta = theta_rad * 180.0 / M_PI;

    std::vector<double> state = {t, x, y, v};
    trajectory_planning_msgs::trajectory_access::setState(*trajectory, state, i);
  }

  trajectory_pub_->publish(std::move(trajectory));
  RCLCPP_DEBUG(this->get_logger(), "Published trajectory");

  // --- publish object list ---

  perception_msgs::msg::ObjectList::UniquePtr object_list = std::make_unique<perception_msgs::msg::ObjectList>();
  object_list->header.stamp = this->now();
  object_list->header.frame_id = message_frame_id_;

  for (int i = 0; i < object_count_; i++) {
    perception_msgs::msg::Object obj;
    perception_msgs::object_access::initializeState(obj, perception_msgs::msg::ISCACTR::MODEL_ID);
    double x = object_delta_x_ * (i + 1);
    double y = object_delta_y_ * (i + 1);
    double h = OBJECT_HEIGHT;
    double z = h / 2;
    perception_msgs::object_access::setX(obj, x);
    perception_msgs::object_access::setY(obj, y);
    perception_msgs::object_access::setZ(obj, z);
    perception_msgs::object_access::setYaw(obj, object_yaw_);
    perception_msgs::object_access::setLength(obj, object_length_);
    perception_msgs::object_access::setWidth(obj, object_width_);
    perception_msgs::object_access::setHeight(obj, h);
    object_list->objects.push_back(obj);
  }

  object_list_pub_->publish(std::move(object_list));
}

}  // namespace dummy_input_generation

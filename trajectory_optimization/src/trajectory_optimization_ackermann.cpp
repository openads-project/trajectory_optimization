
#include <trajectory_optimization/trajectory_optimization_ackermann.hpp>

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimization::TrajectoryOptimizationAckermannNode)


namespace trajectory_optimization {

TrajectoryOptimizationAckermannNode::TrajectoryOptimizationAckermannNode(const rclcpp::NodeOptions &options)
    : TrajectoryOptimizationNode("TrajectoryOptimizationAckermannNode", options)
{
    this->declareAndLoadParameter("bi_level_dDelta", bi_level_dDelta_,
        "Threshold for bi-level stabilization: maximum ackermann steering angle difference [degree]");
}

TrajectoryOptimizationAckermannNode::~TrajectoryOptimizationAckermannNode() = default;

void TrajectoryOptimizationAckermannNode::initializeTrajectory(trajectory_planning_msgs::msg::Trajectory& trajectory) {
    trajectory_planning_msgs::trajectory_access::initializeTrajectory(
        trajectory, trajectory_planning_msgs::msg::DRIVABLE::TYPE_ID, n_shots_ + 1);
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
std::vector<double> TrajectoryOptimizationAckermannNode::getBiLevelX0(const perception_msgs::msg::EgoData& ego_data) {
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
      DELTA.push_back(trajectory_planning_msgs::trajectory_access::getDeltaAck(tf_trajectory, i));
    }

    // interpolate target states by time from the extracted vectors; if not successful, set to ego state (high-level initialization)
    double v_tgt, y_tgt, a_tgt, theta_tgt, delta_tgt;
    double des_time =
        (rclcpp::Time(ego_data.header.stamp) - rclcpp::Time(tf_trajectory.header.stamp)).seconds();
    if (!linearInterpolation(TIME, Y, des_time, y_tgt)) y_tgt = 0.0;
    if (!linearInterpolation(TIME, V, des_time, v_tgt)) v_tgt = perception_msgs::object_access::getVelLon(ego_data);
    if (!linearInterpolation(TIME, A, des_time, a_tgt)) a_tgt = perception_msgs::object_access::getAccLon(ego_data);
    if (!linearInterpolation(TIME, THETA, des_time, theta_tgt, true)) theta_tgt = 0.0;
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
    } else if (fabs(delta_tgt - perception_msgs::object_access::getSteeringAngleAck(ego_data)) > bi_level_dDelta_ * M_PI / 180.0) {
        delta_tgt = perception_msgs::object_access::getSteeringAngleAck(ego_data);
    }

    std::vector<double> x_init(*nlp_dims_->nx, 0.0);
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
std::vector<double> TrajectoryOptimizationAckermannNode::getHighLevelX0(const perception_msgs::msg::EgoData& ego_data) {
    std::vector<double> x_init(*nlp_dims_->nx, 0.0);
    x_init[3] = perception_msgs::object_access::getVelLon(ego_data);
    x_init[4] = 0.0;  //x_init[4] = perception_msgs::object_access::getAccLon(ego_data);
    x_init[6] = perception_msgs::object_access::getSteeringAngleAck(ego_data);

    return x_init;
}

void TrajectoryOptimizationAckermannNode::convertToTrajectoryMsg(trajectory_planning_msgs::msg::Trajectory& trajectory) {
    for (int i = 0; i <= n_shots_; ++i) {
        trajectory_planning_msgs::trajectory_access::setX(trajectory, xtraj_[i * *nlp_dims_->nx + 0], i);
        trajectory_planning_msgs::trajectory_access::setY(trajectory, xtraj_[i * *nlp_dims_->nx + 1], i);
        trajectory_planning_msgs::trajectory_access::setS(trajectory, xtraj_[i * *nlp_dims_->nx + 2], i);
        trajectory_planning_msgs::trajectory_access::setV(trajectory, xtraj_[i * *nlp_dims_->nx + 3], i);
        trajectory_planning_msgs::trajectory_access::setA(trajectory, xtraj_[i * *nlp_dims_->nx + 4], i);
        trajectory_planning_msgs::trajectory_access::setTheta(trajectory, xtraj_[i * *nlp_dims_->nx + 5], i);
        trajectory_planning_msgs::trajectory_access::setDeltaAck(trajectory, xtraj_[i * *nlp_dims_->nx + 6], i);
    }
}

} // namespace trajectory_optimization
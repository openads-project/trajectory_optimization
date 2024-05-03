#include <cmath>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

namespace trajectory_optimization {
/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input ego data
 */
void TrajectoryOptimizationNode::egoDataCallback(const perception_msgs::msg::EgoData::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received ego data");
  ego_data_ = *msg;
  received_ego_data_ = true;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input drivable space
 */
void TrajectoryOptimizationNode::driveableSpaceCallback(
    const route_planning_msgs::msg::DriveableSpace::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received driveable space");
  driveable_space_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input object list
 */
void TrajectoryOptimizationNode::objectListCallback(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "Received object list");

  // transform to reference frame
  perception_msgs::msg::ObjectList tf_object_list;
  geometry_msgs::msg::TransformStamped tf;
  try {
    geometry_msgs::msg::TransformStamped tf = tf2_buffer_->lookupTransform(vehicle_frame_id_, msg->header.frame_id, msg->header.stamp);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform object list from frame '%s' to '%s': %s",
                 msg->header.frame_id.c_str(), vehicle_frame_id_.c_str(), ex.what());
    return;
  }
  tf2::doTransform(*msg, tf_object_list, tf);

  // calculate distance to each object
  std::vector<double> distances;
  for (size_t i = 0; i < tf_object_list.objects.size(); ++i) {
    double distance = std::sqrt(std::pow(perception_msgs::object_access::getX(tf_object_list.objects[i]), 2) +
                                std::pow(perception_msgs::object_access::getY(tf_object_list.objects[i]), 2));
    distances.push_back(distance);
  }

  // sort objects by distance
  std::vector<size_t> indices_sorted_by_distance(distances.size());
  std::iota(indices_sorted_by_distance.begin(), indices_sorted_by_distance.end(), 0);
  std::sort(indices_sorted_by_distance.begin(), indices_sorted_by_distance.end(), [&distances](size_t i1, size_t i2) {
    return distances[i1] < distances[i2];
  });

  // keep only the closest objects
  std::vector<perception_msgs::msg::Object> closest_objects;
  const int n_objects = std::min<size_t>(p_obstacles_shape_[0], indices_sorted_by_distance.size());
  for (size_t i = 0; i < n_objects; ++i) {
    closest_objects.push_back(tf_object_list.objects[indices_sorted_by_distance[i]]);
  }
  tf_object_list.objects = closest_objects;

  // store as current object list
  object_list_ = tf_object_list;
  received_object_list_ = true;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input reference trajectory
 */
void TrajectoryOptimizationNode::referenceTrajectoryCallback(
    const trajectory_planning_msgs::msg::Trajectory::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received reference trajectory");
  reference_trajectory_ = *msg;
}

/**
 * @brief This callback is invoked when the subscriber receives a message
 *
 * @param[in] msg   input route
 */
void TrajectoryOptimizationNode::routeCallback(const route_planning_msgs::msg::Route::ConstSharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received route");
  route_ = *msg;
}
}  // namespace trajectory_optimization
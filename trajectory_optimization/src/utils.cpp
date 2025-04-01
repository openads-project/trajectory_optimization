#include <cmath>
#include <limits>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

namespace trajectory_optimization {

/**
 * Wraps an angle in radians within a specified range.
 *
 * @param angle_rad The angle in radians to be wrapped.
 * @param min_val The minimum value of the range (default: -M_PI).
 * @param max_val The maximum value of the range (default: M_PI).
 * @return The wrapped angle within the specified range.
 */
double TrajectoryOptimizationNode::wrap_angle_rad(double angle_rad, double min_val, double max_val) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > max_val) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < min_val) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

/**
 * @brief Performs linear interpolation to find the corresponding y-value for a given x-value.
 *
 * This function takes two vectors, X and Y, representing the x and y values of a dataset, and a desired x-value.
 * It performs linear interpolation to find the corresponding y-value for the desired x-value.
 *
 * @param X The vector of x-values.
 * @param Y The vector of y-values.
 * @param desired_x The desired x-value.
 * @param output_y The output variable to store the interpolated y-value.
 * @param wrap_angle If true (relevant if Y is an angle list), angle differences are wrapped to [-pi, pi] (default: false).
 * @return True if the interpolation is successful, false otherwise.
 */
bool TrajectoryOptimizationNode::linearInterpolation(const std::vector<double>& X, const std::vector<double>& Y,
                                                     const double& desired_x, double& output_y, bool wrap_angle) {
  if(desired_x == X.front()) {
    RCLCPP_DEBUG(get_logger(), "Desired Time is equal to Time-Min of the given vector!");
    output_y = Y.front();
    return true;
  } else if(desired_x == X.back()) {
    RCLCPP_DEBUG(get_logger(), "Desired Time is equal to Time-Max of the given vector!");
    output_y = Y.back();
    return true;
  } else if (desired_x < *min_element(X.begin(), X.end())) {
    RCLCPP_WARN(get_logger(), "Desired Time is smaller than Time-Min of the given vector! Using first valid value.");
    RCLCPP_DEBUG(get_logger(), "Desired Time: %f s", desired_x);
    RCLCPP_DEBUG(get_logger(), "Time-Min: %f s", *min_element(X.begin(), X.end()));
    output_y = Y.front();
    return false;
  } else if (desired_x > *max_element(X.begin(), X.end())) {
    RCLCPP_WARN(get_logger(), "Desired Time is greater than Time-Max of the given vector! Using last valid value.");
    RCLCPP_DEBUG(get_logger(), "Desired Time: %f s", desired_x);
    RCLCPP_DEBUG(get_logger(), "Time-Max: %f s", *max_element(X.begin(), X.end()));
    output_y = Y.back();
    return false;
  } else if (X.size() != Y.size()) {
    RCLCPP_ERROR(get_logger(), "Input vectors don't have the same length!");
    return false;
  }

  //go through array and search for sampling points
  size_t i;
  for (i = 0; i < X.size(); i++) {
    if (X[i] < desired_x) {
      continue;
    } else if (X[i] == desired_x) {
      output_y = Y[i];
      return true;
    } else {
      break;
    }
  }
  double diff = Y[i] - Y[i-1];
  if (wrap_angle){
    diff = wrap_angle_rad(diff);
  }
  output_y = Y[i - 1] + (diff / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
  if (wrap_angle) {
    output_y = wrap_angle_rad(output_y);
  }
  return true;
}

/**
 * @brief Converts a trajectory from the vehicle frame to the output frame.
 *
 * This function converts a trajectory from the vehicle frame to the output frame using the tf2 library.
 *
 * @param trajectory The trajectory to be converted.
 */
void TrajectoryOptimizationNode::trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory& trajectory) {
  if (trajectory_frame_id_ != vehicle_frame_id_) {
    trajectory_planning_msgs::msg::Trajectory tf_trajectory;
    try {
      tf_trajectory = tf2_buffer_->transform(trajectory, trajectory_frame_id_, tf2::durationFromSec(0.01));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Transformation into output frame is not available. Publishing no trajectory. Ex: %s", ex.what());
      return;
    }
    trajectory = tf_trajectory;
  }
}

/**
 * @brief Keeps the N closest objects from the given object list (relative to the header frame).
 * 
 * This function calculates the distance to each object in the object list and keeps the N closest objects (relative to the header frame).
 * All objects with a negative x-coordinate are discarded (ignores objects behind the ego vehicle).
 *
 * @param object_list The object list to filter.
 * @param n_objects The number of closest objects to keep.
 */
void TrajectoryOptimizationNode::keepNClosestObjects(perception_msgs::msg::ObjectList& object_list,
                                                     const int n_objects) {
  // calculate distance to each object
  std::vector<double> distances;
  for (size_t i = 0; i < object_list.objects.size(); ++i) {
    double distance = std::sqrt(std::pow(perception_msgs::object_access::getX(object_list.objects[i]), 2) +
                                std::pow(perception_msgs::object_access::getY(object_list.objects[i]), 2));
    distances.push_back(distance);
  }

  // sort objects by distance
  std::vector<size_t> indices_sorted_by_distance(distances.size());
  std::iota(indices_sorted_by_distance.begin(), indices_sorted_by_distance.end(), 0);
  std::sort(indices_sorted_by_distance.begin(), indices_sorted_by_distance.end(),
            [&distances](size_t i1, size_t i2) { return distances[i1] < distances[i2]; });

  // keep only the closest objects
  std::vector<perception_msgs::msg::Object> closest_objects;
  const int n_objects_to_keep = std::min<size_t>(n_objects, indices_sorted_by_distance.size());
  int i = 0;
  while(closest_objects.size() < (size_t)n_objects_to_keep) {
    if((size_t)i >= indices_sorted_by_distance.size()) break;
    // ignore object with negative x-coordinate (behind the ego vehicle)
    if(perception_msgs::object_access::getX(object_list.objects[indices_sorted_by_distance[i]]) > 0.0) {
      closest_objects.push_back(object_list.objects[indices_sorted_by_distance[i]]);
    }
    ++i;
  }
  object_list.objects = closest_objects;
}

std::vector<double> TrajectoryOptimizationNode::discretizeBB2Circles(const double x, const double y, const double yaw, const double length, const double width) {
  
  uint8_t n_circles = 1;
  if (length <= 0.0 || width <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid bounding box dimensions: length = %f, width = %f. Setting n_circles = 1.", length, width);
  } else {
    double aspect_ratio = length / width;
    if (aspect_ratio > 8.0) n_circles = 9;
    else if (aspect_ratio > 6.0) n_circles = 7;
    else if (aspect_ratio > 4.0) n_circles = 5;
    else if (aspect_ratio > 1.8) n_circles = 3;
    else if (aspect_ratio > 1.3) n_circles = 2;
    else n_circles = 1;
  }

  double radius = std::sqrt(std::pow(length / (2 * n_circles), 2) + std::pow(width / 2.0, 2));

  std::vector<double> circles(p_obstacle_circles_shape_[1] * n_circles);

  for (int i = 0; i < n_circles; i++) {
    double lon_offset = -length / 2 + (2 * i + 1) * length / (2 * n_circles);
    double x_offset = lon_offset * std::cos(yaw);
    double y_offset = lon_offset * std::sin(yaw);
    circles[p_obstacle_circles_shape_[1] * i + 0] = x + x_offset;
    circles[p_obstacle_circles_shape_[1] * i + 1] = y + y_offset;
    circles[p_obstacle_circles_shape_[1] * i + 2] = radius;
  }

  return circles;
}

void TrajectoryOptimizationNode::vizCircles(const std::vector<double>& obstacles) {
  visualization_msgs::msg::MarkerArray marker_array;
  int n_circles = obstacles.size() / p_obstacle_circles_shape_[1];
  for (int j = 0; j < n_circles; ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = vehicle_frame_id_;
    marker.header.stamp = rclcpp::Time(ego_data_.header.stamp);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "obstacle-circles";
    marker.id = j;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obstacles[p_obstacle_circles_shape_[1] * j + 0];
    marker.pose.position.y = obstacles[p_obstacle_circles_shape_[1] * j + 1];
    marker.scale.x = obstacles[p_obstacle_circles_shape_[1] * j + 2] * 2.0;
    marker.scale.y = obstacles[p_obstacle_circles_shape_[1] * j + 2] * 2.0;
    marker.scale.z = 0.1;
    marker.color.a = 0.3;
    marker.color.r = 1.0;
    marker_array.markers.push_back(marker);
  }
  circles_pub_->publish(marker_array);
}

/**
 * @brief Prints the solution of the trajectory optimization problem.
 *
 * This function prints the solution of the trajectory optimization problem, including the resulting xtraj and utraj,
 * as well as some solver statistics.
 *
 * @param status The status of the trajectory optimization solver.
 */
void TrajectoryOptimizationNode::printSolution(int status) {
  if (status == ACADOS_SUCCESS) {
    RCLCPP_INFO(get_logger(), "\033[1;32mOptimization: SUCCESS!\033[0m");
  } else if (status == ACADOS_MAXITER) {
    RCLCPP_WARN(get_logger(), "Optimization failed with status %d (max iterations).", status);
  } else if (status == ACADOS_TIMEOUT) {
    RCLCPP_WARN(get_logger(), "\033[38;5;214mOptimization failed with status %d (timeout).\033[0m", status);
  } else {
    RCLCPP_ERROR(get_logger(), "%s_acados_solve() failed with status %d.", model_name_.c_str(), status);
  }

  // print duration, KKT, and number of SQP iterations
  double elapsed_time, kkt_norm_inf;
  int sqp_iter;
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);
  RCLCPP_INFO(get_logger(),
            "Optimization took \033[1m%f ms.\033[0m (SQP iter: \033[1m%2d\033[0m; KKT: \033[1m%e\033[0m)",
            elapsed_time * 1000, sqp_iter, kkt_norm_inf);
  
  // print cost value and residuals
  double cost_value, nlp_res;
  ocp_nlp_eval_cost(nlp_solver_, nlp_in_, nlp_out_);
  ocp_nlp_eval_residuals(nlp_solver_, nlp_in_, nlp_out_);
  ocp_nlp_get(nlp_solver_, "cost_value", &cost_value);
  ocp_nlp_get(nlp_solver_, "nlp_res", &nlp_res);
  RCLCPP_INFO(get_logger(), "cost_value: \033[1m%f\033[0m; nlp_res: \033[1m%f\033[0m", cost_value, nlp_res);

  if (verbose_) {
    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat(*nlp_dims_->nx, n_shots_ + 1, xtraj_, *nlp_dims_->nx);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat(*nlp_dims_->nu, n_shots_, utraj_, *nlp_dims_->nu);
    trajectory_optimization::acados_print_stats(acados_ocp_capsule_);
  }
}

std::vector<double> TrajectoryOptimizationNode::projectVector(const perception_msgs::gm::Vector3& a, const perception_msgs::gm::Vector3& b) {
  double b_magnitude_squared = b.x * b.x + b.y * b.y;
  if (b_magnitude_squared < std::numeric_limits<double>::epsilon()) {
    return {0.0, 0.0};
  }
  double scale = (a.x * b.x + a.y * b.y) / b_magnitude_squared;
  return {scale * b.x, scale * b.y};
}

double TrajectoryOptimizationNode::computeMagnitude(const std::vector<double>& a) {
  return std::sqrt(a[0] * a[0] + a[1] * a[1]);
}

}  // namespace trajectory_optimization
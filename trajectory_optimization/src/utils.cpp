// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

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
 * @return True if conversion succeeded or was not needed, false if conversion failed.
 */
bool TrajectoryOptimizationNode::trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory& trajectory) {
  if (trajectory_frame_id_ != vehicle_frame_id_) {
    trajectory_planning_msgs::msg::Trajectory tf_trajectory;
    try {
      tf_trajectory = tf2_buffer_->transform(trajectory, trajectory_frame_id_, tf2::durationFromSec(0.01));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Transformation into output frame is not available. Publishing no trajectory. Ex: %s", ex.what());
      return false;
    }
    trajectory = tf_trajectory;
  }
  return true;
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

std::vector<std::pair<double, double>> TrajectoryOptimizationNode::normalBoundaryDistance(const trajectory_planning_msgs::msg::Trajectory& reference_trajectory,
                                                                                          const route_planning_msgs::msg::Route& route) {
  const double NO_BOUNDARY_DISTANCE = 1e6; // should be smaller than MAX_BOUNDARY_CONSTRAINT from ocp

  struct Boundaries {
    std::vector<std::pair<double, double>> min_normal_distances;
    std::vector<Eigen::Vector2d> left_boundary_points;
    std::vector<Eigen::Vector2d> right_boundary_points;
    std::vector<Eigen::Vector2d> left_boundary_intersections;
    std::vector<Eigen::Vector2d> right_boundary_intersections;
  };

  Boundaries boundaries;
  const auto remaining_route = route_planning_msgs::route_access::getRemainingRouteElements(route, true);
  const int ref_sample_size = trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory);
  boundaries.left_boundary_points.reserve(remaining_route.size());
  boundaries.right_boundary_points.reserve(remaining_route.size());
  boundaries.min_normal_distances.reserve(ref_sample_size);
  boundaries.left_boundary_intersections.reserve(ref_sample_size);
  boundaries.right_boundary_intersections.reserve(ref_sample_size);

  if (remaining_route.size() < 1) {
    RCLCPP_WARN(get_logger(), "Remaining route is empty. Do not constrain boundaries.");
    for (int i = 0; i < ref_sample_size; ++i) {
      boundaries.min_normal_distances.emplace_back(NO_BOUNDARY_DISTANCE, NO_BOUNDARY_DISTANCE);
    }
    return boundaries.min_normal_distances;
  }

  for (const auto& route_element : remaining_route) {
    if (route_element.is_enriched) {
      if (consider_boundaries_ == CONSIDER_BOUNDARIES::SUGGESTED_LANE) {
        route_planning_msgs::msg::LaneElement suggested_lane = route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
        boundaries.left_boundary_points.emplace_back(suggested_lane.left_boundary.point.x, suggested_lane.left_boundary.point.y);
        boundaries.right_boundary_points.emplace_back(suggested_lane.right_boundary.point.x, suggested_lane.right_boundary.point.y);
      } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::INCLUDING_ADJACENT) {
        const auto& lane_elements = route_element.lane_elements;
        if (!lane_elements.empty()) {
          boundaries.left_boundary_points.emplace_back(lane_elements.front().left_boundary.point.x, lane_elements.front().left_boundary.point.y);
          boundaries.right_boundary_points.emplace_back(lane_elements.back().right_boundary.point.x, lane_elements.back().right_boundary.point.y);
        }
      } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::DRIVABLE_SPACE) {
        boundaries.left_boundary_points.emplace_back(route_element.left_boundary.x, route_element.left_boundary.y);
        boundaries.right_boundary_points.emplace_back(route_element.right_boundary.x, route_element.right_boundary.y);
      }
    }
  }

  // Helper lambda to find intersection
  auto findIntersection = [this](const Eigen::Vector2d& ref_pos,
                                  double sin_yaw,
                                  double cos_yaw,
                                  const std::vector<Eigen::Vector2d>& boundary_points,
                                  bool isLeft) -> std::pair<double, Eigen::Vector2d>
    {
      std::pair<double,Eigen::Vector2d> intersection_result = {
        std::numeric_limits<double>::infinity(),
        Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity())
      };
      
      const Eigen::Vector2d normal_dir = isLeft ? Eigen::Vector2d(-sin_yaw, cos_yaw)
                                                : Eigen::Vector2d(sin_yaw, -cos_yaw);
      const auto cross2d = [](const Eigen::Vector2d& u, const Eigen::Vector2d& v) {
        return u.x() * v.y() - u.y() * v.x();
      };

      for (size_t i = 0; i + 1 < boundary_points.size(); ++i) {
        const Eigen::Vector2d& a = boundary_points[i];
        const Eigen::Vector2d& b = boundary_points[i + 1];
        Eigen::Vector2d seg = b - a;
        Eigen::Vector2d ap = ref_pos - a;

        const double denom = cross2d(seg, normal_dir);
        if (std::abs(denom) < 1e-9) {
          continue;  // Lines are close to parallel; ignore this segment
        }

        const double s = cross2d(ap, normal_dir) / denom;
        const double t = cross2d(ap, seg) / denom;

        if (s >= 0.0 && s <= 1.0 && t >= 0.0) {
          Eigen::Vector2d intersection = a + s * seg;
          double euklidean_distance = (ref_pos - intersection).norm();
          if (euklidean_distance < intersection_result.first) {
            intersection_result.first = euklidean_distance;
            intersection_result.second = intersection;
          }
        } 
      }
      return intersection_result;
    };

  // Loop over trajectory points and compute intersections
  for (int i = 0; i < ref_sample_size; ++i) {
    Eigen::Vector2d ref_pos(trajectory_planning_msgs::trajectory_access::getX(reference_trajectory, i),
                            trajectory_planning_msgs::trajectory_access::getY(reference_trajectory, i));
    double yaw = trajectory_planning_msgs::trajectory_access::getTheta(reference_trajectory, i);
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);

    auto left_intersection = findIntersection(ref_pos, sin_yaw, cos_yaw, boundaries.left_boundary_points, true);
    auto right_intersection = findIntersection(ref_pos, sin_yaw, cos_yaw, boundaries.right_boundary_points, false);
    if ( left_intersection.first != std::numeric_limits<double>::infinity() && right_intersection.first != std::numeric_limits<double>::infinity()) {
      boundaries.min_normal_distances.emplace_back(left_intersection.first, right_intersection.first);
      boundaries.left_boundary_intersections.emplace_back(left_intersection.second);
      boundaries.right_boundary_intersections.emplace_back(right_intersection.second);
      RCLCPP_DEBUG(this->get_logger(), "Minimum left boundary distance: %.2f m", left_intersection.first);
      RCLCPP_DEBUG(this->get_logger(), "Minimum right boundary distance: %.2f m", right_intersection.first);
    } else {
      boundaries.min_normal_distances.emplace_back(NO_BOUNDARY_DISTANCE, NO_BOUNDARY_DISTANCE);
      RCLCPP_WARN(get_logger(), "No boundary intersection found for trajectory point %d. Do not constrain boundaries at this point.", i);
    }
  }
  if (debug_viz_) {
    vizBoundaryPoints(boundaries.left_boundary_points, boundaries.right_boundary_points, false);
    vizBoundaryPoints(boundaries.left_boundary_intersections, boundaries.right_boundary_intersections, true);
  }

  return boundaries.min_normal_distances;
}

void TrajectoryOptimizationNode::vizBoundaryPoints(const std::vector<Eigen::Vector2d>& left_boundary_points,
                                                   const std::vector<Eigen::Vector2d>& right_boundary_points,
                                                   bool is_intersection) {
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;

  auto createMarker = [&](int id, const std::string& ns, const Eigen::Vector2d& pos, float r, float g, float b) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = vehicle_frame_id_;
    m.header.stamp = rclcpp::Time(ego_data_.header.stamp);
    m.lifetime = rclcpp::Duration::from_seconds(0.5);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = 0.0;
    m.scale.x = m.scale.y = m.scale.z = 0.4;
    m.color.a = 1.0;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    return m;
  };

  auto addMarkers = [&](const std::vector<Eigen::Vector2d>& points, const std::string& ns, float r, float g, float b) {
    for (const auto& point : points) {
      marker_array.markers.push_back(createMarker(id++, ns, point, r, g, b));
    }
  };

  if (is_intersection) {
    addMarkers(left_boundary_points,  "left_intersection_points",  0.5f, 0.0f, 1.0f);
    addMarkers(right_boundary_points, "right_intersection_points", 0.0f, 0.5f, 1.0f);
  } else {
    addMarkers(left_boundary_points,  "left_boundary_points",  0.0f, 1.0f, 0.0f);
    addMarkers(right_boundary_points, "right_boundary_points", 1.0f, 0.0f, 0.0f);
  }

  boundary_pub_->publish(marker_array);
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
 * @brief Publishes visualization markers representing the ego vehicle as its represented in the OCP.
 * 
 * Enables debug view of the ego vehicle approximation to check collision with bounds or obstacles.
 * Relevant parameters are hardcoded here (as they are defined in the OCP).
 *
 * @param x_trajectory The trajectory of the ego vehicle. As represented in the OCP.
 * @param model_name The name of the acados model used in the OCP.
 */
void TrajectoryOptimizationNode::vizEgoCircles(const double* x_trajectory, const std::string& model_name) {
  if (!ego_circles_pub_) return;

  double ego_length, ego_width;
  int n_ego_circles;
  std::vector<double> ego_offset2geocenter;

  // define vehicle geometry based on model name (should match the OCP definition)
  if (model_name == "karl") {
    ego_length = 5.173;
    ego_width = 2.252;
    ego_offset2geocenter = {1.4895, 0.0};
    n_ego_circles = 5;
  } else if (model_name == "shuttle" || model_name == "shuttle_ackermann") {
    ego_length = 4.97;
    ego_width = 2.12;
    ego_offset2geocenter = {0.0, 0.0};
    n_ego_circles = 3;
  } else if (model_name == "taxi") {
    ego_length = 4.37;
    ego_width = 2.12;
    ego_offset2geocenter = {0.0, 0.0};
    n_ego_circles = 3;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown model '%s'. Could not visualize ego circles.", model_name.c_str());
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  if (!x_trajectory || n_ego_circles <= 0) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    ego_circles_pub_->publish(marker_array);
    return;
  }

  const double offset_x = ego_offset2geocenter.size() > 0 ? ego_offset2geocenter[0] : 0.0;
  const double offset_y = ego_offset2geocenter.size() > 1 ? ego_offset2geocenter[1] : 0.0;

  const double radius = std::sqrt(std::pow(ego_length / (2.0 * n_ego_circles), 2) + std::pow(ego_width / 2.0, 2));
  const int state_dim = *nlp_dims_->nx;

  int marker_id = 0;
  for (int stage = 0; stage <= n_shots_; ++stage) {
    const double* state = &x_trajectory[stage * state_dim];
    const double base_x = state[0];
    const double base_y = state[1];
    const double psi = state[5];

    const double ego_center_x = base_x + offset_x * std::cos(psi) - offset_y * std::sin(psi);
    const double ego_center_y = base_y + offset_x * std::sin(psi) + offset_y * std::cos(psi);

    for (int i = 0; i < n_ego_circles; ++i) {
      const double lon_offset = -ego_length / 2.0 + (2 * i + 1) * ego_length / (2.0 * n_ego_circles);
      const double x_offset = lon_offset * std::cos(psi);
      const double y_offset = lon_offset * std::sin(psi);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = vehicle_frame_id_;
      marker.header.stamp = rclcpp::Time(ego_data_.header.stamp);
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker.ns = "ego-circles";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = ego_center_x + x_offset;
      marker.pose.position.y = ego_center_y + y_offset;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = radius * 2.0;
      marker.scale.y = radius * 2.0;
      marker.scale.z = 0.05;
      marker.color.a = 0.4;
      marker.color.r = 0.0f;
      marker.color.g = 0.6f;
      marker.color.b = 1.0f;
      marker_array.markers.push_back(marker);
    }
  }

  ego_circles_pub_->publish(marker_array);
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
  // Status codes:
  // 0: Success (ACADOS_SUCCESS)
  // 1: NaN detected (ACADOS_NAN_DETECTED)
  // 2: Maximum number of iterations reached (ACADOS_MAXITER)
  // 3: Minimum step size reached (ACADOS_MINSTEP)
  // 4: QP solver failed (ACADOS_QP_FAILURE)
  // 5: Solver created (ACADOS_READY)
  // 6: Problem unbounded (ACADOS_UNBOUNDED)
  // 7: Solver timeout (ACADOS_TIMEOUT)
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
    trajectory_optimization::acados_print_stats(ocp_capsule_);
  }
}

}  // namespace trajectory_optimization

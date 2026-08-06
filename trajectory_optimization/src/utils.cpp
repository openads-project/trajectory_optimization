// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <trajectory_optimization/trajectory_optimization_node.hpp>

#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

namespace trajectory_optimization {

double TrajectoryOptimizationNode::wrap_angle_rad(double angle_rad, double min_val, double max_val) {
  double capped_angle_rad = angle_rad;
  while (capped_angle_rad > max_val) capped_angle_rad -= 2 * M_PI;
  while (capped_angle_rad < min_val) capped_angle_rad += 2 * M_PI;
  return capped_angle_rad;
}

bool TrajectoryOptimizationNode::linearInterpolation(
    const std::vector<double>& X, const std::vector<double>& Y, const double& desired_x, double& output_y, bool wrap_angle) {
  if (desired_x == X.front()) {
    RCLCPP_DEBUG(get_logger(), "Desired Time is equal to Time-Min of the given vector!");
    output_y = Y.front();
    return true;
  } else if (desired_x == X.back()) {
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
  size_t i = 0;
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
  double diff = Y[i] - Y[i - 1];
  if (wrap_angle) {
    diff = wrap_angle_rad(diff);
  }
  output_y = Y[i - 1] + (diff / (X[i] - X[i - 1])) * (desired_x - X[i - 1]);
  if (wrap_angle) {
    output_y = wrap_angle_rad(output_y);
  }
  return true;
}

bool TrajectoryOptimizationNode::trajectory2outputFrame(trajectory_planning_msgs::msg::Trajectory& trajectory) {
  if (trajectory_frame_id_ != vehicle_frame_id_) {
    trajectory_planning_msgs::msg::Trajectory tf_trajectory;
    try {
      tf_trajectory = tf2_buffer_->transform(trajectory, trajectory_frame_id_, tf2::durationFromSec(0.01));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Transformation into output frame is not available. Publishing no trajectory. Ex: %s",
                  ex.what());
      return false;
    }
    trajectory = tf_trajectory;
  }
  return true;
}

void TrajectoryOptimizationNode::keepNClosestObjects(perception_msgs::msg::ObjectList& object_list, const int n_objects) {
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
  const auto n_objects_to_keep = std::min(static_cast<size_t>(n_objects), indices_sorted_by_distance.size());
  int i = 0;
  while (closest_objects.size() < static_cast<size_t>(n_objects_to_keep)) {
    if (static_cast<size_t>(i) >= indices_sorted_by_distance.size()) break;
    // ignore object with negative x-coordinate (behind the ego vehicle)
    if (perception_msgs::object_access::getX(object_list.objects[indices_sorted_by_distance[i]]) > 0.0) {
      closest_objects.push_back(object_list.objects[indices_sorted_by_distance[i]]);
    }
    ++i;
  }
  object_list.objects = closest_objects;
}

std::vector<double> TrajectoryOptimizationNode::discretizeBB2Circles(
    const double x, const double y, const double yaw, const double length, const double width) {
  uint8_t n_circles = 1;
  if (length <= 0.0 || width <= 0.0) {
    RCLCPP_WARN(get_logger(), "Invalid bounding box dimensions: length = %f, width = %f. Setting n_circles = 1.", length, width);
  } else {
    double aspect_ratio = length / width;
    if (aspect_ratio > 8.0) {
      n_circles = 9;
    } else if (aspect_ratio > 6.0) {
      n_circles = 7;
    } else if (aspect_ratio > 4.0) {
      n_circles = 5;
    } else if (aspect_ratio > 1.8) {
      n_circles = 3;
    } else if (aspect_ratio > 1.3) {
      n_circles = 2;
    } else {
      n_circles = 1;
    }
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

std::vector<std::pair<double, double>> TrajectoryOptimizationNode::normalBoundaryDistance(
    const trajectory_planning_msgs::msg::Trajectory& reference_trajectory, const route_planning_msgs::msg::Route& route) {
  const double NO_BOUNDARY_DISTANCE = 1e6;  // finite sentinel avoids NaNs during reference-path interpolation
  constexpr double MAX_ROUTE_S_DIFFERENCE = 20.0;
  constexpr double LOOK_BEHIND_REMAINING_ROUTE_S = 2.0;

  struct Boundaries {
    std::vector<std::pair<double, double>> min_normal_distances;
    std::vector<Eigen::Vector2d> left_boundary_points;
    std::vector<Eigen::Vector2d> right_boundary_points;
    std::vector<double> left_boundary_route_s;
    std::vector<double> right_boundary_route_s;
    std::vector<Eigen::Vector2d> left_boundary_intersections;
    std::vector<Eigen::Vector2d> right_boundary_intersections;
  };

  Boundaries boundaries;
  auto route_elements = route_planning_msgs::route_access::getRemainingRouteElements(route, true);
  const int ref_sample_size = trajectory_planning_msgs::trajectory_access::getSamplePointSize(reference_trajectory);

  if (route_elements.empty()) {
    if (consider_boundaries_ != CONSIDER_BOUNDARIES::NO_BOUNDS) {
      RCLCPP_WARN(get_logger(), "Remaining route is empty. Do not constrain boundaries.");
    }
    for (int i = 0; i < ref_sample_size; ++i) {
      boundaries.min_normal_distances.emplace_back(NO_BOUNDARY_DISTANCE, NO_BOUNDARY_DISTANCE);
    }
    return boundaries.min_normal_distances;
  }

  const double current_route_s = route_elements.front().s;
  const size_t current_route_index = std::min(static_cast<size_t>(route.current_route_element_idx), route.route_elements.size());
  size_t overlap_begin = current_route_index;
  while (overlap_begin > 0 && current_route_s - route.route_elements[overlap_begin - 1].s <= LOOK_BEHIND_REMAINING_ROUTE_S) {
    --overlap_begin;
  }
  route_elements.insert(route_elements.begin(), route.route_elements.begin() + static_cast<std::ptrdiff_t>(overlap_begin),
                        route.route_elements.begin() + static_cast<std::ptrdiff_t>(current_route_index));

  boundaries.left_boundary_points.reserve(route_elements.size());
  boundaries.right_boundary_points.reserve(route_elements.size());
  boundaries.left_boundary_route_s.reserve(route_elements.size());
  boundaries.right_boundary_route_s.reserve(route_elements.size());
  boundaries.min_normal_distances.reserve(ref_sample_size);
  boundaries.left_boundary_intersections.reserve(ref_sample_size);
  boundaries.right_boundary_intersections.reserve(ref_sample_size);

  for (const auto& route_element : route_elements) {
    if (route_element.is_enriched) {
      if (consider_boundaries_ == CONSIDER_BOUNDARIES::SUGGESTED_LANE) {
        route_planning_msgs::msg::LaneElement suggested_lane =
            route_planning_msgs::route_access::getSuggestedLaneElement(route_element);
        boundaries.left_boundary_points.emplace_back(suggested_lane.left_boundary.point.x, suggested_lane.left_boundary.point.y);
        boundaries.right_boundary_points.emplace_back(suggested_lane.right_boundary.point.x,
                                                      suggested_lane.right_boundary.point.y);
        boundaries.left_boundary_route_s.push_back(route_element.s);
        boundaries.right_boundary_route_s.push_back(route_element.s);
      } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::INCLUDING_ADJACENT) {
        const auto& lane_elements = route_element.lane_elements;
        if (!lane_elements.empty()) {
          boundaries.left_boundary_points.emplace_back(lane_elements.front().left_boundary.point.x,
                                                       lane_elements.front().left_boundary.point.y);
          boundaries.right_boundary_points.emplace_back(lane_elements.back().right_boundary.point.x,
                                                        lane_elements.back().right_boundary.point.y);
          boundaries.left_boundary_route_s.push_back(route_element.s);
          boundaries.right_boundary_route_s.push_back(route_element.s);
        }
      } else if (consider_boundaries_ == CONSIDER_BOUNDARIES::DRIVABLE_SPACE) {
        boundaries.left_boundary_points.emplace_back(route_element.left_boundary.x, route_element.left_boundary.y);
        boundaries.right_boundary_points.emplace_back(route_element.right_boundary.x, route_element.right_boundary.y);
        boundaries.left_boundary_route_s.push_back(route_element.s);
        boundaries.right_boundary_route_s.push_back(route_element.s);
      }
    }
  }

  // Helper lambda to find intersection
  auto findIntersection = [&](const Eigen::Vector2d& ref_pos, double sin_yaw, double cos_yaw, double expected_route_s,
                              const std::vector<Eigen::Vector2d>& boundary_points, const std::vector<double>& boundary_route_s,
                              bool isLeft) -> std::pair<double, Eigen::Vector2d> {
    std::pair<double, Eigen::Vector2d> intersection_result = {
        std::numeric_limits<double>::infinity(),
        Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity())};
    double best_route_s_difference = std::numeric_limits<double>::infinity();

    const Eigen::Vector2d normal_dir = isLeft ? Eigen::Vector2d(-sin_yaw, cos_yaw) : Eigen::Vector2d(sin_yaw, -cos_yaw);
    const auto cross2d = [](const Eigen::Vector2d& u, const Eigen::Vector2d& v) { return u.x() * v.y() - u.y() * v.x(); };

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
        double euclidean_distance = (ref_pos - intersection).norm();
        const double intersection_route_s = boundary_route_s[i] + s * (boundary_route_s[i + 1] - boundary_route_s[i]);
        const double route_s_difference = std::abs(intersection_route_s - expected_route_s);

        if (route_s_difference <= MAX_ROUTE_S_DIFFERENCE &&
            (route_s_difference < best_route_s_difference ||
             (std::abs(route_s_difference - best_route_s_difference) < 1e-9 && euclidean_distance < intersection_result.first))) {
          best_route_s_difference = route_s_difference;
          intersection_result = {euclidean_distance, intersection};
        }
      }
    }
    return intersection_result;
  };

  // Loop over trajectory points and compute intersections
  double reference_progress = 0.0;
  Eigen::Vector2d previous_ref_pos = Eigen::Vector2d::Zero();
  for (int i = 0; i < ref_sample_size; ++i) {
    Eigen::Vector2d ref_pos(trajectory_planning_msgs::trajectory_access::getX(reference_trajectory, i),
                            trajectory_planning_msgs::trajectory_access::getY(reference_trajectory, i));
    reference_progress += (ref_pos - previous_ref_pos).norm();
    previous_ref_pos = ref_pos;
    const double expected_route_s = current_route_s + reference_progress;

    double yaw = trajectory_planning_msgs::trajectory_access::getTheta(reference_trajectory, i);
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);

    auto left_intersection = findIntersection(ref_pos, sin_yaw, cos_yaw, expected_route_s, boundaries.left_boundary_points,
                                              boundaries.left_boundary_route_s, true);
    auto right_intersection = findIntersection(ref_pos, sin_yaw, cos_yaw, expected_route_s, boundaries.right_boundary_points,
                                               boundaries.right_boundary_route_s, false);
    if (left_intersection.first != std::numeric_limits<double>::infinity() &&
        right_intersection.first != std::numeric_limits<double>::infinity()) {
      boundaries.min_normal_distances.emplace_back(left_intersection.first, right_intersection.first);
      boundaries.left_boundary_intersections.emplace_back(left_intersection.second);
      boundaries.right_boundary_intersections.emplace_back(right_intersection.second);
      RCLCPP_DEBUG(this->get_logger(), "Minimum left boundary distance: %.2f m", left_intersection.first);
      RCLCPP_DEBUG(this->get_logger(), "Minimum right boundary distance: %.2f m", right_intersection.first);
    } else {
      boundaries.min_normal_distances.emplace_back(NO_BOUNDARY_DISTANCE, NO_BOUNDARY_DISTANCE);
      RCLCPP_WARN(get_logger(),
                  "No boundary intersection found for trajectory point %d. Do not constrain boundaries at this point.", i);
    }
  }
  if (debug_viz_) {
    vizBoundaryPoints(boundaries.left_boundary_intersections, boundaries.right_boundary_intersections);
  }

  return boundaries.min_normal_distances;
}

void TrajectoryOptimizationNode::vizBoundaryPoints(const std::vector<Eigen::Vector2d>& left_boundary_points,
                                                   const std::vector<Eigen::Vector2d>& right_boundary_points) {
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

  addMarkers(left_boundary_points, "left_boundary_constraint", 0.0F, 1.0F, 0.0F);
  addMarkers(right_boundary_points, "right_boundary_constraint", 1.0F, 0.0F, 0.0F);

  boundary_pub_->publish(marker_array);
}

void TrajectoryOptimizationNode::vizCircles(const std::vector<double>& obstacles) {
  visualization_msgs::msg::MarkerArray marker_array;
  const int n_circles = static_cast<int>(obstacles.size() / static_cast<size_t>(p_obstacle_circles_shape_[1]));
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

void TrajectoryOptimizationNode::vizObbs(const std::vector<double>& obstacles) {
  visualization_msgs::msg::MarkerArray marker_array;
  const size_t width = static_cast<size_t>(p_obstacle_obbs_shape_[1]);
  const size_t box_count = obstacles.size() / width;
  for (size_t index = 0; index < box_count; ++index) {
    const size_t offset = index * width;
    if (std::abs(obstacles[offset]) > 5000.0 || std::abs(obstacles[offset + 1]) > 5000.0) continue;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = vehicle_frame_id_;
    marker.header.stamp = rclcpp::Time(ego_data_.header.stamp);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "obstacle-obbs";
    marker.id = static_cast<int>(index);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = obstacles[offset];
    marker.pose.position.y = obstacles[offset + 1];
    const double half_yaw = 0.5 * obstacles[offset + 2];
    marker.pose.orientation.z = std::sin(half_yaw);
    marker.pose.orientation.w = std::cos(half_yaw);
    marker.scale.x = 2.0 * obstacles[offset + 3];
    marker.scale.y = 2.0 * obstacles[offset + 4];
    marker.scale.z = 0.1;
    marker.color.a = 0.25F;
    marker.color.r = 1.0F;
    marker_array.markers.push_back(marker);
  }
  circles_pub_->publish(marker_array);
}

void TrajectoryOptimizationNode::vizEgoCircles(const std::vector<double>& x_trajectory, const std::string& model_name) {
  if (!ego_circles_pub_) return;

  double ego_length = 0.0, ego_width = 0.0;
  int n_ego_circles = 0;
  std::vector<double> ego_offset2geocenter;

  // define vehicle geometry based on model name (should match the OCP definition)
  if (model_name == "karl") {
    ego_length = 5.173;
    ego_width = 1.94;
    ego_offset2geocenter = {1.4895, 0.0};
    n_ego_circles = 5;
  } else if (model_name == "shuttle") {
    ego_length = 4.97;
    ego_width = 2.12;
    ego_offset2geocenter = {0.0, 0.0};
    n_ego_circles = 3;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown model '%s'. Could not visualize ego circles.", model_name.c_str());
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;

  if (x_trajectory.empty() || n_ego_circles <= 0) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    ego_circles_pub_->publish(marker_array);
    return;
  }

  const double offset_x = !ego_offset2geocenter.empty() ? ego_offset2geocenter[0] : 0.0;
  const double offset_y = ego_offset2geocenter.size() > 1 ? ego_offset2geocenter[1] : 0.0;

  const double radius = std::sqrt(std::pow(ego_length / (2.0 * n_ego_circles), 2) + std::pow(ego_width / 2.0, 2));
  const int state_dim = *nlp_dims_->nx;

  int marker_id = 0;
  for (int stage = 0; stage <= n_shots_; ++stage) {
    const size_t state_offset = static_cast<size_t>(stage) * static_cast<size_t>(state_dim);
    const double base_x = x_trajectory[state_offset + 0];
    const double base_y = x_trajectory[state_offset + 1];
    const double psi = x_trajectory[state_offset + 5];

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
      marker.color.r = 0.0F;
      marker.color.g = 0.6F;
      marker.color.b = 1.0F;
      marker_array.markers.push_back(marker);
    }
  }

  ego_circles_pub_->publish(marker_array);
}

void TrajectoryOptimizationNode::vizEgoObbs(const std::vector<double>& x_trajectory, const std::string& model_name) {
  visualization_msgs::msg::MarkerArray marker_array;
  if (x_trajectory.empty()) {
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    ego_circles_pub_->publish(marker_array);
    return;
  }

  const auto geometry = vehicleGeometry(model_name);
  const int state_dim = *nlp_dims_->nx;
  for (int stage = 0; stage <= n_shots_; ++stage) {
    const size_t offset = static_cast<size_t>(stage) * static_cast<size_t>(state_dim);
    const auto box =
        orientedBoxFromReference(x_trajectory[offset], x_trajectory[offset + 1], x_trajectory[offset + 5], geometry.length,
                                 geometry.width, geometry.center_offset_long, geometry.center_offset_lat);
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = vehicle_frame_id_;
    marker.header.stamp = rclcpp::Time(ego_data_.header.stamp);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.ns = "ego-obbs";
    marker.id = stage;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = box.x;
    marker.pose.position.y = box.y;
    marker.pose.orientation.z = std::sin(0.5 * box.yaw);
    marker.pose.orientation.w = std::cos(0.5 * box.yaw);
    marker.scale.x = 2.0 * box.half_length;
    marker.scale.y = 2.0 * box.half_width;
    marker.scale.z = 0.05;
    marker.color.a = 0.35F;
    marker.color.g = 0.6F;
    marker.color.b = 1.0F;
    marker_array.markers.push_back(marker);
  }
  ego_circles_pub_->publish(marker_array);
}

void TrajectoryOptimizationNode::printSolution(const PerformanceMetrics& metrics) {
  // Status codes:
  // 0: Success (ACADOS_SUCCESS)
  // 1: NaN detected (ACADOS_NAN_DETECTED)
  // 2: Maximum number of iterations reached (ACADOS_MAXITER)
  // 3: Minimum step size reached (ACADOS_MINSTEP)
  // 4: QP solver failed (ACADOS_QP_FAILURE)
  // 5: Solver created (ACADOS_READY)
  // 6: Problem unbounded (ACADOS_UNBOUNDED)
  // 7: Solver timeout (ACADOS_TIMEOUT)
  if (metrics.status == ACADOS_SUCCESS && verbose_) {
    RCLCPP_INFO(get_logger(), "\033[1;32mOptimization: SUCCESS!\033[0m");
  } else if (metrics.status == ACADOS_MAXITER) {
    RCLCPP_WARN(get_logger(), "Optimization failed with status %d (max iterations).", metrics.status);
  } else if (metrics.status == ACADOS_TIMEOUT) {
    RCLCPP_WARN(get_logger(), "\033[38;5;214mOptimization failed with status %d (timeout).\033[0m", metrics.status);
  } else if (metrics.status != ACADOS_SUCCESS) {
    RCLCPP_ERROR(get_logger(), "%s_acados_solve() failed with status %d.", model_name_.c_str(), metrics.status);
  }

  if (verbose_) {
    RCLCPP_INFO(get_logger(), "Optimization took %.3f ms (SQP iter: %d; QP iter: %d; KKT: %e)", metrics.acados_total_ms,
                metrics.sqp_iter, metrics.qp_iter, metrics.kkt_norm_inf);
    RCLCPP_INFO(get_logger(), "cost_value: %f; residuals: stat=%e eq=%e ineq=%e comp=%e", metrics.cost_value, metrics.res_stat,
                metrics.res_eq, metrics.res_ineq, metrics.res_comp);

    std::fputs("\n--- xtraj ---\n", stdout);
    d_print_exp_tran_mat(*nlp_dims_->nx, n_shots_ + 1, xtraj_.data(), *nlp_dims_->nx);
    std::fputs("\n--- utraj ---\n", stdout);
    d_print_exp_tran_mat(*nlp_dims_->nu, n_shots_, utraj_.data(), *nlp_dims_->nu);
    trajectory_optimization::acados_print_stats(ocp_capsule_);
  }
}

}  // namespace trajectory_optimization

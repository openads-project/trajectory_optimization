// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>

#include <trajectory_optimization/collision_geometry.hpp>

namespace trajectory_optimization {

namespace {

using Vector = Point2d;

double dot(const Vector& first, const Vector& second) { return first[0] * second[0] + first[1] * second[1]; }

std::array<Vector, 2> axes(const OrientedBox& box) {
  return {{{std::cos(box.yaw), std::sin(box.yaw)}, {-std::sin(box.yaw), std::cos(box.yaw)}}};
}

double smoothAbsUpper(double value, double epsilon) { return std::hypot(value, epsilon); }

double smoothAbsLower(double value, double epsilon) { return std::hypot(value, epsilon) - epsilon; }

double smoothMaxLower(double first, double second, double tau) {
  return 0.5 * (first + second + std::hypot(first - second, tau) - tau);
}

double support(const OrientedBox& box, const Vector& axis, bool smooth, double epsilon) {
  const auto box_axes = axes(box);
  const auto absolute = [smooth, epsilon](double value) { return smooth ? smoothAbsUpper(value, epsilon) : std::abs(value); };
  return box.half_length * absolute(dot(box_axes[0], axis)) + box.half_width * absolute(dot(box_axes[1], axis));
}

}  // namespace

VehicleGeometry vehicleGeometry(const std::string& model_name) {
  if (model_name == "karl") return {5.173, 1.94, 1.4895, 0.0, 5};
  if (model_name == "shuttle") return {4.97, 2.12, 0.0, 0.0, 3};
  throw std::invalid_argument("Unknown vehicle model: " + model_name);
}

OrientedBox orientedBoxFromReference(
    double x, double y, double yaw, double length, double width, double center_offset_long, double center_offset_lat) {
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  return {x + center_offset_long * cos_yaw - center_offset_lat * sin_yaw,
          y + center_offset_long * sin_yaw + center_offset_lat * cos_yaw, yaw, std::max(0.0, 0.5 * length),
          std::max(0.0, 0.5 * width)};
}

OrientedBox expandBoxForward(const OrientedBox& box, double front_margin, double rear_margin, double lateral_margin) {
  front_margin = std::max(0.0, front_margin);
  rear_margin = std::max(0.0, rear_margin);
  lateral_margin = std::max(0.0, lateral_margin);
  const double center_shift = 0.5 * (front_margin - rear_margin);
  return {box.x + center_shift * std::cos(box.yaw), box.y + center_shift * std::sin(box.yaw), box.yaw,
          box.half_length + 0.5 * (front_margin + rear_margin), box.half_width + lateral_margin};
}

double exactSatSeparationMargin(const OrientedBox& first, const OrientedBox& second) {
  const auto first_axes = axes(first);
  const auto second_axes = axes(second);
  const std::array<Vector, 4> separating_axes = {first_axes[0], first_axes[1], second_axes[0], second_axes[1]};
  const Vector center_difference = {second.x - first.x, second.y - first.y};
  double maximum_gap = -std::numeric_limits<double>::infinity();
  for (const auto& axis : separating_axes) {
    const double gap =
        std::abs(dot(center_difference, axis)) - support(first, axis, false, 0.0) - support(second, axis, false, 0.0);
    maximum_gap = std::max(maximum_gap, gap);
  }
  return maximum_gap;
}

double conservativeSmoothSatMargin(const OrientedBox& first, const OrientedBox& second, double epsilon, double tau) {
  if (epsilon <= 0.0 || tau <= 0.0) throw std::invalid_argument("OBB smoothing parameters must be positive");
  const auto first_axes = axes(first);
  const auto second_axes = axes(second);
  const std::array<Vector, 4> separating_axes = {first_axes[0], first_axes[1], second_axes[0], second_axes[1]};
  const Vector center_difference = {second.x - first.x, second.y - first.y};
  std::vector<double> gaps;
  gaps.reserve(separating_axes.size());
  for (const auto& axis : separating_axes) {
    gaps.push_back(smoothAbsLower(dot(center_difference, axis), epsilon) - support(first, axis, true, epsilon) -
                   support(second, axis, true, epsilon));
  }
  return smoothMaxLower(smoothMaxLower(gaps[0], gaps[1], tau), smoothMaxLower(gaps[2], gaps[3], tau), tau);
}

double boxSupport(const OrientedBox& box, double normal_x, double normal_y) {
  return support(box, {normal_x, normal_y}, false, 0.0);
}

std::array<Point2d, 4> boxCorners(const OrientedBox& box) {
  const auto box_axes = axes(box);
  const auto corner = [&](double longitudinal, double lateral) {
    return Point2d{box.x + longitudinal * box_axes[0][0] + lateral * box_axes[1][0],
                   box.y + longitudinal * box_axes[0][1] + lateral * box_axes[1][1]};
  };
  return {corner(-box.half_length, -box.half_width), corner(-box.half_length, box.half_width),
          corner(box.half_length, -box.half_width), corner(box.half_length, box.half_width)};
}

OrientedBox interpolateBox(const OrientedBox& first, const OrientedBox& second, double factor) {
  factor = std::clamp(factor, 0.0, 1.0);
  double yaw_difference = std::remainder(second.yaw - first.yaw, 2.0 * M_PI);
  return {first.x + factor * (second.x - first.x), first.y + factor * (second.y - first.y),
          std::remainder(first.yaw + factor * yaw_difference, 2.0 * M_PI),
          first.half_length + factor * (second.half_length - first.half_length),
          first.half_width + factor * (second.half_width - first.half_width)};
}

double boxBoundaryViolationDepth(const OrientedBox& box, const std::vector<Point2d>& boundary, bool left_boundary) {
  if (boundary.size() < 2) return -std::numeric_limits<double>::infinity();
  constexpr double TOLERANCE = 1e-9;
  double maximum_violation = -std::numeric_limits<double>::infinity();
  for (const auto& corner : boxCorners(box)) {
    double best_distance_squared = std::numeric_limits<double>::infinity();
    double best_signed_distance = 0.0;
    for (size_t index = 0; index + 1 < boundary.size(); ++index) {
      const Vector segment = {boundary[index + 1][0] - boundary[index][0], boundary[index + 1][1] - boundary[index][1]};
      const double segment_length_squared = dot(segment, segment);
      if (segment_length_squared <= TOLERANCE) continue;
      const Vector relative = {corner[0] - boundary[index][0], corner[1] - boundary[index][1]};
      const double factor = std::clamp(dot(relative, segment) / segment_length_squared, 0.0, 1.0);
      const Vector nearest = {boundary[index][0] + factor * segment[0], boundary[index][1] + factor * segment[1]};
      const double dx = corner[0] - nearest[0];
      const double dy = corner[1] - nearest[1];
      const double distance_squared = dx * dx + dy * dy;
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_signed_distance = (segment[0] * relative[1] - segment[1] * relative[0]) / std::sqrt(segment_length_squared);
      }
    }
    const double violation = left_boundary ? best_signed_distance : -best_signed_distance;
    maximum_violation = std::max(maximum_violation, violation);
  }
  return maximum_violation;
}

bool boxOutsideBoundary(const OrientedBox& box, const std::vector<Point2d>& boundary, bool left_boundary) {
  constexpr double TOLERANCE = 1e-9;
  return boxBoundaryViolationDepth(box, boundary, left_boundary) > TOLERANCE;
}

double interpolateSample(const std::vector<double>& times,
                         const std::vector<double>& values,
                         double desired_time,
                         bool wrap_angle) {
  if (times.empty() || times.size() != values.size()) throw std::invalid_argument("Invalid interpolation samples");
  if (desired_time <= times.front()) return values.front();
  if (desired_time >= times.back()) return values.back();
  const auto upper = std::upper_bound(times.begin(), times.end(), desired_time);
  const size_t upper_index = static_cast<size_t>(std::distance(times.begin(), upper));
  const size_t lower_index = upper_index - 1;
  const double duration = times[upper_index] - times[lower_index];
  if (duration <= 0.0) return values[upper_index];
  double difference = values[upper_index] - values[lower_index];
  if (wrap_angle) difference = std::remainder(difference, 2.0 * M_PI);
  const double result = values[lower_index] + (desired_time - times[lower_index]) / duration * difference;
  return wrap_angle ? std::remainder(result, 2.0 * M_PI) : result;
}

std::vector<size_t> rankHypotheses(const std::vector<HypothesisPriority>& priorities) {
  std::vector<size_t> indices(priorities.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::stable_sort(indices.begin(), indices.end(), [&](size_t first, size_t second) {
    const auto& first_priority = priorities[first];
    const auto& second_priority = priorities[second];
    return std::tie(first_priority.minimum_reference_gap, first_priority.earliest_minimum_stage, first_priority.object_id,
                    first_priority.prediction_index) < std::tie(second_priority.minimum_reference_gap,
                                                                second_priority.earliest_minimum_stage, second_priority.object_id,
                                                                second_priority.prediction_index);
  });
  return indices;
}

}  // namespace trajectory_optimization

// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <tuple>

#include <trajectory_optimization/collision_geometry.hpp>

namespace trajectory_optimization {
namespace {

using Vector = std::array<double, 2>;

double dot(const Vector& first, const Vector& second) { return first[0] * second[0] + first[1] * second[1]; }

std::array<Vector, 2> axes(const OrientedBox& box) {
  return {{{std::cos(box.yaw), std::sin(box.yaw)}, {-std::sin(box.yaw), std::cos(box.yaw)}}};
}

double support(const OrientedBox& box, const Vector& axis) {
  const auto box_axes = axes(box);
  return box.half_length * std::abs(dot(box_axes[0], axis)) + box.half_width * std::abs(dot(box_axes[1], axis));
}

}  // namespace

VehicleGeometry vehicleGeometry(const std::string& model_name) {
  if (model_name == "karl") return {5.173, 1.94, 1.4895, 0.0};
  if (model_name == "shuttle") return {4.97, 2.12, 0.0, 0.0};
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
    maximum_gap = std::max(maximum_gap, std::abs(dot(center_difference, axis)) - support(first, axis) - support(second, axis));
  }
  return maximum_gap;
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
    const auto& lhs = priorities[first];
    const auto& rhs = priorities[second];
    return std::tie(lhs.minimum_reference_gap, lhs.earliest_minimum_stage, lhs.object_id, lhs.prediction_index) <
           std::tie(rhs.minimum_reference_gap, rhs.earliest_minimum_stage, rhs.object_id, rhs.prediction_index);
  });
  return indices;
}

}  // namespace trajectory_optimization

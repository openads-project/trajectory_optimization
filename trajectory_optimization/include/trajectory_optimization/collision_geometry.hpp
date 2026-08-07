// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace trajectory_optimization {

struct VehicleGeometry {
  double length;
  double width;
  double center_offset_long;
  double center_offset_lat;
  int circle_count;
};

struct OrientedBox {
  double x;
  double y;
  double yaw;
  double half_length;
  double half_width;
};

using Point2d = std::array<double, 2>;

struct HypothesisPriority {
  double minimum_reference_gap;
  int earliest_minimum_stage;
  uint64_t object_id;
  int prediction_index;
};

VehicleGeometry vehicleGeometry(const std::string& model_name);

OrientedBox orientedBoxFromReference(
    double x, double y, double yaw, double length, double width, double center_offset_long = 0.0, double center_offset_lat = 0.0);

OrientedBox expandBoxForward(const OrientedBox& box, double front_margin, double rear_margin, double lateral_margin);

double exactSatSeparationMargin(const OrientedBox& first, const OrientedBox& second);

double conservativeSmoothSatMargin(const OrientedBox& first, const OrientedBox& second, double epsilon = 1e-3, double tau = 0.02);

double boxSupport(const OrientedBox& box, double normal_x, double normal_y);

std::array<Point2d, 4> boxCorners(const OrientedBox& box);

OrientedBox interpolateBox(const OrientedBox& first, const OrientedBox& second, double factor);

bool boxOutsideBoundary(const OrientedBox& box, const std::vector<Point2d>& boundary, bool left_boundary);

/**
 * @brief Returns the largest signed penetration of an OBB beyond a boundary polyline.
 *
 * Positive values denote a violation, zero contact, and negative values clearance. The
 * boundary is expected to be ordered in driving direction.
 */
double boxBoundaryViolationDepth(const OrientedBox& box, const std::vector<Point2d>& boundary, bool left_boundary);

double interpolateSample(const std::vector<double>& times,
                         const std::vector<double>& values,
                         double desired_time,
                         bool wrap_angle = false);

std::vector<size_t> rankHypotheses(const std::vector<HypothesisPriority>& priorities);

}  // namespace trajectory_optimization

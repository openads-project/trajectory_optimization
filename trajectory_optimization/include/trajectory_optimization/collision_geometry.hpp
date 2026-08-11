// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace trajectory_optimization {

struct VehicleGeometry {
  double length;
  double width;
  double center_offset_long;
  double center_offset_lat;
};

struct OrientedBox {
  double x;
  double y;
  double yaw;
  double half_length;
  double half_width;
};

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

double interpolateSample(const std::vector<double>& times,
                         const std::vector<double>& values,
                         double desired_time,
                         bool wrap_angle = false);

std::vector<size_t> rankHypotheses(const std::vector<HypothesisPriority>& priorities);

}  // namespace trajectory_optimization

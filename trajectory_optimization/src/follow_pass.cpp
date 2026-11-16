// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include <trajectory_optimization/follow_pass.hpp>

namespace trajectory_optimization {

namespace {

constexpr double INF = std::numeric_limits<double>::infinity();

using Point = std::array<double, 2>;

/** Corners of an oriented box: front left, front right, rear right, rear left. */
std::array<Point, 4> boxCorners(double x, double y, double yaw, double half_length, double half_width) {
  constexpr std::array<std::array<double, 2>, 4> SIGNS{{{1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}}};
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  std::array<Point, 4> corners{};
  for (size_t i = 0; i < corners.size(); ++i) {
    const double l = SIGNS[i][0] * half_length;
    const double w = SIGNS[i][1] * half_width;
    corners[i] = {x + l * cos_yaw - w * sin_yaw, y + l * sin_yaw + w * cos_yaw};
  }
  return corners;
}

/** Position along (arc length s) and lateral offset from (e, left positive) the reference path. */
struct Projection {
  double s = 0.0;
  double e = 0.0;
};

/** Reference polyline, extended by rays along the first and last heading; s = 0 at the first sample. */
class Path {
 public:
  explicit Path(const FollowPass::Reference& reference) {
    constexpr double MIN_SAMPLE_DISTANCE = 1e-6;  // repeated samples (padding, standstill) carry no direction
    for (size_t i = 0; i < reference.x.size(); ++i) {
      const Point point{reference.x[i], reference.y[i]};
      const double distance = points_.empty() ? INF : std::hypot(point[0] - points_.back()[0], point[1] - points_.back()[1]);
      if (distance > MIN_SAMPLE_DISTANCE) {
        s_points_.push_back(points_.empty() ? 0.0 : s_points_.back() + distance);
        points_.push_back(point);
        distinct_samples_.push_back(i);
      }
      s_samples_.push_back(s_points_.back());
    }
    first_direction_ = {std::cos(reference.psi[distinct_samples_.front()]), std::sin(reference.psi[distinct_samples_.front()])};
    last_direction_ = {std::cos(reference.psi[distinct_samples_.back()]), std::sin(reference.psi[distinct_samples_.back()])};
  }

  /** Arc length of every reference sample. */
  const std::vector<double>& sampleArcLengths() const { return s_samples_; }

  Projection project(const Point& point) const {
    double best_distance = INF;
    Projection projection;
    const auto consider = [&](const Point& start, const Point& direction, double s_start, double lambda_min, double lambda_max) {
      const double dx = point[0] - start[0];
      const double dy = point[1] - start[1];
      const double lambda = std::clamp(dx * direction[0] + dy * direction[1], lambda_min, lambda_max);
      const double distance = std::hypot(dx - lambda * direction[0], dy - lambda * direction[1]);
      if (distance < best_distance) {
        best_distance = distance;
        projection = {s_start + lambda, direction[0] * dy - direction[1] * dx};
      }
    };
    consider(points_.front(), first_direction_, 0.0, -INF, 0.0);
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const double length = s_points_[i + 1] - s_points_[i];
      const Point direction{(points_[i + 1][0] - points_[i][0]) / length, (points_[i + 1][1] - points_[i][1]) / length};
      consider(points_[i], direction, s_points_[i], 0.0, length);
    }
    consider(points_.back(), last_direction_, s_points_.back(), 0.0, INF);
    return projection;
  }

  /** Interpolates per-sample values at arc length s; constant beyond the reference ends. */
  double at(double s, const std::vector<double>& sample_values) const {
    if (s <= s_points_.front()) return sample_values[distinct_samples_.front()];
    if (s >= s_points_.back()) return sample_values[distinct_samples_.back()];
    const size_t upper = static_cast<size_t>(std::upper_bound(s_points_.begin(), s_points_.end(), s) - s_points_.begin());
    const double weight = (s - s_points_[upper - 1]) / (s_points_[upper] - s_points_[upper - 1]);
    const double lower_value = sample_values[distinct_samples_[upper - 1]];
    return lower_value + weight * (sample_values[distinct_samples_[upper]] - lower_value);
  }

 private:
  std::vector<Point> points_;             // distinct samples
  std::vector<double> s_points_;          // arc length of the distinct samples
  std::vector<size_t> distinct_samples_;  // sample index of every distinct sample
  std::vector<double> s_samples_;         // arc length of every sample
  Point first_direction_{};
  Point last_direction_{};
};

/**
 * @brief Arc length of the foremost front ego corner as the OCP evaluates it: linearised at the reference point matched
 * to the ego center (same matching as determine_spacially_matched_ref_path_point in the OCP).
 */
double ocpFrontArcLength(const FollowPass::Reference& reference,
                         const std::vector<double>& s_samples,
                         const Point& center,
                         double yaw,
                         const FollowPass::Params& params) {
  const size_t n = reference.x.size();
  std::vector<double> distances(n);
  for (size_t i = 0; i < n; ++i) distances[i] = std::hypot(reference.x[i] - center[0], reference.y[i] - center[1]);
  const size_t i = static_cast<size_t>(std::min_element(distances.begin(), distances.end()) - distances.begin());
  size_t j = i;
  if (n > 1) j = i == 0 ? 1 : (i == n - 1 ? n - 2 : (distances[i - 1] < distances[i + 1] ? i - 1 : i + 1));
  const double segment_x = reference.x[j] - reference.x[i];
  const double segment_y = reference.y[j] - reference.y[i];
  const double segment_squared = segment_x * segment_x + segment_y * segment_y;
  double lambda = segment_squared == 0.0
                      ? 0.0
                      : ((center[0] - reference.x[i]) * segment_x + (center[1] - reference.y[i]) * segment_y) / segment_squared;
  if (i != 0) lambda = std::clamp(lambda, 0.0, 1.0);  // the OCP extrapolates only before the first sample
  const Point matched{reference.x[i] + lambda * segment_x, reference.y[i] + lambda * segment_y};
  const double s_matched = s_samples[i] + lambda * (s_samples[j] - s_samples[i]);
  const double psi_difference = std::remainder(reference.psi[j] - reference.psi[i], 2.0 * M_PI);
  const double psi_matched = reference.psi[i] + std::clamp(lambda, 0.0, 1.0) * psi_difference;
  const auto corners = boxCorners(center[0], center[1], yaw, params.ego_half_length, params.ego_half_width);
  double s_front = -INF;
  for (size_t corner = 0; corner < 2; ++corner) {
    const double along =
        (corners[corner][0] - matched[0]) * std::cos(psi_matched) + (corners[corner][1] - matched[1]) * std::sin(psi_matched);
    s_front = std::max(s_front, s_matched + along);
  }
  return s_front;
}

}  // namespace

FollowPass::Result FollowPass::decide(const Reference& reference,
                                      const std::vector<std::vector<Box>>& boxes,
                                      const std::vector<Slot>& slots,
                                      const std::array<double, 3>& ego_pose,
                                      const std::vector<double>& braking_travel,
                                      const std::vector<double>& braking_speed,
                                      const Params& params) {
  const size_t n_ref = reference.x.size();
  const size_t n_stages = boxes.size();
  const Path path(reference);
  const std::vector<double>& s_ref = path.sampleArcLengths();

  std::vector<double> lane_left(n_ref), lane_right(n_ref), boundary_left(n_ref), boundary_right(n_ref);
  for (size_t i = 0; i < n_ref; ++i) {
    std::tie(lane_left[i], lane_right[i]) = reference.lane_distances[i];
    std::tie(boundary_left[i], boundary_right[i]) = reference.boundary_distances[i];
  }
  const auto in_lane_at = [&](const Projection& p) { return p.e <= path.at(p.s, lane_left) && p.e >= -path.at(p.s, lane_right); };

  // current ego box along the path and the reference samples next to it
  const double ego_yaw = ego_pose[2];
  const Point ego_center{ego_pose[0] + params.ego_offset_to_geometric_center[0] * std::cos(ego_yaw) -
                             params.ego_offset_to_geometric_center[1] * std::sin(ego_yaw),
                         ego_pose[1] + params.ego_offset_to_geometric_center[0] * std::sin(ego_yaw) +
                             params.ego_offset_to_geometric_center[1] * std::cos(ego_yaw)};
  double ego_s_min = INF, ego_s_max = -INF, ego_e_min = INF, ego_e_max = -INF;
  for (const auto& corner : boxCorners(ego_center[0], ego_center[1], ego_yaw, params.ego_half_length, params.ego_half_width)) {
    const Projection p = path.project(corner);
    ego_s_min = std::min(ego_s_min, p.s);
    ego_s_max = std::max(ego_s_max, p.s);
    ego_e_min = std::min(ego_e_min, p.e);
    ego_e_max = std::max(ego_e_max, p.e);
  }

  // an object is next to a reference sample if it is within half the ego length plus the sample spacing
  std::vector<double> window(n_ref);
  for (size_t i = 0; i < n_ref; ++i) {
    const double previous = i > 0 ? std::abs(s_ref[i] - s_ref[i - 1]) : 0.0;
    const double next = i + 1 < n_ref ? std::abs(s_ref[i + 1] - s_ref[i]) : 0.0;
    window[i] = params.ego_half_length + std::max(previous, next);
  }

  Result result;
  result.d_object_left.assign(n_ref, NO_BOUND);
  result.d_object_right.assign(n_ref, NO_BOUND);
  result.s = s_ref;
  result.follow.assign(n_stages, {0.0, 0.0, 0.0});
  result.follow_slot.assign(n_stages, -1);

  // follow or pass, per prediction
  std::map<std::pair<uint64_t, size_t>, Memory> memory;
  std::vector<bool> followed(slots.size(), false);
  for (size_t slot = 0; slot < slots.size() && n_stages > 0; ++slot) {
    if (!boxes[0][slot].active) continue;
    const auto key = std::make_pair(slots[slot].object_id, slots[slot].prediction);
    const auto previous_it = memory_.find(key);
    const Memory previous = previous_it != memory_.end() ? previous_it->second : Memory{};

    // only corners the ego can still be next to at that stage count, i.e. not behind the ego even with full braking
    std::vector<Projection> centers, corners;
    for (size_t stage = 0; stage < n_stages; ++stage) {
      const Box& box = boxes[stage][slot];
      if (!box.active) continue;
      const size_t n_corners = corners.size();
      for (const auto& corner : boxCorners(box.x, box.y, box.yaw, box.half_length, box.half_width)) {
        const Projection p = path.project(corner);
        if (p.s >= ego_s_min + braking_travel[stage]) corners.push_back(p);
      }
      if (corners.size() > n_corners) centers.push_back(path.project({box.x, box.y}));
    }
    if (centers.empty()) continue;  // never next to the ego: neither followed nor passed

    // lane membership with hysteresis: the prediction enters our lane when its center is in it at any stage, and
    // leaves it only when none of its corners is in it any more
    const bool in_lane = previous.in_lane ? std::any_of(corners.begin(), corners.end(), in_lane_at)
                                          : std::any_of(centers.begin(), centers.end(), in_lane_at);

    // side of the reference path the prediction is on; a prediction that crosses the path is never passed
    std::vector<double> center_offsets;
    for (const auto& p : centers) center_offsets.push_back(p.e);
    const size_t middle = center_offsets.size() / 2;
    std::nth_element(center_offsets.begin(), center_offsets.begin() + static_cast<std::ptrdiff_t>(middle), center_offsets.end());
    double median = center_offsets[middle];
    if (center_offsets.size() % 2 == 0) {
      median = 0.5 *
               (median + *std::max_element(center_offsets.begin(), center_offsets.begin() + static_cast<std::ptrdiff_t>(middle)));
    }
    const bool right = median < 0.0;  // the ego passes a prediction on the right of the path on the left
    const int side = right ? 1 : -1;
    const bool crosses =
        std::any_of(corners.begin(), corners.end(), [&](const Projection& p) { return right ? p.e > 0.0 : p.e < 0.0; });

    // lateral extent of the prediction next to each reference sample
    std::vector<double> e_near(n_ref, right ? -INF : INF);
    std::vector<bool> near(n_ref, false), alongside(n_ref, false);
    for (const auto& p : corners) {
      const double s_clamped = std::clamp(p.s, s_ref.front(), s_ref.back());  // beyond the ends: the first/last sample
      for (size_t i = 0; i < n_ref; ++i) {
        if (std::abs(s_clamped - s_ref[i]) <= window[i]) {
          near[i] = true;
          e_near[i] = right ? std::max(e_near[i], p.e) : std::min(e_near[i], p.e);
        }
        if (std::abs(p.s - s_ref[i]) <= window[i]) alongside[i] = true;
      }
    }

    // the ego fits next to the prediction with margin m: in our lane only within the lane, otherwise within the boundaries
    const auto fits = [&](double m) {
      for (size_t i = 0; i < n_ref; ++i) {
        if (!near[i]) continue;
        const double far = in_lane ? (right ? lane_left[i] : lane_right[i])
                                   : (right ? boundary_left[i] : boundary_right[i]) - params.d_min_boundary_lat;
        if (right ? e_near[i] + m + params.ego_half_width > far - params.ego_half_width
                  : -far + params.ego_half_width > e_near[i] - m - params.ego_half_width) {
          return false;
        }
      }
      return true;
    };
    // the current ego pose already keeps margin m where the prediction is next to the ego now
    const auto ego_clear = [&](double m) {
      for (size_t i = 0; i < n_ref; ++i) {
        if (!alongside[i] || s_ref[i] < ego_s_min || s_ref[i] > ego_s_max) continue;
        if (right ? ego_e_min < e_near[i] + m : ego_e_max > e_near[i] - m) return false;
      }
      return true;
    };

    const auto margin_it = params.class_lateral_margins.find(slots[slot].object_class);
    const double margin = margin_it != params.class_lateral_margins.end() ? margin_it->second : params.d_min_object_lat;
    const double d_coll = params.d_min_object_collision;
    const bool keep_passing = previous.passed_side == side && fits(d_coll) && ego_clear(d_coll);
    const int passed_side = !crosses && (keep_passing || (fits(margin) && ego_clear(margin))) ? side : 0;
    memory[key] = {in_lane, passed_side};

    if (passed_side == 0) {
      followed[slot] = true;
      continue;
    }
    for (size_t i = 0; i < n_ref; ++i) {
      if (!alongside[i]) continue;
      if (right) {
        result.d_object_right[i] = std::min(result.d_object_right[i], -(e_near[i] + margin));
      } else {
        result.d_object_left[i] = std::min(result.d_object_left[i], e_near[i] - margin);
      }
    }
  }
  memory_ = std::move(memory);

  // follow distance per stage: the followed prediction in our lane whose rear is closest along the path, if the ego can
  // still stay behind it with full braking; relaxed by the part of the desired distance full braking cannot reach
  const double s_front = ocpFrontArcLength(reference, s_ref, ego_center, ego_yaw, params);
  for (size_t stage = 0; stage < n_stages; ++stage) {
    double s_rear = INF;
    int follow_slot = -1;
    for (size_t slot = 0; slot < slots.size(); ++slot) {
      const Box& box = boxes[stage][slot];
      if (!followed[slot] || !box.active) continue;
      bool in_lane = false;
      double s_min = INF;
      for (const auto& corner : boxCorners(box.x, box.y, box.yaw, box.half_length, box.half_width)) {
        const Projection p = path.project(corner);
        in_lane = in_lane || in_lane_at(p);
        s_min = std::min(s_min, p.s);
      }
      if (in_lane && s_min < s_rear) {
        s_rear = s_min;
        follow_slot = static_cast<int>(slot);
      }
    }
    if (follow_slot < 0) continue;
    const double gap = s_rear - s_front - braking_travel[stage];
    if (gap < params.d_min_object_collision) continue;
    const double relax = std::max({0.0, params.d_min_object_long - gap, params.thw * braking_speed[stage] - gap});
    result.follow[stage] = {s_rear, 1.0, relax};
    result.follow_slot[stage] = follow_slot;
    result.follow_active = true;
    result.brake_needed = result.brake_needed || gap < params.d_min_object_long;
  }
  return result;
}

}  // namespace trajectory_optimization

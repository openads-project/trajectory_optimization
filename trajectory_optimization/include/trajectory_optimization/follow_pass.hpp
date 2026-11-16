// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

namespace trajectory_optimization {

/**
 * @brief Decides per object prediction whether the ego follows or passes it and derives the corresponding OCP inputs.
 *
 * A prediction is passed if it never crosses the reference path, the ego fits next to it with the lateral margin of its
 * class, and the current ego pose already keeps that margin where the prediction is next to the ego. The room for passing
 * is the suggested lane if the prediction is in that lane, otherwise the active route boundaries. A passed prediction stays
 * passed while this holds with d_min_object_collision as margin. All other predictions are followed.
 *
 * Passed predictions bound the lateral offset of the ego from the reference path. Per stage, the followed prediction in the
 * suggested lane whose rear is closest along the reference path gives the follow distance, if the ego can still stay behind
 * it with full braking. The desired distance is relaxed by the part that full braking cannot reach.
 */
class FollowPass {
 public:
  /** Finite sentinel for "no bound", like the boundary distances (infinity would propagate NaNs in the OCP). */
  static constexpr double NO_BOUND = 1e6;

  /** Oriented box of an object slot at one stage. */
  struct Box {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double half_length = 0.0;
    double half_width = 0.0;
    bool active = false;
  };

  /** Identity of an OCP object slot (constant over the stages). */
  struct Slot {
    uint64_t object_id = 0;
    size_t prediction = 0;
    uint8_t object_class = 0;
  };

  /** Reference path samples as passed to the OCP. Distances are (left, right) and positive. */
  struct Reference {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> psi;
    std::vector<std::pair<double, double>> boundary_distances;  // active route boundaries
    std::vector<std::pair<double, double>> lane_distances;      // suggested lane
  };

  struct Params {
    double thw = 0.0;
    double d_min_object_long = 0.0;
    double d_min_object_lat = 0.0;  // lateral margin of classes without an own margin
    double d_min_object_collision = 0.0;
    double d_min_boundary_lat = 0.0;
    std::map<uint8_t, double> class_lateral_margins;
    double ego_half_length = 0.0;
    double ego_half_width = 0.0;
    std::array<double, 2> ego_offset_to_geometric_center{};
  };

  struct Result {
    std::vector<double> d_object_left;          // per reference sample
    std::vector<double> d_object_right;         // per reference sample
    std::vector<double> s;                      // arc length per reference sample
    std::vector<std::array<double, 3>> follow;  // per stage [s_rear, active, relax]
    std::vector<int> follow_slot;               // per stage the slot replaced by the follow distance, -1 if none
    bool follow_active = false;                 // a follow distance is set at any stage
    bool brake_needed = false;                  // d_min_object_long is reachable only with full braking at an active stage
  };

  /**
   * @brief Decides follow/pass for all active object slots and derives bounds and follow distances.
   *
   * @param[in] reference Reference samples as passed to the OCP.
   * @param[in] boxes Object boxes per stage and slot.
   * @param[in] slots Identity of every slot.
   * @param[in] ego_pose Pose (x, y, yaw) of the vehicle frame at the first stage.
   * @param[in] braking_travel Distance travelled with full braking until each stage.
   * @param[in] braking_speed Speed at each stage with full braking.
   * @param[in] params Parameters.
   * @return OCP inputs derived from the decisions.
   */
  Result decide(const Reference& reference,
                const std::vector<std::vector<Box>>& boxes,
                const std::vector<Slot>& slots,
                const std::array<double, 3>& ego_pose,
                const std::vector<double>& braking_travel,
                const std::vector<double>& braking_speed,
                const Params& params);

 private:
  struct Memory {
    bool in_lane = false;
    int passed_side = 0;  // +1 passed on the left, -1 passed on the right, 0 followed
  };

  // decisions of the previous cycle per (object id, prediction)
  std::map<std::pair<uint64_t, size_t>, Memory> memory_;
};

}  // namespace trajectory_optimization

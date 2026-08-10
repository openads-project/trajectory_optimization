// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

namespace trajectory_optimization {

enum class InitialGuessMode { WARM_START, COLD_START, BRAKING, LEFT, RIGHT, BRAKING_LEFT, BRAKING_RIGHT };

/** Parse a configured Ackermann initial-guess mode. */
InitialGuessMode parseInitialGuessMode(const std::string& name);

/** Return the stable configuration name of an initial-guess mode. */
const char* initialGuessModeName(InitialGuessMode mode);

/** Whether the mode contains the jerk-limited stopping profile. */
bool initialGuessUsesBraking(InitialGuessMode mode);

/** Whether the mode contains a left or right lane-change profile. */
bool initialGuessUsesLateralMotion(InitialGuessMode mode);

/**
 * @brief Compute one jerk sample for a dynamically feasible forward-only
 * braking seed.
 *
 * The braking and release arcs are joined on the discrete stopping switching
 * curve. Once release starts, positive jerk brings acceleration back to zero
 * without starting another braking arc.
 */
double brakingInitialGuessJerk(double velocity,
                               double acceleration,
                               double time_step,
                               double jerk_min,
                               double jerk_max,
                               double acceleration_min,
                               bool& release_started);

/** Compute one bounded steering-rate sample for a smooth left/right offset
 * seed. */
double lateralInitialGuessSteeringRate(InitialGuessMode mode,
                                       double time,
                                       double horizon,
                                       double time_step,
                                       double steering_angle,
                                       double steering_rate_min,
                                       double steering_rate_max,
                                       double steering_angle_min,
                                       double steering_angle_max);

}  // namespace trajectory_optimization

// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <trajectory_optimization/initial_guess.hpp>

namespace trajectory_optimization {

InitialGuessMode parseInitialGuessMode(const std::string& name) {
  if (name == "warm_start") {
    return InitialGuessMode::WARM_START;
  }
  if (name == "cold_start") {
    return InitialGuessMode::COLD_START;
  }
  if (name == "braking") {
    return InitialGuessMode::BRAKING;
  }
  if (name == "left") {
    return InitialGuessMode::LEFT;
  }
  if (name == "right") {
    return InitialGuessMode::RIGHT;
  }
  if (name == "braking_left") {
    return InitialGuessMode::BRAKING_LEFT;
  }
  if (name == "braking_right") {
    return InitialGuessMode::BRAKING_RIGHT;
  }
  throw std::invalid_argument("Unknown initial guess mode: " + name);
}

const char* initialGuessModeName(const InitialGuessMode mode) {
  switch (mode) {
    case InitialGuessMode::WARM_START:
      return "warm_start";
    case InitialGuessMode::COLD_START:
      return "cold_start";
    case InitialGuessMode::BRAKING:
      return "braking";
    case InitialGuessMode::LEFT:
      return "left";
    case InitialGuessMode::RIGHT:
      return "right";
    case InitialGuessMode::BRAKING_LEFT:
      return "braking_left";
    case InitialGuessMode::BRAKING_RIGHT:
      return "braking_right";
  }
  throw std::invalid_argument("Invalid initial guess mode.");
}

bool initialGuessUsesBraking(const InitialGuessMode mode) {
  return mode == InitialGuessMode::BRAKING || mode == InitialGuessMode::BRAKING_LEFT || mode == InitialGuessMode::BRAKING_RIGHT;
}

bool initialGuessUsesLateralMotion(const InitialGuessMode mode) {
  return mode == InitialGuessMode::LEFT || mode == InitialGuessMode::RIGHT || mode == InitialGuessMode::BRAKING_LEFT ||
         mode == InitialGuessMode::BRAKING_RIGHT;
}

double brakingInitialGuessJerk(const double velocity,
                               const double acceleration,
                               const double time_step,
                               const double jerk_min,
                               const double jerk_max,
                               const double acceleration_min,
                               bool& release_started) {
  if (time_step <= 0.0 || jerk_min >= 0.0 || jerk_max <= 0.0) {
    throw std::invalid_argument(
        "Braking initial guess requires dt > 0, "
        "jerk_min < 0 and jerk_max > 0.");
  }

  if (release_started) {
    return acceleration < 0.0 ? std::clamp(-acceleration / time_step, jerk_min, jerk_max) : 0.0;
  }

  const double jerk_to_acceleration_limit = (acceleration_min - acceleration) / time_step;
  const double braking_jerk = acceleration > acceleration_min - jerk_min * time_step
                                  ? jerk_min
                                  : std::clamp(jerk_to_acceleration_limit, jerk_min, jerk_max);
  const auto switchingFunction = [&](const double jerk) {
    const double next_acceleration = acceleration + jerk * time_step;
    const double next_velocity = velocity + acceleration * time_step + 0.5 * jerk * time_step * time_step;
    const double release_velocity = next_acceleration < 0.0 ? next_acceleration * next_acceleration / (2.0 * jerk_max) : 0.0;
    return next_velocity - release_velocity;
  };

  if (acceleration >= 0.0 || switchingFunction(braking_jerk) > 0.0) {
    return braking_jerk;
  }

  release_started = true;
  if (switchingFunction(jerk_max) <= 0.0) {
    return jerk_max;
  }

  // Find the jerk that lands exactly on v = a^2 / (2*j_max). Subsequent
  // positive jerk then reaches v ~= a ~= 0.
  double lower = braking_jerk;
  double upper = jerk_max;
  for (int iteration = 0; iteration < 40; ++iteration) {
    const double middle = 0.5 * (lower + upper);
    if (switchingFunction(middle) > 0.0) {
      upper = middle;
    } else {
      lower = middle;
    }
  }
  return 0.5 * (lower + upper);
}

double lateralInitialGuessSteeringRate(const InitialGuessMode mode,
                                       const double time,
                                       const double horizon,
                                       const double time_step,
                                       const double steering_angle,
                                       const double steering_rate_min,
                                       const double steering_rate_max,
                                       const double steering_angle_min,
                                       const double steering_angle_max) {
  if (!initialGuessUsesLateralMotion(mode)) {
    return 0.0;
  }
  if (horizon <= 0.0 || time_step <= 0.0) {
    throw std::invalid_argument("Lateral initial guess requires horizon > 0 and dt > 0.");
  }

  constexpr double STEERING_RATE_FRACTION = 0.15;
  const double direction = mode == InitialGuessMode::LEFT || mode == InitialGuessMode::BRAKING_LEFT ? 1.0 : -1.0;
  const double available_rate = std::min(std::abs(steering_rate_min), steering_rate_max);
  const double phase = 2.0 * M_PI * (time + 0.5 * time_step) / horizon;
  double steering_rate = direction * STEERING_RATE_FRACTION * available_rate * std::cos(phase);

  // Keep the rolled-out steering state inside its hard bounds despite the
  // discrete integration.
  const double lower_rate = std::max(steering_rate_min, (steering_angle_min - steering_angle) / time_step);
  const double upper_rate = std::min(steering_rate_max, (steering_angle_max - steering_angle) / time_step);
  return std::clamp(steering_rate, lower_rate, upper_rate);
}

}  // namespace trajectory_optimization

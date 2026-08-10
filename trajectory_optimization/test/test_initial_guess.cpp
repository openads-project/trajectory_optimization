// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <trajectory_optimization/initial_guess.hpp>

namespace trajectory_optimization {

TEST(InitialGuess, ParsesAllSupportedModes) {
  for (const std::string name : {"warm_start", "braking", "left", "right", "braking_left", "braking_right"}) {
    EXPECT_EQ(name, initialGuessModeName(parseInitialGuessMode(name)));
  }
  EXPECT_THROW(parseInitialGuessMode("unknown"), std::invalid_argument);
}

TEST(InitialGuess, BrakingProfileIsBoundedAndDoesNotReverse) {
  constexpr double DT = 0.1;
  constexpr double JERK_MIN = -5.0;
  constexpr double JERK_MAX = 2.0;
  constexpr double ACCELERATION_MIN = -4.5;
  constexpr double VELOCITY_MIN = -0.01;
  double velocity = 5.0;
  double acceleration = 0.0;
  bool release_started = false;

  for (int stage = 0; stage < 50; ++stage) {
    const double jerk =
        brakingInitialGuessJerk(velocity, acceleration, DT, JERK_MIN, JERK_MAX, ACCELERATION_MIN, release_started);
    EXPECT_GE(jerk, JERK_MIN);
    EXPECT_LE(jerk, JERK_MAX);
    velocity += acceleration * DT + 0.5 * jerk * DT * DT;
    acceleration += jerk * DT;
    EXPECT_GE(velocity, VELOCITY_MIN);
    EXPECT_GE(acceleration, ACCELERATION_MIN);
  }

  EXPECT_TRUE(release_started);
  EXPECT_NEAR(velocity, 0.0, 5e-3);
  EXPECT_NEAR(acceleration, 0.0, 1e-12);
}

TEST(InitialGuess, LateralProfilesAreMirroredAndBounded) {
  constexpr double HORIZON = 5.0;
  constexpr double DT = 0.1;
  constexpr double RATE_MIN = -0.7330382858;
  constexpr double RATE_MAX = 0.7330382858;
  constexpr double ANGLE_MIN = -0.5434955291;
  constexpr double ANGLE_MAX = 0.5434955291;
  double left_angle = 0.0;
  double right_angle = 0.0;

  for (int stage = 0; stage < 50; ++stage) {
    const double time = stage * DT;
    const double left = lateralInitialGuessSteeringRate(InitialGuessMode::LEFT, time, HORIZON, DT, left_angle, RATE_MIN, RATE_MAX,
                                                        ANGLE_MIN, ANGLE_MAX);
    const double right = lateralInitialGuessSteeringRate(InitialGuessMode::RIGHT, time, HORIZON, DT, right_angle, RATE_MIN,
                                                         RATE_MAX, ANGLE_MIN, ANGLE_MAX);
    EXPECT_NEAR(left, -right, 1e-12);
    EXPECT_GE(left, RATE_MIN);
    EXPECT_LE(left, RATE_MAX);
    left_angle += left * DT;
    right_angle += right * DT;
    EXPECT_GE(left_angle, ANGLE_MIN);
    EXPECT_LE(left_angle, ANGLE_MAX);
    EXPECT_GE(right_angle, ANGLE_MIN);
    EXPECT_LE(right_angle, ANGLE_MAX);
  }

  EXPECT_NEAR(left_angle, 0.0, 1e-12);
  EXPECT_NEAR(right_angle, 0.0, 1e-12);
}

}  // namespace trajectory_optimization

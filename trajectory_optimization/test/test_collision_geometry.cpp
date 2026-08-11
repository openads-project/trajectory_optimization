// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>

#include <gtest/gtest.h>

#include <trajectory_optimization/collision_geometry.hpp>

namespace trajectory_optimization {

TEST(CollisionGeometry, AppliesBothGeometricCenterOffsets) {
  const auto box = orientedBoxFromReference(1.0, 2.0, M_PI_2, 4.0, 2.0, 3.0, 0.5);
  EXPECT_NEAR(box.x, 0.5, 1e-12);
  EXPECT_NEAR(box.y, 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(box.half_length, 2.0);
  EXPECT_DOUBLE_EQ(box.half_width, 1.0);
}

TEST(CollisionGeometry, DefinesObbsForBothVehicleModels) {
  const auto karl = vehicleGeometry("karl");
  const auto shuttle = vehicleGeometry("shuttle");
  EXPECT_DOUBLE_EQ(karl.length, 5.173);
  EXPECT_DOUBLE_EQ(karl.center_offset_long, 1.4895);
  EXPECT_DOUBLE_EQ(shuttle.length, 4.97);
  EXPECT_DOUBLE_EQ(shuttle.width, 2.12);
  EXPECT_DOUBLE_EQ(shuttle.center_offset_long, 0.0);
}

TEST(CollisionGeometry, ExpandsFrontAndRearIndependently) {
  const OrientedBox physical{1.0, -2.0, M_PI_2, 2.5, 1.0};
  const auto safety = expandBoxForward(physical, 20.0, 0.1, 0.2);
  EXPECT_NEAR(safety.y - safety.half_length, physical.y - physical.half_length - 0.1, 1e-12);
  EXPECT_NEAR(safety.y + safety.half_length, physical.y + physical.half_length + 20.0, 1e-12);
  EXPECT_NEAR(safety.half_width, 1.2, 1e-12);
}

TEST(CollisionGeometry, HandlesOverlapContactAndSeparation) {
  const OrientedBox ego{0.0, 0.0, 0.0, 2.0, 1.0};
  EXPECT_LT(exactSatSeparationMargin(ego, {0.0, 0.0, M_PI_4, 2.0, 1.0}), 0.0);
  EXPECT_NEAR(exactSatSeparationMargin(ego, {4.0, 0.0, 0.0, 2.0, 1.0}), 0.0, 1e-12);
  EXPECT_GT(exactSatSeparationMargin(ego, {0.0, 3.1, M_PI_2, 2.0, 0.5}), 0.0);
}

TEST(CollisionGeometry, InterpolatesPredictionsAndRanksOverflowDeterministically) {
  const std::vector<double> times{10.0, 11.0, 12.0};
  EXPECT_DOUBLE_EQ(interpolateSample(times, {2.0, 4.0, 8.0}, 11.5), 6.0);
  EXPECT_NEAR(std::abs(interpolateSample(times, {3.0, -3.0, -2.0}, 10.5, true)), M_PI, 1e-12);

  const std::vector<HypothesisPriority> priorities{{1.0, 2, 9, 1}, {1.0, 1, 10, 3}, {1.0, 1, 10, 2}, {-1.0, 5, 99, 0}};
  EXPECT_EQ(rankHypotheses(priorities), (std::vector<size_t>{3, 2, 1, 0}));
}

}  // namespace trajectory_optimization

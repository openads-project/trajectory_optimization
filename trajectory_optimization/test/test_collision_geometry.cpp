// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <random>

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

TEST(CollisionGeometry, ExpandsTimeHeadwayOnlyTowardsTheFront) {
  const OrientedBox physical{1.0, -2.0, M_PI_2, 2.5, 1.0};
  const auto safety = expandBoxForward(physical, 20.0, 1.0, 0.1);

  EXPECT_NEAR(safety.x, physical.x, 1e-12);
  EXPECT_NEAR(safety.y, physical.y + 9.5, 1e-12);
  EXPECT_NEAR(safety.half_length, 13.0, 1e-12);
  EXPECT_NEAR(safety.half_width, 1.1, 1e-12);

  const double physical_rear_y = physical.y - physical.half_length;
  const double physical_front_y = physical.y + physical.half_length;
  EXPECT_NEAR(safety.y - safety.half_length, physical_rear_y - 1.0, 1e-12);
  EXPECT_NEAR(safety.y + safety.half_length, physical_front_y + 20.0, 1e-12);
}

TEST(CollisionGeometry, HandlesParallelRotatedAndTouchingBoxes) {
  const OrientedBox ego{0.0, 0.0, 0.0, 2.0, 1.0};
  EXPECT_LT(exactSatSeparationMargin(ego, {0.0, 0.0, M_PI_4, 2.0, 1.0}), 0.0);
  EXPECT_NEAR(exactSatSeparationMargin(ego, {4.0, 0.0, 0.0, 2.0, 1.0}), 0.0, 1e-12);
  EXPECT_GT(exactSatSeparationMargin(ego, {0.0, 3.1, M_PI_2, 2.0, 0.5}), 0.0);
}

TEST(CollisionGeometry, SmoothMarginNeverExceedsExactSatMargin) {
  std::mt19937 generator(42);
  std::uniform_real_distribution<double> position(-20.0, 20.0);
  std::uniform_real_distribution<double> yaw(-M_PI, M_PI);
  std::uniform_real_distribution<double> extent(0.05, 8.0);
  for (int sample = 0; sample < 20000; ++sample) {
    const OrientedBox first{position(generator), position(generator), yaw(generator), extent(generator), extent(generator)};
    const OrientedBox second{position(generator), position(generator), yaw(generator), extent(generator), extent(generator)};
    EXPECT_LE(conservativeSmoothSatMargin(first, second), exactSatSeparationMargin(first, second) + 1e-12);
  }
}

TEST(CollisionGeometry, SupportContainsAllCorners) {
  const OrientedBox box{2.0, -1.0, 0.7, 3.0, 0.8};
  const double normal_x = 0.3;
  const double normal_y = std::sqrt(1.0 - normal_x * normal_x);
  const double support = boxSupport(box, normal_x, normal_y);
  const double center_projection = box.x * normal_x + box.y * normal_y;
  const double cos_yaw = std::cos(box.yaw);
  const double sin_yaw = std::sin(box.yaw);
  for (double longitudinal : {-box.half_length, box.half_length}) {
    for (double lateral : {-box.half_width, box.half_width}) {
      const double corner_x = box.x + longitudinal * cos_yaw - lateral * sin_yaw;
      const double corner_y = box.y + longitudinal * sin_yaw + lateral * cos_yaw;
      EXPECT_LE(std::abs(corner_x * normal_x + corner_y * normal_y - center_projection), support + 1e-12);
    }
  }
}

TEST(CollisionGeometry, DetectsStraightAndCurvedBoundaryViolations) {
  const std::vector<Point2d> left{{-10.0, 2.0}, {0.0, 2.0}, {10.0, 3.0}};
  const std::vector<Point2d> right{{-10.0, -2.0}, {0.0, -2.0}, {10.0, -1.0}};
  EXPECT_FALSE(boxOutsideBoundary({0.0, 0.0, 0.0, 1.0, 0.5}, left, true));
  EXPECT_FALSE(boxOutsideBoundary({0.0, 0.0, 0.0, 1.0, 0.5}, right, false));
  EXPECT_TRUE(boxOutsideBoundary({0.0, 2.0, 0.0, 1.0, 0.5}, left, true));
  EXPECT_TRUE(boxOutsideBoundary({0.0, -2.0, 0.0, 1.0, 0.5}, right, false));
  EXPECT_NEAR(boxBoundaryViolationDepth({0.0, 2.0, 0.0, 1.0, 0.5}, left, true), 0.5, 1e-12);
  EXPECT_GT(boxBoundaryViolationDepth({0.0, -2.0, 0.0, 1.0, 0.5}, right, false), 0.5);
  EXPECT_NEAR(boxBoundaryViolationDepth({0.0, 0.0, 0.0, 1.0, 0.5}, left, true), -1.5, 1e-12);
}

TEST(CollisionGeometry, InterpolatesYawAcrossWrapAround) {
  const auto interpolated = interpolateBox({0.0, 0.0, 3.0, 2.0, 1.0}, {2.0, 4.0, -3.0, 4.0, 2.0}, 0.5);
  EXPECT_NEAR(interpolated.x, 1.0, 1e-12);
  EXPECT_NEAR(interpolated.y, 2.0, 1e-12);
  EXPECT_NEAR(std::abs(interpolated.yaw), M_PI, 1e-12);
  EXPECT_NEAR(interpolated.half_length, 3.0, 1e-12);
}

TEST(CollisionGeometry, InterpolatesPredictionSamplesAndClampsHorizon) {
  const std::vector<double> times{10.0, 11.0, 12.0};
  EXPECT_DOUBLE_EQ(interpolateSample(times, {2.0, 4.0, 8.0}, 9.0), 2.0);
  EXPECT_DOUBLE_EQ(interpolateSample(times, {2.0, 4.0, 8.0}, 13.0), 8.0);
  EXPECT_DOUBLE_EQ(interpolateSample(times, {2.0, 4.0, 8.0}, 11.5), 6.0);
  EXPECT_NEAR(std::abs(interpolateSample(times, {3.0, -3.0, -2.0}, 10.5, true)), M_PI, 1e-12);
}

TEST(CollisionGeometry, RanksMultimodalHypothesesDeterministicallyAndSupportsOverflow) {
  const std::vector<HypothesisPriority> priorities{{1.0, 2, 9, 1}, {1.0, 1, 10, 3}, {1.0, 1, 10, 2}, {-1.0, 5, 99, 0}};
  EXPECT_EQ(rankHypotheses(priorities), (std::vector<size_t>{3, 2, 1, 0}));

  std::vector<HypothesisPriority> overflow;
  for (uint64_t index = 0; index < 35; ++index) {
    overflow.push_back({static_cast<double>(35 - index), 0, index / 2, static_cast<int>(index % 2)});
  }
  const auto ranking = rankHypotheses(overflow);
  ASSERT_EQ(ranking.size(), 35U);
  EXPECT_EQ(ranking.front(), 34U);
  EXPECT_EQ(ranking[29], 5U);
  EXPECT_EQ(ranking.size() - 30U, 5U);
}

}  // namespace trajectory_optimization

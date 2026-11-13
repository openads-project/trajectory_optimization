#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <trajectory_optimization/performance_logger.hpp>

namespace {

std::vector<std::string> split(const std::string& line) {
  std::istringstream input(line);
  std::vector<std::string> result;
  for (std::string value; std::getline(input, value, ',');) result.push_back(value);
  return result;
}

class PerformanceLoggerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* previous = std::getenv("TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR");
    if (previous != nullptr) previous_directory_ = previous;
    directory_ = std::filesystem::temp_directory_path() /
                 ("optimizer-timing-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    setenv("TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR", directory_.c_str(), 1);
  }

  void TearDown() override {
    if (previous_directory_) {
      setenv("TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR", previous_directory_->c_str(), 1);
    } else {
      unsetenv("TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR");
    }
    std::filesystem::remove_all(directory_);
  }

  std::filesystem::path directory_;
  std::optional<std::string> previous_directory_;
};

TEST_F(PerformanceLoggerTest, DistinguishesSolutionsEmptyUpdatesAndSkippedCycles) {
  using trajectory_optimization::PerformanceLogger;
  using trajectory_optimization::PerformanceMetrics;
  std::filesystem::path path;
  {
    PerformanceLogger logger("test");
    path = logger.path();
    PerformanceMetrics metrics;
    metrics.ego_stamp_ns = 1000000000;
    metrics.ego_callback_ros_ns = 1010000000;
    metrics.cycle_start_ros_ns = 1025000000;
    metrics.solve_start_ros_ns = 1050000000;
    metrics.publish_ros_ns = 1100000000;
    metrics.ego_callback_to_cycle_wall_ms = 15.0;
    metrics.cycle_interval_wall_ms = 100.0;
    metrics.solver_called = true;
    metrics.status = 0;
    metrics.published = true;
    metrics.message_published = true;
    metrics.trajectory_states = 51;
    metrics.outcome = "valid_solution";
    logger.write(metrics);

    metrics.published = false;
    metrics.status = 7;
    metrics.trajectory_states = 0;
    metrics.outcome = "rejected_solution";
    logger.write(metrics);

    PerformanceMetrics skipped;
    skipped.outcome = "outdated_ego";
    logger.write(skipped);

    metrics.solver_called = false;
    metrics.status = -1;
    metrics.published = true;
    metrics.trajectory_states = 51;
    metrics.outcome = "standstill";
    logger.write(metrics);

    metrics.solver_called = true;
    metrics.ego_callback_ros_ns = 500000000;
    metrics.cycle_start_ros_ns = 600000000;
    metrics.solve_start_ros_ns = 700000000;
    metrics.publish_ros_ns = 800000000;
    metrics.outcome = "valid_solution";
    logger.write(metrics);
  }

  std::ifstream input(path);
  std::string line;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
  const auto header = split(line);
  std::vector<std::vector<std::string>> rows;
  while (std::getline(input, line)) {
    rows.push_back(split(line));
    ASSERT_EQ(rows.back().size(), header.size());
  }
  ASSERT_EQ(rows.size(), 5u);
  auto value = [&](size_t row, const std::string& name) {
    const auto column = std::find(header.begin(), header.end(), name);
    EXPECT_NE(column, header.end());
    return column == header.end() ? std::string() : rows.at(row).at(column - header.begin());
  };
  EXPECT_EQ(value(0, "schema_version"), "8");
  EXPECT_EQ(value(0, "published"), "1");
  EXPECT_EQ(value(0, "message_published"), "1");
  EXPECT_DOUBLE_EQ(std::stod(value(0, "ego_age_at_callback_ms")), 10.0);
  EXPECT_DOUBLE_EQ(std::stod(value(0, "ego_age_at_cycle_start_ms")), 25.0);
  EXPECT_DOUBLE_EQ(std::stod(value(0, "ego_age_at_solve_start_ms")), 50.0);
  EXPECT_DOUBLE_EQ(std::stod(value(0, "ego_age_at_publish_ms")), 100.0);
  EXPECT_DOUBLE_EQ(std::stod(value(0, "ego_callback_to_cycle_wall_ms")), 15.0);
  EXPECT_EQ(value(1, "published"), "0");
  EXPECT_EQ(value(1, "message_published"), "1");
  EXPECT_EQ(value(1, "trajectory_states"), "0");
  EXPECT_EQ(value(2, "solver_called"), "0");
  EXPECT_EQ(value(2, "message_published"), "0");
  EXPECT_EQ(value(2, "status"), "-1");
  EXPECT_TRUE(std::isnan(std::stod(value(2, "ego_age_at_solve_start_ms"))));
  EXPECT_TRUE(std::isnan(std::stod(value(2, "ego_age_at_publish_ms"))));
  EXPECT_TRUE(std::isnan(std::stod(value(2, "ego_age_at_callback_ms"))));
  EXPECT_EQ(value(3, "solver_called"), "0");
  EXPECT_EQ(value(3, "published"), "1");
  EXPECT_EQ(value(3, "outcome"), "standstill");
  EXPECT_TRUE(std::isnan(std::stod(value(3, "ego_age_at_solve_start_ms"))));
  EXPECT_DOUBLE_EQ(std::stod(value(4, "ego_age_at_callback_ms")), -500.0);
  EXPECT_DOUBLE_EQ(std::stod(value(4, "ego_age_at_publish_ms")), -200.0);
}

}  // namespace

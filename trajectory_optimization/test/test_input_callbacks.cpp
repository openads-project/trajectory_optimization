// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <atomic>
#include <future>
#include <thread>
#include <trajectory_optimization/trajectory_optimization_ackermann.hpp>

namespace trajectory_optimization {
namespace {
using namespace std::chrono_literals;

class TestOptimizer : public TrajectoryOptimizationAckermannNode {
 public:
  explicit TestOptimizer(bool reference_triggered)
      : TrajectoryOptimizationAckermannNode(rclcpp::NodeOptions()
                                                .arguments({"--ros-args", "--params-file", TEST_PARAMETER_FILE})
                                                .parameter_overrides({rclcpp::Parameter("run_as_callback", reference_triggered),
                                                                      rclcpp::Parameter("debug_visualization", false)})) {
    if (planning_timer_) planning_timer_->cancel();
  }

  bool buffered(int stamp) {
    std::lock_guard<std::mutex> lock(input_mutex_);
    return buffered_inputs_.ego_data && buffered_inputs_.ego_data->header.stamp.sec == stamp && buffered_inputs_.object_list &&
           buffered_inputs_.object_list->header.stamp.sec == stamp && buffered_inputs_.route &&
           buffered_inputs_.route->header.stamp.sec == stamp && buffered_inputs_.reference_trajectory &&
           buffered_inputs_.reference_trajectory->header.stamp.sec == stamp;
  }

  void snapshot() { takeInputSnapshot(); }
  int activeStamp() const { return ego_data_.header.stamp.sec; }
  int referenceStamp() const { return reference_trajectory_.header.stamp.sec; }
  int objectStamp() const { return object_list_.header.stamp.sec; }
  int routeStamp() const { return route_.header.stamp.sec; }
  uint64_t cycles() const { return logging_cycle_; }
  bool hasCallbackTime() const { return ego_callback_steady_time_.has_value() && ego_callback_ros_ns_ > 0; }
};

template <typename Predicate>
bool waitUntil(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::sleep_for(1ms);
  }
  return false;
}

class InputCallbacksTest : public testing::TestWithParam<bool> {
 protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_P(InputCallbacksTest, InputsAdvanceWhileSolverGroupIsBusy) {
  const bool reference_triggered = GetParam();
  auto optimizer = std::make_shared<TestOptimizer>(reference_triggered);
  auto peer = std::make_shared<rclcpp::Node>("input_callback_test_peer");
  const std::string prefix = std::string(optimizer->get_fully_qualified_name()) + "/";
  auto ego = peer->create_publisher<perception_msgs::msg::EgoData>(prefix + "ego_data", 1);
  auto objects = peer->create_publisher<perception_msgs::msg::ObjectList>(prefix + "object_list", 1);
  auto route = peer->create_publisher<route_planning_msgs::msg::Route>(prefix + "route", 1);
  auto reference = peer->create_publisher<trajectory_planning_msgs::msg::Trajectory>(prefix + "reference_trajectory", 1);
  ASSERT_TRUE(waitUntil([&]() {
    return ego->get_subscription_count() && objects->get_subscription_count() && route->get_subscription_count() &&
           reference->get_subscription_count();
  }));

  std::promise<void> entered;
  std::promise<void> release;
  auto released = release.get_future().share();
  auto blocker = optimizer->create_wall_timer(1ms, [&]() {
    // hold the same callback group used by the solver, with a frozen active snapshot.
    optimizer->snapshot();
    entered.set_value();
    released.wait();
  });
  std::atomic<bool> observed{false};
  auto observer = optimizer->create_wall_timer(1ms, [&]() {
    if (!reference_triggered) optimizer->snapshot();
    if (optimizer->activeStamp() == 12 && (!reference_triggered || optimizer->cycles() > 0)) observed = true;
  });

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(optimizer);
  std::thread spin([&]() { executor.spin(); });
  // always release and join, including after a failed expectation.
  struct Cleanup {
    std::promise<void>& release;
    rclcpp::Executor& executor;
    std::thread& thread;
    bool released = false;
    void unblock() {
      if (!released) {
        release.set_value();
        released = true;
      }
    }
    void stop() {
      unblock();
      executor.cancel();
      if (thread.joinable()) thread.join();
    }
    ~Cleanup() { stop(); }
  } cleanup{release, executor, spin};
  ASSERT_EQ(entered.get_future().wait_for(5s), std::future_status::ready);
  blocker->cancel();

  auto publish = [&](int stamp) {
    perception_msgs::msg::EgoData ego_msg;
    perception_msgs::msg::ObjectList objects_msg;
    route_planning_msgs::msg::Route route_msg;
    trajectory_planning_msgs::msg::Trajectory reference_msg;
    // old stamps make any reference-triggered planning cycle exit before solving.
    ego_msg.header.stamp.sec = stamp;
    objects_msg.header.stamp.sec = stamp;
    route_msg.header.stamp.sec = stamp;
    reference_msg.header.stamp.sec = stamp;
    ego->publish(ego_msg);
    objects->publish(objects_msg);
    route->publish(route_msg);
    reference->publish(reference_msg);
  };
  publish(11);
  ASSERT_TRUE(waitUntil([&]() { return optimizer->buffered(11); }));
  publish(12);
  ASSERT_TRUE(waitUntil([&]() { return optimizer->buffered(12); }));
  EXPECT_EQ(optimizer->activeStamp(), 0);
  EXPECT_EQ(optimizer->cycles(), 0u);

  cleanup.unblock();
  EXPECT_TRUE(waitUntil([&]() { return observed.load(); }));
  cleanup.stop();
  EXPECT_EQ(optimizer->referenceStamp(), 12);
  EXPECT_EQ(optimizer->objectStamp(), 12);
  EXPECT_EQ(optimizer->routeStamp(), 12);
  EXPECT_TRUE(optimizer->hasCallbackTime());
  EXPECT_EQ(optimizer->cycles(), reference_triggered ? 1u : 0u);
}

INSTANTIATE_TEST_SUITE_P(TimerAndReferenceModes, InputCallbacksTest, testing::Bool());
}  // namespace
}  // namespace trajectory_optimization

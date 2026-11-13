// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <trajectory_optimization/trajectory_optimization_ackermann.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<trajectory_optimization::TrajectoryOptimizationAckermannNode>(rclcpp::NodeOptions());
  // reserve one executor thread for inputs while the other runs a serialized solve.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

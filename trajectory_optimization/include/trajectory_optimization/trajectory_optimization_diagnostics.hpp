// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_planning_msgs/msg/trajectory.hpp>

namespace trajectory_optimization {

/**
 * @brief Configuration parameters for topic diagnostics.
 */
struct TopicDiagnosticConfig {
  double min_frequency = 1.0;                    // minimum acceptable frequency [Hz]
  double max_frequency = 100.0;                  // maximum acceptable frequency [Hz]
  double min_acceptable_timestamp_delta = -1.0;  // minimum acceptable difference between message timestamp and receipt time [s]
  double max_acceptable_timestamp_delta = 1.0;   // maximum acceptable difference between message timestamp and receipt time [s]
};

/**
 * @brief Basic scalar statistics for tracked duration values.
 */
struct DurationStats {
  size_t count = 0;
  double average = std::numeric_limits<double>::quiet_NaN();
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
  double stddev = std::numeric_limits<double>::quiet_NaN();
};

/**
 * @brief Holds diagnostics state and behavior for the trajectory optimization node.
 */
class TrajectoryOptimizationDiagnostics {
 public:
  explicit TrajectoryOptimizationDiagnostics(rclcpp::Node* node) : updater(node), node_(node) {}

  void health(diagnostic_updater::DiagnosticStatusWrapper& stat) {
    stat.summary(health_.status, health_.message);
    for (const auto& [key, value] : health_.key_value_pairs) {
      stat.add(key, value);
    }
  }

  void setHealth(const unsigned char status,
                 const std::string& msg,
                 const std::map<std::string, std::string>& key_value_pairs = {}) {
    health_.status = status;
    health_.message = msg;
    health_.key_value_pairs = key_value_pairs;
    updater.force_update();
  }

  void setHealthWithOcpData(const unsigned char status,
                            const std::string& msg,
                            const std::map<std::string, std::string>& extra_pairs = {}) {
    auto key_value_pairs = getOcpDiagnosticKeyValues();
    for (const auto& [key, value] : extra_pairs) {
      key_value_pairs[key] = value;
    }
    setHealth(status, msg, key_value_pairs);
  }

  void updateOcpTimingStats(const double solve_duration_ms) {
    if (!std::isfinite(solve_duration_ms)) return;
    ocp_current_duration_ms = solve_duration_ms;

    const rclcpp::Time now = node_->now();
    ocp_duration_history_ms.push_back({now, solve_duration_ms});
    while (!ocp_duration_history_ms.empty() &&
           (now - ocp_duration_history_ms.front().first).seconds() > ocp_duration_window_sec) {
      ocp_duration_history_ms.pop_front();
    }

    ++ocp_total_duration_count;
    ocp_total_duration_sum_ms += solve_duration_ms;
    ocp_total_duration_sum_sq_ms += solve_duration_ms * solve_duration_ms;
    ocp_total_duration_min_ms = std::min(ocp_total_duration_min_ms, solve_duration_ms);
    ocp_total_duration_max_ms = std::max(ocp_total_duration_max_ms, solve_duration_ms);
  }

  DurationStats getWindowedOcpDurationStats() const {
    DurationStats stats;
    if (ocp_duration_history_ms.empty()) return stats;

    double sum = 0.0;
    double sum_sq = 0.0;
    double min_value = std::numeric_limits<double>::infinity();
    double max_value = 0.0;
    for (const auto& sample : ocp_duration_history_ms) {
      const double value = sample.second;
      ++stats.count;
      sum += value;
      sum_sq += value * value;
      min_value = std::min(min_value, value);
      max_value = std::max(max_value, value);
    }

    stats.average = sum / static_cast<double>(stats.count);
    stats.minimum = min_value;
    stats.maximum = max_value;
    const double variance = std::max(0.0, (sum_sq / static_cast<double>(stats.count)) - (stats.average * stats.average));
    stats.stddev = std::sqrt(variance);
    return stats;
  }

  DurationStats getTotalOcpDurationStats() const {
    DurationStats stats;
    if (ocp_total_duration_count == 0) return stats;

    stats.count = ocp_total_duration_count;
    stats.average = ocp_total_duration_sum_ms / static_cast<double>(ocp_total_duration_count);
    stats.minimum = ocp_total_duration_min_ms;
    stats.maximum = ocp_total_duration_max_ms;
    const double variance = std::max(
        0.0, (ocp_total_duration_sum_sq_ms / static_cast<double>(ocp_total_duration_count)) - (stats.average * stats.average));
    stats.stddev = std::sqrt(variance);
    return stats;
  }

  std::map<std::string, std::string> getOcpDiagnosticKeyValues() const {
    const DurationStats window_stats = getWindowedOcpDurationStats();
    const DurationStats total_stats = getTotalOcpDurationStats();

    std::map<std::string, std::string> key_value_pairs = {
        {"ocp.solve_time.current_ms", formatDiagnosticDouble(ocp_current_duration_ms)},
        {"ocp.solve_time.last_10s.window_sec", formatDiagnosticDouble(ocp_duration_window_sec, 1)},
        {"ocp.solve_time.last_10s.count", std::to_string(window_stats.count)},
        {"ocp.solve_time.last_10s.avg_ms", formatDiagnosticDouble(window_stats.average)},
        {"ocp.solve_time.last_10s.min_ms", formatDiagnosticDouble(window_stats.minimum)},
        {"ocp.solve_time.last_10s.max_ms", formatDiagnosticDouble(window_stats.maximum)},
        {"ocp.solve_time.last_10s.std_ms", formatDiagnosticDouble(window_stats.stddev)},
        {"ocp.solve_time.total.count", std::to_string(total_stats.count)},
        {"ocp.solve_time.total.avg_ms", formatDiagnosticDouble(total_stats.average)},
        {"ocp.solve_time.total.min_ms", formatDiagnosticDouble(total_stats.minimum)},
        {"ocp.solve_time.total.max_ms", formatDiagnosticDouble(total_stats.maximum)},
        {"ocp.solve_time.total.std_ms", formatDiagnosticDouble(total_stats.stddev)},
        {"ocp.last_status_code", std::to_string(last_ocp_solver_status)},
        {"ocp.last_sqp_iter", std::to_string(last_ocp_sqp_iter)},
        {"ocp.last_kkt_norm_inf", formatDiagnosticDouble(last_ocp_kkt_norm_inf, 6)},
        {"ocp.last_cost_value", formatDiagnosticDouble(last_ocp_cost_value)},
        {"ocp.last_nlp_res", formatDiagnosticDouble(last_ocp_nlp_res)}};

    return key_value_pairs;
  }

  static std::string formatDiagnosticDouble(const double value, const int precision = 3) {
    if (!std::isfinite(value)) return "nan";
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
  }

  diagnostic_updater::Updater updater;

  // input topic diagnostics
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> ego_data_diagnostic;
  TopicDiagnosticConfig ego_data_diagnostic_config;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> object_list_diagnostic;
  TopicDiagnosticConfig object_list_diagnostic_config;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> route_diagnostic;
  TopicDiagnosticConfig route_diagnostic_config;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> reference_trajectory_diagnostic;
  TopicDiagnosticConfig reference_trajectory_diagnostic_config;

  // output topic diagnostics
  std::unique_ptr<diagnostic_updater::DiagnosedPublisher<trajectory_planning_msgs::msg::Trajectory>>
      trajectory_diagnosed_publisher;
  TopicDiagnosticConfig trajectory_diagnosed_publisher_config;

  // OCP diagnostic details
  double ocp_duration_window_sec = 10.0;
  std::deque<std::pair<rclcpp::Time, double>> ocp_duration_history_ms;
  size_t ocp_total_duration_count = 0;
  double ocp_total_duration_sum_ms = 0.0;
  double ocp_total_duration_sum_sq_ms = 0.0;
  double ocp_total_duration_min_ms = std::numeric_limits<double>::infinity();
  double ocp_total_duration_max_ms = 0.0;
  double ocp_current_duration_ms = std::numeric_limits<double>::quiet_NaN();
  int last_ocp_solver_status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  int last_ocp_sqp_iter = -1;
  double last_ocp_kkt_norm_inf = std::numeric_limits<double>::quiet_NaN();
  double last_ocp_cost_value = std::numeric_limits<double>::quiet_NaN();
  double last_ocp_nlp_res = std::numeric_limits<double>::quiet_NaN();
  std::string last_ocp_input_error = "";

 private:
  struct DiagnosticStatusState {
    unsigned char status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    std::string message = "";
    std::map<std::string, std::string> key_value_pairs = {};
  } health_;

  rclcpp::Node* node_;
};

}  // namespace trajectory_optimization

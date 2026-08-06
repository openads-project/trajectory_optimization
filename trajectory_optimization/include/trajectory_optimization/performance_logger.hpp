// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include <acados_c/ocp_nlp_interface.h>

namespace trajectory_optimization {

struct PerformanceMetrics {
  uint64_t cycle = 0;
  int64_t ego_stamp_ns = 0;
  int64_t reference_stamp_ns = 0;
  int64_t route_stamp_ns = 0;
  std::string outcome = "started";
  bool solver_ran = false;
  int status = -1;
  int sqp_iter = 0;
  int qp_iter = 0;
  int qp_status = 0;
  int reference_points = 0;
  int objects = 0;
  std::string collision_geometry = "circles";
  int obstacle_hypotheses = 0;
  int dropped_obstacle_hypotheses = 0;
  bool geometry_validated = false;
  int node_object_collisions = 0;
  int node_boundary_violations = 0;
  int dropped_hypothesis_collisions = 0;
  int intersample_object_collisions = 0;
  int intersample_boundary_violations = 0;
  double max_node_boundary_penetration_m = 0.0;
  double max_intersample_boundary_penetration_m = 0.0;
  bool published = false;

  double cycle_ms = 0.0;
  double preprocessing_ms = 0.0;
  double parameter_update_ms = 0.0;
  double solve_wall_ms = 0.0;
  double postprocessing_ms = 0.0;
  double acados_total_ms = 0.0;
  double acados_lin_ms = 0.0;
  double acados_sim_ms = 0.0;
  double acados_qp_ms = 0.0;
  double acados_qp_solver_ms = 0.0;
  double acados_qp_xcond_ms = 0.0;
  double acados_reg_ms = 0.0;
  double acados_glob_ms = 0.0;
  double acados_preparation_ms = 0.0;
  double acados_feedback_ms = 0.0;

  double cost_value = 0.0;
  double nlp_res = 0.0;
  double kkt_norm_inf = 0.0;
  double res_stat = 0.0;
  double res_eq = 0.0;
  double res_ineq = 0.0;
  double res_comp = 0.0;

  double max_ineq_violation = 0.0;
  int max_ineq_stage = -1;
  std::string max_ineq_type = "none";
  int max_ineq_index = -1;
  std::string max_ineq_side = "none";
  double max_eq_violation = 0.0;
  int max_eq_stage = -1;
  int max_eq_state = -1;
};

class PerformanceLogger {
 public:
  /**
   * @brief Creates a CSV performance log for the given node.
   *
   * @param[in] node_name Node name used as part of the log file name.
   */
  explicit PerformanceLogger(const std::string& node_name);

  /**
   * @brief Flushes and closes the performance log.
   */
  ~PerformanceLogger();

  /**
   * @brief Copy construction is disabled because the logger owns a file stream.
   */
  PerformanceLogger(const PerformanceLogger&) = delete;

  /**
   * @brief Copy assignment is disabled because the logger owns a file stream.
   *
   * @return Reference to this logger. The operator is deleted.
   */
  PerformanceLogger& operator=(const PerformanceLogger&) = delete;

  /**
   * @brief Move construction is disabled to keep the log stream bound to one logger.
   */
  PerformanceLogger(PerformanceLogger&&) = delete;

  /**
   * @brief Move assignment is disabled to keep the log stream bound to one logger.
   *
   * @return Reference to this logger. The operator is deleted.
   */
  PerformanceLogger& operator=(PerformanceLogger&&) = delete;

  /**
   * @brief Appends one set of performance metrics to the CSV log.
   *
   * @param[in] metrics Metrics collected for one optimization cycle.
   */
  void write(const PerformanceMetrics& metrics);

  /**
   * @brief Reads timing, iteration, cost, and residual statistics from acados.
   *
   * @param[out] metrics Metrics structure populated with the solver statistics.
   * @param[in] solver acados NLP solver instance.
   * @param[in] config acados NLP configuration.
   * @param[in] dims acados NLP dimensions.
   * @param[in] input acados NLP input.
   * @param[in] output acados NLP output.
   */
  static void collectSolverStatistics(PerformanceMetrics& metrics,
                                      ocp_nlp_solver* solver,
                                      ocp_nlp_config* config,
                                      ocp_nlp_dims* dims,
                                      ocp_nlp_in* input,
                                      ocp_nlp_out* output,
                                      bool collect_details);

  /**
   * @brief Collects the largest equality and inequality constraint violations from acados.
   *
   * @param[in,out] metrics Metrics structure populated with constraint diagnostics.
   * @param[in] solver acados NLP solver instance.
   * @param[in] dims acados NLP dimensions.
   * @param[in] boundary_constraints Number of leading nonlinear boundary constraints.
   * @param[in] obstacle_constraints Number of following nonlinear obstacle constraints.
   */
  static void collectConstraintDiagnostics(PerformanceMetrics& metrics,
                                           ocp_nlp_solver* solver,
                                           const ocp_nlp_dims* dims,
                                           int boundary_constraints,
                                           int obstacle_constraints);

  /**
   * @brief Returns the path of the CSV performance log.
   *
   * @return Path of the active performance log file.
   */
  const std::filesystem::path& path() const { return path_; }

 private:
  static constexpr uint64_t FLUSH_INTERVAL = 100;

  std::filesystem::path path_;
  std::filesystem::path active_file_;
  std::ofstream stream_;
  std::string run_id_;
  uint64_t records_since_flush_ = 0;
};

}  // namespace trajectory_optimization

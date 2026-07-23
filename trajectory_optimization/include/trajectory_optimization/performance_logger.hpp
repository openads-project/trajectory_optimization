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
  int status = 0;
  int sqp_iter = 0;
  int qp_iter = 0;
  int qp_status = 0;
  int reference_points = 0;
  int objects = 0;
  bool published = false;

  double cycle_ms = 0.0;
  double preprocessing_ms = 0.0;
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
  explicit PerformanceLogger(const std::string& node_name);
  ~PerformanceLogger();

  PerformanceLogger(const PerformanceLogger&) = delete;
  PerformanceLogger& operator=(const PerformanceLogger&) = delete;
  PerformanceLogger(PerformanceLogger&&) = delete;
  PerformanceLogger& operator=(PerformanceLogger&&) = delete;

  void write(const PerformanceMetrics& metrics);
  static void collectSolverStatistics(PerformanceMetrics& metrics,
                                      ocp_nlp_solver* solver,
                                      ocp_nlp_config* config,
                                      ocp_nlp_dims* dims,
                                      ocp_nlp_in* input,
                                      ocp_nlp_out* output,
                                      bool collect_details);
  void collectConstraintDiagnostics(PerformanceMetrics& metrics,
                                    ocp_nlp_solver* solver,
                                    const ocp_nlp_dims* dims,
                                    int obstacle_circles) const;
  const std::filesystem::path& path() const { return path_; }

 private:
  static constexpr uint64_t FLUSH_INTERVAL = 100;

  std::filesystem::path path_;
  std::ofstream stream_;
  uint64_t records_since_flush_ = 0;
};

}  // namespace trajectory_optimization

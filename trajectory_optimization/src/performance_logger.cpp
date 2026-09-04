// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <trajectory_optimization/performance_logger.hpp>

namespace trajectory_optimization {

namespace {

std::string timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc_time{};
  gmtime_r(&time, &utc_time);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

  std::ostringstream value;
  value << std::put_time(&utc_time, "%Y%m%dT%H%M%S") << '_' << std::setfill('0') << std::setw(3) << milliseconds << 'Z';
  return value.str();
}

int64_t nowNanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

PerformanceLogger::PerformanceLogger(const std::string& node_name) {
  const char* configured_directory = std::getenv("TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR");
  const std::filesystem::path directory = configured_directory != nullptr && *configured_directory != '\0'
                                              ? std::filesystem::path(configured_directory)
                                              : std::filesystem::temp_directory_path() / "trajectory_optimization_benchmarks";
  path_ = directory / (node_name + '_' + timestamp() + ".csv");

  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    throw std::runtime_error("could not create directory '" + directory.string() + "': " + error.message());
  }

  stream_.open(path_, std::ios::out | std::ios::trunc);
  if (!stream_) {
    throw std::runtime_error("could not open '" + path_.string() + "'");
  }
  stream_
      << "schema_version,source,run_id,cycle,record_stamp_ns,ego_stamp_ns,reference_stamp_ns,route_stamp_ns,status,"
         "published,ref_points,objects,relaxed_attempted,failure_phase,relaxed_status,sqp_iter,qp_iter,"
         "qp_status,cycle_ms,preprocessing_ms,solve_wall_ms,relaxed_solve_wall_ms,constrained_solve_wall_ms,postprocessing_ms,"
         "acados_total_ms,acados_lin_ms,"
         "acados_sim_ms,acados_qp_ms,"
         "acados_qp_solver_ms,acados_qp_xcond_ms,acados_reg_ms,acados_glob_ms,acados_preparation_ms,"
         "acados_feedback_ms,cost,kkt,nlp_res,res_stat,res_eq,res_ineq,res_comp,"
         "max_ineq_violation,max_ineq_stage,max_ineq_type,max_ineq_index,max_ineq_side,"
         "max_eq_violation,max_eq_stage,max_eq_state\n";
  stream_.flush();
}

PerformanceLogger::~PerformanceLogger() {
  stream_.flush();
  stream_.close();
}

void PerformanceLogger::collectSolverStatistics(PerformanceMetrics& metrics,
                                                ocp_nlp_solver* solver,
                                                ocp_nlp_config* config,
                                                ocp_nlp_dims* dims,
                                                ocp_nlp_in* input,
                                                ocp_nlp_out* output,
                                                bool collect_details) {
  ocp_nlp_eval_cost(solver, input, output);
  ocp_nlp_eval_residuals(solver, input, output);
  ocp_nlp_get(solver, "cost_value", &metrics.cost_value);
  ocp_nlp_get(solver, "res_eq", &metrics.res_eq);
  ocp_nlp_get(solver, "res_ineq", &metrics.res_ineq);

  if (!collect_details) return;

  ocp_nlp_get(solver, "res_stat", &metrics.res_stat);
  ocp_nlp_get(solver, "res_comp", &metrics.res_comp);
  metrics.nlp_res = std::max({metrics.res_stat, metrics.res_eq, metrics.res_ineq, metrics.res_comp});

  auto readTime = [&](const char* field, double& destination_ms) {
    double time_seconds = 0.0;
    ocp_nlp_get(solver, field, &time_seconds);
    destination_ms = time_seconds * 1000.0;
  };

  readTime("time_tot", metrics.acados_total_ms);
  readTime("time_lin", metrics.acados_lin_ms);
  readTime("time_sim", metrics.acados_sim_ms);
  readTime("time_qp", metrics.acados_qp_ms);
  readTime("time_qp_solver_call", metrics.acados_qp_solver_ms);
  readTime("time_qp_xcond", metrics.acados_qp_xcond_ms);
  readTime("time_reg", metrics.acados_reg_ms);
  readTime("time_glob", metrics.acados_glob_ms);
  readTime("time_preparation", metrics.acados_preparation_ms);
  readTime("time_feedback", metrics.acados_feedback_ms);
  ocp_nlp_get(solver, "sqp_iter", &metrics.sqp_iter);
  ocp_nlp_get(solver, "qp_iter", &metrics.qp_iter);
  ocp_nlp_get(solver, "qp_status", &metrics.qp_status);
  ocp_nlp_out_get(config, dims, output, 0, "kkt_norm_inf", &metrics.kkt_norm_inf);
}

void PerformanceLogger::collectConstraintDiagnostics(PerformanceMetrics& metrics,
                                                     ocp_nlp_solver* solver,
                                                     const ocp_nlp_dims* dims,
                                                     int object_boxes) {
  // The acados C API exposes stage-dependent dimensions as raw arrays.
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  for (int stage = 0; stage <= dims->N; ++stage) {
    std::vector<double> residuals(2 * dims->ni[stage]);
    ocp_nlp_get_at_stage(solver, stage, "ineq_fun", residuals.data());
    for (size_t index = 0; index < residuals.size(); ++index) {
      if (residuals[index] > metrics.max_ineq_violation) {
        metrics.max_ineq_violation = residuals[index];
        metrics.max_ineq_stage = stage;
        metrics.max_ineq_index = static_cast<int>(index);
      }
    }
  }

  for (int stage = 0; stage < dims->N; ++stage) {
    std::vector<double> residuals(dims->nx[stage + 1]);
    ocp_nlp_get_at_stage(solver, stage, "res_eq", residuals.data());
    for (size_t state = 0; state < residuals.size(); ++state) {
      if (std::abs(residuals[state]) > metrics.max_eq_violation) {
        metrics.max_eq_violation = std::abs(residuals[state]);
        metrics.max_eq_stage = stage;
        metrics.max_eq_state = static_cast<int>(state);
      }
    }
  }

  if (metrics.max_ineq_stage < 0) return;

  const int ni = dims->ni[metrics.max_ineq_stage];
  const int nb = dims->nb[metrics.max_ineq_stage];
  const int ng = dims->ng[metrics.max_ineq_stage];
  const int nh = ni - nb - ng - dims->ns[metrics.max_ineq_stage];
  const int index = metrics.max_ineq_index % ni;
  metrics.max_ineq_side = metrics.max_ineq_index < ni ? "lower" : "upper";
  metrics.max_ineq_index = index;
  if (index < nb) {
    std::vector<int> bound_indices(nb);
    ocp_nlp_get_at_stage(solver, metrics.max_ineq_stage, "idxb", bound_indices.data());
    const int variable_index = bound_indices[index];
    if (variable_index < dims->nu[metrics.max_ineq_stage]) {
      metrics.max_ineq_type = "control";
      metrics.max_ineq_index = variable_index;
    } else {
      metrics.max_ineq_type = "state";
      metrics.max_ineq_index = variable_index - dims->nu[metrics.max_ineq_stage];
    }
  } else if (index < nb + ng) {
    metrics.max_ineq_type = "linear";
  } else if (index < nb + ng + nh) {
    const int h_index = index - nb - ng;
    if (h_index < 2) {
      metrics.max_ineq_type = h_index == 0 ? "boundary_left" : "boundary_right";
      metrics.max_ineq_index = 0;
    } else if (h_index < object_boxes + 2) {
      metrics.max_ineq_type = "object";
      metrics.max_ineq_index = h_index - 2;
    } else {
      metrics.max_ineq_type = "vehicle";
      metrics.max_ineq_index = h_index - object_boxes - 2;
    }
  } else {
    metrics.max_ineq_type = "slack";
    metrics.max_ineq_index = index - nb - ng - nh;
  }
  // NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
}

void PerformanceLogger::write(const PerformanceMetrics& metrics) {
  stream_ << std::setprecision(17) << 6 << ",runtime,," << metrics.cycle << ',' << nowNanoseconds() << ',' << metrics.ego_stamp_ns
          << ',' << metrics.reference_stamp_ns << ',' << metrics.route_stamp_ns << ',' << metrics.status << ','
          << (metrics.published ? 1 : 0) << ',' << metrics.reference_points << ',' << metrics.objects << ','
          << (metrics.relaxed_attempted ? 1 : 0) << ',' << metrics.failure_phase << ',' << metrics.relaxed_status << ','
          << metrics.sqp_iter << ',' << metrics.qp_iter << ',' << metrics.qp_status << ',' << metrics.cycle_ms << ','
          << metrics.preprocessing_ms << ',' << metrics.solve_wall_ms << ',' << metrics.relaxed_solve_wall_ms << ','
          << metrics.constrained_solve_wall_ms << ',' << metrics.postprocessing_ms << ',' << metrics.acados_total_ms << ','
          << metrics.acados_lin_ms << ',' << metrics.acados_sim_ms << ',' << metrics.acados_qp_ms << ','
          << metrics.acados_qp_solver_ms << ',' << metrics.acados_qp_xcond_ms << ',' << metrics.acados_reg_ms << ','
          << metrics.acados_glob_ms << ',' << metrics.acados_preparation_ms << ',' << metrics.acados_feedback_ms << ','
          << metrics.cost_value << ',' << metrics.kkt_norm_inf << ',' << metrics.nlp_res << ',' << metrics.res_stat << ','
          << metrics.res_eq << ',' << metrics.res_ineq << ',' << metrics.res_comp << ',' << metrics.max_ineq_violation << ','
          << metrics.max_ineq_stage << ',' << metrics.max_ineq_type << ',' << metrics.max_ineq_index << ','
          << metrics.max_ineq_side << ',' << metrics.max_eq_violation << ',' << metrics.max_eq_stage << ','
          << metrics.max_eq_state << '\n';

  if (++records_since_flush_ >= FLUSH_INTERVAL) {
    stream_.flush();
    records_since_flush_ = 0;
  }
}

}  // namespace trajectory_optimization

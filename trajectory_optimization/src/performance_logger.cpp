// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

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
  stream_ << "schema_version,source,run_id,cycle,record_stamp_ns,ego_stamp_ns,reference_stamp_ns,route_stamp_ns,status,"
             "published,ref_points,objects,sqp_iter,qp_iter,"
             "qp_status,cycle_ms,preprocessing_ms,solve_wall_ms,postprocessing_ms,acados_total_ms,acados_lin_ms,"
             "acados_sim_ms,acados_qp_ms,"
             "acados_qp_solver_ms,acados_qp_xcond_ms,acados_reg_ms,acados_glob_ms,acados_preparation_ms,"
             "acados_feedback_ms,cost,kkt,nlp_res,res_stat,res_eq,res_ineq,res_comp\n";
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
                                                ocp_nlp_out* output) {
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

  ocp_nlp_eval_cost(solver, input, output);
  ocp_nlp_eval_residuals(solver, input, output);
  ocp_nlp_get(solver, "cost_value", &metrics.cost_value);
  ocp_nlp_get(solver, "res_stat", &metrics.res_stat);
  ocp_nlp_get(solver, "res_eq", &metrics.res_eq);
  ocp_nlp_get(solver, "res_ineq", &metrics.res_ineq);
  ocp_nlp_get(solver, "res_comp", &metrics.res_comp);
  metrics.nlp_res = std::max({metrics.res_stat, metrics.res_eq, metrics.res_ineq, metrics.res_comp});
}

void PerformanceLogger::write(const PerformanceMetrics& metrics) {
  stream_ << std::setprecision(17) << 4 << ",runtime,," << metrics.cycle << ',' << nowNanoseconds() << ',' << metrics.ego_stamp_ns
          << ',' << metrics.reference_stamp_ns << ',' << metrics.route_stamp_ns << ',' << metrics.status << ','
          << (metrics.published ? 1 : 0) << ',' << metrics.reference_points << ',' << metrics.objects << ',' << metrics.sqp_iter
          << ',' << metrics.qp_iter << ',' << metrics.qp_status << ',' << metrics.cycle_ms << ',' << metrics.preprocessing_ms
          << ',' << metrics.solve_wall_ms << ',' << metrics.postprocessing_ms << ',' << metrics.acados_total_ms << ','
          << metrics.acados_lin_ms << ',' << metrics.acados_sim_ms << ',' << metrics.acados_qp_ms << ','
          << metrics.acados_qp_solver_ms << ',' << metrics.acados_qp_xcond_ms << ',' << metrics.acados_reg_ms << ','
          << metrics.acados_glob_ms << ',' << metrics.acados_preparation_ms << ',' << metrics.acados_feedback_ms << ','
          << metrics.cost_value << ',' << metrics.kkt_norm_inf << ',' << metrics.nlp_res << ',' << metrics.res_stat << ','
          << metrics.res_eq << ',' << metrics.res_ineq << ',' << metrics.res_comp << '\n';

  if (++records_since_flush_ >= FLUSH_INTERVAL) {
    stream_.flush();
    records_since_flush_ = 0;
  }
}

}  // namespace trajectory_optimization

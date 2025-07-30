#pragma once

// acados
#include <acados/utils/math.h>
#include <acados/utils/print.h>
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

// models
#include <acados_ocp/acados_solver_passat_cc.h>
#include <acados_ocp/acados_solver_karl.h>
#include <acados_ocp/acados_solver_auto_shuttle.h>
#include <acados_ocp/acados_solver_omni_shuttle.h>

namespace trajectory_optimization {

typedef std::variant<passat_cc_solver_capsule*, karl_solver_capsule*, auto_shuttle_solver_capsule*, omni_shuttle_solver_capsule*> ocp_model_capsule_t;

inline ocp_model_capsule_t acados_create_capsule(const std::string& model_name) {
  if (model_name == "passat_cc") {
    return passat_cc_acados_create_capsule();
  } else if (model_name == "karl") {
    return karl_acados_create_capsule();
  } else if (model_name == "auto_shuttle") {
    return auto_shuttle_acados_create_capsule();
  } else if (model_name == "omni_shuttle") {
    return omni_shuttle_acados_create_capsule();
  } else {
    throw std::invalid_argument("Invalid model name: " + model_name);
  }
}

#define ACADOS_DISPATCH(function_name, ...)                                                              \
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {                                      \
    return passat_cc_##function_name(std::get<passat_cc_solver_capsule*>(capsule), ##__VA_ARGS__);       \
  } else if (std::holds_alternative<karl_solver_capsule*>(capsule)) {                                    \
    return karl_##function_name(std::get<karl_solver_capsule*>(capsule), ##__VA_ARGS__);                 \
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {                            \
    return auto_shuttle_##function_name(std::get<auto_shuttle_solver_capsule*>(capsule), ##__VA_ARGS__); \
  } else if (std::holds_alternative<omni_shuttle_solver_capsule*>(capsule)) {                            \
    return omni_shuttle_##function_name(std::get<omni_shuttle_solver_capsule*>(capsule), ##__VA_ARGS__); \
  } else {                                                                                               \
    throw std::invalid_argument("Invalid capsule type.");                                                \
  }

inline int acados_create(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_create); }

inline int acados_create_with_discretization(ocp_model_capsule_t capsule, int n_time_steps, double* new_time_steps) {
  ACADOS_DISPATCH(acados_create_with_discretization, n_time_steps, new_time_steps);
}

inline int acados_free_capsule(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_free_capsule); }

inline int acados_free(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_free); }

inline int acados_solve(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_solve); }

inline int acados_update_params_sparse(ocp_model_capsule_t capsule, int stage, int* idx, double* p, int n_update) {
  ACADOS_DISPATCH(acados_update_params_sparse, stage, idx, p, n_update);
}

inline void acados_print_stats(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_print_stats); }

inline ocp_nlp_in* acados_get_nlp_in(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_in); }

inline ocp_nlp_out* acados_get_nlp_out(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_out); }

inline ocp_nlp_solver* acados_get_nlp_solver(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_solver); }

inline ocp_nlp_config* acados_get_nlp_config(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_config); }

inline void* acados_get_nlp_opts(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_opts); }

inline ocp_nlp_dims* acados_get_nlp_dims(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_dims); }

}  // namespace trajectory_optimization

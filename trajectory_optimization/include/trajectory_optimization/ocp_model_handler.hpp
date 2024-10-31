#pragma once

// acados
#include <acados/utils/math.h>
#include <acados/utils/print.h>
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

// models
#include <acados_ocp/acados_solver_passat_cc.h>
#include <acados_ocp/acados_solver_auto_shuttle.h>

namespace trajectory_optimization {

typedef std::variant<passat_cc_solver_capsule*, auto_shuttle_solver_capsule*> ocp_model_capsule_t;

ocp_model_capsule_t acados_create_capsule(const std::string& model_name) {
  if (model_name == "passat_cc") {
    return passat_cc_acados_create_capsule();
  } else if (model_name == "auto_shuttle") {
    return auto_shuttle_acados_create_capsule();
  } else {
    throw std::invalid_argument("Invalid model name: " + model_name);
  }
}

int acados_create_with_discretization(ocp_model_capsule_t capsule, int n_time_steps, double* new_time_steps) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_create_with_discretization(std::get<passat_cc_solver_capsule*>(capsule), n_time_steps, new_time_steps);
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_create_with_discretization(std::get<auto_shuttle_solver_capsule*>(capsule), n_time_steps, new_time_steps);
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

int acados_free_capsule(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_free_capsule(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_free_capsule(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

int acados_free(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_free(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_free(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

int acados_solve(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_solve(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_solve(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

int acados_update_params_sparse(ocp_model_capsule_t capsule, int stage, int *idx, double *p, int n_update) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_update_params_sparse(std::get<passat_cc_solver_capsule*>(capsule), stage, idx, p, n_update);
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_update_params_sparse(std::get<auto_shuttle_solver_capsule*>(capsule), stage, idx, p, n_update);
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

void acados_print_stats(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    passat_cc_acados_print_stats(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    auto_shuttle_acados_print_stats(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

ocp_nlp_in* acados_get_nlp_in(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_in(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_in(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

ocp_nlp_out* acados_get_nlp_out(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_out(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_out(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

ocp_nlp_solver* acados_get_nlp_solver(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_solver(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_solver(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

ocp_nlp_config* acados_get_nlp_config(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_config(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_config(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

void* acados_get_nlp_opts(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_opts(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_opts(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

ocp_nlp_dims* acados_get_nlp_dims(ocp_model_capsule_t capsule) {
  if (std::holds_alternative<passat_cc_solver_capsule*>(capsule)) {
    return passat_cc_acados_get_nlp_dims(std::get<passat_cc_solver_capsule*>(capsule));
  } else if (std::holds_alternative<auto_shuttle_solver_capsule*>(capsule)) {
    return auto_shuttle_acados_get_nlp_dims(std::get<auto_shuttle_solver_capsule*>(capsule));
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
}

}  // namespace trajectory_optimization

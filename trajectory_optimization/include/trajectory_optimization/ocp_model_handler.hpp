// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>

// NOLINTBEGIN(clang-diagnostic-gnu-zero-variadic-macro-arguments)

// acados
#include <acados/utils/math.h>
#include <acados/utils/print.h>
#include <acados_c/external_function_interface.h>
#include <acados_c/ocp_nlp_interface.h>
#include <blasfeo_d_aux_ext_dep.h>  // for printing dense matrices

// models
#include <trajectory_optimization_ocp/acados_solver_karl.h>
#include <trajectory_optimization_ocp/acados_solver_shuttle.h>

namespace trajectory_optimization {

typedef std::variant<karl_solver_capsule*, karl_safety_solver_capsule*, shuttle_solver_capsule*> ocp_model_capsule_t;

/**
 * @brief Creates the model-specific acados capsule selected by the configured model name.
 *
 * @param[in] model_name Name of the generated acados model.
 * @return Variant containing the concrete acados capsule pointer.
 * @throws std::invalid_argument If no generated model matches the given name.
 */
inline ocp_model_capsule_t acados_create_capsule(const std::string& model_name) {
  if (model_name == "karl") {
    return karl_acados_create_capsule();
  } else if (model_name == "karl_safety") {
    return karl_safety_acados_create_capsule();
  } else if (model_name == "shuttle") {
    return shuttle_acados_create_capsule();
  } else {
    throw std::invalid_argument("Invalid model name: " + model_name);
  }
}

#define ACADOS_DISPATCH(function_name, ...)                                                            \
  if (std::holds_alternative<karl_solver_capsule*>(capsule)) {                                         \
    return karl_##function_name(std::get<karl_solver_capsule*>(capsule), ##__VA_ARGS__);               \
  } else if (std::holds_alternative<karl_safety_solver_capsule*>(capsule)) {                           \
    return karl_safety_##function_name(std::get<karl_safety_solver_capsule*>(capsule), ##__VA_ARGS__); \
  } else if (std::holds_alternative<shuttle_solver_capsule*>(capsule)) {                               \
    return shuttle_##function_name(std::get<shuttle_solver_capsule*>(capsule), ##__VA_ARGS__);         \
  } else {                                                                                             \
    throw std::invalid_argument("Invalid capsule type.");                                              \
  }

/**
 * @brief Wrapper around the generated acados create function for the selected model.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Status code returned by the generated acados function.
 */
inline int acados_create(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_create); }

/**
 * @brief Wrapper around the generated acados create function with custom discretization.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @param[in] n_time_steps Number of shooting intervals.
 * @param[in] new_time_steps Time-step array passed through to acados.
 * @return Status code returned by the generated acados function.
 */
inline int acados_create_with_discretization(ocp_model_capsule_t capsule, int n_time_steps, double* new_time_steps) {
  ACADOS_DISPATCH(acados_create_with_discretization, n_time_steps, new_time_steps);
}

/**
 * @brief Wrapper around the generated acados capsule cleanup function.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Status code returned by the generated acados function.
 */
inline int acados_free_capsule(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_free_capsule); }

/**
 * @brief Wrapper around the generated acados solver cleanup function.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Status code returned by the generated acados function.
 */
inline int acados_free(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_free); }

/**
 * @brief Wrapper around the generated acados solve function.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Status code returned by the generated acados function.
 */
inline int acados_solve(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_solve); }

/**
 * @brief Evaluates the explicit model dynamics for a shooting stage.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @param[in] stage Shooting stage whose dynamics function is evaluated.
 * @param[in] x State vector at which to evaluate the dynamics.
 * @param[in] u Control vector at which to evaluate the dynamics.
 * @param[out] x_dot Evaluated state derivative.
 */
inline void acados_evaluate_dynamics(ocp_model_capsule_t capsule, int stage, double* x, double* u, double* x_dot) {
  std::array<ext_fun_arg_t, 2> input_types = {COLMAJ, COLMAJ};
  std::array<void*, 2> inputs = {x, u};
  std::array<ext_fun_arg_t, 1> output_types = {COLMAJ};
  std::array<void*, 1> outputs = {x_dot};

  external_function_external_param_casadi* function = nullptr;

  if (std::holds_alternative<karl_solver_capsule*>(capsule)) {
    function = &std::get<karl_solver_capsule*>(capsule)->expl_ode_fun[stage];  // NOLINT
  } else if (std::holds_alternative<karl_safety_solver_capsule*>(capsule)) {
    function = &std::get<karl_safety_solver_capsule*>(capsule)->expl_ode_fun[stage];  // NOLINT
  } else if (std::holds_alternative<shuttle_solver_capsule*>(capsule)) {
    function = &std::get<shuttle_solver_capsule*>(capsule)->expl_ode_fun[stage];  // NOLINT
  } else {
    throw std::invalid_argument("Invalid capsule type.");
  }
  function->evaluate(function, input_types.data(), inputs.data(), output_types.data(), outputs.data());
}

/**
 * @brief Resets selected solver memory without freeing and recreating the capsule.
 */
inline int acados_reset(ocp_model_capsule_t capsule,
                        int reset_qp_solver_mem,
                        int reset_numerical_values,
                        int reset_solver_options,
                        int reset_x_to_x0_bar) {
  ACADOS_DISPATCH(acados_reset, reset_qp_solver_mem, reset_numerical_values, reset_solver_options, reset_x_to_x0_bar);
}

/**
 * @brief Wrapper around the generated sparse parameter update function.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @param[in] stage Shooting stage to update.
 * @param[in] idx Indices of the parameters to overwrite.
 * @param[in] p Parameter values written to the selected indices.
 * @param[in] n_update Number of updated parameter entries.
 * @return Status code returned by the generated acados function.
 */
inline int acados_update_params_sparse(ocp_model_capsule_t capsule, int stage, int* idx, double* p, int n_update) {
  ACADOS_DISPATCH(acados_update_params_sparse, stage, idx, p, n_update);
}

/**
 * @brief Wrapper around the generated global-parameter update function.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @param[in] data Global parameter buffer.
 * @param[in] data_len Number of entries in `data`.
 * @return Status code returned by the generated acados function.
 */
inline int acados_set_p_global_and_precompute_dependencies(ocp_model_capsule_t capsule, double* data, int data_len) {
  ACADOS_DISPATCH(acados_set_p_global_and_precompute_dependencies, data, data_len);
}

/**
 * @brief Wrapper around the generated acados statistics printer.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 */
inline void acados_print_stats(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_print_stats); }

/**
 * @brief Wrapper around the generated accessor for the OCP input structure.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific `ocp_nlp_in`.
 */
inline ocp_nlp_in* acados_get_nlp_in(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_in); }

/**
 * @brief Wrapper around the generated accessor for the OCP output structure.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific `ocp_nlp_out`.
 */
inline ocp_nlp_out* acados_get_nlp_out(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_out); }

/**
 * @brief Wrapper around the generated accessor for the OCP solver instance.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific `ocp_nlp_solver`.
 */
inline ocp_nlp_solver* acados_get_nlp_solver(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_solver); }

/**
 * @brief Wrapper around the generated accessor for the OCP configuration.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific `ocp_nlp_config`.
 */
inline ocp_nlp_config* acados_get_nlp_config(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_config); }

/**
 * @brief Wrapper around the generated accessor for the OCP solver options.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific options object.
 */
inline void* acados_get_nlp_opts(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_opts); }

/**
 * @brief Returns the common NLP options stored inside the generated solver-specific options.
 */
inline const ocp_nlp_opts* acados_get_common_nlp_opts(ocp_nlp_config* config, void* solver_opts) {
  void* common_opts = nullptr;
  config->opts_get(config, solver_opts, "nlp_opts", &common_opts);
  return static_cast<ocp_nlp_opts*>(common_opts);
}

/**
 * @brief Wrapper around the generated accessor for the OCP dimensions.
 *
 * @param[in] capsule Variant holding the model-specific acados capsule.
 * @return Pointer to the model-specific `ocp_nlp_dims`.
 */
inline ocp_nlp_dims* acados_get_nlp_dims(ocp_model_capsule_t capsule) { ACADOS_DISPATCH(acados_get_nlp_dims); }

}  // namespace trajectory_optimization

// NOLINTEND(clang-diagnostic-gnu-zero-variadic-macro-arguments)

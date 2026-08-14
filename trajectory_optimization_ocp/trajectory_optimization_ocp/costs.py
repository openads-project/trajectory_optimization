# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpCost
from constants import (
    CONTROL_INDEX_ALPHA_F,
    CONTROL_INDEX_ALPHA_R,
    CONTROL_INDEX_J_T,
    STATE_INDEX_A_T,
    STATE_INDEX_DELTA_F,
    STATE_INDEX_DELTA_R,
    STATE_INDEX_PSI,
    STATE_INDEX_V_T,
    STATE_INDEX_X,
    STATE_INDEX_Y,
    V_SCALE_MIN,
)
from utils import determine_spacially_matched_ref_path_point, stable_tan, wrap_angle


def set_costs(ocp: AcadosOcp, config):
    """Set up cost functions for the optimal control problem.

    Combines multiple seperate defined cost terms into a single external cost function for the initial,
    intermediate and terminal shooting nodes. The cost terms include reference path tracking costs,
    acceleration costs and control variable costs. The cost function is defined as a weighted sum of the individual cost terms,
    where the weights are defined as parameters that can be modified at runtime.

    Args:
        ocp: The Acados OCP object to configure.
        config: Configuration dictionary containing cost parameters and model settings.
    """
    # set up as external cost function
    cost = AcadosOcpCost()
    cost.cost_type = "EXTERNAL"
    cost.cost_type_0 = "EXTERNAL"
    cost.cost_type_e = "EXTERNAL"
    ocp.cost = cost

    # initialize parameters
    n_params_dynamic_weight = np.prod(config["p_dynamic_weight_shape"])
    n_params_boundary_activation = np.prod(config["p_boundary_activation_shape"])
    n_params_objects = np.prod(config["p_objects_shape"])
    # total number of parameters
    n_params = n_params_dynamic_weight + n_params_boundary_activation + n_params_objects
    #  set initial parameter values
    ocp.parameter_values = np.zeros(n_params)
    # get parameters
    idx_params = 0
    p_dynamic_weight = ocp.model.p[idx_params : (idx_params := idx_params + n_params_dynamic_weight)]
    _ = ocp.model.p[
        idx_params : (idx_params := idx_params + n_params_boundary_activation)
    ]  # boundary activation, not used in cost function
    # p_objects = ocp.model.p[idx_params:(idx_params := idx_params + n_params_objects)] # not used in cost function
    assert idx_params == n_params - n_params_objects

    # initialize global parameters
    n_params_cost_weights = np.prod(config["p_cost_weights_shape"])
    n_params_cost_params = np.prod(config["p_cost_params_shape"])
    n_params_ref_path = np.prod(config["p_ref_path_shape"])
    # total number of global parameters
    n_global_params = n_params_cost_weights + n_params_cost_params + n_params_ref_path
    #  set initial global parameter values
    ocp.p_global_values = np.zeros(n_global_params)
    # get global parameters
    idx_global_params = 0
    p_cost_weights = ocp.model.p_global[idx_global_params : (idx_global_params := idx_global_params + n_params_cost_weights)]
    _ = ocp.model.p_global[
        idx_global_params : (idx_global_params := idx_global_params + n_params_cost_params)
    ]  # cost_params, not used in cost function
    p_ref_path = ocp.model.p_global[idx_global_params : (idx_global_params := idx_global_params + n_params_ref_path)]
    assert idx_global_params == n_global_params

    # cost term weights
    w_lat = p_cost_weights[0]
    w_psi = p_cost_weights[1]
    w_overspeed = p_cost_weights[2]
    w_underspeed = p_cost_weights[3]
    w_a_n = p_cost_weights[4]
    w_a_t_pos = p_cost_weights[5]
    w_a_t_neg = p_cost_weights[6]
    w_j_n = p_cost_weights[7]
    w_j_t_pos = p_cost_weights[8]
    w_j_t_neg = p_cost_weights[9]
    w_alpha = p_cost_weights[10]
    w_end_yaw = p_cost_weights[11]

    # === External cost function ===

    # calculate quantities needed for cost terms
    interpolated_state_ref = determine_spacially_matched_ref_path_point(
        config, p_ref_path, ocp.model.x[STATE_INDEX_X], ocp.model.x[STATE_INDEX_Y]
    )

    # calculate cost terms
    ref_path_costs = calc_ref_path_cost(ocp, config, interpolated_state_ref)
    a_costs = calc_acceleration_cost(ocp, config)
    control_costs = calc_control_cost(ocp, config)

    # --- Define costs at intermediate nodes ---
    # reference path costs
    ocp.model.cost_expr_ext_cost = p_dynamic_weight * w_lat * ref_path_costs["dlat"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_overspeed * ref_path_costs["overspeed"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_underspeed * ref_path_costs["underspeed"]
    # acceleration costs
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_t_pos * a_costs["a_t_pos"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_t_neg * a_costs["a_t_neg"]
    ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_a_n * a_costs["a_n"]
    # control variable costs
    ocp.model.cost_expr_ext_cost += w_j_t_pos * control_costs["j_t_pos"]
    ocp.model.cost_expr_ext_cost += w_j_t_neg * control_costs["j_t_neg"]
    ocp.model.cost_expr_ext_cost += w_j_n * control_costs["j_n"]
    ocp.model.cost_expr_ext_cost += w_alpha * control_costs["alpha_f"]
    # cost exclusivly for RWS
    if config["model_type"] == "RWS":
        ocp.model.cost_expr_ext_cost += p_dynamic_weight * w_psi * ref_path_costs["psi"]
        ocp.model.cost_expr_ext_cost += w_alpha * control_costs["alpha_r"]

    # --- Define costs at initial shooting node ---
    ocp.model.cost_expr_ext_cost_0 = ocp.model.cost_expr_ext_cost

    # --- Define costs at terminal shooting node ---
    ocp.model.cost_expr_ext_cost_e = w_end_yaw * ref_path_costs["psi"]


def calc_ref_path_cost(ocp: AcadosOcp, config: dict, ref_inter: dict) -> dict:
    """Compute normalized reference-tracking cost terms.

    Args:
        ocp: Configured OCP object containing symbolic state variables.
        config: Configuration dictionary with normalization constants.
        ref_inter: Interpolated reference state at the current position.

    Returns:
        Dictionary with normalized terms for lateral error, yaw error, and velocity errors for over- and underspeeding.
    """

    # lateral deviation term
    dlat = ca.sqrt(
        ca.power(ocp.model.x[STATE_INDEX_X] - ref_inter["x"], 2) + ca.power(ocp.model.x[STATE_INDEX_Y] - ref_inter["y"], 2)
    )
    dlat_term = ca.power(dlat, 2) / ca.power(config["c_lat"], 2)

    # v deviation term
    # first ensure that the reference velocity is > 0
    v_ref = ca.fmax(ref_inter["v"], 0.0)
    # we define the scaling value of v to v_ref: a velocity deviation of v_ref leads to a cost of 1
    # for numeric stability (low reference speeds) we ensure that v_scale > V_SCALE_MIN > 0
    v_scale = ca.fmax(v_ref, V_SCALE_MIN)
    v_dev = v_ref - ocp.model.x[STATE_INDEX_V_T]
    v_term = ca.power(v_dev, 2) / ca.power(v_scale, 2)
    overspeed_term = ca.if_else(v_dev < 0, v_term, 0.0)
    underspeed_term = ca.if_else(v_dev > 0, v_term, 0.0)

    # psi deviation term
    psi_term = ca.power(wrap_angle(ocp.model.x[STATE_INDEX_PSI] - ref_inter["psi"]), 2) / ca.power(config["c_psi"], 2)

    cost_terms = {"dlat": dlat_term, "psi": psi_term, "overspeed": overspeed_term, "underspeed": underspeed_term}
    return cost_terms


def calc_control_cost(ocp: AcadosOcp, config: dict) -> dict:
    """Compute normalized control-related cost terms.

    Includes positive/negative tangential jerk, nominal lateral jerk, and
    steering-rate penalties (front and rear for RWS).

    Args:
        ocp: Configured OCP object containing symbolic state/control variables.
        config: Configuration dictionary with model type and scaling constants.

    Returns:
        Dictionary with normalized control cost terms.
    """
    j_t_pos = ca.fmax(0, ocp.model.u[CONTROL_INDEX_J_T])
    j_t_pos_term = ca.power(j_t_pos, 2) / ca.power(config["c_j_t"], 2)
    j_t_neg = ca.fmin(0, ocp.model.u[CONTROL_INDEX_J_T])
    j_t_neg_term = ca.power(j_t_neg, 2) / ca.power(config["c_j_t"], 2)

    # derive nominal jerk j_n = d(a_n)/dt =d(v_t * psi_dot)/dt = a_t * psi_dot + v_t * psi_ddot (second derivative)
    psi_dot = compute_psi_dot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_dot_Ack(ocp, config)
    psi_ddot = compute_psi_ddot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_ddot_Ack(ocp, config)

    j_n = ocp.model.x[STATE_INDEX_A_T] * psi_dot + ocp.model.x[STATE_INDEX_V_T] * psi_ddot
    j_n_term = ca.power(j_n, 2) / ca.power(config["c_j_n"], 2)

    alpha_f_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA_F], 2) / ca.power(config["c_alpha"], 2)
    alpha_r_term = compute_alpha_r_cost_RWS(ocp, config) if config["model_type"] == "RWS" else 0.0

    cost_terms = {
        "j_t_pos": j_t_pos_term,
        "j_t_neg": j_t_neg_term,
        "j_n": j_n_term,
        "alpha_f": alpha_f_term,
        "alpha_r": alpha_r_term,
    }
    return cost_terms


def calc_acceleration_cost(ocp: AcadosOcp, config: dict) -> dict:
    """Compute normalized acceleration-related cost terms.

    Includes positive/negative tangential acceleration and nominal lateral
    acceleration derived from yaw rate and speed.

    Args:
        ocp: Configured OCP object containing symbolic state variables.
        config: Configuration dictionary with model type and scaling constants.

    Returns:
        Dictionary with normalized acceleration cost terms.
    """

    # tangential acceleration is given as state variable
    a_t_pos = ca.fmax(0, ocp.model.x[STATE_INDEX_A_T])
    a_t_pos_term = ca.power(a_t_pos, 2) / ca.power(config["c_a_t"], 2)
    a_t_neg = ca.fmin(0, ocp.model.x[STATE_INDEX_A_T])
    a_t_neg_term = ca.power(a_t_neg, 2) / ca.power(config["c_a_t"], 2)

    # derive nominal acceleration a_n = psi_dot * v
    psi_dot = compute_psi_dot_RWS(ocp, config) if config["model_type"] == "RWS" else compute_psi_dot_Ack(ocp, config)
    a_n = psi_dot * ocp.model.x[STATE_INDEX_V_T]
    a_n_term = ca.power(a_n, 2) / ca.power(config["c_a_n"], 2)

    cost_terms = {"a_n": a_n_term, "a_t_pos": a_t_pos_term, "a_t_neg": a_t_neg_term}
    return cost_terms


# === Helper functions ===


def compute_side_slip_angle(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Computes the vehicle side slip (angle between vehicle frame and absolute velocity vector).

    Ackermann: beta = 0.0
    RWS: beta = atan( L_r / (L_f + L_r) * delta_f +  L_f / (L_f + L_r) * delta_r)
    """
    if config["model_type"] == "Ackermann":
        return 0.0
    # RWS
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = ca.atan(
        (L_r / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
        + (L_f / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])
    )
    return beta


def compute_alpha_r_cost_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Compute normalized rear steering-rate penalty term for RWS."""
    alpha_r_term = ca.power(ocp.model.u[CONTROL_INDEX_ALPHA_R], 2) / ca.power(config["c_alpha"], 2)
    return alpha_r_term


def compute_psi_dot_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Computes the yaw rate for for a vehicle with RWS.

    psi_dot = v_t * cos(beta) * (tan(delta_f) - tan(delta_r)) / (L_f + L_r)
    """
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = compute_side_slip_angle(ocp, config)
    psi_dot = (
        ocp.model.x[STATE_INDEX_V_T]
        * ca.cos(beta)
        * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
        / (L_f + L_r)
    )
    return psi_dot


def compute_psi_dot_Ack(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Computes the yaw rate for a vehicle with Ackermann steering.

    psi_dot = v_t / L * tan(delta)
    """
    L = config["wheelbase"]
    tan_delta = stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
    psi_dot = ocp.model.x[STATE_INDEX_V_T] / L * tan_delta
    return psi_dot


def compute_psi_ddot_RWS(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Computes the second derivative of yaw angle (yaw acceleration) for RWS.

    ψ̈ = (∂ψ̇/∂v) * ∂v/∂t + (∂ψ̇/∂β) * (∂β/∂t) + (∂ψ̇/∂δ_f) * (∂δ_f/∂t) + (∂ψ̇/∂δ_r) * (∂δ_r/∂t)
    """
    L_f = config["distance_cg_front_axle"]
    L_r = config["distance_cg_rear_axle"]
    beta = compute_side_slip_angle(ocp, config)
    # Compute beta_dot (rate of change of beta which is a function of delta_f and delta_r)
    beta_dot = (
        1
        / (
            1
            + ((L_r * ca.tan(ocp.model.x[STATE_INDEX_DELTA_F]) + L_f * ca.tan(ocp.model.x[STATE_INDEX_DELTA_R])) / (L_f + L_r))
            ** 2
        )
        * (
            (
                L_r * ocp.model.u[CONTROL_INDEX_ALPHA_F] / ca.cos(ocp.model.x[STATE_INDEX_DELTA_F]) ** 2
                + L_f * ocp.model.u[CONTROL_INDEX_ALPHA_R] / ca.cos(ocp.model.x[STATE_INDEX_DELTA_R]) ** 2
            )
            / (L_f + L_r)
        )
    )

    # Compute partial derivatives
    d_psi_dot_d_v = (
        ocp.model.x[STATE_INDEX_A_T]
        * ca.cos(beta)
        * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
        / (L_f + L_r)
    )

    d_psi_dot_d_beta = (
        -ocp.model.x[STATE_INDEX_V_T]
        * ca.sin(beta)
        * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
        / (L_f + L_r)
    )

    d_psi_dot_d_delta_f = (
        ocp.model.x[STATE_INDEX_V_T]
        * ca.cos(beta)
        * ocp.model.u[CONTROL_INDEX_ALPHA_F]
        / (ca.cos(ocp.model.x[STATE_INDEX_DELTA_F]) ** 2)
        / (L_f + L_r)
    )

    d_psi_dot_d_delta_r = (
        -ocp.model.x[STATE_INDEX_V_T]
        * ca.cos(beta)
        * ocp.model.u[CONTROL_INDEX_ALPHA_R]
        / (ca.cos(ocp.model.x[STATE_INDEX_DELTA_R]) ** 2)
        / (L_f + L_r)
    )

    # Compute final yaw acceleration
    yaw_ddot = d_psi_dot_d_v + d_psi_dot_d_beta * beta_dot + d_psi_dot_d_delta_f + d_psi_dot_d_delta_r
    return yaw_ddot


def compute_psi_ddot_Ack(ocp: AcadosOcp, config: dict) -> ca.MX:
    """Computes the second derivative of yaw angle (yaw acceleration) for Ackermann steering:

    ψ̈ = (∂ψ̇/∂v) * ∂v/∂t + (∂ψ̇/∂δ_f) * (∂δ_f/∂t)
    """
    L = config["wheelbase"]
    tan_delta = stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
    psi_ddot = ocp.model.x[STATE_INDEX_A_T] / L * tan_delta + ocp.model.x[STATE_INDEX_V_T] / L * ocp.model.u[
        CONTROL_INDEX_ALPHA_F
    ] * (1 + tan_delta**2)
    return psi_ddot

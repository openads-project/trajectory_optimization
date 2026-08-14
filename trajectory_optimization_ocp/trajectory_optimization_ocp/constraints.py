# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import casadi as ca
import numpy as np
from acados_template import ACADOS_INFTY, AcadosOcp, AcadosOcpConstraints
from constants import (
    CONTROL_INDEX_ALPHA_F,
    CONTROL_INDEX_ALPHA_R,
    CONTROL_INDEX_J_T,
    P_OBJECTS_INDEX_ACTIVE,
    P_OBJECTS_INDEX_HALF_LENGTH,
    P_OBJECTS_INDEX_HALF_WIDTH,
    P_OBJECTS_INDEX_X,
    P_OBJECTS_INDEX_Y,
    P_OBJECTS_INDEX_YAW,
    STATE_INDEX_A_T,
    STATE_INDEX_DELTA_F,
    STATE_INDEX_DELTA_R,
    STATE_INDEX_V_T,
)
from utils import (
    activate_constraint,
    activate_upper_bounded_constraint,
    conservative_smooth_sat_margin,
    determine_spacially_matched_ref_path_point,
    ego_obb_geometry,
    expand_ego_obb_forward,
    saturate_positive_margin,
    smooth_abs_upper,
    stable_tan,
)

OBB_REAR_MARGIN_M = 0.1


def _expand_slack_weights(values, count, name):
    arr = np.asarray(values, dtype=float).reshape(-1)
    if arr.size == 1:
        return np.full(count, float(arr[0]))
    if arr.size != count:
        raise ValueError(f"Slack weight '{name}' has length {arr.size}, expected {count}.")
    return arr


def set_constraints(ocp: AcadosOcp, config):
    """Set up constraints for the optimal control problem.

    Args:
        ocp: The AcadosOcp object to configure constraints for.
        config: Configuration dictionary containing constraint parameters.
    """
    cons = AcadosOcpConstraints()

    # === Static constraints on state ===
    # set v_min < v < v_max [m/s]
    # set a_min < a < a_max [m/s^2]
    # set -delta_max < delta_f < delta_max [rad]
    # RWS: set -delta_max < delta_r < delta_max [rad]

    # constraints on initiall shooting node
    # initial state
    cons.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    if config["model_type"] == "RWS":
        cons.x0 = np.concatenate((cons.x0, [0.0]))

    # constraints on intermediate shooting nodes
    cons.lbx = np.array([config["v_min"], config["acceleration_t_min"], -config["delta_max"]])
    cons.ubx = np.array([config["v_max"], config["acceleration_t_max"], config["delta_max"]])
    cons.idxbx = np.array([STATE_INDEX_V_T, STATE_INDEX_A_T, STATE_INDEX_DELTA_F])

    if config["model_type"] == "RWS":
        cons.lbx = np.concatenate((cons.lbx, [-config["delta_max"]]))
        cons.ubx = np.concatenate((cons.ubx, [config["delta_max"]]))
        cons.idxbx = np.concatenate((cons.idxbx, [STATE_INDEX_DELTA_R]))

    # constraints on terminal shooting node
    cons.lbx_e = cons.lbx
    cons.ubx_e = cons.ubx
    cons.idxbx_e = cons.idxbx

    # === Static constraints on control ===
    # set j_min < j < j_max [m/s^3]
    # set -alpha_max < alpha_f < alpha_max [rad]
    # RWS: set -alpha_max < alpha_r < alpha_max [rad]
    cons.lbu = np.array([config["jerk_min"], -config["alpha_max"]])
    cons.ubu = np.array([config["jerk_max"], config["alpha_max"]])
    cons.idxbu = np.array([CONTROL_INDEX_J_T, CONTROL_INDEX_ALPHA_F])

    if config["model_type"] == "RWS":
        cons.lbu = np.concatenate((cons.lbu, [-config["alpha_max"]]))
        cons.ubu = np.concatenate((cons.ubu, [config["alpha_max"]]))
        cons.idxbu = np.concatenate((cons.idxbu, [CONTROL_INDEX_ALPHA_R]))

    # === Nonlinear constraints ===
    ocp.model.con_h_expr = ca.vertcat()  # initialize empty expression for nonlinear constraints
    cons.lh = np.array([])  # initialize empty lower bounds for nonlinear constraints
    cons.uh = np.array([])  # initialize empty upper bounds for nonlinear constraints

    ego_obb = ego_obb_geometry(ocp, config)

    # get stage-wise constraint activation parameters
    idx_params = np.prod(config["p_dynamic_weight_shape"])
    p_constraint_activation = ocp.model.p[
        idx_params : (idx_params := idx_params + np.prod(config["p_constraint_activation_shape"]))
    ]
    p_object_activation = p_constraint_activation[0]
    p_boundary_activation = p_constraint_activation[1]

    # --- Route boundaries ---

    # get ref path with boundaries from global parameters
    idx_global_params = 0
    _ = ocp.model.p_global[
        idx_global_params : (idx_global_params := idx_global_params + np.prod(config["p_cost_weights_shape"]))
    ]  # cost weights, not used in constraints
    p_cost_params = ocp.model.p_global[
        idx_global_params : (idx_global_params := idx_global_params + np.prod(config["p_cost_params_shape"]))
    ]
    p_ref_path = ocp.model.p_global[
        idx_global_params : (idx_global_params := idx_global_params + np.prod(config["p_ref_path_shape"]))
    ]
    assert idx_global_params == np.prod(config["p_cost_weights_shape"]) + np.prod(config["p_cost_params_shape"]) + np.prod(
        config["p_ref_path_shape"]
    )
    p_thw = p_cost_params[0]
    d_min_object_long = p_cost_params[1]
    d_min_object_lat = p_cost_params[2]
    d_min_boundary_lat = p_cost_params[3]

    # calc normal vector from interpolated reference point to the current position
    ref_inter = determine_spacially_matched_ref_path_point(config, p_ref_path, ego_obb["x"], ego_obb["y"])
    normal_vec = ca.vertcat(-ca.sin(ref_inter["psi"]), ca.cos(ref_inter["psi"]))

    epsilon = config["obb_smoothing_epsilon"]
    boundary_support = ego_obb["half_length"] * smooth_abs_upper(ca.dot(normal_vec, ego_obb["long_axis"]), epsilon)
    boundary_support += ego_obb["half_width"] * smooth_abs_upper(ca.dot(normal_vec, ego_obb["lat_axis"]), epsilon)
    center_ref_diff = ca.vertcat(ego_obb["x"] - ref_inter["x"], ego_obb["y"] - ref_inter["y"])
    d_center_ref_path = ca.dot(center_ref_diff, normal_vec)
    left_margin = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_left_boundary"] - boundary_support, 0.0))
    right_margin = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_right_boundary"] - boundary_support, 0.0))
    left_constraint = activate_upper_bounded_constraint(
        d_center_ref_path + boundary_support + left_margin - ref_inter["d_left_boundary"], p_boundary_activation
    )
    right_constraint = activate_upper_bounded_constraint(
        -d_center_ref_path + boundary_support + right_margin - ref_inter["d_right_boundary"], p_boundary_activation
    )
    ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, left_constraint, right_constraint)
    cons.lh = np.concatenate((cons.lh, [-ACADOS_INFTY, -ACADOS_INFTY]))
    cons.uh = np.concatenate((cons.uh, [0.0, 0.0]))

    # --- Object avoidance ---
    # get objects from parameters
    object_shape = config["p_objects_shape"]
    p_objects = ocp.model.p[idx_params : (idx_params := idx_params + np.prod(object_shape))]
    assert idx_params == (
        np.prod(config["p_dynamic_weight_shape"]) + np.prod(config["p_constraint_activation_shape"]) + np.prod(object_shape)
    )

    v_t = ocp.model.x[STATE_INDEX_V_T]
    # Both vehicle models are forward-only. THW and d_min_object_long protect
    # the front; the rear only needs a small physical clearance.
    front_margin = ca.fmax(ca.fmax(d_min_object_long, 0.0), p_thw * ca.fmax(v_t, 0.0))
    lateral_margin = ca.fmax(d_min_object_lat, 0.0)
    safety_ego_obb = expand_ego_obb_forward(ego_obb, front_margin, OBB_REAR_MARGIN_M, lateral_margin)
    for i in range(object_shape[0]):
        base = i * object_shape[1]
        object_yaw = p_objects[base + P_OBJECTS_INDEX_YAW]
        object_box = {
            "x": p_objects[base + P_OBJECTS_INDEX_X],
            "y": p_objects[base + P_OBJECTS_INDEX_Y],
            "long_axis": ca.vertcat(ca.cos(object_yaw), ca.sin(object_yaw)),
            "lat_axis": ca.vertcat(-ca.sin(object_yaw), ca.cos(object_yaw)),
            "half_length": p_objects[base + P_OBJECTS_INDEX_HALF_LENGTH],
            "half_width": p_objects[base + P_OBJECTS_INDEX_HALF_WIDTH],
        }
        sat_margin = conservative_smooth_sat_margin(
            safety_ego_obb, object_box, config["obb_smoothing_epsilon"], config["obb_smoothing_tau"]
        )
        sat_margin = saturate_positive_margin(sat_margin, config["obb_positive_margin_scale"])
        active = p_object_activation * p_objects[base + P_OBJECTS_INDEX_ACTIVE]
        constraint = activate_constraint(sat_margin, active)
        ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, constraint)
        cons.lh = np.concatenate((cons.lh, [0.0]))
        cons.uh = np.concatenate((cons.uh, [ACADOS_INFTY]))

    # --- a_abs_squared, psi_dot, steering_mode_constraint ---
    # Ackermann: psi_dot = v / l * tan(delta_f)
    # RWS: psi_dot = v * cos(beta) * (tan(delta_f) - tan(delta_r)) / (L_f + L_r)
    # beta = atan((L_r / (L_f + L_r)) * tan(delta_f) + (L_f / (L_f + L_r)) * tan(delta_r))
    psi_dot = None
    if config["model_type"] == "RWS":
        L_f = config["distance_cg_front_axle"]
        L_r = config["distance_cg_rear_axle"]
        beta = ca.atan(
            (L_r / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])
            + (L_f / (L_f + L_r)) * stable_tan(ocp.model.x[STATE_INDEX_DELTA_R])
        )
        psi_dot = (
            ocp.model.x[STATE_INDEX_V_T]
            * ca.cos(beta)
            * (stable_tan(ocp.model.x[STATE_INDEX_DELTA_F]) - stable_tan(ocp.model.x[STATE_INDEX_DELTA_R]))
            / (L_f + L_r)
        )
    else:
        wheelbase = config["wheelbase"]
        psi_dot = ocp.model.x[STATE_INDEX_V_T] / wheelbase * stable_tan(ocp.model.x[STATE_INDEX_DELTA_F])

    # compute normal acceleration
    a_n = ocp.model.x[STATE_INDEX_V_T] * psi_dot

    # Set boundaries for nonlinear constraints
    # a_abs_squared < a_max_squared
    # -psi_dot_max < psi_dot < psi_dot_max
    # -beta_max < beta < beta_max TODO: check if this would improve stability

    # compute absolute acceleration
    a_abs_squared = ocp.model.x[STATE_INDEX_A_T] ** 2 + (a_n) ** 2
    ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, a_n, a_abs_squared)
    a_max_squared = config["acceleration_max"] ** 2
    cons.lh = np.concatenate((cons.lh, [-config["acceleration_n_max"]], [0.0]))
    cons.uh = np.concatenate((cons.uh, [config["acceleration_n_max"]], [a_max_squared]))

    if config["model_type"] == "RWS":
        ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, psi_dot)
        cons.lh = np.concatenate((cons.lh, [-config["psi_dot_max"]]))
        cons.uh = np.concatenate((cons.uh, [config["psi_dot_max"]]))

        # nonlinear constraints for steering mode
        steering_mode_constraints = {
            "in-phase": ocp.model.x[STATE_INDEX_DELTA_F] - ocp.model.x[STATE_INDEX_DELTA_R],
            "anti-phase": ocp.model.x[STATE_INDEX_DELTA_F] + ocp.model.x[STATE_INDEX_DELTA_R],
        }
        if config["steering_mode_constraint"] in steering_mode_constraints:
            ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, steering_mode_constraints[config["steering_mode_constraint"]])
            cons.lh = np.concatenate((cons.lh, [0.0]))
            cons.uh = np.concatenate((cons.uh, [0.0]))
        elif config["steering_mode_constraint"] != "none":
            raise ValueError("Invalid steering mode. Choose between 'in-phase', 'anti-phase' or 'none'(default).")

    # also apply same constraints on terminal shooting node
    ocp.model.con_h_expr_e = ocp.model.con_h_expr
    cons.lh_e = cons.lh
    cons.uh_e = cons.uh

    # === Soft constraints ===

    slack_indices = []
    slack_weights = {"zl": [], "zu": [], "Zl": [], "Zu": []}
    boundary_constraints = 2
    object_constraints = object_shape[0]

    if config["enable_boundary_slack"]:
        slack_indices.extend(range(0, boundary_constraints))
        boundary_weights = config["boundary_slack_weights"]
        slack_weights["zl"].append(
            _expand_slack_weights(boundary_weights["linear_lower"], boundary_constraints, "boundary_slack_weights.linear_lower")
        )
        slack_weights["zu"].append(
            _expand_slack_weights(boundary_weights["linear_upper"], boundary_constraints, "boundary_slack_weights.linear_upper")
        )
        slack_weights["Zl"].append(
            _expand_slack_weights(
                boundary_weights["quadratic_lower"], boundary_constraints, "boundary_slack_weights.quadratic_lower"
            )
        )
        slack_weights["Zu"].append(
            _expand_slack_weights(
                boundary_weights["quadratic_upper"], boundary_constraints, "boundary_slack_weights.quadratic_upper"
            )
        )

    if config["enable_object_slack"]:
        slack_indices.extend(range(boundary_constraints, boundary_constraints + object_constraints))
        object_weights = config["object_slack_weights"]
        slack_weights["zl"].append(
            _expand_slack_weights(object_weights["linear_lower"], object_constraints, "object_slack_weights.linear_lower")
        )
        slack_weights["zu"].append(
            _expand_slack_weights(object_weights["linear_upper"], object_constraints, "object_slack_weights.linear_upper")
        )
        slack_weights["Zl"].append(
            _expand_slack_weights(object_weights["quadratic_lower"], object_constraints, "object_slack_weights.quadratic_lower")
        )
        slack_weights["Zu"].append(
            _expand_slack_weights(object_weights["quadratic_upper"], object_constraints, "object_slack_weights.quadratic_upper")
        )

    if slack_indices:
        cons.idxsh = np.array(slack_indices, dtype=int)
        zl = np.concatenate(slack_weights["zl"])
        zu = np.concatenate(slack_weights["zu"])
        Zl = np.concatenate(slack_weights["Zl"])
        Zu = np.concatenate(slack_weights["Zu"])

        # In the cost terms, the slack variables are arranged as follows: idxsbu, idxsbx, idxsg, idxsh
        # Zl/Zu are diagonals of the Hessian wrt slack variables (vector form).
        ocp.cost.Zl = Zl
        ocp.cost.Zu = Zu
        ocp.cost.zl = zl
        ocp.cost.zu = zu

        cons.idxsh_e = cons.idxsh
        ocp.cost.Zl_e = ocp.cost.Zl
        ocp.cost.Zu_e = ocp.cost.Zu
        ocp.cost.zl_e = ocp.cost.zl
        ocp.cost.zu_e = ocp.cost.zu

    ocp.constraints = cons

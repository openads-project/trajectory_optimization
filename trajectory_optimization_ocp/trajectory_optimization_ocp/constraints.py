# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import casadi as ca
import numpy as np
from acados_template import ACADOS_INFTY, AcadosOcp, AcadosOcpConstraints
from constants import (
    CONTROL_INDEX_ALPHA_F,
    CONTROL_INDEX_ALPHA_R,
    CONTROL_INDEX_J_T,
    P_OBSTACLES_INDEX_ACTIVE,
    P_OBSTACLES_INDEX_HALF_LENGTH,
    P_OBSTACLES_INDEX_HALF_WIDTH,
    P_OBSTACLES_INDEX_RADIUS,
    P_OBSTACLES_INDEX_X,
    P_OBSTACLES_INDEX_Y,
    P_OBSTACLES_INDEX_YAW,
    STATE_INDEX_A_T,
    STATE_INDEX_DELTA_F,
    STATE_INDEX_DELTA_R,
    STATE_INDEX_PSI,
    STATE_INDEX_V_T,
    STATE_INDEX_X,
    STATE_INDEX_Y,
)
from utils import (
    activate_constraint,
    approximate_ego_geometry,
    conservative_smooth_sat_margin,
    determine_spacially_matched_ref_path_point,
    ego_obb_geometry,
    expand_ego_obb_forward,
    obstacle_parameter_shape,
    smooth_abs_upper,
    stable_tan,
)


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

    collision_geometry = config.get("collision_geometry", "circles")
    if collision_geometry not in ("circles", "obb_sat"):
        raise ValueError(f"Unknown collision geometry: {collision_geometry}")

    ego_approximation = approximate_ego_geometry(ocp, config) if collision_geometry == "circles" else None
    ego_obb = ego_obb_geometry(ocp, config) if collision_geometry == "obb_sat" else None

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
    d_min_obstacle_long = p_cost_params[1]
    d_min_obstacle_lat = p_cost_params[2]
    d_min_boundary_lat = p_cost_params[3]

    # calc normal vector from interpolated reference point to the current position
    match_x = ego_obb["x"] if ego_obb is not None else ocp.model.x[STATE_INDEX_X]
    match_y = ego_obb["y"] if ego_obb is not None else ocp.model.x[STATE_INDEX_Y]
    ref_inter = determine_spacially_matched_ref_path_point(config, p_ref_path, match_x, match_y)
    normal_vec = ca.vertcat(-ca.sin(ref_inter["psi"]), ca.cos(ref_inter["psi"]))

    if collision_geometry == "circles":
        ego_radius = ego_approximation["radius"]
        left_margin_required = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_left_boundary"] - ego_radius, 0.0))
        right_margin_required = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_right_boundary"] - ego_radius, 0.0))
        for i in range(config["n_ego_circles"]):
            circle_ref_diff = ca.vertcat(ego_approximation["x"][i] - ref_inter["x"], ego_approximation["y"][i] - ref_inter["y"])
            d_circle_center_ref_path = ca.dot(circle_ref_diff, normal_vec)
            left_constraint = d_circle_center_ref_path + ego_radius + left_margin_required - ref_inter["d_left_boundary"]
            right_constraint = -d_circle_center_ref_path + ego_radius + right_margin_required - ref_inter["d_right_boundary"]
            ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, left_constraint, right_constraint)
            cons.lh = np.concatenate((cons.lh, [-ACADOS_INFTY, -ACADOS_INFTY]))
            cons.uh = np.concatenate((cons.uh, [0.0, 0.0]))
    else:
        epsilon = config["obb_smoothing_epsilon"]
        boundary_support = ego_obb["half_length"] * smooth_abs_upper(ca.dot(normal_vec, ego_obb["long_axis"]), epsilon)
        boundary_support += ego_obb["half_width"] * smooth_abs_upper(ca.dot(normal_vec, ego_obb["lat_axis"]), epsilon)
        center_ref_diff = ca.vertcat(ego_obb["x"] - ref_inter["x"], ego_obb["y"] - ref_inter["y"])
        d_center_ref_path = ca.dot(center_ref_diff, normal_vec)
        left_margin_required = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_left_boundary"] - boundary_support, 0.0))
        right_margin_required = ca.fmin(d_min_boundary_lat, ca.fmax(ref_inter["d_right_boundary"] - boundary_support, 0.0))
        left_constraint = d_center_ref_path + boundary_support + left_margin_required - ref_inter["d_left_boundary"]
        right_constraint = -d_center_ref_path + boundary_support + right_margin_required - ref_inter["d_right_boundary"]
        ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, left_constraint, right_constraint)
        cons.lh = np.concatenate((cons.lh, [-ACADOS_INFTY, -ACADOS_INFTY]))
        cons.uh = np.concatenate((cons.uh, [0.0, 0.0]))

    # --- Obstacle avoidance ---
    # get obstacles from parameters
    idx_params = 0
    _ = ocp.model.p[
        idx_params : (idx_params := idx_params + np.prod(config["p_dynamic_weight_shape"]))
    ]  # dynamic weights, not used in constraints
    obstacle_shape = obstacle_parameter_shape(config)
    p_obstacles = ocp.model.p[idx_params : (idx_params := idx_params + np.prod(obstacle_shape))]
    assert idx_params == np.prod(config["p_dynamic_weight_shape"]) + np.prod(obstacle_shape)

    beta = compute_side_slip_angle(ocp, config)
    v_t = ocp.model.x[STATE_INDEX_V_T]
    psi = ocp.model.x[STATE_INDEX_PSI]
    cos_psi = ca.cos(psi)
    sin_psi = ca.sin(psi)

    dyn_long_buffer = ca.fabs(p_thw * v_t * ca.cos(beta))
    dyn_lat_buffer = ca.fabs(p_thw * v_t * ca.sin(beta))

    if collision_geometry == "circles":
        for i in range(obstacle_shape[0]):
            x_center = p_obstacles[i * obstacle_shape[1] + P_OBSTACLES_INDEX_X]
            y_center = p_obstacles[i * obstacle_shape[1] + P_OBSTACLES_INDEX_Y]
            r_circle = p_obstacles[i * obstacle_shape[1] + P_OBSTACLES_INDEX_RADIUS]

            combined_radius = ego_radius + r_circle
            d_long_min = ca.fmax(d_min_obstacle_long, dyn_long_buffer) + combined_radius
            d_lat_min = ca.fmax(d_min_obstacle_lat, dyn_lat_buffer) + combined_radius
            d_long_min = ca.fmax(d_long_min, 1e-3)
            d_lat_min = ca.fmax(d_lat_min, 1e-3)

            for j in range(config["n_ego_circles"]):
                dx = x_center - ego_approximation["x"][j]
                dy = y_center - ego_approximation["y"][j]
                d_long_rel = dx * cos_psi + dy * sin_psi
                d_lat_rel = -dx * sin_psi + dy * cos_psi
                ellipse_constraint = ca.power(d_long_rel / d_long_min, 2) + ca.power(d_lat_rel / d_lat_min, 2) - 1.0
                ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, ellipse_constraint)
                cons.lh = np.concatenate((cons.lh, [0.0]))
                cons.uh = np.concatenate((cons.uh, [ACADOS_INFTY]))
    else:
        epsilon = config["obb_smoothing_epsilon"]
        tau = config["obb_smoothing_tau"]
        rear_margin = ca.fmax(d_min_obstacle_long, 0.0)
        front_margin = ca.fmax(rear_margin, dyn_long_buffer)
        lateral_margin = ca.fmax(d_min_obstacle_lat, dyn_lat_buffer)
        safety_ego_obb = expand_ego_obb_forward(ego_obb, front_margin, rear_margin, lateral_margin)

        for i in range(obstacle_shape[0]):
            base = i * obstacle_shape[1]
            obstacle_center = ca.vertcat(p_obstacles[base + P_OBSTACLES_INDEX_X], p_obstacles[base + P_OBSTACLES_INDEX_Y])
            obstacle_yaw = p_obstacles[base + P_OBSTACLES_INDEX_YAW]
            obstacle_half_length = p_obstacles[base + P_OBSTACLES_INDEX_HALF_LENGTH]
            obstacle_half_width = p_obstacles[base + P_OBSTACLES_INDEX_HALF_WIDTH]
            obstacle_active = p_obstacles[base + P_OBSTACLES_INDEX_ACTIVE]
            obstacle_obb = {
                "x": obstacle_center[0],
                "y": obstacle_center[1],
                "long_axis": ca.vertcat(ca.cos(obstacle_yaw), ca.sin(obstacle_yaw)),
                "lat_axis": ca.vertcat(-ca.sin(obstacle_yaw), ca.cos(obstacle_yaw)),
                "half_length": obstacle_half_length,
                "half_width": obstacle_half_width,
            }
            sat_margin = conservative_smooth_sat_margin(safety_ego_obb, obstacle_obb, epsilon, tau)
            ocp.model.con_h_expr = ca.vertcat(ocp.model.con_h_expr, activate_constraint(sat_margin, obstacle_active))
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
    ego_primitives = config["n_ego_circles"] if collision_geometry == "circles" else 1
    boundary_constraints = ego_primitives * 2
    obstacle_constraints = obstacle_shape[0] * ego_primitives

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

    if config["enable_obstacle_slack"]:
        slack_indices.extend(range(boundary_constraints, boundary_constraints + obstacle_constraints))
        obstacle_weights = config["obstacle_slack_weights"]
        slack_weights["zl"].append(
            _expand_slack_weights(obstacle_weights["linear_lower"], obstacle_constraints, "obstacle_slack_weights.linear_lower")
        )
        slack_weights["zu"].append(
            _expand_slack_weights(obstacle_weights["linear_upper"], obstacle_constraints, "obstacle_slack_weights.linear_upper")
        )
        slack_weights["Zl"].append(
            _expand_slack_weights(
                obstacle_weights["quadratic_lower"], obstacle_constraints, "obstacle_slack_weights.quadratic_lower"
            )
        )
        slack_weights["Zu"].append(
            _expand_slack_weights(
                obstacle_weights["quadratic_upper"], obstacle_constraints, "obstacle_slack_weights.quadratic_upper"
            )
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

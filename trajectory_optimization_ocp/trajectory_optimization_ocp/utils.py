# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import casadi as ca
from acados_template import AcadosOcp
from constants import (
    P_REF_PATH_INDEX_D_BOUND_LEFT,
    P_REF_PATH_INDEX_D_BOUND_RIGHT,
    P_REF_PATH_INDEX_PSI,
    P_REF_PATH_INDEX_V,
    P_REF_PATH_INDEX_X,
    P_REF_PATH_INDEX_Y,
    STATE_INDEX_PSI,
    STATE_INDEX_X,
    STATE_INDEX_Y,
)


def stable_tan(rad):
    """Computes the tangent function with numerical stability bounds.

    Args:
        rad: The input angle in radians.

    Returns:
        The tangent value clamped to the range [-100, 100].
    """
    # for numerical stability of the tangent function
    return ca.fmax(-100, ca.fmin(100, ca.tan(rad)))


def determine_spacially_matched_ref_path_point(config: dict, p_ref_path: ca.MX, x_position: ca.MX, y_position: ca.MX) -> ca.MX:
    """Determines the spatially matched reference path point for a given position (current state).

    This is done by calculating the closest point on the reference path to the current state.
    The reference path is given as a parameter p_ref_path.

    Args:
        config: Configuration dictionary containing p_ref_path_shape.
        p_ref_path: Reference path containing psi, x, y, v, and boundary values.
        x_position: X-coordinate of the current position.
        y_position: Y-coordinate of the current position.

    Returns:
        A dictionary with interpolated reference path point containing psi, x, y, v, and boundary values.
    """
    ref_path_state_dim = config["p_ref_path_shape"][1]
    # p_ref_path shuold be sortet like this: (psi1, x1, y1, v1, psi2, x2, y2, v2, ...)
    psi_ref_path = p_ref_path[P_REF_PATH_INDEX_PSI::ref_path_state_dim]  # every 6th element starting from index 0
    x_ref_path = p_ref_path[P_REF_PATH_INDEX_X::ref_path_state_dim]  # every 6th element starting from index 1
    y_ref_path = p_ref_path[P_REF_PATH_INDEX_Y::ref_path_state_dim]  # every 6th element starting from index 2
    v_ref_path = p_ref_path[P_REF_PATH_INDEX_V::ref_path_state_dim]  # every 6th element starting from index 3
    d_left_boundary_ref_path = p_ref_path[
        P_REF_PATH_INDEX_D_BOUND_LEFT::ref_path_state_dim
    ]  # every 6th element starting from index 4
    d_right_boundary_ref_path = p_ref_path[
        P_REF_PATH_INDEX_D_BOUND_RIGHT::ref_path_state_dim
    ]  # every 6th element starting from index 5

    n_ref_path_points = config["p_ref_path_shape"][0]
    # initialize closest distances to path sample with a large value
    closest_distance = ca.inf
    idx_min = ca.MX(0)

    for i in range(n_ref_path_points):
        dx = x_ref_path[i] - x_position
        dy = y_ref_path[i] - y_position
        c = ca.sqrt(dx**2 + dy**2)
        idx_min = ca.if_else(c < closest_distance, ca.MX(i), idx_min)
        closest_distance = ca.if_else(c < closest_distance, c, closest_distance)

    # find nearest adjacent sample on reference path
    condition_begin = idx_min == 0
    condition_end = idx_min == n_ref_path_points - 1
    condition_intermediate = ca.logic_and(ca.logic_not(condition_begin), ca.logic_not(condition_end))
    dist_1 = ca.if_else(
        condition_intermediate,
        ca.sqrt(ca.power(x_ref_path[idx_min - 1] - x_position, 2) + ca.power(y_ref_path[idx_min - 1] - y_position, 2)),
        ca.inf,
        True,
    )
    dist_2 = ca.if_else(
        condition_intermediate,
        ca.sqrt(ca.power(x_ref_path[idx_min + 1] - x_position, 2) + ca.power(y_ref_path[idx_min + 1] - y_position, 2)),
        ca.inf,
        True,
    )
    condition_dist = dist_1 < dist_2
    next_idx_min = ca.if_else(
        condition_begin,
        idx_min + 1,
        ca.if_else(condition_end, idx_min - 1, ca.if_else(condition_dist, idx_min - 1, idx_min + 1, True), True),
        True,
    )

    # compute the shortest distance between the state-point and a line segment idx_min---next_idx_min
    # extend the segment to a complete line first and determine the point with
    # shortest distance to the state point, following
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line, but
    # formulate it as parameter lambda
    # values [0, 1] for lambda mean the nearest point is on the segment
    # and the computed distance is perpendicular to the line segment
    psi1 = psi_ref_path[idx_min]
    x1 = x_ref_path[idx_min]
    y1 = y_ref_path[idx_min]
    v1 = v_ref_path[idx_min]
    d_left_boundary1 = d_left_boundary_ref_path[idx_min]
    d_right_boundary1 = d_right_boundary_ref_path[idx_min]

    psi2 = psi_ref_path[next_idx_min]
    x2 = x_ref_path[next_idx_min]
    y2 = y_ref_path[next_idx_min]
    v2 = v_ref_path[next_idx_min]
    d_left_boundary2 = d_left_boundary_ref_path[next_idx_min]
    d_right_boundary2 = d_right_boundary_ref_path[next_idx_min]

    dxy_sq = ca.power(x2 - x1, 2) + ca.power(y2 - y1, 2)
    raw_lmd = ca.if_else((dxy_sq == 0), 0, ((x_position - x1) * (x2 - x1) + (y_position - y1) * (y2 - y1)) / dxy_sq)
    # allow extrapolation for beginning of reference but not at the end (to penalize overshooting)
    position_lmd = ca.if_else(condition_begin, raw_lmd, ca.fmin(ca.fmax(raw_lmd, 0), 1))
    x_ref_inter = x1 + position_lmd * (x2 - x1)
    y_ref_inter = y1 + position_lmd * (y2 - y1)

    # Boundary distances, heading, and velocity are only defined at the provided
    # reference samples and must not be extrapolated with the reference line.
    bounded_lmd = ca.fmin(ca.fmax(position_lmd, 0), 1)
    d_left_boundary_inter = d_left_boundary1 + bounded_lmd * (d_left_boundary2 - d_left_boundary1)
    d_right_boundary_inter = d_right_boundary1 + bounded_lmd * (d_right_boundary2 - d_right_boundary1)
    psi_ref_inter = psi1 + bounded_lmd * wrap_angle(psi2 - psi1)
    v_ref_inter = v1 + bounded_lmd * (v2 - v1)

    interpolated_ref_path_point = {
        "psi": psi_ref_inter,
        "x": x_ref_inter,
        "y": y_ref_inter,
        "v": v_ref_inter,
        "d_left_boundary": d_left_boundary_inter,
        "d_right_boundary": d_right_boundary_inter,
    }
    return interpolated_ref_path_point


def ego_obb_geometry(ocp: AcadosOcp, config: dict) -> dict:
    """Return the exact oriented ego bounding box in the optimizer frame."""
    psi = ocp.model.x[STATE_INDEX_PSI]
    cos_psi = ca.cos(psi)
    sin_psi = ca.sin(psi)
    offset_long, offset_lat = config["offset2geocenter"]
    return {
        "x": ocp.model.x[STATE_INDEX_X] + offset_long * cos_psi - offset_lat * sin_psi,
        "y": ocp.model.x[STATE_INDEX_Y] + offset_long * sin_psi + offset_lat * cos_psi,
        "long_axis": ca.vertcat(cos_psi, sin_psi),
        "lat_axis": ca.vertcat(-sin_psi, cos_psi),
        "half_length": 0.5 * config["length"],
        "half_width": 0.5 * config["width"],
    }


def expand_ego_obb_forward(ego_obb: dict, front_margin: ca.MX, rear_margin: ca.MX, lateral_margin: ca.MX) -> dict:
    """Expand an OBB independently at its front, rear and lateral sides."""
    center_shift = 0.5 * (front_margin - rear_margin)
    return {
        **ego_obb,
        "x": ego_obb["x"] + center_shift * ego_obb["long_axis"][0],
        "y": ego_obb["y"] + center_shift * ego_obb["long_axis"][1],
        "half_length": ego_obb["half_length"] + 0.5 * (front_margin + rear_margin),
        "half_width": ego_obb["half_width"] + lateral_margin,
    }


def smooth_abs_upper(value: ca.MX, epsilon: float) -> ca.MX:
    """Differentiable upper bound of ``abs(value)``."""
    return ca.sqrt(value * value + epsilon * epsilon)


def smooth_abs_lower(value: ca.MX, epsilon: float) -> ca.MX:
    """Differentiable lower bound of ``abs(value)``."""
    return ca.sqrt(value * value + epsilon * epsilon) - epsilon


def smooth_max_lower_pair(first: ca.MX, second: ca.MX, tau: float) -> ca.MX:
    """Differentiable lower bound of the maximum of two values."""
    difference = first - second
    return 0.5 * (first + second + ca.sqrt(difference * difference + tau * tau) - tau)


def smooth_max_lower(values: list[ca.MX], tau: float) -> ca.MX:
    """Balanced conservative reduction of multiple maximum candidates."""
    if not values:
        raise ValueError("smooth_max_lower requires at least one value")
    level = values
    while len(level) > 1:
        level = [
            smooth_max_lower_pair(level[index], level[index + 1], tau) if index + 1 < len(level) else level[index]
            for index in range(0, len(level), 2)
        ]
    return level[0]


def conservative_smooth_sat_margin(first: dict, second: dict, epsilon: float, tau: float) -> ca.MX:
    """Return a smooth lower bound of the exact four-axis OBB SAT margin."""
    center_difference = ca.vertcat(second["x"] - first["x"], second["y"] - first["y"])
    axes = [first["long_axis"], first["lat_axis"], second["long_axis"], second["lat_axis"]]
    gaps = []
    for axis in axes:
        center_projection = smooth_abs_lower(ca.dot(center_difference, axis), epsilon)
        first_support = first["half_length"] * smooth_abs_upper(ca.dot(first["long_axis"], axis), epsilon)
        first_support += first["half_width"] * smooth_abs_upper(ca.dot(first["lat_axis"], axis), epsilon)
        second_support = second["half_length"] * smooth_abs_upper(ca.dot(second["long_axis"], axis), epsilon)
        second_support += second["half_width"] * smooth_abs_upper(ca.dot(second["lat_axis"], axis), epsilon)
        gaps.append(center_projection - first_support - second_support)
    return smooth_max_lower(gaps, tau)


def activate_constraint(value: ca.MX, active: ca.MX, inactive_margin: float = 1.0) -> ca.MX:
    """Turn an unused fixed obstacle slot into a constant feasible constraint."""
    return active * value + (1.0 - active) * inactive_margin


def activate_upper_bounded_constraint(value: ca.MX, active: ca.MX, inactive_margin: float = 1.0) -> ca.MX:
    """Turn an inactive upper-bounded constraint into a constant feasible value."""
    return active * value - (1.0 - active) * inactive_margin


def saturate_positive_margin(value: ca.MX, scale: float) -> ca.MX:
    """Limit irrelevant positive SAT margins without changing their sign."""
    if scale <= 0.0:
        raise ValueError("Margin saturation scale must be positive")
    return ca.if_else(value > 0.0, scale * (1.0 - ca.exp(-value / scale)), value)


def wrap_angle(angle: ca.MX) -> ca.MX:
    """Wraps an angle to the interval [-pi, pi]."""
    return angle - 2 * ca.pi * ca.floor((angle + ca.pi) / (2 * ca.pi))

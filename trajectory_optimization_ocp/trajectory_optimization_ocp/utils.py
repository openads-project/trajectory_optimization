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


def approximate_ego_geometry(ocp: AcadosOcp, config: dict) -> dict:
    """Approximates the ego vehicle geometry as a set of circles.

    Args:
        ocp: The ACADOS OCP object containing the vehicle model.
        config: Configuration dictionary with parameters like length, width, n_ego_circles, and offset2geocenter.

    Returns:
        A dictionary containing circle positions (x, y) and their radius.
    """
    # rectangular ego-vehicle approximation with n_circles circles
    psi = ocp.model.x[STATE_INDEX_PSI]
    cos_psi = ca.cos(psi)
    sin_psi = ca.sin(psi)
    offset_long, offset_lat = config["offset2geocenter"]
    ego_center_x = ocp.model.x[STATE_INDEX_X] + offset_long * cos_psi - offset_lat * sin_psi
    ego_center_y = ocp.model.x[STATE_INDEX_Y] + offset_long * sin_psi + offset_lat * cos_psi
    # Calculate the radius using symbolic operations
    radius = ca.sqrt(ca.power(config["length"] / (2 * config["n_ego_circles"]), 2) + ca.power((config["width"] / 2.0), 2))

    # Initialize an empty list for circle centers coordinates
    circle_position_x = []
    circle_position_y = []

    if config["n_ego_circles"] == 1:
        circle_position_x.append(ego_center_x)
        circle_position_y.append(ego_center_y)
    else:
        # Loop to compute the centers of each circle
        for i in range(config["n_ego_circles"]):
            lon_offset = -config["length"] / 2 + (2 * i + 1) * config["length"] / (2 * config["n_ego_circles"])
            x_offset = lon_offset * cos_psi
            y_offset = lon_offset * sin_psi
            circle_position_x.append(ego_center_x + x_offset)
            circle_position_y.append(ego_center_y + y_offset)

    return {
        "x": circle_position_x,
        "y": circle_position_y,
        "radius": radius,
    }


def ego_obb_geometry(ocp: AcadosOcp, config: dict) -> dict:
    """Return the exact oriented ego bounding box in the optimizer frame."""
    psi = ocp.model.x[STATE_INDEX_PSI]
    cos_psi = ca.cos(psi)
    sin_psi = ca.sin(psi)
    offset_long, offset_lat = config["offset2geocenter"]
    return {
        "x": ocp.model.x[STATE_INDEX_X] + offset_long * cos_psi - offset_lat * sin_psi,
        "y": ocp.model.x[STATE_INDEX_Y] + offset_long * sin_psi + offset_lat * cos_psi,
        "yaw": psi,
        "long_axis": ca.vertcat(cos_psi, sin_psi),
        "lat_axis": ca.vertcat(-sin_psi, cos_psi),
        "half_length": 0.5 * config["length"],
        "half_width": 0.5 * config["width"],
    }


def smooth_abs_upper(value: ca.MX, epsilon: float) -> ca.MX:
    """Differentiable upper bound of ``abs(value)``."""
    return ca.sqrt(value * value + epsilon * epsilon)


def smooth_abs_lower(value: ca.MX, epsilon: float) -> ca.MX:
    """Differentiable lower bound of ``abs(value)``."""
    return ca.sqrt(value * value + epsilon * epsilon) - epsilon


def smooth_max_lower_pair(first: ca.MX, second: ca.MX, tau: float) -> ca.MX:
    """Stable differentiable lower bound of the maximum of two values."""
    difference = first - second
    return 0.5 * (first + second + ca.sqrt(difference * difference + tau * tau) - tau)


def smooth_max_lower(values: list[ca.MX], tau: float) -> ca.MX:
    """Balanced reduction using the conservative pairwise smooth maximum."""
    if not values:
        raise ValueError("smooth_max_lower requires at least one value")
    level = values
    while len(level) > 1:
        next_level = []
        for index in range(0, len(level), 2):
            if index + 1 < len(level):
                next_level.append(smooth_max_lower_pair(level[index], level[index + 1], tau))
            else:
                next_level.append(level[index])
        level = next_level
    return level[0]


def obstacle_parameter_shape(config: dict) -> list[int]:
    """Return the active stage-wise obstacle layout for circles or OBBs."""
    if config.get("collision_geometry", "circles") == "obb_sat":
        return config["p_obstacle_obbs_shape"]
    return config["p_obstacle_circles_shape"]


def wrap_angle(angle: ca.MX) -> ca.MX:
    """Wraps an angle to the interval [-pi, pi]."""
    return angle - 2 * ca.pi * ca.floor((angle + ca.pi) / (2 * ca.pi))

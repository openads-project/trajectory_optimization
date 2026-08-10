# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Property tests for the symbolic conservative OBB building blocks."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import casadi as ca
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "trajectory_optimization_ocp"))

from utils import (  # noqa: E402, I202
    activate_constraint,
    conservative_smooth_sat_margin,
    ego_obb_geometry,
    expand_ego_obb_forward,
    saturate_margin,
    smooth_abs_lower,
    smooth_abs_upper,
    smooth_max_lower,
)


def test_margin_saturation_preserves_feasible_set_and_suppresses_far_gradients():
    """Saturation must preserve the sign and leave the contact gradient unchanged."""
    value = ca.MX.sym("margin")
    saturated = saturate_margin(value, 0.5)
    function = ca.Function("saturation", [value], [saturated, ca.jacobian(saturated, value)])
    for sample in (-100.0, -2.0, -1e-6, 0.0, 1e-6, 2.0, 100.0):
        result, _ = (float(item) for item in function(sample))
        assert (result >= 0.0) == (sample >= 0.0)
    _, contact_gradient = (float(item) for item in function(0.0))
    _, negative_gradient = (float(item) for item in function(-20.0))
    _, far_gradient = (float(item) for item in function(40.0))
    assert math.isclose(contact_gradient, 1.0, abs_tol=1e-12)
    assert math.isclose(negative_gradient, 1.0, abs_tol=1e-12)
    assert abs(far_gradient) < 1e-8


def test_smooth_bounds_are_conservative():
    """The smooth primitives must bound their exact counterparts in the safe direction."""
    value = ca.MX.sym("value")
    first = ca.MX.sym("first")
    second = ca.MX.sym("second")
    third = ca.MX.sym("third")
    fourth = ca.MX.sym("fourth")
    function = ca.Function(
        "bounds",
        [value, first, second, third, fourth],
        [smooth_abs_lower(value, 1e-3), smooth_abs_upper(value, 1e-3), smooth_max_lower([first, second, third, fourth], 0.02)],
    )
    generator = np.random.default_rng(42)
    for sample in generator.uniform(-100.0, 100.0, size=(10000, 5)):
        lower_abs, upper_abs, lower_max = (float(result) for result in function(*sample))
        assert lower_abs <= abs(sample[0]) + 1e-12
        assert upper_abs >= abs(sample[0]) - 1e-12
        assert lower_max <= max(sample[1:]) + 1e-12


def test_symbolic_ego_obb_applies_lateral_offset():
    """Both configured center offsets are rotated with the ego heading."""
    state = ca.MX.sym("state", 7)
    ocp = SimpleNamespace(model=SimpleNamespace(x=state))
    config = {"length": 4.0, "width": 2.0, "offset2geocenter": [3.0, 0.5]}
    box = ego_obb_geometry(ocp, config)
    function = ca.Function("ego_box", [state], [box["x"], box["y"], box["half_length"], box["half_width"]])
    values = np.zeros(7)
    values[0] = 1.0
    values[1] = 2.0
    values[5] = math.pi / 2.0
    x, y, half_length, half_width = (float(result) for result in function(values))
    assert math.isclose(x, 0.5, abs_tol=1e-12)
    assert math.isclose(y, 5.0, abs_tol=1e-12)
    assert half_length == 2.0
    assert half_width == 1.0


def _symbolic_box(x, y, yaw, half_length, half_width):
    return {
        "x": x,
        "y": y,
        "long_axis": ca.vertcat(ca.cos(yaw), ca.sin(yaw)),
        "lat_axis": ca.vertcat(-ca.sin(yaw), ca.cos(yaw)),
        "half_length": half_length,
        "half_width": half_width,
    }


def test_forward_expansion_keeps_rear_margin_independent_of_speed():
    """A growing THW margin must not move the rear safety edge backwards."""
    speed = ca.MX.sym("speed")
    yaw = ca.MX.sym("yaw")
    physical = _symbolic_box(1.0, 2.0, yaw, 2.5, 1.0)
    rear_margin = 1.0
    front_margin = ca.fmax(rear_margin, 2.0 * speed)
    safety = expand_ego_obb_forward(physical, front_margin, rear_margin, 0.1)
    function = ca.Function("forward_expansion", [speed, yaw], [safety["x"], safety["y"], safety["half_length"]])

    for test_speed in (0.0, 5.0, 10.0):
        x, y, half_length = (float(value) for value in function(test_speed, math.pi / 2.0))
        rear_edge = y - half_length
        front_edge = y + half_length
        assert math.isclose(x, 1.0, abs_tol=1e-12)
        assert math.isclose(rear_edge, 2.0 - 2.5 - rear_margin, abs_tol=1e-12)
        assert math.isclose(front_edge, 2.0 + 2.5 + max(rear_margin, 2.0 * test_speed), abs_tol=1e-12)


def test_sat_jacobian_matches_finite_differences_near_contacts_and_axis_changes():
    """Symbolic SAT derivatives must remain correct in difficult geometries."""
    variables = ca.MX.sym("variables", 7)
    ego = _symbolic_box(variables[0], variables[1], variables[2], 2.5, 1.0)
    safety_ego = expand_ego_obb_forward(ego, 8.0, 1.0, 0.1)
    obstacle = _symbolic_box(variables[3], variables[4], variables[5], variables[6], 0.8)
    margin = conservative_smooth_sat_margin(safety_ego, obstacle, 1e-3, 0.02)
    function = ca.Function("sat_margin_jacobian", [variables], [margin, ca.jacobian(margin, variables)])

    cases = [
        np.array([0.0, 0.0, 0.0, 8.51, 0.0, 0.0, 1.5]),
        np.array([0.0, 0.0, 0.0, 7.8, 1.91, 1e-3, 1.5]),
        np.array([0.0, 0.0, 0.3, 6.0, 3.0, math.pi / 2.0, 2.0]),
        np.array([0.0, 0.0, -0.4, 5.5, -2.5, math.pi / 4.0, 0.05]),
        np.array([0.0, 0.0, math.pi / 2.0, -1.91, 7.8, math.pi / 2.0 - 1e-3, 6.0]),
    ]
    step = 1e-6
    for values in cases:
        _, analytic = function(values)
        analytic = np.asarray(analytic).reshape(-1)
        finite_difference = np.empty_like(analytic)
        for index in range(values.size):
            delta = np.zeros_like(values)
            delta[index] = step
            plus = float(function(values + delta)[0])
            minus = float(function(values - delta)[0])
            finite_difference[index] = (plus - minus) / (2.0 * step)
        np.testing.assert_allclose(analytic, finite_difference, rtol=2e-5, atol=2e-6)


def test_inactive_sat_constraint_is_constant_and_has_zero_state_derivative():
    """Disabled slots must not affect the NLP values or derivatives."""
    variables = ca.MX.sym("variables", 6)
    active = ca.MX.sym("active")
    ego = _symbolic_box(variables[0], variables[1], variables[2], 2.5, 1.0)
    obstacle = _symbolic_box(variables[3], variables[4], variables[5], 2.0, 0.8)
    sat_margin = conservative_smooth_sat_margin(ego, obstacle, 1e-3, 0.02)
    constraint = activate_constraint(sat_margin, active)
    constraint_hessian = ca.hessian(constraint, variables)[0]
    function = ca.Function(
        "activated_sat", [variables, active], [constraint, ca.jacobian(constraint, variables), constraint_hessian]
    )
    values = np.array([1.0, -2.0, 0.3, 4.0, 5.0, -0.7])

    inactive_value, inactive_jacobian, inactive_hessian = function(values, 0.0)
    active_value, active_jacobian, active_hessian = function(values, 1.0)
    raw_function = ca.Function(
        "raw_sat", [variables], [sat_margin, ca.jacobian(sat_margin, variables), ca.hessian(sat_margin, variables)[0]]
    )
    raw_value, raw_jacobian, raw_hessian = raw_function(values)

    assert math.isclose(float(inactive_value), 1.0, abs_tol=1e-12)
    np.testing.assert_array_equal(np.asarray(inactive_jacobian), np.zeros((1, variables.numel())))
    np.testing.assert_array_equal(np.asarray(inactive_hessian), np.zeros((variables.numel(), variables.numel())))
    assert math.isclose(float(active_value), float(raw_value), abs_tol=1e-12)
    np.testing.assert_allclose(np.asarray(active_jacobian), np.asarray(raw_jacobian), rtol=0.0, atol=1e-12)
    np.testing.assert_allclose(np.asarray(active_hessian), np.asarray(raw_hessian), rtol=0.0, atol=1e-12)

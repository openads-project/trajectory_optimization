# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Properties of the symbolic OBB constraints used by the Karl OCP."""

import math
import sys
from pathlib import Path
from types import SimpleNamespace

import casadi as ca
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "trajectory_optimization_ocp"))

from utils import (  # noqa: E402, I202
    activate_constraint,
    activate_upper_bounded_constraint,
    conservative_smooth_sat_margin,
    ego_obb_geometry,
    expand_ego_obb_forward,
    saturate_positive_margin,
)


def _box(x, y, yaw, half_length, half_width):
    return {
        "x": x,
        "y": y,
        "long_axis": ca.vertcat(ca.cos(yaw), ca.sin(yaw)),
        "lat_axis": ca.vertcat(-ca.sin(yaw), ca.cos(yaw)),
        "half_length": half_length,
        "half_width": half_width,
    }


def _exact_sat(values):
    x1, y1, yaw1, hl1, hw1, x2, y2, yaw2, hl2, hw2 = values
    axes1 = ((math.cos(yaw1), math.sin(yaw1)), (-math.sin(yaw1), math.cos(yaw1)))
    axes2 = ((math.cos(yaw2), math.sin(yaw2)), (-math.sin(yaw2), math.cos(yaw2)))
    gaps = []

    def dot(lhs, rhs):
        return lhs[0] * rhs[0] + lhs[1] * rhs[1]

    for axis in (*axes1, *axes2):
        support1 = hl1 * abs(dot(axes1[0], axis)) + hw1 * abs(dot(axes1[1], axis))
        support2 = hl2 * abs(dot(axes2[0], axis)) + hw2 * abs(dot(axes2[1], axis))
        gaps.append(abs(dot((x2 - x1, y2 - y1), axis)) - support1 - support2)
    return max(gaps)


def test_smooth_sat_separation_is_conservative():
    """A positive smooth margin must always imply exact SAT separation."""
    values = ca.MX.sym("values", 10)
    first = _box(values[0], values[1], values[2], values[3], values[4])
    second = _box(values[5], values[6], values[7], values[8], values[9])
    function = ca.Function("smooth_sat", [values], [conservative_smooth_sat_margin(first, second, 1e-3, 0.02)])
    generator = np.random.default_rng(42)
    for _ in range(10000):
        sample = np.array(
            [
                *generator.uniform(-20.0, 20.0, 2),
                generator.uniform(-math.pi, math.pi),
                *generator.uniform(0.05, 8.0, 2),
                *generator.uniform(-20.0, 20.0, 2),
                generator.uniform(-math.pi, math.pi),
                *generator.uniform(0.05, 8.0, 2),
            ]
        )
        assert float(function(sample)) <= _exact_sat(sample) + 1e-12


def test_offsets_forward_expansion_and_inactive_slots():
    """Offsets and margins are directional, while unused slots are constant."""
    state = ca.MX.sym("state", 7)
    ocp = SimpleNamespace(model=SimpleNamespace(x=state))
    physical = ego_obb_geometry(ocp, {"length": 4.0, "width": 2.0, "offset2geocenter": [3.0, 0.5]})
    expanded = expand_ego_obb_forward(physical, 20.0, 0.1, 0.2)
    active = ca.MX.sym("active")
    constraint = activate_constraint(expanded["x"], active)
    function = ca.Function(
        "geometry",
        [state, active],
        [physical["x"], physical["y"], expanded["half_length"], constraint, ca.jacobian(constraint, state)],
    )
    values = np.zeros(7)
    values[0:2] = (1.0, 2.0)
    values[5] = math.pi / 2.0
    x, y, half_length, inactive, derivative = function(values, 0.0)
    assert math.isclose(float(x), 0.5, abs_tol=1e-12)
    assert math.isclose(float(y), 5.0, abs_tol=1e-12)
    assert math.isclose(float(half_length), 12.05, abs_tol=1e-12)
    assert math.isclose(float(inactive), 1.0, abs_tol=1e-12)
    np.testing.assert_array_equal(np.asarray(derivative), np.zeros((1, 7)))


def test_inactive_upper_bounded_constraint_is_constant_and_feasible():
    """Disabled boundary constraints must not depend on the state and must satisfy h <= 0."""
    value = ca.MX.sym("value")
    active = ca.MX.sym("active")
    constraint = activate_upper_bounded_constraint(value, active)
    function = ca.Function("upper_activation", [value, active], [constraint, ca.jacobian(constraint, value)])

    inactive, inactive_derivative = (float(result) for result in function(42.0, 0.0))
    assert inactive == -1.0
    assert inactive_derivative == 0.0

    active_value, active_derivative = (float(result) for result in function(0.25, 1.0))
    assert active_value == 0.25
    assert active_derivative == 1.0


def test_positive_margin_saturation_preserves_sign_and_contact_gradient():
    """Saturation must not change feasibility or the gradient at contact."""
    margin = ca.MX.sym("margin")
    saturated = saturate_positive_margin(margin, 0.5)
    function = ca.Function("saturation", [margin], [saturated, ca.jacobian(saturated, margin)])
    for sample in (-10.0, -1e-6, 0.0, 1e-6, 10.0):
        result, _ = (float(value) for value in function(sample))
        assert (result >= 0.0) == (sample >= 0.0)
    _, gradient = (float(value) for value in function(0.0))
    assert math.isclose(gradient, 1.0, abs_tol=1e-12)

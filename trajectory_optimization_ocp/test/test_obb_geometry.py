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

from utils import ego_obb_geometry, smooth_abs_lower, smooth_abs_upper, smooth_max_lower  # noqa: E402, I202


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

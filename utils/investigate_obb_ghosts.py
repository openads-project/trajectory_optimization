#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Probe whether inactive OBB constraints disturb the generated Karl solver."""

import argparse
import importlib
import math
import statistics
import sys
from pathlib import Path

import numpy as np
import yaml
from acados_template import AcadosOcp, AcadosOcpSolver

REPO_ROOT = Path(__file__).resolve().parents[1]
OCP_MODULE_DIR = REPO_ROOT / "trajectory_optimization_ocp" / "trajectory_optimization_ocp"
OCP_BUILD_DIR = REPO_ROOT.parents[1] / "build" / "trajectory_optimization_ocp"
N_OBSTACLES = 30
GHOST = (10000.0, 10000.0, 0.0, 0.05, 0.05)


def load_configuration():
    """Load the generated OCP configuration and current ROS parameters."""
    with (OCP_MODULE_DIR / "config" / "karl_params.yml").open(encoding="utf-8") as config_file:
        ocp_config = yaml.safe_load(config_file)
    with (REPO_ROOT / "trajectory_optimization" / "config" / "example_params_ackermann.yml").open(
        encoding="utf-8"
    ) as params_file:
        node_params = yaml.safe_load(params_file)["/**"]["ros__parameters"]
    ocp_config["model_name"] = "karl_obb_sat"
    ocp_config["collision_geometry"] = "obb_sat"
    return ocp_config, node_params


def create_solver(config):
    """Load the already generated solver while reconstructing its Python metadata."""
    sys.path[:0] = [str(OCP_MODULE_DIR), str(OCP_MODULE_DIR / "models")]
    constraints = importlib.import_module("constraints")
    costs = importlib.import_module("costs")
    model_ackermann = importlib.import_module("model_Ackermann")
    opts = importlib.import_module("opts")

    ocp = AcadosOcp()
    model_ackermann.set_model(ocp, config)
    costs.set_costs(ocp, config)
    constraints.set_constraints(ocp, config)
    opts.set_opts(ocp, config)
    model_name = config["model_name"]
    ocp.code_gen_options.code_export_directory = str(OCP_BUILD_DIR / "c_generated_code")
    ocp.code_gen_options.json_file = str(OCP_BUILD_DIR / f"{model_name}.json")
    solver_library = OCP_BUILD_DIR / "c_generated_code" / f"libacados_ocp_solver_{model_name}.so"
    if not solver_library.exists():
        raise FileNotFoundError(f"Generated solver not found at {solver_library}. Run the Release build first.")
    return AcadosOcpSolver(ocp, build=False, generate=False, verbose=False, save_p_global=False, check_reuse_possible=False)


def exact_sat_margin(first, second):
    """Compute the exact separating-axis margin of two numeric OBBs."""
    first_axes = ((math.cos(first[2]), math.sin(first[2])), (-math.sin(first[2]), math.cos(first[2])))
    second_axes = ((math.cos(second[2]), math.sin(second[2])), (-math.sin(second[2]), math.cos(second[2])))
    difference = (second[0] - first[0], second[1] - first[1])

    def support(box, box_axes, axis):
        return box[3] * abs(np.dot(box_axes[0], axis)) + box[4] * abs(np.dot(box_axes[1], axis))

    return max(
        abs(np.dot(difference, axis)) - support(first, first_axes, axis) - support(second, second_axes, axis)
        for axis in (*first_axes, *second_axes)
    )


def smooth_sat_margin(first, second, epsilon=1e-3, tau=0.02):
    """Compute the conservative smooth SAT margin used by the OCP."""
    first_axes = ((math.cos(first[2]), math.sin(first[2])), (-math.sin(first[2]), math.cos(first[2])))
    second_axes = ((math.cos(second[2]), math.sin(second[2])), (-math.sin(second[2]), math.cos(second[2])))
    difference = (second[0] - first[0], second[1] - first[1])

    def smooth_abs_upper(value):
        return math.hypot(value, epsilon)

    def support(box, box_axes, axis):
        return box[3] * smooth_abs_upper(np.dot(box_axes[0], axis)) + box[4] * smooth_abs_upper(np.dot(box_axes[1], axis))

    gaps = []
    for axis in (*first_axes, *second_axes):
        center_projection = math.hypot(np.dot(difference, axis), epsilon) - epsilon
        gaps.append(center_projection - support(first, first_axes, axis) - support(second, second_axes, axis))

    def smooth_max_lower(first_value, second_value):
        return 0.5 * (first_value + second_value + math.hypot(first_value - second_value, tau) - tau)

    return smooth_max_lower(smooth_max_lower(gaps[0], gaps[1]), smooth_max_lower(gaps[2], gaps[3]))


def safety_ego_box(state, config, params):
    """Construct the forward-expanded Ego OBB corresponding to an OCP state."""
    yaw = state[5]
    physical_center_x = state[0] + config["offset2geocenter"][0] * math.cos(yaw)
    physical_center_y = state[1] + config["offset2geocenter"][0] * math.sin(yaw)
    rear_margin = max(0.0, params["d_min_obstacle_long"])
    front_margin = max(rear_margin, params["thw"] * max(0.0, state[3]))
    center_shift = 0.5 * (front_margin - rear_margin)
    return (
        physical_center_x + center_shift * math.cos(yaw),
        physical_center_y + center_shift * math.sin(yaw),
        yaw,
        0.5 * config["length"] + 0.5 * (front_margin + rear_margin),
        0.5 * config["width"] + max(0.0, params["d_min_obstacle_lat"]),
    )


def make_clutter(count):
    """Create static OBBs that cannot intersect the straight drivable corridor."""
    clutter = []
    for index in range(count):
        if index % 3 == 0:
            # The OCP cannot drive backwards; these boxes stay safely behind it.
            box = (-9.0 - 0.4 * index, 0.0, 0.15 * (index % 4), 2.4, 1.0)
        else:
            # The route boundaries are at y=+-4 m, while these boxes start at |y|=8 m.
            x = 2.0 + 2.2 * (index % 12)
            y = 8.0 if index % 2 == 0 else -8.0
            yaw = (0.0, math.pi / 2.0, 0.35)[index % 3]
            box = (x, y, yaw, 2.4, 1.0)
        clutter.append(box)
    return clutter


def initialize_problem(solver, config, params, obstacles, speed, filler=GHOST, boundary_half_width=4.0):
    """Set a straight reference, OCP parameters, and a dynamically consistent initial guess."""
    horizon = config["optimization_horizon"]
    stages = config["n_shots"]
    dt = horizon / stages
    reference = []
    for stage in range(stages + 1):
        reference.extend((0.0, speed * stage * dt, 0.0, speed, boundary_half_width, boundary_half_width))
    global_parameters = np.asarray(
        [
            *params["cost_weights"],
            params["thw"],
            params["d_min_obstacle_long"],
            params["d_min_obstacle_lat"],
            0.0,
            *reference,
        ],
        dtype=float,
    )
    solver.set_p_global_and_precompute_dependencies(global_parameters)

    active_obstacles = [(*box, 1.0) for box in obstacles]
    inactive_obstacles = [(*filler, 0.0)] * (N_OBSTACLES - len(obstacles))
    padded_obstacles = [*active_obstacles, *inactive_obstacles]
    flat_obstacles = [value for box in padded_obstacles for value in box]
    stage_parameters = np.asarray([params["dynamic_weight"], *flat_obstacles])
    initial_state = np.asarray([0.0, 0.0, 0.0, speed, 0.0, 0.0, 0.0])
    solver.set(0, "lbx", initial_state)
    solver.set(0, "ubx", initial_state)
    for stage in range(stages + 1):
        state = np.asarray([speed * stage * dt, 0.0, speed * stage * dt, speed, 0.0, 0.0, 0.0])
        solver.set(stage, "x", state)
        solver.set(stage, "p", stage_parameters)
        if stage < stages:
            solver.set(stage, "u", np.zeros(2))


def minimum_margins(states, obstacles, config, params):
    """Return minimum exact and smooth margins over all supplied real obstacles."""
    if not obstacles:
        return math.inf, math.inf
    exact = math.inf
    smooth = math.inf
    for state in states:
        ego_box = safety_ego_box(state, config, params)
        for obstacle in obstacles:
            exact = min(exact, exact_sat_margin(ego_box, obstacle))
            smooth = min(smooth, smooth_sat_margin(ego_box, obstacle))
    return exact, smooth


def run_case(
    solver,
    config,
    params,
    name,
    obstacles,
    margin_obstacles,
    speed,
    repeats,
    filler=GHOST,
    boundary_half_width=4.0,
):
    """Solve one synthetic case repeatedly from the same cold initial guess."""
    results = []
    dt = config["optimization_horizon"] / config["n_shots"]
    initial_states = [
        np.asarray([speed * stage * dt, 0.0, speed * stage * dt, speed, 0.0, 0.0, 0.0]) for stage in range(config["n_shots"] + 1)
    ]
    initial_exact, initial_smooth = minimum_margins(initial_states, margin_obstacles, config, params)
    for _ in range(repeats):
        solver.reset(reset_qp_solver_mem=True, reset_numerical_values=True)
        initialize_problem(solver, config, params, obstacles, speed, filler, boundary_half_width)
        status = solver.solve()
        states = np.vstack([solver.get(stage, "x") for stage in range(config["n_shots"] + 1)])
        controls = np.vstack([solver.get(stage, "u") for stage in range(config["n_shots"])])
        exact, smooth = minimum_margins(states, margin_obstacles, config, params)
        results.append(
            {
                "status": status,
                "iterations": int(solver.get_stats("sqp_iter")),
                "time_ms": 1000.0 * float(solver.get_stats("time_tot")),
                "residuals": np.asarray(solver.get_residuals()),
                "states": states,
                "controls": controls,
                "exact": exact,
                "smooth": smooth,
            }
        )
    return {
        "name": name,
        "results": results,
        "initial_exact": initial_exact,
        "initial_smooth": initial_smooth,
    }


def format_margin(value):
    """Format finite margins while keeping the no-obstacle case readable."""
    return "-" if math.isinf(value) else f"{value:.3f}"


def filler_yaw_sensitivity(config, params, speed, distance):
    """Estimate first and second yaw derivatives for a diagonal dummy box."""
    obstacle = (distance, distance, 0.0, 0.05, 0.05)

    def margin(yaw):
        state = [0.0, 0.0, 0.0, speed, 0.0, yaw, 0.0]
        return smooth_sat_margin(safety_ego_box(state, config, params), obstacle)

    step = 1e-5
    center = margin(0.0)
    first = (margin(step) - margin(-step)) / (2.0 * step)
    second = (margin(step) - 2.0 * center + margin(-step)) / (step * step)
    return center, first, second


def main():
    """Run the synthetic clutter investigation and print a compact comparison."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=3, help="cold solves per scenario")
    parser.add_argument("--speed", type=float, default=5.0, help="reference speed in m/s")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be at least one")

    config, params = load_configuration()
    solver = create_solver(config)
    cases = []
    for count in (0, 1, 5, 10, 20, 30):
        clutter = make_clutter(count)
        cases.append(run_case(solver, config, params, f"clutter_{count}", clutter, clutter, args.speed, args.repeats))

    clutter = make_clutter(20)
    permutation = list(reversed(clutter))
    cases.append(run_case(solver, config, params, "clutter_20_reversed", permutation, permutation, args.speed, args.repeats))

    relevant = [(18.0, 0.0, 0.0, 2.4, 1.0)]
    cases.append(run_case(solver, config, params, "blocking_only", relevant, relevant, args.speed, args.repeats))
    cases.append(
        run_case(
            solver,
            config,
            params,
            "blocking_stop_only",
            relevant,
            relevant,
            args.speed,
            args.repeats,
            boundary_half_width=1.25,
        )
    )
    for filler_distance in (10.0, 100.0, 1000.0):
        cases.append(
            run_case(
                solver,
                config,
                params,
                f"blocking_diag_{int(filler_distance)}",
                relevant,
                relevant,
                args.speed,
                args.repeats,
                (filler_distance, filler_distance, 0.0, 0.05, 0.05),
            )
        )
    cases.append(
        run_case(
            solver,
            config,
            params,
            "blocking_rear_fill",
            relevant,
            relevant,
            args.speed,
            args.repeats,
            (-100.0, 0.0, 0.0, 0.05, 0.05),
        )
    )
    cases.append(
        run_case(
            solver,
            config,
            params,
            "blocking_side_fill",
            relevant,
            relevant,
            args.speed,
            args.repeats,
            (0.0, 20.0, 0.0, 0.05, 0.05),
        )
    )
    mixed = [*relevant, *make_clutter(29)]
    cases.append(run_case(solver, config, params, "blocking_plus_29", mixed, relevant, args.speed, args.repeats))

    offset_relevant = [(18.0, 0.5, 0.0, 2.4, 1.0)]
    cases.append(
        run_case(solver, config, params, "offset_blocking_only", offset_relevant, offset_relevant, args.speed, args.repeats)
    )
    offset_mixed = [*offset_relevant, *make_clutter(29)]
    cases.append(
        run_case(solver, config, params, "offset_blocking_plus", offset_mixed, offset_relevant, args.speed, args.repeats)
    )

    baseline = cases[0]["results"][0]
    print(
        "scenario              status  iter  time_med_ms  max|dx|   max|du|   min_v  max|y| "
        "init_exact init_smooth sol_exact sol_smooth max_residual  max_eq max_ineq"
    )
    for case in cases:
        representative = case["results"][0]
        statuses = sorted({result["status"] for result in case["results"]})
        iterations = [result["iterations"] for result in case["results"]]
        times = [result["time_ms"] for result in case["results"]]
        max_state_delta = np.max(np.abs(representative["states"] - baseline["states"]))
        max_control_delta = np.max(np.abs(representative["controls"] - baseline["controls"]))
        max_residual = max(np.max(result["residuals"]) for result in case["results"])
        max_eq = max(abs(result["residuals"][1]) for result in case["results"])
        max_ineq = max(abs(result["residuals"][2]) for result in case["results"])
        print(
            f"{case['name']:<21} {str(statuses):>6} {statistics.median(iterations):5.1f} "
            f"{statistics.median(times):12.3f} {max_state_delta:9.2e} {max_control_delta:9.2e} "
            f"{np.min(representative['states'][:, 3]):6.2f} {np.max(np.abs(representative['states'][:, 1])):7.2f} "
            f"{format_margin(case['initial_exact']):>10} {format_margin(case['initial_smooth']):>11} "
            f"{format_margin(representative['exact']):>9} {format_margin(representative['smooth']):>10} "
            f"{max_residual:12.2e} {max_eq:7.1e} {max_ineq:8.1e}"
        )

    blocking_reference = next(case for case in cases if case["name"] == "blocking_only")["results"][0]
    print("\ninactive filler influence relative to blocking_only: scenario  max|dx|  max|du|")
    for case in cases:
        if case["name"].startswith("blocking_diag_") or case["name"] in {
            "blocking_rear_fill",
            "blocking_side_fill",
        }:
            representative = case["results"][0]
            state_delta = np.max(np.abs(representative["states"] - blocking_reference["states"]))
            control_delta = np.max(np.abs(representative["controls"] - blocking_reference["controls"]))
            print(f"{case['name']:69} {state_delta:9.2e} {control_delta:9.2e}")

    print(
        "\nraw diagonal SAT before activation (active=0 makes the effective margin 1 and both derivatives 0):"
        "\n distance  raw_margin  |raw_dh/dyaw|  |raw_d2h/dyaw2|"
    )
    for distance in (10.0, 100.0, 1000.0, 10000.0):
        margin, first, second = filler_yaw_sensitivity(config, params, args.speed, distance)
        print(f"{distance:9.0f} {margin:11.1f} {abs(first):15.2e} {abs(second):17.2e}")


if __name__ == "__main__":
    main()

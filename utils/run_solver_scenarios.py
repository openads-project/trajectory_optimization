#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Compare circle, OBB single-start and OBB multistart on controlled scenes."""

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "utils"))
from investigate_obb_ghosts import create_solver, exact_sat_margin  # noqa: E402, I202

OCP_CONFIG = REPO_ROOT / "trajectory_optimization_ocp" / "trajectory_optimization_ocp" / "config" / "karl_params.yml"
NODE_CONFIG = REPO_ROOT / "trajectory_optimization" / "config" / "example_params_ackermann.yml"
OBSTACLE_CAPACITY = 30
MODES = ("warm_start", "cold_start", "braking", "left", "right", "braking_left", "braking_right")


@dataclass(frozen=True)
class Scenario:
    """Deterministic straight-road scenario with static OBB obstacles."""

    name: str
    speed: float
    obstacles: tuple
    boundary_half_width: float
    expected_feasible: bool


def load_configuration(geometry):
    """Load one generated Karl solver configuration and shared node parameters."""
    with OCP_CONFIG.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    with NODE_CONFIG.open(encoding="utf-8") as params_file:
        params = yaml.safe_load(params_file)["/**"]["ros__parameters"]
    config["collision_geometry"] = geometry
    config["model_name"] = "karl_obb_sat" if geometry == "obb_sat" else "karl"
    return config, params


def obstacle_circles(box):
    """Apply the runtime node's conservative OBB-to-circle discretization."""
    x, y, yaw, half_length, half_width = box
    length = 2.0 * half_length
    width = 2.0 * half_width
    aspect_ratio = length / width
    if aspect_ratio > 8.0:
        count = 9
    elif aspect_ratio > 6.0:
        count = 7
    elif aspect_ratio > 4.0:
        count = 5
    elif aspect_ratio > 1.8:
        count = 3
    elif aspect_ratio > 1.3:
        count = 2
    else:
        count = 1
    radius = math.hypot(length / (2.0 * count), width / 2.0)
    circles = []
    for index in range(count):
        longitudinal_offset = -length / 2.0 + (2 * index + 1) * length / (2.0 * count)
        circles.append(
            (
                x + longitudinal_offset * math.cos(yaw),
                y + longitudinal_offset * math.sin(yaw),
                radius,
            )
        )
    return circles


def braking_jerk(velocity, acceleration, dt, release_started, config):
    """Python equivalent of the node's jerk-limited forward-only braking seed."""
    jerk_min = config["jerk_min"]
    jerk_max = config["jerk_max"]
    acceleration_min = config["acceleration_t_min"]
    if release_started:
        return (min(jerk_max, max(jerk_min, -acceleration / dt)) if acceleration < 0.0 else 0.0), True

    jerk_to_limit = (acceleration_min - acceleration) / dt
    brake = jerk_min if acceleration > acceleration_min - jerk_min * dt else min(jerk_max, max(jerk_min, jerk_to_limit))

    def switching_function(jerk):
        next_acceleration = acceleration + jerk * dt
        next_velocity = velocity + acceleration * dt + 0.5 * jerk * dt * dt
        release_velocity = next_acceleration**2 / (2.0 * jerk_max) if next_acceleration < 0.0 else 0.0
        return next_velocity - release_velocity

    if acceleration >= 0.0 or switching_function(brake) > 0.0:
        return brake, False
    if switching_function(jerk_max) <= 0.0:
        return jerk_max, True
    lower = brake
    upper = jerk_max
    for _ in range(40):
        middle = 0.5 * (lower + upper)
        if switching_function(middle) > 0.0:
            upper = middle
        else:
            lower = middle
    return 0.5 * (lower + upper), True


def derivative(state, control, config):
    """Evaluate the generated Ackermann model dynamics numerically."""
    velocity = state[3]
    yaw = state[5]
    steering = state[6]
    return np.asarray(
        [
            velocity * math.cos(yaw),
            velocity * math.sin(yaw),
            velocity,
            state[4],
            control[0],
            velocity / config["wheelbase"] * math.tan(steering),
            control[1],
        ]
    )


def integrate(state, control, dt, config):
    """Match the OCP's ERK4, two-step integration for seed rollout."""
    result = state.copy()
    step = dt / 2.0
    for _ in range(2):
        k1 = derivative(result, control, config)
        k2 = derivative(result + 0.5 * step * k1, control, config)
        k3 = derivative(result + 0.5 * step * k2, control, config)
        k4 = derivative(result + step * k3, control, config)
        result += step / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return result


def initial_iterate(mode, speed, config):
    """Construct the same deterministic state/control seeds as the C++ node."""
    stages = config["n_shots"]
    horizon = config["optimization_horizon"]
    dt = horizon / stages
    state = np.asarray([0.0, 0.0, 0.0, speed, 0.0, 0.0, 0.0])
    states = [state.copy()]
    controls = []
    release_started = False
    for stage in range(stages):
        jerk = 0.0
        if mode in ("braking", "braking_left", "braking_right"):
            jerk, release_started = braking_jerk(state[3], state[4], dt, release_started, config)
        steering_rate = 0.0
        if mode in ("left", "right", "braking_left", "braking_right"):
            direction = 1.0 if mode in ("left", "braking_left") else -1.0
            steering_rate = direction * 0.15 * config["alpha_max"] * math.cos(2.0 * math.pi * (stage * dt + 0.5 * dt) / horizon)
            steering_rate = min(config["alpha_max"], max(-config["alpha_max"], steering_rate))
            steering_rate = min(steering_rate, (config["delta_max"] - state[6]) / dt)
            steering_rate = max(steering_rate, (-config["delta_max"] - state[6]) / dt)
        control = np.asarray([jerk, steering_rate])
        controls.append(control)
        state = integrate(state, control, dt, config)
        states.append(state.copy())
    return np.vstack(states), np.vstack(controls)


def set_problem(solver, config, params, scenario, mode):
    """Set global/stage parameters, hard initial state and one initial iterate."""
    stages = config["n_shots"]
    dt = config["optimization_horizon"] / stages
    reference = []
    for stage in range(stages + 1):
        reference.extend(
            (0.0, scenario.speed * stage * dt, 0.0, scenario.speed, scenario.boundary_half_width, scenario.boundary_half_width)
        )
    global_parameters = np.asarray(
        [
            *params["cost_weights"],
            params["thw"],
            params["d_min_obstacle_long"],
            params["d_min_obstacle_lat"],
            0.0 if config["collision_geometry"] == "obb_sat" else params["d_min_boundary_lat"],
            *reference,
        ],
        dtype=float,
    )
    solver.set_p_global_and_precompute_dependencies(global_parameters)

    if config["collision_geometry"] == "obb_sat":
        active = [(*box, 1.0) for box in scenario.obstacles]
        padded = [*active, *([(0.0, 0.0, 0.0, 0.05, 0.05, 0.0)] * (OBSTACLE_CAPACITY - len(active)))]
    else:
        active = [circle for box in scenario.obstacles for circle in obstacle_circles(box)]
        padded = [*active, *([(10000.0, 10000.0, 1.0)] * (OBSTACLE_CAPACITY - len(active)))]
    if len(active) > OBSTACLE_CAPACITY:
        raise ValueError(f"Scenario {scenario.name} exceeds the generated obstacle capacity")

    states, controls = initial_iterate(mode, scenario.speed, config)
    initial_state = states[0]
    solver.set(0, "lbx", initial_state)
    solver.set(0, "ubx", initial_state)
    dynamic_weight = 1.0
    for stage in range(stages + 1):
        solver.set(stage, "x", states[stage])
        solver.set(stage, "p", np.asarray([dynamic_weight, *(value for obstacle in padded for value in obstacle)]))
        dynamic_weight *= params["dynamic_weight"]
        if stage < stages:
            solver.set(stage, "u", controls[stage])


def physical_ego_box(state, config):
    """Construct the exact physical Ego OBB without THW or safety margins."""
    yaw = state[5]
    offset_long, offset_lat = config["offset2geocenter"]
    return (
        state[0] + offset_long * math.cos(yaw) - offset_lat * math.sin(yaw),
        state[1] + offset_long * math.sin(yaw) + offset_lat * math.cos(yaw),
        yaw,
        0.5 * config["length"],
        0.5 * config["width"],
    )


def validate_exact(states, config, scenario):
    """Return exact node and ten-intersample collision/boundary counts."""
    node_violations = 0
    intersample_violations = 0

    def violates(state):
        ego = physical_ego_box(state, config)
        collision = any(exact_sat_margin(ego, obstacle) < -1e-7 for obstacle in scenario.obstacles)
        lateral_support = ego[3] * abs(math.sin(ego[2])) + ego[4] * abs(math.cos(ego[2]))
        boundary = abs(ego[1]) + lateral_support > scenario.boundary_half_width + 1e-7
        return collision or boundary

    for state in states:
        node_violations += int(violates(state))
    for first, second in zip(states[:-1], states[1:]):
        for fraction in np.linspace(0.0, 1.0, 12)[1:-1]:
            state = first + fraction * (second - first)
            yaw_delta = math.atan2(math.sin(second[5] - first[5]), math.cos(second[5] - first[5]))
            state[5] = first[5] + fraction * yaw_delta
            intersample_violations += int(violates(state))
    return node_violations, intersample_violations


def solve_attempt(solver, config, params, scenario, mode):
    """Run one cold acados solve and return normalized diagnostics."""
    solver.reset(reset_qp_solver_mem=True, reset_numerical_values=True)
    set_problem(solver, config, params, scenario, mode)
    seed_residuals = np.asarray(solver.get_residuals(recompute=True))
    seed_states = np.vstack([solver.get(stage, "x") for stage in range(config["n_shots"] + 1)])
    seed_controls = np.vstack([solver.get(stage, "u") for stage in range(config["n_shots"])])
    seed_finite = np.all(np.isfinite(seed_states)) and np.all(np.isfinite(seed_controls)) and np.all(np.isfinite(seed_residuals))
    seed_feasible = bool(seed_finite and seed_residuals[1] <= 1e-3 and seed_residuals[2] <= 1e-4)
    seed_cost = float(solver.get_cost()) if seed_finite else math.inf
    status = solver.solve()
    residuals = np.asarray(solver.get_residuals())
    states = np.vstack([solver.get(stage, "x") for stage in range(config["n_shots"] + 1)])
    controls = np.vstack([solver.get(stage, "u") for stage in range(config["n_shots"])])
    finite = np.all(np.isfinite(states)) and np.all(np.isfinite(controls)) and np.all(np.isfinite(residuals))
    feasible = bool(finite and status in (0, 2, 7) and residuals[1] <= 1e-3 and residuals[2] <= 1e-4)
    node_violations, intersample_violations = validate_exact(states, config, scenario) if finite else (-1, -1)
    solve_time_ms = 1000.0 * float(solver.get_stats("time_tot"))
    result = {
        "mode": mode,
        "status": status,
        "feasible": feasible,
        "cost": float(solver.get_cost()) if finite else math.inf,
        "time_ms": solve_time_ms,
        "attempt_time_ms": solve_time_ms,
        "res_eq": float(residuals[1]),
        "res_ineq": float(residuals[2]),
        "node_violations": node_violations,
        "intersample_violations": intersample_violations,
        "min_velocity": float(np.min(states[:, 3])) if finite else math.nan,
        "max_lateral": float(np.max(np.abs(states[:, 1]))) if finite else math.nan,
    }
    if seed_feasible and (not feasible or seed_cost < result["cost"]):
        node_violations, intersample_violations = validate_exact(seed_states, config, scenario)
        result.update(
            {
                "mode": f"{mode}_seed",
                "status": 0,
                "feasible": True,
                "cost": seed_cost,
                "time_ms": 0.0,
                "res_eq": float(seed_residuals[1]),
                "res_ineq": float(seed_residuals[2]),
                "node_violations": node_violations,
                "intersample_violations": intersample_violations,
                "min_velocity": float(np.min(seed_states[:, 3])),
                "max_lateral": float(np.max(np.abs(seed_states[:, 1]))),
            }
        )
    return result


def run_variant(variant, scenarios, repeats):
    """Run all scenes with one geometry/start strategy."""
    geometry = "circles" if variant == "circles" else "obb_sat"
    modes = ("warm_start",) if variant != "obb_multistart" else MODES
    config, params = load_configuration(geometry)
    solvers = [create_solver(config) for _ in modes]
    rows = []
    for scenario in scenarios:
        for repetition in range(repeats):
            attempts = [solve_attempt(solver, config, params, scenario, mode) for solver, mode in zip(solvers, modes)]
            feasible = [attempt for attempt in attempts if attempt["feasible"]]
            selected = (
                min(feasible, key=lambda attempt: attempt["cost"])
                if feasible
                else min(attempts, key=lambda attempt: max(attempt["res_eq"] / 1e-3, attempt["res_ineq"] / 1e-4))
            )
            selected = dict(selected)
            selected.update(
                {
                    "variant": variant,
                    "scenario": scenario.name,
                    "repetition": repetition,
                    "expected_feasible": scenario.expected_feasible,
                    "attempts": len(attempts),
                    "parallel_wall_ms": max(attempt["attempt_time_ms"] for attempt in attempts),
                    "serial_wall_ms": sum(attempt["attempt_time_ms"] for attempt in attempts),
                    "feasible_attempts": len(feasible),
                }
            )
            rows.append(selected)
    return rows


def scenarios():
    """Return scenes with analytically clear initial feasibility."""
    blocker = (18.0, 0.0, 0.0, 2.4, 1.0)
    clutter = tuple((5.0 + 2.5 * index, 8.0 if index % 2 == 0 else -8.0, 0.2 * (index % 3), 2.4, 1.0) for index in range(8))
    return (
        Scenario("clear_road", 5.0, (), 4.0, True),
        Scenario("feasible_stop", 5.0, (blocker,), 1.25, True),
        # At x=16 m the THW-expanded Ego OBB already overlaps the blocker at
        # shooting node zero. Since x0 is hard, no valid OCP solution exists.
        Scenario("infeasible_initial_overlap", 5.0, ((16.0, 0.0, 0.0, 2.4, 1.0),), 1.25, False),
        Scenario("lateral_or_stop", 5.0, (blocker,), 4.0, True),
        Scenario("irrelevant_parked", 5.0, clutter, 4.0, True),
        Scenario("blocker_with_clutter", 5.0, (blocker, *clutter), 1.25, True),
    )


def print_report(rows):
    """Print one compact comparison table and aggregate summary."""
    print(
        "variant         scenario                    ok mode           feasible time_ms parallel_ms cost      "
        "eq       ineq     node inter min_v max_y"
    )
    for row in rows:
        print(
            f"{row['variant']:<15} {row['scenario']:<27} {int(row['feasible']):2d} {row['mode']:<14} "
            f"{row['feasible_attempts']:2d}/{row['attempts']:<2d} {row['time_ms']:7.2f} {row['parallel_wall_ms']:11.2f} "
            f"{row['cost']:9.4f} {row['res_eq']:8.1e} {row['res_ineq']:8.1e} {row['node_violations']:4d} "
            f"{row['intersample_violations']:5d} {row['min_velocity']:5.2f} {row['max_lateral']:5.2f}"
        )
    print("\naggregate (median selected solve / conceptual parallel wall):")
    for variant in ("circles", "obb_single", "obb_multistart"):
        selected = [row for row in rows if row["variant"] == variant]
        expected = [row for row in selected if row["expected_feasible"]]
        print(
            f"{variant:<15} feasible={sum(row['feasible'] for row in expected)}/{len(expected)} "
            f"safe_nodes={sum(row['node_violations'] == 0 for row in expected)}/{len(expected)} "
            f"selected_med={statistics.median(row['time_ms'] for row in selected):.2f} ms "
            f"parallel_med={statistics.median(row['parallel_wall_ms'] for row in selected):.2f} ms"
        )


def validate_standard(rows):
    """Return human-readable failures of the proposed OBB multistart standard."""
    failures = []
    standard = [row for row in rows if row["variant"] == "obb_multistart"]
    for row in standard:
        if row["expected_feasible"] and not row["feasible"]:
            failures.append(f"{row['scenario']}: expected a feasible solution")
        if not row["expected_feasible"] and row["feasible"]:
            failures.append(f"{row['scenario']}: expected rejection of an infeasible x0")
        if row["feasible"] and row["node_violations"] != 0:
            failures.append(f"{row['scenario']}: exact node validator found {row['node_violations']} violations")
    return failures


def main():
    """Run deterministic comparisons and fail if the proposed standard violates its logical gates."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, default=1)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be at least one")
    scene_list = scenarios()
    rows = []
    for variant in ("circles", "obb_single", "obb_multistart"):
        rows.extend(run_variant(variant, scene_list, args.repeats))
    print_report(rows)
    failures = validate_standard(rows)
    if failures:
        raise SystemExit("Scenario gates failed:\n- " + "\n- ".join(failures))


if __name__ == "__main__":
    main()

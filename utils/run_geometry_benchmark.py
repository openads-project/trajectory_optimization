#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Benchmark circles, OBB single-start, and parallel OBB multistart on input-only bags."""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml

INPUT_TOPICS = [
    "/localization/ego_state_estimation/ego_data",
    "/understanding/lanelet2_object_list_prediction/predicted_object_list",
    "/planning/lanelet2_route_planning/route",
    "/planning/simple_planner/trajectory",
    "/tf",
    "/tf_static",
]

REMAPS = [
    "ego_data_topic:=/localization/ego_state_estimation/ego_data",
    "object_list_topic:=/understanding/lanelet2_object_list_prediction/predicted_object_list",
    "route_topic:=/planning/lanelet2_route_planning/route",
    "reference_trajectory_topic:=/planning/simple_planner/trajectory",
    "trajectory_topic:=/benchmark/trajectory",
]


def arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", type=Path, nargs="+", help="original rosbag2 directories")
    parser.add_argument("--workspace", type=Path, default=Path("/docker-ros/ws"))
    parser.add_argument("--results", type=Path, default=Path("/tmp/trajectory_optimization_geometry_benchmark"))
    parser.add_argument("--input-cache", type=Path, help="reuse/store filtered input bags independently of --results")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--warmup-seconds", type=float, default=30.0)
    parser.add_argument(
        "--playback-start-offset", type=float, default=0.0, help="start each replay this many seconds into the bag"
    )
    parser.add_argument("--playback-duration", type=float, help="limit measured replays; omit for complete bags")
    parser.add_argument("--node-cpu", default="2-9", help="taskset CPU list for optimizer threads")
    parser.add_argument("--player-cpu", type=int, default=10)
    parser.add_argument("--start-zenoh-router", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--run-tag", default="", help="suffix run directories, e.g. v2 (letters, digits, '_' or '-')")
    parser.add_argument("--resume", action="store_true", help="skip completed schema-v9 runs with matching names")
    parser.add_argument("--status-interval", type=float, default=5.0, help="seconds between progress updates")
    parser.add_argument("--analyze", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--prepare-only", action="store_true")
    return parser.parse_args()


def filtered_bag(source, destination):
    """Create an input-only MCAP without modifying the original recording."""
    if (destination / "metadata.yaml").exists():
        print(f"[prepare] Reusing {destination}", flush=True)
        return destination
    print(f"[prepare] Filtering {source} -> {destination} (this can take several minutes)", flush=True)
    destination.parent.mkdir(parents=True, exist_ok=True)
    options = {
        "output_bags": [
            {
                "uri": str(destination),
                "storage_id": "mcap",
                "all_topics": False,
                "topics": INPUT_TOPICS,
                "all_services": False,
            }
        ]
    }
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as output_options:
        yaml.safe_dump(options, output_options)
        options_path = Path(output_options.name)
    try:
        subprocess.run(
            ["ros2", "bag", "convert", "--input", str(source), "mcap", "--output-options", str(options_path)], check=True
        )
    finally:
        options_path.unlink(missing_ok=True)
    return destination


def benchmark_parameters(source, destination, variant):
    """Copy the repo defaults and change only benchmark instrumentation and geometry."""
    with source.open(encoding="utf-8") as parameter_file:
        parameters = yaml.safe_load(parameter_file)
    values = parameters["/**"]["ros__parameters"]
    geometry = "circles" if variant == "circles" else "obb_sat"
    values["collision_geometry"] = geometry
    values["performance_logging"] = True
    if variant == "obb_single":
        values["multistart_initial_guesses"] = ["warm_start"]
        values["multistart_parallel"] = False
    elif variant == "obb_multistart":
        values["multistart_parallel"] = True
    if geometry == "obb_sat":
        # The circle baseline needs its historic negative margin to compensate
        # its geometric over-approximation. The exact OBB does not.
        values["d_min_boundary_lat"] = 0.0
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as parameter_file:
        yaml.safe_dump(parameters, parameter_file, sort_keys=False)
    return values["d_min_boundary_lat"]


def stop(process):
    """Stop a ROS process group and escalate only if graceful shutdown stalls."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def start(command, environment, log_path):
    """Start one process in its own process group with a persistent log."""
    log = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(command, env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    process.log_file = log
    return process


def bag_duration_seconds(bag):
    """Read the recorded duration from rosbag2 metadata."""
    with (bag / "metadata.yaml").open(encoding="utf-8") as metadata_file:
        metadata = yaml.safe_load(metadata_file)
    return metadata["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9


def wait_with_progress(process, label, expected_seconds, status_interval, watched_processes=()):
    """Wait for a replay while showing a terminal bar or periodic log lines."""
    start_time = time.monotonic()
    next_log = 0.0
    while process.poll() is None:
        for watched_process, description, log_path in watched_processes:
            if watched_process.poll() is not None:
                raise RuntimeError(f"{description} exited during replay; see {log_path}")
        elapsed = time.monotonic() - start_time
        fraction = min(1.0, elapsed / expected_seconds) if expected_seconds > 0.0 else 0.0
        if sys.stdout.isatty():
            width = 30
            filled = int(width * fraction)
            bar = "#" * filled + "-" * (width - filled)
            print(f"\r{label} [{bar}] {100.0 * fraction:5.1f}%  {elapsed:6.1f}/{expected_seconds:.1f}s", end="", flush=True)
        elif elapsed >= next_log:
            print(f"{label}: {100.0 * fraction:5.1f}% ({elapsed:.1f}/{expected_seconds:.1f}s)", flush=True)
            next_log = elapsed + status_interval
        time.sleep(min(1.0, status_interval))
    if sys.stdout.isatty():
        elapsed = time.monotonic() - start_time
        print(f"\r{label} [{'#' * 30}] 100.0%  {elapsed:6.1f}/{expected_seconds:.1f}s", flush=True)
    return process.returncode


def completed_metrics(run_directory):
    """Return the sole current-schema metrics CSV for a completed run, otherwise None."""
    if not (run_directory / "run.json").exists() or not (run_directory / "optimizer_output" / "metadata.yaml").exists():
        return None
    metrics = list((run_directory / "metrics").glob("*.csv"))
    if len(metrics) != 1:
        return None
    with metrics[0].open(encoding="utf-8") as source:
        source.readline()
        first_record = source.readline()
    return metrics[0] if first_record.startswith("9,") else None


def run_once(args, bag, variant, run_name, run_index, total_runs, duration=None):
    """Replay one bag once and capture optimizer metrics and output trajectories."""
    run_directory = args.results / "runs" / run_name
    if run_directory.exists():
        metrics = completed_metrics(run_directory) if args.resume else None
        if metrics is not None:
            print(f"[{run_index}/{total_runs}] RESUME: skipping completed {run_name}", flush=True)
            return metrics
        if not args.resume:
            raise RuntimeError(f"Run directory already exists: {run_directory}. Use --resume or a new --run-tag.")
        archive = run_directory.with_name(f"{run_directory.name}_incomplete_{time.strftime('%Y%m%dT%H%M%S')}")
        suffix = 1
        while archive.exists():
            archive = run_directory.with_name(f"{run_directory.name}_incomplete_{time.strftime('%Y%m%dT%H%M%S')}_{suffix}")
            suffix += 1
        run_directory.rename(archive)
        print(f"[{run_index}/{total_runs}] RESUME: moved incomplete run to {archive}", flush=True)
    run_directory.mkdir(parents=True, exist_ok=False)
    parameter_path = run_directory / "params.yml"
    default_parameters = args.workspace / "src" / "target" / "trajectory_optimization" / "config" / "params.yml"
    boundary_margin = benchmark_parameters(default_parameters, parameter_path, variant)

    environment = os.environ.copy()
    environment["TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR"] = str(run_directory / "metrics")
    environment["TRAJECTORY_OPTIMIZATION_RUN_ID"] = run_name
    active_file = run_directory / ".replay_active"
    environment["TRAJECTORY_OPTIMIZATION_LOG_ACTIVE_FILE"] = str(active_file)
    launch = [
        "taskset",
        "--cpu-list",
        str(args.node_cpu),
        "ros2",
        "launch",
        "trajectory_optimization",
        "trajectory_optimization.launch.py",
        "driving_mode:=ackermann",
        "name:=trajectory_optimization_benchmark",
        f"params:={parameter_path}",
        "use_sim_time:=true",
        *REMAPS,
    ]
    record = [
        "ros2",
        "bag",
        "record",
        "--storage",
        "mcap",
        "--output",
        str(run_directory / "optimizer_output"),
        "/benchmark/trajectory",
    ]
    play = [
        "taskset",
        "--cpu-list",
        str(args.player_cpu),
        "ros2",
        "bag",
        "play",
        str(bag),
        "--clock",
        "100",
        "--disable-keyboard-controls",
    ]
    if args.playback_start_offset > 0.0:
        play.extend(["--start-offset", str(args.playback_start_offset)])
    if duration is not None:
        play.extend(["--playback-duration", str(duration)])

    available_duration = max(0.0, bag_duration_seconds(bag) - args.playback_start_offset)
    expected_duration = min(duration, available_duration) if duration is not None else available_duration
    print(f"[{run_index}/{total_runs}] START {run_name} (~{expected_duration:.1f}s); logs: {run_directory}", flush=True)

    node_process = start(launch, environment, run_directory / "node.log")
    record_process = start(record, environment, run_directory / "record.log")
    try:
        time.sleep(2.0)
        if node_process.poll() is not None:
            raise RuntimeError(f"optimizer exited before replay; see {run_directory / 'node.log'}")
        if record_process.poll() is not None:
            raise RuntimeError(f"recorder exited before replay; see {run_directory / 'record.log'}")
        active_file.touch()
        player_process = start(play, environment, run_directory / "play.log")
        try:
            player_status = wait_with_progress(
                player_process,
                f"[{run_index}/{total_runs}] {variant:14}",
                expected_duration,
                args.status_interval,
                (
                    (node_process, "optimizer", run_directory / "node.log"),
                    (record_process, "recorder", run_directory / "record.log"),
                ),
            )
        finally:
            active_file.unlink(missing_ok=True)
            stop(player_process)
            player_process.log_file.close()
        if player_status != 0:
            raise RuntimeError(f"ros2 bag play failed with status {player_status}; see {run_directory / 'play.log'}")
        time.sleep(1.0)
    finally:
        stop(record_process)
        stop(node_process)
        record_process.log_file.close()
        node_process.log_file.close()

    metadata = {
        "bag": str(bag),
        "variant": variant,
        "run": run_name,
        "node_cpu": args.node_cpu,
        "player_cpu": args.player_cpu,
        "duration": duration,
        "start_offset": args.playback_start_offset,
        "parameters": str(parameter_path),
        "d_min_boundary_lat": boundary_margin,
    }
    (run_directory / "run.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    metrics = completed_metrics(run_directory)
    if metrics is None:
        raise RuntimeError(f"run completed without a valid schema-v9 metrics CSV: {run_directory}")
    print(f"[{run_index}/{total_runs}] DONE  {run_name}", flush=True)
    return metrics


def main():
    """Prepare reduced bags, run warm-up, then alternate three measured replays."""
    args = arguments()
    if args.repetitions < 1:
        raise SystemExit("--repetitions must be at least one")
    if args.playback_start_offset < 0.0:
        raise SystemExit("--playback-start-offset must be non-negative")
    if args.playback_duration is not None and args.playback_duration <= 0.0:
        raise SystemExit("--playback-duration must be positive")
    if args.status_interval <= 0.0:
        raise SystemExit("--status-interval must be positive")
    if any(character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for character in args.run_tag):
        raise SystemExit("--run-tag may contain only letters, digits, '_' and '-'")
    if shutil.which("ros2") is None:
        raise SystemExit("ros2 is not available; source the workspace setup first")
    args.results.mkdir(parents=True, exist_ok=True)
    input_cache = args.input_cache if args.input_cache is not None else args.results / "inputs"

    prepared = []
    for source in args.bags:
        source = source.resolve()
        destination = input_cache / f"{source.name}_inputs"
        prepared.append(filtered_bag(source, destination))
    if args.prepare_only:
        return

    run_specs = []
    tag = f"_{args.run_tag}" if args.run_tag else ""
    for bag in prepared:
        if args.warmup_seconds > 0.0:
            for variant in ("circles", "obb_single", "obb_multistart"):
                run_specs.append((bag, variant, f"{bag.name}{tag}_warmup_{variant}", args.warmup_seconds, False))
        for repetition in range(args.repetitions):
            variants = ("circles", "obb_single", "obb_multistart")
            order = variants[repetition % len(variants) :] + variants[: repetition % len(variants)]
            for variant in order:
                run_specs.append((bag, variant, f"{bag.name}{tag}_r{repetition + 1}_{variant}", args.playback_duration, True))

    router = None
    measured = {bag: {variant: [] for variant in ("circles", "obb_single", "obb_multistart")} for bag in prepared}
    if args.start_zenoh_router:
        router = start(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"], os.environ.copy(), args.results / "zenoh_router.log")
        time.sleep(1.0)
    try:
        for index, (bag, variant, run_name, duration, is_measured) in enumerate(run_specs, start=1):
            metrics = run_once(args, bag, variant, run_name, index, len(run_specs), duration)
            if is_measured:
                measured[bag][variant].append(metrics)
    finally:
        if router is not None:
            stop(router)
            router.log_file.close()

    if args.analyze:
        analyzer = args.workspace / "src" / "target" / "utils" / "analyze_performance.py"
        reports = []
        for bag in prepared:
            for baseline in ("circles", "obb_single"):
                command = [
                    sys.executable,
                    str(analyzer),
                    *map(str, measured[bag]["obb_multistart"]),
                    "--compare",
                    *map(str, measured[bag][baseline]),
                    "--color",
                    "never",
                ]
                result = subprocess.run(command, check=True, text=True, capture_output=True)
                reports.append(
                    f"===== {bag.name}: obb_multistart vs {baseline} "
                    f"({args.repetitions} repetitions pooled) =====\n{result.stdout}"
                )

        for baseline in ("circles", "obb_single"):
            all_multistart = [path for bag in prepared for path in measured[bag]["obb_multistart"]]
            all_baseline = [path for bag in prepared for path in measured[bag][baseline]]
            command = [
                sys.executable,
                str(analyzer),
                *map(str, all_multistart),
                "--compare",
                *map(str, all_baseline),
                "--color",
                "never",
            ]
            result = subprocess.run(command, check=True, text=True, capture_output=True)
            reports.append(
                f"===== ALL BAGS: obb_multistart vs {baseline} " f"({args.repetitions} repetitions pooled) =====\n{result.stdout}"
            )
        report = "\n".join(reports)
        report_path = args.results / f"analysis{tag}.txt"
        report_path.write_text(report, encoding="utf-8")
        print(f"\n{report}", end="", flush=True)
        print(f"Analysis written to {report_path}", flush=True)


if __name__ == "__main__":
    main()

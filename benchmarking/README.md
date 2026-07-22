# Trajectory optimization benchmarking

This directory contains standalone tooling for extracting and comparing trajectory optimizer performance measurements. The tools are intentionally not installed as part of the ROS package.

## Recording a new run

Enable `performance_logging` in the optimizer configuration. The node creates a timestamped file such as
`trajectory_optimization_ackermann_node_20260715T142355_123Z.csv` and buffers up to 100 records before flushing.

By default, files are written to `/tmp/trajectory_optimization_benchmarks`. To place them in this directory, set the output directory before starting the node:

```bash
export TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR="$(pwd)/benchmarking"
```

The filename identifies the node and start time, so no run ID or output path is needed in the ROS parameter file. Rename the completed CSV if a descriptive name such as `acados-0.5.5-warmstart-0.csv` is more useful. Alternatively, fill the initially empty `run_id` column after the run; avoid editing measurement values.

For comparable runs, use the same optimizer configuration, rosbag playback rate, warm-up removal, and deadline. Keep `verbose` and `debug_visualization` disabled.

### Recorded values

The runtime CSV deliberately contains only values needed to compare solver behavior or explain a regression:

- Context: schema version, source, optional run ID, cycle, record timestamp, reference-point count, and object count.
- Outcome: ACADOS status and whether a trajectory was published.
- Runtime: complete planning-cycle wall time split into preprocessing, `acados_solve()`, and postprocessing, plus ACADOS' internal total, linearization, simulation, QP, QP-solver, condensing, regularization, globalization, preparation, and feedback times. The three top-level phases add up to the complete cycle; CSV writing happens afterwards and is excluded.
- Work and quality: SQP/QP iterations, QP status, cost, KKT norm, aggregate NLP residual, and stationarity, equality, inequality, and complementarity residuals.

Timers for input transformation, initial-guess construction, boundary preparation, individual parameter updates, solution reading, diagnostics, and message output are intentionally not recorded. They required instrumentation throughout the planning code but are not needed for the initial ACADOS version and option comparisons. They can be profiled separately if a later result points at non-solver overhead.

## Extracting a legacy rosout baseline

The extractor needs the ROS Python environment, including `rosbag2_py`, `rclpy`, and `rcl_interfaces`. These modules come from the ROS installation and are not available as ordinary PyPI dependencies.

```bash
python3 benchmarking/extract_rosout_performance.py \
  rosbag2_2026_05_06-18_57_54 \
  benchmarking/bag-2026-05-06_objects_legacy.csv \
  --node planning.trajectory_optimization --exact-node
```

The resulting CSV only contains values actually present in the old logs: status, publication outcome, ACADOS total time, iterations, KKT, cost, and NLP residual. Missing values remain empty rather than being interpreted as zero.

## Analyzing and comparing runs

Analyze one run:

```bash
python3 benchmarking/analyze_performance.py \
  benchmarking/bag-2026-05-06_objects_baseline.csv \
  --skip 10 --deadline-ms 100
```

Compare a candidate with a baseline:

```bash
python3 benchmarking/analyze_performance.py \
  benchmarking/bag-2026-05-06_objects_baseline.csv \
  --compare benchmarking/bag-2026-05-06_objects_legacy.csv \
  --skip 10 --deadline-ms 100
```

The report contains status and publication rates, deadline compliance, consecutive failure streaks, and timing and quality distributions. A comparison prints the deltas and a threshold-based `BETTER`, `WORSE`, or `MIXED / NO MATERIAL CHANGE` verdict.

Output is colored automatically when stdout is a terminal. Use `--color always` to preserve colors in a compatible log viewer, `--color never` to disable them, or set the conventional `NO_COLOR` environment variable.

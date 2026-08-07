# Utilities

## Performance benchmarking

The trajectory optimizer can write one CSV record per scheduled planning tick.
Enable `performance_logging` in the node parameters to record the outcome
(including standstill and skipped ticks), solver status and timing, residuals,
publication outcome, and diagnostics for rejected solutions.

Files are written to `/tmp/trajectory_optimization_benchmarks` by default. Set
an alternative output directory before starting the node if required:

```bash
export TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR=/path/to/results
```

## Recorded data

Each row describes one completed planning cycle:

- **Context:** timestamps, cycle number, reference points, and perceived objects.
- **Outcome:** whether the solver ran, a normalized outcome/skip reason, ACADOS
  status, and whether a trajectory was published.
- **Cycle timing:** total cycle time split into preprocessing, solver, and
  postprocessing. These phases add up to `cycle_ms`.
- **Solver timing:** ACADOS total, linearization, simulation, QP, condensing,
  regularization, globalization, preparation, and feedback times.
- **Solver work:** SQP/QP iterations and the final QP status.
- **Solution quality:** cost, KKT norm, and stationarity (`res_stat`), dynamics
  (`res_eq`), inequality (`res_ineq`), and complementarity (`res_comp`)
  residuals.
- **Rejected solutions:** largest constraint or dynamics violation including
  shooting stage, constraint type, index, and bound side.

Detailed constraint diagnostics are collected only for finite solver outputs
that are rejected. CSV writing happens after cycle timing and is therefore not
included in `cycle_ms`.

Analyze one run:

```bash
python3 utils/analyze_performance.py run.csv
```

Compare it with a baseline:

```bash
python3 utils/analyze_performance.py run.csv --compare baseline.csv
```

Multiple repetitions can be pooled by passing more than one CSV on each side:

```bash
python3 utils/analyze_performance.py obb_r*.csv --compare circles_r*.csv
```

Use `--skip` to discard warm-up cycles and `--deadline-ms` to configure the
planning-cycle deadline. Comparable runs should use the same inputs, optimizer
configuration, playback rate, and deadline.

## Circle versus OBB-SAT replay

`run_geometry_benchmark.py` creates input-only MCAPs under `/tmp`, leaves the
original bags untouched, and alternates three full Ackermann replays per
geometry. It uses the current repository defaults, enables instrumentation,
and sets `d_min_boundary_lat=0.0` only for the exact OBB geometry; the circle
baseline retains its configured negative compensation margin. Optimizer and
player are pinned to separate CPUs. Each replay displays a progress bar (or
periodic status lines when redirected), stores component logs, and records the
generated trajectories:

```bash
source /docker-ros/ws/install/setup.bash
python3 utils/run_geometry_benchmark.py bagfiles/rosbag2_* \
  --workspace /docker-ros/ws
```

Use `--input-cache` to reuse previously filtered bags and `--run-tag` to keep a
new experiment separate in an existing results directory. `--resume` skips
completed schema-v7 runs, making the command safe to restart. At completion,
per-bag and pooled three-repetition reports are written to `analysis_<tag>.txt`.
`--playback-start-offset` and `--playback-duration` are available for short,
targeted diagnostics before a complete replay.

Use `--prepare-only` to perform the expensive 243 GB input filtering without
starting the benchmark. Override `--node-cpu` and `--player-cpu` if CPUs 2 and
3 are not available.

In both geometry modes the runtime metrics use the physical ego OBB and exact
object OBBs for every input hypothesis, including hypotheses beyond the
30-slot OBB OCP capacity. The validator reports object collisions and
route-boundary violations at shooting nodes as hard failures, together with
the maximum boundary penetration. Ten uniformly spaced intersamples per
interval are reported separately and do not affect the spike gate.

The comparison command applies the agreed performance gates (no more than one
percentage point publication-rate loss, solver p95 below 100 ms, and at least
20 percent p95 improvement) together with the exact node-level geometry gate.

## Synthetic OBB ghost probe

`investigate_obb_ghosts.py` runs the generated Karl OBB solver on a straight,
hard-bounded synthetic corridor. It compares empty OBB slots with increasing
amounts of unreachable parked-vehicle clutter and stresses both with one real
blocking obstacle. The output includes solver status, SQP iterations, timing,
trajectory changes, and exact/smoothed SAT margins:

```bash
source /docker-ros/ws/install/setup.bash
python3 utils/investigate_obb_ghosts.py --repeats 5 --speed 5.0
```

Run the Release build first so the probe loads the current generated
`karl_obb_sat` solver. This utility is a formulation diagnostic rather than a
bag benchmark; all scenarios are deterministic and independent of recorded
data. Each OBB parameter row ends in an activation value. Real hypotheses use
`active=1`; unused slots use `active=0`, which replaces their SAT constraint by
the constant feasible margin `1 m`. Consequently unused slots have zero state
Jacobian regardless of their placeholder pose.

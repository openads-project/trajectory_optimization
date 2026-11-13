# Utilities

## Performance benchmarking

The trajectory optimizer can write one CSV record per planning cycle. Enable
`performance_logging` in the node parameters to record solver status, timing,
residuals, publication outcome, and diagnostics for rejected solutions.

Files are written to `/tmp/trajectory_optimization_benchmarks` by default. Set
an alternative output directory before starting the node if required:

```bash
export TRAJECTORY_OPTIMIZATION_BENCHMARK_DIR=/path/to/results
```

## Recorded data

Each schema-8 row describes one planning-cycle invocation, including skipped
cycles, standstill commands, valid solutions and rejected-solve empty updates:

- **Context:** timestamps, cycle number, reference points, and perceived objects.
- **Outcome:** `outcome`, `solver_called`, `message_published` and
  `trajectory_states` distinguish solver work, usable solutions, empty updates
  and no publication. `published` retains its usable-trajectory meaning; solver
  status is -1 when no solve ran.
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
- **Desired clearance:** largest lower object-clearance slack in meters, with stage
  and object-slot index, including accepted solutions (introduced in CSV schema 7). Boundary slacks
  are excluded. Object constraint diagnostics distinguish `object_collision` from
  `object_clearance`; positive clearance slacks do not relax physical separation.

Detailed constraint diagnostics are collected only for finite solver outputs
that are rejected. CSV writing happens after cycle timing and is therefore not
included in `cycle_ms`.

All lower/upper slacks are checked for finiteness before accepting solver output.
Clearance statistics are collected for usable solver statuses when logging or verbose
output is enabled; stage/index are -1 when no positive object-clearance slack is found.

Analyze a run:

```bash
python3 utils/analyze_performance.py run.csv
```

Compare it with a baseline:

```bash
python3 utils/analyze_performance.py run.csv --compare baseline.csv
```

Use `--skip` to discard warm-up cycles and `--deadline-ms` to configure the
planning-cycle deadline. Comparable runs should use the same inputs, optimizer
configuration, playback rate, and deadline.

## Clock and input-age timing

The timing investigation uses schema 8 and preserves the schema-7 columns.
`cycle_start_ros_ns`, `ego_callback_ros_ns`, `solve_start_ros_ns` and
`publish_ros_ns` all use the node's ROS clock. Their corresponding
`ego_age_at_*_ms` columns subtract the consumed EgoData header stamp. Publication
age is sampled just before calling the middleware publisher, for both valid
solutions and empty updates; it is not downstream receipt age. Compare it with
mux receipt age to investigate the return path, accounting for clock delivery
and matching the consumed EgoData stamp. Repeated stamps can occur, so they
are not unique cycle identifiers.

`ego_callback_to_cycle_wall_ms` uses a monotonic clock to measure how long the
last processed EgoData callback preceded cycle start. It does not measure how
long the message waited in the middleware or executor before that callback.

Inputs now update independently of solver execution. The callback timestamp in
each row belongs to the EgoData selected in that cycle's snapshot; later callbacks
do not change it. Snapshot copying is included in preprocessing time. The
snapshot selects the latest processed message from each topic without timestamp
synchronization.

`cycle_interval_wall_ms` measures successive cycle starts, making timer phase
and overruns observable locally. `cycle_start_wall_ns` is a system-clock epoch
for correlating logs across hosts; it requires verified host clock alignment.
Never subtract a system-clock epoch from a ROS simulation timestamp.

Unavailable observation points are `nan`, not zero; negative ROS ages across
clock jumps remain negative. No-solver cycles count their time as preprocessing.
Solver statistics and status rates exclude skipped cycles in the analyzer.
Timing collection does not change timers, callback groups, solver settings,
initialization, acceptance criteria or message stamps. Detailed solver-statistic
collection has overhead; CSV writing is excluded from `cycle_ms`. Logs are
flushed every 100 rows and at normal shutdown.

For the simulation comparison, retrieve each pod's CSV from
`/tmp/trajectory_optimization_benchmarks` before deleting the pod, and identify
onboard/offboard explicitly in the copied file names. The module default for
`performance_logging` stays false; enable it before node startup. The logger is
created at startup, so setting the parameter later does not start CSV logging.

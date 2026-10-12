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

Each row describes one completed planning cycle:

- **Context:** timestamps, cycle number, reference points, and perceived objects.
- **Outcome:** ACADOS status and whether a trajectory was published.
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
  and object-slot index, including accepted solutions (CSV schema 7). Boundary slacks
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

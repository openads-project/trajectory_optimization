#!/usr/bin/env python3

# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

"""Summarize one optimizer CSV and optionally compare it with a baseline run."""

import argparse
import csv
import math
import os
import statistics
import sys
from collections import Counter
from pathlib import Path

TIMING_METRICS = [
    "acados_total_ms",
    "solve_wall_ms",
    "cycle_ms",
    "preprocessing_ms",
    "parameter_update_ms",
    "postprocessing_ms",
    "acados_qp_ms",
    "acados_qp_solver_ms",
    "acados_lin_ms",
    "acados_preparation_ms",
    "acados_feedback_ms",
]
QUALITY_METRICS = ["sqp_iter", "qp_iter", "kkt", "nlp_res", "cost"]
SOLVER_ONLY_METRICS = set(TIMING_METRICS + QUALITY_METRICS) - {
    "cycle_ms",
    "preprocessing_ms",
    "parameter_update_ms",
    "postprocessing_ms",
}


class Color:
    """ANSI colors used for terminal summaries."""

    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    CYAN = "\033[36m"
    DIM = "\033[2m"


use_color = False


def paint(text, color):
    """Apply an ANSI color when colored output is enabled."""
    return f"{color}{text}{Color.RESET}" if use_color else text


def higher_rate_color(value):
    """Color rates where values close to 100% are desirable."""
    if value is None or value < 0.95:
        return Color.RED
    return Color.GREEN if value >= 0.99 else Color.YELLOW


def lower_rate_color(value):
    """Color rates where values close to 0% are desirable."""
    if value is None or value > 0.01:
        return Color.RED
    return Color.GREEN if value == 0.0 else Color.YELLOW


def number(value):
    """Convert a CSV value to a finite float or return None."""
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def percentile(values, fraction):
    """Return a linearly interpolated percentile."""
    if not values:
        return None
    values = sorted(values)
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return values[lower]
    return values[lower] + (values[upper] - values[lower]) * (position - lower)


def values_for(records, key):
    """Collect finite numeric values for one field."""
    values = []
    for record in records:
        if key in SOLVER_ONLY_METRICS and record.get("solver_ran", "1") != "1":
            continue
        value = number(record.get(key))
        if key == "nlp_res":
            residuals = [number(record.get(field)) for field in ("res_stat", "res_eq", "res_ineq", "res_comp")]
            finite_residuals = [residual for residual in residuals if residual is not None]
            if finite_residuals:
                value = max(finite_residuals)
        if value is not None:
            values.append(value)
    return values


def longest_streak(statuses, predicate):
    """Return the maximum number of consecutive statuses matching predicate."""
    longest = 0
    current = 0
    for status in statuses:
        current = current + 1 if predicate(status) else 0
        longest = max(longest, current)
    return longest


def load_csv(paths, skip):
    """Load and pool canonical performance records, discarding warm-up rows per file."""
    records = []
    for path in paths:
        with path.open(encoding="utf-8", newline="") as source:
            file_records = list(csv.DictReader(source))[skip:]
        for record in file_records:
            record["_input_file"] = str(path)
        records.extend(file_records)
    if not records:
        raise SystemExit(f"No records remain after skipping {skip} rows per file.")
    if "status" not in records[0]:
        raise SystemExit(f"{paths[0]} is not a performance CSV (missing status column).")
    return records


def summarize(records, deadline_ms):
    """Calculate the stability, deadline, and distribution scorecard."""
    solver_records = [record for record in records if record.get("solver_ran", "1") == "1"]
    statuses = [int(value) for record in solver_records if (value := number(record.get("status"))) is not None]
    counts = Counter(statuses)
    known = len(statuses)
    published_values = values_for(records, "published")
    hard_failure = lambda status: status not in (0, 2, 7)  # noqa: E731
    record_groups = []
    for input_file in dict.fromkeys(record["_input_file"] for record in records):
        record_groups.append([record for record in records if record["_input_file"] == input_file])

    timing_values = values_for(records, "acados_total_ms")
    cycle_values = values_for(records, "cycle_ms")

    rejected_diagnostics = Counter()
    rejected_constraint_indices = Counter()
    for record in solver_records:
        if number(record.get("published")) != 0:
            continue
        constraint_type = record.get("max_ineq_type", "")
        if constraint_type and constraint_type != "none":
            rejected_diagnostics[constraint_type] += 1
            rejected_constraint_indices[(constraint_type, record.get("max_ineq_index", ""))] += 1
        elif number(record.get("max_eq_stage")) is not None and number(record.get("max_eq_stage")) >= 0:
            rejected_diagnostics["dynamics"] += 1
        else:
            rejected_diagnostics["unavailable"] += 1

    return {
        "records": len(records),
        "solver_records": len(solver_records),
        "outcome_counts": Counter(record.get("outcome", "legacy_solver_record") for record in records),
        "status_counts": counts,
        "status_rates": {status: count / known for status, count in counts.items()} if known else {},
        "success_rate": counts[0] / known if known else None,
        "timeout_rate": counts[7] / known if known else None,
        "hard_failure_rate": sum(count for status, count in counts.items() if hard_failure(status)) / known if known else None,
        "published_rate": statistics.mean(published_values) if published_values else None,
        "deadline_rate": (sum(value <= deadline_ms for value in cycle_values) / len(cycle_values) if cycle_values else None),
        "timing_p50": percentile(timing_values, 0.50),
        "timing_p95": percentile(timing_values, 0.95),
        "timing_p99": percentile(timing_values, 0.99),
        "max_unpublished_streak": max(
            longest_streak(values_for(group, "published"), lambda published: published == 0) for group in record_groups
        ),
        "max_timeout_streak": max(
            longest_streak(
                [
                    int(value)
                    for record in group
                    if record.get("solver_ran", "1") == "1" and (value := number(record.get("status"))) is not None
                ],
                lambda status: status == 7,
            )
            for group in record_groups
        ),
        "max_status4_streak": max(
            longest_streak(
                [
                    int(value)
                    for record in group
                    if record.get("solver_ran", "1") == "1" and (value := number(record.get("status"))) is not None
                ],
                lambda status: status == 4,
            )
            for group in record_groups
        ),
        "max_hard_failure_streak": max(
            longest_streak(
                [
                    int(value)
                    for record in group
                    if record.get("solver_ran", "1") == "1" and (value := number(record.get("status"))) is not None
                ],
                hard_failure,
            )
            for group in record_groups
        ),
        "rejected_diagnostics": rejected_diagnostics,
        "rejected_constraint_indices": rejected_constraint_indices,
        "max_obstacle_hypotheses": max(values_for(records, "obstacle_hypotheses"), default=0),
        "max_dropped_obstacle_hypotheses": max(values_for(records, "dropped_obstacle_hypotheses"), default=0),
        "node_object_collisions": sum(values_for(records, "node_object_collisions")),
        "node_boundary_violations": sum(values_for(records, "node_boundary_violations")),
        "dropped_hypothesis_collisions": sum(values_for(records, "dropped_hypothesis_collisions")),
        "intersample_object_collisions": sum(values_for(records, "intersample_object_collisions")),
        "intersample_boundary_violations": sum(values_for(records, "intersample_boundary_violations")),
        "unvalidated_optimized_publications": sum(
            record.get("outcome") == "published_optimized" and number(record.get("geometry_validated")) != 1 for record in records
        ),
        "max_node_boundary_penetration_m": max(values_for(records, "max_node_boundary_penetration_m"), default=0.0),
        "max_intersample_boundary_penetration_m": max(values_for(records, "max_intersample_boundary_penetration_m"), default=0.0),
    }


def format_rate(value):
    """Format an optional fraction as percentage."""
    return "n/a" if value is None else f"{100.0 * value:.2f}%"


def print_run(label, records, summary, deadline_ms):
    """Print the scorecard and useful metric distributions for one run."""
    run_ids = sorted({record.get("run_id", "") for record in records if record.get("run_id")})
    suffix = f"  run_id={','.join(run_ids)}" if run_ids else ""
    print(paint(f"\nRUN {label}{suffix}", Color.BOLD + Color.CYAN))
    status_parts = []
    for status, count in sorted(summary["status_counts"].items()):
        color = Color.GREEN if status == 0 else Color.YELLOW if status in (2, 7) else Color.RED
        status_parts.append(paint(f"{status}: {count}", color))
    print(
        f"scheduled_ticks={summary['records']} solver_runs={summary['solver_records']} "
        f"status_counts={{{', '.join(status_parts)}}}"
    )
    print("outcomes={" + ", ".join(f"{key}: {count}" for key, count in summary["outcome_counts"].most_common()) + "}")
    status_rate_parts = []
    for status, rate in sorted(summary["status_rates"].items()):
        color = Color.GREEN if status == 0 else Color.YELLOW if status in (2, 7) else Color.RED
        status_rate_parts.append(paint(f"{status}: {format_rate(rate)}", color))
    print(f"status_rates={{{', '.join(status_rate_parts)}}}")
    print(
        f"success={paint(format_rate(summary['success_rate']), higher_rate_color(summary['success_rate']))}  "
        f"timeout={paint(format_rate(summary['timeout_rate']), lower_rate_color(summary['timeout_rate']))}  "
        f"hard_failure={paint(format_rate(summary['hard_failure_rate']), lower_rate_color(summary['hard_failure_rate']))}  "
        f"published={paint(format_rate(summary['published_rate']), higher_rate_color(summary['published_rate']))}  "
        f"cycle_ms_within_{deadline_ms:g}ms="
        f"{paint(format_rate(summary['deadline_rate']), higher_rate_color(summary['deadline_rate']))}"
    )
    print(
        f"obstacle_hypotheses_max={summary['max_obstacle_hypotheses']:.0f}  "
        f"dropped_hypotheses_max={summary['max_dropped_obstacle_hypotheses']:.0f}"
    )
    print(
        f"geometry: node_object_collisions={summary['node_object_collisions']:.0f}  "
        f"node_boundary_violations={summary['node_boundary_violations']:.0f}  "
        f"dropped_hypothesis_collisions={summary['dropped_hypothesis_collisions']:.0f}  "
        f"intersample_object_collisions={summary['intersample_object_collisions']:.0f}  "
        f"intersample_boundary_violations={summary['intersample_boundary_violations']:.0f}  "
        f"max_node_boundary_penetration_m={summary['max_node_boundary_penetration_m']:.4g}  "
        f"max_intersample_boundary_penetration_m={summary['max_intersample_boundary_penetration_m']:.4g}"
    )
    if summary["unvalidated_optimized_publications"]:
        print(f"unvalidated_optimized_publications={summary['unvalidated_optimized_publications']}")
    print(
        f"max_streaks: unpublished="
        f"{paint(str(summary['max_unpublished_streak']), Color.GREEN if summary['max_unpublished_streak'] == 0 else Color.RED)} "
        f"timeout="
        f"{paint(str(summary['max_timeout_streak']), Color.GREEN if summary['max_timeout_streak'] == 0 else Color.YELLOW)} "
        f"status4={paint(str(summary['max_status4_streak']), Color.GREEN if summary['max_status4_streak'] == 0 else Color.RED)} "
        f"hard_failure="
        f"{paint(str(summary['max_hard_failure_streak']), Color.GREEN if summary['max_hard_failure_streak'] == 0 else Color.RED)}"
    )
    if summary["rejected_diagnostics"]:
        diagnostics = ", ".join(f"{key}: {count}" for key, count in summary["rejected_diagnostics"].most_common())
        indices = ", ".join(
            f"{constraint_type}[{index}]: {count}"
            for (constraint_type, index), count in summary["rejected_constraint_indices"].most_common()
        )
        print(f"rejected_constraint_types={{{diagnostics}}}")
        if indices:
            print(f"rejected_constraint_indices={{{indices}}}")
    print(paint(f"{'metric':24} {'count':>7} {'mean':>11} {'p50':>11} {'p95':>11} {'p99':>11} {'max':>11}", Color.BOLD))
    for key in TIMING_METRICS + QUALITY_METRICS:
        values = values_for(records, key)
        if not values:
            continue
        print(
            f"{key:24} {len(values):7d} {statistics.mean(values):11.4g} "
            f"{percentile(values, 0.50):11.4g} {percentile(values, 0.95):11.4g} "
            f"{percentile(values, 0.99):11.4g} {max(values):11.4g}"
        )


def relative_delta(candidate, baseline):
    """Return relative candidate change, or None where it is undefined."""
    if candidate is None or baseline in (None, 0):
        return None
    return (candidate - baseline) / baseline


def compare(candidate, baseline, deadline_ms):
    """Evaluate the OBB spike's explicit publication and solver-p95 gates."""
    failed_gates = []
    if candidate["hard_failure_rate"] is not None and baseline["hard_failure_rate"] is not None:
        if candidate["hard_failure_rate"] - baseline["hard_failure_rate"] > 0.01:
            failed_gates.append("hard failures increased by >1 percentage point")
    if candidate["node_object_collisions"] > 0:
        failed_gates.append("exact validator found object collisions at shooting nodes")
    if candidate["node_boundary_violations"] > 0:
        failed_gates.append("exact validator found boundary violations at shooting nodes")
    if candidate["unvalidated_optimized_publications"] > 0:
        failed_gates.append("optimized publications escaped exact geometry validation")
    if candidate["published_rate"] is not None and baseline["published_rate"] is not None:
        if baseline["published_rate"] - candidate["published_rate"] > 0.01:
            failed_gates.append("published rate decreased by >1 percentage point")
    p95_delta = relative_delta(candidate["timing_p95"], baseline["timing_p95"])
    if candidate["timing_p95"] is None or candidate["timing_p95"] >= deadline_ms:
        failed_gates.append(f"solver p95 is not below {deadline_ms:g} ms")
    if p95_delta is None or p95_delta > -0.20:
        failed_gates.append("solver p95 did not improve by at least 20%")

    if failed_gates:
        return "FAIL", failed_gates
    return "PASS", ["publication robustness, solver-p95, and recorded node-geometry gates passed"]


def print_comparison(candidate, baseline, deadline_ms):
    """Print deltas and the threshold-based overall verdict."""
    print(paint("\nCOMPARISON (candidate relative to baseline)", Color.BOLD + Color.CYAN))
    candidate_deadline_rate = candidate["deadline_rate"]
    baseline_deadline_rate = baseline["deadline_rate"]
    rows = [
        ("success rate", candidate["success_rate"], baseline["success_rate"], "rate", True),
        ("timeout rate", candidate["timeout_rate"], baseline["timeout_rate"], "rate", False),
        ("hard failure rate", candidate["hard_failure_rate"], baseline["hard_failure_rate"], "rate", False),
        ("published rate", candidate["published_rate"], baseline["published_rate"], "rate", True),
        ("cycle deadline rate", candidate_deadline_rate, baseline_deadline_rate, "rate", True),
        ("solver p50 [ms]", candidate["timing_p50"], baseline["timing_p50"], "number", False),
        ("solver p95 [ms]", candidate["timing_p95"], baseline["timing_p95"], "number", False),
        ("solver p99 [ms]", candidate["timing_p99"], baseline["timing_p99"], "number", False),
    ]
    print(paint(f"{'metric':24} {'candidate':>12} {'baseline':>12} {'delta':>12}", Color.BOLD))
    for label, candidate_value, baseline_value, value_type, higher_is_better in rows:
        if candidate_value is None or baseline_value is None:
            print(f"{label:24} {'n/a':>12} {'n/a':>12} {'n/a':>12}")
            continue
        if value_type == "rate":
            delta = 100.0 * (candidate_value - baseline_value)
            delta_text = f"{delta:+11.2f}pp"
        else:
            delta = relative_delta(candidate_value, baseline_value)
            delta_text = "n/a" if delta is None else f"{100.0 * delta:+.2f}%"
        raw_delta = candidate_value - baseline_value
        delta_color = Color.DIM if raw_delta == 0 else Color.GREEN if (raw_delta > 0) == higher_is_better else Color.RED
        candidate_text = format_rate(candidate_value) if value_type == "rate" else f"{candidate_value:.4g}"
        baseline_text = format_rate(baseline_value) if value_type == "rate" else f"{baseline_value:.4g}"
        print(f"{label:24} {candidate_text:>12} {baseline_text:>12} {paint(f'{delta_text:>12}', delta_color)}")

    verdict, reasons = compare(candidate, baseline, deadline_ms)
    verdict_color = Color.GREEN if verdict == "PASS" else Color.RED
    print(f"{paint('VERDICT:', Color.BOLD)} {paint(verdict, Color.BOLD + verdict_color)} ({'; '.join(reasons)})")


def main():
    """Parse arguments and report one run plus an optional baseline comparison."""
    global use_color
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, nargs="+", help="candidate performance CSV file(s), pooled when multiple are given")
    parser.add_argument(
        "--compare", type=Path, nargs="+", metavar="BASELINE", help="baseline CSV file(s), pooled when multiple are given"
    )
    parser.add_argument("--skip", type=int, default=0, help="discard this many warm-up rows from both runs")
    parser.add_argument("--deadline-ms", type=float, default=100.0, help="planning-cycle deadline")
    parser.add_argument(
        "--color",
        choices=("auto", "always", "never"),
        default="auto",
        help="colorize output (default: auto when stdout is a terminal)",
    )
    args = parser.parse_args()
    use_color = args.color == "always" or (args.color == "auto" and sys.stdout.isatty() and "NO_COLOR" not in os.environ)

    records = load_csv(args.run, args.skip)
    summary = summarize(records, args.deadline_ms)
    print_run(", ".join(map(str, args.run)), records, summary, args.deadline_ms)

    if args.compare:
        baseline_records = load_csv(args.compare, args.skip)
        baseline_summary = summarize(baseline_records, args.deadline_ms)
        print_run(", ".join(map(str, args.compare)), baseline_records, baseline_summary, args.deadline_ms)
        print_comparison(summary, baseline_summary, args.deadline_ms)


if __name__ == "__main__":
    main()

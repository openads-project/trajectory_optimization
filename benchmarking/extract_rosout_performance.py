#!/usr/bin/env python3

"""Extract legacy trajectory optimizer measurements from a rosbag /rosout topic."""

import argparse
import csv
import re
from pathlib import Path

import rosbag2_py
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message

CSV_FIELDS = [
    "schema_version",
    "source",
    "run_id",
    "cycle",
    "record_stamp_ns",
    "status",
    "published",
    "ref_points",
    "objects",
    "sqp_iter",
    "qp_iter",
    "qp_status",
    "cycle_ms",
    "preprocessing_ms",
    "solve_wall_ms",
    "postprocessing_ms",
    "acados_total_ms",
    "acados_lin_ms",
    "acados_sim_ms",
    "acados_qp_ms",
    "acados_qp_solver_ms",
    "acados_qp_xcond_ms",
    "acados_reg_ms",
    "acados_glob_ms",
    "acados_preparation_ms",
    "acados_feedback_ms",
    "cost",
    "kkt",
    "nlp_res",
    "res_stat",
    "res_eq",
    "res_ineq",
    "res_comp",
]

ANSI_ESCAPE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
STATUS = re.compile(r"Optimization failed with status\s+(\d+)")
SOLVER_STATUS = re.compile(r"acados_solve\(\) failed with status\s+(\d+)")
DURATION = re.compile(
    r"Optimization took\s+([+\-\d.eE]+)\s+ms\.?\s+" r"\(SQP iter:\s*(\d+)(?:;\s*QP iter:\s*(\d+))?;\s*KKT:\s*([+\-\d.eE]+)\)"
)
COST = re.compile(r"cost_value:\s*([+\-\d.eE]+);\s*nlp_res:\s*([+\-\d.eE]+)")
RESIDUALS = re.compile(
    r"cost_value:\s*([+\-\d.eE]+);\s*residuals:\s*"
    r"stat=([+\-\d.eE]+)\s+eq=([+\-\d.eE]+)\s+"
    r"ineq=([+\-\d.eE]+)\s+comp=([+\-\d.eE]+)"
)


def stamp_to_nanoseconds(message: Log, bag_timestamp: int) -> int:
    """Prefer the ROS log timestamp and fall back to the bag record timestamp."""
    stamp = message.stamp
    timestamp = stamp.sec * 1_000_000_000 + stamp.nanosec
    return timestamp or bag_timestamp


def main():
    """Read /rosout, reconstruct solve records, and write the canonical CSV."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag directory containing /rosout")
    parser.add_argument("output", type=Path, help="destination CSV")
    parser.add_argument(
        "--node",
        default="trajectory_optimization",
        help="only use rosout records whose node name contains this value",
    )
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    records = []
    pending = None
    cycle = 0

    def new_record(timestamp):
        nonlocal cycle
        cycle += 1
        return {
            "schema_version": 1,
            "source": "rosout",
            "cycle": cycle,
            "record_stamp_ns": timestamp,
            "published": 0,
        }

    def finish_pending():
        nonlocal pending
        if pending is not None:
            records.append(pending)
            pending = None

    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        if topic != "/rosout":
            continue
        message = deserialize_message(serialized, Log)
        if args.node and args.node not in message.name:
            continue
        text = ANSI_ESCAPE.sub("", message.msg)
        timestamp = stamp_to_nanoseconds(message, bag_timestamp)

        status_match = STATUS.search(text) or SOLVER_STATUS.search(text)
        if "Optimization: SUCCESS!" in text or status_match:
            finish_pending()
            pending = new_record(timestamp)
            pending["status"] = int(status_match.group(1)) if status_match else 0
            continue

        duration_match = DURATION.search(text)
        if duration_match:
            if pending is None:
                pending = new_record(timestamp)
                pending["status"] = 0
            pending["record_stamp_ns"] = timestamp
            pending["acados_total_ms"] = float(duration_match.group(1))
            pending["sqp_iter"] = int(duration_match.group(2))
            if duration_match.group(3) is not None:
                pending["qp_iter"] = int(duration_match.group(3))
            pending["kkt"] = float(duration_match.group(4))
            continue

        cost_match = COST.search(text)
        if cost_match and pending is not None:
            pending["cost"] = float(cost_match.group(1))
            pending["nlp_res"] = float(cost_match.group(2))
            continue

        residual_match = RESIDUALS.search(text)
        if residual_match and pending is not None:
            pending["cost"] = float(residual_match.group(1))
            pending["res_stat"] = float(residual_match.group(2))
            pending["res_eq"] = float(residual_match.group(3))
            pending["res_ineq"] = float(residual_match.group(4))
            pending["res_comp"] = float(residual_match.group(5))
            continue

        if "Published trajectory" in text and pending is not None:
            pending["published"] = 1
            finish_pending()

    finish_pending()
    if not records:
        raise SystemExit("No optimizer performance records found on /rosout.")

    with args.output.open("w", encoding="utf-8", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)

    print(f"Wrote {len(records)} records to {args.output}")


if __name__ == "__main__":
    main()

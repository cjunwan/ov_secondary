#!/usr/bin/env python3
"""Summarize one frame-reset failure-recovery experiment from its CSV logs."""

import argparse
import csv
import glob
import math
import os
import sys


def read_rows(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def finite_float(value, default=math.nan):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def newest_nonempty(pattern):
    candidates = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    for path in candidates:
        rows = read_rows(path)
        if rows:
            return path, rows
    return None, []


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", required=True, help="experiment_tag from the launch")
    parser.add_argument("--robot", default="omo2")
    parser.add_argument("--workspace", default="/home/junwan/microswarm_ws")
    args = parser.parse_args()

    frame_path = os.path.join(
        args.workspace, "logs", "frame_reset", args.tag,
        "{}_frame_reset.csv".format(args.robot),
    )
    relative_pattern = os.path.join(
        args.workspace, "logs", "relative_recovery", args.tag,
        "{}_relative_recovery_*.csv".format(args.robot),
    )

    if not os.path.isfile(frame_path):
        print("ERROR: frame reset log not found: {}".format(frame_path), file=sys.stderr)
        return 2
    frame_rows = read_rows(frame_path)
    reset_rows = [row for row in frame_rows if row.get("event") == "RESET_ACCEPTED"]
    if not reset_rows:
        print("ERROR: RESET_ACCEPTED not found in {}".format(frame_path), file=sys.stderr)
        return 2

    relative_path, relative_rows = newest_nonempty(relative_pattern)
    if relative_path is None:
        print("ERROR: recovery audit not found: {}".format(relative_pattern), file=sys.stderr)
        return 2

    reset_time = finite_float(reset_rows[-1].get("ros_time"))
    starts = [row for row in relative_rows if row.get("event") == "RECOVERY_STARTED"]
    start_time = finite_float(starts[-1].get("ros_time")) if starts else reset_time
    terminal_events = [
        row for row in relative_rows
        if row.get("event") in ("RECOVERY_COMPLETE", "RECOVERY_CANCELLED")
        and finite_float(row.get("ros_time")) >= start_time
    ]
    terminal = terminal_events[0] if terminal_events else None

    accepted_ranges = [
        row for row in relative_rows
        if row.get("event") == "RANGE_OPTIMIZATION_ACCEPTED"
        and finite_float(row.get("ros_time")) >= start_time
    ]
    rejections = [
        row for row in relative_rows
        if row.get("event") in (
            "RANGE_OPTIMIZATION_REJECTED", "RANGE_GEOMETRY_REJECTED",
            "RANGE_CONTINUITY_REJECTED", "RANGE_WAITING")
        and finite_float(row.get("ros_time")) >= start_time
    ]

    print("experiment_tag: {}".format(args.tag))
    print("robot: {}".format(args.robot))
    print("reset_ros_time: {:.6f}".format(reset_time))
    print("frame_reset_mode: {}".format(reset_rows[-1].get("rotation_mode", "unknown")))
    print("recovery_audit: {}".format(relative_path))

    if terminal is None:
        print("status: NOT_RECOVERED")
        if rejections:
            last = rejections[-1]
            print("last_range_state: {} ({})".format(
                last.get("event", ""), last.get("detail", "")))
        return 1

    end_time = finite_float(terminal.get("ros_time"))
    print("status: RECOVERED")
    print("recovery_event: {}".format(terminal.get("event", "")))
    print("recovery_source: {}".format(terminal.get("mode", "")))
    print("recovery_delay_sec: {:.3f}".format(end_time - start_time))
    print("range_used_in_final_recovery: {}".format(
        "yes" if accepted_ranges else "no"))

    if accepted_ranges:
        accepted = accepted_ranges[-1]
        print("range_mode: {}".format(accepted.get("mode", "")))
        print("range_factors: {}".format(accepted.get("factor_count", "0")))
        print("range_neighbors: {}".format(accepted.get("neighbor_count", "0")))
        print("range_temporal_span_sec: {}".format(accepted.get("temporal_span", "0")))
        print("range_normalized_rms: {}".format(accepted.get("normalized_rms", "0")))
        print("range_solution: {}".format(accepted.get("detail", "")))
    elif rejections:
        last = rejections[-1]
        print("last_range_state: {} ({})".format(
            last.get("event", ""), last.get("detail", "")))
    return 0


if __name__ == "__main__":
    sys.exit(main())

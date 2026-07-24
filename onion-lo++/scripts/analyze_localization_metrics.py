#!/usr/bin/env python3
"""Summarize Onion-LO++ per-callback metrics and compare two trajectories."""

import argparse
import csv
import json
import math
import statistics
from pathlib import Path


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite(values):
    return [value for value in values if math.isfinite(value)]


def distribution(values):
    values = finite(values)
    if not values:
        return {"count": 0}
    return {
        "count": len(values),
        "min": min(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def load_metrics(path):
    rows = []
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "reset_id", "scan_index", "stamp_sec", "input_delta_ms",
            "processing_ms", "min_registration_inliers",
            "x", "y", "z", "qx", "qy", "qz", "qw",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "metrics CSV is missing columns: " + ", ".join(sorted(missing)))
        for raw in reader:
            row = {
                key: float(value)
                for key, value in raw.items()
                if key not in {"reset_id", "scan_index"}
            }
            row["reset_id"] = int(raw["reset_id"])
            row["scan_index"] = int(raw["scan_index"])
            rows.append(row)
    if not rows:
        raise ValueError("metrics CSV contains no data rows")
    return rows


def quaternion_error_deg(left, right):
    dot = sum(left[key] * right[key] for key in ("qx", "qy", "qz", "qw"))
    left_norm = math.sqrt(
        sum(left[key] * left[key] for key in ("qx", "qy", "qz", "qw")))
    right_norm = math.sqrt(
        sum(right[key] * right[key] for key in ("qx", "qy", "qz", "qw")))
    if left_norm <= 0.0 or right_norm <= 0.0:
        return math.nan
    cosine = min(1.0, max(-1.0, abs(dot) / (left_norm * right_norm)))
    return math.degrees(2.0 * math.acos(cosine))


def translation_distance(left, right):
    return math.sqrt(sum(
        (left[key] - right[key]) ** 2 for key in ("x", "y", "z")))


def summarize(rows, expected_hz):
    groups = {}
    for row in rows:
        groups.setdefault(row["reset_id"], []).append(row)

    result = {}
    expected_period_ms = 1000.0 / expected_hz if expected_hz > 0.0 else None
    for reset_id, group in sorted(groups.items()):
        group.sort(key=lambda item: item["stamp_sec"])
        gaps = [row["input_delta_ms"] for row in group[1:]]
        processing = [row["processing_ms"] for row in group]
        inliers = [
            row["min_registration_inliers"]
            for row in group
            if row["min_registration_inliers"] > 0.0
        ]
        path_length = sum(
            translation_distance(previous, current)
            for previous, current in zip(group, group[1:])
        )
        summary = {
            "callbacks": len(group),
            "first_scan_index": group[0]["scan_index"],
            "last_scan_index": group[-1]["scan_index"],
            "sensor_duration_sec":
                group[-1]["stamp_sec"] - group[0]["stamp_sec"],
            "wall_duration_sec":
                group[-1].get("wall_elapsed_sec", 0.0) -
                group[0].get("wall_elapsed_sec", 0.0),
            "input_delta_ms": distribution(gaps),
            "timestamp_gaps_over_150ms":
                sum(value > 150.0 for value in gaps),
            "processing_ms": distribution(processing),
            "registration_inliers": distribution(inliers),
            "path_length_m": path_length,
            "start_to_end_translation_m":
                translation_distance(group[0], group[-1]),
            "start_to_end_rotation_deg":
                quaternion_error_deg(group[0], group[-1]),
            "start_pose": {
                key: group[0][key]
                for key in ("x", "y", "z", "qx", "qy", "qz", "qw")
            },
            "end_pose": {
                key: group[-1][key]
                for key in ("x", "y", "z", "qx", "qy", "qz", "qw")
            },
            "map_points": int(group[-1].get("map_points", 0)),
            "map_voxels": int(group[-1].get("map_voxels", 0)),
        }
        if expected_period_ms is not None:
            summary["expected_period_ms"] = expected_period_ms
            summary["callbacks_over_expected_period"] = sum(
                value > expected_period_ms for value in processing)
        result[str(reset_id)] = summary
    return result


def compare(reference, estimate, max_time_difference):
    reference = sorted(reference, key=lambda item: item["stamp_sec"])
    estimate = sorted(estimate, key=lambda item: item["stamp_sec"])
    translation_errors = []
    rotation_errors = []
    time_errors = []
    reference_index = 0
    for row in estimate:
        while (
            reference_index + 1 < len(reference)
            and abs(reference[reference_index + 1]["stamp_sec"] -
                    row["stamp_sec"])
            <= abs(reference[reference_index]["stamp_sec"] -
                   row["stamp_sec"])
        ):
            reference_index += 1
        reference_row = reference[reference_index]
        time_error = abs(reference_row["stamp_sec"] - row["stamp_sec"])
        if time_error > max_time_difference:
            continue
        time_errors.append(time_error)
        translation_errors.append(translation_distance(reference_row, row))
        rotation_errors.append(quaternion_error_deg(reference_row, row))
    return {
        "matched_poses": len(translation_errors),
        "maximum_time_difference_sec":
            max(time_errors) if time_errors else None,
        "translation_error_m": distribution(translation_errors),
        "rotation_error_deg": distribution(rotation_errors),
        "translation_rmse_m": (
            math.sqrt(statistics.fmean(
                value * value for value in translation_errors))
            if translation_errors else None
        ),
        "rotation_rmse_deg": (
            math.sqrt(statistics.fmean(
                value * value for value in rotation_errors))
            if rotation_errors else None
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_csv")
    parser.add_argument("--reference")
    parser.add_argument("--expected-hz", type=float, default=10.0)
    parser.add_argument("--max-time-difference", type=float, default=0.02)
    parser.add_argument("--output")
    args = parser.parse_args()

    metrics = load_metrics(args.metrics_csv)
    report = {
        "metrics_csv": str(Path(args.metrics_csv)),
        "runs": summarize(metrics, args.expected_hz),
    }
    if args.reference:
        reference = load_metrics(args.reference)
        report["reference_csv"] = str(Path(args.reference))
        report["comparison"] = compare(
            reference, metrics, args.max_time_difference)

    encoded = json.dumps(report, indent=2, sort_keys=True)
    print(encoded)
    if args.output:
        Path(args.output).write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

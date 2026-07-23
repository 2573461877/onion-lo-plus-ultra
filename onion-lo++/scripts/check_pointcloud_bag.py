#!/usr/bin/env python3
"""Check ROS1 PointCloud2 frame and per-point timestamp continuity.

This is an offline diagnostic utility; it does not publish ROS topics or
modify the input bag. It checks three independent failure indicators:

1. discontinuous ``header.seq`` values;
2. unexpectedly large intervals between consecutive ``header.stamp`` values;
3. gaps between the maximum point time of one frame and the minimum point
   time of the next frame.

MID-360 / livox_ros_driver2 PointCloud2 defaults are built in:
``/livox/lidar``, a FLOAT64 ``timestamp`` field containing absolute
nanoseconds, and a 10 Hz frame rate. Override the command-line options when
checking another topic or timestamp layout.

After building and sourcing the catkin workspace:

    rosrun onion_lo_plus check_pointcloud_bag.py /path/to/input.bag
"""

import argparse
import math
import statistics
import struct
import sys

import rosbag


FLOAT64_DATATYPE = 8


def point_time_bounds(message, field_name, scale, is_offset):
    """Return finite minimum/maximum point times in seconds.

    The PointCloud2 byte layout is read directly so this utility also checks
    that the configured time field fits inside each point record.
    """
    field = next(
        (candidate for candidate in message.fields
         if candidate.name == field_name),
        None,
    )
    if field is None:
        raise ValueError("PointCloud2 field '{}' is missing".format(field_name))
    if field.datatype != FLOAT64_DATATYPE or field.count != 1:
        raise ValueError(
            "PointCloud2 field '{}' must be FLOAT64[1], got datatype={} "
            "count={}".format(field_name, field.datatype, field.count)
        )
    if field.offset + 8 > message.point_step:
        raise ValueError(
            "PointCloud2 field '{}' exceeds point_step={}".format(
                field_name, message.point_step
            )
        )

    unpack_time = struct.Struct(">d" if message.is_bigendian else "<d")
    raw_data = memoryview(message.data)
    minimum = float("inf")
    maximum = -float("inf")
    finite_count = 0
    header_time = message.header.stamp.to_sec()

    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            offset = (
                row_offset + column * message.point_step + field.offset
            )
            raw_time = unpack_time.unpack_from(raw_data, offset)[0]
            point_time = (
                header_time + raw_time * scale
                if is_offset
                else raw_time * scale
            )
            if not math.isfinite(point_time):
                continue
            minimum = min(minimum, point_time)
            maximum = max(maximum, point_time)
            finite_count += 1

    if finite_count == 0:
        raise ValueError("PointCloud2 contains no finite point timestamps")
    return minimum, maximum, finite_count


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Report missing PointCloud2 sequence numbers, large header gaps, "
            "and gaps between consecutive frames' per-point timestamps."
        )
    )
    parser.add_argument("bag", help="Path to the ROS1 bag file")
    parser.add_argument(
        "--topic", default="/livox/lidar", help="PointCloud2 topic"
    )
    parser.add_argument(
        "--header-gap-ms",
        type=float,
        default=150.0,
        help="Report header intervals greater than this value",
    )
    parser.add_argument(
        "--point-gap-ms",
        type=float,
        default=20.0,
        help="Report point-time gaps greater than this value",
    )
    parser.add_argument(
        "--time-field", default="timestamp", help="Per-point time field"
    )
    parser.add_argument(
        "--time-scale",
        type=float,
        default=1.0e-9,
        help="Scale from the raw point-time unit to seconds",
    )
    parser.add_argument(
        "--time-is-offset",
        action="store_true",
        help="Interpret point time as an offset from header.stamp",
    )
    parser.add_argument(
        "--max-reports",
        type=int,
        default=100,
        help="Maximum number of anomaly details to print",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    previous = None
    message_count = 0
    anomaly_count = 0
    reported_count = 0
    header_intervals_ms = []
    point_spans_ms = []

    print("bag={}".format(args.bag))
    print("topic={}".format(args.topic))
    print(
        "thresholds: header_gap_ms>{:.3f}, point_gap_ms>{:.3f}".format(
            args.header_gap_ms, args.point_gap_ms
        )
    )

    with rosbag.Bag(args.bag, "r") as bag:
        for _, message, bag_time in bag.read_messages(topics=[args.topic]):
            if getattr(message, "_type", "") != "sensor_msgs/PointCloud2":
                raise TypeError(
                    "{} is {}, expected sensor_msgs/PointCloud2".format(
                        args.topic, getattr(message, "_type", "unknown")
                    )
                )

            point_min, point_max, finite_count = point_time_bounds(
                message,
                args.time_field,
                args.time_scale,
                args.time_is_offset,
            )
            header_time = message.header.stamp.to_sec()
            sequence = int(message.header.seq)
            point_count = int(message.width) * int(message.height)
            point_spans_ms.append((point_max - point_min) * 1000.0)
            message_count += 1

            # Keep sequence, message-time, and point-time checks separate.
            # A ROS subscriber can miss a frame even when the source bag is
            # continuous; comparing the original bag with a captured context
            # bag reveals which side of the transport path lost the frame.
            reasons = []
            header_gap_ms = None
            point_gap_ms = None
            if previous is not None:
                header_gap_ms = (
                    header_time - previous["header_time"]
                ) * 1000.0
                point_gap_ms = (
                    point_min - previous["point_max"]
                ) * 1000.0
                header_intervals_ms.append(header_gap_ms)

                expected_sequence = (previous["sequence"] + 1) & 0xFFFFFFFF
                if sequence != expected_sequence:
                    reasons.append(
                        "sequence {} -> {} (expected {})".format(
                            previous["sequence"],
                            sequence,
                            expected_sequence,
                        )
                    )
                if header_gap_ms > args.header_gap_ms:
                    reasons.append(
                        "header gap {:.3f} ms".format(header_gap_ms)
                    )
                if point_gap_ms > args.point_gap_ms:
                    reasons.append(
                        "point-time gap {:.3f} ms".format(point_gap_ms)
                    )

            if reasons:
                anomaly_count += 1
                if reported_count < args.max_reports:
                    reported_count += 1
                    print("\nANOMALY {}".format(anomaly_count))
                    print("  reasons: {}".format("; ".join(reasons)))
                    print(
                        "  previous: index={} seq={} header={:.9f} "
                        "point_max={:.9f}".format(
                            previous["index"],
                            previous["sequence"],
                            previous["header_time"],
                            previous["point_max"],
                        )
                    )
                    print(
                        "  current:  index={} seq={} header={:.9f} "
                        "point_min={:.9f} point_max={:.9f}".format(
                            message_count - 1,
                            sequence,
                            header_time,
                            point_min,
                            point_max,
                        )
                    )
                    print(
                        "  current_points={} finite_point_times={} "
                        "bag_record_time={:.9f}".format(
                            point_count,
                            finite_count,
                            bag_time.to_sec(),
                        )
                    )

            previous = {
                "index": message_count - 1,
                "sequence": sequence,
                "header_time": header_time,
                "point_min": point_min,
                "point_max": point_max,
            }

    if message_count == 0:
        print("ERROR: no messages found on {}".format(args.topic))
        return 2

    print("\nSUMMARY")
    print("  messages={}".format(message_count))
    print("  anomalies={}".format(anomaly_count))
    if header_intervals_ms:
        print(
            "  header_interval_ms min={:.3f} median={:.3f} "
            "mean={:.3f} max={:.3f}".format(
                min(header_intervals_ms),
                statistics.median(header_intervals_ms),
                statistics.mean(header_intervals_ms),
                max(header_intervals_ms),
            )
        )
    print(
        "  point_span_ms min={:.3f} median={:.3f} "
        "mean={:.3f} max={:.3f}".format(
            min(point_spans_ms),
            statistics.median(point_spans_ms),
            statistics.mean(point_spans_ms),
            max(point_spans_ms),
        )
    )
    if anomaly_count > reported_count:
        print(
            "  omitted_anomaly_reports={}".format(
                anomaly_count - reported_count
            )
        )
    print(
        "  result={}".format(
            "ANOMALIES_FOUND" if anomaly_count else "CONTINUOUS"
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

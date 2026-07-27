#!/usr/bin/env python3
"""Test-only relative-motion feed from an Onion mapping metrics CSV.

This helper is deliberately separate from the runtime launch. It listens to
the test PointCloud2 topic, looks up the time-aligned mapping pose, and
publishes nav_msgs/Odometry for inter-frame accumulation compensation. The
global relocalizer does not read the absolute pose as a candidate position.
Production use must replace this helper with wheel/INS relative odometry.
"""

import argparse
import bisect
import csv

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def load_records(path):
    records = []
    with open(path, "r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            records.append(
                (
                    float(row["stamp_sec"]),
                    float(row["x"]),
                    float(row["y"]),
                    float(row["z"]),
                    float(row["qx"]),
                    float(row["qy"]),
                    float(row["qz"]),
                    float(row["qw"]),
                )
            )
    if not records:
        raise RuntimeError("reference metrics CSV contains no poses")
    return records


class ReferenceOdometryPublisher:
    def __init__(
        self,
        records,
        cloud_topic,
        odom_topic,
        max_time_error,
        relay_cloud_topic,
    ):
        self.records = records
        self.stamps = [record[0] for record in records]
        self.max_time_error = max_time_error
        self.publisher = rospy.Publisher(odom_topic, Odometry, queue_size=200)
        self.cloud_publisher = (
            rospy.Publisher(relay_cloud_topic, PointCloud2, queue_size=200)
            if relay_cloud_topic
            else None
        )
        self.subscriber = rospy.Subscriber(
            cloud_topic, PointCloud2, self.cloud_callback, queue_size=200
        )

    def cloud_callback(self, cloud):
        stamp = cloud.header.stamp.to_sec()
        upper = bisect.bisect_left(self.stamps, stamp)
        candidates = []
        if upper < len(self.records):
            candidates.append(self.records[upper])
        if upper > 0:
            candidates.append(self.records[upper - 1])
        if not candidates:
            return
        record = min(candidates, key=lambda value: abs(value[0] - stamp))
        if abs(record[0] - stamp) > self.max_time_error:
            rospy.logwarn_throttle(
                1.0,
                "No reference odometry within %.3f s of cloud stamp %.6f",
                self.max_time_error,
                stamp,
            )
            return

        message = Odometry()
        message.header.stamp = cloud.header.stamp
        message.header.frame_id = "mapping_reference"
        message.child_frame_id = cloud.header.frame_id or "lidar"
        message.pose.pose.position.x = record[1]
        message.pose.pose.position.y = record[2]
        message.pose.pose.position.z = record[3]
        message.pose.pose.orientation.x = record[4]
        message.pose.pose.orientation.y = record[5]
        message.pose.pose.orientation.z = record[6]
        message.pose.pose.orientation.w = record[7]
        self.publisher.publish(message)
        if self.cloud_publisher is not None:
            self.cloud_publisher.publish(cloud)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("metrics_csv")
    parser.add_argument(
        "--cloud-topic", default="/onion/points_filtered_vehicle"
    )
    parser.add_argument(
        "--odom-topic", default="/scancontext_reference_odometry"
    )
    parser.add_argument("--max-time-error", type=float, default=0.02)
    parser.add_argument("--relay-cloud-topic", default="")
    args, _ = parser.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("scancontext_reference_odometry")
    ReferenceOdometryPublisher(
        load_records(args.metrics_csv),
        args.cloud_topic,
        args.odom_topic,
        args.max_time_error,
        args.relay_cloud_topic,
    )
    rospy.logwarn(
        "Using same-bag Onion mapping poses only for test motion "
        "compensation; this is not independent ground truth"
    )
    rospy.spin()


if __name__ == "__main__":
    main()

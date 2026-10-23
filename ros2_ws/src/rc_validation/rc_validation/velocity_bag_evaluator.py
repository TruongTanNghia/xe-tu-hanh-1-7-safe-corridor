#!/usr/bin/env python3

import argparse
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from rc_validation.velocity_report import SOURCES, write_velocity_results


TOPICS = {
    "/odometry/rf2o_raw": "raw",
    "/odometry/rf2o": "conditioned",
    "/odometry/filtered": "filtered",
}


def read_velocity_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="",
            output_serialization_format="",
        ),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    missing = sorted(set(TOPICS) - set(topic_types))
    if missing:
        raise ValueError(
            "bag thiếu topic canonical: " + ", ".join(missing)
        )
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=list(TOPICS))
    )

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in TOPICS
    }
    samples = {source: [] for source in SOURCES}
    first_timestamp_ns = None
    last_timestamp_ns = None

    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        last_timestamp_ns = timestamp_ns
        message = deserialize_message(serialized, message_types[topic])
        samples[TOPICS[topic]].append(
            {
                "elapsed_s": (timestamp_ns - first_timestamp_ns) * 1e-9,
                "stamp_sec": message.header.stamp.sec,
                "stamp_nanosec": message.header.stamp.nanosec,
                "value_mps": message.twist.twist.linear.x,
            }
        )

    if first_timestamp_ns is None or last_timestamp_ns is None:
        raise ValueError("bag không có message vận tốc")
    duration_sec = (last_timestamp_ns - first_timestamp_ns) * 1e-9
    return samples, duration_sec


def parse_args():
    parser = argparse.ArgumentParser(
        description="Đánh giá raw/conditioned/EKF trực tiếp từ rosbag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-speed", type=float, default=0.0)
    parser.add_argument("--spike-threshold", type=float, default=0.5)
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = args.bag.resolve()
    if not bag_path.exists():
        raise SystemExit(f"Không tìm thấy bag: {bag_path}")
    try:
        samples, duration_sec = read_velocity_bag(bag_path)
        passed, _ = write_velocity_results(
            samples=samples,
            output_dir=args.output_dir,
            expected_speed_mps=args.expected_speed,
            spike_threshold_mps=args.spike_threshold,
            duration_sec=duration_sec,
            started_utc=datetime.now(timezone.utc),
            source_kind="rosbag",
            source_path=bag_path,
        )
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f"Không đánh giá được bag: {error}") from error
    print(
        f"{'PASS' if passed else 'FAIL'}; "
        f"report={args.output_dir / 'report.md'}"
    )
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()


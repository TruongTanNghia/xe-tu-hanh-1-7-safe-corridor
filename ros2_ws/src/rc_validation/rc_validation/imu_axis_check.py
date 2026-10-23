#!/usr/bin/env python3

import argparse
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


def evaluate_gyro_direction(
    samples,
    expected_direction,
    active_threshold_radps=0.05,
    minimum_direction_fraction=0.8,
    minimum_rotation_rad=0.25,
):
    if expected_direction not in ("ccw", "cw"):
        raise ValueError("expected_direction must be ccw or cw")
    if len(samples) < 2:
        raise ValueError("need at least two gyro samples")

    sign = 1.0 if expected_direction == "ccw" else -1.0
    values = [float(item[1]) for item in samples]
    finite_values = [value for value in values if math.isfinite(value)]
    all_finite = len(finite_values) == len(values)
    if not finite_values:
        raise ValueError("gyro samples are all non-finite")
    active = [
        value for value in finite_values
        if abs(value) >= active_threshold_radps
    ]
    direction_fraction = (
        0.0
        if not active
        else sum(sign * value > 0.0 for value in active) / len(active)
    )
    integrated_angle = 0.0
    for previous, current in zip(samples, samples[1:]):
        delta_s = float(current[0] - previous[0])
        if (
            0.0 < delta_s < 0.5
            and math.isfinite(previous[1])
            and math.isfinite(current[1])
        ):
            integrated_angle += 0.5 * (
                float(previous[1]) + float(current[1])
            ) * delta_s

    failures = []
    if not all_finite:
        failures.append("non_finite_gyro_sample")
    if len(active) < 20:
        failures.append("too_few_active_rotation_samples")
    if direction_fraction < minimum_direction_fraction:
        failures.append("gyro_z_direction_inconsistent")
    if sign * integrated_angle < minimum_rotation_rad:
        failures.append("integrated_rotation_too_small_or_wrong_sign")

    return {
        "passed": not failures,
        "failures": failures,
        "expected_direction": expected_direction,
        "sample_count": len(samples),
        "active_sample_count": len(active),
        "active_threshold_radps": active_threshold_radps,
        "mean_gyro_z_radps": statistics.fmean(finite_values),
        "max_abs_gyro_z_radps": max(
            abs(value) for value in finite_values
        ),
        "direction_fraction": direction_fraction,
        "integrated_angle_rad": integrated_angle,
        "all_finite": all_finite,
    }


def read_imu_samples(bag_path, topic):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type
        for item in reader.get_all_topics_and_types()
    }
    if topic_types.get(topic) != "sensor_msgs/msg/Imu":
        raise ValueError(f"bag thiếu {topic} kiểu sensor_msgs/msg/Imu")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    samples = []
    first_timestamp_ns = None
    while reader.has_next():
        _, serialized, timestamp_ns = reader.read_next()
        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        message = deserialize_message(serialized, Imu)
        samples.append(
            (
                (timestamp_ns - first_timestamp_ns) * 1e-9,
                message.angular_velocity.z,
            )
        )
    return samples


def parse_args():
    parser = argparse.ArgumentParser(
        description="Kiểm tra dấu gyro.z khi quay xe từ rosbag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--expected", choices=("ccw", "cw"), required=True)
    parser.add_argument("--topic", default="/bno055/imu")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--active-threshold",
        type=float,
        default=0.05,
        help=(
            "Ngưỡng |gyro.z| để tính là đang quay. Mặc định 0.05 rad/s để "
            "bỏ qua nhiễu/nhích nhẹ sau khi dừng xe."
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        samples = read_imu_samples(args.bag.resolve(), args.topic)
        result = evaluate_gyro_direction(
            samples,
            args.expected,
            active_threshold_radps=args.active_threshold,
        )
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f"Không kiểm tra được gyro: {error}") from error

    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "gyro_axis_metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    report = [
        "# Kiểm tra trục BNO055 gyro.z",
        "",
        f"- Kết quả: **{'PASS' if result['passed'] else 'FAIL'}**",
        f"- Hướng mong đợi: `{args.expected}`",
        f"- Active threshold: `{result['active_threshold_radps']:.3f} rad/s`",
        f"- Active samples: `{result['active_sample_count']}`",
        f"- Direction fraction: `{result['direction_fraction']:.3f}`",
        f"- Integrated angle: `{result['integrated_angle_rad']:.3f} rad`",
        f"- Failures: `{', '.join(result['failures']) or 'none'}`",
    ]
    (args.output_dir / "gyro_axis_report.md").write_text(
        "\n".join(report) + "\n",
        encoding="utf-8",
    )
    print(
        f"{'PASS' if result['passed'] else 'FAIL'}; "
        f"report={args.output_dir / 'gyro_axis_report.md'}"
    )
    if not result["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

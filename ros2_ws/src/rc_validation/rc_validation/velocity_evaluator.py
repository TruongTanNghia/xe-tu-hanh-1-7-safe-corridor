#!/usr/bin/env python3

from datetime import datetime, timezone
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from rc_validation.velocity_report import (
    SOURCES,
    write_velocity_results,
)


class VelocityEvaluator(Node):
    SOURCES = SOURCES

    def __init__(self):
        super().__init__("velocity_evaluator")
        self.declare_parameter("raw_topic", "/odometry/rf2o_raw")
        self.declare_parameter("conditioned_topic", "/odometry/rf2o")
        self.declare_parameter("filtered_topic", "/odometry/filtered")
        self.declare_parameter("duration_sec", 60.0)
        self.declare_parameter("expected_speed_mps", 0.0)
        self.declare_parameter("spike_threshold_mps", 0.5)
        self.declare_parameter(
            "output_dir",
            "/tmp/rc_velocity_evaluation",
        )

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.expected_speed_mps = float(
            self.get_parameter("expected_speed_mps").value
        )
        self.spike_threshold_mps = float(
            self.get_parameter("spike_threshold_mps").value
        )
        self.output_dir = Path(str(self.get_parameter("output_dir").value))
        if self.duration_sec <= 0.0:
            raise ValueError("duration_sec must be positive")
        if self.spike_threshold_mps <= 0.0:
            raise ValueError("spike_threshold_mps must be positive")

        self.start_monotonic = time.monotonic()
        self.start_utc = datetime.now(timezone.utc)
        self.samples = {source: [] for source in self.SOURCES}
        self.done = False

        for source in self.SOURCES:
            topic = str(self.get_parameter(f"{source}_topic").value)
            self.create_subscription(
                Odometry,
                topic,
                lambda message, source=source: self._callback(source, message),
                10,
            )

        self.timer = self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            f"Recording velocity for {self.duration_sec:.1f}s; "
            f"expected={self.expected_speed_mps:.3f} m/s"
        )

    def _callback(self, source, message):
        self.samples[source].append(
            {
                "elapsed_s": time.monotonic() - self.start_monotonic,
                "stamp_sec": message.header.stamp.sec,
                "stamp_nanosec": message.header.stamp.nanosec,
                "value_mps": message.twist.twist.linear.x,
            }
        )

    def _timer_callback(self):
        if time.monotonic() - self.start_monotonic >= self.duration_sec:
            self.done = True

    def write_results(self):
        return write_velocity_results(
            samples=self.samples,
            output_dir=self.output_dir,
            expected_speed_mps=self.expected_speed_mps,
            spike_threshold_mps=self.spike_threshold_mps,
            duration_sec=self.duration_sec,
            started_utc=self.start_utc,
            source_kind="live",
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityEvaluator()
    passed = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        passed, _ = node.write_results()
        node.get_logger().info(
            f"{'PASS' if passed else 'FAIL'}; results={node.output_dir}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(2)

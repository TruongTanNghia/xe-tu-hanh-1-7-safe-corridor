#!/usr/bin/env python3

import csv
from pathlib import Path
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray


class LongitudinalRecorder(Node):
    ARMED_FLAG = 0x02

    def __init__(self):
        super().__init__("longitudinal_recorder")
        self.declare_parameter("duration_sec", 60.0)
        self.declare_parameter(
            "output_path",
            "/tmp/longitudinal_samples.csv",
        )
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("actuator_state_topic", "/mcu/actuator_state")
        self.declare_parameter("state_timeout_sec", 0.2)
        self.declare_parameter("esc_stop_us", 1550)
        self.declare_parameter("esc_forward_start_us", 1580)
        self.declare_parameter("esc_max_us", 1600)

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.output_path = Path(
            str(self.get_parameter("output_path").value)
        )
        self.state_timeout_sec = float(
            self.get_parameter("state_timeout_sec").value
        )
        self.esc_stop_us = int(self.get_parameter("esc_stop_us").value)
        self.esc_forward_start_us = int(
            self.get_parameter("esc_forward_start_us").value
        )
        self.esc_max_us = int(self.get_parameter("esc_max_us").value)
        if self.duration_sec <= 0.0 or self.state_timeout_sec <= 0.0:
            raise ValueError("duration and timeout must be positive")
        if not self.esc_stop_us < self.esc_forward_start_us < self.esc_max_us:
            raise ValueError("invalid ESC calibration")

        self.started = time.monotonic()
        self.latest_state = None
        self.latest_state_time = None
        self.samples = []
        self.done = False
        self.create_subscription(
            Int16MultiArray,
            str(self.get_parameter("actuator_state_topic").value),
            self._state_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            10,
        )
        self.create_timer(0.1, self._timer_callback)

    def _state_callback(self, message):
        if len(message.data) < 3:
            return
        self.latest_state = tuple(int(value) for value in message.data[:3])
        self.latest_state_time = time.monotonic()

    def _effort_from_pwm(self, esc_us):
        if esc_us <= self.esc_stop_us:
            return 0.0
        return float(
            np.clip(
                (esc_us - self.esc_forward_start_us)
                / (self.esc_max_us - self.esc_forward_start_us),
                0.0,
                1.0,
            )
        )

    def _odom_callback(self, message):
        now = time.monotonic()
        if (
            self.latest_state is None
            or self.latest_state_time is None
            or now - self.latest_state_time > self.state_timeout_sec
        ):
            return
        velocity = float(message.twist.twist.linear.x)
        if not np.isfinite(velocity):
            return
        _, esc_us, flags = self.latest_state
        self.samples.append(
            {
                "time_s": now - self.started,
                "esc_us": esc_us,
                "drive_effort": self._effort_from_pwm(esc_us),
                "vx_mps": velocity,
                "armed": int(bool(flags & self.ARMED_FLAG)),
            }
        )

    def _timer_callback(self):
        if time.monotonic() - self.started >= self.duration_sec:
            self.done = True

    def write_csv(self):
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "time_s",
                    "esc_us",
                    "drive_effort",
                    "vx_mps",
                    "armed",
                ],
            )
            writer.writeheader()
            writer.writerows(self.samples)
        self.get_logger().info(
            f"Saved {len(self.samples)} samples to {self.output_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LongitudinalRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


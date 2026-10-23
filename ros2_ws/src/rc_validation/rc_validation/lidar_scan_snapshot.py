#!/usr/bin/env python3

import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rc_validation.lidar_render import payload_to_svg, scan_to_payload


class LidarScanSnapshot(Node):
    def __init__(self):
        super().__init__("lidar_scan_snapshot")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("output_path", "/tmp/lidar_scan.svg")
        self.declare_parameter("timeout_sec", 10.0)

        self.topic = str(self.get_parameter("topic").value)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        if self.timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        if self.output_path.suffix.lower() != ".svg":
            raise ValueError("output_path must end with .svg")

        self.saved = False
        self.subscription = self.create_subscription(
            LaserScan,
            self.topic,
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, message: LaserScan):
        if self.saved:
            return
        payload = scan_to_payload(
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            ranges=message.ranges,
            range_min=message.range_min,
            range_max=message.range_max,
            frame_id=message.header.frame_id,
            stamp_sec=message.header.stamp.sec,
            stamp_nanosec=message.header.stamp.nanosec,
            scan_rate_hz=0.0,
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path.write_text(
            payload_to_svg(payload),
            encoding="utf-8",
        )
        json_path = self.output_path.with_suffix(".json")
        json_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        self.saved = True
        self.get_logger().info(
            f"Saved {payload['valid_count']}/{payload['total_count']} points "
            f"to {self.output_path} and {json_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanSnapshot()
    deadline = time.monotonic() + node.timeout_sec
    try:
        while rclpy.ok() and not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.saved:
            raise RuntimeError(
                f"Timeout after {node.timeout_sec:.1f}s waiting for {node.topic}"
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

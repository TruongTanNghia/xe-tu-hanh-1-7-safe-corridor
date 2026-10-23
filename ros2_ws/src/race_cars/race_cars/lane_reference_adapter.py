#!/usr/bin/env python3

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rc_interfaces.msg import LaneReference
from std_msgs.msg import Float64, Float64MultiArray, String


class LaneReferenceAdapter(Node):
    def __init__(self):
        super().__init__("lane_reference_adapter")
        self.declare_parameter("max_skew_s", 0.12)
        self.declare_parameter("source_timeout_s", 0.3)
        self.declare_parameter("frame_id", "base_link")

        self.max_skew_s = float(self.get_parameter("max_skew_s").value)
        self.source_timeout_s = float(
            self.get_parameter("source_timeout_s").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.values = {}
        self.times = {}
        self.last_publish_time = 0.0
        self.publisher = self.create_publisher(
            LaneReference,
            "/lane/reference",
            10,
        )

        self.create_subscription(
            Float64,
            "/line/cte",
            lambda message: self._store("cte", message.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/heading_angle",
            lambda message: self._store("heading", message.data),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/line/s_metric",
            lambda message: self._store("s", list(message.data)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/line/kappa_s",
            self._curvature_callback,
            10,
        )
        self.create_subscription(
            String,
            "/line/status",
            lambda message: self._store("status", message.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/confidence",
            lambda message: self._store("confidence", message.data),
            10,
        )
        # Optional extras consumed by the safe-corridor planner. They are not
        # part of the coherence check so the NMPC path is unchanged.
        self.create_subscription(
            Float64,
            "/line/evaluation_forward",
            lambda message: self._store("evaluation_forward", message.data),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/line/reference_path",
            lambda message: self._store("path_lateral", list(message.data)),
            10,
        )
        self.timer = self.create_timer(0.05, self._health_timer)

    def _store(self, key, value):
        self.values[key] = value
        self.times[key] = time.monotonic()

    def _curvature_callback(self, message):
        self._store("curvature", list(message.data))
        self._publish_detected_if_coherent()

    def _publish_detected_if_coherent(self):
        required = ("cte", "heading", "s", "curvature", "status", "confidence")
        if not all(key in self.values for key in required):
            return
        if self.values["status"] != "line_detected":
            self._publish_invalid(LaneReference.STATUS_INVALID)
            return

        status_time = self.times["status"]
        measurement_times = [
            self.times[key] for key in ("cte", "heading", "s", "curvature")
        ]
        if any(value < status_time for value in measurement_times):
            return
        all_times = measurement_times + [
            status_time,
            self.times["confidence"],
        ]
        if max(all_times) - min(all_times) > self.max_skew_s:
            # A single incoherent callback set is not evidence that the lane
            # itself is stale.  Drop this partial frame and let the health
            # timer invalidate the reference only when no coherent frame has
            # arrived for source_timeout_s.
            return

        message = LaneReference()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.s_m = self.values["s"]
        message.curvature_inv_m = self.values["curvature"]
        message.cte_m = float(self.values["cte"])
        message.heading_error_rad = float(self.values["heading"])
        message.confidence = float(self.values["confidence"])
        message.status = LaneReference.STATUS_DETECTED
        message.evaluation_forward_m = float(
            self.values.get("evaluation_forward", 0.0)
        )
        path_lateral = self.values.get("path_lateral", [])
        if len(path_lateral) == len(message.s_m):
            message.path_lateral_m = [float(v) for v in path_lateral]
        self.publisher.publish(message)
        self.last_publish_time = time.monotonic()

    def _publish_invalid(self, status):
        message = LaneReference()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.status = status
        message.confidence = float(self.values.get("confidence", 0.0))
        self.publisher.publish(message)
        self.last_publish_time = time.monotonic()

    def _health_timer(self):
        now = time.monotonic()
        status_time = self.times.get("status")
        if status_time is None or now - status_time > self.source_timeout_s:
            self._publish_invalid(LaneReference.STATUS_STALE)
        elif self.values.get("status") != "line_detected":
            self._publish_invalid(LaneReference.STATUS_INVALID)
        elif now - self.last_publish_time > self.source_timeout_s:
            self._publish_invalid(LaneReference.STATUS_STALE)


def main(args=None):
    rclpy.init(args=args)
    node = LaneReferenceAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

#!/usr/bin/env python3
"""PID baseline controller using shared image_processing perception.

Subscribes to /line/cte and /line/heading_angle from image_processor.
Publishes /cmd_vel for the ackermann_cmd_bridge.

Stops (zero Twist) when:
- /line/status != "line_detected"
- /line/confidence < min_confidence
- line data has timed out
- required topics not yet received
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64, String


class PIDBaselineController(Node):
    def __init__(self):
        super().__init__('pid_baseline_controller')

        self.declare_parameter('target_speed', 0.30)
        self.declare_parameter('kp_cte', 1.0)
        self.declare_parameter('ki_cte', 0.0)
        self.declare_parameter('kd_cte', 0.05)
        self.declare_parameter('kp_heading', 1.0)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('max_angular_z', 2.0)
        self.declare_parameter('min_confidence', 0.25)
        self.declare_parameter('line_timeout_s', 0.5)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('publish_rate_hz', 30.0)

        self._target_speed = self.get_parameter('target_speed').as_double()
        self._kp_cte = self.get_parameter('kp_cte').as_double()
        self._ki_cte = self.get_parameter('ki_cte').as_double()
        self._kd_cte = self.get_parameter('kd_cte').as_double()
        self._kp_heading = self.get_parameter('kp_heading').as_double()
        self._steering_sign = self.get_parameter('steering_sign').as_double()
        self._max_angular_z = self.get_parameter('max_angular_z').as_double()
        self._min_confidence = self.get_parameter('min_confidence').as_double()
        self._line_timeout = self.get_parameter('line_timeout_s').as_double()
        self._integral_limit = self.get_parameter('integral_limit').as_double()
        self._rate = self.get_parameter('publish_rate_hz').as_double()

        self._cte = None
        self._heading_angle = None
        self._line_status = None
        self._confidence = None
        self._last_line_time = 0.0
        self._integral_cte = 0.0
        self._prev_cte = None
        self._prev_time = None

        self.create_subscription(Float64, '/line/cte', self._on_cte, 10)
        self.create_subscription(Float64, '/line/heading_angle', self._on_heading, 10)
        self.create_subscription(String, '/line/status', self._on_status, 10)
        self.create_subscription(Float64, '/line/confidence', self._on_confidence, 10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / self._rate, self._on_timer)

        self.get_logger().info(
            'PID Baseline Controller started | speed=%.2f | kp_cte=%.1f | kd_cte=%.2f | '
            'kp_heading=%.1f | steering_sign=%.1f',
            self._target_speed, self._kp_cte, self._kd_cte,
            self._kp_heading, self._steering_sign
        )

    def _on_cte(self, msg: Float64):
        self._cte = msg.data

    def _on_heading(self, msg: Float64):
        self._heading_angle = msg.data

    def _on_status(self, msg: String):
        self._line_status = msg.data
        self._last_line_time = time.monotonic()

    def _on_confidence(self, msg: Float64):
        self._confidence = msg.data

    def _stopped(self) -> bool:
        now = time.monotonic()

        if self._line_status is None or self._cte is None or self._heading_angle is None or self._confidence is None:
            return True

        if self._line_status != 'line_detected':
            return True

        if self._confidence < self._min_confidence:
            return True

        if now - self._last_line_time > self._line_timeout:
            return True

        return False

    def _on_timer(self):
        cmd = Twist()

        if self._stopped():
            if self._cmd_pub.get_subscription_count() > 0:
                self.get_logger().warn(
                    '[PID] Stopped: status=%s conf=%.2f timeout=%.1fs',
                    str(self._line_status),
                    self._confidence if self._confidence is not None else -1.0,
                    time.monotonic() - self._last_line_time if self._last_line_time > 0 else 999.0
                )
            self._integral_cte = 0.0
            self._prev_cte = None
            self._cmd_pub.publish(cmd)
            return

        now = time.monotonic()
        dt = now - self._prev_time if self._prev_time is not None else 0.0
        self._prev_time = now

        error = self._steering_sign * (
            self._kp_cte * self._cte +
            self._ki_cte * self._integral_cte +
            self._kd_cte * ((self._cte - self._prev_cte) / dt if dt > 0 and self._prev_cte is not None else 0.0)
        )
        error += self._steering_sign * self._kp_heading * self._heading_angle

        angular_z = max(-self._max_angular_z, min(self._max_angular_z, error))

        self._integral_cte += self._ki_cte * self._cte * dt
        self._integral_cte = max(-self._integral_limit, min(self._integral_limit, self._integral_cte))

        self._prev_cte = self._cte

        cmd.linear.x = self._target_speed
        cmd.angular.z = angular_z
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PIDBaselineController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
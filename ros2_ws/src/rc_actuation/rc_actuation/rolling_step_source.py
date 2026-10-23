#!/usr/bin/env python3

import math
import time

from nav_msgs.msg import Odometry
from rc_interfaces.msg import ControlCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from rc_actuation.esc_step_source import EscStepSchedule


DEFAULT_ODOM_TIMEOUT_S = 0.6


class RollingStepSource(Node):
    """Safety-gated low-speed step source for rolling model calibration."""

    REQUIRED_ACK = "ROLLING_SLOW_MOTOR_POWER_CONTROLLED"

    def __init__(self):
        super().__init__("rolling_step_source")
        self.declare_parameter("safety_ack", "")
        self.declare_parameter(
            "step_efforts",
            [0.01, 0.25, 0.50, 0.75, 1.0],
        )
        self.declare_parameter("stop_duration_s", 3.0)
        self.declare_parameter("step_duration_s", 5.0)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("odom_topic", "/odometry/filtered")
        # LiDAR/RF2O is observed at about 6.4 Hz. A 0.3 s timeout is too close
        # to two scan periods and aborts on normal DDS/scan jitter. Keep the
        # freshness guard, but allow a bounded gap of roughly three periods.
        self.declare_parameter("odom_timeout_s", DEFAULT_ODOM_TIMEOUT_S)
        self.declare_parameter("max_speed_mps", 0.35)

        safety_ack = str(self.get_parameter("safety_ack").value)
        if safety_ack != self.REQUIRED_ACK:
            raise RuntimeError(
                "Refusing rolling calibration: set safety_ack="
                f"{self.REQUIRED_ACK}"
            )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.odom_timeout_s = float(
            self.get_parameter("odom_timeout_s").value
        )
        self.max_speed_mps = float(
            self.get_parameter("max_speed_mps").value
        )
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.odom_timeout_s <= 0.0 or self.max_speed_mps <= 0.0:
            raise ValueError("odom timeout and max speed must be positive")

        self.schedule = EscStepSchedule(
            [
                float(value)
                for value in self.get_parameter("step_efforts").value
            ],
            float(self.get_parameter("stop_duration_s").value),
            float(self.get_parameter("step_duration_s").value),
        )
        self.started_at = None
        self.finished = False
        self.aborted = False
        self.abort_reason = ""
        self.latest_vx = None
        self.latest_odom_time = None
        self.disarm_requested = False

        odom_topic = str(self.get_parameter("odom_topic").value)
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.command_publisher = self.create_publisher(
            ControlCommand,
            "/control/nmpc",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/calibration/rolling_step_status",
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            "/calibration/rolling_start",
            self._start_callback,
        )
        self.cancel_service = self.create_service(
            Trigger,
            "/calibration/rolling_cancel",
            self._cancel_callback,
        )
        self.disarm_client = self.create_client(SetBool, "/actuation/arm")
        self.timer = self.create_timer(1.0 / rate_hz, self._timer_callback)
        self.get_logger().warning(
            "ROLLING calibration READY_STOP. It will not step until "
            "/calibration/rolling_start is called."
        )

    def _odom_callback(self, message):
        value = float(message.twist.twist.linear.x)
        if math.isfinite(value):
            self.latest_vx = value
            self.latest_odom_time = time.monotonic()

    def _fresh_safe_speed(self):
        if self.latest_vx is None or self.latest_odom_time is None:
            return False, "odometry missing"
        age = time.monotonic() - self.latest_odom_time
        if age > self.odom_timeout_s:
            return False, f"odometry stale ({age:.3f}s)"
        if abs(self.latest_vx) > self.max_speed_mps:
            return (
                False,
                f"speed limit exceeded ({self.latest_vx:.3f} m/s)",
            )
        return True, ""

    def _other_control_publishers_exist(self):
        return len(self.get_publishers_info_by_topic("/control/nmpc")) > 1

    def _publish(self, effort, valid, reason, status):
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.steering_angle_rad = 0.0
        message.drive_effort = float(effort)
        message.valid = bool(valid)
        message.reason = int(reason)
        self.command_publisher.publish(message)
        self.status_publisher.publish(String(data=status))

    def _request_disarm(self):
        if self.disarm_requested:
            return
        self.disarm_requested = True
        if not self.disarm_client.service_is_ready():
            self.get_logger().error(
                "Actuation arm service unavailable; use operator disarm/E-stop"
            )
            return
        request = SetBool.Request()
        request.data = False
        self.disarm_client.call_async(request)

    def _abort(self, reason):
        if self.aborted:
            return
        self.aborted = True
        self.finished = True
        self.abort_reason = reason
        self._publish(
            0.0,
            False,
            ControlCommand.REASON_INPUT_INVALID,
            f"ABORTED_{reason}",
        )
        self._request_disarm()
        self.get_logger().error(f"Rolling calibration aborted: {reason}")

    def _start_callback(self, _request, response):
        if self._other_control_publishers_exist():
            response.success = False
            response.message = "other /control/nmpc publisher detected"
            return response
        safe, reason = self._fresh_safe_speed()
        if not safe:
            response.success = False
            response.message = reason
            return response
        if self.started_at is not None and not self.finished:
            response.success = False
            response.message = "sequence already running"
            return response
        self.started_at = time.monotonic()
        self.finished = False
        self.aborted = False
        self.abort_reason = ""
        self.disarm_requested = False
        response.success = True
        response.message = (
            f"started; duration={self.schedule.total_duration_s:.1f}s; "
            f"max_speed={self.max_speed_mps:.3f}m/s"
        )
        return response

    def _cancel_callback(self, _request, response):
        self._abort("operator_cancel")
        response.success = True
        response.message = "cancelled; publishing invalid stop"
        return response

    def _timer_callback(self):
        if self._other_control_publishers_exist():
            self._abort("MULTIPLE_CONTROL_PUBLISHERS")
            return
        if self.aborted:
            self._publish(
                0.0,
                False,
                ControlCommand.REASON_INPUT_INVALID,
                f"ABORTED_{self.abort_reason}",
            )
            return
        if self.finished:
            self._publish(
                0.0,
                False,
                ControlCommand.REASON_STARTUP,
                "DONE_STOP",
            )
            return
        if self.started_at is None:
            self._publish(
                0.0,
                True,
                ControlCommand.REASON_OK,
                "READY_STOP",
            )
            return

        safe, reason = self._fresh_safe_speed()
        if not safe:
            self._abort(reason)
            return

        point = self.schedule.at(time.monotonic() - self.started_at)
        if point is None:
            self.finished = True
            self._publish(
                0.0,
                False,
                ControlCommand.REASON_STARTUP,
                "COMPLETE_STOP",
            )
            return
        self._publish(
            point.effort,
            True,
            ControlCommand.REASON_OK,
            point.label,
        )

    def _shutdown_stop(self):
        for _ in range(5):
            self._publish(
                0.0,
                False,
                ControlCommand.REASON_STARTUP,
                "SHUTDOWN_STOP",
            )
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RollingStepSource()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._shutdown_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

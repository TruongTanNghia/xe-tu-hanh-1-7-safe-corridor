#!/usr/bin/env python3

from dataclasses import dataclass
import time

from rc_interfaces.msg import ControlCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True)
class SchedulePoint:
    label: str
    effort: float
    duration_s: float


class EscStepSchedule:
    def __init__(self, efforts, stop_duration_s, step_duration_s):
        if not efforts or any(not 0.0 < value <= 1.0 for value in efforts):
            raise ValueError("step efforts must be in (0, 1]")
        if stop_duration_s <= 0.0 or step_duration_s <= 0.0:
            raise ValueError("durations must be positive")
        self.points = []
        for effort in efforts:
            self.points.append(
                SchedulePoint("stop", 0.0, float(stop_duration_s))
            )
            pwm = round(1580 + effort * 20)
            self.points.append(
                SchedulePoint(
                    f"step_{pwm}us",
                    float(effort),
                    float(step_duration_s),
                )
            )
        self.points.append(
            SchedulePoint("final_stop", 0.0, float(stop_duration_s))
        )
        self.total_duration_s = sum(point.duration_s for point in self.points)

    def at(self, elapsed_s):
        if elapsed_s < 0.0:
            raise ValueError("elapsed time must be non-negative")
        cursor = 0.0
        for point in self.points:
            cursor += point.duration_s
            if elapsed_s < cursor:
                return point
        return None


class EscStepSource(Node):
    REQUIRED_ACK = "WHEELS_UP_MOTOR_POWER_CONTROLLED"

    def __init__(self):
        super().__init__("esc_step_source")
        self.declare_parameter("safety_ack", "")
        self.declare_parameter(
            "step_efforts",
            [0.01, 0.25, 0.50, 0.75, 1.0],
        )
        self.declare_parameter("stop_duration_s", 3.0)
        self.declare_parameter("step_duration_s", 5.0)
        self.declare_parameter("publish_rate_hz", 50.0)

        safety_ack = str(self.get_parameter("safety_ack").value)
        if safety_ack != self.REQUIRED_ACK:
            raise RuntimeError(
                "Refusing calibration: set safety_ack="
                f"{self.REQUIRED_ACK}"
            )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
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
        self.cancelled = False

        self.command_publisher = self.create_publisher(
            ControlCommand,
            "/control/nmpc",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/calibration/esc_step_status",
            10,
        )
        self.start_service = self.create_service(
            Trigger,
            "/calibration/start",
            self._start_callback,
        )
        self.cancel_service = self.create_service(
            Trigger,
            "/calibration/cancel",
            self._cancel_callback,
        )
        self.timer = self.create_timer(1.0 / rate_hz, self._timer_callback)
        self.get_logger().warning(
            "ESC calibration source READY_STOP. It will not step until "
            "/calibration/start is called."
        )

    def _other_control_publishers_exist(self):
        return len(self.get_publishers_info_by_topic("/control/nmpc")) > 1

    def _start_callback(self, _request, response):
        if self._other_control_publishers_exist():
            response.success = False
            response.message = "other /control/nmpc publisher detected"
            return response
        if self.started_at is not None and not self.finished:
            response.success = False
            response.message = "sequence already running"
            return response
        self.started_at = time.monotonic()
        self.finished = False
        self.cancelled = False
        response.success = True
        response.message = (
            f"started; duration={self.schedule.total_duration_s:.1f}s"
        )
        return response

    def _cancel_callback(self, _request, response):
        self.cancelled = True
        self.finished = True
        response.success = True
        response.message = "cancelled; publishing invalid stop"
        return response

    def _publish(self, effort, valid, reason, status):
        message = ControlCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.steering_angle_rad = 0.0
        message.drive_effort = float(effort)
        message.valid = bool(valid)
        message.reason = int(reason)
        self.command_publisher.publish(message)
        self.status_publisher.publish(String(data=status))

    def _timer_callback(self):
        if self._other_control_publishers_exist():
            self.cancelled = True
            self.finished = True
            self._publish(
                0.0,
                False,
                ControlCommand.REASON_INPUT_INVALID,
                "ABORTED_MULTIPLE_CONTROL_PUBLISHERS",
            )
            return
        if self.cancelled or self.finished:
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


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EscStepSource()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(5):
                    node._publish(
                        0.0,
                        False,
                        ControlCommand.REASON_STARTUP,
                        "SHUTDOWN_STOP",
                    )
                    rclpy.spin_once(node, timeout_sec=0.0)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

#!/usr/bin/env python3

from enum import Enum
import time

from rc_interfaces.msg import ControlCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from std_msgs.msg import Bool, Int16MultiArray, String, UInt32
from std_srvs.srv import SetBool, Trigger

from rc_actuation.actuation_mapper import ActuationMapper


class ActuationState(Enum):
    DISARMED = "DISARMED"
    ACTIVE = "ACTIVE"
    ESTOP = "ESTOP"


class ActuationManager(Node):
    def __init__(self):
        super().__init__("actuation_manager")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.2)
        self.declare_parameter("mcu_timeout_s", 0.75)
        self.declare_parameter("invalid_command_grace_s", 0.5)
        self.declare_parameter("steering_calibrated", False)
        self.declare_parameter("esc_calibrated", True)
        self.declare_parameter("servo_center_us", 1500)
        self.declare_parameter("servo_min_us", 1150)
        self.declare_parameter("servo_max_us", 1850)
        self.declare_parameter("servo_us_per_rad", 245.9)
        self.declare_parameter("esc_stop_us", 1550)
        self.declare_parameter("esc_forward_start_us", 1580)
        self.declare_parameter("esc_max_us", 1600)
        self.declare_parameter("esc_rise_rate_us_per_s", 50.0)

        self.control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self.mcu_timeout_s = float(
            self.get_parameter("mcu_timeout_s").value
        )
        self.invalid_command_grace_s = float(
            self.get_parameter("invalid_command_grace_s").value
        )
        self.steering_calibrated = bool(
            self.get_parameter("steering_calibrated").value
        )
        self.esc_calibrated = bool(
            self.get_parameter("esc_calibrated").value
        )
        if (
            self.control_rate_hz <= 0.0
            or self.command_timeout_s <= 0.0
            or self.mcu_timeout_s <= 0.0
            or self.invalid_command_grace_s < 0.0
        ):
            raise ValueError("invalid rate or timeout")

        self.mapper = ActuationMapper(
            servo_center_us=self.get_parameter("servo_center_us").value,
            servo_min_us=self.get_parameter("servo_min_us").value,
            servo_max_us=self.get_parameter("servo_max_us").value,
            servo_us_per_rad=self.get_parameter("servo_us_per_rad").value,
            esc_stop_us=self.get_parameter("esc_stop_us").value,
            esc_forward_start_us=self.get_parameter(
                "esc_forward_start_us"
            ).value,
            esc_max_us=self.get_parameter("esc_max_us").value,
            esc_rise_rate_us_per_s=self.get_parameter(
                "esc_rise_rate_us_per_s"
            ).value,
        )

        self.state = ActuationState.DISARMED
        self.latest_command = None
        self.last_command_time = None
        self.last_mcu_time = None
        self.invalid_command_since = None
        self.current_esc_us = self.mapper.esc_stop_us
        self.last_control_time = time.monotonic()
        self.estop_reason = ""

        self.command_subscription = self.create_subscription(
            ControlCommand,
            "/control/nmpc",
            self._command_callback,
            10,
        )
        self.mcu_subscription = self.create_subscription(
            UInt32,
            "/mcu/status",
            self._mcu_callback,
            10,
        )
        self.pwm_publisher = self.create_publisher(
            Int16MultiArray,
            "/mcu/actuator_cmd",
            10,
        )
        self.arm_publisher = self.create_publisher(Bool, "/mcu/arm", 10)
        self.status_publisher = self.create_publisher(
            String,
            "/actuation/status",
            10,
        )
        self.arm_service = self.create_service(
            SetBool,
            "/actuation/arm",
            self._arm_callback,
        )
        self.estop_service = self.create_service(
            Trigger,
            "/actuation/estop",
            self._estop_callback,
        )
        self.clear_estop_service = self.create_service(
            Trigger,
            "/actuation/clear_estop",
            self._clear_estop_callback,
        )
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_loop,
        )
        self._publish_stop("startup")
        self.get_logger().warning(
            "Actuation starts DISARMED. Arm requires fresh valid control, "
            "fresh MCU heartbeat, and completed calibration."
        )

    def _command_callback(self, message):
        self.latest_command = message
        self.last_command_time = time.monotonic()

    def _mcu_callback(self, _message):
        self.last_mcu_time = time.monotonic()

    def _is_fresh(self, timestamp, timeout):
        return timestamp is not None and time.monotonic() - timestamp <= timeout

    def _arm_precondition_failure(self):
        if self.state == ActuationState.ESTOP:
            return "E-stop đang latch"
        if not self.steering_calibrated:
            return "steering_calibrated=false"
        if not self.esc_calibrated:
            return "esc_calibrated=false"
        if not self._is_fresh(self.last_mcu_time, self.mcu_timeout_s):
            return "MCU heartbeat thiếu hoặc stale"
        if not self._is_fresh(self.last_command_time, self.command_timeout_s):
            return "control command thiếu hoặc stale"
        if self.latest_command is None or not self.latest_command.valid:
            return "control command invalid/shadow"
        return ""

    def _arm_callback(self, request, response):
        if not request.data:
            self.state = ActuationState.DISARMED
            self.invalid_command_since = None
            self._publish_stop("operator_disarm")
            response.success = True
            response.message = "DISARMED"
            return response

        failure = self._arm_precondition_failure()
        if failure:
            self._publish_stop(f"arm_rejected:{failure}")
            response.success = False
            response.message = failure
            return response

        self.state = ActuationState.ACTIVE
        self.invalid_command_since = None
        response.success = True
        response.message = "ACTIVE"
        return response

    def _estop_callback(self, _request, response):
        self.state = ActuationState.ESTOP
        self.invalid_command_since = None
        self.estop_reason = "operator_estop"
        self._publish_stop(self.estop_reason)
        response.success = True
        response.message = "ESTOP latched"
        return response

    def _clear_estop_callback(self, _request, response):
        if self.state != ActuationState.ESTOP:
            response.success = True
            response.message = "E-stop was not latched"
            return response
        self.state = ActuationState.DISARMED
        self.invalid_command_since = None
        self.estop_reason = ""
        self._publish_stop("estop_cleared_disarmed")
        response.success = True
        response.message = "E-stop cleared; state is DISARMED"
        return response

    def _other_actuator_publishers_exist(self):
        publishers = self.get_publishers_info_by_topic("/mcu/actuator_cmd")
        return len(publishers) > 1

    def _publish_pwm(self, servo_us, esc_us, status):
        self.pwm_publisher.publish(
            Int16MultiArray(data=[int(servo_us), int(esc_us)])
        )
        armed = self.state == ActuationState.ACTIVE
        self.arm_publisher.publish(Bool(data=armed))
        self.status_publisher.publish(
            String(
                data=(
                    f"state={self.state.value};status={status};"
                    f"servo_us={int(servo_us)};esc_us={int(esc_us)}"
                )
            )
        )

    def _publish_stop(self, status):
        self.current_esc_us = self.mapper.esc_stop_us
        self._publish_pwm(
            self.mapper.servo_center_us,
            self.mapper.esc_stop_us,
            status,
        )

    def _control_loop(self):
        now = time.monotonic()
        delta_s = now - self.last_control_time
        self.last_control_time = now

        if self._other_actuator_publishers_exist():
            self.state = ActuationState.ESTOP
            self.estop_reason = "multiple_actuator_publishers"
            self._publish_stop(self.estop_reason)
            return
        if self.state != ActuationState.ACTIVE:
            self.invalid_command_since = None
            self._publish_stop(self.state.value.lower())
            return
        if not self._is_fresh(self.last_mcu_time, self.mcu_timeout_s):
            self.state = ActuationState.DISARMED
            self._publish_stop("mcu_stale")
            return
        if not self._is_fresh(self.last_command_time, self.command_timeout_s):
            self.state = ActuationState.DISARMED
            self._publish_stop("command_stale")
            return
        if self.latest_command is None or not self.latest_command.valid:
            if self.invalid_command_since is None:
                self.invalid_command_since = now
            invalid_age_s = now - self.invalid_command_since
            if invalid_age_s >= self.invalid_command_grace_s:
                self.state = ActuationState.DISARMED
                self._publish_stop("command_invalid")
            else:
                self._publish_stop(
                    f"command_invalid_grace:{invalid_age_s:.3f}s"
                )
            return
        self.invalid_command_since = None

        try:
            servo_us = self.mapper.steering_to_pwm(
                self.latest_command.steering_angle_rad
            )
            target_esc_us = self.mapper.effort_to_target_pwm(
                self.latest_command.drive_effort
            )
            self.current_esc_us = self.mapper.apply_esc_slew(
                self.current_esc_us,
                target_esc_us,
                delta_s,
            )
        except ValueError as error:
            self.state = ActuationState.DISARMED
            self._publish_stop(f"mapping_error:{error}")
            return

        self._publish_pwm(servo_us, self.current_esc_us, "active")

    def send_stop_burst(self, count=10):
        for _ in range(count):
            try:
                self._publish_stop("shutdown")
                rclpy.spin_once(self, timeout_sec=0.0)
            except RCLError:
                # SIGINT can invalidate the ROS context between the ok() check
                # in main() and this final best-effort stop publication. The
                # MCU freshness timeout remains the independent application
                # level stop path in that case.
                break
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ActuationManager()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.state = ActuationState.DISARMED
            # SIGINT may already have invalidated the rclpy context. The stop
            # burst is therefore best effort; the MCU command/arm freshness
            # timeout is the independent application-level stop path.
            if rclpy.ok():
                node.send_stop_burst()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

#!/usr/bin/env python3
"""Block F: independent safety monitor.

Reads only /scan, /odometry/filtered, /control/nmpc and /mcu/arm. When at
least `min_points` returns fall into the brake zone in front of the bumper,
or the control command / scan goes stale while armed, it latches an E-stop:
  * publishes /safety/estop = true (latched),
  * calls /actuation/arm(false) (the service already exists in rc_actuation).
The latch is cleared only through the /safety/reset service.
"""

import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rc_interfaces.msg import ControlCommand
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from rc_safe_corridor.geometry import yaw_from_quaternion
from rc_safe_corridor.safety_zone import SafetyParams, evaluate_brake_zone
from rc_safe_corridor.scan_clustering import scan_to_points

try:
    import tf2_ros
except Exception:  # pragma: no cover
    tf2_ros = None


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__("safety_monitor")
        defaults = SafetyParams()
        declared = [
            ("scan_topic", "/scan"),
            ("odom_topic", "/odometry/filtered"),
            ("command_topic", "/control/nmpc"),
            ("arm_state_topic", "/mcu/arm"),
            ("disarm_service", "/actuation/arm"),
            ("base_frame", "base_link"),
            ("laser_frame", "laser"),
            ("use_tf", True),
            ("laser_x", -0.005),
            ("laser_y", 0.0),
            ("laser_yaw", 0.0),
            ("command_timeout_s", 0.2),
            ("scan_timeout_s", 0.6),
            ("odom_timeout_s", 0.5),
            ("watchdog_rate_hz", 20.0),
            ("enable_disarm", True),
            ("latch_when_disarmed", False),
        ]
        for name, value in vars(defaults).items():
            declared.append((f"zone.{name}", value))
        params = self.declare_parameters("", declared)
        self.cfg = {param.name: param.value for param in params}
        self.zone = SafetyParams(
            **{
                name: type(value)(self.cfg[f"zone.{name}"])
                for name, value in vars(defaults).items()
            }
        )
        self.laser_pose = (
            float(self.cfg["laser_x"]), float(self.cfg["laser_y"]), float(self.cfg["laser_yaw"])
        )
        self.tf_resolved = False
        self.tf_buffer = None
        if bool(self.cfg["use_tf"]) and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.speed = 0.0
        self.odom_time = None
        self.command_time = None
        self.command_valid = False
        self.scan_time = None
        self.armed = False
        self.latched = False
        self.latch_reason = ""
        self.last_count = 0
        self.last_zone = 0.0
        self.disarm_pending = False

        self.estop_publisher = self.create_publisher(Bool, "/safety/estop", LATCHED_QOS)
        self.zone_publisher = self.create_publisher(Bool, "/safety/zone_occupied", 5)
        self.status_publisher = self.create_publisher(String, "/safety/status", 5)
        self.disarm_client = self.create_client(SetBool, str(self.cfg["disarm_service"]))
        self.reset_service = self.create_service(Trigger, "/safety/reset", self._reset_callback)

        self.create_subscription(
            LaserScan, str(self.cfg["scan_topic"]), self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, str(self.cfg["odom_topic"]), self._odom_callback, 10
        )
        self.create_subscription(
            ControlCommand, str(self.cfg["command_topic"]), self._command_callback, 10
        )
        self.create_subscription(
            Bool, str(self.cfg["arm_state_topic"]), self._arm_callback, 10
        )
        self.timer = self.create_timer(
            1.0 / float(self.cfg["watchdog_rate_hz"]), self._watchdog
        )
        self._publish_estop()
        self.get_logger().info(
            "safety_monitor ready: a_brake=%.2f t_delay=%.2f d0=%.2f half_width=%.2f "
            "min_points=%d disarm=%s"
            % (
                self.zone.a_brake, self.zone.t_delay, self.zone.d0, self.zone.half_width,
                self.zone.min_points, bool(self.cfg["enable_disarm"]),
            )
        )

    # ------------------------------------------------------------ callbacks
    def _odom_callback(self, msg):
        self.speed = float(msg.twist.twist.linear.x)
        self.odom_time = time.monotonic()

    def _command_callback(self, msg):
        self.command_time = time.monotonic()
        self.command_valid = bool(msg.valid)

    def _arm_callback(self, msg):
        self.armed = bool(msg.data)

    def _reset_callback(self, _request, response):
        if self.latched:
            self.latched = False
            self.latch_reason = ""
            self.disarm_pending = False
            self._publish_estop()
            self.get_logger().warning("safety latch cleared by operator")
        response.success = True
        response.message = "safety latch cleared"
        return response

    def _resolve_laser_pose(self):
        if self.tf_resolved or self.tf_buffer is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.cfg["base_frame"]), str(self.cfg["laser_frame"]), Time()
            )
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        self.laser_pose = (float(t.x), float(t.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.tf_resolved = True

    def _scan_callback(self, msg):
        self.scan_time = time.monotonic()
        self._resolve_laser_pose()
        points = scan_to_points(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(float(msg.range_min), 0.05), float(msg.range_max), laser_pose=self.laser_pose,
        )
        odom_fresh = (
            self.odom_time is not None
            and time.monotonic() - self.odom_time <= float(self.cfg["odom_timeout_s"])
        )
        trigger, count, zone = evaluate_brake_zone(points.xy, self.speed, self.zone, odom_fresh)
        self.last_count = count
        self.last_zone = zone
        self.zone_publisher.publish(Bool(data=bool(trigger)))
        if trigger and (self.armed or bool(self.cfg["latch_when_disarmed"])):
            self._latch(f"brake_zone:{count}pts_in_{zone:.2f}m")

    # --------------------------------------------------------------- safety
    def _latch(self, reason):
        if not self.latched:
            self.latched = True
            self.latch_reason = reason
            self.get_logger().error(f"SAFETY STOP: {reason}")
            self._publish_estop()
        self.disarm_pending = bool(self.cfg["enable_disarm"])
        self._try_disarm()

    def _try_disarm(self):
        if not self.disarm_pending:
            return
        if not self.disarm_client.service_is_ready():
            self.get_logger().warning(
                "disarm service unavailable; operator must disarm", throttle_duration_sec=1.0
            )
            return
        request = SetBool.Request()
        request.data = False
        future = self.disarm_client.call_async(request)
        future.add_done_callback(self._disarm_done)
        self.disarm_pending = False

    def _disarm_done(self, future):
        try:
            result = future.result()
            self.get_logger().warning(f"disarm result: {result.message}")
        except Exception as error:  # pragma: no cover
            self.get_logger().error(f"disarm call failed: {error}")
            self.disarm_pending = True

    def _publish_estop(self):
        self.estop_publisher.publish(Bool(data=bool(self.latched)))

    def _watchdog(self):
        now = time.monotonic()
        if self.armed:
            if self.command_time is None or now - self.command_time > float(
                self.cfg["command_timeout_s"]
            ):
                self._latch("command_watchdog")
            if self.scan_time is None or now - self.scan_time > float(self.cfg["scan_timeout_s"]):
                self._latch("scan_stale")
        if self.latched:
            self._try_disarm()
            if self.armed:
                self._publish_estop()
        self.status_publisher.publish(
            String(
                data=(
                    f"latched={self.latched} reason={self.latch_reason or '-'} armed={self.armed} "
                    f"zone_len={self.last_zone:.2f} zone_pts={self.last_count} "
                    f"v={self.speed:.2f} tf={'tf' if self.tf_resolved else 'param'}"
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyMonitorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Convert /cmd_vel or NMPC commands into the Phase 4 ros2_control topics."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry


class AckermannCmdBridge(Node):
    """Ackermann command bridge for manual simulation and NMPC control."""

    def __init__(self) -> None:
        super().__init__('ackermann_cmd_bridge')

        # Vehicle Geometry Parameters
        # Defaults are for the 1/7 scale vehicle (rc_1_7.urdf.xacro).
        # Pass wheelbase/track_width/wheel_radius overrides from launch for 1/16.
        self.declare_parameter('wheelbase', 0.375)
        self.declare_parameter('track_width', 0.260)
        self.declare_parameter('wheel_radius', 0.049)
        self.declare_parameter('max_steering_angle', 0.61)
        self.declare_parameter('max_speed', 0.4)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # The Gazebo rear-wheel controller accepts a velocity setpoint, not
        # motor force. Map NMPC duty cycle directly to speed by default.
        # The old one-step physics conversion remains available for experiments.
        self.declare_parameter('nmpc_longitudinal_mode', 'duty_to_speed')
        self.declare_parameter('duty_to_speed_gain', 1.0)
        self.declare_parameter('max_acceleration', 0.5)

        # NMPC physics parameters (used only in "physics" mode).
        # Mass must match the URDF total_mass. rc_1_7.urdf.xacro: 6.5 kg.
        # Override at launch time if running the 1/16 model (2.5 kg).
        self.declare_parameter('mass', 6.5)
        self.declare_parameter('cm1', 0.28)
        self.declare_parameter('cm2', 0.05)
        self.declare_parameter('cr0', 0.006)
        self.declare_parameter('cr2', 0.011)
        self.declare_parameter('cr3', 5.0)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.nmpc_longitudinal_mode = str(
            self.get_parameter('nmpc_longitudinal_mode').value
        )
        self.duty_to_speed_gain = float(
            self.get_parameter('duty_to_speed_gain').value
        )
        self.max_acceleration = float(
            self.get_parameter('max_acceleration').value
        )

        self.mass = float(self.get_parameter('mass').value)
        self.cm1 = float(self.get_parameter('cm1').value)
        self.cm2 = float(self.get_parameter('cm2').value)
        self.cr0 = float(self.get_parameter('cr0').value)
        self.cr2 = float(self.get_parameter('cr2').value)
        self.cr3 = float(self.get_parameter('cr3').value)

        # State and Command variables
        self._last_cmd_time = 0.0
        self._last_nmpc_time = 0.0
        self._current_v_state = 0.0
        
        # Targets for Publish Timer
        self._target_v_cmd = 0.0
        self._target_delta_cmd = 0.0
        self._published_v_cmd = 0.0
        self._timed_out = True

        # Subscribers
        self._cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._nmpc_sub = self.create_subscription(Float64MultiArray, '/nmpc_cmd', self._on_nmpc_cmd, 1)
        self._odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 10
        )

        # Publishers
        self._steering_pub = self.create_publisher(
            Float64MultiArray,
            '/front_steering_position_controller/commands',
            10,
        )
        self._rear_wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/rear_wheel_velocity_controller/commands',
            10,
        )

        self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f'Ackermann bridge started | NMPC mode={self.nmpc_longitudinal_mode} | '
            f'duty gain={self.duty_to_speed_gain:.2f} m/s | '
            f'max accel={self.max_acceleration:.2f} m/s² | odom={self.odom_topic}'
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Update current vehicle state velocity."""
        self._current_v_state = msg.twist.twist.linear.x

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Fallback for manual Twist commands."""
        self._target_v_cmd = self._clamp(float(msg.linear.x), -self.max_speed, self.max_speed)
        # For Twist, we still need to approximate delta if we want to use the same logic
        # radius = v / omega -> delta = atan(L/radius) = atan(L*omega / v)
        if abs(msg.linear.x) > 0.001:
            self._target_delta_cmd = math.atan(self.wheelbase * msg.angular.z / msg.linear.x)
        else:
            self._target_delta_cmd = 0.0
        
        self._last_cmd_time = time.monotonic()
        self._timed_out = False

    def _on_nmpc_cmd(self, msg: Float64MultiArray) -> None:
        """Handle NMPC commands: [steering_angle, duty_cycle]."""
        if len(msg.data) < 2:
            return

        delta = float(msg.data[0])
        duty_cycle = float(msg.data[1])

        now = time.monotonic()

        if self.nmpc_longitudinal_mode == 'physics':
            dt = now - self._last_nmpc_time if self._last_nmpc_time > 0 else 0.02
            self._last_nmpc_time = now

            v = self._current_v_state
            fdx = (
                (self.cm1 - self.cm2 * v) * duty_cycle
                - self.cr2 * (v**2)
                - self.cr0 * math.tanh(self.cr3 * v)
            )
            v_cmd = v + (fdx / self.mass) * dt
        else:
            # Gazebo uses an ideal velocity controller. Treat D as the
            # normalized longitudinal command and convert it to m/s.
            v_cmd = duty_cycle * self.duty_to_speed_gain

        self._target_v_cmd = self._clamp(v_cmd, -self.max_speed, self.max_speed)
        self._target_delta_cmd = self._clamp(delta, -self.max_steering_angle, self.max_steering_angle)
        
        self._last_cmd_time = now
        self._timed_out = False

    def _on_timer(self) -> None:
        """Publish commands at fixed rate."""
        if time.monotonic() - self._last_cmd_time > self.cmd_timeout:
            v = 0.0
            delta = 0.0
            self._published_v_cmd = 0.0
            if not self._timed_out:
                self.get_logger().info('Command timeout: stopping vehicle')
            self._timed_out = True
        else:
            if abs(self._target_v_cmd) < 1e-4:
                # A zero command is a safety stop, so do not ramp it down.
                self._published_v_cmd = 0.0
            else:
                max_step = self.max_acceleration / self.publish_rate
                speed_error = self._target_v_cmd - self._published_v_cmd
                self._published_v_cmd += self._clamp(
                    speed_error, -max_step, max_step
                )
            v = self._published_v_cmd
            delta = self._target_delta_cmd

        steering_left, steering_right, rear_left_vel, rear_right_vel = self.compute_ackermann(v, delta)

        self._steering_pub.publish(Float64MultiArray(data=[steering_left, steering_right]))
        self._rear_wheel_pub.publish(Float64MultiArray(data=[rear_left_vel, rear_right_vel]))

    def compute_ackermann(self, v_cmd: float, delta: float):
        """Compute wheel-specific commands using direct Ackermann geometry."""
        L = self.wheelbase
        W = self.track_width
        r = self.wheel_radius

        if abs(v_cmd) < 1e-4:
            return 0.0, 0.0, 0.0, 0.0

        # Straight line case
        if abs(delta) < 1e-4:
            wheel_omega = v_cmd / r
            return 0.0, 0.0, wheel_omega, wheel_omega

        # Front Steering Angles
        # tan(delta_l) = L / (R - W/2), where R = L / tan(delta)
        # -> delta_l = atan( L * tan(delta) / (L - W/2 * tan(delta)) )
        tan_delta = math.tan(delta)
        steering_left = math.atan(L * tan_delta / (L - (W / 2.0) * tan_delta))
        steering_right = math.atan(L * tan_delta / (L + (W / 2.0) * tan_delta))

        # Rear Wheel Velocities (Differential)
        # v_l = v_cmd * (R - W/2) / R = v_cmd * (1 - W/2R) = v_cmd * (1 - W * tan(delta) / 2L)
        v_left = v_cmd * (1.0 - (W * tan_delta) / (2.0 * L))
        v_right = v_cmd * (1.0 + (W * tan_delta) / (2.0 * L))

        return steering_left, steering_right, v_left / r, v_right / r

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannCmdBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

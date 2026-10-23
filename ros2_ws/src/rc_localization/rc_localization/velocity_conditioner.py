#!/usr/bin/env python3

from copy import deepcopy
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from rc_localization.velocity_filter import (
    PlanarPoseSpeedEstimator,
    RobustVelocityFilter,
    VelocitySampleError,
)


class VelocityConditioner(Node):
    def __init__(self):
        super().__init__("rf2o_velocity_conditioner")
        self.declare_parameter("input_topic", "/odometry/rf2o_raw")
        self.declare_parameter("output_topic", "/odometry/rf2o")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("median_window", 3)
        self.declare_parameter("ema_alpha", 0.45)
        self.declare_parameter("use_pose_delta_speed", True)
        self.declare_parameter("pose_direction_epsilon_mps", 0.0001)
        self.declare_parameter("zero_threshold_mps", 0.006)
        self.declare_parameter("max_abs_speed_mps", 1.5)
        self.declare_parameter("max_acceleration_mps2", 3.0)
        self.declare_parameter("reset_gap_s", 0.5)
        self.declare_parameter("variance_window", 30)
        self.declare_parameter("variance_floor", 0.000025)
        self.declare_parameter("stale_timeout_s", 0.3)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        if self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive")

        self.use_pose_delta_speed = bool(
            self.get_parameter("use_pose_delta_speed").value
        )
        reset_gap_s = float(self.get_parameter("reset_gap_s").value)
        self.pose_speed_estimator = PlanarPoseSpeedEstimator(
            reset_gap_s=reset_gap_s,
            direction_epsilon_mps=float(
                self.get_parameter("pose_direction_epsilon_mps").value
            ),
        )
        self.filter = RobustVelocityFilter(
            median_window=int(self.get_parameter("median_window").value),
            ema_alpha=float(self.get_parameter("ema_alpha").value),
            zero_threshold_mps=float(
                self.get_parameter("zero_threshold_mps").value
            ),
            max_abs_speed_mps=float(
                self.get_parameter("max_abs_speed_mps").value
            ),
            max_acceleration_mps2=float(
                self.get_parameter("max_acceleration_mps2").value
            ),
            reset_gap_s=reset_gap_s,
            variance_window=int(
                self.get_parameter("variance_window").value
            ),
            variance_floor=float(
                self.get_parameter("variance_floor").value
            ),
        )

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )
        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self._odometry_callback,
            10,
        )
        self.diagnostics_timer = self.create_timer(
            0.5,
            self._publish_diagnostics,
        )

        self.last_receive_monotonic = None
        self.last_raw_mps = 0.0
        self.last_estimated_mps = 0.0
        self.last_filtered_mps = 0.0
        self.last_variance = self.filter.variance_floor
        self.accepted_count = 0
        self.rejected_count = 0
        self.last_rejection = ""

        self.get_logger().info(
            f"Conditioning {self.input_topic} -> {self.output_topic}; "
            f"source={'pose_delta' if self.use_pose_delta_speed else 'raw_vx'}, "
            "median=3, EMA enabled, non-zero covariance"
        )

    @staticmethod
    def _stamp_seconds(message: Odometry) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _large_diagonal_covariance(valid_x_variance=None):
        covariance = [0.0] * 36
        diagonal = [1.0e3] * 6
        if valid_x_variance is not None:
            diagonal[0] = float(valid_x_variance)
        for index, value in enumerate(diagonal):
            covariance[index * 6 + index] = value
        return covariance

    def _odometry_callback(self, message: Odometry):
        stamp_s = self._stamp_seconds(message)
        replacement_stamp = None
        if stamp_s == 0.0:
            now = self.get_clock().now()
            stamp_s = now.nanoseconds * 1e-9
            replacement_stamp = now.to_msg()
        raw_mps = float(message.twist.twist.linear.x)

        try:
            estimated_mps = (
                self.pose_speed_estimator.update(
                    x_m=message.pose.pose.position.x,
                    y_m=message.pose.pose.position.y,
                    raw_vx_mps=raw_mps,
                    stamp_s=stamp_s,
                )
                if self.use_pose_delta_speed
                else raw_mps
            )
            filtered_mps, variance = self.filter.update(
                estimated_mps,
                stamp_s,
            )
        except VelocitySampleError as error:
            self.rejected_count += 1
            self.last_rejection = str(error)
            self.get_logger().warning(
                f"Rejected RF2O vx={raw_mps:.3f} m/s: {error}",
                throttle_duration_sec=1.0,
            )
            return

        output = deepcopy(message)
        if replacement_stamp is not None:
            output.header.stamp = replacement_stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        output.twist.twist.linear.x = filtered_mps
        output.twist.twist.linear.y = 0.0
        output.twist.twist.linear.z = 0.0
        output.twist.twist.angular.x = 0.0
        output.twist.twist.angular.y = 0.0
        output.twist.twist.angular.z = 0.0
        output.pose.covariance = self._large_diagonal_covariance()
        output.twist.covariance = self._large_diagonal_covariance(variance)
        self.publisher.publish(output)

        self.last_receive_monotonic = time.monotonic()
        self.last_raw_mps = raw_mps
        self.last_estimated_mps = estimated_mps
        self.last_filtered_mps = filtered_mps
        self.last_variance = variance
        self.accepted_count += 1

    def _publish_diagnostics(self):
        now = time.monotonic()
        age_s = (
            float("inf")
            if self.last_receive_monotonic is None
            else now - self.last_receive_monotonic
        )

        status = DiagnosticStatus()
        status.name = "rc_localization/rf2o_velocity"
        status.hardware_id = "sllidar_a1m8_rf2o"
        if self.last_receive_monotonic is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "No accepted RF2O odometry"
        elif age_s > self.stale_timeout_s:
            status.level = DiagnosticStatus.ERROR
            status.message = f"RF2O velocity stale ({age_s:.3f}s)"
        elif self.rejected_count > 0 and self.last_rejection:
            status.level = DiagnosticStatus.WARN
            status.message = f"Running; last rejection: {self.last_rejection}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "RF2O velocity healthy"

        values = {
            "age_s": age_s,
            "raw_vx_mps": self.last_raw_mps,
            "pose_delta_vx_mps": self.last_estimated_mps,
            "filtered_vx_mps": self.last_filtered_mps,
            "vx_variance": self.last_variance,
            "accepted_count": self.accepted_count,
            "rejected_count": self.rejected_count,
        }
        status.values = [
            KeyValue(key=key, value=str(value))
            for key, value in values.items()
        ]

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostics_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityConditioner()
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

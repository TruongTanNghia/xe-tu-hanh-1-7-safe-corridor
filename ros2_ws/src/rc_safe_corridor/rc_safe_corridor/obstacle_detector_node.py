#!/usr/bin/env python3
"""Block A: LiDAR obstacle detector + tracker (ROS 2 node).

/scan (+ TF laser->base_link, /lane/reference, /odometry/filtered)
  -> /perception/obstacles (rc_interfaces/ObstacleArray, base_link)
  -> /perception/obstacle_markers (visualization_msgs/MarkerArray)
  -> /perception/obstacle_status (std_msgs/String)
"""

import math
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rc_interfaces.msg import LaneReference, Obstacle, ObstacleArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from rc_safe_corridor.frenet import build_reference_path
from rc_safe_corridor.geometry import (
    inverse_transform_points,
    rotate_vectors,
    transform_points,
    yaw_from_quaternion,
)
from rc_safe_corridor.markers import obstacle_markers
from rc_safe_corridor.scan_clustering import (
    apply_lane_gate,
    apply_roi,
    detect_obstacles,
    scan_to_points,
)
from rc_safe_corridor.tracking import MultiObjectTracker, TrackerParams

try:
    import tf2_ros
except Exception:  # pragma: no cover
    tf2_ros = None


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__("obstacle_detector")
        p = self.declare_parameters(
            "",
            [
                ("scan_topic", "/scan"),
                ("lane_topic", "/lane/reference"),
                ("odom_topic", "/odometry/filtered"),
                ("base_frame", "base_link"),
                ("laser_frame", "laser"),
                ("use_tf", True),
                ("laser_x", -0.005),
                ("laser_y", 0.0),
                ("laser_yaw", 0.0),
                ("roi_x_min", 0.12),
                ("roi_x_max", 2.5),
                ("roi_y_abs_max", 1.0),
                ("body_x_min", -0.40),
                ("body_x_max", 0.40),
                ("body_y_abs", 0.18),
                ("lane_gate_enabled", True),
                ("lane_half_width", 0.45),
                ("lane_gate_margin", 0.15),
                ("lane_timeout_s", 0.5),
                ("cluster_lambda_deg", 10.0),
                ("cluster_sigma_r", 0.01),
                ("cluster_min_points", 3),
                ("cluster_max_extent_m", 1.2),
                ("small_object_max_m", 0.12),
                ("min_radius_m", 0.03),
                ("tracking_frame", "odom"),
                ("odom_timeout_s", 0.5),
                ("track_process_noise_accel", 1.5),
                ("track_measurement_noise_m", 0.03),
                ("track_gate_distance_m", 0.5),
                ("track_confirm_hits", 3),
                ("track_max_misses", 5),
                ("track_max_coast_s", 0.8),
                ("dynamic_speed_on", 0.15),
                ("dynamic_speed_off", 0.08),
                ("publish_markers", True),
            ],
        )
        self.cfg = {param.name: param.value for param in p}
        self.tracker = MultiObjectTracker(
            TrackerParams(
                process_noise_accel=float(self.cfg["track_process_noise_accel"]),
                measurement_noise_m=float(self.cfg["track_measurement_noise_m"]),
                gate_distance_m=float(self.cfg["track_gate_distance_m"]),
                confirm_hits=int(self.cfg["track_confirm_hits"]),
                max_misses=int(self.cfg["track_max_misses"]),
                max_coast_s=float(self.cfg["track_max_coast_s"]),
                dynamic_speed_on=float(self.cfg["dynamic_speed_on"]),
                dynamic_speed_off=float(self.cfg["dynamic_speed_off"]),
            )
        )
        self.laser_pose = (
            float(self.cfg["laser_x"]),
            float(self.cfg["laser_y"]),
            float(self.cfg["laser_yaw"]),
        )
        self.tf_resolved = False
        self.tf_buffer = None
        if bool(self.cfg["use_tf"]) and tf2_ros is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.odom_pose = None          # (x, y, yaw)
        self.odom_time = None
        self.lane_path_xy = None
        self.lane_time = None
        self.last_scan_time = None
        self.active_tracking_frame = str(self.cfg["base_frame"])

        self.obstacle_publisher = self.create_publisher(
            ObstacleArray, "/perception/obstacles", 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/perception/obstacle_markers", 5
        )
        self.status_publisher = self.create_publisher(
            String, "/perception/obstacle_status", 5
        )
        self.create_subscription(
            LaserScan, str(self.cfg["scan_topic"]), self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaneReference, str(self.cfg["lane_topic"]), self._lane_callback, 10
        )
        self.create_subscription(
            Odometry, str(self.cfg["odom_topic"]), self._odom_callback, 10
        )
        self.get_logger().info(
            "obstacle_detector ready: ROI x=[%.2f, %.2f] |y|<=%.2f, lane gate=%s, "
            "tracking frame=%s"
            % (
                float(self.cfg["roi_x_min"]), float(self.cfg["roi_x_max"]),
                float(self.cfg["roi_y_abs_max"]), bool(self.cfg["lane_gate_enabled"]),
                str(self.cfg["tracking_frame"]),
            )
        )

    # ------------------------------------------------------------ callbacks
    def _odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        self.odom_time = time.monotonic()

    def _lane_callback(self, msg):
        if msg.status != LaneReference.STATUS_DETECTED:
            return
        try:
            path = build_reference_path(
                msg.s_m, msg.curvature_inv_m, msg.cte_m, msg.heading_error_rad,
                evaluation_forward=msg.evaluation_forward_m,
                path_lateral=list(msg.path_lateral_m) if len(msg.path_lateral_m) else None,
                horizon_m=float(self.cfg["roi_x_max"]) + 0.5,
            )
        except ValueError:
            return
        self.lane_path_xy = path.xy
        self.lane_time = time.monotonic()

    def _resolve_laser_pose(self):
        if self.tf_resolved or self.tf_buffer is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                str(self.cfg["base_frame"]), str(self.cfg["laser_frame"]),
                Time(),
            )
        except Exception:
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        self.laser_pose = (float(t.x), float(t.y), yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self.tf_resolved = True
        self.get_logger().info(
            "laser pose from TF: x=%.3f y=%.3f yaw=%.3f" % self.laser_pose
        )

    def _scan_callback(self, msg):
        started = time.perf_counter()
        now = time.monotonic()
        dt = 0.1 if self.last_scan_time is None else max(now - self.last_scan_time, 1e-3)
        self.last_scan_time = now
        self._resolve_laser_pose()

        points = scan_to_points(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(float(msg.range_min), 0.05), float(msg.range_max),
            laser_pose=self.laser_pose,
        )
        points = apply_roi(
            points,
            x_min=float(self.cfg["roi_x_min"]), x_max=float(self.cfg["roi_x_max"]),
            y_abs_max=float(self.cfg["roi_y_abs_max"]),
            body_x_min=float(self.cfg["body_x_min"]), body_x_max=float(self.cfg["body_x_max"]),
            body_y_abs=float(self.cfg["body_y_abs"]),
        )
        lane_fresh = (
            self.lane_time is not None
            and now - self.lane_time <= float(self.cfg["lane_timeout_s"])
        )
        if bool(self.cfg["lane_gate_enabled"]) and lane_fresh:
            points = apply_lane_gate(
                points, self.lane_path_xy,
                float(self.cfg["lane_half_width"]), float(self.cfg["lane_gate_margin"]),
            )
        detections = detect_obstacles(
            points, msg.angle_increment,
            lambda_deg=float(self.cfg["cluster_lambda_deg"]),
            sigma_r=float(self.cfg["cluster_sigma_r"]),
            min_points=int(self.cfg["cluster_min_points"]),
            max_cluster_extent_m=float(self.cfg["cluster_max_extent_m"]),
            small_object_max_m=float(self.cfg["small_object_max_m"]),
            min_radius_m=float(self.cfg["min_radius_m"]),
        )

        # Tracking frame: odom when fresh, otherwise base_link (static ego).
        odom_fresh = (
            self.odom_pose is not None
            and self.odom_time is not None
            and now - self.odom_time <= float(self.cfg["odom_timeout_s"])
        )
        use_odom = str(self.cfg["tracking_frame"]) == "odom" and odom_fresh
        frame = "odom" if use_odom else str(self.cfg["base_frame"])
        if frame != self.active_tracking_frame:
            self.tracker.reset()
            self.active_tracking_frame = frame
            self.get_logger().warning(f"tracking frame switched to {frame}")
        if use_odom:
            ox, oy, oyaw = self.odom_pose
            for det in detections:
                det.center = transform_points(det.center, ox, oy, oyaw)[0]
                det.yaw = det.yaw + oyaw
        tracks = self.tracker.update(detections, dt)

        array = ObstacleArray()
        array.header.stamp = msg.header.stamp
        array.header.frame_id = str(self.cfg["base_frame"])
        array.tracking_frame = frame
        array.scan_point_count = int(points.xy.shape[0])
        array.cluster_count = len(detections)
        for track in tracks:
            position = track.position
            velocity = track.velocity
            yaw = float(track.shape.yaw)
            if use_odom:
                ox, oy, oyaw = self.odom_pose
                position = inverse_transform_points(position, ox, oy, oyaw)[0]
                velocity = rotate_vectors(velocity, -oyaw)[0]
                yaw -= oyaw
            obstacle = Obstacle()
            obstacle.id = int(track.track_id)
            obstacle.x = float(position[0])
            obstacle.y = float(position[1])
            obstacle.yaw = float(math.atan2(math.sin(yaw), math.cos(yaw)))
            lengths = np.array(track.shape_history)
            obstacle.length = float(lengths[:, 0].max())
            obstacle.width = float(lengths[:, 1].max())
            obstacle.radius = float(lengths[:, 2].max())
            obstacle.vx = float(velocity[0])
            obstacle.vy = float(velocity[1])
            cov = track.P[:2, :2]
            obstacle.position_covariance = [
                float(cov[0, 0]), float(cov[0, 1]), float(cov[1, 0]), float(cov[1, 1])
            ]
            obstacle.is_dynamic = bool(track.is_dynamic)
            obstacle.confirmed = bool(track.confirmed)
            obstacle.confidence = float(track.confidence)
            obstacle.shape = (
                Obstacle.SHAPE_BOX if track.shape.shape == "box" else Obstacle.SHAPE_CIRCLE
            )
            obstacle.point_count = int(track.shape.point_count)
            obstacle.age_frames = int(track.age)
            array.obstacles.append(obstacle)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        array.processing_time_ms = float(elapsed_ms)
        self.obstacle_publisher.publish(array)
        if bool(self.cfg["publish_markers"]):
            self.marker_publisher.publish(
                obstacle_markers(array.obstacles, array.header.frame_id, msg.header.stamp)
            )
        self.status_publisher.publish(
            String(
                data=(
                    f"points={array.scan_point_count} clusters={array.cluster_count} "
                    f"tracks={len(array.obstacles)} frame={frame} "
                    f"lane_gate={'on' if lane_fresh and bool(self.cfg['lane_gate_enabled']) else 'off'} "
                    f"tf={'tf' if self.tf_resolved else 'param'} "
                    f"proc_ms={elapsed_ms:.1f}"
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleDetectorNode()
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

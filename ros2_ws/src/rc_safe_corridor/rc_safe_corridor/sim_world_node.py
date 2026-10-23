#!/usr/bin/env python3
"""Lightweight simulator so the Safe Corridor stack can run in ROS 2 without
the car, the camera or acados.

It simulates:
  * an oval track (two straights + two R0.925 half circles, lane 0.45 m),
  * a kinematic bicycle ego vehicle driven by a simple controller that
    follows /planning/corridor (d_ref, v_max, v_ref),
  * obstacles placed in (s, d) along the track (static or moving along s),
  * RPLIDAR A1 scans by ray casting (/scan), the lane detector output
    (/lane/reference, same fields as the camera pipeline),
    /odometry/filtered and TF odom->base_link->laser,
  * ground-truth markers for RViz2 (/sim/track_markers).

    ros2 run rc_safe_corridor sim_world --ros-args -p scenario:=tour
    scenarios: clear | right | left | slalom | curve | block | lead | mixed | tour
"""

import math
import time

from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rc_interfaces.msg import Corridor, LaneReference
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from rc_safe_corridor.geometry import inverse_transform_points, wrap_angle
from rc_safe_corridor.sim_track import SCENARIOS, OvalTrack, advance_obstacles, obstacle_active
from rc_safe_corridor.synthetic import SimObstacle, synthetic_scan

try:
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
except Exception:  # pragma: no cover
    StaticTransformBroadcaster = TransformBroadcaster = None


LATCHED_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)

class SimWorldNode(Node):
    def __init__(self):
        super().__init__("sim_world")
        params = self.declare_parameters(
            "",
            [
                ("scenario", "slalom"),
                ("rate_hz", 20.0),
                ("scan_rate_hz", 10.0),
                ("lane_rate_hz", 20.0),
                ("straight_m", 3.5),
                ("radius_m", 0.925),
                ("wheelbase_m", 0.375),
                ("max_steer_rad", 0.52),
                ("max_steer_rate_radps", 2.5),
                ("accel_mps2", 1.0),
                ("decel_mps2", 1.5),
                ("lookahead_m", 0.5),
                ("heading_gain", 1.8),
                ("lane_first_sample_m", 0.35),
                ("lane_samples", 12),
                ("lane_spacing_m", 0.09),
                ("corridor_timeout_s", 0.5),
                ("laser_x", -0.005),
                ("laser_z", 0.19),
                ("scan_noise_m", 0.005),
                ("start_s", 0.0),
                ("loop", True),
            ],
        )
        self.cfg = {p.name: p.value for p in params}
        self.track = OvalTrack(float(self.cfg["straight_m"]), float(self.cfg["radius_m"]))
        scenario = str(self.cfg["scenario"])
        if scenario not in SCENARIOS:
            raise ValueError(f"scenario must be one of {sorted(SCENARIOS)}")
        self.obstacles = [dict(o) for o in SCENARIOS[scenario]]

        # Ego state in odom.
        x, y, h, _k = self.track.pose(float(self.cfg["start_s"]))
        self.x, self.y, self.yaw = x, y, h
        self.v = 0.0
        self.steer = 0.0
        self.corridor = None
        self.corridor_time = None
        self.laps = 0
        self.last_s = 0.0
        self.t_sim = 0.0

        self.scan_publisher = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.lane_publisher = self.create_publisher(LaneReference, "/lane/reference", 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.track_publisher = self.create_publisher(MarkerArray, "/sim/track_markers", LATCHED_QOS)
        self.status_publisher = self.create_publisher(String, "/sim/status", 5)
        self.create_subscription(Corridor, "/planning/corridor", self._corridor_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if TransformBroadcaster else None
        if StaticTransformBroadcaster:
            self.static_broadcaster = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "base_link"
            t.child_frame_id = "laser"
            t.transform.translation.x = float(self.cfg["laser_x"])
            t.transform.translation.z = float(self.cfg["laser_z"])
            t.transform.rotation.w = 1.0
            self.static_broadcaster.sendTransform(t)

        self.dt = 1.0 / float(self.cfg["rate_hz"])
        self.create_timer(self.dt, self._step)
        self.create_timer(1.0 / float(self.cfg["scan_rate_hz"]), self._publish_scan)
        self.create_timer(1.0 / float(self.cfg["lane_rate_hz"]), self._publish_lane)
        self.create_timer(1.0, self._publish_track_markers)
        self._publish_track_markers()
        self.get_logger().info(
            f"sim_world ready: scenario={scenario} obstacles={len(self.obstacles)} "
            f"track={self.track.length:.2f} m"
        )

    # ------------------------------------------------------------ callbacks
    def _corridor_callback(self, msg):
        self.corridor = msg
        self.corridor_time = time.monotonic()

    # -------------------------------------------------------------- physics
    def _obstacle_pose(self, obs):
        x, y, h = self.track.frenet_to_xy(obs["s"], obs["d"])
        return x, y, h

    def _active(self, obs):
        return obstacle_active(obs, self.t_sim)

    def _step(self):
        dt = self.dt
        self.t_sim += dt
        advance_obstacles(self.obstacles, dt, self.t_sim, self.track, ego_xy=(self.x, self.y))

        # Controller from the corridor (d_ref / v_max / v_ref).
        fresh = (
            self.corridor is not None
            and self.corridor_time is not None
            and time.monotonic() - self.corridor_time <= float(self.cfg["corridor_timeout_s"])
            and len(self.corridor.s) >= 2
        )
        s_ego, d_ego, h_path, kappa = self.track.project(self.x, self.y)
        e_psi = wrap_angle(self.yaw - h_path)
        lookahead = float(self.cfg["lookahead_m"])
        if fresh:
            c = self.corridor
            d_target = float(np.interp(lookahead, c.s, c.d_ref))
            v_target = min(float(c.v_ref), float(np.interp(0.3, c.s, c.v_max)))
            if not c.feasible and c.blocked_at_s >= 0.0:
                v_target = min(v_target, float(np.interp(0.0, c.s, c.v_max)))
        else:
            d_target = d_ego
            v_target = 0.0
        psi_des = math.atan2(d_target - d_ego, lookahead)
        steer_cmd = float(self.cfg["heading_gain"]) * (psi_des - e_psi)
        steer_cmd += math.atan(kappa * float(self.cfg["wheelbase_m"]))
        max_steer = float(self.cfg["max_steer_rad"])
        steer_cmd = max(-max_steer, min(max_steer, steer_cmd))
        max_rate = float(self.cfg["max_steer_rate_radps"]) * dt
        self.steer += max(-max_rate, min(max_rate, steer_cmd - self.steer))
        if v_target > self.v:
            self.v = min(v_target, self.v + float(self.cfg["accel_mps2"]) * dt)
        else:
            self.v = max(v_target, self.v - float(self.cfg["decel_mps2"]) * dt)

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(
            self.yaw + self.v / float(self.cfg["wheelbase_m"]) * math.tan(self.steer) * dt
        )
        if s_ego < self.last_s - self.track.length * 0.5:
            self.laps += 1
        self.last_s = s_ego

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        odom.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.v / float(self.cfg["wheelbase_m"]) * math.tan(self.steer)
        self.odom_publisher.publish(odom)
        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = odom.pose.pose.orientation.z
            t.transform.rotation.w = odom.pose.pose.orientation.w
            self.tf_broadcaster.sendTransform(t)
        self.status_publisher.publish(
            String(
                data=(
                    f"s={s_ego:.2f} d={d_ego:+.3f} v={self.v:.2f} steer={math.degrees(self.steer):+.1f}deg "
                    f"corridor={'ok' if fresh else 'missing'} laps={self.laps} t={self.t_sim:.0f}s"
                )
            )
        )

    # ------------------------------------------------------------- sensors
    def _obstacles_in_base_link(self):
        result = []
        for obs in self.obstacles:
            if not self._active(obs):
                continue
            ox, oy, oh = self._obstacle_pose(obs)
            local = inverse_transform_points(np.array([[ox, oy]]), self.x, self.y, self.yaw)[0]
            result.append(
                SimObstacle(
                    x=float(local[0]), y=float(local[1]), radius=obs.get("radius", 0.05),
                    length=obs.get("length", 0.0), width=obs.get("width", 0.0),
                    yaw=wrap_angle(oh - self.yaw),
                )
            )
        return result

    def _publish_scan(self):
        laser_pose = (float(self.cfg["laser_x"]), 0.0, 0.0)
        ranges, angle_min, inc = synthetic_scan(
            self._obstacles_in_base_link(), laser_pose=laser_pose,
            noise_std=float(self.cfg["scan_noise_m"]),
            rng=np.random.default_rng(),
        )
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = float(angle_min)
        msg.angle_max = float(angle_min + inc * (len(ranges) - 1))
        msg.angle_increment = float(inc)
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / float(self.cfg["scan_rate_hz"])
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.ranges = [float(r) if math.isfinite(r) else float("inf") for r in ranges]
        self.scan_publisher.publish(msg)

    def _publish_lane(self):
        """Emulate the camera lane detector output in base_link."""
        s_ego, _d, _h, _k = self.track.project(self.x, self.y)
        n = int(self.cfg["lane_samples"])
        first = float(self.cfg["lane_first_sample_m"])
        spacing = float(self.cfg["lane_spacing_m"])
        points = []
        kappas = []
        for i in range(n):
            s = s_ego + first + i * spacing
            px, py, _h, k = self.track.pose(s)
            points.append([px, py])
            kappas.append(k)
        local = inverse_transform_points(np.array(points), self.x, self.y, self.yaw)
        forward = local[:, 0]
        lateral = local[:, 1]
        seg = np.hypot(np.diff(forward), np.diff(lateral))
        s_m = np.concatenate([[0.0], np.cumsum(seg)])
        _px, _py, h0, _k0 = self.track.pose(s_ego + first)
        msg = LaneReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.s_m = [float(v) for v in s_m]
        msg.curvature_inv_m = [float(v) for v in kappas]
        msg.cte_m = float(-lateral[0])
        msg.heading_error_rad = float(-wrap_angle(h0 - self.yaw))
        msg.confidence = 1.0
        msg.status = LaneReference.STATUS_DETECTED
        msg.evaluation_forward_m = float(forward[0])
        msg.path_lateral_m = [float(v) for v in lateral]
        self.lane_publisher.publish(msg)

    # -------------------------------------------------------------- markers
    def _publish_track_markers(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()

        def line(marker_id, pts, r, g, b, width=0.02, ns="track"):
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = width
            m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(x), y=float(y), z=0.0) for x, y in pts]
            return m

        s_values = np.arange(0.0, self.track.length + 0.05, 0.05)
        for marker_id, d, color in ((0, 0.45, (0.95, 0.77, 0.1)), (1, -0.45, (0.95, 0.77, 0.1)),
                                    (2, 0.0, (0.4, 0.4, 0.45))):
            pts = [self.track.frenet_to_xy(s, d)[:2] for s in s_values]
            array.markers.append(line(marker_id, pts, *color, width=0.03 if d else 0.01))
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.ns = "truth_obstacles"
        array.markers.append(clear)
        for i, obs in enumerate(self.obstacles):
            if not self._active(obs):
                continue
            ox, oy, oh = self._obstacle_pose(obs)
            m = Marker()
            m.header.frame_id = "odom"
            m.header.stamp = stamp
            m.ns = "truth_obstacles"
            m.id = i
            m.action = Marker.ADD
            if obs.get("length", 0.0) > 0.0:
                m.type = Marker.CUBE
                m.scale.x = obs["length"]
                m.scale.y = obs["width"]
                m.pose.orientation.z = math.sin(0.5 * oh)
                m.pose.orientation.w = math.cos(0.5 * oh)
            else:
                m.type = Marker.CYLINDER
                m.scale.x = m.scale.y = 2.0 * obs.get("radius", 0.05)
                m.pose.orientation.w = 1.0
            m.scale.z = 0.25
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.125
            m.color = ColorRGBA(r=0.6, g=0.6, b=0.65, a=0.5)
            array.markers.append(m)
        self.track_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimWorldNode()
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

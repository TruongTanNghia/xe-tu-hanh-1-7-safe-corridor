#!/usr/bin/env python3
"""Glue between Gazebo Classic and the Safe Corridor stack.

* /ground_truth/odom (p3d, world frame)  ->  /odometry/filtered + TF odom->base_footprint
* track geometry + ego pose               ->  /lane/reference (same fields as the
                                               camera lane pipeline; replace with
                                               image_processing for a camera test)
* scenario timeline                       ->  moves dynamic/timed obstacles through
                                               /gazebo/set_entity_state
* /sim/track_markers, /sim/status for RViz2 and the terminal.

LiDAR is NOT faked here: /scan comes from the Gazebo ray sensor on the car.
"""

import copy
import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rc_interfaces.msg import LaneReference
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from rc_gazebo_corridor.world_builder import OBSTACLE_HEIGHT, PARK_X
from rc_safe_corridor.geometry import inverse_transform_points, wrap_angle, yaw_from_quaternion
from rc_safe_corridor.sim_track import (
    SCENARIOS,
    OvalTrack,
    advance_obstacles,
    is_triggered,
    obstacle_active,
)

try:
    from tf2_ros import TransformBroadcaster
except Exception:  # pragma: no cover
    TransformBroadcaster = None


LATCHED_QOS = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


class GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__("gazebo_corridor_bridge")
        params = self.declare_parameters(
            "",
            [
                ("scenario", "tour"),
                ("lane_rate_hz", 20.0),
                ("mover_rate_hz", 20.0),
                ("lane_first_sample_m", 0.35),
                ("lane_samples", 12),
                ("lane_spacing_m", 0.09),
                ("odom_frame", "odom"),
                ("base_frame", "base_footprint"),
                ("publish_tf", True),
            ],
        )
        self.cfg = {p.name: p.value for p in params}
        scenario = str(self.cfg["scenario"])
        if scenario not in SCENARIOS:
            raise ValueError(f"scenario must be one of {sorted(SCENARIOS)}")
        self.track = OvalTrack()
        self.obstacles = copy.deepcopy(SCENARIOS[scenario])
        self.moving = [
            i for i, o in enumerate(self.obstacles)
            if o.get("speed") or o.get("lat_speed") or "appear_t" in o or "vanish_t" in o
            or is_triggered(o)
        ]
        self.pose = None             # (x, y, yaw) of base_footprint in world
        self.speed = 0.0
        self.t0 = None
        self.last_mover = None
        self.pending = 0

        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self.lane_pub = self.create_publisher(LaneReference, "/lane/reference", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/sim/track_markers", LATCHED_QOS)
        self.status_pub = self.create_publisher(String, "/sim/status", 5)
        self.tf_broadcaster = TransformBroadcaster(self) if TransformBroadcaster else None
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_callback, 20)
        self.set_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        self.create_timer(1.0 / float(self.cfg["lane_rate_hz"]), self._publish_lane)
        self.create_timer(1.0 / float(self.cfg["mover_rate_hz"]), self._move_obstacles)
        self.create_timer(1.0, self._publish_markers)
        self.get_logger().info(
            f"gazebo bridge ready: scenario={scenario} obstacles={len(self.obstacles)} "
            f"moving={len(self.moving)} track={self.track.length:.2f} m"
        )

    def _sim_time(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- odometry
    def _gt_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vxw = msg.twist.twist.linear.x
        vyw = msg.twist.twist.linear.y
        vx = math.cos(yaw) * vxw + math.sin(yaw) * vyw
        vy = -math.sin(yaw) * vxw + math.cos(yaw) * vyw
        self.pose = (x, y, yaw)
        self.speed = vx
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = str(self.cfg["odom_frame"])
        odom.child_frame_id = str(self.cfg["base_frame"])
        odom.pose.pose = msg.pose.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        self.odom_pub.publish(odom)
        if self.tf_broadcaster and bool(self.cfg["publish_tf"]):
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = odom.header.frame_id
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------- lane
    def _publish_lane(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        s_ego, d_ego, _h, _k = self.track.project(x, y)
        n = int(self.cfg["lane_samples"])
        first = float(self.cfg["lane_first_sample_m"])
        spacing = float(self.cfg["lane_spacing_m"])
        points, kappas = [], []
        for i in range(n):
            px, py, _hh, k = self.track.pose(s_ego + first + i * spacing)
            points.append([px, py])
            kappas.append(k)
        local = inverse_transform_points(np.array(points), x, y, yaw)
        s_m = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(local[:, 0]), np.diff(local[:, 1])))])
        h0 = self.track.pose(s_ego + first)[2]
        msg = LaneReference()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.s_m = [float(v) for v in s_m]
        msg.curvature_inv_m = [float(v) for v in kappas]
        msg.cte_m = float(-local[0, 1])
        msg.heading_error_rad = float(-wrap_angle(h0 - yaw))
        msg.confidence = 1.0
        msg.status = LaneReference.STATUS_DETECTED
        msg.evaluation_forward_m = float(local[0, 0])
        msg.path_lateral_m = [float(v) for v in local[:, 1]]
        self.lane_pub.publish(msg)
        self.status_pub.publish(String(data=(
            f"s={s_ego:.2f} d={d_ego:+.3f} v={self.speed:.2f} "
            f"t={0.0 if self.t0 is None else self._sim_time() - self.t0:.0f}s"
        )))

    # -------------------------------------------------------------- obstacles
    def _move_obstacles(self):
        now = self._sim_time()
        if now <= 0.0 or self.pose is None:
            return
        if self.t0 is None:
            self.t0 = now
            self.last_mover = now
            return
        dt = max(now - self.last_mover, 0.0)
        self.last_mover = now
        t_rel = now - self.t0
        advance_obstacles(self.obstacles, dt, t_rel, self.track, ego_xy=self.pose[:2])
        if not self.moving or not self.set_state.service_is_ready() or self.pending > 20:
            return
        for i in self.moving:
            obs = self.obstacles[i]
            if obstacle_active(obs, t_rel):
                x, y, h = self.track.frenet_to_xy(obs["s"], obs["d"])
            else:
                x, y, h = PARK_X + i, PARK_X, 0.0
            req = SetEntityState.Request()
            state = EntityState()
            state.name = f"obstacle_{i}"
            state.reference_frame = "world"
            state.pose.position.x = float(x)
            state.pose.position.y = float(y)
            state.pose.position.z = OBSTACLE_HEIGHT / 2
            state.pose.orientation.z = math.sin(0.5 * h)
            state.pose.orientation.w = math.cos(0.5 * h)
            req.state = state
            self.pending += 1
            future = self.set_state.call_async(req)
            future.add_done_callback(self._done)

    def _done(self, _future):
        self.pending = max(0, self.pending - 1)

    # ---------------------------------------------------------------- markers
    def _publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        s_values = np.arange(0.0, self.track.length + 0.05, 0.05)
        for marker_id, d, rgb, width in ((0, 0.45, (0.95, 0.77, 0.1), 0.03),
                                         (1, -0.45, (0.95, 0.77, 0.1), 0.03),
                                         (2, 0.0, (0.5, 0.5, 0.55), 0.01)):
            m = Marker()
            m.header.frame_id = str(self.cfg["odom_frame"])
            m.header.stamp = stamp
            m.ns = "track"
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = width
            m.pose.orientation.w = 1.0
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=1.0)
            m.points = [Point(x=float(p[0]), y=float(p[1]), z=0.0)
                        for p in (self.track.frenet_to_xy(s, d)[:2] for s in s_values)]
            array.markers.append(m)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GazeboBridgeNode()
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

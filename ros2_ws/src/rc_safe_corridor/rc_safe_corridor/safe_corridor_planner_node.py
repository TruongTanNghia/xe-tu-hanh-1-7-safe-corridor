#!/usr/bin/env python3
"""Blocks B + C + E: Frenet world model, safe corridor generator, behaviour FSM.

/lane/reference + /perception/obstacles + /odometry/filtered + /safety/estop
  -> /planning/corridor   (rc_interfaces/Corridor, 20 Hz)
  -> /planning/behavior   (rc_interfaces/BehaviorState)
  -> /planning/corridor_markers (MarkerArray)
  -> /planning/status     (String)
"""

import math
import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rc_interfaces.msg import BehaviorState, Corridor, LaneReference, Obstacle, ObstacleArray
from std_msgs.msg import Bool, String
from visualization_msgs.msg import MarkerArray

from rc_safe_corridor.behavior_fsm import (
    BehaviorFSM,
    FsmInputs,
    FsmParams,
    find_lead_obstacle,
)
from rc_safe_corridor.corridor import (
    SIDE_CENTER,
    CorridorParams,
    generate_corridor,
    obstacle_box_to_frenet,
)
from rc_safe_corridor.frenet import (
    LaneSmoother,
    build_reference_path,
    straight_reference_path,
)
from rc_safe_corridor.geometry import box_corners, circle_samples
from rc_safe_corridor.markers import corridor_markers


LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class SafeCorridorPlannerNode(Node):
    def __init__(self):
        super().__init__("safe_corridor_planner")
        corridor_defaults = CorridorParams()
        fsm_defaults = FsmParams()
        declared = [
            ("lane_topic", "/lane/reference"),
            ("obstacle_topic", "/perception/obstacles"),
            ("odom_topic", "/odometry/filtered"),
            ("estop_topic", "/safety/estop"),
            ("base_frame", "base_link"),
            ("rate_hz", 20.0),
            ("lane_timeout_s", 0.5),
            ("obstacle_timeout_s", 0.6),
            ("odom_timeout_s", 0.5),
            ("minimum_lane_confidence", 0.5),
            ("lane_smoothing_alpha", 0.6),
            ("min_obstacle_confidence", 0.3),
            ("publish_markers", True),
        ]
        for name, value in vars(corridor_defaults).items():
            declared.append((f"corridor.{name}", value))
        for name, value in vars(fsm_defaults).items():
            declared.append((f"fsm.{name}", value))
        params = self.declare_parameters("", declared)
        self.cfg = {param.name: param.value for param in params}

        corridor_kwargs = {
            name: type(value)(self.cfg[f"corridor.{name}"])
            for name, value in vars(corridor_defaults).items()
        }
        self.corridor_params = CorridorParams(**corridor_kwargs)
        fsm_kwargs = {
            name: type(value)(self.cfg[f"fsm.{name}"])
            for name, value in vars(fsm_defaults).items()
        }
        self.fsm = BehaviorFSM(FsmParams(**fsm_kwargs))
        self.smoother = LaneSmoother(float(self.cfg["lane_smoothing_alpha"]))

        self.lane = None
        self.lane_time = None
        self.obstacles = None
        self.obstacle_time = None
        self.speed = 0.0
        self.odom_time = None
        self.estop = False
        self.previous_side = SIDE_CENTER
        self.previous_v_ref = 0.0
        self.lead_id = None

        self.corridor_publisher = self.create_publisher(Corridor, "/planning/corridor", 10)
        self.behavior_publisher = self.create_publisher(
            BehaviorState, "/planning/behavior", 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/planning/corridor_markers", 5
        )
        self.status_publisher = self.create_publisher(String, "/planning/status", 5)

        self.create_subscription(
            LaneReference, str(self.cfg["lane_topic"]), self._lane_callback, 10
        )
        self.create_subscription(
            ObstacleArray, str(self.cfg["obstacle_topic"]), self._obstacle_callback, 10
        )
        self.create_subscription(
            Odometry, str(self.cfg["odom_topic"]), self._odom_callback, 10
        )
        self.create_subscription(
            Bool, str(self.cfg["estop_topic"]), self._estop_callback, LATCHED_QOS
        )
        self.timer = self.create_timer(1.0 / float(self.cfg["rate_hz"]), self._plan)
        self.get_logger().info(
            "safe_corridor_planner ready: horizon=%.2f m ds=%.2f m lane_half=%.2f m "
            "vehicle_width=%.2f m v_cruise=%.2f m/s"
            % (
                self.corridor_params.horizon_m, self.corridor_params.ds,
                self.corridor_params.lane_half_width, self.corridor_params.vehicle_width,
                self.corridor_params.v_cruise,
            )
        )

    # ------------------------------------------------------------ callbacks
    def _lane_callback(self, msg):
        self.lane = msg
        self.lane_time = time.monotonic()

    def _obstacle_callback(self, msg):
        self.obstacles = msg
        self.obstacle_time = time.monotonic()

    def _odom_callback(self, msg):
        v = float(msg.twist.twist.linear.x)
        if math.isfinite(v):
            self.speed = v
            self.odom_time = time.monotonic()

    def _estop_callback(self, msg):
        self.estop = bool(msg.data)

    # -------------------------------------------------------------- helpers
    def _fresh(self, stamp, timeout):
        return stamp is not None and time.monotonic() - stamp <= float(timeout)

    def _build_path(self):
        lane_valid = (
            self.lane is not None
            and self._fresh(self.lane_time, self.cfg["lane_timeout_s"])
            and self.lane.status == LaneReference.STATUS_DETECTED
            and self.lane.confidence >= float(self.cfg["minimum_lane_confidence"])
            and len(self.lane.s_m) >= 2
            and len(self.lane.s_m) == len(self.lane.curvature_inv_m)
        )
        if not lane_valid:
            self.smoother.reset()
            return straight_reference_path(
                self.corridor_params.horizon_m, self.corridor_params.ds
            ), False
        cte, heading, kappa, eval_forward = self.smoother.update(
            self.lane.cte_m, self.lane.heading_error_rad,
            self.lane.curvature_inv_m, self.lane.evaluation_forward_m,
        )
        lateral = list(self.lane.path_lateral_m) if len(self.lane.path_lateral_m) else None
        try:
            path = build_reference_path(
                self.lane.s_m, kappa, cte, heading,
                evaluation_forward=eval_forward, path_lateral=lateral,
                horizon_m=self.corridor_params.horizon_m, ds=self.corridor_params.ds,
            )
        except ValueError as error:
            self.get_logger().warning(f"lane -> path failed: {error}")
            return straight_reference_path(
                self.corridor_params.horizon_m, self.corridor_params.ds
            ), False
        return path, True

    def _frenet_obstacles(self, path):
        if self.obstacles is None or not self._fresh(
            self.obstacle_time, self.cfg["obstacle_timeout_s"]
        ):
            return [], False
        result = []
        for obs in self.obstacles.obstacles:
            if not obs.confirmed or obs.confidence < float(self.cfg["min_obstacle_confidence"]):
                continue
            if obs.shape == Obstacle.SHAPE_BOX:
                samples = box_corners(obs.x, obs.y, obs.yaw, obs.length, obs.width)
                mids = 0.5 * (samples + np.roll(samples, -1, axis=0))
                samples = np.vstack([samples, mids])
            else:
                samples = circle_samples(obs.x, obs.y, obs.radius, 12)
            s_vals, d_vals = path.project(samples)
            s_center = float(np.mean(s_vals))
            _x, _y, heading, _k = path.pose_at([s_center])
            h = float(heading[0])
            vs = obs.vx * math.cos(h) + obs.vy * math.sin(h)
            vd = -obs.vx * math.sin(h) + obs.vy * math.cos(h)
            result.append(
                obstacle_box_to_frenet(
                    obs.id, s_vals, d_vals, vs=vs, vd=vd,
                    is_dynamic=obs.is_dynamic, confidence=obs.confidence,
                )
            )
        return result, True

    # ----------------------------------------------------------------- plan
    def _plan(self):
        started = time.perf_counter()
        now = time.monotonic()
        path, lane_valid = self._build_path()
        frenet_obstacles, obstacles_fresh = self._frenet_obstacles(path)
        odom_fresh = self._fresh(self.odom_time, self.cfg["odom_timeout_s"])
        speed = self.speed if odom_fresh else 0.0

        lead, lead_gap = find_lead_obstacle(
            frenet_obstacles, self.corridor_params.lane_half_width,
            lateral_tolerance=self.fsm.params.lead_lateral_tolerance_m,
            min_speed=self.fsm.params.lead_speed_min_mps,
        )
        # A same-direction lead is handled longitudinally (FOLLOW / ACC law),
        # never by cutting it out of the lateral corridor; the safety monitor
        # remains the independent backstop.
        ignore = (lead.obstacle_id,) if lead is not None else ()
        kappa = path.kappa_at(np.arange(int(round(
            self.corridor_params.horizon_m / self.corridor_params.ds)) + 1) * self.corridor_params.ds)
        corridor = generate_corridor(
            frenet_obstacles, self.corridor_params,
            ego_d=path.ego_d, v_ego=speed, previous_side=self.previous_side,
            kappa=kappa, ignore_ids=ignore,
        )
        out = self.fsm.step(
            FsmInputs(
                now=now, lane_valid=lane_valid,
                corridor_feasible=corridor.feasible, blocked_at_s=corridor.blocked_at_s,
                obstacle_influences=corridor.obstacle_count > 0,
                lead_gap_m=lead_gap, lead_speed_mps=0.0 if lead is None else lead.vs,
                estop=self.estop, corridor_side=corridor.side,
            )
        )
        self.previous_side = corridor.side if corridor.obstacle_count > 0 else SIDE_CENTER
        v_max = np.minimum(corridor.v_max, out.v_ref)

        stamp = self.get_clock().now().to_msg()
        msg = Corridor()
        msg.header.stamp = stamp
        msg.header.frame_id = str(self.cfg["base_frame"])
        msg.lane_valid = bool(lane_valid)
        msg.ds = float(self.corridor_params.ds)
        msg.s = [float(v) for v in corridor.s]
        msg.lower = [float(v) for v in corridor.lower]
        msg.upper = [float(v) for v in corridor.upper]
        msg.d_ref = [float(v) for v in corridor.d_ref]
        msg.v_max = [float(v) for v in v_max]
        msg.kappa_ref = [float(v) for v in kappa]
        msg.ego_d = float(path.ego_d)
        msg.ego_heading_error = float(path.ego_heading_error)
        msg.side = int(corridor.side)
        msg.feasible = bool(corridor.feasible and lane_valid)
        msg.blocked_at_s = float(corridor.blocked_at_s)
        msg.min_free_width = float(corridor.min_free_width)
        msg.behavior_state = int(out.state)
        msg.v_ref = float(out.v_ref)
        msg.obstacle_count = int(corridor.obstacle_count)
        self.corridor_publisher.publish(msg)

        behavior = BehaviorState()
        behavior.header.stamp = stamp
        behavior.header.frame_id = msg.header.frame_id
        behavior.state = int(out.state)
        behavior.state_name = out.state_name
        behavior.reason = out.reason
        behavior.v_ref = float(out.v_ref)
        behavior.time_in_state_s = float(out.time_in_state_s)
        behavior.lead_gap_m = float(lead_gap)
        behavior.lead_speed_mps = 0.0 if lead is None else float(lead.vs)
        behavior.avoid_side = int(out.avoid_side)
        self.behavior_publisher.publish(behavior)

        if bool(self.cfg["publish_markers"]):
            self.marker_publisher.publish(
                corridor_markers(path, corridor, msg.header.frame_id, stamp)
            )
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.status_publisher.publish(
            String(
                data=(
                    f"state={out.state_name} reason={out.reason} lane={'ok' if lane_valid else 'invalid'} "
                    f"obstacles={len(frenet_obstacles)}{'' if obstacles_fresh else '(stale)'} "
                    f"influencing={corridor.obstacle_count} side={corridor.side} "
                    f"feasible={corridor.feasible} blocked_at={corridor.blocked_at_s:.2f} "
                    f"ego_d={path.ego_d:+.3f} v_ref={out.v_ref:.2f} plan_ms={elapsed_ms:.1f}"
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafeCorridorPlannerNode()
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

#!/usr/bin/env python3
"""Simple corridor-following controller (no acados needed).

Uses only /planning/corridor (ego_d, ego_heading_error, d_ref, v_max, v_ref,
kappa_ref) and publishes geometry_msgs/Twist on /cmd_vel:
    linear.x  = speed command
    angular.z = speed * tan(steering) / wheelbase
It is the same law as the lightweight simulator and is meant for Gazebo
demos; on the real car the NMPC (race_cars, corridor_mode) is used instead.
"""

import math
import time

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rc_interfaces.msg import Corridor
from std_msgs.msg import String


class CorridorFollowerNode(Node):
    def __init__(self):
        super().__init__("corridor_follower")
        params = self.declare_parameters(
            "",
            [
                ("corridor_topic", "/planning/corridor"),
                ("cmd_topic", "/cmd_vel"),
                ("rate_hz", 20.0),
                ("wheelbase_m", 0.375),
                ("lookahead_m", 0.5),
                ("heading_gain", 1.8),
                ("max_steer_rad", 0.52),
                ("speed_lookahead_m", 0.3),
                ("corridor_timeout_s", 0.5),
            ],
        )
        self.cfg = {p.name: p.value for p in params}
        self.corridor = None
        self.corridor_time = None
        self.cmd_pub = self.create_publisher(Twist, str(self.cfg["cmd_topic"]), 10)
        self.status_pub = self.create_publisher(String, "/control/follower_status", 5)
        self.create_subscription(Corridor, str(self.cfg["corridor_topic"]), self._callback, 10)
        self.create_timer(1.0 / float(self.cfg["rate_hz"]), self._step)
        self.get_logger().info("corridor_follower ready (publishes /cmd_vel)")

    def _callback(self, msg):
        self.corridor = msg
        self.corridor_time = time.monotonic()

    def compute(self, c):
        lookahead = float(self.cfg["lookahead_m"])
        d_target = float(np.interp(lookahead, c.s, c.d_ref))
        v_target = min(float(c.v_ref), float(np.interp(float(self.cfg["speed_lookahead_m"]), c.s, c.v_max)))
        if not c.lane_valid:
            v_target = 0.0
        kappa = float(c.kappa_ref[0]) if len(c.kappa_ref) else 0.0
        psi_des = math.atan2(d_target - float(c.ego_d), lookahead)
        steer = float(self.cfg["heading_gain"]) * (psi_des - float(c.ego_heading_error))
        steer += math.atan(kappa * float(self.cfg["wheelbase_m"]))
        limit = float(self.cfg["max_steer_rad"])
        return max(0.0, v_target), max(-limit, min(limit, steer))

    def _step(self):
        twist = Twist()
        fresh = (
            self.corridor is not None
            and time.monotonic() - self.corridor_time <= float(self.cfg["corridor_timeout_s"])
            and len(self.corridor.s) >= 2
        )
        if fresh:
            v, steer = self.compute(self.corridor)
            twist.linear.x = v
            # Keep the steering angle even at very low speed so the wheels do
            # not snap straight while creeping around an obstacle.
            twist.angular.z = max(v, 0.05) * math.tan(steer) / float(self.cfg["wheelbase_m"])
            status = f"v={v:.2f} steer={math.degrees(steer):+.1f}deg"
        else:
            status = "corridor_stale_stop"
        self.cmd_pub.publish(twist)
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CorridorFollowerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

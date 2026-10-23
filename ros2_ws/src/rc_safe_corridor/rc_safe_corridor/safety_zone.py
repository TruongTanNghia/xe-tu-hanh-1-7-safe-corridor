"""Block F: brake-zone test used by the independent safety monitor.

Deliberately simple: a rectangle in front of the bumper whose length is the
stopping distance at the current speed. If at least `min_points` LiDAR
returns fall inside it, the vehicle must be disarmed.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class SafetyParams:
    a_brake: float = 1.5            # m/s^2, achievable deceleration (coast + friction)
    t_delay: float = 0.25           # s, sensing + actuation latency
    d0: float = 0.20                # m, standstill clearance from the bumper
    front_overhang: float = 0.32    # m, base_link -> bumper
    half_width: float = 0.17        # m, vehicle half width + small margin
    min_points: int = 3
    v_assumed_if_odom_stale: float = 0.8
    max_zone_length: float = 2.0
    min_zone_length: float = 0.25


def brake_distance(speed, params: SafetyParams):
    v = max(float(speed), 0.0)
    distance = v * v / (2.0 * params.a_brake) + v * params.t_delay + params.d0
    return float(min(max(distance, params.min_zone_length), params.max_zone_length))


def points_in_brake_zone(points_xy, zone_length, params: SafetyParams):
    """Count returns inside the rectangle [front, front + zone_length] x [-w, w]."""
    points_xy = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    if points_xy.size == 0:
        return 0
    x = points_xy[:, 0]
    y = points_xy[:, 1]
    x_start = params.front_overhang
    inside = (
        (x >= x_start)
        & (x <= x_start + float(zone_length))
        & (np.abs(y) <= params.half_width)
    )
    return int(np.count_nonzero(inside))


def evaluate_brake_zone(points_xy, speed, params: SafetyParams, odom_fresh=True):
    """Return (trigger, count, zone_length)."""
    v = float(speed) if odom_fresh else params.v_assumed_if_odom_stale
    zone_length = brake_distance(v, params)
    count = points_in_brake_zone(points_xy, zone_length, params)
    return count >= int(params.min_points), count, zone_length

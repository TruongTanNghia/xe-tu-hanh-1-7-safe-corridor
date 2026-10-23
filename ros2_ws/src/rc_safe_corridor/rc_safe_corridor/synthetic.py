"""Synthetic LiDAR scans for offline tests and demos (no ROS required).

Obstacles are circles or axis-aligned boxes in base_link; the scan is ray
cast from the laser pose with the RPLIDAR A1 geometry (360 deg, 1/3 deg).
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class SimObstacle:
    x: float
    y: float
    radius: float = 0.05           # for circles
    length: float = 0.0            # >0 -> box (along x)
    width: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    name: str = "obstacle"
    yaw: float = 0.0            # box orientation

    def advance(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt


def _ray_circle(origin, direction, cx, cy, radius):
    ox, oy = origin
    dx, dy = direction
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return math.inf
    sq = math.sqrt(disc)
    t1 = (-b - sq) / 2.0
    t2 = (-b + sq) / 2.0
    for t in (t1, t2):
        if t > 0.0:
            return t
    return math.inf


def _ray_box(origin, direction, cx, cy, length, width):
    ox, oy = origin
    dx, dy = direction
    t_min, t_max = -math.inf, math.inf
    for o, d, lo, hi in (
        (ox, dx, cx - 0.5 * length, cx + 0.5 * length),
        (oy, dy, cy - 0.5 * width, cy + 0.5 * width),
    ):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return math.inf
            continue
        t1 = (lo - o) / d
        t2 = (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return math.inf
    if t_max < 0.0:
        return math.inf
    return t_min if t_min > 0.0 else t_max


def synthetic_scan(
    obstacles,
    laser_pose=(-0.005, 0.0, 0.0),
    count=1080,
    range_min=0.15,
    range_max=12.0,
    noise_std=0.005,
    walls_y=None,
    rng=None,
):
    """Return (ranges, angle_min, angle_increment) for a 360 deg scan.

    walls_y: optional (y_left, y_right) infinite walls parallel to x to mimic
    the table edge / people standing beside the track.
    """
    rng = np.random.default_rng(0) if rng is None else rng
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / count
    lx, ly, lyaw = laser_pose
    ranges = np.full(count, math.inf)
    for i in range(count):
        angle = angle_min + i * angle_increment + lyaw
        direction = (math.cos(angle), math.sin(angle))
        best = math.inf
        for obs in obstacles:
            if obs.length > 0.0:
                if abs(obs.yaw) > 1e-9:
                    c, s_ = math.cos(-obs.yaw), math.sin(-obs.yaw)
                    ox, oy = lx - obs.x, ly - obs.y
                    origin_b = (c * ox - s_ * oy, s_ * ox + c * oy)
                    dir_b = (c * direction[0] - s_ * direction[1], s_ * direction[0] + c * direction[1])
                    t = _ray_box(origin_b, dir_b, 0.0, 0.0, obs.length, obs.width)
                else:
                    t = _ray_box((lx, ly), direction, obs.x, obs.y, obs.length, obs.width)
            else:
                t = _ray_circle((lx, ly), direction, obs.x, obs.y, obs.radius)
            best = min(best, t)
        if walls_y is not None:
            for wall in walls_y:
                if abs(direction[1]) > 1e-9:
                    t = (wall - ly) / direction[1]
                    if t > 0.0:
                        best = min(best, t)
        if math.isfinite(best):
            best += rng.normal(0.0, noise_std)
            if best < range_min or best > range_max:
                best = math.inf
        ranges[i] = best
    return ranges, angle_min, angle_increment

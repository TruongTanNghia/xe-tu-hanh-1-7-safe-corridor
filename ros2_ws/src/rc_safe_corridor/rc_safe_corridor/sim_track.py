"""Track geometry and obstacle scenarios shared by the ROS simulator
(sim_world_node) and the ROS-free viewer (sim_view)."""

import math

import numpy as np


# Scenarios: list of obstacles as dicts in track coordinates (s along the
# centre line from the start, d positive to the left). Optional keys:
# speed (m/s along s), lat_speed/lat_min/lat_max (crossing oscillation),
# appear_t / vanish_t (seconds since start).
SCENARIOS = {
    "clear": [],
    "right": [dict(s=3.0, d=-0.22, radius=0.05)],
    "left": [dict(s=3.0, d=0.22, radius=0.05)],
    "slalom": [dict(s=2.5, d=-0.22, radius=0.05), dict(s=3.9, d=0.22, radius=0.05),
               dict(s=5.3, d=-0.22, radius=0.05)],
    "curve": [dict(s=4.6, d=0.22, radius=0.05)],
    "block": [dict(s=3.5, d=0.0, length=0.2, width=0.9)],
    "lead": [dict(s=2.0, d=0.0, length=0.3, width=0.2, speed=0.25)],
    "mixed": [dict(s=2.5, d=-0.22, radius=0.05), dict(s=4.6, d=0.22, radius=0.05),
              dict(s=7.5, d=0.0, length=0.3, width=0.2, speed=0.25)],
    # One lap with everything (S1-S5, S7 + crossing pedestrian).
    "tour": [
        dict(s=1.8, d=-0.22, radius=0.05),                                   # S2 right
        dict(s=4.6, d=0.22, radius=0.05),                                    # S3 in corner
        dict(s=7.0, d=0.22, radius=0.05), dict(s=8.4, d=-0.22, radius=0.05), # S4 slalom
        # Pedestrian waiting at the left edge; starts crossing to the right
        # (and out of the lane) when the ego comes within 1.6 m.
        dict(s=9.5, d=0.55, radius=0.08, lat_speed=0.18, cross_to=-0.8, cross_trigger_m=1.6),
        # S5: full block; removed 6 s after the car has come within 1.6 m.
        dict(s=11.6, d=0.0, length=0.2, width=0.9, vanish_after_near_s=6.0, near_m=1.6),
        # S7: slow lead car; appears 1.3 m ahead once the block is gone and the
        # car is back on the first straight, drives 0.25 m/s for 30 s.
        dict(s=0.0, d=0.0, length=0.3, width=0.2, speed=0.25, appear_after=5,
             appear_at_ego_s=(0.2, 1.2), place_ahead_m=1.3, active_for_s=30.0),
    ],
}


class OvalTrack:
    def __init__(self, straight_m=3.5, radius_m=0.925, ds=0.02):
        self.straight = float(straight_m)
        self.radius = float(radius_m)
        self.length = 2.0 * self.straight + 2.0 * math.pi * self.radius
        self.ds = ds
        self.s_samples = np.arange(0.0, self.length, ds)
        self.xy = np.array([self.pose(s)[:2] for s in self.s_samples])

    def pose(self, s):
        """(x, y, heading, kappa) of the centre line at arc length s."""
        s = float(s) % self.length
        L, R = self.straight, self.radius
        half = math.pi * R
        if s < L:
            return s, 0.0, 0.0, 0.0
        s -= L
        if s < half:
            a = s / R
            return L + R * math.sin(a), R - R * math.cos(a), a, 1.0 / R
        s -= half
        if s < L:
            return L - s, 2.0 * R, math.pi, 0.0
        s -= L
        a = s / R
        return -R * math.sin(a), R + R * math.cos(a), math.pi + a, 1.0 / R

    def frenet_to_xy(self, s, d):
        x, y, h, _k = self.pose(s)
        return x - d * math.sin(h), y + d * math.cos(h), h

    def project(self, x, y):
        """Return (s, d, heading, kappa) of the nearest centre-line point."""
        idx = int(np.argmin(np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)))
        s = float(self.s_samples[idx])
        px, py, h, k = self.pose(s)
        d = -math.sin(h) * (x - px) + math.cos(h) * (y - py)
        return s, d, h, k


TRIGGER_KEYS = ("vanish_after_near_s", "appear_after", "appear_at_ego_s")


def is_triggered(obs):
    return any(key in obs for key in TRIGGER_KEYS)


def initially_active(obs):
    if "appear_after" in obs or "appear_at_ego_s" in obs:
        return False
    return obs.get("appear_t", -1.0) <= 0.0


def obstacle_active(obs, t_sim):
    if "_on" in obs:
        return obs["_on"]
    if "appear_after" in obs or "appear_at_ego_s" in obs:
        return False
    return obs.get("appear_t", -1.0) <= t_sim < obs.get("vanish_t", float("inf"))


def _update_triggers(obstacles, t_sim, track, ego_xy):
    """Position-triggered appear / vanish so scenarios do not depend on how
    fast the car happens to drive (lightweight sim vs Gazebo)."""
    if ego_xy is None:
        return
    s_ego = None
    for obs in obstacles:
        if not is_triggered(obs):
            continue
        if "_on" not in obs:
            obs["_on"] = initially_active(obs)
            obs["_done"] = False
        if obs["_on"] and "vanish_after_near_s" in obs:
            if s_ego is None:
                s_ego = track.project(*ego_xy)[0]
            ahead = (obs["s"] - s_ego) % track.length   # along-track distance in front
            if "_near_t" not in obs and ahead <= obs.get("near_m", 1.5):
                obs["_near_t"] = t_sim
            if "_near_t" in obs and t_sim - obs["_near_t"] >= obs["vanish_after_near_s"]:
                obs["_on"] = False
                obs["_done"] = True
        if not obs["_on"] and not obs["_done"] and "appear_at_ego_s" in obs:
            prereq = obs.get("appear_after")
            if prereq is not None and not obstacles[prereq].get("_done", False):
                continue
            if s_ego is None:
                s_ego = track.project(*ego_xy)[0]
            lo, hi = obs["appear_at_ego_s"]
            if lo <= s_ego <= hi:
                obs["_on"] = True
                obs["_on_t"] = t_sim
                obs["s"] = (s_ego + obs.get("place_ahead_m", 1.2)) % track.length
        if obs["_on"] and "active_for_s" in obs and t_sim - obs.get("_on_t", t_sim) >= obs["active_for_s"]:
            obs["_on"] = False
            obs["_done"] = True


def advance_obstacles(obstacles, dt, t_sim, track, ego_xy=None):
    """Move dynamic obstacles.

    * speed: drive along the centre line (lead vehicle).
    * lat_speed + cross_to (+ cross_trigger_m): a pedestrian that starts
      crossing towards d = cross_to once the ego is within cross_trigger_m,
      then stays there (out of the lane).
    * lat_speed + lat_min/lat_max: keep oscillating laterally.
    """
    _update_triggers(obstacles, t_sim, track, ego_xy)
    for obs in obstacles:
        if not obstacle_active(obs, t_sim):
            continue
        if obs.get("speed", 0.0):
            obs["s"] = (obs["s"] + obs["speed"] * dt) % track.length
        if not obs.get("lat_speed", 0.0):
            continue
        if "cross_to" in obs:
            if not obs.get("_crossing", False):
                if ego_xy is None:
                    continue
                ox, oy, _h = track.frenet_to_xy(obs["s"], obs["d"])
                if math.hypot(ox - ego_xy[0], oy - ego_xy[1]) > obs.get("cross_trigger_m", 1.5):
                    continue
                obs["_crossing"] = True
            target = obs["cross_to"]
            step = abs(obs["lat_speed"]) * dt
            if abs(target - obs["d"]) <= step:
                obs["d"] = target
            else:
                obs["d"] += step if target > obs["d"] else -step
            continue
        obs["d"] += obs["lat_speed"] * dt
        if obs["d"] > obs.get("lat_max", 0.7):
            obs["d"] = obs.get("lat_max", 0.7)
            obs["lat_speed"] = -abs(obs["lat_speed"])
        elif obs["d"] < obs.get("lat_min", -0.7):
            obs["d"] = obs.get("lat_min", -0.7)
            obs["lat_speed"] = abs(obs["lat_speed"])

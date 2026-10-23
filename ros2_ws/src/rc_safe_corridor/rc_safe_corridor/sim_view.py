#!/usr/bin/env python3
"""ROS-free real-time viewer (Windows/macOS/Linux) of the Safe Corridor stack.

Runs the same closed loop as the ROS 2 sim demo (synthetic LiDAR -> obstacle
detector -> Frenet -> corridor -> FSM -> simple controller -> bicycle model)
and animates it with matplotlib. Useful when ROS 2 / RViz2 is not available.

    python3 -m rc_safe_corridor.sim_view --scenario tour
    python3 -m rc_safe_corridor.sim_view --scenario tour --save demo.mp4 --duration 120
"""

import argparse
import math
import time

import numpy as np

from rc_safe_corridor.behavior_fsm import (
    STATE_NAMES,
    BehaviorFSM,
    FsmInputs,
    FsmParams,
    find_lead_obstacle,
)
from rc_safe_corridor.corridor import CorridorParams, generate_corridor, obstacle_box_to_frenet
from rc_safe_corridor.frenet import build_reference_path
from rc_safe_corridor.geometry import (
    box_corners,
    circle_samples,
    inverse_transform_points,
    transform_points,
    wrap_angle,
)
from rc_safe_corridor.scan_clustering import (
    apply_lane_gate,
    apply_roi,
    detect_obstacles,
    scan_to_points,
)
from rc_safe_corridor.sim_track import SCENARIOS, OvalTrack, advance_obstacles, obstacle_active
from rc_safe_corridor.synthetic import SimObstacle, synthetic_scan
from rc_safe_corridor.tracking import MultiObjectTracker, TrackerParams


LASER_POSE = (-0.005, 0.0, 0.0)
WHEELBASE = 0.375


class ClosedLoopSim:
    """Same logic as sim_world_node + obstacle_detector + planner, in-process."""

    def __init__(self, scenario="tour", dt=0.05, params=None):
        self.track = OvalTrack()
        self.obstacles = [dict(o) for o in SCENARIOS[scenario]]
        self.params = params or CorridorParams()
        self.dt = dt
        self.t = 0.0
        self.x, self.y, self.yaw = self.track.pose(0.0)[:3]
        self.v = 0.0
        self.steer = 0.0
        self.tracker = MultiObjectTracker(TrackerParams())
        self.fsm = BehaviorFSM(FsmParams(v_cruise=self.params.v_cruise))
        self.stations = np.arange(int(round(self.params.horizon_m / self.params.ds)) + 1) * self.params.ds
        self.prev_side = 0
        self.step_count = 0
        # Last perception/planning outputs (for drawing).
        self.scan_xy = np.zeros((0, 2))
        self.tracks = []
        self.path = None
        self.corridor = None
        self.fsm_out = None
        self.lead_gap = -1.0

    # ---------------------------------------------------------------- helpers
    def _sim_obstacles_base_link(self):
        result = []
        for obs in self.obstacles:
            if not obstacle_active(obs, self.t):
                continue
            ox, oy, oh = self.track.frenet_to_xy(obs["s"], obs["d"])
            local = inverse_transform_points(np.array([[ox, oy]]), self.x, self.y, self.yaw)[0]
            result.append(SimObstacle(
                float(local[0]), float(local[1]), obs.get("radius", 0.05),
                length=obs.get("length", 0.0), width=obs.get("width", 0.0),
                yaw=wrap_angle(oh - self.yaw),
            ))
        return result

    def _lane(self, s_ego):
        pts, kappas = [], []
        for i in range(12):
            s = s_ego + 0.35 + i * 0.09
            px, py, _h, k = self.track.pose(s)
            pts.append([px, py])
            kappas.append(k)
        local = inverse_transform_points(np.array(pts), self.x, self.y, self.yaw)
        s_m = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(local[:, 0]), np.diff(local[:, 1])))])
        h0 = self.track.pose(s_ego + 0.35)[2]
        return build_reference_path(
            s_m, kappas, cte=-local[0, 1], heading_error=-wrap_angle(h0 - self.yaw),
            evaluation_forward=local[0, 0], path_lateral=local[:, 1],
            horizon_m=self.params.horizon_m, ds=self.params.ds,
        )

    def _perceive_and_plan(self, s_ego):
        self.path = self._lane(s_ego)
        ranges, amin, inc = synthetic_scan(
            self._sim_obstacles_base_link(), laser_pose=LASER_POSE, rng=np.random.default_rng()
        )
        pts = apply_roi(scan_to_points(ranges, amin, inc, 0.15, 12.0, laser_pose=LASER_POSE))
        pts = apply_lane_gate(pts, self.path.xy, self.params.lane_half_width, 0.15)
        self.scan_xy = pts.xy
        # Track in odom like the ROS node (static objects stay static).
        dets = detect_obstacles(pts, inc)
        for det in dets:
            det.center = transform_points(det.center, self.x, self.y, self.yaw)[0]
            det.yaw += self.yaw
        self.tracks = self.tracker.update(dets, 0.1)
        frenet = []
        for tr in self.tracks:
            pos = inverse_transform_points(tr.position, self.x, self.y, self.yaw)[0]
            vel = tr.velocity
            c, s_ = math.cos(-self.yaw), math.sin(-self.yaw)
            vel = np.array([c * vel[0] - s_ * vel[1], s_ * vel[0] + c * vel[1]])
            shape = tr.shape
            if shape.shape == "box":
                samples = box_corners(*pos, shape.yaw - self.yaw, shape.length, shape.width)
            else:
                samples = circle_samples(*pos, shape.radius)
            sv, dv = self.path.project(samples)
            _x, _y, heading, _k = self.path.pose_at([float(np.mean(sv))])
            h = float(heading[0])
            frenet.append(obstacle_box_to_frenet(
                tr.track_id, sv, dv,
                vs=vel[0] * math.cos(h) + vel[1] * math.sin(h),
                vd=-vel[0] * math.sin(h) + vel[1] * math.cos(h),
                is_dynamic=tr.is_dynamic,
            ))
        lead, self.lead_gap = find_lead_obstacle(frenet, self.params.lane_half_width)
        ignore = (lead.obstacle_id,) if lead is not None else ()
        self.corridor = generate_corridor(
            frenet, self.params, ego_d=self.path.ego_d, v_ego=self.v,
            previous_side=self.prev_side, kappa=self.path.kappa_at(self.stations), ignore_ids=ignore,
        )
        self.fsm_out = self.fsm.step(FsmInputs(
            now=self.t, lane_valid=True, corridor_feasible=self.corridor.feasible,
            blocked_at_s=self.corridor.blocked_at_s,
            obstacle_influences=self.corridor.obstacle_count > 0,
            lead_gap_m=self.lead_gap, lead_speed_mps=0.0 if lead is None else lead.vs,
            corridor_side=self.corridor.side,
        ))
        self.prev_side = self.corridor.side if self.corridor.obstacle_count else 0
        self.corridor.v_max = np.minimum(self.corridor.v_max, self.fsm_out.v_ref)

    # ------------------------------------------------------------------- step
    def step(self):
        dt = self.dt
        self.t += dt
        advance_obstacles(self.obstacles, dt, self.t, self.track, ego_xy=(self.x, self.y))
        s_ego, d_ego, h_path, kappa = self.track.project(self.x, self.y)
        if self.step_count % 2 == 0:          # perception + planning at 10 Hz
            self._perceive_and_plan(s_ego)
        self.step_count += 1
        c = self.corridor
        e_psi = wrap_angle(self.yaw - h_path)
        d_target = float(np.interp(0.5, c.s, c.d_ref))
        v_target = float(np.interp(0.3, c.s, c.v_max))
        psi_des = math.atan2(d_target - d_ego, 0.5)
        cmd = 1.8 * (psi_des - e_psi) + math.atan(kappa * WHEELBASE)
        cmd = max(-0.52, min(0.52, cmd))
        self.steer += max(-2.5 * dt, min(2.5 * dt, cmd - self.steer))
        if v_target > self.v:
            self.v = min(v_target, self.v + 1.0 * dt)
        else:
            self.v = max(v_target, self.v - 1.5 * dt)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + self.v / WHEELBASE * math.tan(self.steer) * dt)
        return s_ego, d_ego


# ------------------------------------------------------------------ drawing
def run_viewer(scenario, duration, save, speedup):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Polygon

    sim = ClosedLoopSim(scenario)
    track = sim.track
    fig, ax = plt.subplots(figsize=(11, 6.5), facecolor="#1e1e24")
    ax.set_facecolor("#1e1e24")
    ax.set_aspect("equal")
    ax.set_xlim(-1.6, track.straight + 1.6)
    ax.set_ylim(-1.3, 2 * track.radius + 1.3)
    ax.tick_params(colors="#888")
    for spine in ax.spines.values():
        spine.set_color("#444")
    s_all = np.arange(0.0, track.length + 0.05, 0.05)
    for d, color, lw in ((0.45, "#f0c419", 2.0), (-0.45, "#f0c419", 2.0), (0.0, "#55556a", 0.8)):
        pts = np.array([track.frenet_to_xy(s, d)[:2] for s in s_all])
        ax.plot(pts[:, 0], pts[:, 1], color=color, lw=lw, ls="-" if d else "--")
    truth_artists = []
    scan_sc = ax.scatter([], [], s=6, c="#ff9f43", zorder=5)
    corridor_poly = Polygon(np.zeros((3, 2)), closed=True, fc="#2ecc7140", ec="#2ecc71", lw=1.5, zorder=3)
    ax.add_patch(corridor_poly)
    (dref_line,) = ax.plot([], [], color="white", lw=1.5, zorder=4)
    ego_poly = Polygon(np.zeros((4, 2)), closed=True, fc="#9b59b680", ec="#c39bd3", lw=1.5, zorder=6)
    ax.add_patch(ego_poly)
    track_artists = []
    stop_line, = ax.plot([], [], color="#ff3b3b", lw=4, zorder=4)
    text = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left",
                   color="#eeeeee", family="monospace", fontsize=9,
                   bbox=dict(fc="#1e1e24cc", ec="none"))
    ax.set_title(f"Safe Corridor sim · scenario = {scenario}", color="#ddd")

    def frenet_xy(s, d):
        x, y, heading, _k = sim.path.pose_at([s])
        h = float(heading[0])
        return transform_points(np.array([[float(x[0] - d * math.sin(h)), float(y[0] + d * math.cos(h))]]),
                                sim.x, sim.y, sim.yaw)[0]

    def update(_frame):
        for _ in range(speedup):
            s_ego, d_ego = sim.step()
        for art in truth_artists + track_artists:
            art.remove()
        truth_artists.clear()
        track_artists.clear()
        for obs in sim.obstacles:
            if not obstacle_active(obs, sim.t):
                continue
            ox, oy, oh = track.frenet_to_xy(obs["s"], obs["d"])
            if obs.get("length", 0.0) > 0:
                poly = Polygon(box_corners(ox, oy, oh, obs["length"], obs["width"]), closed=True,
                               fc="#99999960", ec="#bbbbbb", zorder=2)
                ax.add_patch(poly)
                truth_artists.append(poly)
            else:
                circ = plt.Circle((ox, oy), obs.get("radius", 0.05), fc="#99999960", ec="#bbbbbb", zorder=2)
                ax.add_patch(circ)
                truth_artists.append(circ)
        scan_world = transform_points(sim.scan_xy, sim.x, sim.y, sim.yaw)
        scan_sc.set_offsets(scan_world if scan_world.size else np.zeros((0, 2)))
        for tr in sim.tracks:
            circ = plt.Circle(tr.position, max(tr.shape.radius, 0.04), fill=False, ec="#ff5e57", lw=2, zorder=7)
            ax.add_patch(circ)
            track_artists.append(circ)
            label = ax.text(tr.position[0] + 0.08, tr.position[1] + 0.08,
                            f"#{tr.track_id}{' dyn' if tr.is_dynamic else ''}", color="#ffd1d1", fontsize=8, zorder=8)
            track_artists.append(label)
        c = sim.corridor
        n = int(np.count_nonzero(c.v_max > 0.0)) or 2
        lower = [frenet_xy(s, d) for s, d in zip(c.s[:n], c.lower[:n])]
        upper = [frenet_xy(s, d) for s, d in zip(c.s[:n][::-1], c.upper[:n][::-1])]
        corridor_poly.set_xy(np.array(lower + upper))
        corridor_poly.set_facecolor("#2ecc7140" if c.feasible else "#e74c3c40")
        corridor_poly.set_edgecolor("#2ecc71" if c.feasible else "#e74c3c")
        dref = np.array([frenet_xy(s, d) for s, d in zip(c.s, c.d_ref)])
        dref_line.set_data(dref[:, 0], dref[:, 1])
        if c.blocked_at_s >= 0.0:
            a = frenet_xy(c.blocked_at_s, -0.5)
            b = frenet_xy(c.blocked_at_s, 0.5)
            stop_line.set_data([a[0], b[0]], [a[1], b[1]])
        else:
            stop_line.set_data([], [])
        ego_poly.set_xy(box_corners(sim.x, sim.y, sim.yaw, 0.632, 0.30))
        out = sim.fsm_out
        text.set_text(
            f"t={sim.t:5.1f}s  s={s_ego:5.2f}m  d={d_ego:+.2f}m  v={sim.v:.2f}m/s\n"
            f"FSM={STATE_NAMES[out.state]} ({out.reason})  v_ref={out.v_ref:.2f}\n"
            f"corridor: feasible={c.feasible} side={c.side} blocked_at={c.blocked_at_s:.2f} "
            f"obstacles={c.obstacle_count} tracks={len(sim.tracks)} lead_gap={sim.lead_gap:.2f}"
        )
        return [scan_sc, corridor_poly, dref_line, ego_poly, stop_line, text]

    interval_ms = int(sim.dt * 1000 * speedup)
    frames = int(duration / (sim.dt * speedup))
    anim = animation.FuncAnimation(fig, update, frames=frames, interval=interval_ms, blit=False, repeat=False)
    if save:
        fps = max(1, int(round(1000 / interval_ms)))
        anim.save(save, writer=animation.FFMpegWriter(fps=fps, bitrate=2500), dpi=110)
        print(f"saved {save} ({frames} frames, {fps} fps)")
    else:
        plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="tour", choices=sorted(SCENARIOS))
    parser.add_argument("--duration", type=float, default=120.0, help="simulated seconds")
    parser.add_argument("--save", default="", help="write an .mp4 instead of opening a window")
    parser.add_argument("--speedup", type=int, default=2, help="sim steps per drawn frame")
    args = parser.parse_args(argv)
    started = time.time()
    run_viewer(args.scenario, args.duration, args.save, max(1, args.speedup))
    print(f"done in {time.time() - started:.1f}s")


if __name__ == "__main__":
    main()

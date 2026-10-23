#!/usr/bin/env python3
"""Offline demo: scenarios S1-S5 rendered to SVG without ROS or a car.

    python3 -m rc_safe_corridor.offline_demo --output /tmp/corridor_demo
    ros2 run rc_safe_corridor corridor_offline_demo --scenario s4

Each SVG shows (top view, base_link, x forward = up): lane band, LiDAR
returns, tracked obstacles, corridor bounds (green/red), d_ref (white)
and the stop line when the lane is blocked.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from rc_safe_corridor.behavior_fsm import (
    STATE_NAMES,
    BehaviorFSM,
    FsmInputs,
    FsmParams,
    find_lead_obstacle,
)
from rc_safe_corridor.corridor import CorridorParams, generate_corridor, obstacle_box_to_frenet
from rc_safe_corridor.frenet import build_reference_path, straight_reference_path
from rc_safe_corridor.geometry import box_corners, circle_samples
from rc_safe_corridor.scan_clustering import (
    apply_lane_gate,
    apply_roi,
    detect_obstacles,
    scan_to_points,
)
from rc_safe_corridor.synthetic import SimObstacle, synthetic_scan
from rc_safe_corridor.tracking import MultiObjectTracker, TrackerParams


LASER_POSE = (-0.005, 0.0, 0.0)

SCENARIOS = {
    "s1": ("Không vật cản", [], 0.0),
    "s2": ("1 vật tĩnh lệch phải trên đoạn thẳng", [SimObstacle(1.3, -0.22, 0.05)], 0.0),
    # Trong cua, vật được đặt theo (s, d) trên tâm làn rồi đổi sang base_link.
    "s3": ("1 vật tĩnh lệch trái trong cua R700", [{"s": 1.1, "d": 0.22, "radius": 0.05}], 1.0 / 0.7),
    "s4": ("2 vật so le (slalom)", [SimObstacle(0.8, -0.22, 0.05), SimObstacle(2.2, 0.22, 0.05)], 0.0),
    "s5": ("Vật chắn kín làn", [SimObstacle(1.5, 0.0, length=0.2, width=0.9)], 0.0),
    "s7": ("Xe trước chạy chậm cùng chiều", [SimObstacle(1.2, 0.0, length=0.3, width=0.2, vx=0.3)], 0.0),
}


def _lane_arrays(kappa, horizon=1.0, n=12, start=0.35):
    s = np.linspace(0.0, horizon, n)
    if kappa == 0.0:
        lateral = np.zeros(n)
    else:
        lateral = (1.0 - np.cos(kappa * s)) / kappa
    return s, np.full(n, kappa), lateral, start


def run_scenario(key, params=None, frames=8, dt=0.15, v_ego=0.6):
    title, sim_obstacles, kappa = SCENARIOS[key]
    params = params or CorridorParams()
    lane_s, lane_kappa, lateral, start = _lane_arrays(kappa)
    path = build_reference_path(
        lane_s, lane_kappa, cte=-lateral[0], heading_error=0.0,
        evaluation_forward=start, path_lateral=lateral,
        horizon_m=params.horizon_m, ds=params.ds,
    )
    resolved = []
    for o in sim_obstacles:
        if isinstance(o, dict):
            x, y, heading, _k = path.pose_at([o["s"]])
            h = float(heading[0])
            resolved.append(SimObstacle(
                float(x[0] - o["d"] * math.sin(h)), float(y[0] + o["d"] * math.cos(h)),
                o.get("radius", 0.05),
            ))
        else:
            resolved.append(SimObstacle(**vars(o)))
    sim_obstacles = resolved
    walls = None if kappa != 0.0 else (1.1, -1.1)
    tracker = MultiObjectTracker(TrackerParams())
    tracks = []
    points = None
    for _ in range(frames):
        ranges, amin, inc = synthetic_scan(sim_obstacles, laser_pose=LASER_POSE, walls_y=walls)
        points = scan_to_points(ranges, amin, inc, 0.15, 12.0, laser_pose=LASER_POSE)
        points = apply_lane_gate(apply_roi(points, y_abs_max=1.5), path.xy, params.lane_half_width, 0.15)
        tracks = tracker.update(detect_obstacles(points, inc), dt)
        for o in sim_obstacles:
            o.advance(dt)
    frenet = []
    for track in tracks:
        shape = track.shape
        if shape.shape == "box":
            samples = box_corners(*track.position, shape.yaw, shape.length, shape.width)
        else:
            samples = circle_samples(*track.position, shape.radius)
        s_vals, d_vals = path.project(samples)
        _x, _y, heading, _k = path.pose_at([float(np.mean(s_vals))])
        h = float(heading[0])
        vx, vy = track.velocity
        frenet.append(obstacle_box_to_frenet(
            track.track_id, s_vals, d_vals,
            vs=vx * math.cos(h) + vy * math.sin(h), vd=-vx * math.sin(h) + vy * math.cos(h),
            is_dynamic=track.is_dynamic,
        ))
    stations = np.arange(int(round(params.horizon_m / params.ds)) + 1) * params.ds
    fsm = BehaviorFSM(FsmParams(v_cruise=params.v_cruise))
    lead, lead_gap = find_lead_obstacle(frenet, params.lane_half_width)
    out = None
    corridor = None
    t = 0.0
    while t < 4.0:
        # Same rule as the planner node: a same-direction lead is handled by
        # the ACC law (FOLLOW), not by the lateral corridor.
        ignore = (lead.obstacle_id,) if lead is not None else ()
        corridor = generate_corridor(
            frenet, params, ego_d=path.ego_d, v_ego=v_ego, kappa=path.kappa_at(stations),
            ignore_ids=ignore, previous_side=corridor.side if corridor else 0,
        )
        out = fsm.step(FsmInputs(
            now=t, lane_valid=True, corridor_feasible=corridor.feasible,
            blocked_at_s=corridor.blocked_at_s, obstacle_influences=corridor.obstacle_count > 0,
            lead_gap_m=lead_gap, lead_speed_mps=0.0 if lead is None else lead.vs,
            corridor_side=corridor.side,
        ))
        t += 0.05
    corridor.v_max = np.minimum(corridor.v_max, out.v_ref)
    return {
        "key": key, "title": title, "path": path, "points": points, "tracks": tracks,
        "frenet": frenet, "corridor": corridor, "fsm": out, "sim": sim_obstacles,
    }


# ------------------------------------------------------------------ rendering
def _svg(result, params, width_px=520, height_px=760, scale=200.0):
    path = result["path"]
    corridor = result["corridor"]
    ox = width_px / 2.0
    oy = height_px - 90.0

    def to_px(x, y):
        return ox - y * scale, oy - x * scale

    def frenet_xy(s, d):
        x, y, heading, _k = path.pose_at([s])
        h = float(heading[0])
        return float(x[0] - d * math.sin(h)), float(y[0] + d * math.cos(h))

    def polyline(pairs, stroke, width=2, dash=None, fill="none"):
        pts = " ".join(f"{px:.1f},{py:.1f}" for px, py in pairs)
        extra = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<polyline points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"{extra}/>'

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width_px}" height="{height_px}" '
        f'viewBox="0 0 {width_px} {height_px}">',
        '<rect width="100%" height="100%" fill="#1e1e24"/>',
    ]
    lane = [to_px(*frenet_xy(s, params.lane_half_width)) for s in corridor.s]
    lane += [to_px(*frenet_xy(s, -params.lane_half_width)) for s in corridor.s[::-1]]
    out.append(polyline(lane + lane[:1], "#f0c419", 3, fill="#2b2b33"))
    out.append(polyline([to_px(*frenet_xy(s, 0.0)) for s in corridor.s], "#6fa8ff", 1, dash="6,6"))
    fill_pts = [to_px(*frenet_xy(s, d)) for s, d in zip(corridor.s, corridor.lower) if True]
    fill_pts += [to_px(*frenet_xy(s, d)) for s, d in zip(corridor.s[::-1], corridor.upper[::-1])]
    color = "#2ecc71" if corridor.feasible else "#e74c3c"
    out.append(polyline(fill_pts + fill_pts[:1], color, 2, fill=color + "40"))
    out.append(polyline([to_px(*frenet_xy(s, d)) for s, d in zip(corridor.s, corridor.d_ref)], "#ffffff", 2))
    if corridor.blocked_at_s >= 0.0:
        a = to_px(*frenet_xy(corridor.blocked_at_s, -0.5))
        b = to_px(*frenet_xy(corridor.blocked_at_s, 0.5))
        out.append(polyline([a, b], "#ff3b3b", 4))
    if result["points"] is not None:
        for x, y in result["points"].xy:
            px, py = to_px(x, y)
            out.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="1.6" fill="#ff9f43"/>')
    for track in result["tracks"]:
        shape = track.shape
        px, py = to_px(*track.position)
        r = max(shape.radius, 0.04) * scale
        out.append(
            f'<circle cx="{px:.1f}" cy="{py:.1f}" r="{r:.1f}" fill="none" stroke="#ff5e57" stroke-width="2"/>'
            f'<text x="{px + r + 4:.1f}" y="{py:.1f}" fill="#ffd1d1" font-size="12">#{track.track_id}'
            f'{" dyn" if track.is_dynamic else ""}</text>'
        )
    # ego footprint
    corners = box_corners(0.0, 0.0, 0.0, params.front_overhang + params.rear_overhang, params.vehicle_width)
    out.append(polyline([to_px(*c) for c in corners] + [to_px(*corners[0])], "#9b59b6", 2, fill="#9b59b650"))
    fsm = result["fsm"]
    lines = [
        f"{result['key'].upper()}: {result['title']}",
        f"state={STATE_NAMES[fsm.state]} ({fsm.reason})  v_ref={fsm.v_ref:.2f} m/s",
        f"feasible={corridor.feasible} side={corridor.side} blocked_at={corridor.blocked_at_s:.2f} m",
        f"min_width={corridor.min_free_width:.2f} m  obstacles={corridor.obstacle_count}  "
        f"v_max(0)={corridor.v_max[0]:.2f}",
    ]
    for i, line in enumerate(lines):
        out.append(f'<text x="12" y="{20 + 16 * i}" fill="#eeeeee" font-size="13" font-family="monospace">{line}</text>')
    out.append("</svg>")
    return "\n".join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    parser.add_argument("--output", default="/tmp/corridor_demo")
    args = parser.parse_args(argv)
    params = CorridorParams()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    keys = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    summary = []
    for key in keys:
        result = run_scenario(key, params)
        (output / f"{key}.svg").write_text(_svg(result, params), encoding="utf-8")
        corridor = result["corridor"]
        row = {
            "scenario": key, "title": result["title"], "state": STATE_NAMES[result["fsm"].state],
            "feasible": bool(corridor.feasible), "side": int(corridor.side),
            "blocked_at_s": round(float(corridor.blocked_at_s), 3),
            "min_free_width": round(float(corridor.min_free_width), 3),
            "tracks": len(result["tracks"]), "v_ref": round(float(result["fsm"].v_ref), 3),
        }
        summary.append(row)
        print(json.dumps(row, ensure_ascii=True))  # console-safe on Windows
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"wrote {len(keys)} scenario(s) to {output}")


if __name__ == "__main__":
    main()

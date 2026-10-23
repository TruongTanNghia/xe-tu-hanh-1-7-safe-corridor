"""Offline end-to-end scenarios S1-S5 from the design document, section 12.

synthetic scan -> ROI/lane gate -> clustering -> tracking -> Frenet
projection -> corridor -> behaviour FSM, without ROS.
"""

import numpy as np

from rc_safe_corridor.behavior_fsm import (
    AVOID,
    LANE_FOLLOW,
    STOP,
    BehaviorFSM,
    FsmInputs,
    FsmParams,
)
from rc_safe_corridor.corridor import (
    SIDE_LEFT,
    SIDE_RIGHT,
    CorridorParams,
    generate_corridor,
)
from rc_safe_corridor.frenet import straight_reference_path
from rc_safe_corridor.geometry import box_corners, circle_samples
from rc_safe_corridor.scan_clustering import (
    apply_lane_gate,
    apply_roi,
    detect_obstacles,
    scan_to_points,
)
from rc_safe_corridor.synthetic import SimObstacle, synthetic_scan
from rc_safe_corridor.tracking import MultiObjectTracker, TrackerParams
from rc_safe_corridor.corridor import obstacle_box_to_frenet


LASER_POSE = (-0.005, 0.0, 0.0)
CORRIDOR = CorridorParams()


def _perceive(sim_obstacles, tracker, frames=5, dt=0.15):
    path = straight_reference_path(horizon_m=2.5)
    tracks = []
    for _ in range(frames):
        ranges, amin, inc = synthetic_scan(sim_obstacles, laser_pose=LASER_POSE)
        pts = scan_to_points(ranges, amin, inc, 0.15, 12.0, laser_pose=LASER_POSE)
        pts = apply_lane_gate(apply_roi(pts), path.xy, 0.45, 0.15)
        tracks = tracker.update(detect_obstacles(pts, inc), dt)
    frenet = []
    for track in tracks:
        shape = track.shape
        if shape.shape == "box":
            samples = box_corners(*track.position, shape.yaw, shape.length, shape.width)
        else:
            samples = circle_samples(*track.position, shape.radius)
        s_vals, d_vals = path.project(samples)
        frenet.append(obstacle_box_to_frenet(
            track.track_id, s_vals, d_vals, is_dynamic=track.is_dynamic))
    return frenet, path


def _settled_fsm():
    fsm = BehaviorFSM(FsmParams())
    t = 0.0
    while t < 3.0:
        fsm.step(FsmInputs(now=t, lane_valid=True, corridor_feasible=True,
                           blocked_at_s=-1.0, obstacle_influences=False))
        t += 0.05
    return fsm, t


def _fsm_after(fsm, t, corridor, seconds=1.0):
    out = None
    end = t + seconds
    while t < end:
        out = fsm.step(FsmInputs(
            now=t, lane_valid=True, corridor_feasible=corridor.feasible,
            blocked_at_s=corridor.blocked_at_s,
            obstacle_influences=corridor.obstacle_count > 0,
            corridor_side=corridor.side,
        ))
        t += 0.05
    return out


def test_s1_no_obstacle_keeps_full_lane_corridor():
    frenet, path = _perceive([], MultiObjectTracker(TrackerParams()))
    assert frenet == []
    corridor = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d)
    assert corridor.feasible and corridor.obstacle_count == 0
    assert np.allclose(corridor.d_ref, 0.0)
    fsm, t = _settled_fsm()
    assert _fsm_after(fsm, t, corridor).state == LANE_FOLLOW


def test_s2_static_obstacle_right_lane_is_avoided_on_the_left():
    frenet, path = _perceive(
        [SimObstacle(x=1.3, y=-0.22, radius=0.05)], MultiObjectTracker(TrackerParams()))
    assert len(frenet) == 1
    corridor = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d)
    assert corridor.feasible
    assert corridor.side == SIDE_LEFT
    idx = int(np.argmin(np.abs(corridor.s - 1.3)))
    assert corridor.d_ref[idx] > 0.05
    assert np.all(corridor.v_max > 0.0)
    fsm, t = _settled_fsm()
    out = _fsm_after(fsm, t, corridor)
    assert out.state == AVOID and out.avoid_side == SIDE_LEFT


def test_s4_slalom_switches_side_once_without_hesitation():
    frenet, path = _perceive(
        [SimObstacle(x=0.9, y=-0.22, radius=0.05), SimObstacle(x=1.9, y=0.22, radius=0.05)],
        MultiObjectTracker(TrackerParams()),
    )
    assert len(frenet) == 2
    corridor = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d)
    assert corridor.feasible
    sign = np.sign(np.where(np.abs(corridor.d_ref) < 0.02, 0.0, corridor.d_ref))
    nonzero = sign[sign != 0]
    switches = int(np.count_nonzero(np.diff(nonzero)))
    assert switches == 1
    assert corridor.d_ref[int(np.argmin(np.abs(corridor.s - 0.9)))] > 0
    assert corridor.d_ref[int(np.argmin(np.abs(corridor.s - 1.9)))] < 0
    # Re-planning with the chosen side keeps the decision.
    again = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d, previous_side=corridor.side)
    assert again.side == corridor.side


def test_s5_lane_blocked_by_wide_box_stops_before_it():
    frenet, path = _perceive(
        [SimObstacle(x=1.5, y=0.0, length=0.2, width=0.9)], MultiObjectTracker(TrackerParams()))
    assert len(frenet) == 1
    corridor = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d)
    assert not corridor.feasible
    assert 0.6 < corridor.blocked_at_s < 1.4
    stop_s = corridor.blocked_at_s - CORRIDOR.stop_margin
    near = int(np.argmin(np.abs(corridor.s - (stop_s - 0.1))))
    assert corridor.v_max[near] < CORRIDOR.v_cruise
    assert corridor.v_max[-1] == 0.0
    fsm, t = _settled_fsm()
    out = _fsm_after(fsm, t, corridor)
    assert out.state == STOP and out.v_ref == 0.0


def test_s3_obstacle_on_left_in_curve_is_passed_right_without_crossing_lane():
    frenet, path = _perceive(
        [SimObstacle(x=1.2, y=0.22, radius=0.05)], MultiObjectTracker(TrackerParams()))
    kappa = np.full(int(round(CORRIDOR.horizon_m / CORRIDOR.ds)) + 1, 1.0 / 0.7)
    corridor = generate_corridor(frenet, CORRIDOR, ego_d=path.ego_d, kappa=kappa)
    assert corridor.feasible and corridor.side == SIDE_RIGHT
    assert np.all(corridor.lower >= -0.45 + 0.15 + 0.03 - 1e-9)
    assert np.all(corridor.upper <= 0.45 - 0.15 - 0.03 + 1e-9)
    assert np.all(corridor.v_max <= np.sqrt(CORRIDOR.a_lat_max * 0.7) + 1e-9)

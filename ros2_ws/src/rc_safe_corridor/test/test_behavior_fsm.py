from rc_safe_corridor.behavior_fsm import (
    AVOID,
    ESTOP,
    FOLLOW,
    LANE_FOLLOW,
    STOP,
    BehaviorFSM,
    FsmInputs,
    FsmParams,
    find_lead_obstacle,
)
from rc_safe_corridor.corridor import FrenetObstacle


def _inputs(now, **overrides):
    base = dict(
        now=now, lane_valid=True, corridor_feasible=True, blocked_at_s=-1.0,
        obstacle_influences=False, lead_gap_m=-1.0, lead_speed_mps=0.0,
        estop=False, corridor_side=0,
    )
    base.update(overrides)
    return FsmInputs(**base)


def _run(fsm, t0, t1, dt=0.05, **overrides):
    out = None
    t = t0
    while t <= t1 + 1e-9:
        out = fsm.step(_inputs(t, **overrides))
        t += dt
    return out


def test_startup_stop_then_lane_follow_after_clear_time():
    fsm = BehaviorFSM(FsmParams(min_dwell_s=0.5, stop_clear_time_s=1.5, v_cruise=0.6))
    out = _run(fsm, 0.0, 1.0)
    assert out.state == STOP and out.v_ref == 0.0
    out = _run(fsm, 1.05, 2.3)
    assert out.state == LANE_FOLLOW
    assert out.v_ref == 0.6


def test_obstacle_triggers_avoid_and_returns_after_clear_time():
    fsm = BehaviorFSM(FsmParams(min_dwell_s=0.5, clear_time_s=1.0))
    _run(fsm, 0.0, 2.5)
    out = _run(fsm, 2.55, 3.0, obstacle_influences=True, corridor_side=1)
    assert out.state == AVOID and out.avoid_side == 1
    out = _run(fsm, 3.05, 3.5)
    assert out.state == AVOID  # hysteresis: clear timer not elapsed
    out = _run(fsm, 3.55, 4.5)
    assert out.state == LANE_FOLLOW


def test_blocked_corridor_stops_and_resumes():
    fsm = BehaviorFSM(FsmParams(min_dwell_s=0.5, stop_clear_time_s=1.0))
    _run(fsm, 0.0, 2.5)
    out = _run(fsm, 2.55, 3.0, corridor_feasible=False, blocked_at_s=0.8)
    assert out.state == STOP and out.v_ref == 0.0
    out = _run(fsm, 3.05, 3.4)
    assert out.state == STOP
    out = _run(fsm, 3.45, 5.0)
    assert out.state == LANE_FOLLOW


def test_estop_latches_until_cleared():
    fsm = BehaviorFSM(FsmParams())
    _run(fsm, 0.0, 2.5)
    out = fsm.step(_inputs(2.55, estop=True))
    assert out.state == ESTOP
    out = _run(fsm, 2.6, 3.5, estop=True)
    assert out.state == ESTOP
    out = _run(fsm, 3.55, 4.1)
    assert out.state == STOP


def test_lane_lost_leads_to_stop_after_timeout():
    fsm = BehaviorFSM(FsmParams(lane_lost_timeout_s=0.3))
    _run(fsm, 0.0, 2.5)
    out = _run(fsm, 2.55, 2.7, lane_valid=False)
    assert out.state == LANE_FOLLOW
    out = _run(fsm, 2.75, 3.0, lane_valid=False)
    assert out.state == STOP and out.reason == "lane_lost"


def test_follow_uses_acc_speed_law():
    fsm = BehaviorFSM(FsmParams(v_cruise=0.6, follow_min_gap_m=0.5,
                                follow_time_headway_s=1.0, follow_gain=0.8))
    _run(fsm, 0.0, 2.5)
    out = _run(fsm, 2.55, 3.0, lead_gap_m=1.0, lead_speed_mps=0.3)
    assert out.state == FOLLOW
    # gap_des = 0.5 + 0.3 = 0.8 ; v = 0.3 + 0.8*(1.0-0.8) = 0.46
    assert abs(out.v_ref - 0.46) < 1e-9
    out = _run(fsm, 3.05, 3.5, lead_gap_m=0.4, lead_speed_mps=0.3)
    assert out.v_ref < 0.3


def test_find_lead_obstacle_filters_crossing_and_static():
    lead = FrenetObstacle(1, 1.0, 1.1, -0.05, 0.05, vs=0.3, vd=0.0, is_dynamic=True)
    crossing = FrenetObstacle(2, 0.8, 0.9, -0.05, 0.05, vs=0.1, vd=0.4, is_dynamic=True)
    static = FrenetObstacle(3, 0.6, 0.7, -0.05, 0.05, is_dynamic=False)
    obs, gap = find_lead_obstacle([static, crossing, lead], lane_half_width=0.45)
    assert obs is lead
    assert gap == 1.0
    obs, gap = find_lead_obstacle([static], lane_half_width=0.45)
    assert obs is None and gap < 0.0

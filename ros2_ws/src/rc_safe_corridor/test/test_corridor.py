import numpy as np

from rc_safe_corridor.corridor import (
    SIDE_CENTER,
    SIDE_LEFT,
    SIDE_RIGHT,
    CorridorParams,
    FrenetObstacle,
    generate_corridor,
)


PARAMS = CorridorParams(
    ds=0.05, horizon_m=2.5, vehicle_width=0.30, lane_half_width=0.45,
    margin_lane=0.03, margin_obstacle=0.08, v_cruise=0.6,
)


def _obs(obstacle_id, s_center, d_center, size=0.10, **kwargs):
    return FrenetObstacle(
        obstacle_id=obstacle_id,
        s_min=s_center - size / 2, s_max=s_center + size / 2,
        d_min=d_center - size / 2, d_max=d_center + size / 2, **kwargs,
    )


def test_free_lane_corridor_is_full_lane_minus_margins():
    result = generate_corridor([], PARAMS)
    assert result.feasible
    assert result.blocked_at_s < 0.0
    assert result.side == SIDE_CENTER
    expected = 0.45 - 0.15 - 0.03
    assert np.allclose(result.lower, -expected)
    assert np.allclose(result.upper, expected)
    assert np.allclose(result.d_ref, 0.0, atol=1e-9)
    assert np.allclose(result.v_max, 0.6)
    assert result.obstacle_count == 0


def test_obstacle_on_right_is_passed_on_the_left():
    # Obstacle in the right lane (d = -0.22) on a straight.
    result = generate_corridor([_obs(1, 1.2, -0.22)], PARAMS, ego_d=0.0)
    assert result.feasible
    assert result.side == SIDE_LEFT
    idx = int(np.argmin(np.abs(result.s - 1.2)))
    # Obstacle d_max = -0.17 ; inflated by 0.15 + 0.08 -> lower >= 0.06
    assert result.lower[idx] >= 0.06 - 1e-9
    assert result.d_ref[idx] > result.lower[idx]
    assert result.d_ref[idx] <= result.upper[idx]
    # Near the ego the corridor still contains the lane centre.
    assert result.lower[0] <= 0.0 <= result.upper[0]
    assert result.obstacle_count == 1
    # The funnel never narrows faster than the configured slope.
    assert np.all(np.diff(result.lower) <= PARAMS.max_lateral_slope * PARAMS.ds + 1e-9)


def test_obstacle_on_left_is_passed_on_the_right():
    result = generate_corridor([_obs(1, 1.2, 0.22)], PARAMS)
    assert result.feasible
    assert result.side == SIDE_RIGHT
    idx = int(np.argmin(np.abs(result.s - 1.2)))
    assert result.d_ref[idx] < 0.0


def test_previous_side_is_kept_when_both_sides_are_similar():
    # A centred obstacle on the 0.9 m track blocks both sides for a 0.3 m
    # car; use a wider drivable band to exercise the keep-side logic.
    wide = CorridorParams(lane_half_width=0.65)
    centered = _obs(1, 1.2, 0.0, size=0.08)
    left = generate_corridor([centered], wide, previous_side=SIDE_LEFT)
    right = generate_corridor([centered], wide, previous_side=SIDE_RIGHT)
    assert left.side == SIDE_LEFT
    assert right.side == SIDE_RIGHT


def test_lane_fully_blocked_reports_stop_point_and_zero_speed():
    wall = FrenetObstacle(1, s_min=1.5, s_max=1.6, d_min=-0.5, d_max=0.5)
    result = generate_corridor([wall], PARAMS)
    assert not result.feasible
    # forbidden starts at 1.5 - rear_overhang - margin_s = 1.08
    assert 1.0 <= result.blocked_at_s <= 1.15
    stop_s = result.blocked_at_s - PARAMS.stop_margin
    assert np.all(result.v_max[result.s >= stop_s] == 0.0)
    near = int(np.argmin(np.abs(result.s - (stop_s - 0.1))))
    assert result.v_max[near] < 0.6
    # Braking profile is monotone and feasible.
    assert np.all(np.diff(result.v_max) <= 1e-9)


def test_slalom_two_staggered_obstacles_switch_side_once():
    obstacles = [_obs(1, 0.9, -0.22), _obs(2, 1.9, 0.22)]
    result = generate_corridor(obstacles, PARAMS)
    assert result.feasible
    d_near = result.d_ref[int(np.argmin(np.abs(result.s - 0.9)))]
    d_far = result.d_ref[int(np.argmin(np.abs(result.s - 1.9)))]
    assert d_near > 0.0 and d_far < 0.0
    # Reference stays inside bounds everywhere.
    assert np.all(result.d_ref >= result.lower - 1e-9)
    assert np.all(result.d_ref <= result.upper + 1e-9)


def test_dynamic_crossing_obstacle_uses_predicted_position():
    # Obstacle currently at d = +0.6 (outside lane) moving right at 0.5 m/s.
    crossing = FrenetObstacle(
        1, s_min=1.15, s_max=1.25, d_min=0.55, d_max=0.65,
        vs=0.0, vd=-0.5, is_dynamic=True,
    )
    static_result = generate_corridor([FrenetObstacle(
        1, 1.15, 1.25, 0.55, 0.65)], PARAMS, v_ego=0.6)
    dynamic_result = generate_corridor([crossing], PARAMS, v_ego=0.6)
    assert static_result.obstacle_count == 0
    assert dynamic_result.obstacle_count == 1
    # The predicted crossing meets the ego around s = 0.75 m (t = 1.25 s):
    # the static placement is harmless, the prediction blocks the lane.
    assert static_result.feasible
    assert not dynamic_result.feasible
    assert 0.5 < dynamic_result.blocked_at_s < 1.2


def test_ignored_ids_are_not_subtracted():
    result = generate_corridor([_obs(7, 1.0, 0.0)], PARAMS, ignore_ids=(7,))
    assert result.obstacle_count == 0
    assert result.feasible


def test_curvature_limits_speed():
    kappa = np.full(int(round(2.5 / 0.05)) + 1, 1.0 / 0.7)
    result = generate_corridor([], PARAMS, kappa=kappa)
    assert np.all(result.v_max <= np.sqrt(PARAMS.a_lat_max * 0.7) + 1e-9)


def test_bounds_at_interpolates():
    result = generate_corridor([_obs(1, 1.2, -0.22)], PARAMS)
    lower, upper, d_ref, v_max = result.bounds_at([0.0, 1.2, 5.0])
    assert lower.shape == (3,)
    assert lower[1] > lower[0]
    assert upper[2] == result.upper[-1]

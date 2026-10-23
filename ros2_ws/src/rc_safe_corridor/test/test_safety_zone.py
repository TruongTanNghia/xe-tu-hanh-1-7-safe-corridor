import numpy as np

from rc_safe_corridor.safety_zone import (
    SafetyParams,
    brake_distance,
    evaluate_brake_zone,
    points_in_brake_zone,
)


def test_brake_distance_grows_with_speed_and_is_clamped():
    params = SafetyParams(a_brake=1.5, t_delay=0.25, d0=0.2)
    assert brake_distance(0.0, params) == params.min_zone_length
    d = brake_distance(0.6, params)
    assert abs(d - (0.36 / 3.0 + 0.15 + 0.2)) < 1e-9
    assert brake_distance(10.0, params) == params.max_zone_length


def test_points_in_zone_respects_rectangle():
    params = SafetyParams(front_overhang=0.32, half_width=0.17)
    points = np.array([
        [0.40, 0.00],   # inside
        [0.50, 0.10],   # inside
        [0.50, 0.30],   # too far left
        [0.20, 0.00],   # behind bumper (own body)
        [1.50, 0.00],   # beyond zone
    ])
    assert points_in_brake_zone(points, 0.6, params) == 2


def test_evaluate_triggers_on_min_points_and_stale_odom_assumes_fast():
    params = SafetyParams(min_points=3, v_assumed_if_odom_stale=0.8)
    cluster = np.array([[0.45, 0.0], [0.46, 0.01], [0.47, -0.01]])
    trigger, count, _ = evaluate_brake_zone(cluster, 0.3, params)
    assert trigger and count == 3
    far = np.array([[0.80, 0.0], [0.81, 0.0], [0.82, 0.0]])
    trigger, _, zone = evaluate_brake_zone(far, 0.3, params)
    assert not trigger
    trigger, _, zone_stale = evaluate_brake_zone(far, 0.3, params, odom_fresh=False)
    assert zone_stale > zone
    assert trigger

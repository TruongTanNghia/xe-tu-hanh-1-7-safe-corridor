import math

import numpy as np

from rc_safe_corridor.frenet import (
    LaneSmoother,
    build_reference_path,
    straight_reference_path,
)


def _lane_from_arc(radius, start_forward=0.35, length=0.8, n=12, lateral0=0.0):
    """Centre line = circular arc of given signed radius starting at
    (start_forward, lateral0) heading +x. Returns lane-like arrays."""
    kappa = 0.0 if radius == 0 else 1.0 / radius
    s = np.linspace(0.0, length, n)
    if kappa == 0.0:
        lateral = np.full(n, lateral0)
    else:
        lateral = lateral0 + (1.0 - np.cos(kappa * s)) / kappa
    return s, np.full(n, kappa), lateral


def test_straight_lane_gives_zero_offset_and_heading():
    s, kappa, lateral = _lane_from_arc(0.0)
    path = build_reference_path(
        s, kappa, cte=0.0, heading_error=0.0, evaluation_forward=0.35,
        path_lateral=lateral, horizon_m=2.0,
    )
    assert path.valid
    assert abs(path.ego_d) < 1e-6
    assert abs(path.ego_heading_error) < 1e-6
    assert path.s[0] < 0.0 < path.s[-1]
    assert path.s[-1] >= 2.0
    x, y, heading, kappa_q = path.pose_at([0.0, 1.0, 2.0])
    assert np.allclose(y, 0.0, atol=1e-6)
    assert np.allclose(x, [0.0, 1.0, 2.0], atol=1e-6)


def test_offset_lane_is_reported_in_ego_d_with_left_positive():
    s, kappa, lateral = _lane_from_arc(0.0, lateral0=0.15)
    path = build_reference_path(
        s, kappa, cte=-0.15, heading_error=0.0, evaluation_forward=0.35,
        path_lateral=lateral,
    )
    # Path is 0.15 m to the left of the ego -> ego is 0.15 m right of path.
    assert abs(path.ego_d + 0.15) < 1e-6
    s_q, d_q = path.project(np.array([[1.0, 0.15], [1.0, 0.45]]))
    assert np.allclose(d_q, [0.0, 0.30], atol=1e-6)
    assert np.allclose(s_q, [1.0, 1.0], atol=1e-6)


def test_without_path_lateral_curvature_is_integrated():
    s, kappa, _lateral = _lane_from_arc(0.7)  # R700 left turn
    path = build_reference_path(
        s, kappa, cte=0.0, heading_error=0.0, evaluation_forward=0.35,
        path_lateral=None, horizon_m=2.0,
    )
    x, y, heading, kappa_q = path.pose_at([1.5])
    assert y[0] > 0.3          # bends to the left
    assert heading[0] > 0.3
    assert abs(kappa_q[0] - 1.0 / 0.7) < 1e-6


def test_heading_error_sign_matches_nmpc_convention():
    # Lane heading error +0.1 rad means the path heading is -0.1 rad in
    # base_link (path turns right relative to the car).
    s, kappa, lateral = _lane_from_arc(0.0)
    path = build_reference_path(
        s, kappa, cte=0.0, heading_error=0.1, evaluation_forward=0.35,
        path_lateral=None,
    )
    assert abs(path.ego_heading_error - 0.1) < 1e-3


def test_fallback_straight_path():
    path = straight_reference_path(horizon_m=1.0, ds=0.1)
    assert not path.valid
    assert path.s[-1] >= 1.0 - 1e-9


def test_lane_smoother_filters_and_resets_on_shape_change():
    smoother = LaneSmoother(alpha=0.5)
    first = smoother.update(0.0, 0.0, np.zeros(3), 0.3)
    second = smoother.update(0.2, 0.1, np.ones(3), 0.3)
    assert math.isclose(second[0], 0.1)
    assert math.isclose(second[1], 0.05)
    assert np.allclose(second[2], 0.5)
    third = smoother.update(1.0, 1.0, np.ones(5), 0.3)
    assert math.isclose(third[0], 1.0)
    assert first[0] == 0.0

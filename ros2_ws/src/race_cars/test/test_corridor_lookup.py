import numpy as np

from race_cars.corridor_lookup import (
    clamp_bounds,
    predicted_stage_s,
    sample_corridor,
    validate_corridor_arrays,
)


def _corridor():
    s = np.arange(0.0, 2.5 + 1e-9, 0.05)
    lower = np.full(s.shape, -0.27)
    upper = np.full(s.shape, 0.27)
    lower[(s >= 1.0) & (s <= 1.6)] = 0.06      # obstacle on the right
    d_ref = np.where((s >= 1.0) & (s <= 1.6), 0.15, 0.0)
    v_max = np.full(s.shape, 0.6)
    kappa = np.zeros(s.shape)
    return s, lower, upper, d_ref, v_max, kappa


def test_validate_accepts_good_arrays_and_rejects_bad_ones():
    s, lower, upper, d_ref, v_max, kappa = _corridor()
    ok, err = validate_corridor_arrays(s, lower, upper, d_ref, v_max, kappa)
    assert ok and err == ""
    ok, err = validate_corridor_arrays(s, lower[:-1], upper, d_ref, v_max)
    assert not ok and "length" in err
    bad_upper = upper.copy()
    bad_upper[3] = -0.5
    ok, err = validate_corridor_arrays(s, lower, bad_upper, d_ref, v_max)
    assert not ok and "lower > upper" in err
    ok, _ = validate_corridor_arrays(s, lower, upper, d_ref, np.r_[v_max[:-1], np.nan])
    assert not ok


def test_predicted_stage_s_without_history_is_constant_speed():
    s_pred = predicted_stage_s(None, 10, 0.13, 0.6, 0.05)
    assert s_pred.shape == (11,)
    assert np.allclose(s_pred, np.arange(11) * 0.13 * 0.6)


def test_predicted_stage_s_shifts_previous_solution():
    previous = np.zeros((11, 6))
    previous[:, 0] = np.arange(11) * 0.1
    s_pred = predicted_stage_s(previous, 10, 0.13, 0.5, 0.05)
    assert s_pred[0] == 0.0
    assert np.isclose(s_pred[1], 0.1 - 0.025)
    assert np.isclose(s_pred[-1], 1.0 - 0.025)
    # Malformed history falls back to the guess.
    assert np.allclose(
        predicted_stage_s(previous[:5], 10, 0.13, 0.5, 0.05),
        predicted_stage_s(None, 10, 0.13, 0.5, 0.05),
    )


def test_sample_corridor_uses_front_and_rear_lookups():
    s, lower, upper, d_ref, v_max, kappa = _corridor()
    sample = sample_corridor(
        s, lower, upper, d_ref, v_max, kappa, s_query=np.array([0.5, 0.8, 1.3, 5.0]),
        front_offset=0.32, rear_offset=0.32,
    )
    # At s = 0.8 the reference point is free but the nose (1.12) is beside
    # the obstacle, so the front bound is already raised.
    assert np.isclose(sample["lower"][1], -0.27)
    assert sample["lower_front"][1] > 0.0
    assert np.isclose(sample["lower"][2], 0.06)
    assert np.isclose(sample["d_ref"][2], 0.15)
    # Beyond the horizon everything clamps to the last station.
    assert np.isclose(sample["upper"][3], upper[-1])
    assert np.isclose(sample["v_max"][3], 0.6)
    assert sample["kappa"].shape == (4,)


def test_clamp_bounds_respects_hard_limits_and_keeps_gap():
    lo, hi = clamp_bounds([-0.9, 0.2, 0.1], [0.9, 0.2, 0.05], -0.45, 0.45)
    assert np.allclose(lo[0], -0.45) and np.allclose(hi[0], 0.45)
    assert hi[1] - lo[1] >= 1e-3 - 1e-12
    assert hi[2] - lo[2] >= 1e-3 - 1e-12

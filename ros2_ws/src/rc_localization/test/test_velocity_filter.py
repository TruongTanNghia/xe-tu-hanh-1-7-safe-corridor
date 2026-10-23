import pytest

from rc_localization.velocity_filter import (
    PlanarPoseSpeedEstimator,
    RobustVelocityFilter,
    VelocitySampleError,
)


def make_filter(**overrides):
    parameters = {
        "median_window": 3,
        "ema_alpha": 0.5,
        "zero_threshold_mps": 0.02,
        "max_abs_speed_mps": 1.5,
        "max_acceleration_mps2": 3.0,
        "reset_gap_s": 0.5,
        "variance_window": 10,
        "variance_floor": 0.0009,
    }
    parameters.update(overrides)
    return RobustVelocityFilter(**parameters)


def test_stationary_noise_is_zeroed_and_covariance_has_floor():
    velocity_filter = make_filter()
    outputs = []
    for index, value in enumerate([0.01, -0.01, 0.015, -0.005]):
        filtered, variance = velocity_filter.update(value, 1.0 + index * 0.1)
        outputs.append(filtered)
        assert variance >= 0.0009
    assert outputs == [0.0, 0.0, 0.0, 0.0]


def test_median_rejects_single_spike_before_ema():
    velocity_filter = make_filter(max_acceleration_mps2=20.0)
    values = [0.2, 0.21, 1.0, 0.2]
    outputs = [
        velocity_filter.update(value, 1.0 + index * 0.1)[0]
        for index, value in enumerate(values)
    ]
    assert outputs[2] < 0.3
    assert outputs[3] < 0.3


def test_invalid_timestamp_and_acceleration_are_rejected():
    velocity_filter = make_filter()
    velocity_filter.update(0.0, 1.0)
    with pytest.raises(VelocitySampleError, match="timestamp"):
        velocity_filter.update(0.0, 1.0)
    with pytest.raises(VelocitySampleError, match="acceleration"):
        velocity_filter.update(1.0, 1.1)


def test_long_gap_resets_filter_instead_of_rejecting_step():
    velocity_filter = make_filter()
    velocity_filter.update(0.0, 1.0)
    filtered, _ = velocity_filter.update(0.4, 2.0)
    assert filtered == pytest.approx(0.4)


def test_pose_estimator_uses_planar_distance_and_raw_direction():
    estimator = PlanarPoseSpeedEstimator(reset_gap_s=0.5)
    assert estimator.update(
        x_m=0.0,
        y_m=0.0,
        raw_vx_mps=0.0,
        stamp_s=1.0,
    ) == 0.0
    assert estimator.update(
        x_m=0.0,
        y_m=0.02,
        raw_vx_mps=0.001,
        stamp_s=1.1,
    ) == pytest.approx(0.2)
    assert estimator.update(
        x_m=0.0,
        y_m=0.01,
        raw_vx_mps=-0.001,
        stamp_s=1.2,
    ) == pytest.approx(-0.1)


def test_pose_estimator_resets_across_gap_and_rejects_bad_timestamp():
    estimator = PlanarPoseSpeedEstimator(reset_gap_s=0.5)
    estimator.update(
        x_m=0.0,
        y_m=0.0,
        raw_vx_mps=0.0,
        stamp_s=1.0,
    )
    assert estimator.update(
        x_m=1.0,
        y_m=1.0,
        raw_vx_mps=0.03,
        stamp_s=2.0,
    ) == pytest.approx(0.03)
    with pytest.raises(VelocitySampleError, match="timestamp"):
        estimator.update(
            x_m=1.0,
            y_m=1.0,
            raw_vx_mps=0.0,
            stamp_s=2.0,
        )

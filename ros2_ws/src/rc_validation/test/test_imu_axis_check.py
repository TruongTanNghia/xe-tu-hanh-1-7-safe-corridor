import pytest

from rc_validation.imu_axis_check import evaluate_gyro_direction


def samples(value):
    return [(index * 0.01, value) for index in range(100)]


def test_ccw_positive_gyro_passes():
    result = evaluate_gyro_direction(samples(0.5), "ccw")
    assert result["passed"]
    assert result["direction_fraction"] == 1.0
    assert result["integrated_angle_rad"] == pytest.approx(0.495)


def test_ccw_negative_gyro_fails_direction_and_angle():
    result = evaluate_gyro_direction(samples(-0.5), "ccw")
    assert not result["passed"]
    assert "gyro_z_direction_inconsistent" in result["failures"]
    assert "integrated_rotation_too_small_or_wrong_sign" in result["failures"]


def test_stationary_data_fails_active_motion_gate():
    result = evaluate_gyro_direction(samples(0.001), "ccw")
    assert not result["passed"]
    assert "too_few_active_rotation_samples" in result["failures"]


def test_small_stop_jitter_does_not_flip_direction_result():
    data = []
    for index in range(100):
        data.append((index * 0.01, 0.5))
    for index in range(100, 200):
        data.append((index * 0.01, -0.03))

    result = evaluate_gyro_direction(data, "ccw")

    assert result["passed"]
    assert result["direction_fraction"] == 1.0

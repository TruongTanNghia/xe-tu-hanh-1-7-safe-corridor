import pytest

from rc_actuation.actuation_mapper import ActuationMapper


def test_steering_mapping_and_clamp():
    mapper = ActuationMapper(servo_us_per_rad=250.0)
    assert mapper.steering_to_pwm(0.0) == 1500
    assert mapper.steering_to_pwm(0.2) == 1550
    assert mapper.steering_to_pwm(-0.2) == 1450
    assert mapper.steering_to_pwm(10.0) == 1850
    assert mapper.steering_to_pwm(-10.0) == 1150


def test_drive_mapping_has_explicit_deadband_jump_and_no_reverse():
    mapper = ActuationMapper()
    assert mapper.effort_to_target_pwm(-1.0) == 1550
    assert mapper.effort_to_target_pwm(0.0) == 1550
    assert mapper.effort_to_target_pwm(0.01) >= 1580
    assert mapper.effort_to_target_pwm(1.0) == 1600
    assert mapper.effort_to_target_pwm(2.0) == 1600


def test_esc_stop_is_immediate_and_rise_is_limited():
    mapper = ActuationMapper(esc_rise_rate_us_per_s=50.0)
    assert mapper.apply_esc_slew(1550, 1600, 0.1) == 1555
    assert mapper.apply_esc_slew(1590, 1550, 0.1) == 1550


def test_non_finite_input_is_rejected():
    mapper = ActuationMapper()
    with pytest.raises(ValueError):
        mapper.steering_to_pwm(float("nan"))
    with pytest.raises(ValueError):
        mapper.effort_to_target_pwm(float("inf"))

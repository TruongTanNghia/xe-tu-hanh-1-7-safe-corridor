from rc_actuation.esc_step_source import EscStepSchedule


def test_rolling_schedule_uses_same_pwm_steps():
    schedule = EscStepSchedule([0.01, 0.25, 0.50, 0.75, 1.0], 3.0, 5.0)
    assert schedule.at(3.0).label == "step_1580us"
    assert schedule.at(11.0).label == "step_1585us"

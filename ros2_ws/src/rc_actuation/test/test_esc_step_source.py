from rc_actuation.esc_step_source import EscStepSchedule


def test_schedule_alternates_stop_and_expected_pwm_steps():
    schedule = EscStepSchedule([0.01, 0.25, 0.5, 0.75, 1.0], 3.0, 5.0)
    assert schedule.at(0.0).label == "stop"
    assert schedule.at(3.0).label == "step_1580us"
    assert schedule.at(8.0).label == "stop"
    assert schedule.at(11.0).label == "step_1585us"
    assert schedule.at(schedule.total_duration_s) is None


def test_schedule_rejects_unsafe_values():
    for efforts in ([], [0.0], [-0.1], [1.1]):
        try:
            EscStepSchedule(efforts, 1.0, 1.0)
        except ValueError:
            pass
        else:
            raise AssertionError(f"expected ValueError for {efforts}")

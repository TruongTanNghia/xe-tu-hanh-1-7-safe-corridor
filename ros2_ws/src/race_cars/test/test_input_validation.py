import math

from race_cars.input_validation import finite_state, validate_lane_arrays


def test_valid_lane_arrays():
    assert validate_lane_arrays(
        [0.0, 0.2, 0.4],
        [0.0, 0.1, 0.2],
    ) == (True, "")


def test_rejects_short_mismatched_or_non_monotonic_arrays():
    assert not validate_lane_arrays([0.0], [0.0])[0]
    assert not validate_lane_arrays([0.0, 0.2], [0.0])[0]
    assert not validate_lane_arrays([0.0, 0.2, 0.1], [0.0, 0.0, 0.0])[0]


def test_rejects_non_finite_values():
    assert not validate_lane_arrays([0.0, math.nan], [0.0, 0.0])[0]
    assert not validate_lane_arrays([0.0, 0.2], [0.0, math.inf])[0]
    assert not finite_state(0.0, math.nan)

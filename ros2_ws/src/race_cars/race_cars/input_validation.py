import math


def validate_lane_arrays(s_values, curvature_values):
    if len(s_values) < 2 or len(curvature_values) < 2:
        return False, "lane arrays require at least two points"
    if len(s_values) != len(curvature_values):
        return False, "s and curvature arrays have different lengths"
    if not all(math.isfinite(float(value)) for value in s_values):
        return False, "s contains non-finite values"
    if not all(math.isfinite(float(value)) for value in curvature_values):
        return False, "curvature contains non-finite values"
    if abs(float(s_values[0])) > 1e-3:
        return False, "s must start at zero"
    if any(
        float(current) <= float(previous)
        for previous, current in zip(s_values, s_values[1:])
    ):
        return False, "s must be strictly increasing"
    return True, ""


def finite_state(*values):
    return all(math.isfinite(float(value)) for value in values)

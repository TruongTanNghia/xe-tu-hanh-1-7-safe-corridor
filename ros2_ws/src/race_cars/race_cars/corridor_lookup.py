"""Pure helpers that map a Corridor message onto NMPC stages (block D).

Kept free of ROS/acados imports so they can be unit-tested anywhere.
"""

import math

import numpy as np


def validate_corridor_arrays(s, lower, upper, d_ref, v_max, kappa=None):
    arrays = {"s": s, "lower": lower, "upper": upper, "d_ref": d_ref, "v_max": v_max}
    if kappa is not None and len(kappa) > 0:
        arrays["kappa"] = kappa
    n = len(s)
    if n < 2:
        return False, "corridor needs at least two stations"
    for name, values in arrays.items():
        if len(values) != n:
            return False, f"{name} length {len(values)} != {n}"
        if not all(math.isfinite(float(v)) for v in values):
            return False, f"{name} contains non-finite values"
    if any(float(b) <= float(a) for a, b in zip(s, s[1:])):
        return False, "s must be strictly increasing"
    if any(float(lo) > float(up) + 1e-9 for lo, up in zip(lower, upper)):
        return False, "lower > upper"
    return True, ""


def predicted_stage_s(previous_states, horizon_steps, stage_dt, speed,
                      control_period, min_speed=0.05):
    """Expected s at every stage for the next solve.

    previous_states: (N+1, nx) trajectory from the last solve (s is column 0)
    or None. The car advanced ~speed * control_period since, so the previous
    prediction is shifted back by that amount; without a previous solution a
    constant-speed guess is used.
    """
    stages = int(horizon_steps) + 1
    v = max(float(speed), float(min_speed))
    guess = np.arange(stages) * float(stage_dt) * v
    if previous_states is None:
        return guess
    previous = np.asarray(previous_states, dtype=float)
    if previous.ndim != 2 or previous.shape[0] != stages:
        return guess
    shifted = previous[:, 0] - previous[0, 0] - v * float(control_period)
    shifted = np.maximum(shifted, 0.0)
    if not np.all(np.isfinite(shifted)) or np.any(np.diff(shifted) < -1e-6):
        return guess
    return np.maximum(shifted, guess * 0.0)


def sample_corridor(s, lower, upper, d_ref, v_max, kappa, s_query,
                    front_offset=0.0, rear_offset=0.0):
    """Interpolate corridor quantities at the predicted stage positions.

    Returns a dict with per-stage arrays for the reference point (`lower`,
    `upper`), the front and rear footprint circles (`lower_front`, ...),
    `d_ref`, `v_max` and `kappa`. Queries beyond the last station clamp to it.
    """
    s = np.asarray(s, dtype=float)
    s_query = np.asarray(s_query, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)

    def interp(values, query):
        return np.interp(query, s, np.asarray(values, dtype=float))

    front = s_query + float(front_offset)
    rear = np.maximum(s_query - float(rear_offset), s[0])
    result = {
        "lower": interp(lower, s_query),
        "upper": interp(upper, s_query),
        "lower_front": interp(lower, front),
        "upper_front": interp(upper, front),
        "lower_rear": interp(lower, rear),
        "upper_rear": interp(upper, rear),
        "d_ref": interp(d_ref, s_query),
        "v_max": interp(v_max, s_query),
    }
    if kappa is not None and len(kappa) == len(s):
        result["kappa"] = interp(kappa, s_query)
    else:
        result["kappa"] = np.zeros_like(s_query)
    return result


def clamp_bounds(lower, upper, n_min, n_max, min_gap=1e-3):
    """Keep corridor bounds inside the hard lane limits and non-crossing."""
    lo = np.clip(np.asarray(lower, dtype=float), n_min, n_max)
    hi = np.clip(np.asarray(upper, dtype=float), n_min, n_max)
    crossed = hi - lo < min_gap
    mid = 0.5 * (lo + hi)
    lo = np.where(crossed, mid - 0.5 * min_gap, lo)
    hi = np.where(crossed, mid + 0.5 * min_gap, hi)
    return lo, hi

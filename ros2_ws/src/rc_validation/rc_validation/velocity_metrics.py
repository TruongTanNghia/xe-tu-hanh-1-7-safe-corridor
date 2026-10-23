import math
import statistics


def percentile(values, percentile_value):
    if not values:
        return None
    if not 0.0 <= percentile_value <= 100.0:
        raise ValueError("percentile must be between 0 and 100")
    ordered = sorted(float(value) for value in values)
    index = math.ceil((percentile_value / 100.0) * len(ordered)) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def calculate_metrics(samples, expected_speed_mps, spike_threshold_mps):
    values = [float(sample["value_mps"]) for sample in samples]
    if not values:
        return {
            "count": 0,
            "mean_mps": None,
            "stddev_mps": None,
            "p95_abs_error_mps": None,
            "max_abs_error_mps": None,
            "spike_count": 0,
            "sample_rate_hz": 0.0,
        }

    errors = [abs(value - expected_speed_mps) for value in values]
    duration_s = (
        float(samples[-1]["elapsed_s"]) - float(samples[0]["elapsed_s"])
        if len(samples) > 1
        else 0.0
    )
    return {
        "count": len(values),
        "mean_mps": statistics.mean(values),
        "stddev_mps": statistics.pstdev(values),
        "p95_abs_error_mps": percentile(errors, 95.0),
        "max_abs_error_mps": max(errors),
        "spike_count": sum(
            error > spike_threshold_mps for error in errors
        ),
        "sample_rate_hz": (
            (len(values) - 1) / duration_s if duration_s > 0.0 else 0.0
        ),
    }


def evaluate_filtered_metrics(metrics, expected_speed_mps):
    if metrics["count"] == 0:
        return False, ["missing_samples"]

    failures = []
    mean_error = abs(metrics["mean_mps"] - expected_speed_mps)
    if abs(expected_speed_mps) <= 1e-9:
        if abs(metrics["mean_mps"]) > 0.02:
            failures.append("stationary_mean_gt_0.02_mps")
        if metrics["p95_abs_error_mps"] > 0.05:
            failures.append("stationary_p95_gt_0.05_mps")
    else:
        if metrics["stddev_mps"] > 0.03:
            failures.append("moving_stddev_gt_0.03_mps")
        allowed_mean_error = max(0.03, 0.15 * abs(expected_speed_mps))
        if mean_error > allowed_mean_error:
            failures.append("moving_mean_error_gt_15_percent")

    if metrics["spike_count"] > 0:
        failures.append("velocity_spike_detected")

    return not failures, failures

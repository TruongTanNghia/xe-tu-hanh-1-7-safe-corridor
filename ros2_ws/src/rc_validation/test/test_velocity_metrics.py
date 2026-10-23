import pytest

from rc_validation.velocity_metrics import (
    calculate_metrics,
    evaluate_filtered_metrics,
)


def samples(values, dt=0.1):
    return [
        {"elapsed_s": index * dt, "value_mps": value}
        for index, value in enumerate(values)
    ]


def test_stationary_acceptance_passes_stable_zero():
    metrics = calculate_metrics(samples([0.0] * 20), 0.0, 0.5)
    passed, failures = evaluate_filtered_metrics(metrics, 0.0)
    assert passed
    assert failures == []


def test_stationary_acceptance_rejects_bias():
    metrics = calculate_metrics(samples([0.04] * 20), 0.0, 0.5)
    passed, failures = evaluate_filtered_metrics(metrics, 0.0)
    assert not passed
    assert "stationary_mean_gt_0.02_mps" in failures


def test_moving_acceptance_and_rate():
    metrics = calculate_metrics(
        samples([0.19, 0.20, 0.21, 0.20] * 5),
        0.2,
        0.5,
    )
    passed, failures = evaluate_filtered_metrics(metrics, 0.2)
    assert passed
    assert failures == []
    assert metrics["sample_rate_hz"] == pytest.approx(10.0)


def test_spike_is_reported():
    metrics = calculate_metrics(samples([0.2, 0.9, 0.2]), 0.2, 0.5)
    passed, failures = evaluate_filtered_metrics(metrics, 0.2)
    assert not passed
    assert metrics["spike_count"] == 1
    assert "velocity_spike_detected" in failures

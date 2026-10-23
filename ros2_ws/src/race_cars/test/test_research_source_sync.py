from pathlib import Path

import yaml

from race_cars.acados_settings import (
    CONTROL_RATE_WEIGHT_DIAGONAL,
    STAGE_STATE_WEIGHT_DIAGONAL,
    TERMINAL_STATE_WEIGHT_DIAGONAL,
)
from race_cars.bicycle_model import DEFAULT_MODEL_PARAMETERS


RESEARCH_SOURCE_REVISION = "0b0723b"


def load_hardware_parameters():
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "nmpc_hardware.yaml"
    )
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    return config["racecar_nmpc_node"]["ros__parameters"]


def test_runtime_parameters_match_current_research_source():
    parameters = load_hardware_parameters()
    assert parameters["target_speed_mps"] == 0.6
    assert parameters["control_rate_hz"] == 20.0
    assert parameters["prediction_horizon_s"] == 1.3
    assert parameters["horizon_steps"] == 10
    assert parameters["minimum_line_confidence"] == 0.5
    assert parameters["line_timeout_s"] == 0.5


def test_model_and_cost_parameters_match_current_research_source():
    parameters = load_hardware_parameters()
    expected_model = {
        "mass_kg": 6.5,
        "c1": 0.5,
        "c2": 15.5,
        "cm1": 0.720766449,
        "cm2": 1.77195852,
        "cr0": 0.0763306087,
        "cr2": 0.450571997,
        "cr3": 20.0,
        "cte_limit_m": 0.45,
        "drive_effort_max": 1.0,
        "drive_effort_rate_min": -12.0,
        "drive_effort_rate_max": 12.0,
        "steering_rate_min_radps": -2.5,
        "steering_rate_max_radps": 2.5,
    }
    for name, expected in expected_model.items():
        assert parameters[f"model.{name}"] == expected
        assert DEFAULT_MODEL_PARAMETERS[name] == expected

    assert STAGE_STATE_WEIGHT_DIAGONAL == (
        0.5,
        1.0,
        5.0,
        0.5,
        1e-3,
        1e-5,
    )
    assert CONTROL_RATE_WEIGHT_DIAGONAL == (3e-4, 5e-4)
    assert TERMINAL_STATE_WEIGHT_DIAGONAL == (
        2.0,
        30.0,
        10.0,
        0.5,
        1e-3,
        1e-9,
    )


def test_hardware_physical_bounds_remain_explicit():
    parameters = load_hardware_parameters()
    assert RESEARCH_SOURCE_REVISION == "0b0723b"
    assert parameters["shadow_mode"] is False
    assert parameters["model_calibrated"] is True
    hardware_bounds = {
        "drive_effort_min": 0.0,
        "steering_min_rad": -0.524,
        "steering_max_rad": 0.524,
        "longitudinal_accel_min_mps2": -2.0,
        "longitudinal_accel_max_mps2": 2.0,
        "lateral_accel_min_mps2": -2.0,
        "lateral_accel_max_mps2": 2.0,
    }
    for name, expected in hardware_bounds.items():
        assert parameters[f"model.{name}"] == expected
        assert DEFAULT_MODEL_PARAMETERS[name] == expected

import csv

import numpy as np

from rc_validation.longitudinal_fit import (
    fit_model,
    load_transitions,
    predict_next_velocity,
    write_report,
)


def test_fit_synthetic_data_passes_heldout_gate(tmp_path):
    rng = np.random.default_rng(7)
    parameters = np.array([7.0, 2.0, 0.15, 0.08, 4.0])
    mass = 6.5
    delta_time = 0.05
    velocity = 0.0
    rows = []
    for index in range(420):
        effort = (index // 70) / 6.0
        rows.append(
            {
                "time_s": index * delta_time,
                "drive_effort": effort,
                "vx_mps": velocity,
                "armed": 1,
            }
        )
        velocity = float(
            predict_next_velocity(
                parameters,
                mass,
                velocity,
                effort,
                delta_time,
            )
            + rng.normal(0.0, 0.001)
        )

    csv_path = tmp_path / "samples.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    transitions = load_transitions(csv_path)
    report = fit_model(transitions, mass_kg=mass)
    assert report["passed"]
    assert report["heldout_velocity_rmse_mps"] < 0.01


def test_load_requires_enough_valid_transitions(tmp_path):
    csv_path = tmp_path / "short.csv"
    csv_path.write_text(
        "time_s,drive_effort,vx_mps,armed\n0,0,0,1\n0.05,0,0,1\n",
        encoding="utf-8",
    )
    try:
        load_transitions(csv_path)
    except ValueError as error:
        assert "at least 50" in str(error)
    else:
        raise AssertionError("expected ValueError")


def test_candidate_yaml_keeps_integer_valued_parameters_as_float(tmp_path):
    report = {
        "mass_kg": 6.5,
        "parameters": {
            "cm1": 0.7207664486051971,
            "cm2": 1.7719585173072712,
            "cr0": 0.07633060869494737,
            "cr2": 0.45057199741220827,
            "cr3": 20.0,
        },
        "sample_count": 184,
        "train_velocity_rmse_mps": 0.021743,
        "heldout_velocity_rmse_mps": 0.007438,
        "passed": True,
    }
    output_dir = tmp_path / "model"
    write_report(report, output_dir)
    candidate = (output_dir / "nmpc_model_candidate.yaml").read_text(
        encoding="utf-8"
    )
    assert "model.cr3: 20.0" in candidate

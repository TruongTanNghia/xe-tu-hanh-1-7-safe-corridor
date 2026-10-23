#!/usr/bin/env python3

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


PARAMETER_NAMES = ("cm1", "cm2", "cr0", "cr2", "cr3")


def format_yaml_float(value):
    """Format a numeric YAML parameter explicitly as a floating-point value."""
    formatted = f"{float(value):.9g}"
    if "." not in formatted and "e" not in formatted.lower():
        formatted += ".0"
    return formatted


def predict_next_velocity(parameters, mass_kg, velocity, effort, delta_time):
    cm1, cm2, cr0, cr2, cr3 = parameters
    force = (
        (cm1 - cm2 * velocity) * effort
        - cr2 * velocity * velocity
        - cr0 * np.tanh(cr3 * velocity)
    )
    return velocity + delta_time * force / mass_kg


def load_transitions(csv_path):
    rows = []
    with Path(csv_path).open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            values = (
                float(row["time_s"]),
                float(row["drive_effort"]),
                float(row["vx_mps"]),
                int(row.get("armed", "1")),
            )
            if all(math.isfinite(value) for value in values[:3]):
                rows.append(values)
    rows.sort(key=lambda item: item[0])

    transitions = []
    for current, following in zip(rows, rows[1:]):
        delta_time = following[0] - current[0]
        if (
            current[3]
            and following[3]
            and 0.005 <= delta_time <= 0.25
            and 0.0 <= current[1] <= 1.0
        ):
            transitions.append(
                (current[2], current[1], delta_time, following[2])
            )
    if len(transitions) < 50:
        raise ValueError(
            f"need at least 50 valid armed transitions, got {len(transitions)}"
        )
    return np.asarray(transitions, dtype=float)


def fit_model(transitions, mass_kg=6.5, heldout_fraction=0.2):
    if mass_kg <= 0.0:
        raise ValueError("mass_kg must be positive")
    if not 0.1 <= heldout_fraction <= 0.5:
        raise ValueError("heldout_fraction must be in [0.1, 0.5]")

    split = int(len(transitions) * (1.0 - heldout_fraction))
    train = transitions[:split]
    heldout = transitions[split:]
    if len(train) < 40 or len(heldout) < 10:
        raise ValueError("not enough train/held-out transitions")

    def residual(parameters, data):
        prediction = predict_next_velocity(
            parameters,
            mass_kg,
            data[:, 0],
            data[:, 1],
            data[:, 2],
        )
        return prediction - data[:, 3]

    result = least_squares(
        lambda parameters: residual(parameters, train),
        x0=np.array([1.0, 1.0, 0.1, 0.1, 5.0]),
        bounds=(
            np.array([0.0, 0.0, 0.0, 0.0, 0.1]),
            np.array([100.0, 100.0, 20.0, 20.0, 20.0]),
        ),
        max_nfev=20000,
    )
    train_rmse = float(np.sqrt(np.mean(residual(result.x, train) ** 2)))
    heldout_rmse = float(
        np.sqrt(np.mean(residual(result.x, heldout) ** 2))
    )
    return {
        "success": bool(result.success),
        "message": result.message,
        "sample_count": int(len(transitions)),
        "train_count": int(len(train)),
        "heldout_count": int(len(heldout)),
        "mass_kg": float(mass_kg),
        "parameters": {
            name: float(value)
            for name, value in zip(PARAMETER_NAMES, result.x)
        },
        "train_velocity_rmse_mps": train_rmse,
        "heldout_velocity_rmse_mps": heldout_rmse,
        "gate_rmse_mps": 0.04,
        "passed": bool(result.success and heldout_rmse <= 0.04),
    }


def write_report(report, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "model_fit.json").write_text(
        json.dumps(report, indent=2) + "\n",
        encoding="utf-8",
    )
    parameters = report["parameters"]
    yaml_lines = [
        "# Copy only after reviewing evidence and held-out PASS.",
        "racecar_nmpc_node:",
        "  ros__parameters:",
        f"    model.mass_kg: {format_yaml_float(report['mass_kg'])}",
    ]
    for name in PARAMETER_NAMES:
        yaml_lines.append(
            f"    model.{name}: {format_yaml_float(parameters[name])}"
        )
    yaml_lines.append("    model_calibrated: true")
    (output_dir / "nmpc_model_candidate.yaml").write_text(
        "\n".join(yaml_lines) + "\n",
        encoding="utf-8",
    )
    result = "PASS" if report["passed"] else "FAIL"
    (output_dir / "report.md").write_text(
        "\n".join(
            [
                "# Báo cáo fit model dọc",
                "",
                f"- Kết quả: **{result}**",
                f"- Samples: `{report['sample_count']}`",
                (
                    "- Train velocity RMSE: "
                    f"`{report['train_velocity_rmse_mps']:.6f} m/s`"
                ),
                (
                    "- Held-out velocity RMSE: "
                    f"`{report['heldout_velocity_rmse_mps']:.6f} m/s`"
                ),
                "- Gate: `≤ 0.04 m/s`",
                "",
                "Candidate YAML chỉ được merge sau review evidence.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--mass-kg", type=float, default=6.5)
    parser.add_argument("--heldout-fraction", type=float, default=0.2)
    arguments = parser.parse_args()
    transitions = load_transitions(arguments.csv_path)
    report = fit_model(
        transitions,
        mass_kg=arguments.mass_kg,
        heldout_fraction=arguments.heldout_fraction,
    )
    write_report(report, arguments.output_dir)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(2)

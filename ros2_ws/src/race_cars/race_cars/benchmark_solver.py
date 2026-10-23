#!/usr/bin/env python3

import argparse
import json
import time

import numpy as np

from race_cars.acados_settings import acados_settings
from race_cars.bicycle_model import DEFAULT_MODEL_PARAMETERS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--output", default="")
    arguments = parser.parse_args()
    if arguments.iterations < 10:
        raise ValueError("iterations must be at least 10")

    build_started = time.perf_counter()
    _, _, solver = acados_settings(1.3, 10, DEFAULT_MODEL_PARAMETERS)
    build_time_s = time.perf_counter() - build_started

    initial_state = np.array([0.0, 0.0, 0.0, 0.2, 0.2, 0.0])
    durations_ms = []
    failures = 0
    for _ in range(arguments.iterations):
        solver.set(0, "lbx", initial_state)
        solver.set(0, "ubx", initial_state)
        started = time.perf_counter()
        status = solver.solve()
        durations_ms.append((time.perf_counter() - started) * 1000.0)
        failures += int(status != 0)

    values = np.asarray(durations_ms)
    report = {
        "iterations": arguments.iterations,
        "failures": failures,
        "solver_build_time_s": build_time_s,
        "mean_ms": float(np.mean(values)),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(np.max(values)),
        "deadline_ms": 50.0,
        "passed": failures == 0 and float(np.percentile(values, 99)) < 50.0,
    }
    text = json.dumps(report, indent=2)
    print(text)
    if arguments.output:
        with open(arguments.output, "w", encoding="utf-8") as stream:
            stream.write(text + "\n")
    if not report["passed"]:
        raise SystemExit(2)

# Derived from the acados race-car example (BSD-2-Clause).

import types

from casadi import Function, MX, cos, sin, tanh, vertcat
import numpy as np


DEFAULT_MODEL_PARAMETERS = {
    "mass_kg": 6.5,
    "c1": 0.5,
    "c2": 15.5,
    "cm1": 0.720766449,
    "cm2": 1.77195852,
    "cr0": 0.0763306087,
    "cr2": 0.450571997,
    "cr3": 20.0,
    "cte_limit_m": 0.45,
    "drive_effort_min": 0.0,
    "drive_effort_max": 1.0,
    "steering_min_rad": -0.524,
    "steering_max_rad": 0.524,
    "drive_effort_rate_min": -12.0,
    "drive_effort_rate_max": 12.0,
    "steering_rate_min_radps": -2.5,
    "steering_rate_max_radps": 2.5,
    "longitudinal_accel_min_mps2": -2.0,
    "longitudinal_accel_max_mps2": 2.0,
    "lateral_accel_min_mps2": -2.0,
    "lateral_accel_max_mps2": 2.0,
    # Safe-corridor footprint: lateral position of the nose/tail circle
    # centres is n +/- l * sin(alpha). Vehicle length 0.632 m, base_link at
    # the middle of the wheelbase.
    "footprint_front_m": 0.32,
    "footprint_rear_m": 0.32,
}


def bicycle_model(parameters=None):
    parameters = {
        **DEFAULT_MODEL_PARAMETERS,
        **(parameters or {}),
    }
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()

    mass = float(parameters["mass_kg"])
    c1 = float(parameters["c1"])
    c2 = float(parameters["c2"])
    cm1 = float(parameters["cm1"])
    cm2 = float(parameters["cm2"])
    cr0 = float(parameters["cr0"])
    cr2 = float(parameters["cr2"])
    cr3 = float(parameters["cr3"])
    if mass <= 0.0:
        raise ValueError("mass_kg must be positive")

    s = MX.sym("s")
    n = MX.sym("n")
    alpha = MX.sym("alpha")
    velocity = MX.sym("v")
    drive_effort = MX.sym("D")
    steering = MX.sym("delta")
    state = vertcat(s, n, alpha, velocity, drive_effort, steering)

    drive_effort_rate = MX.sym("derD")
    steering_rate = MX.sym("derDelta")
    control = vertcat(drive_effort_rate, steering_rate)

    state_dot = vertcat(
        MX.sym("sdot"),
        MX.sym("ndot"),
        MX.sym("alphadot"),
        MX.sym("vdot"),
        MX.sym("Ddot"),
        MX.sym("deltadot"),
    )
    kappa = MX.sym("kappa")

    longitudinal_force = (
        (cm1 - cm2 * velocity) * drive_effort
        - cr2 * velocity * velocity
        - cr0 * tanh(cr3 * velocity)
    )
    denominator = 1.0 - kappa * n
    s_dot = velocity * cos(alpha + c1 * steering) / denominator
    explicit_dynamics = vertcat(
        s_dot,
        velocity * sin(alpha + c1 * steering),
        velocity * c2 * steering - kappa * s_dot,
        longitudinal_force / mass * cos(c1 * steering),
        drive_effort_rate,
        steering_rate,
    )

    longitudinal_accel = longitudinal_force / mass
    lateral_accel = (
        c2 * velocity * velocity * steering
        + longitudinal_force * sin(c1 * steering) / mass
    )

    cte_limit = abs(float(parameters["cte_limit_m"]))
    if cte_limit <= 0.0:
        raise ValueError("cte_limit_m must be positive")
    footprint_front = abs(float(parameters["footprint_front_m"]))
    footprint_rear = abs(float(parameters["footprint_rear_m"]))
    n_front = n + footprint_front * sin(alpha)
    n_rear = n - footprint_rear * sin(alpha)

    model.name = "rc_spatial_bicycle"
    model.x = state
    model.xdot = state_dot
    model.u = control
    model.z = vertcat()
    model.p = vertcat(kappa)
    model.f_expl_expr = explicit_dynamics
    model.f_impl_expr = state_dot - explicit_dynamics
    model.x0 = np.zeros(6)

    model.n_min = -cte_limit
    model.n_max = cte_limit
    model.throttle_min = float(parameters["drive_effort_min"])
    model.throttle_max = float(parameters["drive_effort_max"])
    model.delta_min = float(parameters["steering_min_rad"])
    model.delta_max = float(parameters["steering_max_rad"])
    model.dthrottle_min = float(parameters["drive_effort_rate_min"])
    model.dthrottle_max = float(parameters["drive_effort_rate_max"])
    model.ddelta_min = float(parameters["steering_rate_min_radps"])
    model.ddelta_max = float(parameters["steering_rate_max_radps"])
    model.footprint_front = footprint_front
    model.footprint_rear = footprint_rear

    constraint.along_min = float(
        parameters["longitudinal_accel_min_mps2"]
    )
    constraint.along_max = float(
        parameters["longitudinal_accel_max_mps2"]
    )
    constraint.alat_min = float(parameters["lateral_accel_min_mps2"])
    constraint.alat_max = float(parameters["lateral_accel_max_mps2"])
    # Rows: 0 a_long, 1 a_lat, 2 n (reference point), 3 D, 4 delta,
    #       5 n_front, 6 n_rear. Rows 2, 5, 6 receive per-stage corridor
    #       bounds from the NMPC node; all three are soft (slack).
    constraint.expr = vertcat(
        longitudinal_accel,
        lateral_accel,
        n,
        drive_effort,
        steering,
        n_front,
        n_rear,
    )
    constraint.index_n = 2
    constraint.index_n_front = 5
    constraint.index_n_rear = 6
    constraint.alat = Function(
        "a_lat",
        [state, control, model.p],
        [lateral_accel],
    )
    return model, constraint

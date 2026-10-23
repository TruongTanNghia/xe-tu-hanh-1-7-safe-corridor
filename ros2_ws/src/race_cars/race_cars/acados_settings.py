# Derived from the acados race-car example (BSD-2-Clause).

import hashlib
import json
import os
from pathlib import Path

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import numpy as np
import scipy.linalg

from race_cars.bicycle_model import bicycle_model


STAGE_STATE_WEIGHT_DIAGONAL = (0.5, 1.0, 5.0, 0.5, 1e-3, 1e-5)
CONTROL_RATE_WEIGHT_DIAGONAL = (3e-4, 5e-4)
TERMINAL_STATE_WEIGHT_DIAGONAL = (
    2.0,
    30.0,
    10.0,
    0.5,
    1e-3,
    1e-9,
)
# Soft corridor constraints (rows n_front / n_rear): L1 + L2 slack weights.
# The original n slack keeps its 100 L1 weight so lane-only behaviour is
# unchanged when no corridor is active.
CORRIDOR_SLACK_L1 = 200.0
CORRIDOR_SLACK_L2 = 500.0
OCP_VARIANT = "corridor_footprint_v1"


def acados_settings(tf_seconds, horizon_steps, model_parameters=None):
    model_parameters = model_parameters or {}
    model, constraint = bicycle_model(model_parameters)
    ocp = AcadosOcp()

    acados_model = AcadosModel()
    acados_model.f_impl_expr = model.f_impl_expr
    acados_model.f_expl_expr = model.f_expl_expr
    acados_model.x = model.x
    acados_model.xdot = model.xdot
    acados_model.u = model.u
    acados_model.z = model.z
    acados_model.p = model.p
    acados_model.con_h_expr = constraint.expr
    acados_model.name = model.name
    ocp.model = acados_model

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_terminal = nx
    ocp.solver_options.N_horizon = int(horizon_steps)

    q_matrix = np.diag(STAGE_STATE_WEIGHT_DIAGONAL)
    r_matrix = np.diag(CONTROL_RATE_WEIGHT_DIAGONAL)
    q_terminal = np.diag(TERMINAL_STATE_WEIGHT_DIAGONAL)
    unscale = horizon_steps / tf_seconds

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = unscale * scipy.linalg.block_diag(q_matrix, r_matrix)
    ocp.cost.W_e = q_terminal / unscale

    vx = np.zeros((ny, nx))
    vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = vx
    vu = np.zeros((ny, nu))
    vu[nx:, :] = np.eye(nu)
    ocp.cost.Vu = vu
    vx_terminal = np.zeros((ny_terminal, nx))
    vx_terminal[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = vx_terminal

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_terminal)

    number_of_slacks = 4
    slack_l1 = np.array([100.0, 100.0, CORRIDOR_SLACK_L1, CORRIDOR_SLACK_L1])
    slack_l2 = np.array([0.0, 0.0, CORRIDOR_SLACK_L2, CORRIDOR_SLACK_L2])
    ocp.cost.zl = slack_l1.copy()
    ocp.cost.Zl = slack_l2.copy()
    ocp.cost.zu = slack_l1.copy()
    ocp.cost.Zu = slack_l2.copy()

    ocp.constraints.lbx = np.array([model.n_min])
    ocp.constraints.ubx = np.array([model.n_max])
    ocp.constraints.idxbx = np.array([1])
    ocp.constraints.lbu = np.array(
        [model.dthrottle_min, model.ddelta_min]
    )
    ocp.constraints.ubu = np.array(
        [model.dthrottle_max, model.ddelta_max]
    )
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lh = np.array(
        [
            constraint.along_min,
            constraint.alat_min,
            model.n_min,
            model.throttle_min,
            model.delta_min,
            model.n_min,
            model.n_min,
        ]
    )
    ocp.constraints.uh = np.array(
        [
            constraint.along_max,
            constraint.alat_max,
            model.n_max,
            model.throttle_max,
            model.delta_max,
            model.n_max,
            model.n_max,
        ]
    )
    ocp.constraints.lsh = np.zeros(number_of_slacks)
    ocp.constraints.ush = np.zeros(number_of_slacks)
    ocp.constraints.idxsh = np.array([0, 2, 5, 6])
    ocp.constraints.x0 = model.x0
    ocp.parameter_values = np.array([0.0])

    ocp.solver_options.tf = float(tf_seconds)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 4

    signature_source = {
        "tf_seconds": tf_seconds,
        "horizon_steps": horizon_steps,
        "model_parameters": model_parameters,
        "ocp_variant": OCP_VARIANT,
    }
    signature = hashlib.sha256(
        json.dumps(signature_source, sort_keys=True).encode("utf-8")
    ).hexdigest()[:12]
    cache_root = Path(
        os.environ.get(
            "RC_NMPC_CACHE_DIR",
            str(Path.home() / ".cache" / "rc_nmpc"),
        )
    )
    solver_root = cache_root / signature
    solver_root.mkdir(parents=True, exist_ok=True)
    ocp.code_export_directory = str(solver_root / "c_generated_code")
    json_path = solver_root / "acados_ocp.json"

    solver = AcadosOcpSolver(ocp, json_file=str(json_path))
    return constraint, model, solver

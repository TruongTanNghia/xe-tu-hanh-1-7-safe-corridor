"""Block B: local Frenet world model built from Lane Detection V2 output.

The lane node publishes, in base_link (+x forward, +y left):
  * s_m[]            arc length along the fitted centre line, s_m[0] = 0 at the
                     nearest projected scan band,
  * curvature_inv_m[] curvature at the same samples,
  * cte_m            = -lateral(centre line) at the evaluation point,
  * heading_error_rad = -atan(slope) at the evaluation point,
  * evaluation_forward_m  forward distance of the evaluation point,
  * path_lateral_m[] lateral offset of each sample (optional).

From these we rebuild the centre line as a polyline, extend it backwards to
cover the ego origin and forwards (constant curvature) to the planning
horizon, then re-parametrise by arc length with s = 0 at the projection of
base_link. Lateral coordinate d is positive to the left, matching the NMPC
state n (n = cte = -lateral(path)).
"""

from dataclasses import dataclass
import math

import numpy as np

from rc_safe_corridor.geometry import wrap_angle
from rc_safe_corridor.scan_clustering import lateral_distance_to_polyline


@dataclass
class ReferencePath:
    xy: np.ndarray            # (M,2) base_link
    s: np.ndarray             # (M,) arc length, 0 at ego projection
    heading: np.ndarray       # (M,) rad, tangent direction in base_link
    kappa: np.ndarray         # (M,) 1/m
    ego_d: float              # lateral offset of base_link from the path
    ego_heading_error: float  # ego heading minus path heading at s = 0
    measured_s_max: float     # last s actually observed by the camera
    valid: bool = True

    def pose_at(self, s_query):
        s_query = np.atleast_1d(np.asarray(s_query, dtype=float))
        x = np.interp(s_query, self.s, self.xy[:, 0])
        y = np.interp(s_query, self.s, self.xy[:, 1])
        heading = np.interp(s_query, self.s, np.unwrap(self.heading))
        kappa = np.interp(s_query, self.s, self.kappa)
        return x, y, heading, kappa

    def kappa_at(self, s_query):
        return np.interp(np.asarray(s_query, dtype=float), self.s, self.kappa)

    def project(self, points_xy):
        """Return (s, d) of points (N,2) in base_link."""
        d, s_local, _inside = lateral_distance_to_polyline(points_xy, self.xy)
        return s_local + float(self.s[0]), d

    def frenet_bounds(self, points_xy):
        """Project a set of boundary samples and return their (s, d) box."""
        s, d = self.project(points_xy)
        return float(s.min()), float(s.max()), float(d.min()), float(d.max())


def _integrate_curvature(x0, y0, theta0, kappa_values, ds_values):
    xs = [x0]
    ys = [y0]
    thetas = [theta0]
    theta = theta0
    x, y = x0, y0
    for step, kappa in zip(ds_values, kappa_values):
        theta_mid = theta + 0.5 * kappa * step
        x += step * math.cos(theta_mid)
        y += step * math.sin(theta_mid)
        theta += kappa * step
        xs.append(x)
        ys.append(y)
        thetas.append(theta)
    return np.array(xs), np.array(ys), np.array(thetas)


def build_reference_path(
    lane_s,
    lane_kappa,
    cte,
    heading_error,
    evaluation_forward=0.0,
    path_lateral=None,
    horizon_m=2.5,
    ds=0.05,
    back_m=0.6,
    kappa_clip=3.0,
    default_forward=0.35,
):
    """Build a ReferencePath from one LaneReference message."""
    lane_s = np.asarray(lane_s, dtype=float).ravel()
    lane_kappa = np.asarray(lane_kappa, dtype=float).ravel()
    if lane_s.size < 2 or lane_kappa.size != lane_s.size:
        raise ValueError("lane arrays must have >= 2 matching samples")
    lane_kappa = np.clip(lane_kappa, -abs(kappa_clip), abs(kappa_clip))

    x0 = float(evaluation_forward)
    if not math.isfinite(x0) or x0 <= 0.05:
        x0 = float(default_forward)
    theta0 = -float(heading_error)

    lateral = None
    if path_lateral is not None:
        lateral = np.asarray(path_lateral, dtype=float).ravel()
        if lateral.size != lane_s.size or not np.all(np.isfinite(lateral)):
            lateral = None
    y0 = float(lateral[0]) if lateral is not None else -float(cte)

    ds_meas = np.diff(lane_s)
    if lateral is not None:
        # Exact reconstruction: s was computed from the same polynomial.
        dy = np.diff(lateral)
        dx = np.sqrt(np.maximum(ds_meas**2 - dy**2, (0.25 * ds_meas) ** 2))
        xs = x0 + np.concatenate([[0.0], np.cumsum(dx)])
        ys = lateral.copy()
        thetas = np.concatenate(
            [[theta0], np.arctan2(dy, dx)]
        )
        thetas[0] = theta0
    else:
        kappa_mid = 0.5 * (lane_kappa[:-1] + lane_kappa[1:])
        xs, ys, thetas = _integrate_curvature(x0, y0, theta0, kappa_mid, ds_meas)
    kappas = lane_kappa.copy()
    measured_len = float(lane_s[-1] - lane_s[0])

    # Forward extrapolation with constant (end) curvature.
    kappa_end = float(kappas[-1])
    needed = float(horizon_m) + float(back_m) + x0 - measured_len
    n_fwd = int(max(0, math.ceil(needed / ds))) + 2
    fx, fy, ft = _integrate_curvature(
        xs[-1], ys[-1], thetas[-1], [kappa_end] * n_fwd, [ds] * n_fwd
    )
    # Backward extrapolation: straight line along the start tangent.
    n_back = int(math.ceil((x0 + float(back_m)) / ds)) + 1
    bx, by, bt = _integrate_curvature(
        xs[0], ys[0], thetas[0] + math.pi, [0.0] * n_back, [ds] * n_back
    )
    bx, by = bx[1:][::-1], by[1:][::-1]
    bt = np.full(bx.shape, thetas[0])

    all_x = np.concatenate([bx, xs, fx[1:]])
    all_y = np.concatenate([by, ys, fy[1:]])
    all_t = np.concatenate([bt, thetas, ft[1:]])
    all_k = np.concatenate(
        [np.full(bx.shape, float(kappas[0])), kappas, np.full(fx[1:].shape, kappa_end)]
    )
    seg = np.hypot(np.diff(all_x), np.diff(all_y))
    cum = np.concatenate([[0.0], np.cumsum(seg)])

    # Uniform resampling by arc length.
    s_uniform = np.arange(0.0, cum[-1], ds)
    rx = np.interp(s_uniform, cum, all_x)
    ry = np.interp(s_uniform, cum, all_y)
    rt = np.interp(s_uniform, cum, np.unwrap(all_t))
    rk = np.interp(s_uniform, cum, all_k)

    # Shift s so that 0 is the projection of base_link (0, 0).
    d_ego, s_ego, _ = lateral_distance_to_polyline(
        np.array([[0.0, 0.0]]), np.column_stack([rx, ry])
    )
    s_shifted = s_uniform - float(s_ego[0])
    heading_at_ego = float(np.interp(0.0, s_shifted, rt))
    measured_s_max = float(cum[len(bx) + len(xs) - 1] - float(s_ego[0]))
    return ReferencePath(
        xy=np.column_stack([rx, ry]),
        s=s_shifted,
        heading=rt,
        kappa=rk,
        ego_d=float(d_ego[0]),
        ego_heading_error=wrap_angle(0.0 - heading_at_ego),
        measured_s_max=measured_s_max,
        valid=True,
    )


def straight_reference_path(horizon_m=2.5, ds=0.05, back_m=0.6):
    """Fallback path along +x (used offline or when lane is unavailable)."""
    s = np.arange(-float(back_m), float(horizon_m) + ds, ds)
    xy = np.column_stack([s, np.zeros_like(s)])
    return ReferencePath(
        xy=xy,
        s=s,
        heading=np.zeros_like(s),
        kappa=np.zeros_like(s),
        ego_d=0.0,
        ego_heading_error=0.0,
        measured_s_max=float(horizon_m),
        valid=False,
    )


class LaneSmoother:
    """Exponential smoothing of the lane inputs to stop the corridor jumping."""

    def __init__(self, alpha=0.5):
        self.alpha = float(alpha)
        self._state = None

    def reset(self):
        self._state = None

    def update(self, cte, heading_error, kappa_array, evaluation_forward):
        kappa_array = np.asarray(kappa_array, dtype=float)
        sample = (float(cte), float(heading_error), kappa_array, float(evaluation_forward))
        if (
            self._state is None
            or self._state[2].shape != kappa_array.shape
            or self.alpha >= 1.0
        ):
            self._state = sample
            return sample
        a = self.alpha
        prev = self._state
        self._state = (
            a * sample[0] + (1 - a) * prev[0],
            a * sample[1] + (1 - a) * prev[1],
            a * sample[2] + (1 - a) * prev[2],
            a * sample[3] + (1 - a) * prev[3],
        )
        return self._state

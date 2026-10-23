"""Block C: safe corridor generation in the Frenet frame.

Input: lane free interval per station, obstacles as (s, d) boxes.
Output: per station [lower(s), upper(s)] for the vehicle reference point,
plus d_ref(s), v_max(s), chosen passing side and feasibility.

Steps (see design document section 6):
  1. discretise s_i = i * ds up to the horizon,
  2. start from the lane interval shrunk by w/2 + lane margin,
  3. inflate every obstacle by the vehicle half width + margin (and by the
     front/rear overhang along s), predict dynamic ones forward in time,
  4. subtract the forbidden intervals station by station,
  5. choose the homotopy (left/right) with dynamic programming, with a bonus
     for keeping the previous decision,
  6. smooth the bounds (funnel limited by the steering capability), build
     d_ref with minimal deviation from the lane centre and v_max from the
     corridor width, curvature and the stop point if blocked.
"""

from dataclasses import dataclass, field
import math

import numpy as np


SIDE_CENTER = 0
SIDE_LEFT = 1
SIDE_RIGHT = 2


@dataclass
class CorridorParams:
    ds: float = 0.05
    horizon_m: float = 2.5
    vehicle_width: float = 0.30
    front_overhang: float = 0.32     # base_link -> nose
    rear_overhang: float = 0.32      # base_link -> tail
    lane_half_width: float = 0.45    # drivable half width around the centre line
    margin_lane: float = 0.03
    margin_obstacle: float = 0.08
    margin_s: float = 0.10
    min_obstacle_size: float = 0.10  # LiDAR sees 1-2 faces only
    min_corridor_width: float = 0.02
    min_overlap: float = 0.04
    max_lateral_slope: float = 0.45  # tan(24 deg): d may change this much per metre of s
    cost_width: float = 1.0
    cost_center: float = 2.0
    cost_jump: float = 1.0
    cost_switch: float = 1.5
    keep_side_bonus: float = 0.6
    ref_margin: float = 0.05
    ref_center_blend: float = 0.5
    ref_smooth_window: int = 7
    v_cruise: float = 0.6
    v_min_ratio: float = 0.4
    nominal_width: float = 0.30
    a_brake: float = 1.0
    a_lat_max: float = 1.5
    stop_margin: float = 0.25
    v_ego_floor: float = 0.2
    dynamic_time_inflation: float = 0.10   # m per second of prediction
    max_prediction_s: float = 2.0          # cap on CV prediction time for dynamic obstacles


@dataclass
class FrenetObstacle:
    obstacle_id: int
    s_min: float
    s_max: float
    d_min: float
    d_max: float
    vs: float = 0.0
    vd: float = 0.0
    is_dynamic: bool = False
    confidence: float = 1.0


@dataclass
class Interval:
    lower: float
    upper: float
    side: int = SIDE_CENTER

    @property
    def width(self):
        return self.upper - self.lower

    @property
    def center(self):
        return 0.5 * (self.lower + self.upper)


@dataclass
class CorridorResult:
    s: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    d_ref: np.ndarray
    v_max: np.ndarray
    side: int
    feasible: bool
    blocked_at_s: float
    min_free_width: float
    obstacle_count: int
    ego_inside: bool
    kappa: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def bounds_at(self, s_query):
        s_query = np.asarray(s_query, dtype=float)
        return (
            np.interp(s_query, self.s, self.lower),
            np.interp(s_query, self.s, self.upper),
            np.interp(s_query, self.s, self.d_ref),
            np.interp(s_query, self.s, self.v_max),
        )


def _subtract(intervals, a, b, min_width):
    """Remove [a, b] from a list of Interval objects."""
    result = []
    for iv in intervals:
        if b <= iv.lower or a >= iv.upper:
            result.append(iv)
            continue
        if a > iv.lower:
            result.append(Interval(iv.lower, a, SIDE_RIGHT))
        if b < iv.upper:
            result.append(Interval(b, iv.upper, SIDE_LEFT))
    return [iv for iv in result if iv.width >= min_width]


def _forbidden_at_station(obs: FrenetObstacle, s_i, params: CorridorParams, v_ego):
    """Return (d_lo, d_hi) of the forbidden band at station s_i or None."""
    half_w = 0.5 * params.vehicle_width
    s_lo = obs.s_min
    s_hi = obs.s_max
    d_lo = obs.d_min
    d_hi = obs.d_max
    # Pad partially observed objects to a minimum physical size.
    if (s_hi - s_lo) < params.min_obstacle_size:
        pad = 0.5 * (params.min_obstacle_size - (s_hi - s_lo))
        s_lo -= pad
        s_hi += pad
    if (d_hi - d_lo) < params.min_obstacle_size:
        pad = 0.5 * (params.min_obstacle_size - (d_hi - d_lo))
        d_lo -= pad
        d_hi += pad
    extra = 0.0
    if obs.is_dynamic:
        # Forbid the swept region between now and the predicted position at
        # the time the ego reaches this station. Using the prediction alone
        # lets a slowly accelerating ego drive into an object that is still
        # physically in the lane; the union is conservative but safe.
        t_i = min(max(s_i, 0.0) / max(float(v_ego), params.v_ego_floor), params.max_prediction_s)
        s_lo = min(s_lo, s_lo + obs.vs * t_i)
        s_hi = max(s_hi, s_hi + obs.vs * t_i)
        d_lo = min(d_lo, d_lo + obs.vd * t_i)
        d_hi = max(d_hi, d_hi + obs.vd * t_i)
        extra = params.dynamic_time_inflation * t_i
    s_lo -= params.rear_overhang + params.margin_s
    s_hi += params.front_overhang + params.margin_s
    if s_i < s_lo or s_i > s_hi:
        return None
    inflate = half_w + params.margin_obstacle + extra
    return d_lo - inflate, d_hi + inflate


def _lane_interval(i, lane_left, lane_right, params: CorridorParams):
    half_w = 0.5 * params.vehicle_width
    left = params.lane_half_width if lane_left is None else float(lane_left[i])
    right = -params.lane_half_width if lane_right is None else float(lane_right[i])
    return Interval(right + half_w + params.margin_lane, left - half_w - params.margin_lane)


def _select_homotopy(stations, ego_d, previous_side, params: CorridorParams):
    """Dynamic programming over free intervals; returns chosen intervals and
    the index of the first blocked station (or -1)."""
    n = len(stations)
    best_cost = []
    back_ptr = []
    blocked_index = -1
    for i, nodes in enumerate(stations):
        if not nodes:
            blocked_index = i
            break
        costs = np.full(len(nodes), np.inf)
        ptrs = np.full(len(nodes), -1, dtype=int)
        for j, iv in enumerate(nodes):
            node_cost = (
                -params.cost_width * iv.width
                + params.cost_center * abs(iv.center)
            )
            if previous_side != SIDE_CENTER and iv.side == previous_side:
                node_cost -= params.keep_side_bonus
            if i == 0:
                # Prefer the interval that contains (or is nearest to) the ego.
                if iv.lower <= ego_d <= iv.upper:
                    costs[j] = node_cost
                else:
                    gap = min(abs(ego_d - iv.lower), abs(ego_d - iv.upper))
                    costs[j] = node_cost + 50.0 * gap
                continue
            for k, prev in enumerate(stations[i - 1]):
                if not np.isfinite(best_cost[i - 1][k]):
                    continue
                overlap = min(iv.upper, prev.upper) - max(iv.lower, prev.lower)
                if overlap < params.min_overlap:
                    continue
                edge = params.cost_jump * abs(iv.center - prev.center) / params.ds
                if (
                    iv.side != SIDE_CENTER
                    and prev.side != SIDE_CENTER
                    and iv.side != prev.side
                ):
                    edge += params.cost_switch
                total = best_cost[i - 1][k] + node_cost + edge
                if total < costs[j]:
                    costs[j] = total
                    ptrs[j] = k
        if not np.any(np.isfinite(costs)):
            blocked_index = i
            break
        best_cost.append(costs)
        back_ptr.append(ptrs)
    last = len(best_cost) - 1
    chosen = [None] * (last + 1)
    if last >= 0:
        j = int(np.argmin(best_cost[last]))
        for i in range(last, -1, -1):
            chosen[i] = stations[i][j]
            j = int(back_ptr[i][j]) if i > 0 else j
    return chosen, blocked_index


def _moving_average(values, window):
    window = int(max(1, window))
    if window == 1 or values.size < window:
        return values.copy()
    kernel = np.ones(window) / window
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def generate_corridor(
    obstacles,
    params: CorridorParams,
    ego_d=0.0,
    v_ego=0.0,
    lane_left=None,
    lane_right=None,
    previous_side=SIDE_CENTER,
    kappa=None,
    ignore_ids=(),
    v_cruise=None,
):
    """Build the corridor. obstacles: list[FrenetObstacle]."""
    v_cruise = params.v_cruise if v_cruise is None else float(v_cruise)
    n_stations = int(round(params.horizon_m / params.ds)) + 1
    s = np.arange(n_stations) * params.ds
    active = [o for o in obstacles if o.obstacle_id not in set(ignore_ids)]

    stations = []
    influencing = set()
    for i, s_i in enumerate(s):
        free = [_lane_interval(i, lane_left, lane_right, params)]
        if free[0].width < params.min_corridor_width:
            free = []
        for obs in active:
            band = _forbidden_at_station(obs, s_i, params, v_ego)
            if band is None:
                continue
            before = len(free)
            widths_before = sum(iv.width for iv in free)
            free = _subtract(free, band[0], band[1], params.min_corridor_width)
            if len(free) != before or sum(iv.width for iv in free) < widths_before - 1e-9:
                influencing.add(obs.obstacle_id)
        stations.append(free)

    chosen, blocked_index = _select_homotopy(stations, ego_d, previous_side, params)
    valid_count = len([c for c in chosen if c is not None])
    lower = np.zeros(n_stations)
    upper = np.zeros(n_stations)
    if valid_count == 0:
        # Even the first station is blocked: keep a degenerate corridor at the
        # ego position so the controller receives a consistent stop request.
        lower[:] = ego_d
        upper[:] = ego_d
        v_max = np.zeros(n_stations)
        d_ref = np.full(n_stations, ego_d)
        return CorridorResult(
            s=s, lower=lower, upper=upper, d_ref=d_ref, v_max=v_max,
            side=SIDE_CENTER, feasible=False, blocked_at_s=0.0,
            min_free_width=0.0, obstacle_count=len(influencing),
            ego_inside=False,
            kappa=np.zeros(n_stations) if kappa is None else np.asarray(kappa, float),
        )

    for i in range(n_stations):
        iv = chosen[i] if i < valid_count else chosen[valid_count - 1]
        lower[i] = iv.lower
        upper[i] = iv.upper
    ego_inside = bool(lower[0] - 1e-6 <= ego_d <= upper[0] + 1e-6)

    # Funnel smoothing limited by the lateral slope the car can achieve.
    step = params.max_lateral_slope * params.ds
    for i in range(n_stations - 2, -1, -1):
        upper[i] = min(upper[i], upper[i + 1] + step)
        lower[i] = max(lower[i], lower[i + 1] - step)
    for i in range(1, n_stations):
        upper[i] = min(upper[i], upper[i - 1] + step)
        lower[i] = max(lower[i], lower[i - 1] - step)
    # Never squeeze the first station tighter than the raw choice around ego.
    crossed = lower > upper
    if np.any(crossed):
        mid = 0.5 * (lower[crossed] + upper[crossed])
        lower[crossed] = mid
        upper[crossed] = mid
    width = upper - lower

    # Reference: minimal deviation from the lane centre, blended with the
    # corridor centre, then smoothed and clamped back inside.
    delta = np.minimum(params.ref_margin, 0.5 * width)
    minimal = np.clip(0.0, lower + delta, upper - delta)
    center = 0.5 * (lower + upper)
    d_ref = (1.0 - params.ref_center_blend) * minimal + params.ref_center_blend * center
    d_ref = _moving_average(d_ref, params.ref_smooth_window)
    d_ref = np.clip(d_ref, lower + delta, upper - delta)

    # Speed limit from width, curvature, blocked point and braking feasibility.
    width_ratio = (width - params.min_corridor_width) / max(
        params.nominal_width - params.min_corridor_width, 1e-6
    )
    v_max = v_cruise * np.clip(width_ratio, params.v_min_ratio, 1.0)
    if kappa is not None:
        kappa = np.asarray(kappa, dtype=float)
        if kappa.shape == s.shape:
            v_curve = np.sqrt(params.a_lat_max / np.maximum(np.abs(kappa), 1e-3))
            v_max = np.minimum(v_max, v_curve)
    blocked_at_s = -1.0
    if blocked_index >= 0:
        blocked_at_s = float(s[blocked_index])
        stop_s = blocked_at_s - params.stop_margin
        dist = np.maximum(stop_s - s, 0.0)
        v_stop = np.sqrt(2.0 * params.a_brake * dist)
        v_max = np.minimum(v_max, v_stop)
        v_max[s >= stop_s] = 0.0
    for i in range(n_stations - 2, -1, -1):
        v_max[i] = min(
            v_max[i], math.sqrt(v_max[i + 1] ** 2 + 2.0 * params.a_brake * params.ds)
        )
    min_free_width = float(width[:valid_count].min()) if valid_count else 0.0
    side = SIDE_CENTER
    for iv in chosen[:valid_count]:
        if iv.side != SIDE_CENTER:
            side = iv.side
            break
    feasible = blocked_index < 0
    return CorridorResult(
        s=s,
        lower=lower,
        upper=upper,
        d_ref=d_ref,
        v_max=v_max,
        side=side,
        feasible=feasible,
        blocked_at_s=blocked_at_s,
        min_free_width=min_free_width,
        obstacle_count=len(influencing),
        ego_inside=ego_inside,
        kappa=np.zeros(n_stations) if kappa is None else np.asarray(kappa, float),
    )


def obstacle_box_to_frenet(obstacle_id, s_vals, d_vals, vs=0.0, vd=0.0,
                           is_dynamic=False, confidence=1.0):
    s_vals = np.asarray(s_vals, dtype=float)
    d_vals = np.asarray(d_vals, dtype=float)
    return FrenetObstacle(
        obstacle_id=int(obstacle_id),
        s_min=float(s_vals.min()),
        s_max=float(s_vals.max()),
        d_min=float(d_vals.min()),
        d_max=float(d_vals.max()),
        vs=float(vs),
        vd=float(vd),
        is_dynamic=bool(is_dynamic),
        confidence=float(confidence),
    )

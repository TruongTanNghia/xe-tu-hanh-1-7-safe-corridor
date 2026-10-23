"""Block A4: constant-velocity Kalman tracking with Hungarian association.

State per track: [x, y, vx, vy] in the tracking frame (odom preferred).
Lifecycle: tentative -> confirmed after `confirm_hits` hits, deleted after
`max_misses` consecutive misses. Static/dynamic flag uses speed hysteresis.
"""

from dataclasses import dataclass, field
import math

import numpy as np

try:  # scipy is an exec_depend; keep a greedy fallback for minimal images.
    from scipy.optimize import linear_sum_assignment
except Exception:  # pragma: no cover - exercised only without scipy
    linear_sum_assignment = None


@dataclass
class TrackerParams:
    process_noise_accel: float = 1.5      # m/s^2, CV model acceleration noise
    measurement_noise_m: float = 0.03     # m, cluster center noise
    gate_mahalanobis_sq: float = 9.21     # chi2(2 dof, 99 %)
    gate_distance_m: float = 0.5          # hard gate on euclidean distance
    confirm_hits: int = 3
    max_misses: int = 5
    max_coast_s: float = 0.8
    dynamic_speed_on: float = 0.15        # m/s, become dynamic above
    dynamic_speed_off: float = 0.08       # m/s, become static below
    dynamic_hold_frames: int = 3
    initial_velocity_std: float = 0.3


@dataclass
class Track:
    track_id: int
    x: np.ndarray
    P: np.ndarray
    hits: int = 1
    misses: int = 0
    age: int = 1
    confirmed: bool = False
    is_dynamic: bool = False
    dynamic_votes: int = 0
    time_since_update: float = 0.0
    shape: object = None
    shape_history: list = field(default_factory=list)

    @property
    def position(self):
        return self.x[:2].copy()

    @property
    def velocity(self):
        return self.x[2:].copy()

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))

    @property
    def confidence(self):
        return float(min(1.0, self.hits / 6.0)) * (0.5 if self.misses else 1.0)


class MultiObjectTracker:
    def __init__(self, params: TrackerParams = None):
        self.params = params or TrackerParams()
        self.tracks = []
        self._next_id = 1
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        r = self.params.measurement_noise_m**2
        self.R = np.diag([r, r])

    # ------------------------------------------------------------------ model
    def _predict_track(self, track: Track, dt: float):
        dt = max(float(dt), 1e-3)
        F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        q = self.params.process_noise_accel**2
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        Q = q * np.array(
            [
                [dt4 / 4, 0, dt3 / 2, 0],
                [0, dt4 / 4, 0, dt3 / 2],
                [dt3 / 2, 0, dt2, 0],
                [0, dt3 / 2, 0, dt2],
            ]
        )
        track.x = F @ track.x
        track.P = F @ track.P @ F.T + Q
        track.time_since_update += dt
        track.age += 1

    def _update_track(self, track: Track, measurement, shape):
        z = np.asarray(measurement, dtype=float)
        S = self.H @ track.P @ self.H.T + self.R
        K = track.P @ self.H.T @ np.linalg.inv(S)
        innovation = z - self.H @ track.x
        track.x = track.x + K @ innovation
        track.P = (np.eye(4) - K @ self.H) @ track.P
        track.hits += 1
        track.misses = 0
        track.time_since_update = 0.0
        track.shape = shape
        track.shape_history.append(
            (float(shape.length), float(shape.width), float(shape.radius))
        )
        if len(track.shape_history) > 10:
            track.shape_history.pop(0)
        if track.hits >= self.params.confirm_hits:
            track.confirmed = True
        self._update_dynamic_flag(track)

    def _update_dynamic_flag(self, track: Track):
        speed = track.speed
        if track.is_dynamic:
            if speed < self.params.dynamic_speed_off:
                track.dynamic_votes -= 1
            else:
                track.dynamic_votes = self.params.dynamic_hold_frames
            if track.dynamic_votes <= 0:
                track.is_dynamic = False
                track.dynamic_votes = 0
        else:
            if speed > self.params.dynamic_speed_on and track.confirmed:
                track.dynamic_votes += 1
            else:
                track.dynamic_votes = 0
            if track.dynamic_votes >= self.params.dynamic_hold_frames:
                track.is_dynamic = True

    def _new_track(self, measurement, shape):
        v_std = self.params.initial_velocity_std
        p_std = self.params.measurement_noise_m * 3.0
        track = Track(
            track_id=self._next_id,
            x=np.array([measurement[0], measurement[1], 0.0, 0.0], dtype=float),
            P=np.diag([p_std**2, p_std**2, v_std**2, v_std**2]),
            shape=shape,
        )
        track.shape_history.append(
            (float(shape.length), float(shape.width), float(shape.radius))
        )
        self._next_id += 1
        self.tracks.append(track)
        return track

    # ------------------------------------------------------------- association
    def _assign(self, detections):
        n_tracks = len(self.tracks)
        n_dets = len(detections)
        if n_tracks == 0 or n_dets == 0:
            return [], list(range(n_tracks)), list(range(n_dets))
        cost = np.full((n_tracks, n_dets), 1e6)
        for ti, track in enumerate(self.tracks):
            S = self.H @ track.P @ self.H.T + self.R
            S_inv = np.linalg.inv(S)
            predicted = self.H @ track.x
            for di, det in enumerate(detections):
                diff = np.asarray(det.center, dtype=float) - predicted
                if float(np.linalg.norm(diff)) > self.params.gate_distance_m:
                    continue
                m2 = float(diff @ S_inv @ diff)
                if m2 <= self.params.gate_mahalanobis_sq:
                    cost[ti, di] = m2
        if linear_sum_assignment is not None:
            rows, cols = linear_sum_assignment(cost)
            pairs = list(zip(rows.tolist(), cols.tolist()))
        else:  # greedy fallback
            pairs = []
            used_t, used_d = set(), set()
            for ti, di in sorted(
                np.ndindex(cost.shape), key=lambda idx: cost[idx]
            ):
                if ti in used_t or di in used_d or cost[ti, di] >= 1e6:
                    continue
                used_t.add(ti)
                used_d.add(di)
                pairs.append((ti, di))
        matches = [(ti, di) for ti, di in pairs if cost[ti, di] < 1e6]
        matched_t = {ti for ti, _ in matches}
        matched_d = {di for _, di in matches}
        unmatched_t = [i for i in range(n_tracks) if i not in matched_t]
        unmatched_d = [i for i in range(n_dets) if i not in matched_d]
        return matches, unmatched_t, unmatched_d

    # -------------------------------------------------------------------- step
    def update(self, detections, dt):
        """detections: list of ClusterShape with .center in the tracking frame."""
        for track in self.tracks:
            self._predict_track(track, dt)
        matches, unmatched_t, unmatched_d = self._assign(detections)
        for ti, di in matches:
            self._update_track(
                self.tracks[ti], detections[di].center, detections[di]
            )
        for ti in unmatched_t:
            self.tracks[ti].misses += 1
        for di in unmatched_d:
            self._new_track(detections[di].center, detections[di])
        self.tracks = [
            t
            for t in self.tracks
            if t.misses <= self.params.max_misses
            and t.time_since_update <= self.params.max_coast_s
        ]
        return self.confirmed_tracks()

    def confirmed_tracks(self):
        return [t for t in self.tracks if t.confirmed]

    def reset(self):
        self.tracks = []

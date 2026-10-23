"""Block E: behaviour state machine with hysteresis.

States: LANE_FOLLOW, AVOID, FOLLOW, STOP, ESTOP.
Every transition respects a minimum dwell time and "clear" timers so the
vehicle does not oscillate between behaviours.
"""

from dataclasses import dataclass
import math


LANE_FOLLOW = 0
AVOID = 1
FOLLOW = 2
STOP = 3
ESTOP = 4

STATE_NAMES = {
    LANE_FOLLOW: "LANE_FOLLOW",
    AVOID: "AVOID",
    FOLLOW: "FOLLOW",
    STOP: "STOP",
    ESTOP: "ESTOP",
}


@dataclass
class FsmParams:
    v_cruise: float = 0.6
    min_dwell_s: float = 0.5
    clear_time_s: float = 1.0
    stop_clear_time_s: float = 1.5
    lane_lost_timeout_s: float = 0.5
    follow_time_headway_s: float = 1.0
    follow_min_gap_m: float = 0.5
    follow_gain: float = 0.8
    follow_max_speed_ratio: float = 1.0
    blocked_stop_distance_m: float = 2.0   # blocked_at_s below this forces STOP
    lead_lateral_tolerance_m: float = 0.25
    lead_speed_min_mps: float = 0.10


@dataclass
class FsmInputs:
    now: float
    lane_valid: bool
    corridor_feasible: bool
    blocked_at_s: float          # <0 if not blocked
    obstacle_influences: bool    # at least one obstacle cuts the lane corridor
    lead_gap_m: float = -1.0     # >=0 when a same-direction dynamic lead exists
    lead_speed_mps: float = 0.0
    estop: bool = False
    corridor_side: int = 0


@dataclass
class FsmOutput:
    state: int
    state_name: str
    reason: str
    v_ref: float
    time_in_state_s: float
    avoid_side: int
    follow_lead: bool


class BehaviorFSM:
    def __init__(self, params: FsmParams = None):
        self.params = params or FsmParams()
        self.state = STOP
        self.state_since = None
        self.reason = "startup"
        self.avoid_side = 0
        self._clear_since = None
        self._lane_lost_since = None
        self._obstacle_seen_at = None

    # ------------------------------------------------------------ utilities
    def _transition(self, new_state, now, reason):
        if new_state != self.state:
            self.state = new_state
            self.state_since = now
            self._clear_since = None
        self.reason = reason

    def _dwell_elapsed(self, now):
        if self.state_since is None:
            return True
        return (now - self.state_since) >= self.params.min_dwell_s

    def _clear_elapsed(self, now, duration):
        if self._clear_since is None:
            self._clear_since = now
            return False
        return (now - self._clear_since) >= duration

    def _acc_speed(self, gap, lead_speed):
        p = self.params
        gap_des = p.follow_min_gap_m + p.follow_time_headway_s * max(lead_speed, 0.0)
        v = lead_speed + p.follow_gain * (gap - gap_des)
        return float(max(0.0, min(p.v_cruise * p.follow_max_speed_ratio, v)))

    # ------------------------------------------------------------------ step
    def step(self, inp: FsmInputs) -> FsmOutput:
        p = self.params
        now = float(inp.now)
        if self.state_since is None:
            self.state_since = now

        if inp.lane_valid:
            self._lane_lost_since = None
        elif self._lane_lost_since is None:
            self._lane_lost_since = now
        lane_lost = (
            self._lane_lost_since is not None
            and (now - self._lane_lost_since) >= p.lane_lost_timeout_s
        )
        has_lead = inp.lead_gap_m >= 0.0 and inp.lead_speed_mps >= p.lead_speed_min_mps
        blocked = (not inp.corridor_feasible) or (
            0.0 <= inp.blocked_at_s <= p.blocked_stop_distance_m
        )
        if inp.obstacle_influences:
            self._obstacle_seen_at = now

        # --- highest priority: emergency stop -----------------------------
        if inp.estop:
            self._transition(ESTOP, now, "estop_latched")
        elif self.state == ESTOP:
            if self._dwell_elapsed(now):
                self._transition(STOP, now, "estop_cleared")
        # --- lane lost -> STOP ---------------------------------------------
        elif lane_lost or not inp.lane_valid and self.state == STOP:
            self._transition(STOP, now, "lane_lost")
        elif self.state == STOP:
            resume = inp.lane_valid and not blocked
            if resume and self._dwell_elapsed(now) and self._clear_elapsed(
                now, p.stop_clear_time_s
            ):
                if has_lead:
                    self._transition(FOLLOW, now, "lead_ahead")
                elif inp.obstacle_influences:
                    self._transition(AVOID, now, "corridor_available")
                else:
                    self._transition(LANE_FOLLOW, now, "path_clear")
            elif not resume:
                self._clear_since = None
                self.reason = "blocked" if blocked else "lane_invalid"
        elif blocked:
            if self._dwell_elapsed(now) or self.state == LANE_FOLLOW:
                self._transition(STOP, now, "corridor_blocked")
        elif self.state == LANE_FOLLOW:
            if has_lead and not inp.obstacle_influences:
                self._transition(FOLLOW, now, "lead_ahead")
            elif inp.obstacle_influences:
                self._transition(AVOID, now, "obstacle_in_lane")
            else:
                self.reason = "clear"
        elif self.state == AVOID:
            if inp.corridor_side != 0:
                self.avoid_side = inp.corridor_side
            if inp.obstacle_influences:
                self._clear_since = None
                self.reason = "avoiding"
            elif self._dwell_elapsed(now) and self._clear_elapsed(
                now, p.clear_time_s
            ):
                self.avoid_side = 0
                if has_lead:
                    self._transition(FOLLOW, now, "lead_ahead")
                else:
                    self._transition(LANE_FOLLOW, now, "obstacle_passed")
        elif self.state == FOLLOW:
            if inp.obstacle_influences and not has_lead:
                self._transition(AVOID, now, "static_obstacle")
            elif has_lead:
                self._clear_since = None
                self.reason = "following"
            elif self._dwell_elapsed(now) and self._clear_elapsed(
                now, p.clear_time_s
            ):
                self._transition(LANE_FOLLOW, now, "lead_gone")

        # --- outputs ---------------------------------------------------------
        if self.state in (STOP, ESTOP):
            v_ref = 0.0
        elif self.state == FOLLOW and has_lead:
            v_ref = self._acc_speed(inp.lead_gap_m, inp.lead_speed_mps)
        else:
            v_ref = p.v_cruise
        return FsmOutput(
            state=self.state,
            state_name=STATE_NAMES[self.state],
            reason=self.reason,
            v_ref=float(v_ref),
            time_in_state_s=float(now - self.state_since),
            avoid_side=self.avoid_side if self.state == AVOID else 0,
            follow_lead=self.state == FOLLOW and has_lead,
        )


def find_lead_obstacle(frenet_obstacles, lane_half_width, lateral_tolerance=0.25,
                       min_speed=0.10):
    """Pick the nearest same-direction dynamic obstacle inside the lane.

    Returns (obstacle, gap_m) or (None, -1.0). gap is measured from the ego
    reference point to the rear face of the obstacle.
    """
    best = None
    best_gap = -1.0
    for obs in frenet_obstacles:
        if not obs.is_dynamic or obs.s_min <= 0.0:
            continue
        center_d = 0.5 * (obs.d_min + obs.d_max)
        if abs(center_d) > lane_half_width - lateral_tolerance:
            continue
        if obs.vs < min_speed:
            continue
        if math.hypot(obs.vs, obs.vd) > 0.0 and abs(obs.vd) > 0.7 * abs(obs.vs):
            continue  # crossing, not a lead
        if best is None or obs.s_min < best.s_min:
            best = obs
            best_gap = obs.s_min
    return best, float(best_gap)

from collections import deque
import math
import statistics


class VelocitySampleError(ValueError):
    """Raised when a velocity sample is unsafe to publish."""


class PlanarPoseSpeedEstimator:
    """Estimate signed planar speed from RF2O pose increments.

    RF2O's published ``linear.x`` only uses the x component of its scan-match
    increment.  That under-reports speed when the estimated odometry trajectory
    is aligned with another odom axis.  The vehicle is forward-only at runtime,
    but retaining the sign of RF2O's raw x velocity keeps push/pull validation
    useful.
    """

    def __init__(
        self,
        *,
        reset_gap_s: float = 0.5,
        direction_epsilon_mps: float = 0.0001,
    ):
        if reset_gap_s <= 0.0:
            raise ValueError("reset_gap_s must be positive")
        if direction_epsilon_mps < 0.0:
            raise ValueError("direction_epsilon_mps must be non-negative")
        self.reset_gap_s = float(reset_gap_s)
        self.direction_epsilon_mps = float(direction_epsilon_mps)
        self.previous = None
        self.last_direction = 1.0

    def reset(self):
        self.previous = None
        self.last_direction = 1.0

    def update(
        self,
        *,
        x_m: float,
        y_m: float,
        raw_vx_mps: float,
        stamp_s: float,
    ) -> float:
        values = tuple(float(value) for value in (
            x_m,
            y_m,
            raw_vx_mps,
            stamp_s,
        ))
        if not all(math.isfinite(value) for value in values):
            raise VelocitySampleError("non-finite pose, velocity, or timestamp")
        x_m, y_m, raw_vx_mps, stamp_s = values

        previous = self.previous
        self.previous = (stamp_s, x_m, y_m)
        if previous is None:
            return raw_vx_mps

        delta_s = stamp_s - previous[0]
        if delta_s <= 0.0:
            raise VelocitySampleError("timestamp is not increasing")
        if delta_s > self.reset_gap_s:
            return raw_vx_mps

        if abs(raw_vx_mps) > self.direction_epsilon_mps:
            self.last_direction = math.copysign(1.0, raw_vx_mps)
        distance_m = math.hypot(x_m - previous[1], y_m - previous[2])
        return self.last_direction * distance_m / delta_s


class RobustVelocityFilter:
    def __init__(
        self,
        *,
        median_window: int = 3,
        ema_alpha: float = 0.45,
        zero_threshold_mps: float = 0.02,
        max_abs_speed_mps: float = 1.5,
        max_acceleration_mps2: float = 3.0,
        reset_gap_s: float = 0.5,
        variance_window: int = 30,
        variance_floor: float = 0.0009,
    ):
        if median_window < 1 or median_window % 2 == 0:
            raise ValueError("median_window must be a positive odd integer")
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        if zero_threshold_mps < 0.0:
            raise ValueError("zero_threshold_mps must be non-negative")
        if max_abs_speed_mps <= 0.0 or max_acceleration_mps2 <= 0.0:
            raise ValueError("speed and acceleration limits must be positive")
        if reset_gap_s <= 0.0:
            raise ValueError("reset_gap_s must be positive")
        if variance_window < 2 or variance_floor <= 0.0:
            raise ValueError("variance settings are invalid")

        self.median_window = median_window
        self.ema_alpha = ema_alpha
        self.zero_threshold_mps = zero_threshold_mps
        self.max_abs_speed_mps = max_abs_speed_mps
        self.max_acceleration_mps2 = max_acceleration_mps2
        self.reset_gap_s = reset_gap_s
        self.variance_floor = variance_floor

        self.raw_window = deque(maxlen=median_window)
        self.residual_window = deque(maxlen=variance_window)
        self.last_stamp_s = None
        self.last_raw = None
        self.filtered = None

    def reset(self):
        self.raw_window.clear()
        self.residual_window.clear()
        self.last_stamp_s = None
        self.last_raw = None
        self.filtered = None

    def update(self, value_mps: float, stamp_s: float):
        value_mps = float(value_mps)
        stamp_s = float(stamp_s)
        if not math.isfinite(value_mps) or not math.isfinite(stamp_s):
            raise VelocitySampleError("non-finite value or timestamp")
        if abs(value_mps) > self.max_abs_speed_mps:
            raise VelocitySampleError("absolute speed exceeds configured limit")

        if self.last_stamp_s is not None:
            delta_s = stamp_s - self.last_stamp_s
            if delta_s <= 0.0:
                raise VelocitySampleError("timestamp is not increasing")
            if delta_s > self.reset_gap_s:
                self.raw_window.clear()
                self.residual_window.clear()
                self.last_raw = None
                self.filtered = None
            elif self.last_raw is not None:
                acceleration = abs(value_mps - self.last_raw) / delta_s
                if acceleration > self.max_acceleration_mps2:
                    raise VelocitySampleError(
                        "speed step exceeds acceleration limit"
                    )

        self.raw_window.append(value_mps)
        median = float(statistics.median(self.raw_window))
        target = 0.0 if abs(median) <= self.zero_threshold_mps else median

        if self.filtered is None:
            filtered = target
        else:
            filtered = (
                self.ema_alpha * target
                + (1.0 - self.ema_alpha) * self.filtered
            )
        if abs(filtered) <= self.zero_threshold_mps:
            filtered = 0.0

        self.residual_window.append(value_mps - median)
        if len(self.residual_window) >= 2:
            residual_variance = statistics.variance(self.residual_window)
        else:
            residual_variance = 0.0

        self.last_stamp_s = stamp_s
        self.last_raw = value_mps
        self.filtered = filtered
        return filtered, max(self.variance_floor, residual_variance)

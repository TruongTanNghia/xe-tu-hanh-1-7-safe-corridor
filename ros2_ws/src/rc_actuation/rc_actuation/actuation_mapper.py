import math


class ActuationMapper:
    def __init__(
        self,
        *,
        servo_center_us=1500,
        servo_min_us=1150,
        servo_max_us=1850,
        servo_us_per_rad=245.9,
        esc_stop_us=1550,
        esc_forward_start_us=1580,
        esc_max_us=1600,
        esc_rise_rate_us_per_s=50.0,
    ):
        self.servo_center_us = int(servo_center_us)
        self.servo_min_us = int(servo_min_us)
        self.servo_max_us = int(servo_max_us)
        self.servo_us_per_rad = float(servo_us_per_rad)
        self.esc_stop_us = int(esc_stop_us)
        self.esc_forward_start_us = int(esc_forward_start_us)
        self.esc_max_us = int(esc_max_us)
        self.esc_rise_rate_us_per_s = float(esc_rise_rate_us_per_s)

        if not self.servo_min_us < self.servo_center_us < self.servo_max_us:
            raise ValueError("servo center must be inside servo limits")
        if self.servo_us_per_rad <= 0.0:
            raise ValueError("servo_us_per_rad must be positive")
        if not self.esc_stop_us < self.esc_forward_start_us <= self.esc_max_us:
            raise ValueError("invalid ESC stop/start/max values")
        if self.esc_rise_rate_us_per_s <= 0.0:
            raise ValueError("ESC rise rate must be positive")

    def steering_to_pwm(self, steering_angle_rad):
        steering_angle_rad = float(steering_angle_rad)
        if not math.isfinite(steering_angle_rad):
            raise ValueError("steering angle is non-finite")
        pwm = round(
            self.servo_center_us
            + self.servo_us_per_rad * steering_angle_rad
        )
        return max(self.servo_min_us, min(self.servo_max_us, pwm))

    def effort_to_target_pwm(self, drive_effort):
        drive_effort = float(drive_effort)
        if not math.isfinite(drive_effort):
            raise ValueError("drive effort is non-finite")
        if drive_effort <= 0.0:
            return self.esc_stop_us
        normalized = min(1.0, drive_effort)
        pwm = round(
            self.esc_forward_start_us
            + normalized * (self.esc_max_us - self.esc_forward_start_us)
        )
        return max(self.esc_forward_start_us, min(self.esc_max_us, pwm))

    def apply_esc_slew(self, current_us, target_us, delta_s):
        current_us = int(current_us)
        target_us = int(target_us)
        delta_s = max(0.0, float(delta_s))
        if target_us <= self.esc_stop_us:
            return self.esc_stop_us
        if target_us <= current_us:
            return target_us
        maximum_step = max(1, round(self.esc_rise_rate_us_per_s * delta_s))
        return min(target_us, current_us + maximum_step)

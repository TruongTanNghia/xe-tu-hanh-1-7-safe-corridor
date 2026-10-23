#!/usr/bin/env python3

import time

from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rc_interfaces.msg import BehaviorState, ControlCommand, Corridor, LaneReference
from std_msgs.msg import Float64, Float64MultiArray, String

from race_cars.acados_settings import acados_settings
from race_cars.bicycle_model import DEFAULT_MODEL_PARAMETERS
from race_cars.corridor_lookup import (
    clamp_bounds,
    predicted_stage_s,
    sample_corridor,
    validate_corridor_arrays,
)
from race_cars.input_validation import finite_state, validate_lane_arrays


CORRIDOR_MODES = ("off", "optional", "required")


class RacecarNmpcNode(Node):
    def __init__(self):
        super().__init__("racecar_nmpc_node")
        self.declare_parameter("target_speed_mps", 0.6)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("prediction_horizon_s", 1.3)
        self.declare_parameter("horizon_steps", 10)
        self.declare_parameter("minimum_line_confidence", 0.5)
        self.declare_parameter("line_timeout_s", 0.5)
        self.declare_parameter("odom_timeout_s", 0.5)
        self.declare_parameter("shadow_mode", True)
        self.declare_parameter("model_calibrated", False)
        # Safe corridor (block D)
        self.declare_parameter("corridor_mode", "optional")
        self.declare_parameter("corridor_topic", "/planning/corridor")
        self.declare_parameter("corridor_timeout_s", 0.3)
        self.declare_parameter("use_corridor_kappa", True)
        self.declare_parameter("use_corridor_speed", True)
        self.declare_parameter("slack_warn_m", 0.05)
        self.declare_parameter("slack_stop_m", 0.15)
        self.declare_parameter("slack_stop_cycles", 5)
        self.declare_parameter("publish_prediction", True)

        for name, value in DEFAULT_MODEL_PARAMETERS.items():
            self.declare_parameter(f"model.{name}", value)

        self.target_speed_mps = float(
            self.get_parameter("target_speed_mps").value
        )
        self.control_rate_hz = float(
            self.get_parameter("control_rate_hz").value
        )
        self.prediction_horizon_s = float(
            self.get_parameter("prediction_horizon_s").value
        )
        self.horizon_steps = int(
            self.get_parameter("horizon_steps").value
        )
        self.minimum_line_confidence = float(
            self.get_parameter("minimum_line_confidence").value
        )
        self.line_timeout_s = float(
            self.get_parameter("line_timeout_s").value
        )
        self.odom_timeout_s = float(
            self.get_parameter("odom_timeout_s").value
        )
        self.shadow_mode = bool(self.get_parameter("shadow_mode").value)
        self.model_calibrated = bool(
            self.get_parameter("model_calibrated").value
        )
        self.corridor_mode = str(self.get_parameter("corridor_mode").value)
        self.corridor_timeout_s = float(
            self.get_parameter("corridor_timeout_s").value
        )
        self.use_corridor_kappa = bool(
            self.get_parameter("use_corridor_kappa").value
        )
        self.use_corridor_speed = bool(
            self.get_parameter("use_corridor_speed").value
        )
        self.slack_warn_m = float(self.get_parameter("slack_warn_m").value)
        self.slack_stop_m = float(self.get_parameter("slack_stop_m").value)
        self.slack_stop_cycles = int(
            self.get_parameter("slack_stop_cycles").value
        )
        self.publish_prediction = bool(
            self.get_parameter("publish_prediction").value
        )
        if self.control_rate_hz <= 0.0 or self.horizon_steps < 2:
            raise ValueError("invalid controller rate or horizon")
        if self.corridor_mode not in CORRIDOR_MODES:
            raise ValueError(
                f"corridor_mode must be one of {CORRIDOR_MODES}"
            )
        if not self.shadow_mode and not self.model_calibrated:
            raise RuntimeError(
                "Active NMPC requires model_calibrated=true; use shadow_mode"
            )

        model_parameters = {
            name: self.get_parameter(f"model.{name}").value
            for name in DEFAULT_MODEL_PARAMETERS
        }
        self.constraint, self.model, self.solver = acados_settings(
            self.prediction_horizon_s,
            self.horizon_steps,
            model_parameters,
        )
        self.stage_dt = self.prediction_horizon_s / self.horizon_steps
        self.lane_only_lh = np.array(
            [
                self.constraint.along_min,
                self.constraint.alat_min,
                self.model.n_min,
                self.model.throttle_min,
                self.model.delta_min,
                self.model.n_min,
                self.model.n_min,
            ]
        )
        self.lane_only_uh = np.array(
            [
                self.constraint.along_max,
                self.constraint.alat_max,
                self.model.n_max,
                self.model.throttle_max,
                self.model.delta_max,
                self.model.n_max,
                self.model.n_max,
            ]
        )

        self.state = np.zeros(6)
        self.latest_lane = None
        self.last_lane_time = None
        self.last_odom_time = None
        self.latest_corridor = None
        self.last_corridor_time = None
        self.previous_states = None
        self.slack_violation_cycles = 0
        self.corridor_status_text = "corridor_off"

        self.create_subscription(
            LaneReference,
            "/lane/reference",
            self._lane_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self._odom_callback,
            10,
        )
        if self.corridor_mode != "off":
            self.create_subscription(
                Corridor,
                str(self.get_parameter("corridor_topic").value),
                self._corridor_callback,
                10,
            )

        self.command_publisher = self.create_publisher(
            ControlCommand,
            "/control/nmpc",
            10,
        )
        self.legacy_publisher = self.create_publisher(
            Float64MultiArray,
            "/nmpc_cmd",
            10,
        )
        self.shadow_publisher = self.create_publisher(
            Float64MultiArray,
            "/nmpc_cmd_shadow",
            10,
        )
        self.solve_time_publisher = self.create_publisher(
            Float64,
            "/nmpc/solve_time_ms",
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/nmpc/status",
            10,
        )
        self.corridor_status_publisher = self.create_publisher(
            String,
            "/nmpc/corridor_status",
            10,
        )
        self.slack_publisher = self.create_publisher(
            Float64,
            "/nmpc/max_slack_m",
            10,
        )
        self.prediction_publisher = self.create_publisher(
            Float64MultiArray,
            "/nmpc/prediction_sd",
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._control_loop,
        )
        self._publish_stop(ControlCommand.REASON_STARTUP, "startup")
        self.get_logger().info(
            f"NMPC ready: target={self.target_speed_mps:.2f} m/s, "
            f"rate={self.control_rate_hz:.1f} Hz, "
            f"shadow_mode={self.shadow_mode}, "
            f"model_calibrated={self.model_calibrated}, "
            f"corridor_mode={self.corridor_mode}"
        )

    def _lane_callback(self, message):
        self.latest_lane = message
        self.last_lane_time = time.monotonic()

    def _odom_callback(self, message):
        velocity = float(message.twist.twist.linear.x)
        if np.isfinite(velocity):
            self.state[3] = velocity
            self.last_odom_time = time.monotonic()

    def _corridor_callback(self, message):
        self.latest_corridor = message
        self.last_corridor_time = time.monotonic()

    def _publish_stop(self, reason, status_text):
        command = ControlCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.steering_angle_rad = 0.0
        command.drive_effort = 0.0
        command.valid = False
        command.reason = reason
        self.command_publisher.publish(command)
        self.legacy_publisher.publish(
            Float64MultiArray(data=[0.0, 0.0])
        )
        self.shadow_publisher.publish(
            Float64MultiArray(data=[0.0, 0.0])
        )
        self.status_publisher.publish(String(data=status_text))
        self.corridor_status_publisher.publish(
            String(data=self.corridor_status_text)
        )
        self.previous_states = None

    def _input_failure(self):
        now = time.monotonic()
        if self.latest_lane is None:
            return ControlCommand.REASON_LINE_STALE, "no_lane_reference"
        if (
            self.last_lane_time is None
            or now - self.last_lane_time > self.line_timeout_s
        ):
            return ControlCommand.REASON_LINE_STALE, "lane_stale"
        if self.latest_lane.status != LaneReference.STATUS_DETECTED:
            return ControlCommand.REASON_LINE_INVALID, "lane_invalid"
        if self.latest_lane.confidence < self.minimum_line_confidence:
            return ControlCommand.REASON_LINE_INVALID, "line_low_confidence"
        if (
            self.last_odom_time is None
            or now - self.last_odom_time > self.odom_timeout_s
        ):
            return ControlCommand.REASON_ODOM_STALE, "odom_stale"

        valid_arrays, array_error = validate_lane_arrays(
            self.latest_lane.s_m,
            self.latest_lane.curvature_inv_m,
        )
        if not valid_arrays:
            return (
                ControlCommand.REASON_INPUT_INVALID,
                f"lane_array_invalid:{array_error}",
            )
        if not finite_state(
            self.latest_lane.cte_m,
            self.latest_lane.heading_error_rad,
            self.state[3],
        ):
            return ControlCommand.REASON_INPUT_INVALID, "non_finite_state"
        if not self.model.n_min <= self.latest_lane.cte_m <= self.model.n_max:
            return ControlCommand.REASON_LINE_INVALID, "cte_outside_safe_bound"
        return None

    def _active_corridor(self):
        """Return (corridor_msg or None, failure or None, status_text)."""
        if self.corridor_mode == "off":
            return None, None, "corridor_off"
        now = time.monotonic()
        fresh = (
            self.latest_corridor is not None
            and self.last_corridor_time is not None
            and now - self.last_corridor_time <= self.corridor_timeout_s
        )
        if not fresh:
            if self.corridor_mode == "required":
                return None, (
                    ControlCommand.REASON_CORRIDOR_STALE,
                    "corridor_stale",
                ), "corridor_stale"
            return None, None, "corridor_missing_lane_only"
        corridor = self.latest_corridor
        valid, error = validate_corridor_arrays(
            corridor.s, corridor.lower, corridor.upper, corridor.d_ref,
            corridor.v_max, corridor.kappa_ref,
        )
        if not valid:
            if self.corridor_mode == "required":
                return None, (
                    ControlCommand.REASON_INPUT_INVALID,
                    f"corridor_invalid:{error}",
                ), f"corridor_invalid:{error}"
            return None, None, f"corridor_invalid_lane_only:{error}"
        if corridor.behavior_state == BehaviorState.STATE_ESTOP:
            return corridor, (
                ControlCommand.REASON_SAFETY_STOP,
                "safety_estop",
            ), "safety_estop"
        if not corridor.lane_valid:
            return corridor, (
                ControlCommand.REASON_LINE_INVALID,
                "corridor_lane_invalid",
            ), "corridor_lane_invalid"
        return corridor, None, "corridor_active"

    def _publish_solution(self, steering, drive_effort, solve_time_ms):
        command = ControlCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.steering_angle_rad = steering
        command.drive_effort = drive_effort
        command.valid = not self.shadow_mode
        command.reason = (
            ControlCommand.REASON_SHADOW_MODE
            if self.shadow_mode
            else ControlCommand.REASON_OK
        )
        self.command_publisher.publish(command)
        self.shadow_publisher.publish(
            Float64MultiArray(data=[steering, drive_effort])
        )
        if self.shadow_mode:
            self.legacy_publisher.publish(
                Float64MultiArray(data=[0.0, 0.0])
            )
            status = "SHADOW_VALID"
        else:
            self.legacy_publisher.publish(
                Float64MultiArray(data=[steering, drive_effort])
            )
            status = "ACTIVE_VALID"
        self.solve_time_publisher.publish(Float64(data=solve_time_ms))
        self.status_publisher.publish(String(data=status))
        self.corridor_status_publisher.publish(
            String(data=self.corridor_status_text)
        )

    def _max_slack(self):
        """Largest soft-constraint violation (m) over the horizon."""
        worst = 0.0
        for stage in range(1, self.horizon_steps):
            try:
                sl = np.asarray(self.solver.get(stage, "sl"), dtype=float)
                su = np.asarray(self.solver.get(stage, "su"), dtype=float)
            except Exception:
                return worst
            if sl.size >= 2:
                worst = max(worst, float(np.max(sl[1:])), float(np.max(su[1:])))
        return worst

    def _control_loop(self):
        failure = self._input_failure()
        if failure is not None:
            reason, status = failure
            self._publish_stop(reason, status)
            return
        corridor, corridor_failure, corridor_status = self._active_corridor()
        self.corridor_status_text = corridor_status
        if corridor_failure is not None:
            reason, status = corridor_failure
            self._publish_stop(reason, status)
            return

        lane = self.latest_lane
        initial_state = self.state.copy()
        initial_state[0] = 0.0
        initial_state[1] = lane.cte_m
        initial_state[2] = lane.heading_error_rad

        try:
            self.solver.set(0, "lbx", initial_state)
            self.solver.set(0, "ubx", initial_state)
            curvature = np.asarray(lane.curvature_inv_m, dtype=float)
            s_values = np.asarray(lane.s_m, dtype=float)
            interpolation_axis = np.linspace(
                0.0,
                1.0,
                self.horizon_steps + 1,
            )
            curvature_horizon = np.interp(
                interpolation_axis,
                np.linspace(0.0, 1.0, len(curvature)),
                curvature,
            )
            s_horizon = np.linspace(
                0.0,
                float(s_values[-1]),
                self.horizon_steps + 1,
            )
            d_ref_horizon = np.zeros(self.horizon_steps + 1)
            v_ref_horizon = np.full(
                self.horizon_steps + 1, self.target_speed_mps
            )
            lh_stages = [self.lane_only_lh] * self.horizon_steps
            uh_stages = [self.lane_only_uh] * self.horizon_steps

            if corridor is not None:
                s_pred = predicted_stage_s(
                    self.previous_states,
                    self.horizon_steps,
                    self.stage_dt,
                    self.state[3],
                    1.0 / self.control_rate_hz,
                )
                sample = sample_corridor(
                    corridor.s, corridor.lower, corridor.upper,
                    corridor.d_ref, corridor.v_max, corridor.kappa_ref,
                    s_pred,
                    front_offset=self.model.footprint_front,
                    rear_offset=self.model.footprint_rear,
                )
                lower_c, upper_c = clamp_bounds(
                    sample["lower"], sample["upper"],
                    self.model.n_min, self.model.n_max,
                )
                lower_f, upper_f = clamp_bounds(
                    sample["lower_front"], sample["upper_front"],
                    self.model.n_min, self.model.n_max,
                )
                lower_r, upper_r = clamp_bounds(
                    sample["lower_rear"], sample["upper_rear"],
                    self.model.n_min, self.model.n_max,
                )
                d_ref_horizon = np.clip(
                    sample["d_ref"], self.model.n_min, self.model.n_max
                )
                if self.use_corridor_speed:
                    v_ref_horizon = np.minimum(
                        v_ref_horizon,
                        np.maximum(sample["v_max"], 0.0),
                    )
                    v_ref_horizon = np.minimum(
                        v_ref_horizon, max(float(corridor.v_ref), 0.0)
                    )
                if self.use_corridor_kappa:
                    curvature_horizon = sample["kappa"]
                    s_horizon = s_pred
                lh_stages = []
                uh_stages = []
                for stage in range(self.horizon_steps):
                    lh = self.lane_only_lh.copy()
                    uh = self.lane_only_uh.copy()
                    lh[2], uh[2] = lower_c[stage], upper_c[stage]
                    lh[5], uh[5] = lower_f[stage], upper_f[stage]
                    lh[6], uh[6] = lower_r[stage], upper_r[stage]
                    lh_stages.append(lh)
                    uh_stages.append(uh)

            for stage in range(self.horizon_steps):
                self.solver.set(
                    stage,
                    "p",
                    np.array([curvature_horizon[stage]]),
                )
                self.solver.constraints_set(stage, "lh", lh_stages[stage])
                self.solver.constraints_set(stage, "uh", uh_stages[stage])
                self.solver.set(
                    stage,
                    "yref",
                    np.array(
                        [
                            s_horizon[stage],
                            d_ref_horizon[stage],
                            0.0,
                            v_ref_horizon[stage],
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        ]
                    ),
                )
            self.solver.set(
                self.horizon_steps,
                "p",
                np.array([curvature_horizon[-1]]),
            )
            self.solver.set(
                self.horizon_steps,
                "yref",
                np.array(
                    [
                        s_horizon[-1],
                        d_ref_horizon[-1],
                        0.0,
                        v_ref_horizon[-1],
                        0.0,
                        0.0,
                    ]
                ),
            )

            started = time.perf_counter()
            solver_status = self.solver.solve()
            solve_time_ms = (time.perf_counter() - started) * 1000.0
            if solver_status != 0:
                self._publish_stop(
                    ControlCommand.REASON_SOLVER_FAILED,
                    f"solver_failed:{solver_status}",
                )
                return

            trajectory = np.array(
                [self.solver.get(k, "x") for k in range(self.horizon_steps + 1)]
            )
            self.previous_states = trajectory
            if self.publish_prediction:
                self.prediction_publisher.publish(
                    Float64MultiArray(
                        data=[float(v) for v in trajectory[:, :2].ravel()]
                    )
                )

            if corridor is not None:
                max_slack = self._max_slack()
                self.slack_publisher.publish(Float64(data=max_slack))
                if max_slack > self.slack_stop_m:
                    self.slack_violation_cycles += 1
                else:
                    self.slack_violation_cycles = 0
                if max_slack > self.slack_warn_m:
                    self.corridor_status_text = (
                        f"corridor_active_slack:{max_slack:.3f}"
                    )
                if self.slack_violation_cycles >= self.slack_stop_cycles:
                    self.corridor_status_text = (
                        f"corridor_violated:{max_slack:.3f}"
                    )
                    self._publish_stop(
                        ControlCommand.REASON_CORRIDOR_BLOCKED,
                        "corridor_violated",
                    )
                    return
            else:
                self.slack_violation_cycles = 0

            predicted_state = trajectory[1]
            drive_effort = float(
                np.clip(
                    predicted_state[4],
                    self.model.throttle_min,
                    self.model.throttle_max,
                )
            )
            steering = float(
                np.clip(
                    predicted_state[5],
                    self.model.delta_min,
                    self.model.delta_max,
                )
            )
            self.state[4] = drive_effort
            self.state[5] = steering
            self._publish_solution(steering, drive_effort, solve_time_ms)
        except Exception as error:
            self.get_logger().error(f"NMPC exception: {error}")
            self._publish_stop(
                ControlCommand.REASON_SOLVER_FAILED,
                "solver_exception",
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RacecarNmpcNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node._publish_stop(
                    ControlCommand.REASON_STARTUP,
                    "shutdown",
                )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

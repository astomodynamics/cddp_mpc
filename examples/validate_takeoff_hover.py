#!/usr/bin/env python3
"""Validate PX4 takeoff/hover/landing behavior for cddp_mpc."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleOdometry,
    VehicleStatus,
    VehicleThrustSetpoint,
    VehicleTorqueSetpoint,
)
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass
class ValidationConfig:
    validation_mode: str = "offboard"
    target_z: float = -3.0
    settle_tolerance: float = 0.3
    hold_duration_sec: float = 0.0
    timeout_sec: float = 90.0
    min_heartbeat_msgs: int = 10
    min_setpoint_msgs: int = 10
    require_hover_done: bool = False
    min_climb_m: float = 0.0
    climb_timeout_sec: float = 25.0
    max_downward_speed_mps: float = 0.0
    max_drop_from_start_m: float = 0.0
    max_xy_error_m: float = 0.0
    max_xy_drift_from_start_m: float = 0.0
    require_landing: bool = False
    landing_tolerance_m: float = 0.3


class HoverMissionValidator(Node):
    """Monitor PX4 and cddp_mpc topics until mission success or timeout."""

    def __init__(self, config: ValidationConfig):
        super().__init__("hover_mission_validator")
        self.config = config
        self.start_time = time.monotonic()
        self.done = False
        self.success = False
        self.failure_reason = ""

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.odom_count = 0
        self.status_count = 0
        self.offboard_mode_count = 0
        self.thrust_setpoint_count = 0
        self.torque_setpoint_count = 0
        self.diag_count = 0

        self.current_z = None
        self.current_x = None
        self.current_y = None
        self.current_vz = None
        self.start_z = None
        self.start_xy = None
        self.peak_climb_m = 0.0
        self.altitude_reached = False
        self.altitude_reached_since: float | None = None
        self.armed_seen = False
        self.currently_armed = False
        self.disarmed_after_arming = False
        self.offboard_seen = False
        self.solver_success_seen = False
        self.modes_seen: set[str] = set()
        self.solver_cmd_thrust = None
        self.published_cmd_thrust = None

        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry_callback, qos_profile
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._status_callback, qos_profile
        )
        self.create_subscription(
            OffboardControlMode, "/fmu/in/offboard_control_mode", self._offboard_mode_callback, qos_profile
        )
        self.create_subscription(
            VehicleThrustSetpoint,
            "/fmu/in/vehicle_thrust_setpoint",
            self._thrust_setpoint_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleTorqueSetpoint,
            "/fmu/in/vehicle_torque_setpoint",
            self._torque_setpoint_callback,
            qos_profile,
        )
        self.create_subscription(
            DiagnosticArray, "/cddp_mpc/status", self._diagnostic_callback, qos_profile
        )

        self.create_timer(1.0, self._tick)
        self.get_logger().info("Hover mission validator started.")

    def _odometry_callback(self, msg: VehicleOdometry) -> None:
        self.odom_count += 1
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        self.current_vz = float(msg.velocity[2]) if len(msg.velocity) > 2 else None
        if self.start_z is None:
            self.start_z = self.current_z
        if self.start_xy is None and self.current_x is not None and self.current_y is not None:
            self.start_xy = (self.current_x, self.current_y)
        if self.start_z is not None:
            self.peak_climb_m = max(self.peak_climb_m, self.start_z - self.current_z)
        if abs(self.current_z - self.config.target_z) <= self.config.settle_tolerance:
            self.altitude_reached = True
            if self.altitude_reached_since is None:
                self.altitude_reached_since = time.monotonic()
        else:
            self.altitude_reached_since = None

    def _status_callback(self, msg: VehicleStatus) -> None:
        self.status_count += 1
        is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.armed_seen = self.armed_seen or is_armed
        if self.armed_seen and not is_armed:
            self.disarmed_after_arming = True
        self.currently_armed = is_armed
        self.offboard_seen = self.offboard_seen or (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _offboard_mode_callback(self, _msg: OffboardControlMode) -> None:
        self.offboard_mode_count += 1

    def _thrust_setpoint_callback(self, _msg: VehicleThrustSetpoint) -> None:
        self.thrust_setpoint_count += 1

    def _torque_setpoint_callback(self, _msg: VehicleTorqueSetpoint) -> None:
        self.torque_setpoint_count += 1

    def _diagnostic_callback(self, msg: DiagnosticArray) -> None:
        self.diag_count += 1
        for status in msg.status:
            if getattr(status, "name", "") != "cddp_mpc":
                continue
            mode = str(getattr(status, "message", "")).strip()
            if mode:
                self.modes_seen.add(mode)
            for kv in getattr(status, "values", []):
                if kv.key == "solve_success" and str(kv.value).lower() == "true":
                    self.solver_success_seen = True
                elif kv.key == "solver_cmd_thrust_n":
                    try:
                        self.solver_cmd_thrust = float(kv.value)
                    except (TypeError, ValueError):
                        pass
                elif kv.key == "published_cmd_thrust_n":
                    try:
                        self.published_cmd_thrust = float(kv.value)
                    except (TypeError, ValueError):
                        pass

    def _criteria(self) -> dict[str, bool]:
        validation_mode = self.config.validation_mode.strip().lower()
        offboard_validation = validation_mode == "offboard"
        hover_mode_ok = (
            "HOVER" in self.modes_seen
            or "LAND" in self.modes_seen
            or "LAND_DONE" in self.modes_seen
        )
        hover_done_ok = "LAND" in self.modes_seen or "LAND_DONE" in self.modes_seen
        diagnostics_ok = True
        if offboard_validation:
            diagnostics_ok = self.diag_count > 0 and self.solver_success_seen
            diagnostics_ok = diagnostics_ok and (
                hover_done_ok if self.config.require_hover_done else hover_mode_ok
            )
        landing_ok = True
        if self.config.require_landing:
            landing_ok = (
                self.start_z is not None
                and self.current_z is not None
                and abs(self.current_z - self.start_z) <= self.config.landing_tolerance_m
                and self.disarmed_after_arming
                and "LAND_DONE" in self.modes_seen
            )
        if self.config.hold_duration_sec > 0.0:
            altitude_hold_ok = (
                self.altitude_reached_since is not None
                and (time.monotonic() - self.altitude_reached_since) >= self.config.hold_duration_sec
            )
        else:
            altitude_hold_ok = self.altitude_reached
        return {
            "odometry_stream": self.odom_count > 0,
            "status_stream": self.status_count > 0,
            "armed_seen": self.armed_seen,
            "offboard_seen": self.offboard_seen if offboard_validation else True,
            "heartbeat_stream": self.offboard_mode_count >= self.config.min_heartbeat_msgs
            if offboard_validation
            else True,
            "thrust_stream": self.thrust_setpoint_count >= self.config.min_setpoint_msgs
            if offboard_validation
            else True,
            "torque_stream": self.torque_setpoint_count >= self.config.min_setpoint_msgs
            if offboard_validation
            else True,
            "altitude_reached": altitude_hold_ok,
            "xy_error_ok": True
            if self.config.max_xy_error_m <= 0.0
            else (
                self.current_x is not None
                and self.current_y is not None
                and (self.current_x * self.current_x + self.current_y * self.current_y)
                <= (self.config.max_xy_error_m * self.config.max_xy_error_m)
            ),
            "diagnostics_ok": diagnostics_ok,
            "landing_ok": landing_ok,
        }

    def _early_failure_guard(self, elapsed: float) -> str | None:
        if (
            self.config.max_downward_speed_mps > 0.0
            and self.armed_seen
            and self.current_vz is not None
            and self.current_vz > self.config.max_downward_speed_mps
        ):
            return (
                "unsafe downward speed detected: "
                f"vz={self.current_vz:.2f} m/s > {self.config.max_downward_speed_mps:.2f} m/s"
            )
        if (
            self.config.max_drop_from_start_m > 0.0
            and self.armed_seen
            and self.start_z is not None
            and self.current_z is not None
            and (self.current_z - self.start_z) > self.config.max_drop_from_start_m
        ):
            return (
                "vehicle dropped below start altitude by "
                f"{self.current_z - self.start_z:.2f} m "
                f"(limit {self.config.max_drop_from_start_m:.2f} m)"
            )
        if (
            self.config.max_xy_drift_from_start_m > 0.0
            and self.start_xy is not None
            and self.current_x is not None
            and self.current_y is not None
        ):
            drift = float(
                ((self.current_x - self.start_xy[0]) ** 2 + (self.current_y - self.start_xy[1]) ** 2)
                ** 0.5
            )
            if drift > self.config.max_xy_drift_from_start_m:
                return (
                    "excessive lateral drift from start: "
                    f"{drift:.2f} m > {self.config.max_xy_drift_from_start_m:.2f} m"
                )
        if (
            self.config.min_climb_m > 0.0
            and elapsed >= self.config.climb_timeout_sec
            and self.peak_climb_m < self.config.min_climb_m
        ):
            return (
                "insufficient climb progress: "
                f"{self.peak_climb_m:.2f} m < {self.config.min_climb_m:.2f} m "
                f"within {self.config.climb_timeout_sec:.1f} s"
            )
        return None

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.start_time
        criteria = self._criteria()
        z_text = "n/a" if self.current_z is None else f"{self.current_z:.2f}"
        vz_text = "n/a" if self.current_vz is None else f"{self.current_vz:.2f}"
        climb_text = f"{self.peak_climb_m:.2f}"
        modes = ",".join(sorted(self.modes_seen)) if self.modes_seen else "none"
        solver_thrust_text = "n/a" if self.solver_cmd_thrust is None else f"{self.solver_cmd_thrust:.2f}"
        published_thrust_text = "n/a" if self.published_cmd_thrust is None else f"{self.published_cmd_thrust:.2f}"
        self.get_logger().info(
            f"elapsed={elapsed:.1f}s z={z_text} vz={vz_text} climb={climb_text} "
            f"armed={self.currently_armed} offboard={self.offboard_seen} "
            f"hb={self.offboard_mode_count} thrust={self.thrust_setpoint_count} "
            f"torque={self.torque_setpoint_count} "
            f"modes={modes} solver_thrust_n={solver_thrust_text} "
            f"published_thrust_n={published_thrust_text}"
        )
        early_failure = self._early_failure_guard(elapsed)
        if early_failure is not None:
            self.done = True
            self.success = False
            self.failure_reason = early_failure
            self.get_logger().error(f"FAIL: {self.failure_reason}")
            return
        if all(criteria.values()):
            self.done = True
            self.success = True
            self.get_logger().info("PASS: lift-off and hover validation succeeded.")
            return
        if elapsed >= self.config.timeout_sec:
            self.done = True
            self.success = False
            missing = [name for name, ok in criteria.items() if not ok]
            self.failure_reason = f"timeout after {elapsed:.1f}s; unmet criteria: {', '.join(missing)}"
            self.get_logger().error(f"FAIL: {self.failure_reason}")


def parse_args() -> ValidationConfig:
    parser = argparse.ArgumentParser(
        description="Validate PX4 takeoff/hover/landing behavior for cddp_mpc."
    )
    parser.add_argument("--validation-mode", choices=("offboard", "onboard"), default="offboard")
    parser.add_argument("--target-z", type=float, default=-3.0)
    parser.add_argument("--settle-tolerance", type=float, default=0.3)
    parser.add_argument("--hold-duration-sec", type=float, default=0.0)
    parser.add_argument("--timeout-sec", type=float, default=90.0)
    parser.add_argument("--min-heartbeat-msgs", type=int, default=10)
    parser.add_argument("--min-setpoint-msgs", type=int, default=10)
    parser.add_argument(
        "--require-hover-done",
        action="store_true",
        help="Require diagnostics to progress past HOVER into LAND or LAND_DONE.",
    )
    parser.add_argument("--min-climb-m", type=float, default=0.0)
    parser.add_argument("--climb-timeout-sec", type=float, default=25.0)
    parser.add_argument("--max-downward-speed-mps", type=float, default=0.0)
    parser.add_argument("--max-drop-from-start-m", type=float, default=0.0)
    parser.add_argument("--max-xy-error-m", type=float, default=0.0)
    parser.add_argument("--max-xy-drift-from-start-m", type=float, default=0.0)
    parser.add_argument("--require-landing", action="store_true")
    parser.add_argument("--landing-tolerance-m", type=float, default=0.3)
    args = parser.parse_args()
    return ValidationConfig(
        validation_mode=args.validation_mode,
        target_z=args.target_z,
        settle_tolerance=args.settle_tolerance,
        hold_duration_sec=args.hold_duration_sec,
        timeout_sec=args.timeout_sec,
        min_heartbeat_msgs=args.min_heartbeat_msgs,
        min_setpoint_msgs=args.min_setpoint_msgs,
        require_hover_done=args.require_hover_done,
        min_climb_m=args.min_climb_m,
        climb_timeout_sec=args.climb_timeout_sec,
        max_downward_speed_mps=args.max_downward_speed_mps,
        max_drop_from_start_m=args.max_drop_from_start_m,
        max_xy_error_m=args.max_xy_error_m,
        max_xy_drift_from_start_m=args.max_xy_drift_from_start_m,
        require_landing=args.require_landing,
        landing_tolerance_m=args.landing_tolerance_m,
    )


def main() -> int:
    config = parse_args()
    rclpy.init()
    node = HoverMissionValidator(config)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted by user.")
        node.done = True
        node.success = False
        node.failure_reason = "Interrupted by user."
    finally:
        success = node.success
        reason = node.failure_reason
        node.destroy_node()
        rclpy.shutdown()

    if not success and reason:
        print(reason)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())

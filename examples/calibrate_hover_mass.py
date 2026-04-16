#!/usr/bin/env python3
"""Estimate hover thrust and recommend hover calibration values for cddp_mpc."""

from __future__ import annotations

import argparse
import math
import statistics
import time
from dataclasses import dataclass

import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray


@dataclass
class CalibrationConfig:
    duration_sec: float = 25.0
    min_samples: int = 30
    vz_threshold_mps: float = 0.2
    gravity_mps2: float = 9.81
    max_thrust_n: float = 20.0
    include_takeoff_mode: bool = False
    require_armed_offboard: bool = True
    fmu_prefix: str = "/fmu"
    controller_prefix: str = "/cddp_mpc"


class HoverMassCalibrator(Node):
    """Collect steady hover samples from the configured cddp_mpc status topic."""

    def __init__(self, config: CalibrationConfig):
        super().__init__("hover_mass_calibrator")
        self.config = config
        self.start_time = time.monotonic()
        self.done = False
        self.success = False
        self.failure_reason = ""
        self.result: dict[str, float] = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.armed = False
        self.offboard = False
        self.diag_count = 0
        self.sample_count = 0
        self.mode = "unknown"
        self.current_z = float("nan")
        self.current_vz = float("nan")
        self.current_thrust = float("nan")
        self.hover_thrust_samples: list[float] = []

        fmu_prefix = self.config.fmu_prefix.rstrip("/") or "/fmu"
        controller_prefix = self.config.controller_prefix.rstrip("/") or "/cddp_mpc"

        self.create_subscription(
            VehicleStatus,
            f"{fmu_prefix}/out/vehicle_status",
            self._status_callback,
            qos_profile,
        )
        self.create_subscription(
            DiagnosticArray,
            f"{controller_prefix}/status",
            self._diagnostic_callback,
            qos_profile,
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info("Hover mass calibrator started.")

    def _status_callback(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @staticmethod
    def _safe_float(values: dict[str, str], key: str) -> float:
        raw = values.get(key)
        if raw is None:
            return float("nan")
        try:
            return float(raw)
        except (TypeError, ValueError):
            return float("nan")

    def _diagnostic_callback(self, msg: DiagnosticArray) -> None:
        self.diag_count += 1
        for status in msg.status:
            if getattr(status, "name", "") != "cddp_mpc":
                continue
            values = {kv.key: kv.value for kv in getattr(status, "values", [])}
            self.mode = str(getattr(status, "message", "")).strip() or "unknown"
            self.current_z = self._safe_float(values, "current_z_ned_m")
            self.current_vz = self._safe_float(values, "current_vz_ned_mps")
            self.current_thrust = self._safe_float(values, "published_cmd_thrust_n")

            mode_ok = self.mode == "HOVER" or (
                self.config.include_takeoff_mode and self.mode == "TAKEOFF"
            )
            state_ok = (not self.config.require_armed_offboard) or (self.armed and self.offboard)
            steady_vz = math.isfinite(self.current_vz) and abs(self.current_vz) <= self.config.vz_threshold_mps
            valid_thrust = math.isfinite(self.current_thrust) and self.current_thrust > 0.0

            if mode_ok and state_ok and steady_vz and valid_thrust:
                self.hover_thrust_samples.append(self.current_thrust)
                self.sample_count += 1

    def _finish(self) -> None:
        if self.sample_count < self.config.min_samples:
            self.done = True
            self.success = False
            self.failure_reason = (
                f"not enough steady hover samples: {self.sample_count} < {self.config.min_samples}"
            )
            return

        median_thrust = float(statistics.median(self.hover_thrust_samples))
        mean_thrust = float(statistics.fmean(self.hover_thrust_samples))
        thrust_std = (
            float(statistics.pstdev(self.hover_thrust_samples))
            if self.sample_count > 1
            else 0.0
        )
        mass_est = median_thrust / self.config.gravity_mps2
        norm_hover = median_thrust / max(self.config.max_thrust_n, 1e-6)
        self.result = {
            "hover_thrust_median_n": median_thrust,
            "hover_thrust_mean_n": mean_thrust,
            "hover_thrust_std_n": thrust_std,
            "hover_thrust_norm": norm_hover,
            "recommended_hover_thrust_n": median_thrust,
            "recommended_hover_thrust_ratio": norm_hover,
            "mass_kg_equivalent": mass_est,
            "mass_kg_recommended": mass_est,
            "samples": float(self.sample_count),
        }
        self.done = True
        self.success = True

    def _tick(self) -> None:
        if self.done:
            return

        elapsed = time.monotonic() - self.start_time
        z_text = "n/a" if not math.isfinite(self.current_z) else f"{self.current_z:.2f}"
        vz_text = "n/a" if not math.isfinite(self.current_vz) else f"{self.current_vz:.2f}"
        thrust_text = "n/a" if not math.isfinite(self.current_thrust) else f"{self.current_thrust:.2f}"
        self.get_logger().info(
            f"elapsed={elapsed:.1f}s mode={self.mode} armed={self.armed} offboard={self.offboard} "
            f"z={z_text} vz={vz_text} thrust_n={thrust_text} samples={self.sample_count}"
        )
        if elapsed >= self.config.duration_sec:
            self._finish()


def parse_args() -> CalibrationConfig:
    parser = argparse.ArgumentParser(
        description="Estimate hover thrust calibration from the configured cddp_mpc status topic."
    )
    parser.add_argument("--fmu-prefix", type=str, default="/fmu")
    parser.add_argument("--controller-prefix", type=str, default="/cddp_mpc")
    parser.add_argument("--duration-sec", type=float, default=25.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--vz-threshold-mps", type=float, default=0.2)
    parser.add_argument("--gravity-mps2", type=float, default=9.81)
    parser.add_argument("--max-thrust-n", type=float, default=20.0)
    parser.add_argument("--include-takeoff-mode", action="store_true")
    parser.add_argument("--allow-unarmed", action="store_true")
    args = parser.parse_args()
    return CalibrationConfig(
        duration_sec=args.duration_sec,
        min_samples=args.min_samples,
        vz_threshold_mps=args.vz_threshold_mps,
        gravity_mps2=args.gravity_mps2,
        max_thrust_n=args.max_thrust_n,
        include_takeoff_mode=args.include_takeoff_mode,
        require_armed_offboard=not args.allow_unarmed,
        fmu_prefix=args.fmu_prefix,
        controller_prefix=args.controller_prefix,
    )


def main() -> int:
    config = parse_args()
    rclpy.init()
    node = HoverMassCalibrator(config)
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
        result = dict(node.result)
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        if reason:
            print(reason)
        return 1

    print("Hover calibration result:")
    print(f"  samples={int(result['samples'])}")
    print(f"  hover_thrust_median_n={result['hover_thrust_median_n']:.3f}")
    print(f"  hover_thrust_mean_n={result['hover_thrust_mean_n']:.3f}")
    print(f"  hover_thrust_std_n={result['hover_thrust_std_n']:.3f}")
    print(f"  hover_thrust_norm={result['hover_thrust_norm']:.3f}")
    print(f"  recommended_hover_thrust_n={result['recommended_hover_thrust_n']:.3f}")
    print(f"  recommended_hover_thrust_ratio={result['recommended_hover_thrust_ratio']:.3f}")
    print(f"  mass_kg_equivalent={result['mass_kg_equivalent']:.3f}")
    print("")
    print("Set this in config/mpc_sitl.yaml:")
    print(f"  hover_thrust_n: {result['recommended_hover_thrust_n']:.3f}")
    print(f"  # mass equivalent: {result['mass_kg_recommended']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

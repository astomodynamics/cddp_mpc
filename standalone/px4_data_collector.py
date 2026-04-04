#!/usr/bin/env python3
"""Standalone PX4 data collector for synchronized flight datasets."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy import interpolate

try:
    from px4_msgs.msg import VehicleAngularVelocity
    HAS_ANGULAR_VELOCITY = True
except ImportError:
    HAS_ANGULAR_VELOCITY = False

try:
    from px4_msgs.msg import VehicleThrustSetpoint
    HAS_THRUST_SETPOINT = True
except ImportError:
    HAS_THRUST_SETPOINT = False

try:
    from px4_msgs.msg import VehicleTorqueSetpoint
    HAS_TORQUE_SETPOINT = True
except ImportError:
    HAS_TORQUE_SETPOINT = False


@dataclass
class TimestampedBuffer:
    """Buffer for time-indexed data from a single PX4 topic."""

    timestamps: list[float] = field(default_factory=list)
    data: list[np.ndarray] = field(default_factory=list)

    def add(self, timestamp_us: int, values: np.ndarray) -> None:
        self.timestamps.append(timestamp_us / 1e6)
        self.data.append(np.asarray(values, dtype=float))

    def __len__(self) -> int:
        return len(self.timestamps)

    def to_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.timestamps:
            return np.array([]), np.array([])
        return np.asarray(self.timestamps, dtype=float), np.stack(self.data)

    def interpolate_to(self, target_times: np.ndarray) -> np.ndarray:
        if len(self.timestamps) < 2:
            raise ValueError(f"Need at least 2 points to interpolate, got {len(self.timestamps)}")

        times, data = self.to_arrays()
        result = np.zeros((len(target_times), data.shape[1]), dtype=float)
        for idx in range(data.shape[1]):
            interpolator = interpolate.interp1d(
                times,
                data[:, idx],
                kind="linear",
                bounds_error=False,
                fill_value=(data[0, idx], data[-1, idx]),
            )
            result[:, idx] = interpolator(target_times)
        return result


def build_common_time_grid(buffers: list[TimestampedBuffer], dt: float) -> np.ndarray:
    """Build the overlapping time grid across all populated buffers."""

    populated = [buffer for buffer in buffers if len(buffer) >= 2]
    if not populated:
        raise ValueError("No populated buffers available for synchronization.")

    start = max(buffer.timestamps[0] for buffer in populated)
    end = min(buffer.timestamps[-1] for buffer in populated)
    if end <= start:
        raise ValueError("No overlapping time window across PX4 buffers.")

    step = max(dt, 1e-4)
    sample_count = int(np.floor((end - start) / step)) + 1
    if sample_count < 2:
        raise ValueError("Synchronization window is too small for the requested dt.")
    return start + np.arange(sample_count, dtype=float) * step


class PX4DataCollector(Node):
    """Collect synchronized PX4 state and command data."""

    def __init__(self) -> None:
        super().__init__("px4_data_collector")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.buffers: dict[str, TimestampedBuffer] = {
            "odometry": TimestampedBuffer(),
            "attitude": TimestampedBuffer(),
            "local_position": TimestampedBuffer(),
            "imu": TimestampedBuffer(),
            "angular_velocity": TimestampedBuffer(),
            "thrust_setpoint": TimestampedBuffer(),
            "torque_setpoint": TimestampedBuffer(),
        }
        self.collecting = False

        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._attitude_cb, qos)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._local_position_cb, qos
        )
        self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self._imu_cb, qos)

        if HAS_ANGULAR_VELOCITY:
          self.create_subscription(
              VehicleAngularVelocity,
              "/fmu/out/vehicle_angular_velocity",
              self._angular_velocity_cb,
              qos,
          )
        if HAS_THRUST_SETPOINT:
            self.create_subscription(
                VehicleThrustSetpoint,
                "/fmu/in/vehicle_thrust_setpoint",
                self._thrust_setpoint_cb,
                qos,
            )
        if HAS_TORQUE_SETPOINT:
            self.create_subscription(
                VehicleTorqueSetpoint,
                "/fmu/in/vehicle_torque_setpoint",
                self._torque_setpoint_cb,
                qos,
            )

        self.get_logger().info("PX4 data collector initialized.")

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        if not self.collecting:
            return
        data = np.concatenate(
            [
                np.asarray(msg.position, dtype=float),
                np.asarray(msg.velocity, dtype=float),
                np.asarray(msg.q, dtype=float),
                np.asarray(msg.angular_velocity, dtype=float),
            ]
        )
        self.buffers["odometry"].add(msg.timestamp, data)

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        if not self.collecting:
            return
        self.buffers["attitude"].add(msg.timestamp, np.asarray(msg.q, dtype=float))

    def _local_position_cb(self, msg: VehicleLocalPosition) -> None:
        if not self.collecting:
            return
        data = np.asarray([msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz], dtype=float)
        self.buffers["local_position"].add(msg.timestamp, data)

    def _imu_cb(self, msg: SensorCombined) -> None:
        if not self.collecting:
            return
        self.buffers["imu"].add(msg.timestamp, np.asarray(msg.accelerometer_m_s2, dtype=float))

    def _angular_velocity_cb(self, msg) -> None:  # noqa: ANN001
        if not self.collecting:
            return
        self.buffers["angular_velocity"].add(msg.timestamp, np.asarray(msg.xyz, dtype=float))

    def _thrust_setpoint_cb(self, msg) -> None:  # noqa: ANN001
        if not self.collecting:
            return
        self.buffers["thrust_setpoint"].add(msg.timestamp, np.asarray(msg.xyz, dtype=float))

    def _torque_setpoint_cb(self, msg) -> None:  # noqa: ANN001
        if not self.collecting:
            return
        self.buffers["torque_setpoint"].add(msg.timestamp, np.asarray(msg.xyz, dtype=float))

    def start(self) -> None:
        self.collecting = True
        self.get_logger().info("Started collecting PX4 data.")

    def stop(self) -> None:
        self.collecting = False
        summary = {name: len(buffer) for name, buffer in self.buffers.items()}
        self.get_logger().info(f"Stopped collecting PX4 data: {summary}")

    def build_dataset(self, dt: float) -> dict[str, np.ndarray]:
        time_grid = build_common_time_grid(
            [
                self.buffers["odometry"] if len(self.buffers["odometry"]) >= 2 else self.buffers["local_position"],
                self.buffers["imu"],
            ],
            dt,
        )

        dataset: dict[str, np.ndarray] = {"timestamps": time_grid}

        if len(self.buffers["odometry"]) >= 2:
            odom = self.buffers["odometry"].interpolate_to(time_grid)
            dataset["positions"] = odom[:, 0:3]
            dataset["velocities"] = odom[:, 3:6]
            dataset["quaternions"] = odom[:, 6:10]
            dataset["angular_velocities"] = odom[:, 10:13]
        else:
            local = self.buffers["local_position"].interpolate_to(time_grid)
            dataset["positions"] = local[:, 0:3]
            dataset["velocities"] = local[:, 3:6]
            dataset["quaternions"] = (
                self.buffers["attitude"].interpolate_to(time_grid)
                if len(self.buffers["attitude"]) >= 2
                else np.full((len(time_grid), 4), np.nan, dtype=float)
            )
            dataset["angular_velocities"] = (
                self.buffers["angular_velocity"].interpolate_to(time_grid)
                if len(self.buffers["angular_velocity"]) >= 2
                else np.full((len(time_grid), 3), np.nan, dtype=float)
            )

        dataset["accelerations"] = self.buffers["imu"].interpolate_to(time_grid)
        if len(self.buffers["thrust_setpoint"]) >= 2 and len(self.buffers["torque_setpoint"]) >= 2:
            thrust = self.buffers["thrust_setpoint"].interpolate_to(time_grid)
            torque = self.buffers["torque_setpoint"].interpolate_to(time_grid)
            dataset["controls"] = np.column_stack([thrust[:, 2], torque])
        else:
            dataset["controls"] = np.full((len(time_grid), 4), np.nan, dtype=float)
        return dataset

    def save_dataset(self, output_path: Path, dt: float) -> Path:
        dataset = self.build_dataset(dt)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(output_path, **dataset)
        self.get_logger().info(f"Saved synchronized dataset to {output_path}")
        return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0, help="Collection duration in seconds.")
    parser.add_argument("--dt", type=float, default=0.02, help="Output sample period in seconds.")
    parser.add_argument("--output", type=Path, required=True, help="Output .npz file path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    collector = PX4DataCollector()
    collector.start()
    start = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - start) < args.duration:
            rclpy.spin_once(collector, timeout_sec=0.1)
    except KeyboardInterrupt:
        collector.get_logger().warning("Interrupted by user.")
    finally:
        collector.stop()

    try:
        collector.save_dataset(args.output, args.dt)
    except ValueError as exc:
        collector.get_logger().error(str(exc))
        collector.destroy_node()
        rclpy.shutdown()
        return 1

    collector.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

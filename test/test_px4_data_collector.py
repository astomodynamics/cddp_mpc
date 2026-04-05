#!/usr/bin/env python3
"""Unit tests for the standalone PX4 data collector."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path

import numpy as np


class _FakeLogger:
    def __init__(self) -> None:
        self.messages: dict[str, list[str]] = {"info": [], "warn": [], "error": []}

    def info(self, msg: str) -> None:
        self.messages["info"].append(msg)

    def warning(self, msg: str) -> None:
        self.messages["warn"].append(msg)

    def error(self, msg: str) -> None:
        self.messages["error"].append(msg)


class _FakeNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self.subscriptions = []
        self._logger = _FakeLogger()

    def create_subscription(self, msg_type, topic, callback, qos):  # noqa: ANN001
        record = {"msg_type": msg_type, "topic": topic, "callback": callback, "qos": qos}
        self.subscriptions.append(record)
        return record

    def get_logger(self) -> _FakeLogger:
        return self._logger

    def destroy_node(self) -> None:
        return None


def _install_stub_modules() -> None:
    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda args=None: None
    rclpy_mod.shutdown = lambda: None
    rclpy_mod.ok = lambda: True
    rclpy_mod.spin_once = lambda node, timeout_sec=None: None

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _FakeNode

    qos_mod = types.ModuleType("rclpy.qos")

    class QoSProfile:
        def __init__(self, reliability, history, depth):
            self.reliability = reliability
            self.history = history
            self.depth = depth

    class ReliabilityPolicy:
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    qos_mod.QoSProfile = QoSProfile
    qos_mod.ReliabilityPolicy = ReliabilityPolicy
    qos_mod.HistoryPolicy = HistoryPolicy

    px4_mod = types.ModuleType("px4_msgs")
    px4_msg_mod = types.ModuleType("px4_msgs.msg")
    for name in [
        "SensorCombined",
        "VehicleAttitude",
        "VehicleLocalPosition",
        "VehicleOdometry",
        "VehicleAngularVelocity",
        "VehicleThrustSetpoint",
        "VehicleTorqueSetpoint",
    ]:
        setattr(px4_msg_mod, name, type(name, (), {}))

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["px4_msgs"] = px4_mod
    sys.modules["px4_msgs.msg"] = px4_msg_mod


def _load_module():
    _install_stub_modules()
    script_path = Path(__file__).resolve().parents[1] / "standalone" / "px4_data_collector.py"
    module_name = "_test_px4_data_collector"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


class PX4DataCollectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = _load_module()

    def test_timestamped_buffer_interpolates(self) -> None:
        buffer = self.module.TimestampedBuffer()
        buffer.add(0, np.array([0.0, 0.0]))
        buffer.add(1_000_000, np.array([2.0, 4.0]))

        values = buffer.interpolate_to(np.array([0.0, 0.5, 1.0]))

        np.testing.assert_allclose(values, np.array([[0.0, 0.0], [1.0, 2.0], [2.0, 4.0]]))

    def test_common_time_grid_uses_overlapping_window(self) -> None:
        a = self.module.TimestampedBuffer([0.0, 1.0, 2.0], [np.array([0.0]), np.array([1.0]), np.array([2.0])])
        b = self.module.TimestampedBuffer([0.5, 1.5, 2.5], [np.array([0.0]), np.array([1.0]), np.array([2.0])])

        grid = self.module.build_common_time_grid([a, b], 0.5)

        np.testing.assert_allclose(grid, np.array([0.5, 1.0, 1.5, 2.0]))

    def test_save_dataset_writes_npz(self) -> None:
        collector = self.module.PX4DataCollector()
        collector.buffers["odometry"].timestamps = [0.0, 1.0]
        collector.buffers["odometry"].data = [
            np.zeros(13, dtype=float),
            np.ones(13, dtype=float),
        ]
        collector.buffers["imu"].timestamps = [0.0, 1.0]
        collector.buffers["imu"].data = [
            np.zeros(3, dtype=float),
            np.ones(3, dtype=float),
        ]
        collector.buffers["thrust_setpoint"].timestamps = [0.0, 1.0]
        collector.buffers["thrust_setpoint"].data = [
            np.zeros(3, dtype=float),
            np.ones(3, dtype=float),
        ]
        collector.buffers["torque_setpoint"].timestamps = [0.0, 1.0]
        collector.buffers["torque_setpoint"].data = [
            np.zeros(3, dtype=float),
            np.ones(3, dtype=float),
        ]

        with tempfile.TemporaryDirectory() as tmpdir:
            output = Path(tmpdir) / "flight_data.npz"
            collector.save_dataset(output, 0.5)
            data = np.load(output)

            np.testing.assert_allclose(data["timestamps"], np.array([0.0, 0.5, 1.0]))
            self.assertEqual(data["positions"].shape, (3, 3))
            self.assertEqual(data["controls"].shape, (3, 4))


if __name__ == "__main__":
    unittest.main()

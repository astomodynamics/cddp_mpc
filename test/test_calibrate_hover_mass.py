#!/usr/bin/env python3
"""Unit tests for the hover-thrust calibration example."""

from __future__ import annotations

import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path


class _FakeLogger:
    def __init__(self) -> None:
        self.messages: dict[str, list[str]] = {"info": [], "warn": [], "error": []}

    def info(self, msg: str) -> None:
        self.messages["info"].append(msg)

    def warning(self, msg: str) -> None:
        self.messages["warn"].append(msg)

    def warn(self, msg: str) -> None:
        self.messages["warn"].append(msg)


class _FakeNode:
    def __init__(self, name: str) -> None:
        self.name = name
        self.subscriptions = []
        self.timers = []
        self._logger = _FakeLogger()

    def create_subscription(self, msg_type, topic, callback, qos):  # noqa: ANN001
        record = {
            "msg_type": msg_type,
            "topic": topic,
            "callback": callback,
            "qos": qos,
        }
        self.subscriptions.append(record)
        return record

    def create_timer(self, period_sec, callback):  # noqa: ANN001
        timer = {"period_sec": period_sec, "callback": callback}
        self.timers.append(timer)
        return timer

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
    px4_msg_mod.VehicleStatus = type(
        "VehicleStatus",
        (),
        {
            "ARMING_STATE_ARMED": 2,
            "NAVIGATION_STATE_OFFBOARD": 14,
        },
    )

    diag_mod = types.ModuleType("diagnostic_msgs")
    diag_msg_mod = types.ModuleType("diagnostic_msgs.msg")
    diag_msg_mod.DiagnosticArray = type("DiagnosticArray", (), {})

    sys.modules["rclpy"] = rclpy_mod
    sys.modules["rclpy.node"] = node_mod
    sys.modules["rclpy.qos"] = qos_mod
    sys.modules["px4_msgs"] = px4_mod
    sys.modules["px4_msgs.msg"] = px4_msg_mod
    sys.modules["diagnostic_msgs"] = diag_mod
    sys.modules["diagnostic_msgs.msg"] = diag_msg_mod


def _load_module():
    _install_stub_modules()
    script_path = Path(__file__).resolve().parents[1] / "examples" / "calibrate_hover_mass.py"
    module_name = "_test_calibrate_hover_mass"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _diag_status(mode: str, thrust_n: float, vz_mps: float):
    return types.SimpleNamespace(
        name="cddp_mpc",
        message=mode,
        values=[
            types.SimpleNamespace(key="current_vz_ned_mps", value=str(vz_mps)),
            types.SimpleNamespace(key="published_cmd_thrust_n", value=str(thrust_n)),
            types.SimpleNamespace(key="current_z_ned_m", value="-3.0"),
        ],
    )


class CalibrateHoverMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = _load_module()

    def test_collects_only_steady_hover_samples(self) -> None:
        config = self.module.CalibrationConfig(min_samples=2)
        node = self.module.HoverMassCalibrator(config)
        node.armed = True
        node.offboard = True

        node._diagnostic_callback(types.SimpleNamespace(status=[_diag_status("TAKEOFF", 15.0, 0.1)]))
        node._diagnostic_callback(types.SimpleNamespace(status=[_diag_status("HOVER", 14.3, 0.1)]))
        node._diagnostic_callback(types.SimpleNamespace(status=[_diag_status("HOVER", 14.5, 0.15)]))
        node._diagnostic_callback(types.SimpleNamespace(status=[_diag_status("HOVER", 14.7, 0.5)]))

        self.assertEqual(node.sample_count, 2)
        self.assertEqual(node.hover_thrust_samples, [14.3, 14.5])

    def test_finish_computes_hover_recommendation(self) -> None:
        config = self.module.CalibrationConfig(min_samples=3, gravity_mps2=9.81, max_thrust_n=20.0)
        node = self.module.HoverMassCalibrator(config)
        node.hover_thrust_samples = [14.2, 14.4, 14.6]
        node.sample_count = 3

        node._finish()

        self.assertTrue(node.success)
        self.assertTrue(node.done)
        self.assertAlmostEqual(node.result["recommended_hover_thrust_n"], 14.4, places=6)
        self.assertAlmostEqual(node.result["hover_thrust_norm"], 14.4 / 20.0, places=6)
        self.assertAlmostEqual(node.result["mass_kg_equivalent"], 14.4 / 9.81, places=6)

    def test_finish_fails_with_too_few_samples(self) -> None:
        config = self.module.CalibrationConfig(min_samples=2)
        node = self.module.HoverMassCalibrator(config)
        node.hover_thrust_samples = [14.4]
        node.sample_count = 1

        node._finish()

        self.assertFalse(node.success)
        self.assertTrue(node.done)
        self.assertIn("not enough steady hover samples", node.failure_reason)

    def test_prefix_configuration_is_applied_to_subscriptions(self) -> None:
        config = self.module.CalibrationConfig(
            fmu_prefix="/px4_0/fmu",
            controller_prefix="/px4_0/cddp_mpc",
        )

        node = self.module.HoverMassCalibrator(config)

        topics = {record["topic"] for record in node.subscriptions}
        self.assertIn("/px4_0/fmu/out/vehicle_status", topics)
        self.assertIn("/px4_0/cddp_mpc/status", topics)

    def test_safe_float_handles_bad_values(self) -> None:
        value = self.module.HoverMassCalibrator._safe_float({"x": "bad"}, "x")
        self.assertTrue(math.isnan(value))


if __name__ == "__main__":
    unittest.main()

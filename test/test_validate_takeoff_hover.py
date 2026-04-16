#!/usr/bin/env python3
"""Unit tests for the takeoff/hover/landing validator example."""

from __future__ import annotations

import importlib.util
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

    def error(self, msg: str) -> None:
        self.messages["error"].append(msg)


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
    px4_msg_mod.OffboardControlMode = type("OffboardControlMode", (), {})
    px4_msg_mod.VehicleOdometry = type("VehicleOdometry", (), {})
    px4_msg_mod.VehicleThrustSetpoint = type("VehicleThrustSetpoint", (), {})
    px4_msg_mod.VehicleTorqueSetpoint = type("VehicleTorqueSetpoint", (), {})
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
    script_path = Path(__file__).resolve().parents[1] / "examples" / "validate_takeoff_hover.py"
    module_name = "_test_validate_takeoff_hover"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _seed_success_basics(node) -> None:
    node.odom_count = 1
    node.status_count = 1
    node.diag_count = 1
    node.armed_seen = True
    node.altitude_reached = True
    node.current_z = -3.0
    node.start_z = -1.0
    node.modes_seen = {"HOVER"}
    node.solver_success_seen = True


class ValidateTakeoffHoverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.validator_module = _load_module()

    def test_offboard_validation_requires_offboard_signals(self) -> None:
        config = self.validator_module.ValidationConfig(validation_mode="offboard")
        node = self.validator_module.HoverMissionValidator(config)
        _seed_success_basics(node)

        criteria = node._criteria()

        self.assertFalse(criteria["offboard_seen"])
        self.assertFalse(criteria["heartbeat_stream"])
        self.assertFalse(criteria["thrust_stream"])
        self.assertFalse(criteria["torque_stream"])
        self.assertTrue(criteria["diagnostics_ok"])

    def test_onboard_validation_does_not_require_offboard_signals(self) -> None:
        config = self.validator_module.ValidationConfig(validation_mode="onboard")
        node = self.validator_module.HoverMissionValidator(config)
        _seed_success_basics(node)
        node.diag_count = 0
        node.modes_seen = set()
        node.solver_success_seen = False

        criteria = node._criteria()

        self.assertTrue(criteria["offboard_seen"])
        self.assertTrue(criteria["heartbeat_stream"])
        self.assertTrue(criteria["thrust_stream"])
        self.assertTrue(criteria["torque_stream"])
        self.assertTrue(criteria["diagnostics_ok"])

    def test_require_hover_done_fails_while_still_hovering(self) -> None:
        config = self.validator_module.ValidationConfig(
            validation_mode="offboard",
            require_hover_done=True,
        )
        node = self.validator_module.HoverMissionValidator(config)
        _seed_success_basics(node)

        criteria = node._criteria()

        self.assertFalse(criteria["diagnostics_ok"])

    def test_require_hover_done_passes_after_hover_completes(self) -> None:
        config = self.validator_module.ValidationConfig(
            validation_mode="offboard",
            require_hover_done=True,
        )
        node = self.validator_module.HoverMissionValidator(config)
        _seed_success_basics(node)
        node.modes_seen = {"HOVER", "LAND"}

        criteria = node._criteria()

        self.assertTrue(criteria["diagnostics_ok"])

    def test_prefix_configuration_is_applied_to_subscriptions(self) -> None:
        config = self.validator_module.ValidationConfig(
            fmu_prefix="/px4_1/fmu",
            controller_prefix="/px4_1/cddp_mpc",
        )

        node = self.validator_module.HoverMissionValidator(config)

        topics = {record["topic"] for record in node.subscriptions}
        self.assertIn("/px4_1/fmu/out/vehicle_odometry", topics)
        self.assertIn("/px4_1/fmu/in/offboard_control_mode", topics)
        self.assertIn("/px4_1/cddp_mpc/status", topics)

    def test_landing_requirement_checks_land_done_and_disarm(self) -> None:
        config = self.validator_module.ValidationConfig(
            validation_mode="offboard",
            require_landing=True,
            landing_tolerance_m=0.3,
        )
        node = self.validator_module.HoverMissionValidator(config)
        _seed_success_basics(node)
        node.current_z = -1.05
        node.disarmed_after_arming = True
        node.modes_seen = {"HOVER", "LAND", "LAND_DONE"}

        criteria = node._criteria()

        self.assertTrue(criteria["landing_ok"])

    def test_early_failure_guard_detects_altitude_drop(self) -> None:
        config = self.validator_module.ValidationConfig(max_drop_from_start_m=0.8)
        node = self.validator_module.HoverMissionValidator(config)
        node.armed_seen = True
        node.start_z = -1.0
        node.current_z = 0.0

        failure = node._early_failure_guard(elapsed=5.0)

        self.assertIsNotNone(failure)
        assert failure is not None
        self.assertIn("dropped below start altitude", failure)


if __name__ == "__main__":
    unittest.main()

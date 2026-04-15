from __future__ import annotations

import unittest
from pathlib import Path


class MultiVehicleLaunchDefaultsTest(unittest.TestCase):
    def test_multi_offboard_launch_exposes_fleet_arguments(self) -> None:
        launch_path = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "multi_quadrotor_offboard.launch.py"
        )
        text = launch_path.read_text()

        self.assertIn('DeclareLaunchArgument("vehicle_count", default_value="2")', text)
        self.assertIn('DeclareLaunchArgument("instance_start", default_value="0")', text)
        self.assertIn('"target_system": str(instance + 1)', text)
        self.assertIn('f"/{namespace}/fmu"', text)

    def test_multi_simulation_launch_uses_px4_instance_namespaces(self) -> None:
        launch_path = (
            Path(__file__).resolve().parents[1]
            / "launch"
            / "multi_quadrotor_simulation.launch.py"
        )
        text = launch_path.read_text()

        self.assertIn('"PX4_UXRCE_DDS_NS": namespace', text)
        self.assertIn('cmd=[str(px4_bin), "-i", str(instance)]', text)
        self.assertIn('DeclareLaunchArgument("vehicle_count", default_value="2")', text)

    def test_fleet_overlays_define_isolated_prefixes(self) -> None:
        fleet_dir = Path(__file__).resolve().parents[1] / "config" / "fleet"
        px4_0 = (fleet_dir / "px4_0.yaml").read_text()
        px4_1 = (fleet_dir / "px4_1.yaml").read_text()

        self.assertIn("fmu_prefix: /px4_0/fmu", px4_0)
        self.assertIn("controller_prefix: /px4_0/cddp_mpc", px4_0)
        self.assertIn("target_system: 1", px4_0)
        self.assertIn("fmu_prefix: /px4_1/fmu", px4_1)
        self.assertIn("controller_prefix: /px4_1/cddp_mpc", px4_1)
        self.assertIn("target_system: 2", px4_1)


if __name__ == "__main__":
    unittest.main()

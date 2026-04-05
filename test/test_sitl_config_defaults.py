from __future__ import annotations

import re
import unittest
from pathlib import Path


def _read_scalar(text: str, key: str) -> float:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?[0-9]*\.?[0-9]+)\s*$", text, re.MULTILINE)
    if match is None:
        raise AssertionError(f"missing config key: {key}")
    return float(match.group(1))


class SitlConfigDefaultsTest(unittest.TestCase):
    def test_sitl_params_are_scoped_to_launched_node_name(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertTrue(text.lstrip().startswith("cddp_mpc:\n"))

    def test_sitl_timing_defaults_match_refactor_targets(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertEqual(_read_scalar(text, "control_rate_hz"), 100.0)
        self.assertEqual(_read_scalar(text, "solve_rate_hz"), 20.0)
        self.assertEqual(_read_scalar(text, "mpc_dt"), 0.05)
        self.assertEqual(_read_scalar(text, "horizon_steps"), 20.0)
        self.assertEqual(_read_scalar(text, "stale_plan_timeout_s"), 0.25)

    def test_hover_lateral_trim_is_disabled_by_default_for_sitl(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertEqual(_read_scalar(text, "hover_lateral_kp_nm_per_m"), 0.0)
        self.assertEqual(_read_scalar(text, "hover_lateral_kd_nm_per_mps"), 0.0)
        self.assertEqual(_read_scalar(text, "hover_lateral_correction_limit_nm"), 0.0)

    def test_rotational_indi_parameters_exist(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertGreater(_read_scalar(text, "indi_blend_alpha"), 0.0)
        self.assertGreater(_read_scalar(text, "indi_rate_lpf_cutoff_hz"), 0.0)
        self.assertGreater(_read_scalar(text, "indi_accel_lpf_cutoff_hz"), 0.0)
        self.assertGreater(_read_scalar(text, "indi_torque_correction_limit_nm"), 0.0)
        self.assertGreater(_read_scalar(text, "indi_max_measurement_age_s"), 0.0)
        self.assertGreater(_read_scalar(text, "indi_max_body_rate_rad_s"), 0.0)
        self.assertGreater(_read_scalar(text, "command_validity_thrust_jump_n"), 0.0)
        self.assertGreater(_read_scalar(text, "command_validity_torque_jump_nm"), 0.0)

    def test_sitl_torque_normalization_matches_x500_baseline(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertAlmostEqual(_read_scalar(text, "max_roll_torque_nm"), 2.73, places=2)
        self.assertAlmostEqual(_read_scalar(text, "max_pitch_torque_nm"), 1.69, places=2)
        self.assertAlmostEqual(_read_scalar(text, "max_yaw_torque_nm"), 0.65, places=2)


if __name__ == "__main__":
    unittest.main()

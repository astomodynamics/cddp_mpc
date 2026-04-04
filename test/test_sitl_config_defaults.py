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
    def test_hover_lateral_hold_is_enabled_for_sitl(self) -> None:
        config_path = Path(__file__).resolve().parents[1] / "config" / "mpc_sitl.yaml"
        text = config_path.read_text()

        self.assertGreater(_read_scalar(text, "hover_lateral_kp_rad_s_per_m"), 0.0)
        self.assertGreater(_read_scalar(text, "hover_lateral_kd_rad_s_per_mps"), 0.0)
        self.assertGreater(_read_scalar(text, "hover_lateral_correction_limit_rad_s"), 0.0)


if __name__ == "__main__":
    unittest.main()

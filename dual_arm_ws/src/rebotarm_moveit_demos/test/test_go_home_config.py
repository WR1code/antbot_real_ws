from pathlib import Path
import unittest

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class TestGoHomeConfig(unittest.TestCase):
    def test_configs_are_complete_and_slow(self):
        cases = [
            ("go_home.yaml", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("go_home_rs.yaml", [0.0, 1.75, 0.7, -0.7, 0.0, 0.0]),
        ]
        for filename, expected_home in cases:
            with self.subTest(filename=filename):
                config = yaml.safe_load(
                    (PACKAGE_ROOT / "config" / filename).read_text()
                )
                parameters = config["go_home"]["ros__parameters"]

                self.assertEqual(
                    parameters["joint_names"],
                    [f"joint{index}" for index in range(1, 7)],
                )
                self.assertEqual(parameters["home_joint_values"], expected_home)
                self.assertEqual(
                    len(parameters["home_joint_values"]),
                    len(parameters["joint_names"]),
                )
                self.assertGreater(parameters["velocity_scaling"], 0.0)
                self.assertLessEqual(parameters["velocity_scaling"], 0.05)
                self.assertGreater(parameters["acceleration_scaling"], 0.0)
                self.assertLessEqual(parameters["acceleration_scaling"], 0.05)

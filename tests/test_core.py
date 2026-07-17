from __future__ import annotations

import sys
import tempfile
import unittest
import math
import ast
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from utils.math_utils import angle_error, lerp_angle, rmse
from utils.timeline import StateTimeline
from utils.io_utils import prepare_algorithm_input
from utils.ballistics import solve_ideal_ballistic_pitch
from utils.hit_checker import simulate_physical_shot
from utils.io_utils import load_config


class MathTests(unittest.TestCase):
    def test_angle_wrap(self):
        self.assertAlmostEqual(angle_error(-3.13, 3.13), 0.023185307179586445)

    def test_rmse(self):
        self.assertAlmostEqual(rmse([3.0, 4.0]), 3.5355339059327378)

    def test_timeline_interpolation(self):
        timeline = StateTimeline(
            [
                {"timestamp_ns": 0, "target_id": 1, "position": [0.0, 0.0, 0.0], "yaw": 3.1},
                {"timestamp_ns": 10, "target_id": 1, "position": [10.0, 0.0, 0.0], "yaw": -3.1},
            ],
            ("target_id",),
        )
        state = timeline.sample(5, 1)
        self.assertIsNotNone(state)
        self.assertAlmostEqual(state["position"][0], 5.0)
        self.assertAlmostEqual(abs(state["yaw"]), 3.141592653589793)


class GroundTruthIsolationTests(unittest.TestCase):
    def test_cpp_runner_has_no_ground_truth_access(self):
        source = (ROOT / "apps" / "auto_aim_offline.cpp").read_text(encoding="utf-8")
        self.assertNotIn("ground_truth", source.lower())
        self.assertNotIn("projected_observation", source)
        self.assertNotIn("armor_truth", source)

    def test_mock_runner_has_no_ground_truth_access(self):
        source = (ROOT / "utils" / "mock_runner.py").read_text(encoding="utf-8")
        self.assertNotIn("ground_truth", source.lower())
        self.assertNotIn("target_timeline", source)
        self.assertNotIn("armor_timeline", source)

    def test_algorithm_input_excludes_ground_truth(self):
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            dataset = base / "dataset"
            dataset.mkdir()
            for name in ("metadata.yaml", "frames.jsonl", "observations.jsonl"):
                (dataset / name).write_text("{}\n", encoding="utf-8")
            gt = dataset / "ground_truth"
            gt.mkdir()
            (gt / "secret.jsonl").write_text('{"secret":true}\n', encoding="utf-8")
            isolated = prepare_algorithm_input(dataset, base / "isolated")
            self.assertEqual(
                sorted(path.name for path in isolated.iterdir()),
                ["frames.jsonl", "metadata.yaml", "observations.jsonl"],
            )
            self.assertFalse((isolated / "ground_truth").exists())


class BallisticEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.physics = {
            "muzzle_position": [0.0, 0.0, 0.0],
            "gravity": 9.80665,
            "drag_coefficient": 0.0,
            "wind_velocity": [0.0, 0.0, 0.0],
            "integration_dt_s": 0.001,
            "max_flight_time_s": 2.0,
        }

    def test_ballistic_pitch_is_not_line_of_sight_pitch(self):
        target = [5.0, 0.0, 0.5]
        solution = solve_ideal_ballistic_pitch(target, 27.0, [0.0, 0.0, 0.0], self.physics)
        self.assertIsNotNone(solution)
        line_of_sight = -math.atan2(0.5, 5.0)
        self.assertLess(solution[0], line_of_sight)  # More negative means more elevation.

    def test_independent_pitch_hits_static_armor(self):
        solution = solve_ideal_ballistic_pitch([5.0, 0.0, 0.5], 27.0, [0.0, 0.0, 0.0], self.physics)
        rows = [
            {"timestamp_ns": timestamp, "target_id": 1, "armor_id": 0,
             "position": [5.0, 0.0, 0.5], "yaw": 0.0, "armor_type": "small"}
            for timestamp in (0, 1_000_000_000, 2_000_000_000)
        ]
        result = simulate_physical_shot(
            0.0, solution[0], 27.0, 0, 1, 0,
            StateTimeline(rows, ("target_id", "armor_id")),
            {"small": {"width": 0.135, "height": 0.056}}, self.physics,
        )
        self.assertTrue(result["hit"])
        self.assertTrue(result["evaluation_valid"])
        self.assertLessEqual(result["miss_distance"], 1e-9)


class CameraModelTests(unittest.TestCase):
    def test_offline_camera_parameters_match_tested_demo_config(self):
        offline = load_config(ROOT / "config" / "offline_test.yaml")["camera"]
        demo_text = (ROOT.parent / "2026_EGAIM" / "configs" / "demo.yaml").read_text(encoding="utf-8")

        def demo_array(name):
            match = re.search(rf"^{name}:\s*(\[[^\n]+\])", demo_text, re.MULTILINE)
            self.assertIsNotNone(match, name)
            return [float(value) for value in ast.literal_eval(match.group(1))]

        self.assertEqual([value for row in offline["camera_matrix"] for value in row], demo_array("camera_matrix"))
        self.assertEqual(offline["distortion_coefficients"], demo_array("distort_coeffs"))
        self.assertEqual(offline["R_gimbal2imubody"], demo_array("R_gimbal2imubody"))
        self.assertEqual(offline["R_camera2gimbal"], demo_array("R_camera2gimbal"))
        self.assertEqual(offline["t_camera2gimbal"], demo_array("t_camera2gimbal"))

    def test_generated_target_geometry_matches_normal_tracker_defaults(self):
        for case_name in ("static_target", "linear_target", "spinning_target"):
            case = load_config(ROOT / "config" / "cases" / f"{case_name}.yaml")["target"]
            self.assertEqual(case["radius"], 0.2)
            self.assertEqual(case["radius_delta"], 0.0)
            self.assertEqual(case["height_delta"], 0.0)
            self.assertEqual(case["armor_count"], 4)
        config = load_config(ROOT / "config" / "offline_test.yaml")
        self.assertEqual(config["simulation"]["ballistics"]["muzzle_position"], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()

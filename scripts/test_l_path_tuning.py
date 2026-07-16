#!/usr/bin/env python3
"""ROS-free tests for the fixed-L tuning collector."""

from __future__ import annotations

import sys
import tempfile
import unittest
from subprocess import CompletedProcess
from datetime import datetime, timezone
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import l_path_tuning as tuning  # noqa: E402


class FixedLGeometryTests(unittest.TestCase):
    def test_route_is_exact_fixed_ground_l(self) -> None:
        self.assertEqual(
            tuning.ROUTE_FLOATS,
            [
                0.0, 0.0, 0.0, 0.0,
                50.0, 0.0, 0.0, 74.1,
                150.0, 350.0, 0.0, 74.1,
            ],
        )

    def test_cross_track_for_first_leg(self) -> None:
        self.assertAlmostEqual(tuning.cross_track_cm("LEG_1", 20.0, 5.0), 5.0)

    def test_cross_track_for_second_leg(self) -> None:
        self.assertAlmostEqual(
            tuning.cross_track_cm("LEG_2", 60.0, 35.0), 0.0, places=6
        )

    def test_cross_track_rejects_non_straight_phase(self) -> None:
        with self.assertRaises(ValueError):
            tuning.cross_track_cm("TURN", 50.0, 0.0)

    def test_heading_error_wraps(self) -> None:
        self.assertAlmostEqual(tuning.angle_error_deg(-179.0, 179.0), 2.0)

    def test_first_leg_uses_previous_waypoint_yaw(self) -> None:
        self.assertEqual(tuning.segment_heading_deg("WAIT_START"), 0.0)
        self.assertEqual(tuning.segment_heading_deg("LEG_1"), 0.0)
        self.assertEqual(tuning.segment_heading_deg("TURN"), 74.1)
        self.assertEqual(tuning.segment_heading_deg("LEG_2"), 74.1)


class ParsingAndSegmentationTests(unittest.TestCase):
    def test_parse_overrides(self) -> None:
        self.assertEqual(
            tuning.parse_overrides(["straight_kp_w=0.22", "ki_w=0"]),
            {"straight_kp_w": 0.22, "ki_w": 0.0},
        )

    def test_parse_overrides_rejects_bad_assignment(self) -> None:
        for value in ("bad", "name=", "bad-name=1", "kp_w=nan"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                tuning.parse_overrides([value])

    def test_classify_fixed_targets(self) -> None:
        self.assertEqual(tuning.classify_target(None, 999.0), "WAIT_START")
        self.assertEqual(
            tuning.classify_target((0.0, 0.0, 0.0, 0.0), 1.0), "WAIT_START"
        )
        self.assertEqual(
            tuning.classify_target((50.0, 0.0, 0.0, 74.1), 10.0), "LEG_1"
        )
        self.assertEqual(
            tuning.classify_target((50.0, 0.0, 0.0, 74.1), 3.0), "TURN"
        )
        self.assertEqual(
            tuning.classify_target((150.0, 350.0, 0.0, 74.1), 100.0), "LEG_2"
        )

    def test_classify_rejects_unknown_target(self) -> None:
        with self.assertRaises(ValueError):
            tuning.classify_target((999.0, 999.0, 0.0, 0.0), 5.0)

    def test_effective_sign_flips_ignore_dead_zone(self) -> None:
        self.assertEqual(
            tuning.count_effective_sign_flips([0.1, 0.01, 0.0, -0.1], 0.02), 1
        )
        self.assertEqual(
            tuning.count_effective_sign_flips([0.01, -0.01, 0.0], 0.02), 0
        )

    def test_parameter_set_preserves_double_type(self) -> None:
        def fake_run(command: list[str]) -> CompletedProcess[str]:
            stdout = "Set parameter successful\n" if "set" in command else "/**:\n  ros__parameters: {}\n"
            return CompletedProcess(command, 0, stdout=stdout, stderr="")

        with mock.patch.object(tuning, "_run_cli", side_effect=fake_run) as run_cli:
            tuning._parameter_snapshot({"kp_v": 1.0}, dry_run=False)
        self.assertEqual(run_cli.call_args_list[0].args[0][-1], "1.0")


class SummaryTests(unittest.TestCase):
    @staticmethod
    def sample(
        t_s: float,
        segment: str,
        *,
        v: float = 0.0,
        w: float = 0.0,
        heading: float = 0.0,
        cross: float = 0.0,
    ) -> tuning.Sample:
        return tuning.Sample(
            t_s=t_s,
            segment=segment,
            target_x_cm=0.0,
            target_y_cm=0.0,
            target_yaw_deg=0.0,
            pose_x_cm=0.0,
            pose_y_cm=0.0,
            pose_yaw_deg=0.0,
            v_cmd_mps=v,
            w_cmd_rps=w,
            heading_error_deg=heading,
            cross_track_cm=cross,
            ground_enable=True,
            flight_enable=False,
        )

    def test_straight_summary_metrics(self) -> None:
        summary = tuning.summarize_samples(
            [
                self.sample(1.0, "LEG_1", v=0.1, w=0.1, heading=-2.0, cross=-3.0),
                self.sample(2.0, "LEG_1", v=0.2, w=-0.1, heading=2.0, cross=4.0),
            ]
        )["LEG_1"]
        self.assertEqual(summary["sample_count"], 2)
        self.assertAlmostEqual(summary["duration_s"], 1.0)
        self.assertAlmostEqual(summary["mean_speed_mps"], 0.15)
        self.assertAlmostEqual(summary["heading_error_rms_deg"], 2.0)
        self.assertAlmostEqual(summary["heading_error_peak_to_peak_deg"], 4.0)
        self.assertAlmostEqual(summary["cross_track_max_abs_cm"], 4.0)
        self.assertEqual(summary["w_sign_flips"], 1)

    def test_turn_summary_and_empty_leg(self) -> None:
        summary = tuning.summarize_samples(
            [
                self.sample(1.0, "TURN", heading=10.0),
                self.sample(2.0, "TURN", heading=-3.0),
            ]
        )
        self.assertEqual(summary["LEG_2"], {"sample_count": 0})
        self.assertAlmostEqual(summary["TURN"]["max_overshoot_deg"], 3.0)
        self.assertAlmostEqual(summary["TURN"]["final_yaw_error_deg"], 3.0)


class SafetyHelperTests(unittest.TestCase):
    def test_endpoint_requires_continuous_stability(self) -> None:
        latch = tuning.EndpointLatch(stable_s=0.5)
        self.assertFalse(latch.update(1.0, distance_cm=10.0, yaw_error_deg=4.0))
        self.assertFalse(latch.update(1.3, distance_cm=13.0, yaw_error_deg=4.0))
        self.assertFalse(latch.update(2.0, distance_cm=10.0, yaw_error_deg=4.0))
        self.assertTrue(latch.update(2.5, distance_cm=10.0, yaw_error_deg=4.0))

    def test_endpoint_rejects_bad_yaw(self) -> None:
        latch = tuning.EndpointLatch(stable_s=0.5)
        self.assertFalse(latch.update(1.0, distance_cm=5.0, yaw_error_deg=6.0))
        self.assertFalse(latch.update(2.0, distance_cm=5.0, yaw_error_deg=4.0))

    def test_run_directory_is_collision_safe(self) -> None:
        stamp = datetime(2026, 7, 16, 12, 34, 56, tzinfo=timezone.utc)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = tuning.create_run_directory(root, stamp)
            second = tuning.create_run_directory(root, stamp)
            self.assertTrue(first.is_dir())
            self.assertTrue(second.is_dir())
            self.assertNotEqual(first, second)


if __name__ == "__main__":
    unittest.main()

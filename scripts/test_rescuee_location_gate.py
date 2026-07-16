#!/usr/bin/env python3
"""Static regression checks for the fixed-demo rescuee event gate."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
BRIDGE = ROOT / "fly_car/src/activity_control_pkg/src/xmachine_bridge.cpp"
LAUNCH = ROOT / "fly_car/src/my_launch/launch/patrol_ground.launch.py"


class RescueeLocationGateTests(unittest.TestCase):
    def test_yolo_cannot_trigger_fc05_by_default(self):
        source = BRIDGE.read_text(encoding="utf-8")

        self.assertIn(
            'declare_parameter<bool>("yolo_trigger_rescuee_event", false)',
            source,
        )
        callback = source[source.index("void onDetections(") : source.index("static double normalizeDeg")]
        self.assertIn("if (!yolo_trigger_rescuee_event_)", callback)
        self.assertLess(
            callback.index("if (!yolo_trigger_rescuee_event_)"),
            callback.index("rescuee_remaining_ = resend_count_"),
        )

    def test_fixed_demo_location_is_the_only_enabled_trigger(self):
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('"rescuee_x_m", default_value="0.66"', launch)
        self.assertIn('"rescuee_y_m", default_value="-3.13"', launch)
        self.assertIn('"rescuee_check_hz", default_value="10.0"', launch)
        self.assertIn('"yolo_trigger_rescuee_event": False', launch)


if __name__ == "__main__":
    unittest.main()

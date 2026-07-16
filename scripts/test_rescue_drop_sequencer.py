import unittest
from types import SimpleNamespace

from rescue_drop_sequencer import RescueDropSequencer


class _FakeTfBuffer:
    def lookup_transform(self, *_args):
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))


class RescueDropSequencerTests(unittest.TestCase):
    def test_pose_uses_laser_height_instead_of_2d_tf_z(self):
        fake = SimpleNamespace(
            tf_buffer=_FakeTfBuffer(),
            has_height=True,
            current_height_cm=100.0,
        )

        self.assertEqual(
            RescueDropSequencer.pose(fake),
            (100.0, 200.0, 100.0, 0.0),
        )

    def test_descending_does_not_open_servo_until_height_reaches_50cm(self):
        events = []
        logger = SimpleNamespace(
            info=lambda *_args: None,
            error=lambda *_args: None,
        )
        fake = SimpleNamespace(
            state="DESCENDING",
            a=SimpleNamespace(drop_z=50.0, z_tol=8.0, timeout_s=20.0,
                              open_deg=180),
            pose=lambda: (0.0, 0.0, 100.0, 0.0),
            elapsed=lambda: 0.0,
            servo=lambda angle: events.append(angle),
            enter=lambda state: events.append(state),
            ascend=lambda: events.append("ASCENDING"),
            get_logger=lambda: logger,
        )

        RescueDropSequencer.tick(fake)
        self.assertEqual(events, [])

        fake.pose = lambda: (0.0, 0.0, 50.0, 0.0)
        RescueDropSequencer.tick(fake)
        self.assertEqual(events, [180, "DROPPING"])


if __name__ == "__main__":
    unittest.main()

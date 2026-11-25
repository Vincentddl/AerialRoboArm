import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path[:0] = [str(ROOT / "vendor"), str(ROOT)]

import cv2
import numpy as np

from arm_console.black_blocks import BlackBlockConfig, BlackBlockTracker
from arm_console.rectangle_distance import RectangleConfig, RectangleRangeManager


def rectangle(short=39, long=106, angle=0):
    gray = np.full((320, 400), 230, np.uint8)
    corners = cv2.boxPoints(((200, 160), (short, long), angle)).astype(np.int32)
    cv2.fillConvexPoly(gray, corners, 12)
    return gray


class RectangleTests(unittest.TestCase):
    def test_short_edge_is_stable_under_in_plane_rotation(self):
        for angle in (0, 30, 70):
            with self.subTest(angle=angle):
                block = BlackBlockTracker().process(rectangle(angle=angle), BlackBlockConfig()).blocks[0]
                self.assertEqual(len(block.rotated_box), 4)
                self.assertTrue(block.width_usable)
                self.assertAlmostEqual(block.short_side_px, 39, delta=2)
                self.assertGreater(block.circle_diameter_px, 100)  # Old circle would overestimate width.

    def test_calibration_uses_short_edge_and_separate_store(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            manager = RectangleRangeManager(Path(directory) / "rectangle.json",
                                            RectangleConfig(calibration_frames=3))
            manager.set_context("usb-video", 400, 320)
            tracker = BlackBlockTracker()
            observation = tracker.process(rectangle(), BlackBlockConfig())
            self.assertIn("待标定", manager.observe(1, observation).text)
            self.assertIn("采集中", manager.begin("calibration", 80))
            for frame_id in range(2, 5):
                display = manager.observe(frame_id, observation)
            self.assertAlmostEqual(display.current_cm, 80, delta=2)
            tracker.reset()
            smaller = tracker.process(rectangle(short=20, long=53), BlackBlockConfig())
            measured = manager.observe(5, smaller)
            self.assertGreater(measured.current_cm, 130)
            manager.set_context("other-camera", 400, 320)
            self.assertIsNone(manager.calibration)


if __name__ == "__main__":
    unittest.main()

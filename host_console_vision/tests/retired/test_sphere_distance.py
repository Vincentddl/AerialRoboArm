import tempfile
import unittest
from pathlib import Path
import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

import cv2
import numpy as np

from arm_console.black_blocks import BlackBlockConfig, BlackBlockTracker, diagnostic_mask
from arm_console.sphere_distance import SphereConfig, SphereRangeManager


def sphere_frame(radius=20, center=(160, 120), glare=False):
    gray = np.full((240, 320), 230, np.uint8)
    cv2.circle(gray, center, radius, 15, -1)
    if glare:
        cv2.circle(gray, (center[0] - 4, center[1] - 4), 5, 230, -1)
    return gray


class SphereTests(unittest.TestCase):
    def test_glare_tracking_and_diagnostic(self):
        tracker = BlackBlockTracker()
        cfg = BlackBlockConfig()
        first = tracker.process(sphere_frame(glare=True), cfg)
        self.assertEqual(len(first.blocks), 1)
        self.assertAlmostEqual(first.blocks[0].circle_diameter_px, 40, delta=2)
        self.assertTrue(first.blocks[0].diameter_usable)
        self.assertEqual(diagnostic_mask(sphere_frame(glare=True), cfg).width(), 320)
        lost = tracker.process(np.full((240, 320), 230, np.uint8), cfg)
        self.assertEqual(lost.missed_frames, 1)
        self.assertEqual(tracker.process(sphere_frame(center=(165, 120)), cfg).target_id, first.target_id)

    def test_one_point_calibration_inverse_distance_and_isolation(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            path = Path(directory) / "sphere.json"
            manager = SphereRangeManager(path, SphereConfig(calibration_frames=3, verification_frames=2))
            manager.set_context("camera-a", 320, 240)
            tracker = BlackBlockTracker()
            observation = tracker.process(sphere_frame(radius=20), BlackBlockConfig())
            self.assertIn("待标定", manager.observe(1, observation).text)
            self.assertIn("采集中", manager.begin("calibration", 50))
            for frame_id in range(2, 5):
                display = manager.observe(frame_id, observation)
            self.assertAlmostEqual(display.current_cm, 50, delta=1)
            tracker.reset()
            smaller = tracker.process(sphere_frame(radius=10), BlackBlockConfig())
            distance = manager.observe(5, smaller)
            self.assertGreater(distance.current_cm, 80)
            self.assertLess(distance.current_cm, 120)
            manager.set_context("camera-b", 320, 240)
            self.assertIsNone(manager.calibration)
            manager.set_context("camera-a", 320, 240)
            self.assertIsNotNone(manager.calibration)
            self.assertAlmostEqual(manager.calibration.estimate_cm(20), 100, delta=3)


if __name__ == "__main__":
    unittest.main()

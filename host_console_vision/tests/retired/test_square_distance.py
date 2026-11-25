from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

import cv2
import numpy as np
from PySide6.QtWidgets import QApplication

from arm_console.black_blocks import BlackBlockConfig, BlackBlockTracker, detect_black_blocks
from arm_console.models import VisionResult
from arm_console.pages import VisionDemoPage
from arm_console.square_distance import (SquareCalibrationStore, SquareDistanceConfig,
                                         SquareRangeManager)
from arm_console.video_capture import make_owned_qimage


app = QApplication.instance() or QApplication([])


def gray_square(x: int, y: int, side: int, width: int = 320, height: int = 240) -> np.ndarray:
    gray = np.full((height, width), 255, dtype=np.uint8)
    gray[y:y + side, x:x + side] = 0
    return gray


class SquareDistanceTests(unittest.TestCase):
    def test_square_geometry_rejects_tilt_occlusion_cube_side_and_roi_clipping(self):
        config = BlackBlockConfig()
        front = gray_square(120, 60, 40)
        square = detect_black_blocks(front, config)[0]
        self.assertAlmostEqual(square.square_side_px, 39, delta=0.5)

        tilted = np.full_like(front, 255)
        cv2.fillConvexPoly(tilted, np.array([[110, 55], [165, 62], [151, 102], [120, 105]]), 0)
        self.assertIsNone(detect_black_blocks(tilted, config)[0].square_side_px)

        occluded = front.copy()
        occluded[60:79, 120:139] = 255  # A missing corner changes the silhouette.
        self.assertIsNone(detect_black_blocks(occluded, config)[0].square_side_px)

        cube = np.full_like(front, 255)
        cv2.fillPoly(cube, [np.array([[110, 50], [149, 50], [166, 64],
                                      [166, 105], [126, 105], [110, 89]])], 0)
        self.assertIsNone(detect_black_blocks(cube, config)[0].square_side_px)

        clipped = gray_square(80, 60, 40)  # Touches SEARCH ROI left boundary x=W/4.
        self.assertIsNone(detect_black_blocks(clipped, config)[0].square_side_px)

    def test_calibration_inverse_depth_persistence_verification_and_preview_resize(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            store = SquareCalibrationStore(Path(temporary) / "calibrations.json")
            config = SquareDistanceConfig(calibration_frames=5, verification_frames=3)
            manager = SquareRangeManager(store, config)
            manager.set_context("usb-video-device-a", 320, 240)
            tracker = BlackBlockTracker()
            first = tracker.process(gray_square(120, 60, 40), BlackBlockConfig())
            uncalibrated = manager.observe(1, first)
            self.assertIsNone(uncalibrated.current_cm)
            self.assertIn("未标定", uncalibrated.text)
            self.assertIn("标定已开始", manager.begin("calibration", 50.0))
            for frame_id in range(2, 7):
                observation = tracker.process(gray_square(120, 60, 40), BlackBlockConfig())
                display = manager.observe(frame_id, observation)
            self.assertIsNotNone(display.current_cm, f"{display!r}; {manager.collection!r}")
            self.assertAlmostEqual(display.current_cm, 50.0, delta=0.2)
            self.assertIn("已保存", display.calibration_text)
            calibration = store.get("usb-video-device-a", 320, 240, config)
            self.assertIsNotNone(calibration)
            self.assertAlmostEqual(calibration.distance_cm(20), 2 * calibration.distance_cm(40))
            self.assertIsNone(store.get("usb-video-device-b", 320, 240, config))
            self.assertIsNone(store.get("usb-video-device-a", 640, 480, config))

            # A separate preview widget size cannot change original-frame edge pixels.
            page = VisionDemoPage()
            frame = make_owned_qimage(cv2.cvtColor(gray_square(120, 60, 40), cv2.COLOR_GRAY2BGR))
            result = VisionResult(320, 240, None, "测试", False, observation.blocks,
                                  frame_id=6, roi_overlay=True, distance_cm=display.current_cm)
            page.canvas.resize(640, 360)
            page.set_frame(frame, result)
            first_cm = page.result.distance_cm
            page.canvas.resize(960, 600)
            self.assertEqual(page.result.distance_cm, first_cm)
            self.assertEqual(page.result.black_blocks[0].square_side_px,
                             observation.blocks[0].square_side_px)

            self.assertIn("验证已开始", manager.begin("verification", 75.0))
            for frame_id in range(7, 10):
                smaller = tracker.process(gray_square(126, 67, 26), BlackBlockConfig())
                verified = manager.observe(frame_id, smaller)
            self.assertIn("误差", verified.verification_text)
            self.assertGreater(verified.current_cm, display.current_cm)

            manager.set_context("usb-video-device-a", 640, 480)
            self.assertIsNone(manager.calibration)
            self.assertIsNone(manager.last_measurement)
            manager.set_context("usb-video-device-a", 320, 240)
            self.assertIsNotNone(manager.calibration)

    def test_lost_frames_invalid_shape_and_target_switch_never_show_current_depth(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as temporary:
            store = SquareCalibrationStore(Path(temporary) / "calibrations.json")
            manager = SquareRangeManager(store, SquareDistanceConfig(calibration_frames=3))
            manager.set_context("camera-a", 320, 240)
            tracker = BlackBlockTracker()
            first_image = gray_square(115, 55, 40)
            first = tracker.process(first_image, BlackBlockConfig())
            manager.observe(1, first)
            manager.begin("calibration", 50)
            for frame_id in range(2, 5):
                measured = manager.observe(frame_id, tracker.process(first_image, BlackBlockConfig()))
            self.assertIsNotNone(measured.current_cm, f"{measured!r}; {manager.collection!r}")

            distractor = gray_square(185, 135, 50)
            with_both = first_image.copy()
            with_both[distractor == 0] = 0
            held = manager.observe(5, tracker.process(with_both, BlackBlockConfig()))
            self.assertAlmostEqual(held.current_cm, measured.current_cm)

            temporarily_lost = manager.observe(6, tracker.process(distractor, BlackBlockConfig()))
            self.assertIsNone(temporarily_lost.current_cm)
            self.assertIn("上次测量", temporarily_lost.text)
            self.assertIn("上位机处理时间", temporarily_lost.text)
            self.assertIsNone(tracker.process(None, BlackBlockConfig()))
            self.assertEqual(tracker.missed_frames, 1)

            reacquired = manager.observe(7, tracker.process(first_image, BlackBlockConfig()))
            self.assertIsNotNone(reacquired.current_cm)
            bad_pose = first_image.copy()
            bad_pose[55:74, 115:134] = 255
            invalid_pose = manager.observe(8, tracker.process(bad_pose, BlackBlockConfig()))
            self.assertIsNone(invalid_pose.current_cm)
            self.assertIn("姿态不符合", invalid_pose.text)

            # Restore a valid reading, then lose the target for exactly 20 good frames.
            manager.observe(9, tracker.process(first_image, BlackBlockConfig()))
            for frame_id in range(10, 30):
                lost = manager.observe(frame_id, tracker.process(distractor, BlackBlockConfig()))
            self.assertIsNone(lost.current_cm)
            self.assertNotIn("上次测量", lost.text)
            new_target = manager.observe(30, tracker.process(distractor, BlackBlockConfig()))
            self.assertIsNotNone(new_target.current_cm)
            self.assertNotEqual(new_target.current_cm, measured.current_cm)
            manager.reset_runtime(clear_context=True)
            self.assertIsNone(manager.last_measurement)
            self.assertIsNone(manager.context)


if __name__ == "__main__":
    unittest.main()

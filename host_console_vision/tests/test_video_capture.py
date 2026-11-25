from __future__ import annotations

import os
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

import cv2
import numpy as np
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from arm_console.video_capture import VideoCaptureService, VideoDevice, resolve_device
from arm_console.models import VisionResult
from arm_console.vision import VisionCanvas
from arm_console.window import MainWindow


app = QApplication.instance() or QApplication([])
USB = VideoDevice("USB Video", 2, cv2.CAP_DSHOW, r"\\?\usb#vid_1234&pid_5678#receiver-a")
OTHER = VideoDevice("Integrated Camera", 0, cv2.CAP_DSHOW, r"\\?\usb#vid_0001#builtin")


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        QTest.qWait(20)
    return predicate()


class FakeCapture:
    instances = []
    fail_after: int | None = None
    opened = True

    def __init__(self, index, backend):
        self.index, self.backend = index, backend
        self.reads = 0
        self.released = False
        self.instances.append(self)

    def isOpened(self):
        return self.opened

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: 640.0, cv2.CAP_PROP_FRAME_HEIGHT: 480.0,
                cv2.CAP_PROP_FPS: 30.0}.get(prop, 0.0)

    def read(self):
        time.sleep(0.015)
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            return False, None
        bgr = np.zeros((2, 4, 3), dtype=np.uint8)
        bgr[:, :] = (0, 0, 255)
        return True, bgr

    def release(self):
        self.released = True


class VideoCaptureTests(unittest.TestCase):
    def setUp(self):
        FakeCapture.instances = []
        FakeCapture.fail_after = None
        FakeCapture.opened = True

    def test_resolve_uses_device_path_not_camera_zero_or_name_alone(self):
        same_name = VideoDevice("USB Video", 3, cv2.CAP_DSHOW, r"\\?\usb#vid_1234#receiver-b")
        chosen = resolve_device(USB, [OTHER, same_name, USB])
        self.assertEqual(chosen.index, 2)
        self.assertIn("DirectShow 2", chosen.label)
        self.assertNotEqual(USB.label, same_name.label)
        with self.assertRaises(RuntimeError):
            resolve_device(USB, [OTHER, same_name])

    def test_letterbox_coordinate_mapping_uses_source_pixels(self):
        canvas = VisionCanvas()
        canvas.resize(400, 300)
        canvas.result = VisionResult(640, 360, None, "USB Video", False)
        self.assertEqual(canvas.source_to_widget(320, 180), (200, 150))
        self.assertEqual(canvas.widget_to_source(200, 150), (320, 180))
        self.assertIsNone(canvas.widget_to_source(200, 10))

    def test_shared_stream_pages_color_time_and_reopen(self):
        service = VideoCaptureService(enumerator=lambda: [OTHER, USB], capture_factory=FakeCapture)
        window = MainWindow(video_service=service)
        window.show()
        try:
            self.assertTrue(wait_until(lambda: window.video_select.currentData() == USB))
            self.assertEqual(FakeCapture.instances, [])  # 枚举不会打开摄像头
            with patch.object(window.controller, "request_mode") as request_mode:
                window.nav_buttons[1].click()
                window.nav_buttons[2].click()
                self.assertEqual(request_mode.call_count, 0)
            window.nav_buttons[1].click()
            window.video_open.click()
            self.assertTrue(wait_until(lambda: service.status.phase == "采集中" and window.demo_page.result is not None))
            self.assertTrue(wait_until(lambda: service.status.read_fps > 0))
            self.assertEqual((service.status.width, service.status.height), (4, 2))
            self.assertEqual(service.status.reported_fps, 30)
            first = service.latest_frame
            self.assertIsNotNone(first)
            self.assertEqual((first.width, first.height), (4, 2))
            self.assertGreater(first.frame_id, 0)
            self.assertLess(time.monotonic() - first.received_monotonic_s, 1)
            self.assertEqual(first.image.pixelColor(0, 0).getRgb()[:3], (255, 0, 0))
            self.assertIsNone(window.demo_page.result.detection)
            self.assertIn("未检测到黑色物块", window.demo_page.target_label.text())
            self.assertEqual((FakeCapture.instances[0].index, FakeCapture.instances[0].backend), (2, cv2.CAP_DSHOW))
            window.switch_page(2)
            self.assertTrue(wait_until(lambda: window.grab_page.result is not None))
            self.assertEqual(len(FakeCapture.instances), 1)
            window.video_close.click()
            self.assertIsNone(window.grab_page.canvas.image)
            self.assertTrue(wait_until(lambda: FakeCapture.instances[0].released and service.status.phase == "未打开"))
            self.assertTrue(wait_until(window.video_open.isEnabled))
            window.video_open.click()
            self.assertTrue(wait_until(lambda: len(FakeCapture.instances) == 2 and service.status.phase == "采集中"))
        finally:
            window.close()
        self.assertTrue(FakeCapture.instances[-1].released)

    def test_manual_page_shares_raw_video_without_control_commands(self):
        service = VideoCaptureService(enumerator=lambda: [USB], capture_factory=FakeCapture)
        window = MainWindow(video_service=service)
        window.show()
        try:
            self.assertTrue(wait_until(lambda: window.video_select.currentData() == USB))
            self.assertEqual(window.pages.currentIndex(), 0)
            self.assertTrue(window.video_controls.isVisible())
            with patch.object(window.controller, "request_extend") as extend, patch.object(
                    window.controller, "request_grip") as grip:
                window.video_open.click()
                self.assertTrue(wait_until(lambda: window.manual_page.video_canvas.image is not None))
                canvas = window.manual_page.video_canvas
                self.assertEqual(canvas.image.pixelColor(0, 0).getRgb()[:3], (255, 0, 0))
                self.assertIsNone(canvas.result.detection)
                self.assertFalse(canvas.result.roi_overlay)
                self.assertTrue(canvas.result.center_segments_overlay)
                window.switch_page(1)
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None))
                window.switch_page(0)
                self.assertTrue(wait_until(lambda: window.manual_page.video_canvas.image is not None))
                self.assertEqual(len(FakeCapture.instances), 1)
                self.assertTrue(window.video_controls.isVisible())
                window.video_close.click()
                self.assertIsNone(canvas.image)
                self.assertEqual(extend.call_count, 0)
                self.assertEqual(grip.call_count, 0)
        finally:
            window.close()

    def test_opened_without_frames_is_failure_and_stale_image_clears(self):
        FakeCapture.opened = False
        service = VideoCaptureService(enumerator=lambda: [USB], capture_factory=FakeCapture)
        try:
            self.assertTrue(service.open_device(USB))
            self.assertTrue(wait_until(lambda: service.status.phase == "失败"))
            self.assertIn("OBS", service.status.detail)
            self.assertTrue(FakeCapture.instances[0].released)
        finally:
            self.assertTrue(service.shutdown())

        FakeCapture.instances = []
        FakeCapture.opened = True
        FakeCapture.fail_after = 5
        service = VideoCaptureService(enumerator=lambda: [USB], capture_factory=FakeCapture)
        frames = []
        cleared = []
        service.frame_ready.connect(frames.append)
        service.frame_cleared.connect(lambda: cleared.append(True))
        try:
            self.assertTrue(service.open_device(USB))
            self.assertTrue(wait_until(lambda: bool(frames)))
            self.assertTrue(wait_until(lambda: service.status.phase == "失败", timeout=3.5))
            self.assertIsNone(service.latest_frame)
            self.assertTrue(cleared)
            self.assertTrue(FakeCapture.instances[0].released)
        finally:
            self.assertTrue(service.shutdown())

    def test_blocked_read_does_not_leave_old_frame_marked_live(self):
        class StallingCapture(FakeCapture):
            def read(self):
                if self.reads >= 4:
                    time.sleep(1.4)
                return super().read()

        service = VideoCaptureService(enumerator=lambda: [USB], capture_factory=StallingCapture)
        cleared = []
        service.frame_cleared.connect(lambda: cleared.append(True))
        try:
            self.assertTrue(service.open_device(USB))
            self.assertTrue(wait_until(lambda: service.status.phase == "采集中" and service.latest_frame is not None))
            self.assertTrue(wait_until(lambda: service.status.phase == "无有效画面", timeout=2.0))
            self.assertIsNone(service.latest_frame)
            self.assertTrue(cleared)
        finally:
            self.assertTrue(service.shutdown())


if __name__ == "__main__":
    unittest.main()

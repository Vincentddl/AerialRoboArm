from __future__ import annotations

import os
import math
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
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication

from arm_console.black_blocks import (BlackBlockAlignment, BlackBlockConfig,
                                      BlackBlockTracker, detect_black_blocks, roi_pixel_bounds)
from arm_console.vision import demo_center_regions, demo_overlay_columns, VisionCanvas
from arm_console.models import VisionResult
from arm_console.pages import VisionDemoPage
from arm_console.video_capture import make_owned_qimage
from arm_console.video_capture import VideoCaptureService, VideoDevice
from arm_console.window import MainWindow


app = QApplication.instance() or QApplication([])
USB = VideoDevice("USB Video", 2, cv2.CAP_DSHOW, r"\\?\usb#vid_1234&pid_5678#receiver-a")


def wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        QTest.qWait(20)
    return predicate()


class SyntheticCapture:
    scene = "empty"
    instances = []

    def __init__(self, index, backend):
        self.index, self.backend = index, backend
        self.released = False
        self.instances.append(self)

    def isOpened(self):
        return True

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: 320, cv2.CAP_PROP_FRAME_HEIGHT: 240,
                cv2.CAP_PROP_FPS: 25}.get(prop, 0)

    def read(self):
        time.sleep(0.015)
        if self.scene == "invalid":
            return False, None
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        if self.scene == "one":
            cv2.rectangle(frame, (100, 30), (139, 69), (0, 0, 0), -1)
        elif self.scene == "many":
            cv2.rectangle(frame, (100, 30), (139, 69), (0, 0, 0), -1)
            cv2.rectangle(frame, (175, 150), (224, 199), (0, 0, 0), -1)
        elif self.scene == "moved":
            cv2.rectangle(frame, (150, 20), (189, 59), (0, 0, 0), -1)
        elif self.scene == "left":
            cv2.rectangle(frame, (12, 30), (51, 69), (0, 0, 0), -1)
        elif self.scene == "right":
            cv2.rectangle(frame, (260, 30), (299, 69), (0, 0, 0), -1)
        elif self.scene == "dim":
            cv2.rectangle(frame, (100, 30), (139, 69), (80, 80, 80), -1)
        elif self.scene == "small":
            cv2.rectangle(frame, (110, 40), (129, 59), (0, 0, 0), -1)
        elif self.scene == "large":
            cv2.rectangle(frame, (100, 50), (219, 149), (0, 0, 0), -1)
        elif self.scene == "near":
            cv2.rectangle(frame, (140, 70), (169, 109), (0, 0, 0), -1)
        return True, frame

    def release(self):
        self.released = True


class BlackBlockTests(unittest.TestCase):
    def test_search_track_reacquire_decoy_timeout_and_valid_frame_count(self):
        tracker = BlackBlockTracker()
        config = BlackBlockConfig()

        def image(*rects):
            gray = np.full((240, 320), 255, dtype=np.uint8)
            for x1, y1, x2, y2 in rects:
                gray[y1:y2, x1:x2] = 0
            return gray

        near = (100, 30, 140, 70)
        moved = (110, 35, 150, 75)
        far_larger = (185, 135, 235, 185)
        first = tracker.process(image(near, (180, 30, 210, 60)), config)
        self.assertEqual(first.phase, "TRACK")
        self.assertEqual(first.blocks[0].x, 100)  # SEARCH chooses largest in central half.
        self.assertEqual(first.missed_frames, 0)

        held = tracker.process(image(moved, far_larger), config)
        self.assertEqual(held.phase, "TRACK")
        self.assertEqual(held.blocks[0].x, 110)  # Larger distant block cannot steal lock.
        self.assertEqual(held.missed_frames, 0)

        lost = tracker.process(image(far_larger), config)
        self.assertEqual(lost.phase, "TRACK")
        self.assertEqual(lost.blocks, ())
        self.assertEqual(lost.missed_frames, 1)
        self.assertEqual((lost.last_known.x, lost.last_known.y), (110, 35))
        self.assertIsNone(tracker.process(None, config))
        self.assertIsNone(tracker.process(np.empty((0, 0), dtype=np.uint8), config))
        self.assertEqual(tracker.missed_frames, 1)  # Failed read is not target absence.

        recovered = tracker.process(image((115, 37, 155, 77), far_larger), config)
        self.assertEqual(recovered.blocks[0].x, 115)
        self.assertEqual(recovered.missed_frames, 0)

        tracker.reset()
        tracker.process(image((200, 40, 230, 80)), config)
        outside_search_roi = tracker.process(image((245, 40, 275, 80)), config)
        self.assertEqual(outside_search_roi.phase, "TRACK")
        self.assertEqual(outside_search_roi.blocks, ())
        self.assertEqual(outside_search_roi.missed_frames, 1)  # TRACK obeys the same hard ROI.

        crossing_boundary = tracker.process(image((220, 40, 275, 80)), config)
        self.assertEqual(crossing_boundary.blocks, ())
        self.assertEqual(crossing_boundary.missed_frames, 2)  # Clipped in-ROI fragment is rejected.
        tracker.reset()
        tracker.process(image((115, 37, 155, 77)), config)

        for count in range(1, config.track_miss_limit + 1):
            lost = tracker.process(image(far_larger), config)
            self.assertEqual(lost.blocks, ())
            self.assertEqual(lost.missed_frames, count)
            self.assertEqual(lost.phase, "TRACK")
        self.assertIsNone(tracker.last)
        searched = tracker.process(image(far_larger), config)
        self.assertEqual(searched.phase, "TRACK")
        self.assertEqual(searched.blocks[0].x, 185)  # Next frame returns to SEARCH ROI.

        tracker.reset()
        outside_only = tracker.process(image((5, 30, 65, 90)), config)
        self.assertEqual(outside_only.phase, "SEARCH")
        self.assertEqual(outside_only.blocks, ())

    def test_central_half_roi_raw_coordinates_and_overlay_columns_at_resolutions(self):
        config = BlackBlockConfig(gray_threshold=70, min_area_px=250, max_area_percent=25)
        for width, height in ((320, 240), (640, 480), (800, 600), (641, 360)):
            image = np.full((height, width), 255, dtype=np.uint8)
            self.assertEqual(detect_black_blocks(image, config), ())
            left, right = roi_pixel_bounds(width)
            self.assertEqual((left, right),
                             (math.ceil(width / 4), math.ceil(3 * width / 4)))
            image[30:70, 12:52] = 0  # left quarter
            image[30:70, width - 52:width - 12] = 0  # right quarter
            self.assertEqual(detect_black_blocks(image, config), ())
            center_x = width // 2 + 35
            image[30:70, center_x - 20:center_x + 20] = 0
            blocks = detect_black_blocks(image, config)
            self.assertEqual(len(blocks), 1)
            self.assertAlmostEqual(blocks[0].center_x, center_x - 0.5, delta=1)
            self.assertAlmostEqual(blocks[0].center_y, 49.5, delta=1)
            cols = demo_overlay_columns(width, blocks)
            self.assertEqual(cols[:3], (width / 4, width / 2, 3 * width / 4))
            self.assertEqual(cols[3], (blocks[0].center_x,))
            canvas = VisionCanvas()
            canvas.resize(700, 400)
            canvas.result = VisionResult(width, height, None, "测试", False,
                                         blocks, 1, True)
            for x in (*cols[:3], *cols[3]):
                widget_x, widget_y = canvas.source_to_widget(x, height / 2)
                source_x, source_y = canvas.widget_to_source(widget_x, widget_y)
                self.assertAlmostEqual(source_x, x)
                self.assertAlmostEqual(source_y, height / 2)
        self.assertEqual(detect_black_blocks(np.zeros((240, 320), dtype=np.uint8), config), ())

    def test_center_line_is_split_into_five_bottom_to_top_regions(self):
        for height in (240, 360, 481, 720):
            regions = demo_center_regions(height)
            self.assertEqual([number for number, _, _ in regions], [5, 4, 3, 2, 1])
            self.assertEqual(regions[0][1], 0)
            self.assertEqual(regions[-1][2], height)
            for previous, current in zip(regions, regions[1:]):
                self.assertAlmostEqual(previous[2], current[1])
            bottom_region = next(item for item in regions if item[0] == 1)
            top_region = next(item for item in regions if item[0] == 5)
            self.assertAlmostEqual(bottom_region[1], height * 4 / 5)
            self.assertAlmostEqual(top_region[2], height / 5)

    def test_max_area_is_percentage_of_original_frame_not_roi(self):
        image = np.full((200, 400), 255, dtype=np.uint8)
        image[50:150, 150:250] = 0  # contour ~9801 px; full image 80000 px
        self.assertEqual(len(detect_black_blocks(image, BlackBlockConfig(max_area_percent=15))), 1)
        self.assertEqual(detect_black_blocks(image, BlackBlockConfig(max_area_percent=10)), ())

    def test_selects_largest_contour_area_not_largest_bounding_box(self):
        image = np.full((300, 400), 255, dtype=np.uint8)
        # The L has a 70x70 box, larger than the solid square's 55x55 box.
        # Its filled contour area is smaller, so the square must win.
        image[20:90, 120:144] = 0
        image[20:44, 120:190] = 0
        image[180:235, 220:275] = 0
        blocks = detect_black_blocks(image, BlackBlockConfig())
        self.assertEqual(len(blocks), 1)
        self.assertEqual((blocks[0].x, blocks[0].y), (220, 180))
        self.assertEqual((blocks[0].width, blocks[0].height), (55, 55))

    def test_outside_roi_and_oversized_regions_cannot_win(self):
        image = np.full((300, 400), 255, dtype=np.uint8)
        image[10:150, 5:95] = 0  # Larger dark region outside the central ROI.
        image[10:120, 120:210] = 0  # Within ROI, but over 4% of the full frame.
        image[170:200, 140:170] = 0
        image[220:260, 230:270] = 0  # Largest qualified contour.
        blocks = detect_black_blocks(image, BlackBlockConfig(max_area_percent=4))
        self.assertEqual(len(blocks), 1)
        self.assertEqual((blocks[0].x, blocks[0].y), (230, 220))
        self.assertLess(blocks[0].area_px, 3000)

    def test_alignment_near_cross_hold_release_disappear_and_switch(self):
        tracker = BlackBlockAlignment()
        config = BlackBlockConfig()

        def selected(rectangles):
            image = np.full((240, 320), 255, dtype=np.uint8)
            for x1, y1, x2, y2 in rectangles:
                image[y1:y2, x1:x2] = 0
            blocks = detect_black_blocks(image, config)
            return blocks[0] if blocks else None

        off_line = selected([(100, 30, 140, 70)])
        self.assertFalse(off_line.touches_center_line)
        self.assertFalse(tracker.update(off_line, 320, 5))  # ordinary color

        near = selected([(145, 30, 175, 70)])
        self.assertTrue(tracker.update(near, 320, 5))
        offset_but_touching = selected([(150, 30, 230, 70)])
        self.assertGreater(abs(offset_but_touching.center_x - 160), 5)
        self.assertTrue(offset_but_touching.touches_center_line)
        self.assertTrue(tracker.update(offset_but_touching, 320, 5))  # keep green
        departed = selected([(170, 30, 230, 70)])
        self.assertFalse(departed.touches_center_line)
        self.assertFalse(tracker.update(departed, 320, 5))

        # A narrow component can have a centroid inside tolerance while its
        # actual black pixels are already wholly to one side of the line.
        tracker.reset()
        near_without_contact = selected([(161, 30, 170, 70)])
        self.assertAlmostEqual(near_without_contact.center_x, 165)
        self.assertFalse(near_without_contact.touches_center_line)
        self.assertTrue(tracker.update(near_without_contact, 320, 5))
        self.assertFalse(tracker.update(near_without_contact, 320, 5))
        self.assertFalse(tracker.update(near_without_contact, 320, 5))

        tracker.reset()
        left = selected([(135, 30, 155, 60)])
        right = selected([(165, 30, 185, 60)])
        self.assertFalse(tracker.update(left, 320, 5))
        self.assertFalse(right.touches_center_line)
        self.assertTrue(tracker.update(right, 320, 5))  # skipped center between frames
        self.assertFalse(tracker.update(right, 320, 5))  # no black pixels on line

        self.assertTrue(tracker.update(near, 320, 5))
        switched = selected([(155, 150, 230, 190)])
        self.assertTrue(switched.touches_center_line)
        self.assertFalse(tracker.update(switched, 320, 5))  # new block cannot inherit green
        self.assertFalse(tracker.update(None, 320, 5))
        self.assertFalse(tracker.update(switched, 320, 5))

        # Both blocks remain visible; the largest eligible contour changes.
        first_selected = selected([(145, 30, 185, 70), (150, 130, 200, 150)])
        next_selected = selected([(145, 30, 185, 70), (150, 130, 230, 170)])
        self.assertEqual(first_selected.y, 30)
        self.assertEqual(next_selected.y, 130)
        self.assertTrue(next_selected.touches_center_line)
        tracker.reset()
        self.assertTrue(tracker.update(first_selected, 320, 5))
        self.assertFalse(tracker.update(next_selected, 320, 5))

        # Even nearby boxes can be distinct: the old winner remains as a
        # smaller candidate and must not hand its green state to the new one.
        nearby_first = selected([(145, 30, 185, 70), (145, 75, 225, 93)])
        nearby_next = selected([(145, 30, 185, 70), (145, 75, 225, 100)])
        self.assertEqual(nearby_first.y, 30)
        self.assertEqual(nearby_next.y, 75)
        self.assertTrue(nearby_next.touches_center_line)
        tracker.reset()
        self.assertTrue(tracker.update(nearby_first, 320, 5))
        self.assertFalse(tracker.update(nearby_next, 320, 5))

    def test_center_contact_uses_selected_binary_component(self):
        image = np.full((240, 320), 255, dtype=np.uint8)
        image[30:100, 180:230] = 0  # selected largest, right of center
        image[150:170, 155:165] = 0  # another dark region intersects center
        selected = detect_black_blocks(image, BlackBlockConfig(min_area_px=100))[0]
        self.assertEqual(selected.x, 180)
        self.assertFalse(selected.touches_center_line)

    def test_demo_page_lost_state_does_not_present_old_detection(self):
        image = make_owned_qimage(np.full((240, 320, 3), 255, dtype=np.uint8))
        page = VisionDemoPage()
        block = detect_black_blocks(np.pad(np.zeros((40, 40), dtype=np.uint8),
                                            ((30, 170), (100, 180)), constant_values=255),
                                     BlackBlockConfig())[0]
        page.set_frame(image, VisionResult(320, 240, None, "测试", False,
                                           (block,), 1, True, True, "TRACK", 0, 20))
        self.assertIn("已对齐", page.target_label.text())
        page.set_frame(image, VisionResult(320, 240, None, "测试", False,
                                           (), 2, True, False, "TRACK", 3, 20))
        self.assertEqual(page.target_label.text(), "未检测到黑色物块")
        self.assertEqual(page.coords_label.text(), "图像坐标：—")
        self.assertEqual(page.canvas.result.black_blocks, ())
        self.assertFalse(page.canvas.result.black_block_aligned)
        page.clear_source()
        self.assertIsNone(page.canvas.result)
        page.close()

    def test_canvas_roi_and_distinct_center_lines_clear_with_target(self):
        source = np.full((240, 320, 3), 255, dtype=np.uint8)
        source[30:70, 100:140] = 0
        blocks = detect_black_blocks(cv2.cvtColor(source, cv2.COLOR_BGR2GRAY), BlackBlockConfig())
        canvas = VisionCanvas()
        canvas.resize(640, 480)
        canvas.show()
        try:
            image = make_owned_qimage(source)
            canvas.set_frame(image, VisionResult(320, 240, None, "测试", False, blocks, 1, True))
            QTest.qWait(40)
            painted = canvas.grab().toImage()
            cy = 200
            center = painted.pixelColor(320, cy)
            centroid = painted.pixelColor(240, cy)
            boundary = painted.pixelColor(160, cy)
            outside = painted.pixelColor(40, cy)
            self.assertGreater(center.red(), center.blue())  # amber fixed image center
            self.assertGreater(centroid.blue(), centroid.green())  # magenta contour centroid
            self.assertGreater(boundary.blue(), boundary.red())  # cyan ROI boundary
            self.assertLess(outside.red(), 220)  # dimmed exterior
            normal_box = painted.pixelColor(200, 100)
            canvas.set_frame(image, VisionResult(320, 240, None, "测试", False,
                                                 blocks, 1, True, True))
            QTest.qWait(40)
            aligned_box = canvas.grab().toImage().pixelColor(200, 100)
            self.assertGreater(aligned_box.red(), normal_box.red())
            self.assertLess(aligned_box.green(), normal_box.green())
            self.assertLess(aligned_box.blue(), normal_box.blue())
            canvas.set_frame(image, VisionResult(320, 240, None, "测试", False, (), 2, True))
            QTest.qWait(40)
            cleared = canvas.grab().toImage().pixelColor(240, cy)
            self.assertGreater(cleared.green(), 240)  # old centroid line removed
        finally:
            canvas.close()

    def test_manual_canvas_draws_five_center_segments_without_roi(self):
        source = np.full((240, 320, 3), 255, dtype=np.uint8)
        canvas = VisionCanvas()
        canvas.resize(640, 480)
        canvas.show()
        try:
            canvas.set_frame(make_owned_qimage(source),
                             VisionResult(320, 240, None, "测试", False,
                                          center_segments_overlay=True))
            QTest.qWait(40)
            painted = canvas.grab().toImage()
            for number, top, bottom in demo_center_regions(240):
                y = round((top + bottom) / 2) * 2
                center = painted.pixelColor(320, y)
                self.assertGreater(center.red(), center.blue(), number)
            for x in (40, 160, 480, 600):
                outside = painted.pixelColor(x, 400)
                self.assertGreater(outside.red(), 240)  # no ROI shading or boundaries
        finally:
            canvas.close()

    def test_shared_video_demo_overlay_page_switch_close_and_no_motor_commands(self):
        SyntheticCapture.scene = "empty"
        SyntheticCapture.instances = []
        video = VideoCaptureService(enumerator=lambda: [USB], capture_factory=SyntheticCapture)
        window = MainWindow(video_service=video)
        window.show()
        try:
            self.assertEqual(window.serial.state.value, "未连接")
            self.assertTrue(wait_until(lambda: window.video_select.currentData() == USB))
            with patch.object(window.serial, "send", wraps=window.serial.send) as serial_send, \
                 patch.object(window.controller, "request_grab") as grab, \
                 patch.object(window.controller, "submit_visual_target") as visual_target:
                window.switch_page(1)
                window.video_open.click()
                self.assertTrue(wait_until(lambda: video.status.phase == "采集中" and
                                           window.demo_page.result is not None))
                self.assertEqual(window.demo_page.result.black_blocks, ())
                self.assertIn("未检测到黑色物块", window.demo_page.target_label.text())
                self.assertEqual(serial_send.call_count, 0)
                SyntheticCapture.scene = "one"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           len(window.demo_page.result.black_blocks or ()) == 1))
                first = window.demo_page.result
                self.assertGreater(first.frame_id, 0)
                self.assertEqual((first.width, first.height), (320, 240))
                self.assertIn("(120, 50)", window.demo_page.coords_label.text())
                self.assertTrue(first.roi_overlay)
                mapped = window.demo_page.canvas.source_to_widget(first.black_blocks[0].center_x,
                                                                   first.black_blocks[0].center_y)
                self.assertIsNotNone(mapped)
                window.demo_page.canvas.resize(600, 360)
                mapped = window.demo_page.canvas.source_to_widget(first.black_blocks[0].center_x,
                                                                   first.black_blocks[0].center_y)
                source = window.demo_page.canvas.widget_to_source(*mapped)
                self.assertAlmostEqual(source[0], first.black_blocks[0].center_x)
                self.assertAlmostEqual(source[1], first.black_blocks[0].center_y)
                SyntheticCapture.scene = "many"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.frame_id > first.frame_id + 1 and
                                           window.demo_page.result.black_blocks and
                                           window.demo_page.result.black_blocks[0].x == 100))
                self.assertEqual(len(window.demo_page.result.black_blocks), 1)
                self.assertIn("(120, 50)", window.demo_page.coords_label.text())
                self.assertIn("TRACK", window.demo_page.tracking_label.text())
                self.assertIn("轮廓面积", window.demo_page.coords_label.text())
                self.assertEqual(len(demo_overlay_columns(320, window.demo_page.result.black_blocks)[3]), 1)
                SyntheticCapture.scene = "moved"
                self.assertTrue(wait_until(lambda: len(window.demo_page.result.black_blocks or ()) == 1 and
                                           window.demo_page.result.black_blocks[0].center_x > 160))
                self.assertNotIn("(120, 50)", window.demo_page.coords_label.text())
                previous_id = window.demo_page.result.frame_id
                SyntheticCapture.scene = "left"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.frame_id > previous_id and
                                           window.demo_page.result.black_blocks == ()))
                previous_id = window.demo_page.result.frame_id
                SyntheticCapture.scene = "right"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.frame_id > previous_id and
                                           window.demo_page.result.black_blocks == ()))
                SyntheticCapture.scene = "empty"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.black_blocks == ()))
                self.assertIn("未检测到黑色物块", window.demo_page.target_label.text())
                self.assertEqual(window.demo_page.coords_label.text(), "图像坐标：—")
                self.assertEqual(demo_overlay_columns(320, window.demo_page.result.black_blocks)[3], ())

                # All three controls update the open stream; no second capture is created.
                SyntheticCapture.scene = "dim"
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.frame_id > previous_id and
                                           window.demo_page.result.black_blocks == ()))
                threshold = window.demo_page.gray_threshold
                self.assertFalse(threshold.keyboardTracking())
                threshold.lineEdit().setFocus()
                threshold.lineEdit().selectAll()
                QTest.keyClicks(threshold.lineEdit(), "100")
                self.assertEqual(threshold.value(), 70)  # unconfirmed text has no effect
                QTest.keyClick(threshold.lineEdit(), Qt.Key.Key_Return)
                self.assertEqual(threshold.value(), 100)
                self.assertTrue(wait_until(lambda: len(window.demo_page.result.black_blocks or ()) == 1))
                threshold.setValue(70)
                self.assertTrue(wait_until(lambda: window.demo_page.result.black_blocks == ()))
                SyntheticCapture.scene = "small"
                self.assertTrue(wait_until(lambda: len(window.demo_page.result.black_blocks or ()) == 1))
                window.demo_page.min_area.setValue(500)
                self.assertTrue(wait_until(lambda: window.demo_page.result.black_blocks == ()))
                window.demo_page.min_area.setValue(200)
                self.assertTrue(wait_until(lambda: len(window.demo_page.result.black_blocks or ()) == 1))
                SyntheticCapture.scene = "large"
                self.assertTrue(wait_until(lambda: window.demo_page.result.black_blocks and
                                           window.demo_page.result.black_blocks[0].area_px > 9000,
                                           timeout=5.0),
                                f"large scene: {window.demo_page.result!r}")
                window.demo_page.max_area_percent.setValue(5.0)
                self.assertTrue(wait_until(lambda: window.demo_page.result.black_blocks == ()))
                window.demo_page.max_area_percent.setValue(20.0)
                self.assertTrue(wait_until(lambda: len(window.demo_page.result.black_blocks or ()) == 1))
                SyntheticCapture.scene = "near"
                self.assertTrue(wait_until(lambda: window.demo_page.result.black_blocks and
                                           window.demo_page.result.black_blocks[0].x == 140 and
                                           not window.demo_page.result.black_block_aligned,
                                           timeout=5.0))
                window.demo_page.alignment_tolerance.setValue(8)
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.black_block_aligned))
                window.demo_page.alignment_tolerance.setValue(5)
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None and
                                           window.demo_page.result.black_blocks and
                                           not window.demo_page.result.black_block_aligned))
                self.assertEqual(len(SyntheticCapture.instances), 1)
                self.assertEqual(video.status.phase, "采集中")
                window.switch_page(2)
                self.assertTrue(wait_until(lambda: window.grab_page.result is not None))
                self.assertIsNotNone(window.grab_page.result.black_blocks)
                self.assertIsNone(window.demo_page.result)
                window.switch_page(1)
                self.assertTrue(wait_until(lambda: window.demo_page.result is not None))
                self.assertEqual(len(SyntheticCapture.instances), 1)
                window.video_close.click()
                self.assertIsNone(window.demo_page.canvas.image)
                self.assertIsNone(window.demo_page.result)
                self.assertFalse(window.black_alignment.aligned)
                self.assertTrue(wait_until(lambda: SyntheticCapture.instances[0].released))
                self.assertEqual(serial_send.call_count, 0)
                grab.assert_not_called()
                visual_target.assert_not_called()
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()

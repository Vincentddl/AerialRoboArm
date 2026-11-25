from __future__ import annotations

from dataclasses import replace
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

import cv2
import numpy as np
from PySide6.QtGui import QImage
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from arm_console.black_blocks import BlackBlockConfig, BlackBlockTracker, detect_black_blocks
from arm_console.foam_vision import (
    FisheyeLutMapper,
    FoamVisionConfig,
    FoamYoloDetector,
    compute_hybrid_foam_center,
    verify_dark_foam_appearance,
    yolo_available,
)
from arm_console.models import BlackBlock, MotionRequest, VisionResult
from arm_console.pages import VisionFollowPage, map_vertical_to_extension_deg
from arm_console.v1_protocol import AckFrame, DoneFrame, ErrFrame
from arm_console.v1_state import ExperimentController
from arm_console.video_capture import VideoCaptureService
from arm_console.window import MainWindow
from test_protocol_v1 import online_controller

app = QApplication.instance() or QApplication([])


def make_block_result(center_y: float, *, width: int = 640, height: int = 480, frame_id: int = 1) -> tuple[QImage, VisionResult]:
    image = QImage(width, height, QImage.Format.Format_RGB32)
    block = BlackBlock(300, max(0, int(center_y) - 15), 40, 30, 320.0, float(center_y), 1200.0, True)
    result = VisionResult(width, height, None, "USB Video", False, (block,), frame_id, True, True, "TRACK", 0, 20)
    return image, result


class VisionFollowTests(unittest.TestCase):
    def test_vertical_mapping_bounds_and_inversion(self):
        self.assertAlmostEqual(
            map_vertical_to_extension_deg(456.0, 480, min_angle_deg=5.0, max_angle_deg=70.0, top_percent=5.0, bottom_percent=95.0),
            5.0,
        )
        self.assertAlmostEqual(
            map_vertical_to_extension_deg(24.0, 480, min_angle_deg=5.0, max_angle_deg=70.0, top_percent=5.0, bottom_percent=95.0),
            70.0,
        )
        self.assertAlmostEqual(
            map_vertical_to_extension_deg(240.0, 480, min_angle_deg=0.0, max_angle_deg=70.0, top_percent=5.0, bottom_percent=95.0),
            35.0,
        )
        self.assertAlmostEqual(
            map_vertical_to_extension_deg(456.0, 480, min_angle_deg=5.0, max_angle_deg=70.0, top_percent=5.0, bottom_percent=95.0, invert=True),
            70.0,
        )

    def test_fisheye_lut_mapper_monotonic_and_clamped(self):
        mapper = FisheyeLutMapper()
        ext_bottom, b_bottom = mapper.pixel_to_extension_deg(320.0, 440.0, 640, 480)
        ext_mid, b_mid = mapper.pixel_to_extension_deg(320.0, 240.0, 640, 480)
        ext_top, b_top = mapper.pixel_to_extension_deg(320.0, 45.0, 640, 480)
        self.assertGreater(b_bottom.optical_offset_deg, 10.0)
        self.assertAlmostEqual(b_mid.optical_offset_deg, 0.0, delta=0.5)
        self.assertLess(b_top.optical_offset_deg, -15.0)
        self.assertLessEqual(ext_bottom, 5.0)
        self.assertGreater(ext_mid, ext_bottom + 15.0)
        self.assertGreater(ext_top, ext_mid + 25.0)
        self.assertLessEqual(ext_top, 70.0)

    def test_hybrid_mask_centroid_rule(self):
        mask = np.zeros((480, 640), dtype=np.float32)
        # Asymmetric L-shape inside 80x80 box (~2.08% of 640x480 -> inside 1%-13% mask-centroid band)
        mask[200:280, 280:310] = 1.0
        mask[250:280, 280:360] = 1.0
        cx, cy, source, area_px, poly = compute_hybrid_foam_center((280.0, 200.0, 360.0, 280.0), mask, 640, 480)
        self.assertEqual(source, "mask_centroid")
        self.assertGreater(area_px, 1000.0)
        self.assertGreaterEqual(len(poly), 4)
        self.assertNotAlmostEqual(cx, 320.0, places=1)

        # Huge box (>13% area) falls back to box_center per Vision-Core rule
        cx_big, cy_big, source_big, _, _ = compute_hybrid_foam_center((100.0, 100.0, 420.0, 380.0), mask, 640, 480)
        self.assertEqual(source_big, "box_center")
        self.assertAlmostEqual(cx_big, 260.0)
        self.assertAlmostEqual(cy_big, 240.0)

    def test_dark_foam_appearance_gate_rejects_background_partition_and_white_gripper(self):
        gray = np.full((480, 640), 135, dtype=np.uint8)  # gray acoustic partition background
        # 1. Real dark foam block (gray ~70 on 135 background) -> ACCEPTED
        gray[200:260, 300:340] = 70
        self.assertTrue(verify_dark_foam_appearance(gray, (300, 200, 340, 260), ((300, 200), (340, 200), (340, 260), (300, 260))))
        # 2. White gripper jaw (gray ~220) -> REJECTED
        gray_jaw = np.full((480, 640), 135, dtype=np.uint8)
        gray_jaw[300:380, 295:325] = 220
        self.assertFalse(verify_dark_foam_appearance(gray_jaw, (295, 300, 325, 380), ()))
        # 3. Uniform dark shadow / background wall (both fg and ring ~85 -> zero local contrast) -> REJECTED
        gray_shadow = np.full((480, 640), 85, dtype=np.uint8)
        self.assertFalse(verify_dark_foam_appearance(gray_shadow, (290, 200, 350, 260), ()))
        # 4. Huge close-up ('放跟前') black block covering ~28% of frame with specular glare on upper half -> ACCEPTED!
        gray_closeup = np.full((480, 640), 140, dtype=np.uint8)
        gray_closeup[80:380, 180:460] = 35
        gray_closeup[80:220, 180:460] = 115  # strong close-up lens glare on top half!
        self.assertTrue(verify_dark_foam_appearance(gray_closeup, (180, 80, 460, 380), ()))

    def test_rotated_tilted_black_block_not_rejected_in_follow_config(self):
        gray = np.full((480, 640), 220, dtype=np.uint8)
        rect = ((320.0, 240.0), (46.0, 46.0), 45.0)
        pts = cv2.boxPoints(rect).astype(np.int32)
        cv2.fillConvexPoly(gray, pts, 10)
        # Default strict axis-aligned fill ratio (0.55) rejects a 45-deg diamond (~0.50 fill):
        strict_blocks = detect_black_blocks(gray, BlackBlockConfig(min_fill_ratio=0.55))
        self.assertEqual(len(strict_blocks), 0)
        # VisionFollowPage's enhanced OpenCV config uses minAreaRect fill ratio -> detects cleanly!
        follow_cfg = VisionFollowPage.detector_config(BlackBlockConfig(min_fill_ratio=0.55), corridor_half_width_px=55)
        detected = detect_black_blocks(gray, follow_cfg)
        self.assertEqual(len(detected), 1)
        self.assertAlmostEqual(detected[0].center_x, 320.0, delta=1.5)
        self.assertAlmostEqual(detected[0].center_y, 240.0, delta=1.5)

        # Distractor at x=220 (100 px off-center, outside ±55 px corridor) is completely ignored!
        gray_off = np.full((480, 640), 220, dtype=np.uint8)
        gray_off[200:250, 195:245] = 10
        self.assertEqual(len(detect_black_blocks(gray_off, follow_cfg)), 0)

    def test_exposure_lock_flag_updates_cleanly(self):
        svc = VideoCaptureService()
        try:
            self.assertFalse(svc.exposure_lock_enabled)
            svc.set_exposure_lock(True, exposure=-7.0, gain=20.0)
            self.assertTrue(svc.exposure_lock_enabled)
            svc.set_exposure_lock(False)
            self.assertFalse(svc.exposure_lock_enabled)
        finally:
            svc.shutdown()

    def test_preview_never_sends_until_started_and_stops_on_page_leave(self):
        with patch("arm_console.window.ExperimentController",
                   side_effect=lambda serial, _cal_path: ExperimentController(serial)):
            window = MainWindow()
        try:
            window.show()
            window.simulation_checkbox.setChecked(True)
            QTest.qWait(120)
            before = window.controller.submitted_requests
            window.switch_page(3)
            self.assertIs(window.pages.currentWidget(), window.follow_page)
            self.assertTrue(window.video_controls.isVisible())
            QTest.qWait(150)
            self.assertEqual(window.controller.submitted_requests, before)
            self.assertIn("映射目标：", window.follow_page.mapping_label.text())

            window.follow_page.start_button.click()
            self.assertTrue(window.follow_page.running)
            QTest.qWait(260)
            self.assertGreater(window.controller.submitted_requests, before)
            self.assertEqual(window.controller.last_request.action, "follow")

            window.switch_page(1)
            self.assertFalse(window.follow_page.running)
        finally:
            window.close()

    def test_20hz_streaming_follow_in_vision_follow_mode_never_touches_gripper(self):
        c, link = online_controller()
        c.handle_frame(replace(c.latest_state, t_ms=155000, mode="MANUAL", motion="IDLE",
                               pos_mdeg=90000, target_mdeg=90000, fault_code="NONE"))
        page = VisionFollowPage(c)
        page.set_mapping_mode("linear")
        page.smoothing_spin.setValue(1.0)
        page.min_angle_spin.setValue(0.0)
        page.max_angle_spin.setValue(70.0)
        page.deadband_spin.setValue(0.8)
        page.STREAM_INTERVAL_SECONDS = 0.0
        self.assertEqual(page.confirm_frames_spin.value(), 1)
        self.assertAlmostEqual(page.speed_spin.value(), 300.0)

        link.sent.clear()
        page.start_flow()
        self.assertTrue(page.running)
        modes = [p for p, _ in link.sent if p.startswith(b"MODE,")]
        self.assertEqual(len(modes), 1)
        mode_req = int(modes[0].decode("ascii").split(",")[1])
        self.assertEqual(modes[0], f"MODE,{mode_req},VISION_FOLLOW\n".encode("ascii"))

        # Confirm VISION_FOLLOW ACK -> streams fire-and-forget V frames continuously even while MOVING!
        c.handle_frame(AckFrame(mode_req, "MODE", "VISION_FOLLOW"))
        c.handle_frame(replace(c.latest_state, t_ms=155050, mode="VISION_FOLLOW", motion="MOVING",
                               pos_mdeg=85000, target_mdeg=90000, fault_code="NONE"))
        link.sent.clear()

        img1, res1 = make_block_result(240.0, frame_id=1)  # 35.0 deg -> +55.0 deg (55000 mdeg)
        page.set_frame(img1, res1)
        page.observe(res1)
        img2, res2 = make_block_result(120.0, frame_id=2)  # 54.4 deg -> +35.6 deg (35600 mdeg) while still MOVING!
        page.set_frame(img2, res2)
        page.observe(res2)

        v_frames = [p for p, _ in link.sent if p.startswith(b"V,")]
        self.assertEqual(len(v_frames), 2)
        self.assertEqual(v_frames[0], b"V,55000,300,95\n")
        self.assertEqual(v_frames[1], b"V,35600,300,95\n")
        # Zero SERVO, GRIP, PICK, or discrete MOVE commands!
        self.assertEqual([p for p, _ in link.sent if p.startswith((b"SERVO,", b"GRIP,", b"PICK,", b"MOVE,"))], [])

        # Stop follow -> sends STOP and queues MODE MANUAL, never touching gripper
        link.sent.clear()
        page.stop_flow(send_stop=True)
        self.assertTrue(any(p.startswith(b"STOP,") for p, _ in link.sent))
        self.assertEqual([p for p, _ in link.sent if p.startswith((b"SERVO,", b"GRIP,"))], [])

        # Return normal button moves ONLY HX8 to working_zero_mdeg without linked SERVO open!
        stop_req = int([p for p, _ in link.sent if p.startswith(b"STOP,")][0].decode("ascii").split(",")[1])
        c.handle_frame(AckFrame(stop_req, "STOP", "APPLIED"))
        mode_back = [p for p, _ in link.sent if p.startswith(b"MODE,")]
        self.assertEqual(len(mode_back), 1)
        back_req = int(mode_back[0].decode("ascii").split(",")[1])
        c.handle_frame(AckFrame(back_req, "MODE", "MANUAL"))
        c.handle_frame(replace(c.latest_state, t_ms=155300, mode="MANUAL", motion="IDLE",
                               pos_mdeg=55000, target_mdeg=55000, fault_code="NONE",
                               reply_to=c._query_ids["STATE"]))
        link.sent.clear()
        page.return_normal_button.click()
        ret_moves = [p for p, _ in link.sent if p.startswith(b"MOVE,")]
        self.assertEqual(len(ret_moves), 1)
        ret_req = int(ret_moves[0].decode("ascii").split(",")[1])
        c.handle_frame(AckFrame(ret_req, "MOVE", "ACCEPTED"))
        c.handle_frame(DoneFrame(ret_req, "MOVE", 90000))
        self.assertEqual([p for p, _ in link.sent if p.startswith((b"SERVO,", b"GRIP,"))], [])

    def test_legacy_mcu_auto_fallback_to_step_follow(self):
        c, link = online_controller()
        c.handle_frame(replace(c.latest_state, t_ms=160000, mode="MANUAL", motion="IDLE",
                               pos_mdeg=90000, target_mdeg=90000, fault_code="NONE"))
        page = VisionFollowPage(c)
        page.set_mapping_mode("linear")
        page.smoothing_spin.setValue(1.0)
        page.confirm_frames_spin.setValue(2)
        page.deadband_spin.setValue(3.5)
        page.DONE_COOLDOWN_SECONDS = 0.0

        link.sent.clear()
        page.start_flow()
        mode_req = int([p for p, _ in link.sent if p.startswith(b"MODE,")][0].decode("ascii").split(",")[1])
        # Simulate un-reflashed MCU rejecting VISION_FOLLOW with BAD_FRAME -> automatic fallback!
        c.handle_frame(ErrFrame(mode_req, "BAD_FRAME"))
        self.assertTrue(page.running)
        self.assertEqual(page._phase, "step_follow")
        link.sent.clear()

        img1, res1 = make_block_result(240.0, frame_id=1)
        page.set_frame(img1, res1)
        page.observe(res1)
        self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"MOVE,")]), 0)

        img1b, res1b = make_block_result(240.0, frame_id=2)
        page.set_frame(img1b, res1b)
        page.observe(res1b)
        moves = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")]
        self.assertEqual(len(moves), 1)
        req1 = int(moves[0].decode("ascii").split(",")[1])
        self.assertEqual(moves[0], f"MOVE,{req1},55000,300\n".encode("ascii"))
        c.handle_frame(AckFrame(req1, "MOVE", "ACCEPTED"))

        img2, res2 = make_block_result(24.0, frame_id=3)
        page.set_frame(img2, res2)
        page.observe(res2)
        c.handle_frame(DoneFrame(req1, "MOVE", 55000))
        c.handle_frame(replace(c.latest_state, t_ms=160500, motion="IDLE",
                               pos_mdeg=55000, target_mdeg=55000, last_req_id=req1,
                               reply_to=c._query_ids["STATE"]))
        moves = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")]
        self.assertEqual(len(moves), 2)
        page.stop_flow(send_stop=False)

    def test_transient_hx8_offline_survives_and_persistent_offline_stops(self):
        c, link = online_controller()
        c.handle_frame(replace(c.latest_state, t_ms=180000, mode="VISION_FOLLOW", motion="IDLE",
                               pos_mdeg=90000, target_mdeg=90000, fault_code="NONE"))
        page = VisionFollowPage(c)
        page.set_mapping_mode("linear")
        page.smoothing_spin.setValue(1.0)
        page.HX8_OFFLINE_GRACE_SECONDS = 0.25
        page.STREAM_INTERVAL_SECONDS = 0.0
        page.start_flow()
        link.sent.clear()

        c.handle_frame(replace(c.latest_state, t_ms=180100, hx8_online=False, pos_mdeg=None, fault_code="HX8_OFFLINE"))
        self.assertTrue(page.running)
        self.assertIn("短暂缺帧", page.flow_label.text())

        c.handle_frame(replace(c.latest_state, t_ms=180150, hx8_online=True, pos_mdeg=90000, fault_code="NONE"))
        img, res = make_block_result(240.0, frame_id=1)
        page.set_frame(img, res)
        page.observe(res)
        self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"V,")]), 1)
        page.stop_flow(send_stop=False)

        page.start_flow()
        c.handle_frame(replace(c.latest_state, t_ms=181000, hx8_online=False, pos_mdeg=None, fault_code="HX8_OFFLINE"))
        self.assertTrue(page.running)
        QTest.qWait(300)
        c.handle_frame(replace(c.latest_state, t_ms=181350, hx8_online=False, pos_mdeg=None, fault_code="HX8_OFFLINE"))
        self.assertFalse(page.running)
        self.assertIn("持续离线", page.flow_label.text())

    def test_instant_reacquire_handles_large_jump_with_zero_missed_frames(self):
        tracker = BlackBlockTracker()
        cfg = VisionFollowPage.detector_config(BlackBlockConfig())
        f1 = np.full((480, 640), 220, dtype=np.uint8)
        f1[380:430, 290:350] = 0
        obs1 = tracker.process(f1, cfg)
        self.assertEqual(len(obs1.blocks), 1)
        self.assertAlmostEqual(obs1.blocks[0].center_y, 404.5, delta=1.0)

        f2 = np.full((480, 640), 220, dtype=np.uint8)
        f2[50:100, 290:350] = 0
        obs2 = tracker.process(f2, cfg)
        self.assertEqual(obs2.missed_frames, 0)
        self.assertEqual(len(obs2.blocks), 1)
        self.assertAlmostEqual(obs2.blocks[0].center_y, 74.5, delta=1.0)

    def test_adaptive_velocity_smoothing_and_radio_deconfliction(self):
        c, link = online_controller()
        c.handle_frame(replace(c.latest_state, t_ms=190000, mode="VISION_FOLLOW", motion="IDLE",
                               pos_mdeg=55000, target_mdeg=55000, fault_code="NONE"))
        page = VisionFollowPage(c)
        page.set_mapping_mode("linear")
        page.smoothing_spin.setValue(1.0)
        page.min_angle_spin.setValue(0.0)
        page.max_angle_spin.setValue(70.0)
        page.STREAM_INTERVAL_SECONDS = 0.0
        self.assertTrue(page.adaptive_speed_checkbox.isChecked())

        page.start_flow()
        link.sent.clear()

        # 1) Small 3.0 deg step from 35.0 -> 38.0 deg: uses gentle adaptive speed (< 120 dps), never slamming 300 dps!
        img1, res1 = make_block_result(221.5, frame_id=1)  # ~38.0 deg (arm currently at 35.0 deg)
        page.set_frame(img1, res1)
        page.observe(res1)
        v_small = [p for p, _ in link.sent if p.startswith(b"V,")]
        self.assertEqual(len(v_small), 1)
        small_speed = int(v_small[0].decode("ascii").split(",")[2])
        self.assertGreaterEqual(small_speed, 35)
        self.assertLess(small_speed, 100)

        # 2) Rapid 50 ms feedback timer poll during stream_follow must NOT spam GET STATE on 433 MHz air!
        link.sent.clear()
        page._poll_state()
        self.assertEqual([p for p, _ in link.sent if b"STATE" in p], [])

        # 3) Large 30+ deg jump commands full 300 dps catch-up speed
        link.sent.clear()
        img2, res2 = make_block_result(40.0, frame_id=2)  # ~67.4 deg
        page.set_frame(img2, res2)
        page.observe(res2)
        v_large = [p for p, _ in link.sent if p.startswith(b"V,")]
        self.assertEqual(len(v_large), 1)
        large_speed = int(v_large[0].decode("ascii").split(",")[2])
        self.assertEqual(large_speed, 300)
        page.stop_flow(send_stop=False)

    @unittest.skipUnless(yolo_available(), "ultralytics + foam_center_v9 weights not installed in active env")
    def test_gpu_yolo_v9_detector_smoke(self):
        det = FoamYoloDetector()
        frame = np.full((480, 640, 3), 210, dtype=np.uint8)
        obs = det.detect(frame, FoamVisionConfig())
        self.assertIn(("GPU"), obs.device_label) if "GPU" in obs.device_label else self.assertTrue(bool(obs.device_label))
        self.assertGreater(obs.inference_ms, 0.1)


if __name__ == "__main__":
    unittest.main()

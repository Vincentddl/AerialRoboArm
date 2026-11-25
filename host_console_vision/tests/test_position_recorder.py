from __future__ import annotations

from dataclasses import replace
import os
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtWidgets import QApplication

from arm_console.black_blocks import BlackBlockConfig
from arm_console.pages import VisionFollowPage
from arm_console.position_recorder import (
    CalibratedPositionLut,
    DEFAULT_SWEEP_ANGLES,
    PositionSweepRecorderDialog,
    PositionSweepStateMachine,
    build_position_lut_from_holds,
)
from arm_console.video_capture import VideoCaptureService
from test_protocol_v1 import online_controller

app = QApplication.instance() or QApplication([])


class PositionRecorderTests(unittest.TestCase):
    def test_17_point_outbound_return_state_machine_and_lut_hysteresis(self):
        self.assertEqual(len(DEFAULT_SWEEP_ANGLES), 17)
        self.assertEqual(DEFAULT_SWEEP_ANGLES[0], 30.0)
        self.assertEqual(DEFAULT_SWEEP_ANGLES[8], 70.0)
        self.assertEqual(DEFAULT_SWEEP_ANGLES[-1], 30.0)

        sm = PositionSweepStateMachine(DEFAULT_SWEEP_ANGLES, hold_seconds=3.0)
        self.assertEqual(sm.phase, "IDLE")
        self.assertTrue(sm.start_recording())
        self.assertEqual(sm.phase, "WAIT_STABLE")

        t = 100.0
        frame_idx = 0
        for step in range(17):
            angle = DEFAULT_SWEEP_ANGLES[step]
            self.assertEqual(sm.current_angle_deg, angle)
            self.assertEqual(sm.on_space(t, frame_idx), "HOLD_STARTED")
            self.assertEqual(sm.phase, "HOLDING")

            # Simulate 3.0s hold (return sweep has slight 2.0 px mechanical backlash)
            backlash_px = 0.0 if step <= 8 else 2.0
            y_px = 420.0 - (angle - 30.0) * 6.0 + backlash_px
            pos_mdeg = int(round(90000 - angle * 1000))
            for tick in range(1, 7):
                t += 0.5
                frame_idx += 15
                elapsed, rec = sm.feed_sample(t, frame_idx, 480, y_px, pos_mdeg, angle)
                if tick < 6:
                    self.assertIsNone(rec)
                else:
                    self.assertIsNotNone(rec)
                    self.assertEqual(elapsed, 3.0)
            self.assertEqual(sm.phase, "CAPTURED")

            # Test back/re-record once on step 2 (40 deg)
            if step == 2:
                self.assertTrue(sm.on_back())
                self.assertEqual(sm.phase, "WAIT_STABLE")
                self.assertEqual(sm.on_space(t, frame_idx), "HOLD_STARTED")
                t += 3.0
                frame_idx += 90
                _, rec2 = sm.feed_sample(t, frame_idx, 480, y_px, pos_mdeg, angle)
                self.assertIsNotNone(rec2)
                self.assertEqual(sm.phase, "CAPTURED")

            expected_action = "NEXT_ANGLE" if step < 16 else "SEQUENCE_FINISHED"
            self.assertEqual(sm.on_space(t, frame_idx), expected_action)

        self.assertEqual(sm.phase, "FINISHED")
        holds = sm.ordered_holds()
        self.assertEqual(len(holds), 17)

        lut_dict = build_position_lut_from_holds(
            holds,
            frame_height=480,
            working_zero_mdeg=90000,
            extension_sign=-1,
        )
        self.assertTrue(lut_dict["monotonic"])
        self.assertEqual(len(lut_dict["points"]), 9)
        self.assertAlmostEqual(lut_dict["hysteresis_summary"]["max_y_px"], 2.0, delta=0.01)

        lut = CalibratedPositionLut(lut_dict)
        # 50 deg -> y = 300 (outbound) / 302 (return) -> median 301 px
        self.assertAlmostEqual(lut.pixel_y_to_extension_deg(301.0, 480), 50.0, delta=0.2)
        self.assertAlmostEqual(lut.pixel_y_to_extension_deg(421.0, 480), 30.0, delta=0.2)
        self.assertAlmostEqual(lut.pixel_y_to_extension_deg(180.0, 480), 70.0, delta=0.2)

    def test_dialog_is_non_modal_and_g_moves_only_hx8_without_gripper(self):
        c, link = online_controller()
        c.handle_frame(replace(c.latest_state, t_ms=160000, mode="MANUAL", motion="IDLE",
                               pos_mdeg=90000, target_mdeg=90000, fault_code="NONE"))
        video = VideoCaptureService()
        dialog = PositionSweepRecorderDialog(
            c,
            video,
            lambda: VisionFollowPage.detector_config(BlackBlockConfig(), corridor_half_width_px=55),
            ROOT / "runtime",
        )
        try:
            self.assertFalse(dialog.isModal())
            link.sent.clear()
            dialog._move_to_current_guide()  # 30 deg -> absolute +60.0 deg (60000 mdeg) at 60 deg/s
            moves = [p for p, _ in link.sent if p.startswith(b"MOVE,")]
            self.assertEqual(len(moves), 1)
            self.assertTrue(moves[0].endswith(b",60000,60\n"))
            self.assertEqual([p for p, _ in link.sent if p.startswith((b"SERVO,", b"GRIP,"))], [])
        finally:
            dialog.close()
            video.shutdown()


if __name__ == "__main__":
    unittest.main()

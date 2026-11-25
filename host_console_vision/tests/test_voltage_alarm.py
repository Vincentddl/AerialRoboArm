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

from PySide6.QtWidgets import QApplication

from arm_console.models import ConnectionState
from arm_console.v1_protocol import AckFrame, StateFrame
from arm_console.window import MainWindow, VoltageAlarmGate
from test_protocol_v1 import online_controller

app = QApplication.instance() or QApplication([])


class VoltageAlarmTests(unittest.TestCase):
    def test_transient_sag_is_ignored_and_sustained_low_voltage_alerts_once(self):
        gate = VoltageAlarmGate()
        self.assertFalse(gate.observe(10_300, valid=True, received_at=0.0))
        self.assertFalse(gate.observe(10_600, valid=True, received_at=0.5))
        self.assertFalse(gate.observe(10_450, valid=True, received_at=1.0))
        self.assertFalse(gate.observe(10_400, valid=True, received_at=2.0))
        self.assertTrue(gate.observe(10_350, valid=True, received_at=3.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=4.0))
        self.assertFalse(gate.observe(10_700, valid=True, received_at=5.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=6.0))
        self.assertFalse(gate.observe(10_800, valid=True, received_at=7.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=8.0))
        self.assertFalse(gate.observe(10_250, valid=True, received_at=9.0))
        self.assertTrue(gate.observe(10_200, valid=True, received_at=10.0))

    def test_invalid_feedback_and_gaps_break_low_voltage_streak(self):
        gate = VoltageAlarmGate()
        self.assertFalse(gate.observe(10_500, valid=True, received_at=0.0))
        self.assertFalse(gate.observe(None, valid=False, received_at=1.0))
        self.assertFalse(gate.observe(10_500, valid=True, received_at=2.0))
        self.assertFalse(gate.observe(10_400, valid=True, received_at=5.0))
        self.assertFalse(gate.observe(10_300, valid=True, received_at=6.0))
        self.assertTrue(gate.observe(10_200, valid=True, received_at=7.0))

    def test_real_fresh_state_shows_nonmodal_warning_without_repeating(self):
        window = MainWindow()
        try:
            window.serial.state = ConnectionState.OPEN
            window.controller.link_online = True
            with patch("arm_console.window.QMessageBox") as box:
                base = time.monotonic()
                for t_ms, voltage in ((1, 10_500), (2, 10_450), (3, 10_400), (4, 10_350)):
                    now = base + (t_ms - 1) * 0.5
                    window.controller._last_state_received_at = now
                    window.controller.latest_state = self._state(t_ms, voltage)
                    with patch("arm_console.window.time.monotonic", return_value=now):
                        window._check_low_voltage()
                box.assert_not_called()  # four rapid frames are still a short sag

                # The warning is owned by MainWindow and must keep its
                # accumulation window when the user opens experiment mode.
                window.switch_page(3)
                self.assertEqual(window.pages.currentIndex(), 3)
                window.controller._last_state_received_at = base + 2.0
                window.controller.latest_state = self._state(5, 10_300)
                with patch("arm_console.window.time.monotonic", return_value=window.controller._last_state_received_at):
                    window._check_low_voltage()
                self.assertEqual(box.call_count, 1)
                self.assertFalse(box.return_value.setModal.call_args.args[0])
                self.assertIn("10.60 V", box.return_value.setText.call_args.args[0])
                self.assertTrue(window.controller.voltage_locked)
                self.assertFalse(window.pages.isEnabled())
                self.assertFalse(window.simulation_checkbox.isEnabled())
                self.assertFalse(window.connect_button.isEnabled())
                window.switch_page(0)
                self.assertEqual(window.pages.currentIndex(), 3)
                window._check_low_voltage()
                self.assertEqual(box.call_count, 1)
                window.serial._set_state(ConnectionState.DISCONNECTED)
                self.assertTrue(window.controller.voltage_locked)
                self.assertTrue(window.connect_button.isEnabled())
                window.serial._set_state(ConnectionState.OPEN)
                window.controller.link_online = True
                for t_ms, voltage, locked in ((6, None, True), (7, 10_700, True), (8, 10_800, False)):
                    window.controller.latest_state = self._state(t_ms, voltage)
                    window.controller._last_state_received_at = time.monotonic()
                    window._check_low_voltage()
                    self.assertEqual(window.controller.voltage_locked, locked)
                self.assertTrue(window.pages.isEnabled())
                self.assertTrue(window.simulation_checkbox.isEnabled())
                self.assertIsNone(window._voltage_alarm_dialog)
                self.assertEqual(box.call_count, 1)
        finally:
            window.controller.latest_state = None
            window.serial.state = ConnectionState.DISCONNECTED
            window.close()

    @staticmethod
    def _state(t_ms, voltage):
        return StateFrame(t_ms, "MANUAL", "UNKNOWN", "IDLE", True, True, 160000, 160000,
                          voltage, 200, True, True, None, "UNKNOWN", 0, "NONE", 1)

    def test_controller_discards_pending_motion_and_allows_state_query_during_lock(self):
        controller, link = online_controller()
        self.addCleanup(controller.shutdown)
        self.assertTrue(controller.request_restore())
        move = controller._next_req - 1
        controller.set_voltage_lock(True)
        self.assertEqual(link.queue_discards, 1)
        stop = controller._next_req - 1
        self.assertNotIn(move, controller._pending)
        self.assertEqual(link.sent[-1], (f"STOP,{stop}\n".encode(), True))
        controller.handle_frame(AckFrame(stop, "STOP", "APPLIED"))
        self.assertFalse(controller.request_restore())
        self.assertFalse(controller.request_mode("VISION_PICK"))
        self.assertFalse(controller.request_grip_install_position())
        self.assertFalse(controller.submit_visual_target(20, 125, 99))
        self.assertTrue(controller.query("STATE"))
        count = len(link.sent)
        controller.set_voltage_lock(False)
        self.assertEqual(len(link.sent), count)
        self.assertTrue(controller.request_restore())

if __name__ == "__main__":
    unittest.main()

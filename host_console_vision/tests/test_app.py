from __future__ import annotations

import os
import queue
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from arm_console.models import ConnectionState, MotionRequest
from arm_console.v1_state import ExperimentController
from arm_console.window import MainWindow


app = QApplication.instance() or QApplication([])


class DesktopAcceptanceTests(unittest.TestCase):
    def setUp(self):
        # Acceptance fixtures represent a fresh installation with default poses.
        with patch("arm_console.window.ExperimentController",
                   side_effect=lambda serial, _calibration_path: ExperimentController(serial)):
            self.window = MainWindow()
        self.window.show()
        QTest.qWait(80)

    def tearDown(self):
        self.window.close()
        QTest.qWait(80)

    def test_no_hardware_opens_and_real_commands_never_send_unknown_protocol(self):
        w = self.window
        self.assertEqual(w.serial.state, ConnectionState.DISCONNECTED)
        self.assertIn("等待", w.plot_panel.status.text())
        self.assertFalse(w.controller.request_extend(51.3, -25.7))
        self.assertIn("握手", w.controller.command_status)
        self.assertEqual(w.controller.actual_state, "未知／等待反馈")
        self.assertIsNone(w.serial.worker)

    def test_calibration_saves_actual_normal_position_not_trial_input(self):
        w = self.window
        page = w.calibration_page
        page.normal.setValue(107.3)
        page.offset.setValue(69.0)
        with tempfile.TemporaryDirectory() as directory:
            w.controller.calibration_path = Path(directory) / "pose_calibration.json"
            with patch.object(w.controller, "calibration_position", return_value=174_600), \
                 patch.object(w.controller, "_guard", return_value=True):
                page._refresh()
                self.assertIn("+243.6°", page.save_preview.text())
                page._save()
                self.assertIn("+180.0°", page.status.text())
                self.assertFalse(w.controller.calibration_path.exists())
            with patch.object(w.controller, "calibration_position", return_value=107_300), \
                 patch.object(w.controller, "_guard", return_value=True):
                page._refresh()
                self.assertEqual(page.status.text(), "")
                self.assertIn("+176.3°", page.save_preview.text())
                page._save()
                self.assertTrue(w.controller.calibration_path.exists())
                self.assertEqual(w.controller.working_zero_mdeg, 107_300)
                self.assertEqual(w.controller.fold_target_mdeg, 176_300)

    def test_video_loss_does_not_cancel_already_triggered_grab(self):
        w = self.window
        w.grab_page.running = True
        w.grab_page._phase = "close"
        with patch.object(w.controller, "request_stop") as stop:
            w._clear_real_video()
            self.assertTrue(w.grab_page.running)
            self.assertEqual(w.grab_page._phase, "close")
            stop.assert_not_called()
        w.grab_page.running = False

    def test_simulation_curves_pause_resume_clear_and_page_persistence(self):
        w = self.window
        w.simulation_checkbox.setChecked(True)
        QTest.qWait(260)
        self.assertGreater(len(w.plot_panel.samples), 2)
        self.assertIn("模拟", w.plot_panel.status.text())
        self.assertIsNotNone(w.demo_page.result)
        self.assertEqual(w.demo_page.result.detection.u, w.grab_page.result.detection.u)
        w.plot_panel.toggle_pause()
        count = len(w.plot_panel.samples)
        QTest.qWait(180)
        self.assertGreater(len(w.plot_panel.samples), count)
        self.assertTrue(w.plot_panel.paused)
        w.switch_page(1)
        w.switch_page(2)
        w.switch_page(0)
        self.assertEqual(w.pages.currentIndex(), 0)
        w.plot_panel.toggle_pause()
        self.assertFalse(w.plot_panel.paused)
        w.plot_panel.clear()
        self.assertEqual(len(w.plot_panel.samples), 0)
        self.assertIn("等待", w.plot_panel.status.text())
        QTest.qWait(120)
        self.assertGreater(len(w.plot_panel.samples), 0)
        w.simulation_checkbox.setChecked(False)
        self.assertFalse(w.simulation.timer.isActive())
        self.assertFalse(w.simulation.vision.timer.isActive())
        self.assertEqual(len(w.plot_panel.samples), 0)
        self.assertIsNone(w.demo_page.result)

    def test_manual_snapshot_home_and_shape_are_distinct(self):
        w = self.window
        self.assertEqual(w.manual_page.speed_spin.maximum(), 78.0)
        self.assertEqual(w.manual_page.speed_slider.maximum(), 780)
        self.assertEqual(w.manual_page.angle_spin.maximum(), 70.0)
        self.assertEqual(w.grab_page.speed_spin.maximum(), 100.0)
        self.assertTrue(w.plot_panel.status.isHidden())
        w.simulation_checkbox.setChecked(True)
        w.manual_page.speed_slider.setValue(437)
        w.manual_page.angle_spin.setValue(34.2)
        self.assertAlmostEqual(w.manual_page.speed_spin.value(), 43.7)
        self.assertEqual(w.manual_page.angle_slider.value(), 342)
        before = w.controller.submitted_requests
        w.manual_page.start_button.click()
        self.assertEqual(w.controller.submitted_requests, before + 1)
        self.assertEqual(w.controller.last_request, MotionRequest("extend", 43.7, 34.2))
        w.manual_page.zero_button.click()
        self.assertEqual(w.controller.last_request, MotionRequest("home"))
        w.fold_button.click()
        self.assertEqual(w.controller.last_request, MotionRequest("fold"))
        w.normal_button.click()
        self.assertEqual(w.controller.last_request, MotionRequest("restore"))
        w.switch_page(1)
        w.switch_page(0)
        self.assertAlmostEqual(w.manual_page.speed_spin.value(), 43.7)

    def test_vision_demo_never_moves_and_grab_needs_start_stops_on_leave(self):
        w = self.window
        w.simulation_checkbox.setChecked(True)
        QTest.qWait(120)
        before = w.controller.submitted_requests
        w.switch_page(1)
        QTest.qWait(150)
        self.assertEqual(w.controller.submitted_requests, before)
        w.switch_page(2)
        self.assertFalse(w.grab_page.running)
        w.grab_page.start_button.click()
        self.assertTrue(w.grab_page.running)
        for _ in range(3):
            w.grab_page._advance_simulation()
        self.assertEqual(w.controller.submitted_requests, before + 1)
        self.assertEqual(w.controller.last_request, MotionRequest("grab"))
        self.assertIn("未写入串口", w.grab_page.command_label.text())
        w.switch_page(1)
        self.assertFalse(w.grab_page.running)
        self.assertFalse(w.grab_page.timer.isActive())
        after = w.controller.submitted_requests
        QTest.qWait(100)
        self.assertEqual(w.controller.submitted_requests, after)
        w.simulation_checkbox.setChecked(False)
        self.assertIn("未知", w.grab_page.actual_label.text())
        self.assertIn("暂不可确认", w.grab_page.result_label.text())
        w.switch_page(2)
        w.grab_page.start_button.click()
        self.assertFalse(w.grab_page.running)
        self.assertIn("握手", w.grab_page.flow_label.text())
        self.assertEqual(w.controller.submitted_requests, after)

    def test_non_v2_error_is_explained_without_sending_motion(self):
        w = self.window
        w.serial.state = ConnectionState.OPEN
        w._on_raw(b"ERROR\r\n", time.monotonic())
        self.assertFalse(w.controller.link_online)
        self.assertIn("非MCU V2 ERR", w.controller.command_status)
        self.assertFalse(w.controller.request_restore())
        self.assertIn("ERROR", w.controller.command_status)
        self.assertIn("45 52 52 4F 52 0D 0A", w.log_view.toPlainText())
        self.assertIn("串口RX", w.log_view.toPlainText())

    def test_missing_port_fails_without_freezing(self):
        w = self.window
        w.serial.connect_port("COM999", 115200)
        deadline = time.monotonic() + 4
        while w.serial.worker is not None and time.monotonic() < deadline:
            QTest.qWait(30)
        self.assertIsNone(w.serial.worker)
        self.assertEqual(w.serial.state, ConnectionState.FAILED)
        self.assertIn("串口", w.log_view.toPlainText())

    def test_pick_mode_discards_imu_but_still_processes_state(self):
        w = self.window
        w.controller.mode = "VISION_PICK"
        now = time.monotonic()
        w._on_raw(b"ACC,1,100,100,200,900\nGYRO,1,100,1,2,3\n"
                  b"STATE,100,VISION_PICK,UNKNOWN,IDLE,1,1,0,0,12000,200,1,0,NA,UNKNOWN,0,NONE,0\n", now)
        self.assertEqual(len(w.plot_panel.samples), 0)
        self.assertEqual(w.discarded_imu_frames, 2)
        self.assertIsNotNone(w.controller.latest_state)
        self.assertEqual(w.controller.latest_state.mode, "VISION_PICK")
        self.assertFalse(w.controller.latest_state.acc_tx)
        w._update_reception()
        self.assertIn("IMU显示暂停", w.plot_panel.status.text())
        w._on_raw(b"STATE,200,MANUAL,UNKNOWN,IDLE,1,1,0,0,12000,200,1,1,NA,UNKNOWN,0,NONE,0\n"
                  b"ACC,2,200,101,201,901\n", now + 0.1)
        self.assertEqual(len(w.plot_panel.samples), 1)
        self.assertAlmostEqual(w.plot_panel.samples[0].ax, 0.101)

    def test_serial_worker_reuses_one_port_for_receive_and_send(self):
        class FakePort:
            instances = []

            def __init__(self, *args, **kwargs):
                self.writes = []
                self.closed = False
                self.port = kwargs.get("port")
                self.dtr = True
                self.rts = True
                self.lines_at_open = None
                self.read_sizes = []
                self.incoming = queue.Queue()
                self.incoming.put(b"ACC,128,152340,85,-20,998\n")
                self.instances.append(self)

            def open(self):
                self.lines_at_open = (self.dtr, self.rts)

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.closed = True

            def read(self, length):
                self.read_sizes.append(length)
                try:
                    return self.incoming.get(timeout=0.02)
                except queue.Empty:
                    return b""

            @property
            def in_waiting(self):
                return 0

            def write(self, data):
                self.writes.append(data)
                parts = data.decode("ascii").strip().split(",")
                if parts[0] == "GET":
                    req = parts[1]
                    if parts[2] == "INFO":
                        response = f"INFO,{req},2,73582,113,230400,200,20,500,100,1000,80,300\n"
                    elif parts[2] == "CAL":
                        response = f"CAL,{req},0,0,-1,90000,NA,135000,90000,0,180000,0,0,4000\n"
                    else:
                        response = f"STATE,152340,MANUAL,UNKNOWN,IDLE,1,1,-29700,-30000,11980,200,1,1,120000,ANGLE_CMD,113,NONE,{req}\n"
                    self.incoming.put(response.encode("ascii"))
                elif parts[0] == "PING":
                    self.incoming.put(f"PONG,{parts[1]},152350\n".encode("ascii"))
                elif parts[0] == "MOVE":
                    self.incoming.put(f"ACK,{parts[1]},MOVE,ACCEPTED\n".encode("ascii"))
                return len(data)

        w = self.window
        with patch("arm_console.serial_service.serial.Serial", FakePort):
            w.serial.connect_port("FAKE1", 115200)
            deadline = time.monotonic() + 3
            while w.serial.state != ConnectionState.OPEN and time.monotonic() < deadline:
                QTest.qWait(20)
            self.assertEqual(w.serial.state, ConnectionState.OPEN)
            self.assertEqual(FakePort.instances[0].lines_at_open, (False, False))
            while (w.controller.cal is None or not w.controller.link_online) and time.monotonic() < deadline:
                QTest.qWait(20)
            self.assertTrue(w.controller.link_online)
            self.assertIsNotNone(w.controller.cal)
            self.assertEqual(len(w.plot_panel.samples), 1)
            self.assertAlmostEqual(w.plot_panel.samples[0].ax, 0.085)
            from arm_console.v1_protocol import MotionFrame
            from arm_console.v1_state import Pending
            w.controller._pending[113] = Pending(113, "HOME", b"HOME,113\n", "准备", time.monotonic())
            w.controller.handle_frame(MotionFrame(113, "READY", 152340, 0, 0, "NONE"))
            from arm_console.v1_protocol import DoneFrame
            w.controller.handle_frame(DoneFrame(113, "HOME", 0))
            from dataclasses import replace
            w.controller.latest_state = replace(w.controller.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(w.controller.request_extend(60, 45))
            while not any(b.startswith(b"MOVE,") for b in FakePort.instances[0].writes) and time.monotonic() < deadline:
                QTest.qWait(20)
            moves = [b for b in FakePort.instances[0].writes if b.startswith(b"MOVE,")]
            self.assertEqual(len(moves), 1)
            self.assertTrue(moves[0].endswith(b",45000,300\n"))
            QTest.qWait(80)  # 工作线程的实际写入信号排入主线程后再检查日志。
            self.assertIn("串口TX", w.log_view.toPlainText())
            self.assertIn("MOVE req=", w.log_view.toPlainText())
            self.assertIn("HEX=4D 4F 56 45", w.log_view.toPlainText())
            self.assertEqual(len(FakePort.instances), 1)
            self.assertEqual(FakePort.instances[0].lines_at_open, (False, False))
            self.assertEqual(FakePort.instances[0].port, "FAKE1")
            self.assertNotIn("DONE", w.controller.command_status)
            w.serial.disconnect_port()
            while w.serial.worker is not None and time.monotonic() < deadline:
                QTest.qWait(20)
            self.assertTrue(FakePort.instances[0].closed)
            self.assertEqual(w.serial.state, ConnectionState.DISCONNECTED)


if __name__ == "__main__":
    unittest.main()

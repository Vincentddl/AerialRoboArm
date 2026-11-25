from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import sys
import struct
import tempfile
import time
import unittest
import zlib
from dataclasses import replace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtWidgets import QApplication

from arm_console import experiment_page
from arm_console.models import AccelerationSample
from arm_console.v1_protocol import (AccFrame, AckFrame, CapturePageFrame, DoneFrame, ErrFrame, ExperimentInfoFrame,
                                     Hx8DiagFrame, InvalidFrame, MotionFrame, V1Decoder, V1Encoder)
from test_protocol_v1 import online_controller

app = QApplication.instance() or QApplication([])


class ExperimentModeTests(unittest.TestCase):
    def _ready_automatic(self, vertical=False, gear_index=0):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.capture_supported = True
        controller.pose_calibration_saved = True
        controller.latest_state = replace(controller.latest_state, pos_mdeg=90000 if vertical else 160000)
        page = experiment_page.ExperimentPage(controller)
        self.addCleanup(controller.shutdown)
        self.addCleanup(page.close)
        self.addCleanup(page._timer.stop)
        page.kind.setCurrentIndex(2 if vertical else 0)
        if vertical:
            page.vertical_gear.setCurrentIndex(gear_index)
        page.auto_count.setValue(1)
        page._toggle_batch()
        return page, controller, link

    def _automatic_state(self, page, controller, now, pos, **fields):
        controller.latest_state = replace(controller.latest_state, t_ms=controller.latest_state.t_ms+1,
                                           pos_mdeg=pos, reply_to=controller._query_ids["STATE"], **fields)
        controller._last_state_received_at = now
        page._advance_batch(now)

    def _automatic_acc(self, page, now, *, noisy=False):
        for i in range(22):
            x = .010 if noisy and i % 2 else 0.0
            sample = AccelerationSample(now, x, .031, .998, "g", "test", "MCU")
            page.on_acc(AccFrame(i+1, 50000+i*50, sample), now-1.05+i*.05)

    def _finish_automatic_motion(self, page, controller, now, *, valid=True, settled_error=None, capture=None):
        run = page.active
        target = run.params["target_mdeg"]
        page.on_wire_written(f"XMOVE,{run.req_id},0,1,100\n".encode(), True, now-.5)
        controller.handle_frame(AckFrame(run.req_id, "XMOVE", "ACCEPTED"))
        if settled_error is None:
            page.on_motion(MotionFrame(run.req_id, "END", 50000, target, target, "NONE"), now)
            controller.handle_frame(DoneFrame(run.req_id, "XMOVE", target))
        else:
            pos = target + settled_error
            state = replace(controller.latest_state, pos_mdeg=pos, target_mdeg=target,
                            motion="MOVING", last_req_id=run.req_id)
            settle = controller._move_settle
            controller.handle_frame(replace(state, t_ms=state.t_ms+1))
            controller.handle_frame(replace(state, t_ms=state.t_ms+settle.min_ms+2))
            stop = settle.stop_req_id
            self.assertIsNotNone(stop)
            controller.handle_frame(AckFrame(stop, "STOP", "APPLIED"))
            controller.handle_frame(MotionFrame(run.req_id, "ABORT", state.t_ms+settle.min_ms+3,
                                                target, pos, "STOPPED"))
            controller.handle_frame(ErrFrame(run.req_id, "STOPPED"))
            controller.handle_frame(replace(state, t_ms=state.t_ms+settle.min_ms+4, motion="IDLE",
                                            reply_to=controller._query_ids["STATE"]))
        if run.record:
            run.high_rate_result = capture if capture is not None else (
                {"status": "ok", "acc_hz": 201., "hx8_hz": 100., "issues": []}
                if valid else {"status": "incomplete", "reason": "test timeout"})
            if run.params.get("signal_round") and valid and capture is None:
                run.high_rate_result.update(acc=[{"t_rel_ms": -50}, {"t_rel_ms": 1750}],
                    hx8=[{"t_rel_ms": 0, "pos_mdeg": run.params["start_mdeg"]}] +
                        [{"t_rel_ms": tick, "pos_mdeg": target} for tick in (1710, 1720, 1730)])
            page._finish("完成", now)

    def test_automatic_vertical_waits_for_quiet_and_switches_only_after_return(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "VERTICAL_DATA_PATH", Path(directory) / "vertical.json"):
            page, controller, link = self._ready_automatic(vertical=True, gear_index=2)
            now = time.monotonic()
            self._automatic_state(page, controller, now, 90000)
            self.assertEqual(page.batch.phase, "stable")
            self._automatic_acc(page, now, noisy=True)
            self._automatic_state(page, controller, now, 90000)
            self.assertIsNone(page.active)
            for speed in (45, 78):
                # Fresh wire sample continuity rather than timestamps from the previous cycle.
                page.batch.acc.clear()
                page._baseline.clear()
                self._automatic_acc(page, time.monotonic())
                self._automatic_state(page, controller, time.monotonic(), 90000)
                self.assertIsNotNone(page.active, page.status.text())
                self.assertEqual(page.active.params["speed_percent"], speed)
                self.assertEqual(page.active.params["target_mdeg"], 20000)
                self._finish_automatic_motion(page, controller, time.monotonic(),
                                              settled_error=1100 if speed == 45 else None)
                self.assertEqual(page.batch.valid, 1)
                now = max(time.monotonic(), page.batch.rearm_at)
                self._automatic_state(page, controller, now, 21100 if speed == 45 else 20000)
                self.assertEqual(page.active.params["speed_percent"], 25)
                self.assertFalse(page.active.record)
                if speed == 45:
                    self._finish_automatic_motion(page, controller, time.monotonic(), settled_error=-2500)
                    now = page.batch.rearm_at
                    self._automatic_state(page, controller, now, 87500)
                    self.assertTrue(page.batch.return_corrected)
                    self.assertEqual(page.active.params["target_mdeg"], 90000)
                    # A definitive refusal during firmware rearm must not count as an outgoing run.
                    controller.handle_frame(ErrFrame(page.active.req_id, "NOT_READY"))
                    self.assertEqual(page.batch.completed, 1)
                    self._automatic_state(page, controller, page.batch.rearm_at-.1, 87500)
                    self.assertIsNone(page.active)
                    self._automatic_state(page, controller, page.batch.rearm_at, 87500)
                    self.assertFalse(page.active.record)
                    self.assertEqual(page.active.params["target_mdeg"], 90000)
                self._finish_automatic_motion(page, controller, time.monotonic())
                self._automatic_state(page, controller, time.monotonic(), 90000)
                self.assertEqual(page.batch.phase, "stable")
            self._automatic_acc(page, time.monotonic())
            self._automatic_state(page, controller, time.monotonic(), 90000)
            self.assertIsNone(page.batch)
            self.assertEqual([r["speed_percent"] for r in page.vertical_records], [45, 78])
            self.assertTrue(all(r["automatic_batch"]["valid"] for r in page.vertical_records))
            self.assertEqual([r["terminal"] for r in page.vertical_records], ["SETTLED", "END"])
            self.assertEqual([(r["gear"], r["control_arm_gear"], r["other_arm_gear"])
                              for r in page.vertical_records], [("主控臂 4 挡 · 其他臂 3 挡", 4, 3)] * 2)
            self.assertTrue(all(not payload.endswith(b",100,1\n") for payload, _ in link.sent
                                if payload.startswith(b"XMOVE,") and b",90000," in payload))
            self.assertIsNotNone(page._completion_dialog)
            self.assertFalse(page._completion_dialog.isModal())
            page._completion_dialog.close()

    def test_automatic_structure_conditions_and_tail_only_cutoff(self):
        self.assertEqual([(c.name, c.speed_percent) for c in experiment_page.batch_conditions(0)],
                         [(name, speed) for name in ("短", "中", "长") for speed in (25, 45, 78)])
        capture = {"status": "review", "flags": 1, "issues": ["高频窗口达到板载容量或时限"],
                   "acc": [{"t_rel_ms": 1738}],
                   "hx8": [{"t_rel_ms": i, "pos_mdeg": 31000} for i in range(1450, 1744, 10)]}
        record = {"terminal": "SETTLED", "end_pos_mdeg": 31000, "target_mdeg": 29800,
                  "high_rate_capture": capture}
        self.assertTrue(experiment_page.batch_capture_valid(record))
        capture["hx8"][-2]["pos_mdeg"] = 32500
        self.assertFalse(experiment_page.batch_capture_valid(record))

    def test_automatic_waits_past_old_limit_for_home_and_stillness(self):
        page, controller, link = self._ready_automatic()
        now = time.monotonic()
        home = page.start_target_mdeg
        self._automatic_state(page, controller, now+60, home-10000, motion="MOVING")
        self.assertEqual(page.batch.phase, "route")
        self.assertTrue(page.batch.running)
        self._automatic_state(page, controller, now+61, home, motion="IDLE")
        self.assertEqual(page.batch.phase, "stable")

        self._automatic_state(page, controller, now+120, home)
        page.batch.acc.clear()
        self._automatic_acc(page, now+121, noisy=True)
        self._automatic_state(page, controller, now+121, home)
        self.assertTrue(page.batch.running)
        self.assertIsNone(page.active)
        self.assertFalse(any(payload.startswith(b"XMOVE,") for payload, _ in link.sent))

        self._automatic_state(page, controller, now+122.5, home)
        self._automatic_acc(page, now+123)
        self._automatic_state(page, controller, now+123, home)
        self.assertTrue(page.active.record)
        self.assertEqual(sum(payload.startswith(b"XMOVE,") for payload, _ in link.sent), 1)

    def test_automatic_incomplete_capture_and_low_voltage_pause_preserve_progress(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page, controller, link = self._ready_automatic()
            self._automatic_state(page, controller, time.monotonic(), 160000)
            self._automatic_acc(page, time.monotonic())
            self._automatic_state(page, controller, time.monotonic(), 160000)
            self._finish_automatic_motion(page, controller, time.monotonic(), valid=False)
            self.assertEqual((page.batch.index, page.batch.completed, page.batch.retries), (0, 0, 1))
            self.assertFalse(page.records[-1]["automatic_batch"]["valid"])
            self._automatic_state(page, controller, time.monotonic(), page.records[-1]["target_mdeg"])
            self.assertFalse(page.active.record)
            page.pause_for_low_voltage()
            self.assertTrue(any(payload.startswith(b"STOP,") for payload, _ in link.sent))
            self.assertIsNone(page.active)
            self.assertFalse(page.batch.running)
            self.assertEqual(page.batch.completed, 0)
            self.assertIsNone(page._completion_dialog)

    def test_automatic_imu_dropout_waits_for_feedback_then_returns_and_retries(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page, controller, link = self._ready_automatic()
            now = time.monotonic()
            self._automatic_state(page, controller, now, 160000)
            self._automatic_acc(page, now+1)
            self._automatic_state(page, controller, now+1, 160000)
            target = page.active.params["target_mdeg"]
            capture = {"status": "review", "flags": 6, "issues": ["出臂期间板端 IMU 断流"],
                       "acc_hz": 192., "hx8_hz": 100., "max_acc_gap_ms": 25}
            self._finish_automatic_motion(page, controller, now+1.5, capture=capture)
            saved = page.records[-1]["automatic_batch"]
            self.assertFalse(saved["valid"])
            self.assertIn("25 ms", saved["reason"])
            self._automatic_state(page, controller, now+2, target, imu_online=False, imu_hz=0)
            self.assertIn("板端 IMU 断流", page.status.text())
            self._automatic_state(page, controller, now+3, target, imu_online=True, imu_hz=100)
            self.assertIsNone(page.active)
            self.assertEqual(sum(payload.startswith(b"XMOVE,") for payload, _ in link.sent), 1)
            self._automatic_state(page, controller, now+4, target, imu_online=True, imu_hz=200)
            self.assertFalse(page.active.record)
            self.assertEqual(page.active.params["target_mdeg"], 160000)
            self._finish_automatic_motion(page, controller, now+4.5)
            self._automatic_state(page, controller, now+5, 160000)
            self.assertIsNone(page.active)
            self._automatic_acc(page, now+6.1)
            self._automatic_state(page, controller, now+6.1, 160000)
            self.assertTrue(page.active.record)
            self.assertEqual(page.active.params["target_mdeg"], target)
            self.assertEqual((page.batch.index, page.batch.completed, page.batch.retries), (0, 0, 1))

    def test_capture_reports_board_imu_dropout_from_firmware_flags(self):
        samples = b"".join(struct.pack("<Hhhh", tick, 0, 0, 1000)
                           for tick in [*range(880, 1030, 5), 1050])
        angles = b"".join(struct.pack("<Hi", tick, 160000) for tick in range(1000, 1060, 10))
        download = experiment_page.CaptureDownload(0, metadata=(31, 6, 1000, 1254, 6),
                                                  data=bytearray(samples+angles))
        result = download.result()
        self.assertEqual(result["status"], "review")
        self.assertIn("出臂期间板端 IMU 断流", result["issues"])
        self.assertEqual(result["max_acc_gap_ms"], 25)

    def test_automatic_fault_recovery_requires_current_transient_diagnostic(self):
        for cause in ("AGE", "STATUS"):
            with self.subTest(cause=cause), tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                    experiment_page, "DATA_PATH", Path(directory) / "records.json"):
                page, controller, link = self._ready_automatic()
                self._automatic_state(page, controller, time.monotonic(), 160000)
                self._automatic_acc(page, time.monotonic())
                self._automatic_state(page, controller, time.monotonic(), 160000)
                run = page.active
                event_ms = controller.latest_state.t_ms
                controller.last_hx8_diag = Hx8DiagFrame(event_ms, 450, 80000, 12000, 0, 0, cause)
                page.on_motion(MotionFrame(run.req_id, "ABORT", event_ms, run.params["target_mdeg"],
                                           80000, "FEEDBACK_STALE"), time.monotonic())
                controller.handle_frame(ErrFrame(run.req_id, "FEEDBACK_STALE"))
                self._automatic_state(page, controller, time.monotonic(), 80000, fault_code="FEEDBACK_STALE")
                self.assertEqual(page.batch.completed, 0)
                self.assertFalse(page.records[-1]["automatic_batch"]["valid"])
                if cause == "STATUS":
                    self.assertFalse(page.batch.running)
                    self.assertFalse(any(payload.startswith(b"STOP,") for payload, _ in link.sent))
                    self.assertTrue(page._recover())
                else:
                    self.assertTrue(page.batch.recovery_used)
                stop = page._recovery_req_id
                controller.handle_frame(AckFrame(stop, "STOP", "APPLIED"))
                controller.handle_frame(replace(controller.latest_state, t_ms=event_ms+5, fault_code="NONE",
                                                motion="IDLE", reply_to=controller._query_ids["STATE"]))
                self.assertIsNone(page.active)
                self._automatic_state(page, controller, page.batch.rearm_at, 80000)
                self.assertFalse(page.active.record)
                self.assertEqual(page.active.params["target_mdeg"], 160000)
                self.assertEqual(page.batch.completed, 0)
                page._timer.stop()

    def test_low_voltage_prevents_automatic_resume_after_fault_recovery(self):
        page, controller, link = self._ready_automatic()
        self._automatic_state(page, controller, time.monotonic(), 80000, fault_code="FAULT")
        self.assertTrue(page.batch.resume_after_recovery)
        page.pause_for_low_voltage()
        self.assertTrue(page._recover())
        stop = page._recovery_req_id
        controller.handle_frame(AckFrame(stop, "STOP", "APPLIED"))
        controller.handle_frame(replace(controller.latest_state, t_ms=controller.latest_state.t_ms+1,
                                        fault_code="NONE", reply_to=controller._query_ids["STATE"]))
        page._advance_batch(time.monotonic())
        self.assertFalse(page.batch.running)
        self.assertIsNone(page.active)
        self.assertFalse(any(payload.startswith(b"XMOVE,") for payload, _ in link.sent))

    def test_buffered_capture_retries_missing_page_and_saves_signed_samples(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.capture_supported = True
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000, target_mdeg=160000)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page._start("出臂")
                run = page.active
                sent, _ = link.sent[-1]
                self.assertTrue(sent.startswith(f"XMOVE,{run.req_id},".encode()))
                self.assertTrue(sent.endswith(b",100,1\n"))
                self.assertFalse(any(b",CAPTURE," in payload for payload, _ in link.sent))
                tx = time.monotonic()
                page.on_wire_written(sent, True, tx)
                page.on_motion(MotionFrame(run.req_id, "END", 80, 20000, 20000, "NONE"), tx + .3)
                page._on_result(run.req_id, "XMOVE", "DONE")
                old_id = run.download.request_id
                run.download.sent_at -= 1
                page._check_timeout()
                self.assertNotEqual(run.download.request_id, old_id)
                origin = 65530
                acc = [struct.pack("<Hhhh", (origin+i) % 65536, -25, 30, 998)
                       for i in range(-100, 51, 5)]
                arm = [struct.pack("<Hi", (origin+i) % 65536, -40000-i)
                       for i in range(0, 51, 10)]
                data = b"".join(acc + arm)
                decoder = V1Decoder()
                for start in range(0, len(data), 60):
                    body = (f"CAPDATA,{run.download.request_id},{run.req_id},{start//60},"
                            f"{len(acc)},{len(arm)},{origin},{65580},0,").encode() + base64.b64encode(data[start:start+60])
                    wire = body + f",{zlib.crc32(body):08x}\n".encode()
                    if not start:
                        corrupt = body + b",deadbeef\n"
                        self.assertIsInstance(decoder.feed(corrupt, tx+.4)[0], InvalidFrame)
                    frame = decoder.feed(wire, tx+.4)[0]
                    self.assertIsInstance(frame, CapturePageFrame)
                    if not start:
                        page.on_capture_page(replace(frame, req_id=old_id), tx+.4)
                        self.assertEqual(run.download.page, 0)
                    page.on_capture_page(frame, tx+.4)
                self.assertIsNone(page.active)
                saved = json.loads(experiment_page.DATA_PATH.read_text(encoding="utf-8"))[-1]
                stored = saved["high_rate_capture"]
                self.assertEqual((stored["status"], stored["acc_hz"], stored["hx8_hz"]), ("ok", 200, 100))
                self.assertEqual(stored["acc"][0], {"t_rel_ms": -100, "x_g": -.025, "y_g": .030, "z_g": .998})
                self.assertEqual(stored["hx8"][-1], {"t_rel_ms": 50, "pos_mdeg": -40050})
                transfer = saved["capture_transfer"]
                self.assertEqual(transfer["ignored_pages"], 1)
                self.assertEqual(transfer["requests"][0]["outcome"], "timeout")
                self.assertEqual(transfer["requests"][1]["outcome"], "received")
            finally:
                page._timer.stop()
                page.close()
                controller.shutdown()

    def test_failed_capture_readback_releases_experiment_and_keeps_summary(self):
        controller, _ = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.capture_supported = True
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000, target_mdeg=160000)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page._start("出臂")
                run = page.active
                now = time.monotonic()
                page.on_wire_written(f"XMOVE,{run.req_id},20000,125,100,1\n".encode(), True, now)
                sample = AccelerationSample(now, -.2, 0., 1., "g", "test", "MCU")
                page.on_acc(AccFrame(1, 20, sample), now+.02)
                page.on_motion(MotionFrame(run.req_id, "END", 200, 20000, 20000, "NONE"), now+.2)
                page._on_result(run.req_id, "XMOVE", "DONE")
                for _ in range(experiment_page.CAPTURE_MAX_RETRIES):
                    run.download.sent_at -= experiment_page.CAPTURE_PAGE_TIMEOUT+1
                    page._check_timeout()
                stored = page.records[-1]
                self.assertFalse(page.capturing_trace)
                self.assertEqual((stored["terminal"], stored["sample_count"]), ("END", 1))
                self.assertEqual(stored["high_rate_capture"]["status"], "incomplete")
                self.assertEqual(stored["high_rate_capture"]["reason"], "高频回传首包超时")
                self.assertEqual(stored["capture_transfer"]["received_pages"], 0)
                self.assertEqual(len(stored["capture_transfer"]["requests"]),
                                 experiment_page.CAPTURE_MAX_RETRIES)
            finally:
                page._timer.stop()
                page.close()
                controller.shutdown()

    def test_submode_parameters_are_independent_and_only_relevant_rows_are_visible(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000, target_mdeg=160000)
        page = experiment_page.ExperimentPage(controller)
        try:
            page.travel.setCurrentIndex(2)
            page.gear.setCurrentIndex(1)
            page.structure_speed.setCurrentIndex(2)
            self.assertTrue(page.form.isRowVisible(page.travel))
            self.assertFalse(page.form.isRowVisible(page.signal_round))

            page.kind.setCurrentIndex(2)
            self.assertEqual([page.vertical_gear.itemText(i) for i in range(page.vertical_gear.count())],
                             ["3 中间", "主控臂 3 挡 · 其他臂 4 挡", "主控臂 4 挡 · 其他臂 3 挡", "4 较长"])
            page.vertical_gear.setCurrentIndex(3)
            page.vertical_speed.setCurrentIndex(1)
            self.assertFalse(page.form.isRowVisible(page.travel))
            self.assertFalse(page.form.isRowVisible(page.structure_speed))
            self.assertFalse(page.form.isRowVisible(page.signal_round))
            self.assertTrue(page.form.isRowVisible(page.vertical_gear))
            self.assertTrue(page.vertical_speed.isEnabled())

            page.kind.setCurrentIndex(1)
            page.signal_round.setCurrentIndex(1)
            self.assertTrue(page.form.isRowVisible(page.signal_round))
            self.assertEqual(page.signal_round.count(), 6)
            self.assertFalse(page.form.isRowVisible(page.gear))
            self.assertTrue(page.extend_button.isHidden())
            self.assertTrue(page.auto_count.isHidden())

            page.kind.setCurrentIndex(0)
            self.assertEqual(page.travel.currentIndex(), 2)
            self.assertEqual(page.gear.currentIndex(), 1)
            self.assertEqual(page.gear.count(), 5)
            self.assertEqual(page.structure_speed.currentData(), 78)
            page.kind.setCurrentIndex(2)
            self.assertEqual(page.vertical_gear.currentIndex(), 3)
            self.assertEqual(page.vertical_speed.currentData(), 78)
            page.kind.setCurrentIndex(1)
            self.assertEqual(page.signal_round.currentData(), 2)
            page._start("出臂")
            self.assertIsNone(page.active)
        finally:
            page._timer.stop()
            page.close()
            controller.shutdown()

    def test_manual_retest_waits_for_auto_stability_and_keeps_failure_diagnostics(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"), patch.object(
                experiment_page, "MANUAL_RETEST_LOG_PATH", Path(directory) / "manual.jsonl"):
            controller, _ = online_controller()
            self.addCleanup(controller.shutdown)
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.capture_supported = True
            controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            self.addCleanup(page.close)
            condition = experiment_page.signal_conditions(90000, 160000, 500, 1)[0]
            old_failure = {
                "id": "old-failure", "signal_round": 1,
                "signal_plan_version": experiment_page.SIGNAL_PLAN_VERSION,
                "normal_mdeg": 90000, "fold_mdeg": 160000, "signal_max_speed_dps": 500,
                "travel": condition.name, "speed_percent": condition.speed_percent,
                "ramp_ms": condition.ramp_ms, "target_mdeg": 90000-round(condition.travel_deg*1000),
                "end_pos_mdeg": 90000-round(condition.travel_deg*1000), "terminal": "END",
                "high_rate_capture": {"status": "review", "issues": ["IMU 校验异常"]},
                "automatic_batch": {"valid": False},
                "signal_condition_review": {"code": experiment_page.HIGH_RATE_REVIEW},
            }
            page.records.append(old_failure)
            page._save()
            page.kind.setCurrentIndex(1)
            page.signal_round.setCurrentIndex(5)
            self.assertEqual(page.manual_condition.count(), 1)
            self.assertFalse(page.extend_button.isEnabled())
            now = time.monotonic()
            controller._last_state_received_at = now
            self._automatic_acc(page, now)
            controller.latest_state = replace(controller.latest_state, t_ms=controller.latest_state.t_ms+1)
            page._sync()
            self.assertTrue(page.extend_button.isEnabled(), page.manual_ready_label.text())
            page._start("出臂")
            run = page.active
            self.assertIsNotNone(run, page.status.text())
            self.assertTrue(run.params["manual_retest"])
            self.assertEqual(run.params["signal_round"], 1)
            page.on_wire_written(f"XMOVE,{run.req_id},0,1,100\n".encode(), True, now+.01)
            run.end_at = now+.25
            run.end_pos_mdeg = run.params["target_mdeg"]
            run.terminal = "END"
            run.motion_confirmed = True
            run.high_rate_result = {"status": "review", "flags": 2, "issues": ["IMU 校验异常"],
                                    "reason": "IMU 校验异常", "acc_hz": 100., "hx8_hz": 100.,
                                    "max_acc_gap_ms": 100}
            page._finish("完成", now+.3)
            self.assertEqual(len(page.records), 2)
            self.assertFalse(page.records[-1]["manual_retest_result"]["valid"])
            self.assertEqual(page.records[0]["id"], "old-failure")
            journal = [json.loads(line) for line in (Path(directory) / "manual.jsonl").read_text(
                encoding="utf-8").splitlines()]
            self.assertEqual(journal[0]["reason"], "IMU 校验异常")
            self.assertEqual(journal[0]["diagnostics"]["capture_status"], "review")

    def test_vertical_trace_uses_saved_normal_pose_and_captures_before_and_after_motion(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.working_zero_mdeg = 104500
        controller.pose_calibration_saved = True
        controller.latest_state = replace(controller.latest_state, pos_mdeg=104500, target_mdeg=104500)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "legacy.json"), patch.object(
                experiment_page, "VERTICAL_DATA_PATH", Path(directory) / "vertical.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page.kind.setCurrentIndex(2)
                page.gear.setCurrentIndex(1)
                page.vertical_gear.setCurrentIndex(1)
                self.assertEqual(page.return_button.text(), "回收到正常位")
                self.assertEqual([page.vertical_speed.itemData(i) for i in range(2)], [45, 78])
                origin = time.monotonic()

                def acc(seq, mcu_ms, x, when):
                    sample = AccelerationSample(when, x, 0.1, 1.0, "g", "test", "MCU")
                    page.on_acc(AccFrame(seq, mcu_ms, sample), when)

                for i in range(19):
                    acc(i, 1000+i*50, -0.2, origin-0.95+i*0.05)
                page._start("出臂")
                run = page.active
                self.assertIsNotNone(run)
                payload = f"XMOVE,{run.req_id},34500,225,100\n".encode()
                self.assertEqual(link.sent[-1][0], payload)
                page.on_wire_written(payload, True, origin)
                controller.handle_frame(AckFrame(run.req_id, "XMOVE", "ACCEPTED"))
                start = MotionFrame(run.req_id, "START", 2000, 34500, 104500, "NONE")
                page.on_trace_frame(start, origin+0.02)
                page.on_trace_frame(replace(controller.latest_state, t_ms=2050, pos_mdeg=96500,
                                            motion="MOVING"), origin+0.05)
                acc(19, 2050, -0.6, origin+0.05)
                end = MotionFrame(run.req_id, "END", 2400, 34500, 34500, "NONE")
                page.on_trace_frame(end, origin+0.41)
                page.on_motion(end, origin+0.41)
                with patch.object(experiment_page.time, "monotonic", return_value=origin+0.42):
                    controller.handle_frame(DoneFrame(run.req_id, "XMOVE", 34500))
                self.assertTrue(page.capturing_trace)
                self.assertFalse(experiment_page.VERTICAL_DATA_PATH.exists())
                acc(20, 2450, 0.8, origin+0.46)
                acc(21, 4450, 0.9, origin+2.45)  # Beyond the post window.
                with patch.object(experiment_page.time, "monotonic", return_value=origin+2.43):
                    page._check_timeout()
                saved = json.loads(experiment_page.VERTICAL_DATA_PATH.read_text(encoding="utf-8"))
                self.assertEqual(saved[0]["gear"], "主控臂 3 挡 · 其他臂 4 挡")
                self.assertEqual((saved[0]["control_arm_gear"], saved[0]["other_arm_gear"]), (3, 4))
                trace = saved[0]["trace"]
                self.assertTrue(trace["post_complete"])
                self.assertEqual([a["x_g"] for a in trace["acc"][-2:]], [-0.6, 0.8])
                self.assertEqual(trace["baseline_acc_count"], 19)
                self.assertEqual(trace["post_acc_count"], 1)
                self.assertEqual([m["stage"] for m in trace["motion"]], ["START", "END"])
                self.assertEqual(trace["state"][-1]["pos_mdeg"], 96500)
                self.assertEqual(saved[0]["sample_count"], 1)
                self.assertFalse(experiment_page.DATA_PATH.exists())
                reloaded = experiment_page.ExperimentPage(controller)
                try:
                    reloaded.kind.setCurrentIndex(2)
                    self.assertEqual(reloaded.table.rowCount(), 1)
                finally:
                    reloaded._timer.stop()
                controller.latest_state = replace(controller.latest_state, pos_mdeg=34500, target_mdeg=34500)
                page._start("回收")
                self.assertEqual(link.sent[-1][0], f"XMOVE,{page.active.req_id},104500,225,100\n".encode())
                self.assertFalse(page.active.record)
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_command_and_capability_protocol(self):
        self.assertEqual(V1Encoder.encode("GET", 11, "EXPERIMENT"), b"GET,11,EXPERIMENT\n")
        self.assertEqual(V1Encoder.encode("XMOVE", 12, 20000, 350, 100),
                         b"XMOVE,12,20000,350,100\n")
        decoder = V1Decoder()
        self.assertEqual(decoder.feed(b"EXPERIMENT,11,1,20,2000\n", 0),
                         [ExperimentInfoFrame(11, 1, 20, 2000)])
        self.assertIsInstance(decoder.feed(b"EXPERIMENT,11,1,20,3000\n", 0)[0], InvalidFrame)

    def test_signal_rounds_cover_the_same_84_unique_conditions_and_quantize_ratios(self):
        self.assertEqual([experiment_page.target_for_travel(x) for x in
                          experiment_page.TRAVELS.values()], [66700, 43300, 20000])
        rounds = [experiment_page.signal_conditions(99800, 164800, 500, n) for n in range(1, 5)]
        self.assertEqual(len(set(rounds)), 4)
        for conditions in rounds:
            self.assertEqual(set(conditions), set(rounds[0]))
            self.assertEqual(len({c.key for c in conditions}), 84)
        conditions = rounds[0]
        self.assertEqual(sum(c.ramp_ms in experiment_page.SIGNAL_RAMP_TIMES for c in conditions), 72)
        actual = {(c.name, c.speed_percent, ratio): c.ramp_ms for c in conditions for ratio in c.time_ratios}
        expected = ((180, 260, 380), (100, 140, 220), (60, 80, 120),
                    (220, 340, 480), (120, 180, 260), (80, 100, 160),
                    (280, 400, 580), (160, 220, 320), (80, 140, 180))
        self.assertEqual([tuple(actual[name, speed, ratio] for ratio in (40, 55, 70))
                          for name in ("短", "中", "长") for speed in (25, 45, 78)], list(expected))

    def test_signal_round_continues_only_unsaved_conditions_after_restart(self):
        controller, link = online_controller()
        self.addCleanup(controller.shutdown)
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.capture_supported = True
        controller.working_zero_mdeg = 99800
        controller.fold_offset_mdeg = 65000
        controller.latest_state = replace(controller.latest_state, pos_mdeg=164800)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            page.kind.setCurrentIndex(1)
            page.signal_round.setCurrentIndex(2)
            page._toggle_batch()
            self.assertEqual((page.batch.signal_round, page.batch.repeats, page.batch.total), (3, 1, 84))
            self._automatic_state(page, controller, time.monotonic(), 164800)
            self._automatic_acc(page, time.monotonic())
            self._automatic_state(page, controller, time.monotonic(), 164800)
            run = page.active
            first = page.batch.conditions[0]
            self.assertTrue(link.sent[-1][0].endswith(f",{run.params['speed_dps']},{first.ramp_ms},1\n".encode()))
            self._finish_automatic_motion(page, controller, time.monotonic())
            row = page.records[-1]
            self.assertTrue(row["automatic_batch"]["valid"])
            self.assertEqual(row["signal_round"], 3)
            self.assertEqual(row["signal_time_ratio_targets_percent"], list(first.time_ratios))
            planned = 200*row["ramp_ms"]*row["speed_dps"] / (
                row["fold_mdeg"]-row["target_mdeg"]+row["ramp_ms"]*row["speed_dps"])
            self.assertAlmostEqual(row["signal_nominal_time_ratio_percent"], planned, delta=0.005)
            self.assertEqual((row["control_arm_gear"], row["other_arm_gear"]), (3, 4))
            self._automatic_state(page, controller, time.monotonic(), row["target_mdeg"])
            self.assertEqual((page.active.params["target_mdeg"], page.active.params["speed_percent"],
                              page.active.params["ramp_ms"]), (164800, 25, 100))
            self._finish_automatic_motion(page, controller, time.monotonic())
            self._automatic_state(page, controller, time.monotonic(), 164800)
            page._pause_batch("测试结束")
            page._end_batch()
            self.assertIsNone(page.batch)
            page._timer.stop()
            controller.latest_state = replace(controller.latest_state, pos_mdeg=54100)
            resumed = experiment_page.ExperimentPage(controller)
            self.addCleanup(resumed._timer.stop)
            resumed.kind.setCurrentIndex(1)
            self.assertEqual(resumed.signal_progress.text(), "0 / 84 次有效出臂")
            resumed.signal_round.setCurrentIndex(2)
            self.assertEqual(resumed.signal_progress.text(), "1 / 84 次有效出臂")
            self.assertTrue(resumed.auto_button.isEnabled())
            resumed._toggle_batch()
            self.assertEqual(resumed.batch.total, 84)
            self.assertEqual(resumed.batch.completed, 1)
            self.assertNotIn(first.key, {c.key for c in resumed.batch.conditions})
            controller.set_voltage_lock(True)
            resumed.pause_for_low_voltage()
            resumed._toggle_batch()
            self.assertFalse(resumed.batch.running)
            controller.set_voltage_lock(False)
            self.assertFalse(resumed.batch.running)
            self.assertEqual(resumed.batch.completed, 1)
            resumed._toggle_batch()
            self.assertTrue(resumed.batch.running)
            self._automatic_state(resumed, controller, time.monotonic(), 54100)
            self.assertFalse(resumed.active.record)
            self.assertEqual(resumed.active.params["target_mdeg"], 164800)

    def test_fourth_retest_combines_rounds_one_to_four_and_resumes(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            controller, _ = online_controller()
            self.addCleanup(controller.shutdown)
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.capture_supported = True
            controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            page.kind.setCurrentIndex(1)
            plan1 = experiment_page.signal_conditions(90000, 160000, 500, 1)
            plan2 = experiment_page.signal_conditions(90000, 160000, 500, 2)
            plan3 = experiment_page.signal_conditions(90000, 160000, 500, 3)
            plan4 = experiment_page.signal_conditions(90000, 160000, 500, 4)

            def row(condition, number, *, code=experiment_page.HIGH_RATE_REVIEW, valid=False, **extra):
                return {"req_id": number, "signal_round": number, "signal_plan_version": experiment_page.SIGNAL_PLAN_VERSION,
                        "normal_mdeg": 90000, "fold_mdeg": 160000, "signal_max_speed_dps": 500,
                        "travel": condition.name, "speed_percent": condition.speed_percent, "ramp_ms": condition.ramp_ms,
                        "automatic_batch": {"valid": valid}, "signal_condition_review": {"code": code}, **extra}

            shared = next(condition for condition in plan3 if condition.key == plan2[0].key)
            page.records = [row(plan2[0], 2), row(shared, 3), row(plan4[0], 4),
                            row(plan2[1], 2, code=experiment_page.HX8_POWER_PROTECTION),
                            row(plan2[2], 2), row(plan2[2], 2, valid=True, signal_retest=True),
                            row(plan2[0], 2, signal_retest=True),
                            row(plan4[1], 4, code=experiment_page.HX8_POWER_PROTECTION),
                            row(plan1[1], 1), row(plan3[2], 3, normal_mdeg=91000),
                            row(plan3[3], 3), row(plan3[3], 3, code=experiment_page.HX8_POWER_PROTECTION)]
            page._save()
            page.signal_round.setCurrentIndex(4)
            pending, total, protected, review = page._signal_round_plan()
            self.assertEqual(([(c.signal_round, c.key) for c in pending], total, protected, review),
                             ([(1, plan1[1].key), (2, plan2[0].key), (3, shared.key), (4, plan4[0].key)], 4, 0, 0))
            page._toggle_batch()
            self.assertEqual((page.batch.signal_round, page.batch.signal_retest, page.batch.total), (0, True, 4))
            now = time.monotonic()
            self._automatic_state(page, controller, now, 160000)
            self._automatic_acc(page, now+1)
            self._automatic_state(page, controller, now+1, 160000)
            self._finish_automatic_motion(page, controller, now+1.5)
            self.assertTrue(page.records[-1]["signal_retest"])
            self.assertEqual(page.records[-1]["signal_retest_stage"], experiment_page.SIGNAL_RETEST_STAGE)
            self.assertEqual(page.records[-1]["signal_round"], 1)
            self.assertTrue(page.records[-1]["automatic_batch"]["valid"])
            # Original failures remain, and the successful retest fills round 1 only.
            self.assertEqual(page.records[0]["signal_condition_review"]["code"], experiment_page.HIGH_RATE_REVIEW)
            page._pause_batch("测试结束")
            reopened = experiment_page.ExperimentPage(controller)
            self.addCleanup(reopened._timer.stop)
            reopened.kind.setCurrentIndex(1)
            reopened.signal_round.setCurrentIndex(1)
            normal_pending, _, _, _ = reopened._signal_round_plan()
            self.assertNotIn(plan2[0], normal_pending)
            reopened.signal_round.setCurrentIndex(4)
            pending, total, protected, review = reopened._signal_round_plan()
            self.assertEqual(([(c.signal_round, c.key) for c in pending], total, protected, review),
                             ([(2, plan2[0].key), (3, shared.key), (4, plan4[0].key)], 4, 0, 0))
            reopened._toggle_batch()
            self.assertEqual((reopened.batch.completed, reopened.batch.total), (1, 4))

    def test_fourth_retest_empty_and_repeated_review_outcome(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            controller, _ = online_controller()
            self.addCleanup(controller.shutdown)
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            page.kind.setCurrentIndex(1)
            page.signal_round.setCurrentIndex(4)
            self.assertEqual(page._signal_round_plan(), ((), 0, 0, 0))
            self.assertFalse(page.auto_button.isEnabled())
            self.assertEqual(page.auto_button.text(), "无待复测工况")
            condition = experiment_page.signal_conditions(90000, 160000, 500, 3)[0]
            row = {"signal_round": 3, "signal_plan_version": experiment_page.SIGNAL_PLAN_VERSION,
                   "normal_mdeg": 90000, "fold_mdeg": 160000, "signal_max_speed_dps": 500,
                   "travel": condition.name, "speed_percent": condition.speed_percent, "ramp_ms": condition.ramp_ms,
                   "automatic_batch": {"valid": False},
                   "signal_condition_review": {"code": experiment_page.HIGH_RATE_REVIEW}}
            page.records = [row, dict(row, signal_retest=True)]
            self.assertEqual(page._signal_round_plan(), ((replace(condition, signal_round=3),), 1, 0, 0))
            page.records.append(dict(row, signal_retest=True, signal_retest_stage=4))
            self.assertEqual(page._signal_round_plan(), ((), 1, 0, 1))
            page.records[-1]["signal_condition_review"] = {"code": experiment_page.HX8_POWER_PROTECTION}
            self.assertEqual(page._signal_round_plan(), ((), 1, 1, 0))

    def test_signal_capture_must_include_the_complete_kinematic_window(self):
        capture = {"status": "review", "flags": 1, "issues": ["高频窗口达到板载容量或时限"],
                   "acc": [{"t_rel_ms": -100}, {"t_rel_ms": 1749}],
                   "hx8": [{"t_rel_ms": 0, "pos_mdeg": 160000}] +
                          [{"t_rel_ms": t, "pos_mdeg": 29800} for t in (1710, 1720, 1730, 1740)]}
        record = {"signal_round": 1, "terminal": "SETTLED", "end_pos_mdeg": 29800, "target_mdeg": 29800,
                  "high_rate_capture": capture}
        self.assertTrue(experiment_page.batch_capture_valid(record))
        capture["acc"][-1]["t_rel_ms"] = 1721
        self.assertFalse(experiment_page.batch_capture_valid(record))

    def test_signal_power_protection_counts_once_and_skips_after_recovery(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            controller, link = online_controller()
            self.addCleanup(controller.shutdown)
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.capture_supported = True
            controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            page.kind.setCurrentIndex(1)
            page._toggle_batch()
            blocked = page.batch.conditions[0]
            now = time.monotonic()
            self._automatic_state(page, controller, now, 160000)
            self._automatic_acc(page, now+1)
            self._automatic_state(page, controller, now+1, 160000)
            run = page.active
            event_ms = controller.latest_state.t_ms+10
            controller.handle_frame(Hx8DiagFrame(event_ms, 0, 60100, 10721, 1072, 0x41, "STATUS"))
            page.on_motion(MotionFrame(run.req_id, "ABORT", event_ms, run.params["target_mdeg"],
                                       60100, "FEEDBACK_STALE"), now+1.1)
            controller.handle_frame(ErrFrame(run.req_id, "FEEDBACK_STALE"))
            row = page.records[-1]
            self.assertFalse(row["automatic_batch"]["valid"])
            self.assertFalse(row["comparable"])
            self.assertEqual(row["signal_condition_review"]["evidence"][0]["req_id"], run.req_id)
            self.assertEqual((page.batch.completed, page.batch.valid, page.batch.running), (1, 0, False))
            self.assertIn("功率保护待补测 1", page.signal_progress.text())
            page.records.append(dict(row))
            remaining, total, protected, capture_review = page._signal_round_plan()
            self.assertEqual((len(remaining), protected, capture_review), (total-1, 1, 0))
            self.assertNotIn(blocked.key, {c.key for c in remaining})

            self._automatic_state(page, controller, now+1.2, 60100, motion="FAULT", fault_code="FEEDBACK_STALE")
            self.assertEqual(sum(wire.startswith(b"XMOVE,") for wire, _ in link.sent), 1)
            self.assertTrue(page._recover())
            stop = page._recovery_req_id
            controller.handle_frame(AckFrame(stop, "STOP", "APPLIED"))
            controller.handle_frame(replace(controller.latest_state, t_ms=event_ms+2, fault_code="NONE",
                                            motion="IDLE", reply_to=controller._query_ids["STATE"]))
            self.assertIsNone(page.active)
            at = page.batch.rearm_at
            self._automatic_state(page, controller, at, 60100)
            self.assertFalse(page.active.record)
            self._finish_automatic_motion(page, controller, at+.1)
            self._automatic_state(page, controller, at+.2, 160000)
            self._automatic_acc(page, at+1.3)
            self._automatic_state(page, controller, at+1.3, 160000)
            next_run = page.active
            self.assertTrue(next_run.record)
            self.assertNotEqual((next_run.params["travel"], next_run.params["speed_percent"], next_run.params["ramp_ms"]),
                                blocked.key)
            self.assertEqual(page.batch.completed, 1)

            reloaded = experiment_page.ExperimentPage(controller)
            self.addCleanup(reloaded._timer.stop)
            reloaded.kind.setCurrentIndex(1)
            self.assertEqual(reloaded._signal_round_plan()[2], 1)

    def test_signal_capture_failure_auto_skips_returns_and_persists(self):
        for status, settled_error in (("review", None), ("review", 1500), ("incomplete", None)):
            with self.subTest(status=status, settled_error=settled_error), tempfile.TemporaryDirectory(
                    dir=ROOT / "runtime") as directory, patch.object(
                    experiment_page, "DATA_PATH", Path(directory) / "records.json"):
                controller, link = online_controller()
                self.addCleanup(controller.shutdown)
                controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
                controller.capture_supported = True
                controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
                page = experiment_page.ExperimentPage(controller)
                self.addCleanup(page._timer.stop)
                page.kind.setCurrentIndex(1)
                page._toggle_batch()
                blocked = page.batch.conditions[0]
                now = time.monotonic()
                self._automatic_state(page, controller, now, 160000)
                self._automatic_acc(page, now+1)
                self._automatic_state(page, controller, now+1, 160000)
                capture = ({"status": "review", "flags": 2, "issues": ["IMU 校验异常"],
                            "acc_hz": 100., "hx8_hz": 100., "max_acc_gap_ms": 100}
                           if status == "review" else {"status": "incomplete", "reason": "回传缺页"})
                for attempt in (1, 2):
                    self._finish_automatic_motion(page, controller, now+1.5, capture=capture)
                    self.assertNotIn("signal_condition_review", page.records[-1])
                    self.assertEqual((page.batch.running, page.batch.completed, page.batch.retries), (True, 0, attempt))
                    self._automatic_state(page, controller, now+2, page.records[-1]["end_pos_mdeg"], motion="IDLE")
                    self.assertFalse(page.active.record)
                    self._finish_automatic_motion(page, controller, now+2.1)
                    self._automatic_state(page, controller, now+2.2, 160000)
                    self._automatic_acc(page, now+3.3)
                    self._automatic_state(page, controller, now+3.3, 160000)
                    self.assertTrue(page.active.record)
                    self.assertEqual((page.active.params["travel"], page.active.params["speed_percent"],
                                      page.active.params["ramp_ms"]), blocked.key)
                    now += 4
                self._finish_automatic_motion(page, controller, now+1.5, capture=capture,
                                              settled_error=settled_error)
                row = page.records[-1]
                self.assertFalse(row["automatic_batch"]["valid"])
                self.assertEqual(row["signal_condition_review"]["source"], "automatic_deferred")
                self.assertEqual(row["signal_condition_review"]["consecutive_failures"], 3)
                self.assertEqual((page.batch.running, page.batch.completed, page.batch.valid), (True, 1, 0))
                self.assertIn("高频采样待补测", page.status.text())
                at = max(now+2, page.batch.rearm_at)
                self._automatic_state(page, controller, at, row["end_pos_mdeg"], motion="IDLE")
                self.assertFalse(page.active.record)
                self.assertEqual(page.active.params["target_mdeg"], 160000)
                self._finish_automatic_motion(page, controller, at+.1)
                self._automatic_state(page, controller, at+.2, 160000)
                self.assertIsNone(page.active)
                self._automatic_acc(page, at+1.3)
                self._automatic_state(page, controller, at+1.3, 160000)
                self.assertTrue(page.active.record)
                self.assertNotEqual((page.active.params["travel"], page.active.params["speed_percent"],
                                     page.active.params["ramp_ms"]), blocked.key)
                self.assertFalse(page.batch.capture_deferred)
                reopened = experiment_page.ExperimentPage(controller)
                self.addCleanup(reopened._timer.stop)
                reopened.kind.setCurrentIndex(1)
                pending, total, _, deferred = reopened._signal_round_plan()
                self.assertEqual((len(pending), deferred), (total-1, 1))
                self.assertNotIn(blocked.key, {c.key for c in pending})

    def test_signal_capture_skip_is_not_advanced_when_save_fails(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page, controller, _ = self._ready_automatic()
            page.batch = None
            page.kind.setCurrentIndex(1)
            page._toggle_batch()
            now = time.monotonic()
            self._automatic_state(page, controller, now, 160000)
            self._automatic_acc(page, now+1)
            self._automatic_state(page, controller, now+1, 160000)
            page.batch.retries = 2
            with patch.object(page, "_save", side_effect=OSError("test disk failure")):
                self._finish_automatic_motion(page, controller, now+1.5, valid=False)
            self.assertEqual(page.batch.completed, 0)
            self.assertFalse(page.batch.running)
            self.assertFalse(page.batch.capture_deferred)
            self.assertFalse(page.records)

    def test_signal_operator_defers_one_repeated_capture_failure_without_counting_valid(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            controller, link = online_controller()
            self.addCleanup(controller.shutdown)
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.capture_supported = True
            controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
            page = experiment_page.ExperimentPage(controller)
            self.addCleanup(page._timer.stop)
            page.kind.setCurrentIndex(1)
            remaining, total, _, _ = page._signal_round_plan()
            blocked = remaining[0]
            target = experiment_page.target_for_travel(blocked.travel_deg, controller.working_zero_mdeg)
            for req_id in (1, 2, 3):
                page.records.append({"req_id": req_id, "signal_round": 1, "signal_plan_version": 1,
                    "normal_mdeg": controller.working_zero_mdeg, "fold_mdeg": controller.fold_target_mdeg,
                    "signal_max_speed_dps": controller.info.max_speed_dps, "travel": blocked.name,
                    "speed_percent": blocked.speed_percent, "ramp_ms": blocked.ramp_ms, "target_mdeg": target,
                    "end_pos_mdeg": target, "terminal": "SETTLED", "automatic_batch": {"valid": False},
                    "high_rate_capture": {"status": "incomplete", "reason": "回传缺页"}})
                page._sync()
                self.assertEqual(page.defer_capture_button.isEnabled(), req_id == 3)
            controller.set_voltage_lock(True)
            self.assertFalse(page.defer_capture_button.isEnabled())
            controller.set_voltage_lock(False)
            page.defer_capture_button.click()
            self.assertFalse(page.records[-1]["automatic_batch"]["valid"])
            self.assertEqual(page.records[-1]["signal_condition_review"]["code"], experiment_page.HIGH_RATE_REVIEW)
            self.assertEqual(page.signal_progress.text(), f"已测试 1 / {total} · 有效 0 · 高频采样待补测 1")
            self.assertFalse(any(payload.startswith(b"XMOVE,") for payload, _ in link.sent))
            reopened = experiment_page.ExperimentPage(controller)
            self.addCleanup(reopened._timer.stop)
            reopened.kind.setCurrentIndex(1)
            pending, _, _, capture_review = reopened._signal_round_plan()
            self.assertEqual((len(pending), capture_review), (total-1, 1))
            self.assertNotIn(blocked.key, {c.key for c in pending})
            reopened.signal_round.setCurrentIndex(1)
            self.assertEqual(reopened._signal_round_plan()[3], 0)

    def test_signal_fault_without_matching_power_status_remains_untested(self):
        for status, tick_offset in ((0x04, 0), (0x41, -10)):
            with self.subTest(status=status, tick_offset=tick_offset), tempfile.TemporaryDirectory(
                    dir=ROOT / "runtime") as directory, patch.object(
                    experiment_page, "DATA_PATH", Path(directory) / "records.json"):
                controller, _ = online_controller()
                self.addCleanup(controller.shutdown)
                controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
                controller.capture_supported = True
                controller.latest_state = replace(controller.latest_state, pos_mdeg=160000)
                page = experiment_page.ExperimentPage(controller)
                self.addCleanup(page._timer.stop)
                page.kind.setCurrentIndex(1)
                page._toggle_batch()
                now = time.monotonic()
                self._automatic_state(page, controller, now, 160000)
                self._automatic_acc(page, now+1)
                self._automatic_state(page, controller, now+1, 160000)
                run = page.active
                event_ms = controller.latest_state.t_ms+20
                controller.handle_frame(Hx8DiagFrame(event_ms+tick_offset, 0, 60100, 10721, 1072, status, "STATUS"))
                page.on_motion(MotionFrame(run.req_id, "ABORT", event_ms, run.params["target_mdeg"],
                                           60100, "FEEDBACK_STALE"), now+1.1)
                self.assertNotIn("signal_condition_review", page.records[-1])
                self.assertEqual(page.batch.completed, 0)
                page._timer.stop()

    def test_automatic_return_uses_matched_idle_feedback_when_end_replies_are_missing(self):
        for done_received in (False, True):
            with self.subTest(done_received=done_received), tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                    experiment_page, "DATA_PATH", Path(directory) / "records.json"):
                page, controller, link = self._ready_automatic()
                now = time.monotonic()
                self._automatic_state(page, controller, now, 160000)
                self._automatic_acc(page, now)
                self._automatic_state(page, controller, now, 160000)
                self._finish_automatic_motion(page, controller, now)
                self.assertEqual(page.batch.completed, 1)
                self._automatic_state(page, controller, now, 66700)
                run = page.active
                self.assertFalse(run.record)
                req = run.req_id
                controller.handle_frame(AckFrame(req, "XMOVE", "ACCEPTED"))
                frame = replace(controller.latest_state, pos_mdeg=159200, target_mdeg=160000,
                                last_req_id=req, reply_to=req+1, motion="IDLE")
                if not done_received:
                    for delta in (200, 450, 750):
                        controller.handle_frame(replace(frame, t_ms=frame.t_ms+delta, last_req_id=req+1))
                    self.assertIs(page.active, run)
                    for delta in (950, 1200, 1450):
                        controller.handle_frame(replace(frame, t_ms=frame.t_ms+delta, target_mdeg=162000))
                    self.assertIs(page.active, run)
                    for delta in (1650, 1900):
                        controller.handle_frame(replace(frame, t_ms=frame.t_ms+delta))
                    self.assertIs(page.active, run)
                    controller.handle_frame(replace(frame, t_ms=frame.t_ms+2150))
                else:
                    controller.handle_frame(DoneFrame(req, "XMOVE", 159200))
                self.assertIsNone(page.active)
                self.assertNotIn(req, controller._pending)
                self.assertEqual(page.batch.completed, 1)
                now = time.monotonic()
                self._automatic_state(page, controller, now, 159200)
                self.assertEqual(page.batch.phase, "stable")
                self.assertIsNone(page.active)
                self._automatic_acc(page, now)
                self._automatic_state(page, controller, now, 159200)
                self.assertTrue(page.active.record)
                self.assertEqual(len(page.records), 1)
                self.assertEqual(sum(payload.startswith(b"XMOVE,") for payload, _ in link.sent), 3)

    def test_old_firmware_or_mismatched_calibration_cannot_start_experiment(self):
        controller, link = online_controller()
        try:
            self.assertIsNone(controller.request_experiment_move(66700, 25, 100))
            controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
            controller.cal = replace(controller.cal, max_extension_mdeg=60000)
            self.assertIsNone(controller.request_experiment_move(66700, 25, 100))
            controller.cal = replace(controller.cal, max_extension_mdeg=90000)
            self.assertIsNone(controller.request_experiment_move(66700, 33, 100))
            self.assertFalse(any(payload.startswith(b"XMOVE,") for payload, _ in link.sent))
        finally:
            controller.shutdown()

    def test_one_run_records_actual_wire_window_and_signed_mean(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000,
                                          target_mdeg=160000)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page._start("出臂")
                self.assertIsNotNone(page.active)
                run = page.active
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},66700,125,100\n".encode())
                sent_at = time.monotonic()
                page.on_wire_written(payload, True, sent_at)
                for index, (x, y, z) in enumerate(((-0.2, 0.8, 1.0), (0.4, -0.1, 0.5))):
                    sample = AccelerationSample(1.0 + index, x, y, z, "g", "test", "MCU")
                    page.on_acc(AccFrame(index, index * 50, sample), sent_at + 0.1 + index * 0.05)
                page.on_motion(MotionFrame(run.req_id, "END", 200, 66700, 66700, "NONE"), sent_at + 0.3)
                page._on_result(run.req_id, "XMOVE", "DONE")
                saved = json.loads(experiment_page.DATA_PATH.read_text(encoding="utf-8"))
                self.assertEqual(len(saved), 1)
                self.assertEqual(saved[0]["sample_count"], 2)
                self.assertAlmostEqual(saved[0]["axis_stats"]["x"]["peak_abs_g"], 0.4)
                self.assertAlmostEqual(saved[0]["axis_stats"]["x"]["mean_g"], 0.1)
                self.assertAlmostEqual(saved[0]["axis_stats"]["y"]["mean_g"], 0.35)
                self.assertAlmostEqual(saved[0]["abs_axis_sum_stats"]["peak_g"], 2.0)
                self.assertAlmostEqual(saved[0]["abs_axis_sum_stats"]["mean_g"], 1.5)
                self.assertEqual(page.table.item(0, 7).text(), "2.000/1.500")
                self.assertIsNotNone(saved[0]["duration_s"])
                page._delete(saved[0]["id"])
                self.assertEqual(json.loads(experiment_page.DATA_PATH.read_text(encoding="utf-8")), [])
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_return_moves_without_collecting_or_saving_experiment_data(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=66700,
                                          target_mdeg=66700)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                self.assertEqual(page.return_button.text(), "回收到折叠位")
                page._start("回收")
                run = page.active
                self.assertIsNotNone(run)
                self.assertFalse(run.record)
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},160000,125,100\n".encode())
                sent_at = time.monotonic()
                page.on_wire_written(payload, True, sent_at)
                page.on_acc(AccFrame(1, 50, AccelerationSample(1.0, -0.2, 0.8, 1.0,
                                                               "g", "test", "MCU")), sent_at + 0.1)
                self.assertEqual(run.samples, [])
                page.on_motion(MotionFrame(run.req_id, "END", 200, 160000, 160000, "NONE"), sent_at + 0.3)
                page._on_result(run.req_id, "XMOVE", "DONE")
                self.assertIsNone(page.active)
                self.assertEqual(page.records, [])
                self.assertFalse(experiment_page.DATA_PATH.exists())
                self.assertIn("未写入实验记录", page.status.text())
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_experiment_near_target_settle_uses_stop_and_marks_record_non_exact(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        controller.latest_state = replace(controller.latest_state, pos_mdeg=160000,
                                          target_mdeg=160000)
        (ROOT / "runtime").mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=ROOT / "runtime") as directory, patch.object(
                experiment_page, "DATA_PATH", Path(directory) / "records.json"):
            page = experiment_page.ExperimentPage(controller)
            try:
                page.travel.setCurrentIndex(2)  # Long target +20°, actual fold-to-target sweep is 140°.
                page._start("出臂")
                run = page.active
                self.assertIsNotNone(run)
                payload = [p for p, _ in link.sent if p.startswith(b"XMOVE,")][-1]
                self.assertEqual(payload, f"XMOVE,{run.req_id},20000,125,100\n".encode())
                page.on_wire_written(payload, True, time.monotonic())
                controller.handle_frame(AckFrame(run.req_id, "XMOVE", "ACCEPTED"))
                base = controller.latest_state.t_ms
                # A stop far from target must never count as arrival.
                for offset in (250, 750, 1500):
                    controller.handle_frame(replace(controller.latest_state, t_ms=base+offset,
                        motion="MOVING", pos_mdeg=50000, target_mdeg=20000))
                self.assertFalse(any(p.startswith(b"STOP,") for p, _ in link.sent))
                for offset in (1800, 2200, 2850):
                    controller.handle_frame(replace(controller.latest_state, t_ms=base+offset,
                        motion="MOVING", pos_mdeg=22800, target_mdeg=20000))
                stop = [p for p, _ in link.sent if p.startswith(b"STOP,")][-1]
                stop_req = int(stop.split(b",")[1])
                controller.handle_frame(AckFrame(stop_req, "STOP", "APPLIED"))
                self.assertIsNotNone(page.active)  # ACK alone is not completion.
                query_id = controller._move_settle.stop_state_query_id
                controller.handle_frame(replace(controller.latest_state, t_ms=base+2900,
                    motion="IDLE", pos_mdeg=22800, target_mdeg=20000, reply_to=query_id,
                    fault_code="NONE"))
                self.assertIsNone(page.active)
                self.assertEqual(page.records[-1]["terminal"], "SETTLED")
                self.assertFalse(page.records[-1]["comparable"])
                self.assertEqual(page.records[-1]["end_pos_mdeg"], 22800)
                self.assertIn("2.8°", page.records[-1]["result"])
            finally:
                page._timer.stop()
                controller.shutdown()

    def test_fault_recovery_waits_for_fresh_state_without_motion(self):
        controller, link = online_controller()
        controller.experiment_info = ExperimentInfoFrame(1, 1, 20, 2000)
        page = experiment_page.ExperimentPage(controller)
        try:
            fault = replace(controller.latest_state, motion="FAULT", fault_code="MOTION_TIMEOUT",
                            pos_mdeg=22800, target_mdeg=20000)
            controller.handle_frame(fault)
            self.assertTrue(page.recover_button.isEnabled())
            before = len(link.sent)
            page.recover_button.click()
            stop = link.sent[-1][0]
            self.assertTrue(stop.startswith(b"STOP,"))
            self.assertFalse(page.recover_button.isEnabled())
            controller.handle_frame(AckFrame(int(stop.split(b",")[1]), "STOP", "APPLIED"))
            self.assertIn("等待新鲜 STATE", page.status.text())
            query_id = controller._query_ids["STATE"]
            controller.handle_frame(replace(fault, t_ms=fault.t_ms+100, motion="IDLE",
                                            fault_code="NONE", reply_to=query_id))
            self.assertIn("故障恢复已确认", page.status.text())
            self.assertFalse(page.recover_button.isEnabled())
            self.assertTrue(all(p.startswith((b"STOP,", b"GET,")) for p, _ in link.sent[before:]))
        finally:
            page._timer.stop()
            controller.shutdown()


if __name__ == "__main__":
    unittest.main()

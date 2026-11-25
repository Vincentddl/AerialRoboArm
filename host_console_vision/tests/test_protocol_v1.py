from __future__ import annotations

import os
import base64
from dataclasses import replace
import struct
import sys
import time
import unittest
import zlib
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vendor"))

from PySide6.QtCore import QObject, Signal
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from arm_console.models import ConnectionState
from arm_console.pages import ManualPage, VisionGrabPage
from arm_console.v1_protocol import (
    AccFrame, AckFrame, CalFrame, DoneFrame, ErrFrame, InfoFrame, InvalidFrame,
    Hx8DiagFrame, ImuDiagFrame, ProtocolError, StateFrame, MotionFrame, V1Decoder, V1Encoder,
)
from arm_console.v1_state import ExperimentController, Pending

app = QApplication.instance() or QApplication([])


class FakeLink(QObject):
    state_changed = Signal(object)
    write_result = Signal(bytes, bool, str)

    def __init__(self):
        super().__init__()
        self.state = ConnectionState.OPEN
        self.sent: list[tuple[bytes, bool]] = []
        self.queue_discards = 0

    def send(self, payload: bytes, *, priority: bool = False) -> bool:
        self.sent.append((payload, priority))
        return True

    def discard_normal_queue(self) -> None:
        self.queue_discards += 1


def online_controller(folded: int | None = None) -> tuple[ExperimentController, FakeLink]:
    link = FakeLink()
    c = ExperimentController(link)
    c._on_connection(ConnectionState.OPEN)
    c.handle_frame(InfoFrame(c._query_ids["INFO"], 2, 73582, 113, 230400, 200, 20, 500, 100, 1000, 80, 300))
    c.handle_frame(CalFrame(c._query_ids["CAL"], 0, 0, -1, 90000, folded, 135000, 90000, 0, 180000, 0, 0, 4000))
    c.handle_frame(StateFrame(152340, "MANUAL", "UNKNOWN", "IDLE", True, True,
                              -29700, -30000, 11980, 200, True, True, 120000,
                              "ANGLE_CMD", 113, "NONE", c._query_ids["STATE"]))
    c._pending[112] = Pending(112, "HOME", b"HOME,112\n", "准备", time.monotonic())
    c.handle_frame(MotionFrame(112, "READY", 152340, 0, 0, "NONE"))
    c.handle_frame(DoneFrame(112, "HOME", 0))
    return c, link


def finish_pick_and_refresh(c: ExperimentController, req: int, pos_mdeg: int) -> None:
    """MCU confirms its one-shot close and arrival; then a fresh STATE permits return."""
    c.handle_frame(AckFrame(req, "PICK", "ACCEPTED"))
    c.handle_frame(MotionFrame(req, "START", 152440, pos_mdeg, 90000, "NONE"))
    c.handle_frame(MotionFrame(req, "CLOSE_APPLIED", 152600, pos_mdeg, pos_mdeg + 4000, "NONE"))
    c.handle_frame(DoneFrame(req, "PICK", pos_mdeg))
    deadline = time.monotonic() + 0.45
    while time.monotonic() < deadline:
        QTest.qWait(25)
    c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                           motion="IDLE", pos_mdeg=pos_mdeg, grip_cmd_mdeg=90000,
                           last_req_id=req, reply_to=c._query_ids["STATE"]))


class V1ProtocolTests(unittest.TestCase):
    def test_capture_imu_stale_flag_is_decoded(self):
        payload = base64.b64encode(struct.pack("<Hhhh", 0, 1, -2, 998))
        for flags in (4, 6, 7):
            body = f"CAPDATA,42,43,0,1,0,100,101,{flags},".encode() + payload
            wire = body + f",{zlib.crc32(body):08x}\n".encode()
            self.assertEqual(V1Decoder().feed(wire, 1.0)[0].flags, flags)
        body = b"CAPDATA,42,43,0,1,0,100,101,8," + payload
        wire = body + f",{zlib.crc32(body):08x}\n".encode()
        self.assertIsInstance(V1Decoder().feed(wire, 1.0)[0], InvalidFrame)

    def test_experiment_capability_query_retries_after_lost_reply(self):
        c, link = online_controller()
        try:
            c.experiment_info = None
            c._last_handshake_retry = 0
            link.sent.clear()
            c._tick()
            self.assertTrue(any(payload.startswith(b"GET,") and payload.endswith(b",EXPERIMENT\n")
                                for payload, _priority in link.sent))
        finally:
            c.shutdown()

    def test_imu_diagnostic_query_and_reply(self):
        self.assertEqual(V1Encoder.encode("GET", 42, "IMU"), b"GET,42,IMU\n")
        frames = V1Decoder().feed(b"IMU,42,1234,600,2,1,0,1,0\n", 1.0)
        self.assertEqual(frames, [ImuDiagFrame(42, 1234, 600, 2, 1, 0, 1, 0)])

    def test_imu_diagnostic_does_not_infer_no_uart_bytes_from_dma_counter(self):
        c = ExperimentController(FakeLink())
        messages: list[str] = []
        c.event.connect(lambda _kind, title, message: messages.append(message) if title == "IMU诊断" else None)
        try:
            c._query_ids["IMU"] = 42
            c.handle_frame(ImuDiagFrame(42, 0, 0, 0, 100, 0, 1, 0))
            c.handle_frame(ImuDiagFrame(42, 0, 21, 0, 120, 0, 1, 0))
            self.assertIn("收到新增有效 IMU 帧", messages[-1])
            self.assertIn("DMA半缓冲累计字节=0", messages[-1])
            self.assertNotIn("没有收到新字节", messages[-1])
        finally:
            c.shutdown()

    def test_high_speed_pick_abort_releases_manual_controls_without_late_actions(self):
        c, link = online_controller()
        manual = ManualPage(c)
        flow = VisionGrabPage(c)
        manual.grab_flow = flow
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=160000, target_mdeg=160000,
                                     grip_cmd_mdeg=125000)
            manual.speed_spin.setValue(80)
            manual.angle_spin.setValue(74)
            manual.grab_button.click()
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            req = int(pick.split(b",")[1])
            self.assertTrue(pick.endswith(b",20000,390,4000\n"))
            c.handle_frame(AckFrame(req, "PICK", "ACCEPTED"))
            c.handle_frame(MotionFrame(req, "ABORT", 152500, -4000, 74200, "FEEDBACK_STALE"))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="FAULT", fault_code="FEEDBACK_STALE", pos_mdeg=74200))
            QTest.qWait(30)
            self.assertFalse(flow.running)
            self.assertTrue(manual.start_button.isEnabled())
            self.assertTrue(manual.recover_button.isEnabled())
            c.handle_frame(ErrFrame(req, "FEEDBACK_STALE"))
            self.assertFalse(any(p.startswith((b"MOVE,", b"SERVO,", b"STOP,")) for p, _ in link.sent))
            self.assertTrue(manual.recover_button.isEnabled())
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_pick_state_fault_without_abort_also_releases_flow(self):
        c, link = online_controller()
        manual = ManualPage(c)
        flow = VisionGrabPage(c)
        manual.grab_flow = flow
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            self.assertTrue(flow.start_manual_grab(85, 74))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="FAULT", fault_code="FEEDBACK_STALE", pos_mdeg=24000))
            QTest.qWait(30)
            self.assertFalse(flow.running)
            self.assertTrue(manual.recover_button.isEnabled())
            self.assertFalse(any(p.startswith((b"MOVE,", b"SERVO,")) for p, _ in link.sent))
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_fold_move_abort_releases_linked_action_without_followup(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_fold())
            opened = [p for p, _ in link.sent if p.startswith(b"SERVO,")][-1]
            c.handle_frame(AckFrame(int(opened.split(b",")[1]), "SERVO", "APPLIED"))
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            c.handle_frame(MotionFrame(req, "ABORT", 152500, 160000, 74200, "FEEDBACK_STALE"))
            self.assertIsNone(c._linked_action)
            self.assertFalse(c.control_busy)
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"MOVE,")]), 1)
        finally:
            c.shutdown()

    def test_hx8_diagnostic_decodes_exact_failure_cause(self):
        frames = V1Decoder().feed(b"HX8_DIAG,152500,20,74200,8750,1200,4,VOLT_LOW\n", 1.0)
        self.assertEqual(frames, [Hx8DiagFrame(152500, 20, 74200, 8750, 1200, 4, "VOLT_LOW")])

    def test_power_protection_diagnostic_is_not_reported_as_missing_feedback(self):
        c, _ = online_controller()
        events = []
        c.event.connect(lambda source, title, detail: events.append((source, title, detail)))
        try:
            c.handle_frame(Hx8DiagFrame(133781, 0, 33800, 10799, 1134, 65, "STATUS"))
            self.assertTrue(any("HX8功率保护触发" in detail for _, _, detail in events))
            self.assertTrue(any("反馈龄 0 ms" in detail for _, _, detail in events))
        finally:
            c.shutdown()

    def test_manual_grab_uses_parameter_snapshot_and_ten_percent_fold_return(self):
        c, link = online_controller()
        manual = ManualPage(c)
        flow = VisionGrabPage(c)
        manual.grab_flow = flow
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            manual.speed_spin.setValue(20)
            manual.angle_spin.setValue(30)
            manual.grab_button.click()
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            self.assertTrue(pick.endswith(b",60000,100,4000\n"))
            count = len([p for p, _ in link.sent if p.startswith(b"PICK,")])
            manual.grab_button.click()
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"PICK,")]), count)
            manual.speed_spin.setValue(80)
            manual.angle_spin.setValue(50)
            req = int(pick.split(b",")[1])
            finish_pick_and_refresh(c, req, 60000)
            deadline = time.monotonic() + 1.0
            while not any(p.startswith(b"MOVE,") for p, _ in link.sent) and time.monotonic() < deadline:
                QTest.qWait(20)
            returned = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            self.assertTrue(returned.endswith(b",160000,50\n"))
            self.assertFalse(any(p.startswith((b"SERVO,", b"STOP,")) for p, _ in link.sent))
            return_req = int(returned.split(b",")[1])
            c.handle_frame(AckFrame(return_req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(return_req, "MOVE", 160000))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="IDLE", pos_mdeg=160000, target_mdeg=160000,
                                   last_req_id=return_req))
            self.assertFalse(flow.running)
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_manual_pick_returns_after_closed_arm_stays_within_ten_degrees_for_one_second(self):
        c, link = online_controller()
        flow = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            self.assertTrue(flow.start_manual_grab(20, 30))
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            pick_req = int(pick.split(b",")[1])
            c.handle_frame(AckFrame(pick_req, "PICK", "ACCEPTED"))
            c.handle_frame(MotionFrame(pick_req, "CLOSE_APPLIED", 152400, 60000, 69800, "NONE"))

            for t_ms, pos in ((152600, 69800), (153100, 69900)):
                c.handle_frame(replace(c.latest_state, t_ms=t_ms, motion="MOVING",
                                       pos_mdeg=pos, target_mdeg=60000,
                                       grip_cmd_mdeg=90000, last_req_id=pick_req))
            self.assertFalse(any(p.startswith(b"STOP,") for p, _ in link.sent))
            c.handle_frame(replace(c.latest_state, t_ms=153650, pos_mdeg=69900))
            stops = [p for p, _ in link.sent if p.startswith(b"STOP,")]
            self.assertEqual(len(stops), 1)
            self.assertFalse(any(p.startswith(b"MOVE,") for p, _ in link.sent))

            stop_req = int(stops[0].split(b",")[1].strip())
            c.handle_frame(AckFrame(stop_req, "STOP", "APPLIED"))
            c.handle_frame(replace(c.latest_state, t_ms=153800, motion="IDLE",
                                   pos_mdeg=69900, last_req_id=stop_req,
                                   reply_to=c._query_ids["STATE"]))
            self.assertEqual(flow._phase, "return_wait", (
                c.latest_state, c._pending, c._last_state_received_at,
                flow._settle_wait_since, flow._settle_stop_query_id, flow.flow_label.text()))
            QTest.qWait(500)
            c.handle_frame(replace(c.latest_state, t_ms=154400,
                                   reply_to=c._query_ids["STATE"]))
            moves = [p for p, _ in link.sent if p.startswith(b"MOVE,")]
            self.assertEqual(len(moves), 1)
            self.assertTrue(moves[0].endswith(b",160000,50\n"))
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_manual_pick_does_not_return_without_closed_pwm_or_near_position(self):
        c, link = online_controller()
        flow = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            self.assertTrue(flow.start_manual_grab(20, 30))
            pick_req = int([p for p, _ in link.sent if p.startswith(b"PICK,")][-1].split(b",")[1])
            c.handle_frame(AckFrame(pick_req, "PICK", "ACCEPTED"))
            for t_ms in (152600, 153100, 153650):
                c.handle_frame(replace(c.latest_state, t_ms=t_ms, motion="MOVING",
                                       pos_mdeg=70000, target_mdeg=60000,
                                       grip_cmd_mdeg=90000, last_req_id=pick_req))
            self.assertFalse(any(p.startswith((b"STOP,", b"MOVE,")) for p, _ in link.sent))
            c.handle_frame(MotionFrame(pick_req, "CLOSE_APPLIED", 153700, 60000, 70000, "NONE"))
            for t_ms in (153900, 154400, 155000):
                c.handle_frame(replace(c.latest_state, t_ms=t_ms, pos_mdeg=70500))
            self.assertFalse(any(p.startswith((b"STOP,", b"MOVE,")) for p, _ in link.sent))
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_manual_grab_from_fold_uses_same_target_close_and_return(self):
        c, link = online_controller()
        manual = ManualPage(c)
        flow = VisionGrabPage(c)
        manual.grab_flow = flow
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=160000, target_mdeg=160000,
                                     grip_cmd_mdeg=125000)
            manual.speed_spin.setValue(80)
            manual.angle_spin.setValue(70)
            manual.grab_button.click()
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            self.assertTrue(pick.endswith(b",20000,390,4000\n"))
            req = int(pick.split(b",")[1])
            finish_pick_and_refresh(c, req, 20000)
            returned = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            self.assertTrue(returned.endswith(b",160000,50\n"))
            self.assertFalse(any(p.startswith((b"SERVO,", b"STOP,")) for p, _ in link.sent))
            return_req = int(returned.split(b",")[1])
            c.handle_frame(AckFrame(return_req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(return_req, "MOVE", 160000))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="IDLE", pos_mdeg=160000, target_mdeg=160000,
                                   last_req_id=return_req))
            self.assertIn("折叠位置", flow.flow_label.text())
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_manual_grab_always_returns_to_fold_with_new_parameters(self):
        for base in (90000, 160000):
            with self.subTest(base=base):
                c, link = online_controller()
                manual = ManualPage(c)
                flow = VisionGrabPage(c)
                manual.grab_flow = flow
                try:
                    c.latest_state = replace(c.latest_state, pos_mdeg=base,
                                             target_mdeg=base, grip_cmd_mdeg=125000)
                    self.assertTrue(flow.start_manual_grab(20, 30))
                    first = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
                    first_req = int(first.split(b",")[1])
                    finish_pick_and_refresh(c, first_req, 60000)
                    deadline = time.monotonic() + 1.0
                    while not any(p.startswith(b"MOVE,") for p, _ in link.sent) and time.monotonic() < deadline:
                        QTest.qWait(20)
                    returned = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
                    self.assertTrue(returned.endswith(b",160000,50\n"))
                    return_req = int(returned.split(b",")[1])
                    c.handle_frame(AckFrame(return_req, "MOVE", "ACCEPTED"))
                    c.handle_frame(DoneFrame(return_req, "MOVE", 160000))
                    self.assertTrue(flow.running)  # A cached pre-return STATE cannot unlock PICK.
                    c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                           motion="IDLE", pos_mdeg=160000, target_mdeg=160000,
                                           grip_cmd_mdeg=90000, last_req_id=return_req))
                    self.assertFalse(flow.running)

                    # The second press snapshots new settings and must not
                    # send PICK until the idle SERVO OPEN has been acknowledged.
                    self.assertTrue(flow.start_manual_grab(40, 35))
                    open_frame = [p for p, _ in link.sent if p.startswith(b"SERVO,")][-1]
                    self.assertTrue(open_frame.endswith(b",1,125000\n"))
                    self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"PICK,")]), 1)
                    manual.speed_spin.setValue(70)
                    manual.angle_spin.setValue(50)
                    c.handle_frame(AckFrame(int(open_frame.split(b",")[1]), "SERVO", "APPLIED"))
                    second = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
                    self.assertTrue(second.endswith(b",55000,200,4000\n"))
                    self.assertEqual(flow._return_target_mdeg, 160000)
                    self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"MOVE,")]), 1)
                    finish_pick_and_refresh(c, int(second.split(b",")[1]), 55000)
                    deadline = time.monotonic() + 1.0
                    while len([p for p, _ in link.sent if p.startswith(b"MOVE,")]) < 2 and time.monotonic() < deadline:
                        QTest.qWait(20)
                    second_return = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
                    self.assertTrue(second_return.endswith(b",160000,50\n"))
                finally:
                    flow.stop_flow(send_stop=False)
                    c.shutdown()

    def test_manual_grab_open_rejection_never_starts_extension(self):
        c, link = online_controller()
        flow = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000,
                                     target_mdeg=90000, grip_cmd_mdeg=90000)
            self.assertTrue(flow.start_manual_grab(20, 30))
            opened = [p for p, _ in link.sent if p.startswith(b"SERVO,")][-1]
            c.handle_frame(ErrFrame(int(opened.split(b",")[1]), "NOT_READY"))
            self.assertFalse(flow.running)
            self.assertFalse(any(p.startswith(b"PICK,") for p, _ in link.sent))
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_timed_manual_grab_cancels_close_after_feedback_fault(self):
        c, link = online_controller()
        manual = ManualPage(c)
        flow = VisionGrabPage(c)
        manual.grab_flow = flow
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            manual.speed_spin.setValue(20)
            manual.angle_spin.setValue(30)
            manual.grab_button.click()
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            req = int(pick.split(b",")[1])
            c.handle_frame(AckFrame(req, "PICK", "ACCEPTED"))
            c.handle_frame(MotionFrame(req, "START", 152440, 40000, 90000, "NONE"))
            c.handle_frame(ErrFrame(req, "FEEDBACK_STALE"))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="FAULT", fault_code="FEEDBACK_STALE", pos_mdeg=35000))
            QTest.qWait(500)
            self.assertFalse(flow.running)
            self.assertFalse(any(p.startswith((b"SERVO,", b"STOP,", b"MOVE,")) for p, _ in link.sent))
        finally:
            flow.stop_flow(send_stop=False)
            c.shutdown()

    def test_visual_return_waits_for_stop_readiness_and_retries_only_rejected_move(self):
        c, link = online_controller()
        page = VisionGrabPage(c)
        try:
            page.running = True
            page._phase = "close"
            page._return_target_mdeg = 90_000  # The test enters mid-grab; supply its start snapshot.
            close_req = c._id()
            page._active_req = close_req
            c._pending[close_req] = Pending(close_req, "SERVO",
                                            f"SERVO,{close_req},1,90000\n".encode(),
                                            "闭爪", time.monotonic())
            c.handle_frame(AckFrame(close_req, "SERVO", "APPLIED"))
            self.assertEqual(page._phase, "return_wait")
            self.assertFalse(any(payload.startswith(b"MOVE,") for payload, _ in link.sent))

            def fresh_idle_state():
                c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                       motion="IDLE", pos_mdeg=-17700,
                                       grip_cmd_mdeg=90000, last_req_id=close_req,
                                       reply_to=c._query_ids["STATE"]))

            fresh_idle_state()  # Too early: STOP cleared the MCU stability gate.
            self.assertFalse(any(payload.startswith(b"MOVE,") for payload, _ in link.sent))
            QTest.qWait(500)
            fresh_idle_state()
            moves = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")]
            self.assertEqual(len(moves), 1)
            first_req = int(moves[-1].split(b",")[1])
            self.assertEqual(page._phase, "return")

            c.handle_frame(ErrFrame(first_req, "NOT_READY"))
            self.assertEqual(page._phase, "return_wait")
            fresh_idle_state()
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"MOVE,")]), 1)
            QTest.qWait(500)
            fresh_idle_state()
            moves = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")]
            self.assertEqual(len(moves), 2)
            second_req = int(moves[-1].split(b",")[1])
            c.handle_frame(AckFrame(second_req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(second_req, "MOVE", 89800))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="IDLE", pos_mdeg=89800, target_mdeg=90000,
                                   last_req_id=second_req))
            self.assertEqual(page._phase, "finished")
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"MOVE,")]), 2)
        finally:
            page.stop_flow(send_stop=False)
            page.deleteLater()
            c.shutdown()

    def test_manual_move_and_30_degree_servo_direction_check(self):
        c, link = online_controller()
        try:
            c.cal = replace(c.cal, open_wait_ms=None, close_wait_ms=None)
            c.ready_for_pick = False
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(c.request_extend(20, 30))
            move = link.sent[-1][0]
            self.assertTrue(move.startswith(b"MOVE,"))
            self.assertTrue(move.endswith(b",60000,100\n"))
            req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(req, "MOVE", 100))
            self.assertTrue(c.request_grip("OPEN"))
            opened = link.sent[-1][0]
            self.assertTrue(opened.startswith(b"SERVO,"))
            self.assertTrue(opened.endswith(b",1,125000\n"))
            c.handle_frame(AckFrame(int(opened.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertTrue(c.request_grip("CLOSE"))
            closed = link.sent[-1][0]
            self.assertTrue(closed.endswith(b",1,90000\n"))
            c.handle_frame(AckFrame(int(closed.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertFalse(c.grip_ramp_active)
            sent_count = len(link.sent)
            c.mode = "VISION_PICK"
            self.assertFalse(c.request_grip("OPEN"))
            self.assertEqual(len(link.sent), sent_count)
        finally:
            c.shutdown()

    def test_folded_and_normal_starts_share_absolute_target_and_full_speed(self):
        c, link = online_controller()
        try:
            targets = []
            for start in (90000, 160000):
                c.latest_state = replace(c.latest_state, pos_mdeg=start, target_mdeg=start,
                                         motion="IDLE", fault_code="NONE")
                self.assertTrue(c.request_extend(100, 74))
                payload = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
                self.assertTrue(payload.endswith(b",16000,500\n"))
                targets.append(payload.split(b",")[2:])
                req = int(payload.split(b",")[1])
                c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
                c.handle_frame(DoneFrame(req, "MOVE", 16000))
            self.assertEqual(targets[0], targets[1])
            for start in (90000, 160000):
                c.latest_state = replace(c.latest_state, pos_mdeg=start, target_mdeg=start,
                                         motion="IDLE", fault_code="NONE")
                self.assertTrue(c.request_extend(80, 70))
                payload = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
                self.assertTrue(payload.endswith(b",20000,400\n"))
                req = int(payload.split(b",")[1])
                c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
                c.handle_frame(DoneFrame(req, "MOVE", 20000))
            c.latest_state = replace(c.latest_state, pos_mdeg=62000)
            self.assertFalse(c.request_extend(100, 74))
        finally:
            c.shutdown()

    def test_manual_move_requires_feedback_near_visual_working_position(self):
        c, link = online_controller()
        try:
            before = len(link.sent)
            self.assertFalse(c.request_extend(20, 30))
            self.assertEqual(len(link.sent), before)
            self.assertIn("请先点击正常模式或归零", c.command_status)
            c.latest_state = replace(c.latest_state, pos_mdeg=92500, target_mdeg=90000)
            self.assertTrue(c.request_extend(20, 30))
            self.assertTrue(link.sent[-1][0].endswith(b",60000,100\n"))
        finally:
            c.shutdown()

    def test_first_grip_click_sends_direct_pwm_even_without_prior_command(self):
        c, link = online_controller()
        try:
            c.latest_state = replace(c.latest_state, grip_cmd_mdeg=None)
            before = len(link.sent)
            self.assertTrue(c.request_grip("OPEN"))
            self.assertEqual(len(link.sent), before + 1)
            self.assertTrue(link.sent[-1][0].endswith(b",1,125000\n"))
            self.assertFalse(c.grip_ramp_active)
            self.assertIn("等待设备ACK", c.command_status)
        finally:
            c.shutdown()

    def test_install_position_is_explicit_90_degree_command(self):
        c, link = online_controller()
        try:
            c.latest_state = replace(c.latest_state, grip_cmd_mdeg=None)
            before = len(link.sent)
            self.assertTrue(c.request_grip_install_position())
            self.assertEqual(len(link.sent), before + 1)
            baseline = link.sent[-1][0]
            self.assertTrue(baseline.endswith(b",1,90000\n"))
            c.handle_frame(AckFrame(int(baseline.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertTrue(c.request_grip("OPEN"))
            self.assertTrue(link.sent[-1][0].endswith(b",1,125000\n"))
        finally:
            c.shutdown()

    def test_direct_servo_rejection_and_stop_do_not_generate_followup_steps(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_grip("CLOSE"))
            first = link.sent[-1][0]
            self.assertTrue(first.endswith(b",1,90000\n"))
            c.handle_frame(ErrFrame(int(first.split(b",")[1]), "BUSY"))
            self.assertFalse(c.grip_ramp_active)
            self.assertFalse(c._grip_timer.isActive())

            self.assertTrue(c.request_grip("CLOSE"))
            second = link.sent[-1][0]
            self.assertTrue(second.endswith(b",1,90000\n"))
            c.request_stop()
            self.assertFalse(c.grip_ramp_active)
            c.handle_frame(AckFrame(int(second.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertFalse(c._grip_timer.isActive())
        finally:
            c.shutdown()

    def test_leaving_manual_queues_mode_after_inflight_servo(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_grip("CLOSE"))
            first = link.sent[-1][0]
            self.assertTrue(first.endswith(b",1,90000\n"))
            self.assertTrue(c.request_mode("VISION_DEMO"))
            self.assertFalse(c.grip_ramp_active)
            self.assertEqual(c._queued_mode, "VISION_DEMO")
            c.handle_frame(AckFrame(int(first.split(b",")[1]), "SERVO", "APPLIED"))
            c._tick()
            self.assertTrue(link.sent[-1][0].startswith(b"MODE,"))
            self.assertEqual(len([x for x, _ in link.sent if x.startswith(b"SERVO,")]), 1)
        finally:
            c.shutdown()

    def test_split_coalesced_crlf_acc_and_device_time(self):
        d = V1Decoder()
        self.assertEqual(d.feed(b"ACC,128,152", 1.0), [])
        frames = d.feed(b"340,85,-20,998\r\nACC,129,152390,90,-10,1000\n", 1.2)
        self.assertEqual(len(frames), 2)
        self.assertTrue(all(isinstance(f, AccFrame) for f in frames))
        self.assertEqual(frames[0].seq, 128)
        self.assertEqual(frames[0].sample.time_s, 152.34)
        self.assertEqual((frames[0].sample.ax, frames[0].sample.ay, frames[0].sample.az), (0.085, -0.02, 0.998))
        self.assertIn("含重力", frames[0].sample.unit)

    def test_invalid_and_overlong_line_does_not_execute_following_fragment(self):
        d = V1Decoder()
        frames = d.feed(b"ACC,1,20,0,0,16001\nACK,1,MOVE,ACCEPTED,EXTRA\n", 1)
        self.assertTrue(all(isinstance(f, InvalidFrame) for f in frames))
        frames = d.feed(b"A" * 200 + b"\nACC,2,40,1,2,3\n", 2)
        self.assertIsInstance(frames[0], InvalidFrame)
        self.assertEqual(sum(isinstance(f, AccFrame) for f in frames), 1)
        self.assertIsInstance(d.feed(b"ACC,3,50,0,,0\n", 3)[0], InvalidFrame)
        self.assertIsInstance(d.feed(b"ACC,3,50,0,0,0 \n", 3)[0], InvalidFrame)

    def test_encoder_range_and_exact_wire_format(self):
        self.assertEqual(V1Encoder.encode("MOVE", 101, -30000, 250), b"MOVE,101,-30000,250\n")
        self.assertEqual(V1Encoder.encode("GET", 401, "CAL"), b"GET,401,CAL\n")
        with self.assertRaises(ProtocolError):
            V1Encoder.encode("MOVE", 1, -30050, 250)
        with self.assertRaises(ProtocolError):
            V1Encoder.encode("MOVE", 1, -30000, 0)
        with self.assertRaises(ProtocolError):
            V1Encoder.encode("SERVO", 1, 2, 120000)

    def test_global_position_presets_and_manual_move_conversion(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.ready_for_pick)
            before = len(link.sent)
            self.assertTrue(c.request_fold())
            opened = link.sent[-1][0]
            self.assertTrue(opened.startswith(b"SERVO,"))
            self.assertTrue(opened.endswith(b",1,125000\n"))
            open_req = int(opened.split(b",")[1])
            c.handle_frame(AckFrame(open_req, "SERVO", "APPLIED"))
            fold = link.sent[-1][0]
            self.assertTrue(fold.startswith(b"MOVE,"))
            self.assertTrue(fold.endswith(b",160000,200\n"))
            self.assertTrue(all(not payload.startswith((b"SHAPE,", b"GRIP,"))
                                for payload, _ in link.sent[before:]))
            req = int(fold.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            self.assertIn("折叠运动中", c.requested_shape)
            c.handle_frame(DoneFrame(req, "MOVE", 160000))
            self.assertIn("折叠到位", c.requested_shape)
            self.assertFalse(c.ready_for_pick)
            self.assertTrue(c.request_restore())
            normal = link.sent[-1][0]
            self.assertTrue(normal.startswith(b"MOVE,"))
            self.assertTrue(normal.endswith(b",90000,300\n"))
            req = int(normal.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(req, "MOVE", 90000))
            closed_after_normal = link.sent[-1][0]
            self.assertTrue(closed_after_normal.startswith(b"SERVO,"))
            self.assertTrue(closed_after_normal.endswith(b",1,125000\n"))
            c.handle_frame(AckFrame(int(closed_after_normal.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertIn("正常模式完成", c.command_status)
            self.assertFalse(c.ready_for_pick)  # Position-only MOVE is not SHAPE READY.
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertFalse(c.request_extend(0, 30))
            self.assertFalse(c.request_extend(50, 74.1))
            self.assertTrue(c.request_extend(50, 30))
            move = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")][-1]
            self.assertTrue(move.endswith(b",60000,250\n"))
            req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            self.assertIn("等待实际反馈", c.command_status)
            self.assertIn(req, c._pending)
            c.handle_frame(DoneFrame(req, "MOVE", 10300))
            self.assertNotIn(req, c._pending)
            self.assertIn("DONE", c.command_status)
            self.assertTrue(c.request_home())
            home = link.sent[-1][0]
            self.assertTrue(home.startswith(b"MOVE,") and home.endswith(b",90000,300\n"))
            home_req = int(home.split(b",")[1])
            c.handle_frame(AckFrame(home_req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(home_req, "MOVE", 90000))
            home_close = link.sent[-1][0]
            self.assertTrue(home_close.endswith(b",1,125000\n"))
            c.handle_frame(AckFrame(int(home_close.split(b",")[1]), "SERVO", "APPLIED"))
            self.assertIn("归零完成", c.command_status)
        finally:
            c.shutdown()

    def test_global_position_presets_preserve_motion_gates(self):
        c, link = online_controller()
        try:
            before = len(link.sent)
            c.handle_frame(replace(c.latest_state, motion="FAULT", fault_code="MOTION_TIMEOUT", t_ms=152341))
            self.assertFalse(c.request_fold())
            self.assertFalse(c.request_restore())
            self.assertEqual(len(link.sent), before)
            c.handle_frame(replace(c.latest_state, motion="IDLE", fault_code="NONE", t_ms=152342))
            c.cal = replace(c.cal, extension_sign=1)
            self.assertFalse(c.request_fold())
            self.assertEqual(len(link.sent), before)
            c.cal = replace(c.cal, extension_sign=-1, normal_mdeg=1000)
            self.assertFalse(c.request_restore())
            self.assertEqual(len(link.sent), before)
        finally:
            c.shutdown()

    def test_manual_ui_limits_to_70_while_protocol_retains_74_degree_guard(self):
        c, link = online_controller()
        try:
            manual = ManualPage(c)
            self.assertEqual(manual.angle_spin.maximum(), 70.0)
            self.assertEqual(manual.angle_slider.maximum(), 700)
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(c.request_extend(20, 74))
            self.assertTrue(link.sent[-1][0].endswith(b",16000,100\n"))
            sent_count = len(link.sent)
            self.assertFalse(c.request_extend(20, 74.1))
            self.assertEqual(len(link.sent), sent_count)
            self.assertIn("74°", c.command_status)
        finally:
            c.shutdown()

    def test_visual_regions_above_measured_limit_send_no_pick(self):
        from arm_console.models import BlackBlock, VisionResult

        for center_y in (150, 50):  # Regions 4 and 5 retain their old 90°/110° mappings.
            c, link = online_controller()
            page = VisionGrabPage(c)
            try:
                c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                         grip_cmd_mdeg=125000)
                page.start_flow()
                result = VisionResult(640, 500, None, "test", False,
                                      (BlackBlock(300, center_y - 20, 40, 40, 320, center_y, 1600),),
                                      1, True)
                for _ in range(page.STABLE_FRAMES):
                    page.observe(result)
                self.assertFalse(any(p.startswith((b"PICK,", b"MOVE,")) for p, _ in link.sent))
                self.assertIn("超过实测74°上限", page.flow_label.text())
            finally:
                page.stop_flow(send_stop=False)
                c.shutdown()

    def test_visual_region_once_move_close_return_open(self):
        from arm_console.models import BlackBlock, VisionResult
        c, link = online_controller()
        page = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            page.speed_spin.setValue(40)
            page.start_flow()
            block = BlackBlock(300, 300, 40, 40, 320, 320, 1600)
            result = VisionResult(640, 500, None, "test", False, (block,), 1, True)
            for _ in range(page.STABLE_FRAMES - 1):
                page.observe(result)
            self.assertFalse(any(data.startswith(b"PICK,") for data, _ in link.sent))
            page.observe(result)
            pick = [data for data, _ in link.sent if data.startswith(b"PICK,")][-1]
            self.assertTrue(pick.endswith(b",40000,200,4000\n"))
            page.observe(result)
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"PICK,")]), 1)
            req = int(pick.split(b",")[1])
            finish_pick_and_refresh(c, req, 40000)
            home = [data for data, _ in link.sent if data.startswith(b"MOVE,")][-1]
            self.assertTrue(home.endswith(b",90000,300\n"))
            self.assertFalse(any(data.startswith((b"SERVO,", b"STOP,")) for data, _ in link.sent))
            home_req = int(home.split(b",")[1])
            c.handle_frame(AckFrame(home_req, "MOVE", "ACCEPTED"))
            c.handle_frame(DoneFrame(home_req, "MOVE", 90000))
            c.handle_frame(replace(c.latest_state, t_ms=c.latest_state.t_ms + 100,
                                   motion="IDLE", pos_mdeg=90000, target_mdeg=90000,
                                   last_req_id=home_req))
            self.assertFalse(page.running)
            self.assertIn("未知", page.result_label.text())
        finally:
            page.stop_flow(send_stop=False)
            c.shutdown()

    def test_visual_near_target_settles_then_stops_before_closing(self):
        from arm_console.models import BlackBlock, VisionResult
        c, link = online_controller()
        page = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            page.start_flow()
            result = VisionResult(640, 500, None, "test", False,
                                  (BlackBlock(300, 300, 40, 40, 320, 320, 1600),), 1, True)
            for _ in range(page.STABLE_FRAMES):
                page.observe(result)
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            req = int(pick.split(b",")[1])
            c.handle_frame(AckFrame(req, "PICK", "ACCEPTED"))
            for tick, pos in enumerate((2400, 2200, 2100), 1):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=pos, target_mdeg=0))
            self.assertEqual(page._phase, "pick_extend")
            self.assertFalse(any(p.startswith((b"STOP,", b"SERVO,", b"MOVE,")) for p, _ in link.sent))
        finally:
            page.stop_flow(send_stop=False)
            c.shutdown()

    def test_upper_valid_region_plateau_does_not_close_after_unconfirmed_stop(self):
        from arm_console.models import BlackBlock, VisionResult
        c, link = online_controller()
        page = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            page.start_flow()
            result = VisionResult(640, 500, None, "test", False,
                                  (BlackBlock(300, 230, 40, 40, 320, 250, 1600),), 1, True)
            for _ in range(page.STABLE_FRAMES):
                page.observe(result)
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            self.assertTrue(pick.endswith(b",20000,150,4000\n"))
            req = int(pick.split(b",")[1])
            c.handle_frame(AckFrame(req, "PICK", "ACCEPTED"))
            for tick, pos in enumerate((3900, 3800, 3800), 1):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=pos, target_mdeg=0))
            self.assertFalse(any(p.startswith((b"STOP,", b"SERVO,", b"MOVE,")) for p, _ in link.sent))
            c.handle_frame(ErrFrame(req, "MOTION_TIMEOUT"))
            self.assertFalse(page.running)
            self.assertFalse(any(p.startswith((b"SERVO,", b"MOVE,")) for p, _ in link.sent))
        finally:
            page.stop_flow(send_stop=False)
            c.shutdown()

    def test_linked_restore_near_target_stable_requires_stop_ack_and_fresh_idle(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_restore())
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(req, "MOVE", "ACCEPTED"))
            for tick, pos in enumerate((86200, 86200, 86200), 1):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=pos, target_mdeg=90000))
            stop = [p for p, _ in link.sent if p.startswith(b"STOP,")][-1]
            self.assertFalse(any(p.startswith(b"SERVO,") for p, _ in link.sent))
            c.handle_frame(AckFrame(int(stop.split(b",")[1]), "STOP", "APPLIED"))
            self.assertFalse(any(p.startswith(b"SERVO,") for p, _ in link.sent))
            c.handle_frame(replace(c.latest_state, t_ms=152950, motion="IDLE", pos_mdeg=86200))
            self.assertFalse(any(p.startswith(b"SERVO,") for p, _ in link.sent))
            c.handle_frame(replace(c.latest_state, t_ms=153000, motion="IDLE", pos_mdeg=86200,
                                   reply_to=c._query_ids["STATE"]))
            self.assertTrue(link.sent[-1][0].endswith(b",1,125000\n"))
        finally:
            c.shutdown()

    def test_far_stall_does_not_auto_stop_or_auto_recover(self):
        c, link = online_controller()
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(c.request_extend(30, 70))
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            c.handle_frame(AckFrame(int(move.split(b",")[1]), "MOVE", "ACCEPTED"))
            for tick in range(1, 5):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=10000, target_mdeg=0))
            self.assertFalse(any(p.startswith(b"STOP,") for p, _ in link.sent))
            self.assertFalse(any(p.startswith(b"SERVO,") for p, _ in link.sent))
        finally:
            c.shutdown()

    def test_slow_progress_near_target_is_not_mistaken_for_plateau(self):
        c, link = online_controller()
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(c.request_extend(30, 30))
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            c.handle_frame(AckFrame(int(move.split(b",")[1]), "MOVE", "ACCEPTED"))
            for tick, pos in enumerate((44800, 44600, 44400, 44200), 1):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=pos, target_mdeg=40000))
            self.assertFalse(any(p.startswith(b"STOP,") for p, _ in link.sent))
        finally:
            c.shutdown()

    def test_fold_near_target_holds_with_open_grip_applied(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_fold())
            opened = [p for p, _ in link.sent if p.startswith(b"SERVO,")][-1]
            self.assertTrue(opened.endswith(b",1,125000\n"))
            c.handle_frame(AckFrame(int(opened.split(b",")[1]), "SERVO", "APPLIED"))
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            c.handle_frame(AckFrame(int(move.split(b",")[1]), "MOVE", "ACCEPTED"))
            for tick in range(1, 4):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + tick * 200,
                                       motion="MOVING", pos_mdeg=156200, target_mdeg=160000))
            stop = [p for p, _ in link.sent if p.startswith(b"STOP,")][-1]
            c.handle_frame(AckFrame(int(stop.split(b",")[1]), "STOP", "APPLIED"))
            c.handle_frame(replace(c.latest_state, t_ms=153000, motion="IDLE", pos_mdeg=156200,
                                   reply_to=c._query_ids["STATE"]))
            self.assertIsNone(c._linked_action)
            self.assertEqual(len([p for p, _ in link.sent if p.startswith(b"SERVO,")]), 1)
            self.assertIn("近目标", c.requested_shape)
            self.assertIn("开爪", c.requested_shape)
        finally:
            c.shutdown()

    def test_upper_valid_region_close_then_return_from_negative_twenty_degrees(self):
        from arm_console.models import BlackBlock, VisionResult
        c, link = online_controller()
        page = VisionGrabPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000,
                                     grip_cmd_mdeg=125000)
            page.start_flow()
            result = VisionResult(640, 500, None, "test", False,
                                  (BlackBlock(300, 230, 40, 40, 320, 250, 1600),), 1, True)
            for _ in range(page.STABLE_FRAMES):
                page.observe(result)
            pick = [p for p, _ in link.sent if p.startswith(b"PICK,")][-1]
            self.assertTrue(pick.endswith(b",20000,150,4000\n"))
            finish_pick_and_refresh(c, int(pick.split(b",")[1]), 20000)
            returned = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            self.assertTrue(returned.endswith(b",90000,300\n"))
            self.assertFalse(any(p.startswith((b"SERVO,", b"STOP,")) for p, _ in link.sent))
        finally:
            page.stop_flow(send_stop=False)
            c.shutdown()

    def test_stop_cancels_fold_before_late_open_ack(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_fold())
            opened = link.sent[-1][0]
            open_req = int(opened.split(b",")[1])
            self.assertTrue(c.request_stop())
            move_count = len([payload for payload, _ in link.sent if payload.startswith(b"MOVE,")])
            c.handle_frame(AckFrame(open_req, "SERVO", "APPLIED"))
            self.assertEqual(len([payload for payload, _ in link.sent if payload.startswith(b"MOVE,")]), move_count)
            self.assertIsNone(c._linked_action)
        finally:
            c.shutdown()

    def test_ack_retry_reuses_req_and_stop_has_priority(self):
        c, link = online_controller()
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=90000, target_mdeg=90000)
            self.assertTrue(c.request_extend(50, 30))
            first = [payload for payload, _ in link.sent if payload.startswith(b"MOVE,")][-1]
            pending = next(iter(c._pending.values()))
            for _ in range(2):
                pending.sent_at -= 0.6
                c._tick()
            self.assertEqual([payload for payload, _ in link.sent if payload.startswith(b"MOVE,")], [first] * 3)
            pending.sent_at -= 0.6
            c._tick()
            self.assertTrue(pending.uncertain)
            self.assertIn("结果未确认", c.command_status)
            self.assertTrue(c.request_stop())
            self.assertTrue(any(payload.startswith(b"STOP,") and priority for payload, priority in link.sent))
        finally:
            c.shutdown()

    def test_liveness_timeout_blocks_new_motion(self):
        c, link = online_controller()
        try:
            c.last_liveness = time.monotonic() - 4
            c._tick()
            self.assertFalse(c.link_online)
            self.assertFalse(c.request_extend(50, 30))
            self.assertFalse(any(payload.startswith(b"MOVE,") for payload, _ in link.sent))
        finally:
            c.shutdown()

    def test_same_mode_reports_no_wire_write(self):
        c, link = online_controller()
        try:
            before = len(link.sent)
            self.assertTrue(c.request_mode("MANUAL"))
            self.assertEqual(len(link.sent), before)
            self.assertIn("未重复发送MODE", c.command_status)
        finally:
            c.shutdown()

    def test_visual_target_requires_confirmed_mode_and_active_flow(self):
        c, link = online_controller()
        try:
            c.cal = replace(c.cal, grip_open_mdeg=120000)
            self.assertFalse(c.submit_visual_target(-30, 250, 90))
            self.assertTrue(c.request_mode("VISION_PICK"))
            self.assertFalse(c.submit_visual_target(-30, 250, 90))
            mode = [p for p, _ in link.sent if p.startswith(b"MODE,")][-1]
            c.handle_frame(AckFrame(int(mode.split(b",")[1]), "MODE", "VISION_PICK"))
            self.assertFalse(c.submit_visual_target(-30, 250, 90))
            self.assertTrue(c.request_visual_prepare())
            prep = [p for p, _ in link.sent if p.startswith(b"PREP,")][-1]
            c.handle_frame(MotionFrame(int(prep.split(b",")[1]), "READY", 152450, 0, 0, "NONE"))
            c.set_visual_target_enabled(True)
            self.assertTrue(c.submit_visual_target(-30, 250, 90))
            self.assertEqual(link.sent[-1][0], b"V,-30000,250,90\n")
            self.assertTrue(c.request_stop())
            self.assertFalse(c.submit_visual_target(-30, 250, 90))
        finally:
            c.shutdown()

    def test_v2_single_visual_trigger_and_stop_ignores_late_event(self):
        c, link = online_controller()
        try:
            self.assertTrue(c.request_mode("VISION_PICK"))
            mode = [p for p, _ in link.sent if p.startswith(b"MODE,")][-1]
            c.handle_frame(AckFrame(int(mode.split(b",")[1]), "MODE", "VISION_PICK"))
            self.assertTrue(c.request_visual_prepare())
            prep = [p for p, _ in link.sent if p.startswith(b"PREP,")][-1]
            prep_id = int(prep.split(b",")[1])
            c.handle_frame(AckFrame(prep_id, "PREP", "ACCEPTED"))
            self.assertFalse(c.request_grab())
            c.handle_frame(MotionFrame(prep_id, "READY", 10, 0, 0, "NONE"))
            c.handle_frame(DoneFrame(prep_id, "PREP", 0))
            self.assertTrue(c.submit_visual_target(-30, 200, 90))
            self.assertTrue(c.submit_visual_target(-29, 200, 90))
            self.assertFalse(any(p.startswith(b"TRIGGER,") for p, _ in link.sent))
            self.assertTrue(c.request_grab())
            trigger = [p for p, _ in link.sent if p.startswith(b"TRIGGER,")]
            self.assertEqual(len(trigger), 1)
            self.assertFalse(c.request_grab())
            trigger_id = int(trigger[0].split(b",")[1])
            self.assertTrue(c.request_stop())
            c.handle_frame(MotionFrame(trigger_id, "CLOSE_APPLIED", 20, -29000, -25000, "NONE"))
            self.assertFalse(c.ready_for_pick)
            self.assertNotIn("闭爪命令已应用", c.execution_status)
        finally:
            c.shutdown()

    def test_lost_done_frame_reconciles_from_fresh_idle_state_and_unlocks_buttons(self):
        c, link = online_controller()
        manual = ManualPage(c)
        try:
            c.latest_state = replace(c.latest_state, pos_mdeg=60000, target_mdeg=60000)
            self.assertTrue(c.request_restore())
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            move_req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(move_req, "MOVE", "ACCEPTED"))
            self.assertFalse(manual.start_button.isEnabled())

            # DONE packet collides on 433 MHz air, then fresh STATE reports IDLE at +90.0 deg
            c.query("STATE")
            c.handle_frame(replace(c.latest_state, t_ms=153000, motion="IDLE",
                                   pos_mdeg=90000, target_mdeg=90000,
                                   last_req_id=move_req, fault_code="NONE",
                                   reply_to=c._query_ids["STATE"]))
            # Linked RESTORE automatically proceeds to SERVO OPEN, then unlocks all buttons!
            servos = [p for p, _ in link.sent if p.startswith(b"SERVO,")]
            self.assertEqual(len(servos), 1)
            servo_req = int(servos[0].split(b",")[1])
            c.handle_frame(AckFrame(servo_req, "SERVO", "APPLIED"))
            self.assertFalse(c.control_busy)
            self.assertTrue(manual.start_button.isEnabled())
            self.assertTrue(manual.open_button.isEnabled())
            self.assertIn("正常模式完成", c.command_status)
        finally:
            c.shutdown()

    def test_uncertain_command_clears_from_pending_on_fresh_idle_state_and_unlocks_buttons(self):
        c, link = online_controller()
        manual = ManualPage(c)
        try:
            self.assertTrue(c.request_grip("OPEN"))
            pending = next(iter(c._pending.values()))
            for _ in range(3):
                pending.sent_at -= 0.6
                c._tick()
            self.assertTrue(pending.uncertain)
            self.assertFalse(manual.open_button.isEnabled())

            # Fresh STATE reply arrives -> uncertain command is reconciled and buttons unlock!
            c.handle_frame(replace(c.latest_state, t_ms=153200, motion="IDLE",
                                   fault_code="NONE", reply_to=c._query_ids["STATE"]))
            self.assertEqual(c._pending, {})
            self.assertFalse(c.control_busy)
            self.assertTrue(manual.open_button.isEnabled())
            self.assertTrue(manual.start_button.isEnabled())
        finally:
            c.shutdown()

    def test_small_travel_move_settles_cleanly_under_hx8_gravity_droop_without_10s_timeout(self):
        c, link = online_controller()
        manual = ManualPage(c)
        try:
            # Arm at +86.5 deg -> click歸零 to +90.0 deg (3.5 deg travel <= 5.0 deg)
            c.latest_state = replace(c.latest_state, pos_mdeg=86500, target_mdeg=86500)
            self.assertTrue(c.request_home())
            move = [p for p, _ in link.sent if p.startswith(b"MOVE,")][-1]
            move_req = int(move.split(b",")[1])
            c.handle_frame(AckFrame(move_req, "MOVE", "ACCEPTED"))

            # HX8 settles 1.3 deg short (+88.7 deg) due to gravity (> 1.0 deg MCU gate, <= 2.5 deg small-move settle)
            for offset_ms in (200, 500, 900):
                c.handle_frame(replace(c.latest_state, t_ms=152340 + offset_ms,
                                       motion="MOVING", pos_mdeg=88700, target_mdeg=90000,
                                       last_req_id=move_req))
            stops = [p for p, _ in link.sent if p.startswith(b"STOP,")]
            self.assertEqual(len(stops), 1)
            stop_req = int(stops[0].split(b",")[1])
            c.handle_frame(AckFrame(stop_req, "STOP", "APPLIED"))
            c.handle_frame(replace(c.latest_state, t_ms=153400, motion="IDLE",
                                   pos_mdeg=88700, target_mdeg=90000,
                                   reply_to=c._query_ids["STATE"], fault_code="NONE"))
            servos = [p for p, _ in link.sent if p.startswith(b"SERVO,")]
            self.assertEqual(len(servos), 1)
            c.handle_frame(AckFrame(int(servos[0].split(b",")[1]), "SERVO", "APPLIED"))
            self.assertFalse(c.control_busy)
            self.assertTrue(manual.zero_button.isEnabled())
        finally:
            c.shutdown()

    def test_coprime_retry_backoff_avoids_50ms_acc_harmonic(self):
        for req_id in range(1, 40):
            d0 = round(ExperimentController._retry_backoff_s(req_id, 0) * 1000)
            d1 = round(ExperimentController._retry_backoff_s(req_id, 1) * 1000)
            self.assertLess(d0, 560)
            self.assertLess(d1, 560)
            # Ensure retry delays do not both land on exact 50 ms multiples
            self.assertFalse(d0 % 50 == 0 and d1 % 50 == 0)


if __name__ == "__main__":
    unittest.main()

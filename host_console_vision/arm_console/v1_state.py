"""V2 request matching, handshake, liveness and linked-action safety gates."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import secrets
import time

from PySide6.QtCore import QObject, QTimer, Signal

from .models import ConnectionState, MotionRequest
from .serial_service import SerialService
from .v1_protocol import (
    AckFrame, CalFrame, CaptureInfoFrame, DoneFrame, ErrFrame, ExperimentInfoFrame, Frame, Hx8DiagFrame, ImuDiagFrame, InfoFrame, MotionFrame,
    PongFrame, ProtocolError, StateFrame, U32, V1Encoder,
)


# Temporary manual direction check. These are requested PWM angles, not measured
# jaw positions or calibrated grasp endpoints. The new motor reverses the observed
# button directions; neither endpoint changes the MCU's automated GRIP presets.
GRIP_TEST_ANGLE_MDEG = {"OPEN": 125_000, "CLOSE": 90_000}
GRIP_INSTALL_ANGLE_MDEG = GRIP_TEST_ANGLE_MDEG["CLOSE"]
FOLD_TARGET_MDEG = 160_000
WORKING_ZERO_MDEG = 90_000
MANUAL_START_TOLERANCE_MDEG = 5_000
MANUAL_EXTENSION_LIMIT_MDEG = 74_000  # Existing measured travel setting; recheck after the working-position shift.
MAX_OPERATOR_SPEED_PERCENT = 78
EXPERIMENT_SPEED_LEVELS = (*range(5, MAX_OPERATOR_SPEED_PERCENT, 5), MAX_OPERATOR_SPEED_PERCENT)
GRIP_TEST_RAMP_STEP_MDEG = 2_000
GRIP_TEST_RAMP_INTERVAL_MS = 80
HOST_SETTLE_TOLERANCE_MDEG = 5_000  # Experimental acceptance band, not MCU accuracy.
HOST_SETTLE_DELTA_MDEG = 300
HOST_SETTLE_MIN_MS = 350
HOST_SETTLE_STATE_TIMEOUT_S = 2.0
EXPERIMENT_SETTLE_TOLERANCE_MDEG = 3_000
EXPERIMENT_SETTLE_MIN_MS = 1_000


@dataclass
class Pending:
    req_id: int
    verb: str
    payload: bytes
    label: str
    sent_at: float
    retries: int = 0
    acked: bool = False
    uncertain: bool = False
    uncertain_at: float | None = None
    uncertain_query_id: int | None = None


@dataclass
class GripRamp:
    action: str
    current_mdeg: int  # Last PWM command acknowledged by the MCU, not actual jaw position.
    target_mdeg: int
    waiting_req_id: int | None = None
    next_mdeg: int | None = None


@dataclass
class MoveSettle:
    req_id: int
    target_mdeg: int
    initial_mdeg: int
    verb: str = "MOVE"
    tolerance_mdeg: int = HOST_SETTLE_TOLERANCE_MDEG
    min_ms: int = HOST_SETTLE_MIN_MS
    last_mdeg: int | None = None
    last_t_ms: int | None = None
    stable_since_ms: int | None = None
    stable_anchor_mdeg: int | None = None
    stop_req_id: int | None = None
    stop_ack_at: float | None = None
    stop_state_query_id: int | None = None
    return_tolerance_mdeg: int | None = None
    return_idle_since_ms: int | None = None
    return_idle_anchor_mdeg: int | None = None


class ExperimentController(QObject):
    event = Signal(str, str, str)
    command_result = Signal(int, str, str)
    motion_event = Signal(object)
    state_changed = Signal()
    device_restarted = Signal()

    def __init__(self, serial_service: SerialService, calibration_path: Path | None = None):
        super().__init__()
        self.serial_service = serial_service
        self.calibration_path = calibration_path
        self.working_zero_mdeg = WORKING_ZERO_MDEG
        self.fold_offset_mdeg = FOLD_TARGET_MDEG - WORKING_ZERO_MDEG
        self._pose_synced = False
        self._pose_attempted = False
        self.pose_calibration_saved = False
        self.calibration_load_error = ""
        if calibration_path is not None:
            try:
                data = json.loads(calibration_path.read_text(encoding="utf-8"))
                normal, offset = data["normal_mdeg"], data["fold_offset_mdeg"]
                if not self._valid_pose(normal, offset):
                    raise ValueError("角度超出有效范围")
                self.working_zero_mdeg, self.fold_offset_mdeg = normal, offset
                self.pose_calibration_saved = True
            except FileNotFoundError:
                pass
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.calibration_load_error = f"校准文件读取失败，已使用默认值：{exc}"

        self.encoder = V1Encoder()
        self.simulated = False
        self.voltage_locked = False
        self.requested_shape = "未知"
        self.actual_state = "未知／等待反馈"
        self.execution_status = "未知／等待反馈"
        self.command_status = "未发送命令"
        self.last_request: MotionRequest | None = None
        self.last_extend_start_mdeg: int | None = None  # Snapshot of the accepted MOVE's preset start.
        self.pending_write = False
        self.submitted_requests = 0
        self.info: InfoFrame | None = None
        self.experiment_info: ExperimentInfoFrame | None = None
        self.capture_supported = False
        self._capture_probe_done = False
        self._capture_probe_attempts = 0
        self.cal: CalFrame | None = None
        self.latest_state: StateFrame | None = None
        self.last_hx8_diag: Hx8DiagFrame | None = None
        self.last_imu_diag: ImuDiagFrame | None = None
        self._last_imu_query = 0.0
        self.mode = "UNKNOWN"
        self.link_online = False
        self.wire_diagnostic: str | None = None
        self.visual_target_enabled = False
        self.ready_for_pick = False
        self._vision_start_requested = False
        self._queued_mode: str | None = None
        self._linked_action: str | None = None
        self._linked_req_id: int | None = None
        self._linked_move_settled = False
        self._move_settle: MoveSettle | None = None
        self.last_liveness: float | None = None
        self.last_acc_received: float | None = None
        self.last_visual_target_sent_at: float | None = None
        self._next_req = secrets.randbelow(1_000_000_000) + 1
        self._pending: dict[int, Pending] = {}
        self._cancelled_req_ids: set[int] = set()
        self._query_ids: dict[str, int] = {}
        self._last_ping = 0.0
        self._last_state_query = 0.0
        self._last_handshake_retry = 0.0
        self._last_pong_ms: int | None = None
        self._last_state_received_at: float | None = None
        self._grip_last_ack_mdeg: int | None = None
        self._grip_last_ack_req_id: int | None = None
        self._grip_last_ack_at: float | None = None
        self._pick_close_applied_req_id: int | None = None
        self._pick_close_applied_at: float | None = None
        self._grip_ramp: GripRamp | None = None
        self._grip_timer = QTimer(self)
        self._grip_timer.setSingleShot(True)
        self._grip_timer.timeout.connect(self._advance_grip_ramp)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)
        self.timer.start()
        serial_service.state_changed.connect(self._on_connection)
        serial_service.write_result.connect(self._on_write_result)

    def shutdown(self) -> None:
        self.timer.stop()
        self._grip_timer.stop()
        self._grip_ramp = None

    @property
    def grip_ramp_active(self) -> bool:
        return self._grip_ramp is not None

    @property
    def control_busy(self) -> bool:
        return self.pending_write or bool(self._pending) or self._linked_action is not None

    def cancel_grip_ramp(self, reason: str) -> None:
        if self._grip_ramp is None:
            return
        self._grip_timer.stop()
        self._grip_ramp = None
        self.event.emit("控制", "夹爪慢速测试", f"后续步进已取消：{reason}；已应用的PWM角度保持")
        self.state_changed.emit()

    def _advance_grip_ramp(self) -> None:
        ramp = self._grip_ramp
        if ramp is None or ramp.waiting_req_id is not None:
            return
        if (self.simulated or self.serial_service.state != ConnectionState.OPEN or
                not self.link_online or self.mode != "MANUAL" or self._pending or
                self.latest_state is None or self.latest_state.fault_code != "NONE"):
            self.cancel_grip_ramp("链路、模式或设备状态已变化")
            return
        delta = ramp.target_mdeg - ramp.current_mdeg
        if delta == 0:
            self._grip_ramp = None
            self.command_status = "全部分段角度已获设备ACK；夹爪实际位置无反馈"
            self.state_changed.emit()
            return
        step = min(abs(delta), GRIP_TEST_RAMP_STEP_MDEG) * (1 if delta > 0 else -1)
        next_mdeg = ramp.current_mdeg + step
        label = (f"夹爪{'张开' if ramp.action == 'OPEN' else '闭合'}方向测试："
                 f"SERVO1 {next_mdeg // 1000}°／目标 {ramp.target_mdeg // 1000}°")
        if not self._submit("SERVO", (1, next_mdeg), label, manual=True):
            self.cancel_grip_ramp("本步未能提交")
            return
        ramp.next_mdeg = next_mdeg
        ramp.waiting_req_id = self._next_req - 1

    def _id(self) -> int:
        if self._next_req > U32:
            raise ProtocolError("请求序号已用尽，等待设备新启动会话")
        req = self._next_req
        self._next_req += 1
        return req

    @property
    def fold_target_mdeg(self) -> int:
        return self.working_zero_mdeg + self.fold_offset_mdeg

    @property
    def pose_synced(self) -> bool:
        return self._pose_synced

    @staticmethod
    def _valid_pose(normal: int, offset: int) -> bool:
        return (type(normal) is int and type(offset) is int and normal % 100 == offset % 100 == 0
                and -90_000 + MANUAL_EXTENSION_LIMIT_MDEG <= normal
                and offset >= 10_000 and normal + offset <= 180_000)

    def calibration_position(self) -> int | None:
        state = self.latest_state
        now = time.monotonic()
        if (not self.simulated and self.link_online and self._last_state_received_at is not None
                and now - self._last_state_received_at <= 2 and state and state.hx8_online
                and state.motion == "IDLE" and state.fault_code == "NONE"
                and state.pos_mdeg is not None):
            return state.pos_mdeg
        return None

    def move_for_calibration(self, target: int) -> bool:
        if self.calibration_position() is None or self.info is None:
            return self._blocked("需要新鲜、空闲且无故障的 HX8 反馈")
        speed = max(1, round(self.info.max_speed_dps * 0.05))
        return self._submit("MOVE", (target, speed), f"校准试移 {target / 1000:+.1f}°",
                            manual=True, motion=True)

    def save_pose_calibration(self, offset: int) -> bool:
        if self.voltage_locked:
            return self._blocked("低压锁定，等待供电恢复")
        current = self.calibration_position()
        if current is None:
            return self._blocked("请等待 HX8 停稳并取得新鲜角度")
        if self._linked_action or self._grip_ramp or not self._guard(manual=True, motion=True):
            return False
        normal = round(current / 100) * 100
        if not self._valid_pose(normal, offset):
            folded = normal + offset
            if folded > 180_000:
                return self._blocked(
                    f"当前实际角度 {normal / 1000:+.1f}° 将被保存为正常位，"
                    f"加上折叠偏移后为 {folded / 1000:+.1f}°，超过固件 +180.0° 上限；"
                    "请先低速试移到实际正常位，停稳后再保存"
                )
            return self._blocked("当前实际角度无法保留 74° 出臂范围，或折叠偏移小于 10°")
        if self.calibration_path is None:
            return self._blocked("校准文件路径未配置")
        tmp = self.calibration_path.with_suffix(".tmp")
        try:
            tmp.parent.mkdir(parents=True, exist_ok=True)
            tmp.write_text(json.dumps({"normal_mdeg": normal, "fold_offset_mdeg": offset}), encoding="utf-8")
            tmp.replace(self.calibration_path)
        except OSError as exc:
            return self._blocked(f"校准保存失败：{exc}")
        self.working_zero_mdeg, self.fold_offset_mdeg = normal, offset
        self.pose_calibration_saved = True
        self.calibration_load_error = ""
        self.last_extend_start_mdeg = None
        self._pose_synced = False
        self._pose_attempted = False
        self.state_changed.emit()
        return True

    def _sync_pose_if_idle(self) -> None:
        if (self.voltage_locked or not self.pose_calibration_saved or self._pose_attempted or self._pending
                or self._linked_action or self._grip_ramp or self.mode != "MANUAL"
                or self.calibration_position() is None or self.info is None):
            return
        self._pose_attempted = self._submit("POSE", (self.working_zero_mdeg, self.fold_target_mdeg),
                                            "同步正常位与折叠位", manual=True, motion=True)

    def _reset_real(self) -> None:
        self._pose_synced = False
        self._pose_attempted = False
        self._grip_timer.stop()
        self._grip_ramp = None
        self._last_state_received_at = None
        self._grip_last_ack_mdeg = None
        self._grip_last_ack_req_id = None
        self._grip_last_ack_at = None
        self._pick_close_applied_req_id = None
        self._pick_close_applied_at = None
        self.info = None
        self.experiment_info = None
        self.capture_supported = False
        self._capture_probe_done = False
        self._capture_probe_attempts = 0
        self.cal = None
        self.latest_state = None
        self.last_hx8_diag = None
        self.last_imu_diag = None
        self._last_imu_query = 0.0
        self.mode = "UNKNOWN"
        self.link_online = False
        self.wire_diagnostic = None
        self.visual_target_enabled = False
        self.ready_for_pick = False
        self._vision_start_requested = False
        self._queued_mode = None
        self._linked_action = None
        self._linked_req_id = None
        self._linked_move_settled = False
        self._move_settle = None
        self.last_liveness = None
        self.requested_shape = "未知"
        self.last_extend_start_mdeg = None
        self.actual_state = "未知／等待反馈"
        self.execution_status = "未知／等待反馈"
        self.command_status = "当前会话无已确认命令"
        self.pending_write = False
        self._pending.clear()
        self._cancelled_req_ids.clear()
        self._query_ids.clear()
        self._last_pong_ms = None
        self.last_visual_target_sent_at = None
        self.state_changed.emit()

    def set_simulation(self, enabled: bool) -> None:
        if self.voltage_locked or self.simulated == enabled:
            return
        self.simulated = enabled
        self._reset_real()
        self.command_status = "未发送命令"
        self.last_request = None
        self.event.emit("模拟" if enabled else "系统", "模式切换",
                        "模拟演示已开启，不向真实串口发指令" if enabled else "已退出模拟，实际状态未知")
        if not enabled and self.serial_service.state == ConnectionState.OPEN:
            self._begin_handshake()

    def set_voltage_lock(self, locked: bool) -> None:
        if self.voltage_locked == locked:
            return
        self.voltage_locked = locked
        if locked:
            self.serial_service.discard_normal_queue()
            self._queued_mode = None
            self._vision_start_requested = False
            self.visual_target_enabled = False
            self.ready_for_pick = False
            state = self.latest_state
            pending_control = any(p.verb != "STOP" for p in self._pending.values())
            stopping = any(p.verb == "STOP" for p in self._pending.values())
            self._pending = {req: p for req, p in self._pending.items() if p.verb == "STOP"}
            self.pending_write = bool(self._pending)
            self._cancelled_req_ids.clear()
            self._move_settle = None
            self.cancel_grip_ramp("供电过低")
            self._cancel_linked_action("供电过低")
            if not stopping and (pending_control or (state is not None and state.motion not in ("IDLE", "FAULT"))):
                self.request_stop()
            self.command_status = "供电过低，已锁定控制"
        else:
            self.command_status = "供电已恢复，可重新提交操作"
        self.state_changed.emit()

    def _on_connection(self, state: ConnectionState) -> None:
        if state == ConnectionState.OPEN and not self.simulated:
            self._reset_real()
            self._begin_handshake()
        elif state in (ConnectionState.DISCONNECTED, ConnectionState.FAILED):
            self._reset_real()

    def _begin_handshake(self) -> None:
        # Send only the 3 essential V2 queries upfront so 433 MHz half-duplex HC-12/HC-13
        # air buffers and MCU TX slots (6) are not overwhelmed by a 5-frame burst;
        # EXPERIMENT and CAPTURE are probed staggered on _tick once INFO arrives.
        for kind in ("INFO", "CAL", "STATE"):
            self.query(kind)
        self._last_ping = time.monotonic() - 0.5
        self._last_handshake_retry = time.monotonic() - 0.55
        self.event.emit("协议", "握手", "已查询 INFO、CAL、STATE；收到有效V2回复前不允许运动")

    def note_unexpected_wire_response(self, response: bytes) -> None:
        """记录非 V2 回应；不能据此确认无线或 MCU 已在线。"""
        if self.link_online:
            self.event.emit("协议", "异常回应", f"收到非V2原始数据 {response!r}，请检查链路")
            return
        diagnostic = ("收到裸 ERROR（非MCU V2 ERR 帧）；检查是否选错COM口、HC13 KEY是否进入AT模式或机载端固件版本"
                      if response == b"ERROR" else
                      f"收到非V2数据 {response[:40]!r}；检查串口与固件协议")
        if diagnostic == self.wire_diagnostic:
            return
        self.wire_diagnostic = diagnostic
        self.command_status = f"设备未确认在线：{self.wire_diagnostic}"
        self.event.emit("协议", "非V2回应", self.command_status)
        self.state_changed.emit()

    def query(self, kind: str) -> bool:
        if self.simulated or self.serial_service.state != ConnectionState.OPEN:
            return False
        try:
            req = self._id()
            payload = self.encoder.encode("GET", req, kind)
        except ProtocolError as exc:
            self.event.emit("协议", "查询", str(exc))
            return False
        if not self.serial_service.send(payload):
            return False
        self._query_ids[kind] = req
        if kind == "CAPTURE":
            self._capture_probe_attempts += 1
        if kind == "STATE":
            self._last_state_query = time.monotonic()
        return True

    def _ping(self) -> None:
        if self.simulated or self.serial_service.state != ConnectionState.OPEN:
            return
        try:
            req = self._id()
            payload = self.encoder.encode("PING", req)
        except ProtocolError:
            return
        if self.serial_service.send(payload):
            self._query_ids["PING"] = req
            self._last_ping = time.monotonic()

    @staticmethod
    def _retry_backoff_s(req_id: int, retries: int) -> float:
        # Non-resonant jittered backoff (< 0.56 s) breaking 0.500 s == 10 x 50 ms ACC phase lock!
        if retries <= 0:
            return 0.33 + (req_id % 17) * 0.007
        return 0.42 + ((req_id // 17) % 19) * 0.007

    def _release_uncertain_pending(self, pending: Pending, *, confirmed_by_state: bool, detail_note: str = "") -> None:
        if self._pending.pop(pending.req_id, None) is None:
            return
        self._cancelled_req_ids.discard(pending.req_id)
        if self._move_settle is not None and self._move_settle.req_id == pending.req_id:
            self._move_settle = None
        if self._linked_req_id == pending.req_id:
            self._cancel_linked_action("无线回执未确认，已释放控制锁")
        self.pending_write = any(not p.acked and not p.uncertain for p in self._pending.values())
        if detail_note:
            self.command_status = detail_note
        elif confirmed_by_state:
            self.command_status = f"{pending.label}：无线回执超时，已由STATE确认设备空闲并自动解锁按键"
        else:
            self.command_status = f"{pending.label}：无线回执未确认，超时自动释放按键锁（请检查状态后重试）"
        self.event.emit("协议", pending.label, self.command_status)
        self.command_result.emit(pending.req_id, pending.verb, "UNCERTAIN_CLEARED")
        self.state_changed.emit()

    def _tick(self) -> None:
        if self.simulated or self.serial_service.state != ConnectionState.OPEN:
            return
        now = time.monotonic()
        awaiting_ack = any(not p.acked and not p.uncertain for p in self._pending.values())
        # Coprime intervals (1.07s / 0.93s / 0.37s / 1.13s) avoid locking onto MCU's 50 ms ACC stream,
        # and pausing routine polls while awaiting ACK prevents shooting down incoming ACK packets.
        if (not awaiting_ack and now - self._last_ping >= 1.07) or (now - self._last_ping >= 2.3):
            self._ping()
        if (not awaiting_ack and self.info is not None and
                now - self._last_state_query >= (0.37 if self._move_settle else 0.93)):
            self.query("STATE")
        state = self.latest_state
        if (not awaiting_ack and self.info is not None and state is not None and
                (not state.imu_online or state.imu_hz < 180 or state.imu_hz > 220) and
                now - self._last_imu_query >= 5.13):
            if self.query("IMU"):
                self._last_imu_query = now
        if (self._move_settle is not None and self._move_settle.stop_ack_at is not None and
                now - self._move_settle.stop_ack_at > HOST_SETTLE_STATE_TIMEOUT_S):
            self._abandon_move_settle("STOP 后未取得新鲜无故障空闲反馈")
        if not awaiting_ack and now - self._last_handshake_retry >= 1.13:
            sent_hs = 0
            if self.info is None and sent_hs < 2:
                sent_hs += int(self.query("INFO"))
            if self.cal is None and sent_hs < 2:
                sent_hs += int(self.query("CAL"))
            if (self.latest_state is None or not self.link_online) and sent_hs < 2:
                sent_hs += int(self.query("STATE"))
            if self.info is not None and self.experiment_info is None and sent_hs < 2:
                sent_hs += int(self.query("EXPERIMENT"))
            if (self.info is not None and self.experiment_info is not None and
                    not self._capture_probe_done and self._capture_probe_attempts < 3 and sent_hs < 2):
                sent_hs += int(self.query("CAPTURE"))
            self._last_handshake_retry = now
        if self.last_liveness is not None and now - self.last_liveness > 3.0 and self.link_online:
            self.cancel_grip_ramp("链路超时")
            self._cancel_linked_action("链路超时")
            self.link_online = False
            self.visual_target_enabled = False
            self.actual_state = "未知／链路离线"
            self.execution_status = "未知／等待反馈"
            self.event.emit("协议", "链路", "3秒没有新STATE/PONG，设备状态标为离线")
            self.state_changed.emit()
        self._sync_pose_if_idle()
        for pending in list(self._pending.values()):
            if self.voltage_locked and pending.verb != "STOP":
                continue
            if pending.uncertain:
                if (pending.uncertain_at is not None and now - pending.uncertain_at >= 1.8 and
                        (self.latest_state is None or self.latest_state.motion not in ("MOVING", "SETTLING")
                         or not self.link_online)):
                    self._release_uncertain_pending(pending, confirmed_by_state=False)
                continue
            if pending.acked or now - pending.sent_at < self._retry_backoff_s(pending.req_id, pending.retries):
                continue
            if pending.retries < 2:
                if self.serial_service.send(pending.payload, priority=pending.verb == "STOP"):
                    pending.retries += 1
                    pending.sent_at = now
                    self.event.emit("协议", pending.label, f"ACK超时，原req_id重发第{pending.retries}次")
            else:
                pending.uncertain = True
                pending.uncertain_at = now
                if (self._move_settle is not None and
                        pending.req_id == self._move_settle.stop_req_id):
                    self._abandon_move_settle("STOP回执超时")
                if self._linked_req_id == pending.req_id:
                    self._cancel_linked_action("联动步骤回执超时")
                if self._grip_ramp is not None and self._grip_ramp.waiting_req_id == pending.req_id:
                    self.cancel_grip_ramp("SERVO回执超时")
                self.pending_write = False
                self.command_status = f"{pending.label}结果未确认；已停止重试并查询STATE"
                self.event.emit("协议", pending.label, self.command_status)
                self.query("STATE")
                pending.uncertain_query_id = self._query_ids.get("STATE")
                self.state_changed.emit()
        if self._queued_mode is not None and not self._pending and self.link_online:
            mode = self._queued_mode
            self._queued_mode = None
            self.request_mode(mode)

    def handle_frame(self, frame: Frame) -> None:
        if self.simulated:
            return
        now = time.monotonic()
        if isinstance(frame, InfoFrame):
            if frame.req_id != self._query_ids.get("INFO"):
                return
            if self.info is not None and frame.boot_id != self.info.boot_id:
                self._reset_real()
                self.device_restarted.emit()
                self.event.emit("协议", "设备重启", "boot_id变化，已清除旧请求匹配")
                self._begin_handshake()
                return
            self.info = frame
            self.wire_diagnostic = None
            self._next_req = max(self._next_req, frame.last_req_id + 1)
            self.link_online = self.latest_state is not None and self.last_liveness is not None and now - self.last_liveness <= 3.0
            self.event.emit("协议", "INFO", f"V{frame.version} · boot={frame.boot_id} · 上限{frame.max_speed_dps}°/s")
        elif isinstance(frame, ExperimentInfoFrame):
            if frame.req_id != self._query_ids.get("EXPERIMENT"):
                return
            self.experiment_info = frame
            self.event.emit("协议", "实验固件", "已确认支持 XMOVE 与可调加减速时间")
        elif isinstance(frame, CaptureInfoFrame):
            if frame.req_id != self._query_ids.get("CAPTURE"):
                return
            self._capture_probe_done = True
            self.capture_supported = True
            self.event.emit("协议", "实验固件", "已确认支持板载高频采样")
        elif isinstance(frame, ImuDiagFrame):
            if frame.req_id != self._query_ids.get("IMU"):
                return
            previous = self.last_imu_diag
            self.last_imu_diag = frame
            stage = ("空闲", "扫描波特率", "确认波特率", "确认采样率")[frame.recovery_stage]
            if previous is not None and frame.good_frames > previous.good_frames:
                cause = "收到新增有效 IMU 帧，采样率尚未恢复" if frame.imu_hz == 0 else "收到新增有效 IMU 帧"
            elif previous is not None and frame.uart_errors > previous.uart_errors:
                cause = "USART1 错误持续增加，需区分波特率不匹配与线路异常"
            elif previous is not None and frame.rx_bytes == previous.rx_bytes:
                cause = "DMA 半缓冲计数未变化，无法据此判断 USART1 是否收到字节"
            elif frame.good_frames and frame.imu_hz == 0:
                cause = "已有有效帧，正在确认 200 Hz 采样率"
            else:
                cause = "等待下一次诊断对照"
            self.event.emit("设备反馈", "IMU诊断",
                            f"{cause}；恢复阶段={stage}；DMA半缓冲累计字节={frame.rx_bytes}，有效帧={frame.good_frames}，"
                            f"校验错={frame.checksum_errors}，UART错误={frame.uart_errors}，DMA重启失败={frame.rearm_failures}")
            self.state_changed.emit()
        elif isinstance(frame, CalFrame):
            if frame.req_id != self._query_ids.get("CAL"):
                return
            self.cal = frame
            self.event.emit("协议", "CAL", "标定已读取；缺失项仍禁止对应动作")
        elif isinstance(frame, StateFrame):
            if self.latest_state is not None and frame.t_ms < self.latest_state.t_ms and self.latest_state.t_ms - frame.t_ms < 0x80000000:
                self._reset_real()
                self.device_restarted.emit()
                self.event.emit("协议", "设备时间归零", "可能已重启，重新读取INFO/CAL/STATE")
                self._begin_handshake()
                return
            previous_fault = self.latest_state.fault_code if self.latest_state is not None else None
            self.latest_state = frame
            self._last_state_received_at = now
            if self._grip_last_ack_req_id is None or frame.last_req_id >= self._grip_last_ack_req_id:
                self._grip_last_ack_mdeg = frame.grip_cmd_mdeg
            if frame.fault_code != "NONE" or frame.mode != "MANUAL":
                self.cancel_grip_ramp("设备故障或模式变化")
            self.mode = frame.mode
            if frame.mode != "VISION_PICK":
                self.visual_target_enabled = False
                self.last_visual_target_sent_at = None
            self.actual_state = {"UNKNOWN": "未知", "NORMAL": "正常", "FOLDED": "折叠"}[frame.shape]
            self.execution_status = f"{frame.motion} · 位置 {frame.pos_mdeg / 1000:.1f}°" if frame.pos_mdeg is not None else f"{frame.motion} · 位置未知"
            self.last_liveness = now
            self.link_online = self.info is not None
            self._handle_move_settle_state(frame, now)
            self._reconcile_pending_on_state(frame, now)
            if frame.fault_code != "NONE":
                self.ready_for_pick = False
                self.visual_target_enabled = False
                if frame.fault_code != previous_fault:
                    position = f"{frame.pos_mdeg / 1000:.1f}°" if frame.pos_mdeg is not None else "未知"
                    target = f"{frame.target_mdeg / 1000:.1f}°" if frame.target_mdeg is not None else "未知"
                    voltage = f"{frame.voltage_mv / 1000:.2f} V" if frame.voltage_mv is not None else "未知"
                    self.event.emit("设备反馈", "故障",
                                    f"{frame.fault_code} · HX8实际 {position} · 目标 {target} · 回报电压 {voltage}")
        elif isinstance(frame, PongFrame):
            if frame.req_id != self._query_ids.get("PING"):
                return
            if self._last_pong_ms is not None and frame.t_ms < self._last_pong_ms and self._last_pong_ms - frame.t_ms < 0x80000000:
                self._reset_real()
                self.device_restarted.emit()
                self.event.emit("协议", "设备时间归零", "PONG时间回退，重新握手")
                self._begin_handshake()
                return
            self._last_pong_ms = frame.t_ms
            self.last_liveness = now
            self.link_online = self.info is not None and self.latest_state is not None
        elif isinstance(frame, Hx8DiagFrame):
            self.last_hx8_diag = frame
            reasons = {
                "NO_SAMPLE": "没有HX8反馈", "AGE": "HX8反馈超龄",
                "ANGLE": "HX8角度异常", "VOLT_LOW": "HX8瞬时欠压",
                "VOLT_HIGH": "HX8电压过高", "STATUS": "HX8状态位异常",
                "HX8_OK_OR_OTHER": "HX8快照正常，检查接收溢出或IMU",
            }
            if frame.cause == "STATUS" and frame.last_status & 0x40:
                reasons["STATUS"] = "HX8功率保护触发（状态位bit6）"
            pos = "未知" if frame.last_pos_mdeg is None else f"{frame.last_pos_mdeg / 1000:.1f}°"
            self.event.emit("设备诊断", "HX8高速中止",
                            f"{reasons[frame.cause]} · 反馈龄 {frame.feedback_age_ms} ms · "
                            f"位置 {pos} · 电压 {frame.last_voltage_mv / 1000:.2f} V · "
                            f"电流 {frame.last_current_ma} mA · 状态位 0x{frame.last_status:02X}")
        elif isinstance(frame, AckFrame):
            pending = self._pending.get(frame.req_id)
            if pending is None or pending.verb != frame.verb:
                return
            if frame.req_id in self._cancelled_req_ids:
                return
            if frame.verb == "MODE" and not pending.payload.rstrip().endswith(frame.result.encode("ascii")):
                self.event.emit("协议", "ACK不匹配", f"req={frame.req_id} 的模式与请求不一致")
                return
            pending.acked = True
            self.pending_write = False
            self.command_status = f"设备ACK {frame.verb}:{frame.result}；等待实际反馈" if frame.verb in ("MOVE", "XMOVE", "HOME", "SHAPE", "PREP", "PICK", "TRIGGER") else f"设备ACK {frame.verb}:{frame.result}"
            self.event.emit("设备反馈", pending.label, self.command_status)
            if frame.verb == "MODE":
                if frame.result != self.mode:
                    self.ready_for_pick = False
                    self.last_visual_target_sent_at = None
                self.mode = frame.result
                if frame.result != "VISION_PICK":
                    self.visual_target_enabled = False
            if frame.verb == "POSE":
                self._pose_synced = True
            if frame.verb not in ("MOVE", "XMOVE", "HOME", "SHAPE", "PREP", "PICK", "TRIGGER"):
                self._pending.pop(frame.req_id, None)
            if frame.verb == "SERVO":
                # ACK confirms the PWM command, not the physical jaw position.
                self._grip_last_ack_mdeg = int(pending.payload.rstrip().split(b",")[-1])
                self._grip_last_ack_req_id = frame.req_id
                self._grip_last_ack_at = now
                self.command_result.emit(frame.req_id, "SERVO", "APPLIED")
                if self._linked_req_id == frame.req_id:
                    if self._linked_action == "FOLD":
                        self._linked_req_id = None
                        if not self._submit_fold_move():
                            self._cancel_linked_action("开爪后无法提交折叠运动")
                    elif self._linked_action in ("RESTORE", "HOME"):
                        action = self._linked_action
                        self._linked_action = None
                        self._linked_req_id = None
                        location = ("HX8 近目标稳定保持（非精确到位）" if self._linked_move_settled else
                                    f"HX8 已到 {self.working_zero_mdeg / 1000:+g}°工作位")
                        self._linked_move_settled = False
                        self.command_status = ("正常模式完成：" if action == "RESTORE" else "归零完成：") + location + "，夹爪张开 PWM 已应用（无位置反馈）"
                        self.event.emit("设备反馈", "联动流程完成", self.command_status)
            if (frame.verb == "SERVO" and self._grip_ramp is not None and
                    self._grip_ramp.waiting_req_id == frame.req_id):
                ramp = self._grip_ramp
                ramp.current_mdeg = ramp.next_mdeg
                self._grip_last_ack_mdeg = ramp.current_mdeg
                self._grip_last_ack_req_id = frame.req_id
                self._grip_last_ack_at = now
                ramp.waiting_req_id = None
                ramp.next_mdeg = None
                if ramp.current_mdeg == ramp.target_mdeg:
                    self._grip_ramp = None
                    self.command_status = "全部分段角度已获设备ACK；夹爪实际位置无反馈"
                    self.event.emit("设备反馈", pending.label, self.command_status)
                    self.query("STATE")
                else:
                    self._grip_timer.start(GRIP_TEST_RAMP_INTERVAL_MS)
            if frame.verb == "STOP":
                settle = self._move_settle
                if settle is not None and settle.stop_req_id == frame.req_id:
                    self._pending.pop(settle.req_id, None)
                    settle.stop_ack_at = now
                    self.query("STATE")
                    settle.stop_state_query_id = self._query_ids.get("STATE")
                for cancelled in self._cancelled_req_ids:
                    self._pending.pop(cancelled, None)
                self._cancelled_req_ids.clear()
                self.query("STATE")
                self.command_result.emit(frame.req_id, "STOP", "APPLIED")
                if self._queued_mode is not None:
                    mode = self._queued_mode
                    self._queued_mode = None
                    self.request_mode(mode)
            if frame.verb == "MODE" and frame.result == "VISION_PICK" and self._vision_start_requested:
                self._vision_start_requested = False
                self.request_visual_prepare()
        elif isinstance(frame, DoneFrame):
            if self._move_settle is not None and frame.req_id == self._move_settle.req_id:
                if self._move_settle.stop_req_id is not None:
                    return  # STOP is in flight; verify its ACK and a fresh IDLE state first.
                self._move_settle = None
            if frame.req_id in self._cancelled_req_ids:
                self._pending.pop(frame.req_id, None)
                self._cancelled_req_ids.discard(frame.req_id)
                return
            pending = self._pending.get(frame.req_id)
            if pending is None or pending.verb != frame.verb:
                return
            self._complete_motion_done(pending, frame.actual_pos_mdeg, now, query_state=True)
        elif isinstance(frame, ErrFrame):
            if frame.req_id == self._query_ids.get("CAPTURE") and frame.code in ("BAD_FRAME", "UNSUPPORTED"):
                self._capture_probe_done = True
                return
            if (self._move_settle is not None and frame.req_id == self._move_settle.req_id and
                    self._move_settle.stop_req_id is not None and frame.code == "STOPPED"):
                return  # STOP may produce a late STOPPED for the original MOVE.
            pending = self._pending.pop(frame.req_id, None)
            if pending is None:
                return
            if self._move_settle is not None and frame.req_id in (self._move_settle.req_id, self._move_settle.stop_req_id):
                self._abandon_move_settle(f"设备返回 {frame.code}")
            self.command_result.emit(frame.req_id, pending.verb if pending else "UNKNOWN", frame.code)
            self._cancelled_req_ids.discard(frame.req_id)
            self.pending_write = False
            label = pending.label if pending else f"req_id={frame.req_id}"
            reasons = {
                "NOT_CALIBRATED": "动作所需标定缺失",
                "NOT_READY": "执行器或准备状态未就绪",
                "HX8_OFFLINE": "HX8位置反馈离线",
                "IMU_OFFLINE": "IMU离线",
                "IMU_RATE_LOW": "IMU有效采样率不足",
                "MODE_DENIED": "当前工作模式不允许此命令",
                "BUSY": "设备正执行其他动作",
                "FEEDBACK_STALE": "MCU中止：HX8反馈健康检查未通过；可能是运动中反馈超过400 ms、瞬时电压或状态位异常，需原始HX8日志区分",
                "STALE_REQUEST": "请求序号旧于设备记录",
                "REQUEST_CONFLICT": "同一序号对应不同命令",
            }
            suffix = f"（{reasons[frame.code]}）" if frame.code in reasons else ""
            self.command_status = f"设备拒绝／故障：{frame.code}{suffix}"
            self.event.emit("设备反馈", label, self.command_status)
            if self._grip_ramp is not None and self._grip_ramp.waiting_req_id == frame.req_id:
                self.cancel_grip_ramp("设备拒绝本步")
            if pending and pending.verb in ("MOVE", "XMOVE", "HOME", "SHAPE", "PREP", "PICK", "TRIGGER"):
                self.execution_status = self.command_status
            if self._linked_req_id == frame.req_id:
                self._cancel_linked_action(f"设备拒绝联动步骤：{frame.code}")
        elif isinstance(frame, MotionFrame):
            pending = self._pending.get(frame.req_id)
            if pending is None:
                return  # Late/foreign event cannot rearm a cancelled action.
            if frame.req_id in self._cancelled_req_ids:
                return
            if frame.stage == "CLOSE_APPLIED" and pending.verb == "PICK":
                # This confirms PWM application, not grip force or HX8 arrival.
                self._pick_close_applied_req_id = frame.req_id
                self._pick_close_applied_at = now
            if (self._move_settle is not None and self._move_settle.req_id == frame.req_id and
                    self._move_settle.stop_req_id is not None):
                if frame.stage == "ABORT" and frame.reason != "STOPPED":
                    self._abandon_move_settle(f"动作中止：{frame.reason}")
                else:
                    return
            self.event.emit("设备动作", frame.stage, f"req={frame.req_id} · 原因 {frame.reason}")
            self.motion_event.emit(frame)
            stages = {
                "PREPARING": "准备中：HX8回竖直位", "OPEN_APPLIED": "开爪命令已应用，等待配置时间",
                "READY": "准备完成，待视觉目标／手动触发", "START": "出臂／折叠运动中",
                "CLOSE_APPLIED": "闭爪命令已应用（无夹持反馈）", "ARRIVED": "HX8反馈确认到位",
            }
            if frame.stage in stages:
                self.execution_status = stages[frame.stage]
            if frame.stage == "READY" and frame.req_id not in self._cancelled_req_ids and pending.verb in ("PREP", "HOME", "SHAPE"):
                self.ready_for_pick = True
                self.visual_target_enabled = self.mode == "VISION_PICK"
                self.last_visual_target_sent_at = None
            elif frame.stage in ("START", "CLOSE_APPLIED", "ABORT"):
                self.ready_for_pick = False
                self.visual_target_enabled = False
                self.last_visual_target_sent_at = None
            if frame.stage == "ABORT":
                if self._move_settle is not None and self._move_settle.req_id == frame.req_id:
                    self._abandon_move_settle(f"动作中止：{frame.reason}")
                if self._linked_req_id == frame.req_id:
                    self._cancel_linked_action(f"动作中止：{frame.reason}")
                self._pending.pop(frame.req_id, None)
                self._cancelled_req_ids.discard(frame.req_id)
                self.pending_write = False
                self.execution_status = f"动作中止：{frame.reason}"
                self.command_status = self.execution_status
                # ABORT can be the only terminal reply: a later ERR may be lost
                # over the radio. Release the page flow from this event itself.
                self.command_result.emit(frame.req_id, pending.verb, frame.reason)
        self.state_changed.emit()

    def _on_write_result(self, payload: bytes, success: bool, detail: str) -> None:
        matched = next((p for p in self._pending.values() if p.payload == payload), None)
        if matched is None or matched.uncertain:
            return  # GET/PING写入结果不改变控制状态
        self.pending_write = False
        if not success:
            self.command_status = detail
            self.event.emit("实际串口", matched.label, detail)
            if self._grip_ramp is not None and self._grip_ramp.waiting_req_id == matched.req_id:
                self._pending.pop(matched.req_id, None)
                self.cancel_grip_ramp("本步串口写入失败")
            if self._linked_req_id == matched.req_id:
                self._cancel_linked_action("联动步骤串口写入失败")
        else:
            matched.sent_at = time.monotonic()
            self.command_status = f"已写入 {matched.verb}，等待设备ACK"
            self.event.emit("实际串口", matched.label, self.command_status)
        self.state_changed.emit()

    def _guard(self, *, manual: bool = False, motion: bool = False) -> bool:
        if self.voltage_locked:
            self.command_status = "低压锁定，等待供电恢复"
        elif self.serial_service.state != ConnectionState.OPEN or self.info is None or not self.link_online:
            self.command_status = (f"设备V2握手／在线状态未确认，禁止控制；{self.wire_diagnostic}"
                                   if self.wire_diagnostic else "设备V2握手／在线状态未确认，禁止控制")
        elif self._pending:
            self.command_status = "上一请求尚未确认；可使用STOP"
        elif manual and self.mode != "MANUAL":
            self.command_status = "当前非MANUAL模式，禁止此操作"
        elif motion and (self.latest_state is None or not self.latest_state.hx8_online or
                         self.latest_state.motion != "IDLE" or self.latest_state.fault_code != "NONE"):
            self.command_status = "HX8未就绪、动作未结束或存在故障"
        else:
            return True
        self.event.emit("控制", "拦截", self.command_status)
        self.state_changed.emit()
        return False

    def _require_link(self) -> bool:
        """先报告真实握手状态，避免把尚未读到 CAL 误报为标定缺失。"""
        if self.simulated or (self.serial_service.state == ConnectionState.OPEN and
                              self.info is not None and self.link_online):
            return True
        return self._guard()

    def _submit(self, verb: str, values: tuple, label: str, *, manual=False, motion=False,
                priority=False, host_settle=True) -> bool:
        if self.voltage_locked and verb != "STOP":
            return self._blocked("低压锁定，等待供电恢复")
        if self._grip_ramp is not None and verb not in ("SERVO", "STOP"):
            self.cancel_grip_ramp(f"改为执行 {verb}")
        if self.simulated:
            self.submitted_requests += 1
            self.command_status = "模拟请求已提交（未写入串口）"
            self.event.emit("模拟", label, self.command_status)
            self.state_changed.emit()
            return True
        if not priority and not self._guard(manual=manual, motion=motion):
            return False
        if priority and (self.serial_service.state != ConnectionState.OPEN or not self.link_online):
            self.command_status = "链路未确认，无法发送STOP"
            self.state_changed.emit()
            return False
        try:
            req = self._id()
            payload = self.encoder.encode(verb, req, *values)
        except ProtocolError as exc:
            self.command_status = str(exc)
            self.event.emit("控制", label, self.command_status)
            self.state_changed.emit()
            return False
        if not self.serial_service.send(payload, priority=priority):
            self.command_status = "串口发送队列已满或连接不可用"
            self.state_changed.emit()
            return False
        self._pending[req] = Pending(req, verb, payload, label, time.monotonic())
        if (verb in ("MOVE", "XMOVE") and host_settle and values and self.latest_state is not None and
                self.latest_state.pos_mdeg is not None):
            self._move_settle = MoveSettle(req, int(values[0]), self.latest_state.pos_mdeg,
                                          verb=verb,
                                          tolerance_mdeg=(EXPERIMENT_SETTLE_TOLERANCE_MDEG if verb == "XMOVE"
                                                          else HOST_SETTLE_TOLERANCE_MDEG),
                                          min_ms=(EXPERIMENT_SETTLE_MIN_MS if verb == "XMOVE"
                                                  else HOST_SETTLE_MIN_MS))
        self.submitted_requests += 1
        self.pending_write = True
        self.command_status = f"{label}已入发送队列，等待设备ACK"
        self.event.emit("控制", label, self.command_status)
        self.state_changed.emit()
        return True

    def request_experiment_capture_page(self, move_id: int, page: int) -> int | None:
        if self.simulated or not self.link_online or not self.capture_supported:
            return None
        req_id = self._id()
        payload = self.encoder.encode("GET", req_id, "CAPTURE", move_id, page)
        return req_id if self.serial_service.send(payload) else None

    def request_experiment_move(self, target_mdeg: int, speed_percent: int, ramp_ms: int,
                                *, capture: bool = False, return_tolerance_mdeg: int | None = None) -> int | None:
        """Submit one measured experiment move; return its request id, never a motion claim."""
        state = self.latest_state
        if self.simulated or self.cal is None or self.info is None or state is None:
            self.event.emit("实验", "拦截", "需要真实设备的 INFO、CAL、STATE 反馈")
            return None
        if self.experiment_info is None:
            self.event.emit("实验", "拦截", "XMOVE 能力回复尚未收到，正在重试查询")
            return None
        if not state.imu_online or state.imu_hz < 60:
            self.event.emit("实验", "拦截", "IMU 无有效反馈或采样率过低")
            return None
        if (self.cal.home_mdeg != 0 or self.cal.extension_sign != -1 or
                self.cal.max_extension_mdeg is None or self.cal.max_extension_mdeg < 70_000):
            self.event.emit("实验", "拦截", "当前设备标定与实验行程不符；需要零位 0°、负向前伸且至少 70° 行程")
            return None
        if (target_mdeg < -90_000 or target_mdeg > 180_000 or target_mdeg % 100 or
                speed_percent not in EXPERIMENT_SPEED_LEVELS):
            self.event.emit("实验", "拦截", "目标角度或速度超出实验范围")
            return None
        if not 20 <= ramp_ms <= 2000:
            self.event.emit("实验", "拦截", "加减速时间需在 20–2000 ms")
            return None
        speed = max(1, round(self.info.max_speed_dps * speed_percent / 100))
        label = f"实验运动：目标 {target_mdeg / 1000:.1f}°，速度 {speed_percent}%（{speed}°/s），加减速各 {ramp_ms} ms"
        values = (target_mdeg, speed, ramp_ms, 1) if capture and self.capture_supported else (target_mdeg, speed, ramp_ms)
        if not self._submit("XMOVE", values, label,
                            manual=True, motion=True, host_settle=True):
            return None
        if not capture and self._move_settle is not None:
            self._move_settle.return_tolerance_mdeg = return_tolerance_mdeg
        return self._next_req - 1

    def _cancel_linked_action(self, reason: str) -> None:
        if self._linked_action is None:
            return
        action = self._linked_action
        self._linked_action = None
        self._linked_req_id = None
        self._linked_move_settled = False
        self.event.emit("控制", "联动流程取消", f"{action}：{reason}；已应用的夹爪PWM保持")

    def _abandon_move_settle(self, reason: str) -> None:
        settle = self._move_settle
        if settle is None:
            return
        self._move_settle = None
        self._cancel_linked_action(reason)
        self.command_status = f"近目标保持未确认：{reason}；不会自动继续动作"
        self.event.emit("控制", "位置稳定判定", self.command_status)
        self.state_changed.emit()

    def _handle_move_settle_state(self, frame: StateFrame, now: float) -> None:
        settle = self._move_settle
        if settle is None:
            return
        if frame.fault_code != "NONE" or not frame.hx8_online or frame.pos_mdeg is None:
            self._abandon_move_settle(f"HX8反馈异常或故障 {frame.fault_code}")
            return
        if settle.stop_req_id is not None:
            if (settle.stop_ack_at is None or settle.stop_state_query_id is None or
                    frame.reply_to < settle.stop_state_query_id):
                return
            if frame.motion != "IDLE" or abs(frame.pos_mdeg - settle.target_mdeg) > settle.tolerance_mdeg:
                return
            pending = self._pending.pop(settle.req_id, None)
            self._move_settle = None
            self.pending_write = False
            error = abs(frame.pos_mdeg - settle.target_mdeg) / 1000
            self.command_status = (f"近目标稳定保持：实际 {frame.pos_mdeg / 1000:.1f}°，"
                                   f"目标 {settle.target_mdeg / 1000:.1f}°，误差 {error:.1f}°；非精确到位")
            self.execution_status = self.command_status
            self.event.emit("设备反馈", pending.label if pending else "MOVE", self.command_status)
            if self._linked_req_id == settle.req_id:
                if self._linked_action == "FOLD":
                    self._linked_action = None
                    self._linked_req_id = None
                    self.requested_shape = "折叠近目标保持；开爪PWM已应用"
                elif self._linked_action in ("RESTORE", "HOME"):
                    self._linked_req_id = None
                    if not self._submit_linked_open():
                        self._cancel_linked_action("近目标保持后开爪命令未提交")
            self.command_result.emit(settle.req_id, settle.verb, "SETTLED")
            return
        pending = self._pending.get(settle.req_id)
        if pending is not None and pending.acked and settle.return_tolerance_mdeg is not None:
            if self._confirm_experiment_return(frame, settle, pending):
                return
        travel_mdeg = abs(settle.target_mdeg - settle.initial_mdeg)
        if (pending is None or not pending.acked or frame.motion != "MOVING" or
                frame.t_ms == settle.last_t_ms or travel_mdeg == 0):
            return
        if travel_mdeg <= settle.tolerance_mdeg:
            eff_tol = min(settle.tolerance_mdeg, 2_500)
            eff_min_ms = max(settle.min_ms, 600)
        else:
            eff_tol = settle.tolerance_mdeg
            eff_min_ms = settle.min_ms
        near = abs(frame.pos_mdeg - settle.target_mdeg) <= eff_tol
        quiet = (settle.stable_anchor_mdeg is not None and
                 abs(frame.pos_mdeg - settle.stable_anchor_mdeg) <= HOST_SETTLE_DELTA_MDEG)
        if not near:
            settle.stable_since_ms = None
            settle.stable_anchor_mdeg = None
        elif not quiet:
            settle.stable_since_ms = frame.t_ms
            settle.stable_anchor_mdeg = frame.pos_mdeg
        settle.last_mdeg = frame.pos_mdeg
        settle.last_t_ms = frame.t_ms
        if (settle.stable_since_ms is not None and frame.t_ms - settle.stable_since_ms >= eff_min_ms and
                self._last_state_received_at is not None and now - self._last_state_received_at < 0.75):
            if self._submit("STOP", (), "HX8近目标稳定，停止并保持", priority=True):
                settle.stop_req_id = self._next_req - 1
                self.event.emit("控制", "位置稳定判定", "已发送STOP；等待设备ACK和新鲜空闲反馈")
            else:
                self._abandon_move_settle("STOP未能提交")

    def _complete_motion_done(self, pending: Pending, actual_pos_mdeg: int, now: float, *, query_state: bool) -> None:
        req_id = pending.req_id
        verb = pending.verb
        self._pending.pop(req_id, None)
        self.pending_write = any(not p.acked and not p.uncertain for p in self._pending.values())
        self.command_status = f"DONE {verb}，HX8位置 {actual_pos_mdeg / 1000:.1f}°"
        if verb == "PICK":
            self._grip_last_ack_req_id = req_id
            self._grip_last_ack_mdeg = GRIP_TEST_ANGLE_MDEG["CLOSE"]
            self._grip_last_ack_at = now
        self.execution_status = self.command_status
        if verb in ("PICK", "TRIGGER", "SHAPE", "MOVE"):
            self.ready_for_pick = False
        self.event.emit("设备反馈", pending.label, self.command_status)
        if query_state:
            self.query("STATE")
        if self._linked_req_id == req_id:
            if self._linked_action == "FOLD":
                self._linked_action = None
                self._linked_req_id = None
                self.command_status = f"折叠流程完成：开爪 PWM 已应用，HX8 已到 {self.fold_target_mdeg / 1000:+g}°（夹爪无位置反馈）"
                self.requested_shape = f"{self.fold_target_mdeg / 1000:+g}° 折叠到位；开爪命令已应用"
                self.event.emit("设备反馈", "折叠流程完成", self.command_status)
            elif self._linked_action in ("RESTORE", "HOME"):
                self._linked_req_id = None
                self._linked_move_settled = False
                if not self._submit_linked_open():
                    self._cancel_linked_action("HX8 到位后无法提交开爪命令")
        self.command_result.emit(req_id, verb, "DONE")

    def _reconcile_pending_on_state(self, frame: StateFrame, now: float) -> None:
        """Recover cleanly when HC-12/HC-13 half-duplex RF drops an ACK or a single-shot DONE/ERR frame."""
        for pending in list(self._pending.values()):
            if pending.uncertain:
                query_matched = pending.uncertain_query_id is None or frame.reply_to >= pending.uncertain_query_id
                if query_matched and frame.motion not in ("MOVING", "SETTLING"):
                    if frame.last_req_id >= pending.req_id and frame.fault_code == "NONE":
                        if pending.verb == "SERVO" and frame.grip_cmd_mdeg is not None:
                            self._grip_last_ack_mdeg = frame.grip_cmd_mdeg
                            self._grip_last_ack_req_id = pending.req_id
                            self._grip_last_ack_at = now
                        pos_note = f"（位置 {frame.pos_mdeg / 1000:.1f}°）" if frame.pos_mdeg is not None else ""
                        note = f"{pending.label}：无线ACK漏收，已由STATE确认设备空闲完成{pos_note}并解锁按键"
                    else:
                        note = f"{pending.label}：无线回执超时，STATE确认设备未在运动，按键已自动解锁"
                    self._release_uncertain_pending(pending, confirmed_by_state=True, detail_note=note)
                continue

            if not pending.acked or pending.verb not in ("MOVE", "XMOVE", "HOME", "SHAPE", "PREP", "PICK", "TRIGGER"):
                continue
            settle = self._move_settle
            if settle is not None and settle.req_id == pending.req_id:
                if settle.stop_req_id is not None or settle.return_tolerance_mdeg is not None:
                    continue
            if frame.reply_to <= pending.req_id or frame.last_req_id < pending.req_id:
                continue
            if pending.req_id in self._cancelled_req_ids and frame.motion in ("IDLE", "FAULT"):
                self._pending.pop(pending.req_id, None)
                self._cancelled_req_ids.discard(pending.req_id)
                if settle is not None and settle.req_id == pending.req_id:
                    self._move_settle = None
                continue
            if frame.motion == "FAULT" and frame.fault_code != "NONE" and now - pending.sent_at >= 1.2:
                if settle is not None and settle.req_id == pending.req_id:
                    self._abandon_move_settle(f"STATE确认动作故障 {frame.fault_code}")
                if self._linked_req_id == pending.req_id:
                    self._cancel_linked_action(f"STATE确认动作故障 {frame.fault_code}")
                self._pending.pop(pending.req_id, None)
                self.pending_write = any(not p.acked and not p.uncertain for p in self._pending.values())
                self.command_status = f"设备故障中止：{frame.fault_code}"
                self.execution_status = self.command_status
                self.command_result.emit(pending.req_id, pending.verb, frame.fault_code)
                continue
            if (frame.motion == "IDLE" and frame.fault_code == "NONE" and
                    frame.hx8_online and frame.pos_mdeg is not None):
                if pending.verb == "XMOVE" and now - pending.sent_at < 1.5:
                    continue
                if settle is not None and settle.req_id == pending.req_id:
                    self._move_settle = None
                self.event.emit("协议", "无线DONE补偿",
                                f"req={pending.req_id} ({pending.verb}) 的 DONE 无线丢帧，已由新鲜 IDLE STATE 自动确认完成")
                self._complete_motion_done(pending, frame.pos_mdeg, now, query_state=False)

    def _confirm_experiment_return(self, frame: StateFrame, settle: MoveSettle, pending: Pending) -> bool:
        # GETs don't advance STATE.last_req_id. An ACK plus that exact ID,
        # target and repeated idle feedback can confirm a non-recording return.
        pos = frame.pos_mdeg
        tolerance = settle.return_tolerance_mdeg
        if (tolerance is None or self.voltage_locked or frame.mode != "MANUAL" or frame.motion != "IDLE" or pos is None
                or pending.req_id in self._cancelled_req_ids or frame.last_req_id != settle.req_id
                or frame.reply_to <= settle.req_id or frame.target_mdeg != settle.target_mdeg
                or abs(pos-settle.target_mdeg) > tolerance):
            settle.return_idle_since_ms = None
            settle.return_idle_anchor_mdeg = None
            return False
        anchor = settle.return_idle_anchor_mdeg
        if (settle.return_idle_since_ms is None or anchor is None or abs(pos-anchor) > HOST_SETTLE_DELTA_MDEG
                or settle.last_t_ms is None
                or (frame.t_ms-settle.last_t_ms) % (2**32) > 750):
            settle.return_idle_since_ms = frame.t_ms
            settle.return_idle_anchor_mdeg = pos
        settle.last_t_ms = frame.t_ms
        if (frame.t_ms-settle.return_idle_since_ms) % (2**32) < 500:
            return False
        self._move_settle = None
        self._pending.pop(settle.req_id, None)
        self.pending_write = False
        self.command_status = f"已通过连续空闲反馈确认回收，HX8 位置 {pos/1000:.1f}°"
        self.event.emit("设备反馈", pending.label, self.command_status)
        self.command_result.emit(settle.req_id, settle.verb, "RETURN_STATE")
        return True

    def _submit_fold_move(self) -> bool:
        if self.info is None:
            return False
        speed = max(1, round(self.info.max_speed_dps * 0.40))
        sent = self._submit("MOVE", (self.fold_target_mdeg, speed),
                            f"折叠第2步：HX8 到 {self.fold_target_mdeg / 1000:+g}°，速度 40%（{speed}°/s）",
                            manual=True, motion=True)
        if sent:
            self._linked_req_id = self._next_req - 1
            self.requested_shape = f"{self.fold_target_mdeg / 1000:+g}° 折叠运动中；开爪命令已应用"
        return sent

    def _submit_linked_open(self) -> bool:
        sent = self._submit("SERVO", (1, GRIP_TEST_ANGLE_MDEG["OPEN"]),
                            "正常／归零后张开夹爪：PWM 125°（无位置反馈）", manual=True)
        if sent:
            self._linked_req_id = self._next_req - 1
        return sent

    def request_fold(self) -> bool:
        self.last_request = MotionRequest("fold")
        if not self._require_link():
            return False
        if self.simulated:
            sent = self._submit("MOVE", (), f"模拟折叠：先开爪，再以40%速度到绝对{self.fold_target_mdeg / 1000:+g}°")
        else:
            if self.info is None or self.cal is None or self.cal.home_mdeg != 0 or self.cal.normal_mdeg != 0:
                return self._blocked(f"设备竖直零位未确认是 0°，不能发送绝对 {self.fold_target_mdeg / 1000:+g}°")
            if self.cal.extension_sign != -1:
                return self._blocked("设备出臂方向未确认是负向，不能确定折叠的正向")
            if not self._guard(manual=True, motion=True):
                return False
            self._linked_action = "FOLD"
            sent = self._submit("SERVO", (1, GRIP_TEST_ANGLE_MDEG["OPEN"]),
                                "折叠第1步：张开夹爪 PWM 125°（无位置反馈）", manual=True)
            if sent:
                self._linked_req_id = self._next_req - 1
            else:
                self._cancel_linked_action("无法提交开爪命令")
        if sent:
            self.requested_shape = f"折叠请求：等待开爪 ACK 后运动到 {self.fold_target_mdeg / 1000:+g}°"
            self.state_changed.emit()
        return sent

    def request_restore(self) -> bool:
        self.last_request = MotionRequest("restore")
        if not self._require_link():
            return False
        if self.simulated:
            sent = self._submit("MOVE", (), f"模拟正常模式：60%速度到{self.working_zero_mdeg / 1000:+g}°工作位，到位后张开夹爪")
        else:
            if self.info is None or self.cal is None or self.cal.home_mdeg != 0 or self.cal.normal_mdeg != 0:
                return self._blocked(f"设备竖直基准未确认是 0°，不能换算 {self.working_zero_mdeg / 1000:+g}°工作位")
            speed = max(1, round(self.info.max_speed_dps * 0.60))
            self._linked_action = "RESTORE"
            sent = self._submit("MOVE", (self.working_zero_mdeg, speed),
                                f"正常模式第1步：HX8 到 {self.working_zero_mdeg / 1000:+g}°工作位，速度 60%（{speed}°/s）",
                                manual=True, motion=True)
            if sent:
                self._linked_req_id = self._next_req - 1
            else:
                self._cancel_linked_action("无法提交归零运动")
        if sent:
            self.requested_shape = f"正常模式：HX8 到绝对 {self.working_zero_mdeg / 1000:+g}°工作位，到位后张开夹爪"
            self.state_changed.emit()
        return sent

    def request_home(self) -> bool:
        self.last_request = MotionRequest("home")
        if not self._require_link():
            return False
        if self.simulated:
            return self._submit("MOVE", (), f"模拟归零：60%速度到{self.working_zero_mdeg / 1000:+g}°工作位，到位后张开夹爪")
        if self.info is None or self.cal is None or self.cal.home_mdeg is None:
            return self._blocked("归零位置未标定")
        if self.cal.home_mdeg != 0:
            return self._blocked("设备归零标定未确认是 0°")
        speed = max(1, round(self.info.max_speed_dps * 0.60))
        self._linked_action = "HOME"
        sent = self._submit("MOVE", (self.working_zero_mdeg, speed),
                            f"归零第1步：HX8 到 {self.working_zero_mdeg / 1000:+g}°工作位，速度 60%（{speed}°/s）",
                            manual=True, motion=True)
        if sent:
            self._linked_req_id = self._next_req - 1
        else:
            self._cancel_linked_action("无法提交归零运动")
        return sent

    def request_visual_return(self, close_req_id: int, *, target_mdeg: int,
                              speed_percent: float = 60.0) -> bool:
        """Return HX8 to this grab's snapshotted start preset, retaining the grip PWM."""
        if not self._require_link():
            return False
        if target_mdeg not in (self.working_zero_mdeg, self.fold_target_mdeg):
            return self._blocked("抓取回程起点无效；只允许正常位或折叠位")
        if self.simulated:
            return self._submit("MOVE", (), "模拟抓取回位：保持闭爪PWM")
        if self.info is None or self.cal is None or self.cal.home_mdeg != 0 or self.cal.normal_mdeg != 0:
            return self._blocked("设备竖直基准未确认是0°，不能视觉回正")
        closed_at = None
        if (self._grip_last_ack_req_id == close_req_id and
                self._grip_last_ack_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"]):
            closed_at = self._grip_last_ack_at
        elif self._pick_close_applied_req_id == close_req_id:
            closed_at = self._pick_close_applied_at
        if closed_at is None or time.monotonic() - closed_at > 5.0:
            return self._blocked("缺少本次闭爪PWM的有效回执，不能视觉回正")
        if (self.latest_state is None or not self.latest_state.hx8_online or
                self.latest_state.motion != "IDLE" or self.latest_state.fault_code != "NONE" or
                self.latest_state.grip_cmd_mdeg != GRIP_TEST_ANGLE_MDEG["CLOSE"] or
                self.latest_state.last_req_id < close_req_id or
                self._last_state_received_at is None or
                self._last_state_received_at <= closed_at or
                time.monotonic() - self._last_state_received_at > 1.0):
            return self._blocked("缺少闭爪后的新鲜空闲反馈，不能视觉回正")
        if not 0 < speed_percent <= 100:
            return self._blocked("回正速度百分比超出范围")
        speed = max(1, round(self.info.max_speed_dps * speed_percent / 100))
        return self._submit("MOVE", (target_mdeg, speed),
                            f"抓取回位：HX8 到 +{target_mdeg / 1000:g}°，速度 {speed_percent:g}%；保持已应用的闭爪PWM",
                            manual=True)

    def request_extend(self, speed_percent: float, angle_deg: float, lead_deg: float = 4.0,
                       *, host_settle: bool = True, linked_pick: bool = False) -> bool:
        self.last_request = MotionRequest("extend", speed_percent, angle_deg)
        self.last_extend_start_mdeg = None
        label = f"手动出臂：速度 {speed_percent:g}%，伸出 {angle_deg:g}°"
        if self.simulated:
            self.last_extend_start_mdeg = self.working_zero_mdeg
            return self._submit("MOVE", (), label)
        if not self._require_link():
            return False
        if self.info is None or self.cal is None or self.cal.normal_mdeg is None or self.cal.extension_sign is None or self.cal.max_extension_mdeg is None:
            return self._blocked("缺少有效INFO/CAL，不能换算出臂方向与速度")
        if self.latest_state is None or self.latest_state.pos_mdeg is None:
            return self._blocked(
                "没有HX8实际位置反馈；请先确认正常位或折叠位"
            )
        start_position = self.latest_state.pos_mdeg
        start_base = next((base for base in (self.working_zero_mdeg, self.fold_target_mdeg)
                           if abs(start_position - base) <= MANUAL_START_TOLERANCE_MDEG), None)
        if start_base is None:
            return self._blocked(
                f"当前HX8实际位置 {start_position / 1000:.1f}°，不在正常位 {self.working_zero_mdeg / 1000:+g}°"
                f"或折叠位 {self.fold_target_mdeg / 1000:+g}°的±{MANUAL_START_TOLERANCE_MDEG / 1000:g}°内；"
                "请先点击正常模式或归零，也可先进入折叠位置"
            )
        extension = round(angle_deg * 1000)
        speed = round(self.info.max_speed_dps * speed_percent / 100)
        # The manual angle is travel from the configured normal/visual working
        # position. Using the firmware's calibrated 0° here would add an
        # unintended extra travel and can drive the arm into timeout.
        target = self.working_zero_mdeg + self.cal.extension_sign * extension
        calibrated_endpoint = self.cal.normal_mdeg + self.cal.extension_sign * self.cal.max_extension_mdeg
        crosses_calibrated_endpoint = (
            self.cal.extension_sign < 0 and target < calibrated_endpoint or
            self.cal.extension_sign > 0 and target > calibrated_endpoint
        )
        if (not 0 < speed_percent <= 100 or extension <= 0 or
                extension > MANUAL_EXTENSION_LIMIT_MDEG or extension % 100 or
                crosses_calibrated_endpoint):
            return self._blocked(
                f"速度或角度超出有效范围；出臂行程实测上限为 {MANUAL_EXTENSION_LIMIT_MDEG / 1000:g}°，"
                "还须满足设备标定端点与0.1°步进")
        label += (f"，从{'折叠' if start_base == self.fold_target_mdeg else '正常'}位"
                  f" +{start_base / 1000:g}°起步，绝对目标 {target / 1000:g}°，速度指令 {max(1, speed)}°/s")
        if linked_pick and self.pose_calibration_saved and not self._pose_synced:
            self._pose_attempted = False
            return self._blocked("抓取需要固件确认校准同步；请等同步 ACK，旧版固件须更新")
        verb = "PICK" if linked_pick else "MOVE"
        values = (target, max(1, speed), round(lead_deg * 1000)) if linked_pick else (target, max(1, speed))
        sent = self._submit(verb, values, label, manual=True, motion=True,
                            host_settle=host_settle and not linked_pick)
        if sent:
            self.last_extend_start_mdeg = start_base
        return sent

    def request_follow_move(self, speed_percent: float, angle_deg: float,
                            *, speed_dps: float | None = None, host_settle: bool = True) -> bool:
        """Move HX8 to a relative extension angle during visual following from any safe in-range pose."""
        quantized_deg = round(angle_deg * 10) / 10
        self.last_request = MotionRequest("follow", speed_percent, quantized_deg)
        speed_desc = f"{round(speed_dps)}°/s" if speed_dps is not None else f"{speed_percent:g}%"
        label = f"视觉跟随：速度 {speed_desc}，目标行程 {quantized_deg:.1f}°"
        if self.voltage_locked:
            return self._blocked("低压锁定，等待供电恢复")
        if self.simulated:
            return self._submit("MOVE", (), label)
        if not self._require_link():
            return False
        if (self.info is None or self.cal is None or self.cal.normal_mdeg is None or
                self.cal.extension_sign is None or self.cal.max_extension_mdeg is None):
            return self._blocked("缺少有效INFO/CAL，不能换算出臂方向与速度")
        if (self.latest_state is None or self.latest_state.pos_mdeg is None or
                self._last_state_received_at is None or
                time.monotonic() - self._last_state_received_at > 2.0):
            return self._blocked("缺少新鲜HX8实际位置反馈，暂不发送跟随动作")
        current_pos = self.latest_state.pos_mdeg
        ext_endpoint = self.working_zero_mdeg + self.cal.extension_sign * MANUAL_EXTENSION_LIMIT_MDEG
        allowed_low = min(self.fold_target_mdeg, ext_endpoint) - MANUAL_START_TOLERANCE_MDEG
        allowed_high = max(self.fold_target_mdeg, ext_endpoint) + MANUAL_START_TOLERANCE_MDEG
        if not allowed_low <= current_pos <= allowed_high:
            return self._blocked(
                f"当前HX8实际位置 {current_pos / 1000:.1f}° 超出安全跟随区间 "
                f"[{allowed_low / 1000:.1f}°, {allowed_high / 1000:.1f}°]；请先点击正常模式或归零"
            )
        extension = round(angle_deg * 10) * 100
        speed = round(speed_dps) if speed_dps is not None else round(self.info.max_speed_dps * speed_percent / 100)
        speed_ok = (1 <= speed <= self.info.max_speed_dps) if speed_dps is not None else (0 < speed_percent <= 100)
        target = self.working_zero_mdeg + self.cal.extension_sign * extension
        calibrated_endpoint = self.cal.normal_mdeg + self.cal.extension_sign * self.cal.max_extension_mdeg
        crosses_calibrated_endpoint = (
            self.cal.extension_sign < 0 and target < calibrated_endpoint or
            self.cal.extension_sign > 0 and target > calibrated_endpoint
        )
        if (not speed_ok or extension < 0 or
                extension > MANUAL_EXTENSION_LIMIT_MDEG or extension % 100 or
                not -90_000 <= target <= 180_000 or crosses_calibrated_endpoint):
            return self._blocked(
                f"跟随速度或角度超出有效范围；出臂行程须在 0°～{MANUAL_EXTENSION_LIMIT_MDEG / 1000:g}° 内且满足设备标定端点"
            )
        label += (f"，当前 {current_pos / 1000:+.1f}° → 绝对目标 {target / 1000:+.1f}°，"
                  f"速度指令 {max(1, speed)}°/s")
        sent = self._submit("MOVE", (target, max(1, speed)), label,
                            manual=True, motion=True, host_settle=host_settle)
        if sent and self._move_settle is not None:
            step_mdeg = abs(target - current_pos)
            if 2_000 <= step_mdeg <= HOST_SETTLE_TOLERANCE_MDEG:
                self._move_settle.tolerance_mdeg = max(1_200, step_mdeg // 2)
        return sent

    def request_mode(self, mode: str) -> bool:
        if self.voltage_locked:
            return self._blocked("低压锁定，等待供电恢复")
        if mode != "MANUAL":
            self.cancel_grip_ramp("切换工作模式")
            if not self.simulated and any(p.verb == "SERVO" for p in self._pending.values()):
                self._queued_mode = mode
                self.command_status = f"当前SERVO回执后切换 {mode}；不再发送夹爪步进"
                self.state_changed.emit()
                return True
        if mode != "VISION_PICK":
            self.visual_target_enabled = False
        if not self.simulated and any(p.verb == "STOP" for p in self._pending.values()):
            self._queued_mode = mode
            self.command_status = f"STOP回执后切换 {mode}"
            self.state_changed.emit()
            return True
        if self.simulated:
            self.mode = mode
            return self._submit("MODE", (mode,), f"工作模式 {mode}")
        if mode == self.mode:
            self.command_status = f"设备当前已是 {mode}；未重复发送MODE"
            self.event.emit("控制", "模式", self.command_status)
            self.state_changed.emit()
            return True
        # MODE changes only routing. PREP is a separate accepted/completed action.
        if (self.latest_state is not None and self.latest_state.motion != "IDLE" and
                self.mode not in ("VISION_PICK", "VISION_FOLLOW")):
            return self._blocked("当前动作未结束，不能切换工作模式")
        return self._submit("MODE", (mode,), f"切换工作模式 {mode}")

    def request_recover(self) -> bool:
        if self.voltage_locked:
            return self._blocked("低压锁定，等待供电恢复")
        if self.simulated or not self._require_link():
            return self._blocked("故障恢复需要真实设备在线")
        if self.mode != "MANUAL" or self.latest_state is None:
            return self._blocked("请在手动模式等待设备状态后恢复")
        if not self.latest_state.hx8_online:
            return self._blocked("HX8离线，请先恢复供电和通信")
        if self.latest_state.fault_code == "NONE":
            return self._blocked("设备未报告故障，无需恢复")
        if any(p.verb == "STOP" for p in self._pending.values()):
            return self._blocked("恢复／停止请求尚未确认")
        # Recovery must never resume a previously queued mode/vision action.
        self._queued_mode = None
        return self.request_stop()

    def request_stop(self) -> bool:
        self._move_settle = None
        self.cancel_grip_ramp("STOP")
        self._cancel_linked_action("STOP")
        self.visual_target_enabled = False
        self.ready_for_pick = False
        self._vision_start_requested = False
        self.last_visual_target_sent_at = None
        self._cancelled_req_ids.update(self._pending)
        return self._submit("STOP", (), "停止自动流程并保持位置", priority=True)

    def request_visual_prepare(self) -> bool:
        if self.simulated:
            self.mode = "VISION_PICK"
            return self._submit("PREP", (), "模拟视觉准备")
        if not self._require_link():
            return False
        if self.cal is None or self.cal.home_mdeg is None or self.cal.grip_open_mdeg is None or self.cal.open_wait_ms is None:
            return self._blocked("竖直位、开爪角度或开爪等待时间未标定")
        if self.mode != "VISION_PICK":
            self._vision_start_requested = True
            if not self.request_mode("VISION_PICK"):
                self._vision_start_requested = False
                return False
            return True
        self.ready_for_pick = False
        self.visual_target_enabled = False
        self.last_visual_target_sent_at = None
        return self._submit("PREP", (), "视觉抓取准备", motion=True)

    def set_visual_target_enabled(self, enabled: bool) -> None:
        self.visual_target_enabled = bool(enabled and not self.simulated and self.link_online and
                                          self.mode == "VISION_PICK" and self.ready_for_pick)

    def request_grab(self) -> bool:
        self.last_request = MotionRequest("grab")
        if self.simulated:
            return self._submit("TRIGGER", (), "模拟单次抓取请求")
        if not self.ready_for_pick or not self.visual_target_enabled or self.last_visual_target_sent_at is None or (
            time.monotonic() - self.last_visual_target_sent_at > (self.info.vision_ttl_ms / 1000 if self.info else 0.5)
        ):
            return self._blocked("视觉准备未完成或没有有效候选目标")
        self.visual_target_enabled = False
        self.last_visual_target_sent_at = None
        return self._submit("TRIGGER", (), "视觉单次联动抓取", motion=True)

    def request_grip(self, action: str) -> bool:
        if action not in GRIP_TEST_ANGLE_MDEG:
            return self._blocked("夹爪动作无效")
        if not self._require_link():
            return False
        target_mdeg = GRIP_TEST_ANGLE_MDEG[action]
        # SERVO is an absolute PWM angle command. The user installs the jaw in
        # its closed mechanical position at startup; there is no angle sensor
        # and therefore no measured zero to infer or seek in software.
        return self._submit("SERVO", (1, target_mdeg),
                            f"夹爪{'张开' if action == 'OPEN' else '闭合'}：直接发送 PWM {target_mdeg // 1000}°（无位置反馈）",
                            manual=True)

    def request_grip_install_position(self) -> bool:
        """Hold a known PWM angle while the operator fits the jaw closed."""
        if not self._require_link():
            return False
        return self._submit("SERVO", (1, GRIP_INSTALL_ANGLE_MDEG),
                            "夹爪安装基准：保持 PWM 90°（请在舵机通电保持时安装闭合位；无位置反馈）",
                            manual=True)

    def request_record(self, enabled: bool) -> bool:
        return self._submit("RECORD", (int(enabled),), "MCU记录门控")

    def submit_follow_stream_target(self, angle_deg: float, speed_dps: int, confidence: int = 95) -> bool:
        """20 Hz fire-and-forget HX8 streaming target in VISION_FOLLOW mode (never touches gripper)."""
        quantized_deg = round(angle_deg * 10) / 10
        if self.voltage_locked:
            return False
        if self.simulated:
            self.last_request = MotionRequest("follow", round(speed_dps / 500 * 100, 1), quantized_deg)
            self.submitted_requests += 1
            return True
        if (self.serial_service.state != ConnectionState.OPEN or not self.link_online or
                self.mode != "VISION_FOLLOW" or self.info is None or self.cal is None or
                self.cal.normal_mdeg is None or self.cal.extension_sign is None or
                self.cal.max_extension_mdeg is None):
            return False
        state = self.latest_state
        if state is None or not state.hx8_online or state.fault_code not in ("NONE", "STOPPED"):
            return False
        extension = round(angle_deg * 10) * 100
        if extension < 0 or extension > MANUAL_EXTENSION_LIMIT_MDEG:
            return False
        target = self.working_zero_mdeg + self.cal.extension_sign * extension
        calibrated_endpoint = self.cal.normal_mdeg + self.cal.extension_sign * self.cal.max_extension_mdeg
        crosses_calibrated_endpoint = (
            self.cal.extension_sign < 0 and target < calibrated_endpoint or
            self.cal.extension_sign > 0 and target > calibrated_endpoint
        )
        if not (-90_000 <= target <= self.fold_target_mdeg) or crosses_calibrated_endpoint:
            return False
        speed = max(1, min(self.info.max_speed_dps, round(speed_dps)))
        conf = max(self.info.vision_min_confidence, min(100, round(confidence)))
        try:
            payload = self.encoder.encode_visual(target, speed, conf)
        except ProtocolError:
            return False
        sent = self.serial_service.send(payload)
        if sent:
            self.last_request = MotionRequest(
                "follow",
                round(speed / self.info.max_speed_dps * 100, 1),
                quantized_deg,
            )
            self.submitted_requests += 1
            self.last_visual_target_sent_at = time.monotonic()
        return sent

    def request_follow_return_normal(self) -> bool:
        """Return HX8 to the calibrated normal working pose WITHOUT actuating the gripper."""
        self.last_request = MotionRequest("restore")
        if not self._require_link():
            return False
        if self.simulated:
            return self._submit("MOVE", (), f"模拟回正常位：60%速度到{self.working_zero_mdeg / 1000:+g}°工作位（不动夹爪）")
        if self.info is None or self.cal is None or self.cal.home_mdeg != 0 or self.cal.normal_mdeg != 0:
            return self._blocked(f"设备竖直基准未确认是 0°，不能换算 {self.working_zero_mdeg / 1000:+g}°工作位")
        speed = max(1, round(self.info.max_speed_dps * 0.60))
        sent = self._submit(
            "MOVE",
            (self.working_zero_mdeg, speed),
            f"视觉跟随回正常位：仅 HX8 到 {self.working_zero_mdeg / 1000:+g}°工作位，速度 60%（{speed}°/s，不操作夹爪）",
            manual=True,
            motion=True,
        )
        if sent:
            self.requested_shape = f"视觉跟随回正常位：HX8 到绝对 {self.working_zero_mdeg / 1000:+g}°工作位（保持夹爪不变）"
            self.state_changed.emit()
        return sent

    def submit_visual_target(self, angle_deg: float, speed_dps: int, confidence: int) -> bool:
        """未来视觉算法的目标输入；仅在确认VISION_PICK后允许发送。"""
        if (self.voltage_locked or self.simulated or not self.visual_target_enabled or not self.link_online
                or self.mode != "VISION_PICK" or self.info is None):
            return False
        if confidence < self.info.vision_min_confidence:
            return False
        try:
            payload = self.encoder.encode_visual(round(angle_deg * 1000), speed_dps, confidence)
        except ProtocolError:
            return False
        sent = self.serial_service.send(payload)
        if sent:
            self.last_visual_target_sent_at = time.monotonic()
        return sent

    def _blocked(self, message: str) -> bool:
        self.command_status = message
        self.event.emit("控制", "拦截", message)
        self.state_changed.emit()
        return False

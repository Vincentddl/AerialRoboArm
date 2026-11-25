"""HC13 V2: strict ASCII framing, typed telemetry, and command encoding."""
from __future__ import annotations

import base64
import binascii
from dataclasses import dataclass
import re
from typing import TypeAlias
import zlib

from .models import AccelerationSample

U32 = 0xFFFFFFFF
MAX_PAYLOAD = 160
CAPTURE_PAGE_BYTES = 60
CAPTURE_ACC_BYTES = 8
CAPTURE_HX_BYTES = 6
MODES = {"MANUAL", "VISION_DEMO", "VISION_PICK", "VISION_FOLLOW"}
SHAPES = {"UNKNOWN", "NORMAL", "FOLDED"}
MOTIONS = {"IDLE", "MOVING", "SETTLING", "FAULT"}
GRIP_ACTIONS = {"UNKNOWN", "OPEN_CMD", "CLOSE_CMD", "ANGLE_CMD"}
ERRORS = {
    "BAD_FRAME", "BAD_NUMBER", "OUT_OF_RANGE", "NOT_READY", "BUSY",
    "MODE_DENIED", "NOT_CALIBRATED", "HX8_OFFLINE", "IMU_OFFLINE",
    "IMU_RATE_LOW", "FAULT", "MOTION_TIMEOUT", "FEEDBACK_STALE",
    "VISION_STALE", "LINK_LOST", "STOPPED", "REQUEST_CONFLICT",
    "STALE_REQUEST", "UNSUPPORTED",
}
VERBS = {"POSE", "MOVE", "XMOVE", "HOME", "STOP", "MODE", "SHAPE", "GRIP", "SERVO", "RECORD", "PREP", "PICK", "TRIGGER"}


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class InvalidFrame:
    reason: str


@dataclass(frozen=True)
class AccFrame:
    seq: int
    sample_ms: int
    sample: AccelerationSample


@dataclass(frozen=True)
class StateFrame:
    t_ms: int
    mode: str
    shape: str
    motion: str
    hx8_online: bool
    imu_online: bool
    pos_mdeg: int | None
    target_mdeg: int | None
    voltage_mv: int | None
    imu_hz: int
    record_on: bool
    acc_tx: bool
    grip_cmd_mdeg: int | None
    grip_action: str
    last_req_id: int
    fault_code: str
    reply_to: int


@dataclass(frozen=True)
class InfoFrame:
    req_id: int
    version: int
    boot_id: int
    last_req_id: int
    uart_baud: int
    imu_target_hz: int
    acc_tx_hz: int
    max_speed_dps: int
    ramp_ms: int
    post_ms: int
    vision_min_confidence: int
    vision_ttl_ms: int


@dataclass(frozen=True)
class ExperimentInfoFrame:
    req_id: int
    version: int
    min_ramp_ms: int
    max_ramp_ms: int


@dataclass(frozen=True)
class ImuDiagFrame:
    req_id: int
    rx_bytes: int
    good_frames: int
    checksum_errors: int
    uart_errors: int
    rearm_failures: int
    recovery_stage: int
    imu_hz: int


@dataclass(frozen=True)
class CaptureInfoFrame:
    req_id: int


@dataclass(frozen=True)
class CapturePageFrame:
    req_id: int
    move_id: int
    page: int
    acc_count: int
    hx_count: int
    start_ms: int
    end_ms: int
    flags: int
    data: bytes


@dataclass(frozen=True)
class CalFrame:
    req_id: int
    home_mdeg: int | None
    normal_mdeg: int | None
    extension_sign: int | None
    max_extension_mdeg: int | None
    folded_mdeg: int | None
    grip_open_mdeg: int | None
    grip_close_mdeg: int | None
    servo1_min_mdeg: int | None
    servo1_max_mdeg: int | None
    open_wait_ms: int | None = None
    close_wait_ms: int | None = None
    default_lead_mdeg: int | None = None


@dataclass(frozen=True)
class AckFrame:
    req_id: int
    verb: str
    result: str


@dataclass(frozen=True)
class DoneFrame:
    req_id: int
    verb: str
    actual_pos_mdeg: int


@dataclass(frozen=True)
class MotionFrame:
    req_id: int
    stage: str
    t_ms: int
    target_mdeg: int | None
    pos_mdeg: int | None
    reason: str


@dataclass(frozen=True)
class Hx8DiagFrame:
    t_ms: int
    feedback_age_ms: int
    last_pos_mdeg: int | None
    last_voltage_mv: int
    last_current_ma: int
    last_status: int
    cause: str


@dataclass(frozen=True)
class ErrFrame:
    req_id: int
    code: str


@dataclass(frozen=True)
class PongFrame:
    req_id: int
    t_ms: int


@dataclass(frozen=True)
class OptionalImuFrame:
    kind: str
    seq: int
    sample_ms: int
    xyz: tuple[int, int, int]


Frame: TypeAlias = (AccFrame | StateFrame | InfoFrame | ExperimentInfoFrame | ImuDiagFrame | CalFrame | AckFrame |
                    DoneFrame | MotionFrame | Hx8DiagFrame | ErrFrame | PongFrame |
                    OptionalImuFrame | CaptureInfoFrame | CapturePageFrame | InvalidFrame)


def integer(value: str, minimum: int, maximum: int, *, na: bool = False) -> int | None:
    if na and value == "NA":
        return None
    if not re.fullmatch(r"-?[0-9]+", value):
        raise ProtocolError("非法十进制整数")
    number = int(value)
    if not minimum <= number <= maximum:
        raise ProtocolError("数字超出协议范围")
    return number


def enum(value: str, allowed: set[str]) -> str:
    if value not in allowed:
        raise ProtocolError("枚举字段无效")
    return value


def count(parts: list[str], expected: int) -> None:
    if len(parts) != expected:
        raise ProtocolError("字段数量错误")


def flag(value: str) -> bool:
    return bool(integer(value, 0, 1))


class V1Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.discarding = False
        self._acc_epoch_ms = 0
        self._last_acc_ms: int | None = None

    def reset(self) -> None:
        self.buffer.clear()
        self.discarding = False
        self._acc_epoch_ms = 0
        self._last_acc_ms = None

    def feed(self, data: bytes, received_monotonic_s: float) -> list[Frame]:
        result: list[Frame] = []
        for byte in data:
            if byte == 10:
                if self.discarding:
                    self.discarding = False
                    self.buffer.clear()
                    continue
                payload = bytes(self.buffer)
                self.buffer.clear()
                if payload.endswith(b"\r"):
                    payload = payload[:-1]
                try:
                    result.append(self._parse(payload))
                except ProtocolError as exc:
                    result.append(InvalidFrame(str(exc)))
            elif not self.discarding:
                self.buffer.append(byte)
                if len(self.buffer) > MAX_PAYLOAD + 1:
                    self.buffer.clear()
                    self.discarding = True
                    result.append(InvalidFrame("帧超过160字节，丢弃至下一换行"))
        return result

    def _parse(self, payload: bytes) -> Frame:
        if not payload or len(payload) > MAX_PAYLOAD:
            raise ProtocolError("空帧或超长帧")
        try:
            line = payload.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ProtocolError("非ASCII字符") from exc
        if any(ch.isspace() for ch in line) or "\x00" in line:
            raise ProtocolError("帧中包含空白或控制字符")
        p = line.split(",")
        if any(field == "" for field in p):
            raise ProtocolError("字段不可为空")
        k = p[0]
        if k == "ACC":
            count(p, 6)
            seq = integer(p[1], 0, U32)
            t = integer(p[2], 0, U32)
            axes = [integer(v, -16000, 16000) for v in p[3:6]]
            if self._last_acc_ms is not None and t < self._last_acc_ms and self._last_acc_ms - t > 0x80000000:
                self._acc_epoch_ms += U32 + 1
            self._last_acc_ms = t
            sample = AccelerationSample((self._acc_epoch_ms + t) / 1000.0,
                                        *(v / 1000.0 for v in axes), "g（含重力）", "HC13 ACC", "MCU样本时间")
            return AccFrame(seq, t, sample)
        if k == "STATE":
            count(p, 18)
            return StateFrame(
                integer(p[1], 0, U32), enum(p[2], MODES), enum(p[3], SHAPES),
                enum(p[4], MOTIONS), flag(p[5]), flag(p[6]),
                integer(p[7], -2147483648, 2147483647, na=True),
                integer(p[8], -2147483648, 2147483647, na=True),
                integer(p[9], 0, 65535, na=True), integer(p[10], 0, 1000),
                flag(p[11]), flag(p[12]), integer(p[13], 0, 180000, na=True),
                enum(p[14], GRIP_ACTIONS), integer(p[15], 0, U32),
                enum(p[16], ERRORS | {"NONE"}), integer(p[17], 0, U32),
            )
        if k == "INFO":
            count(p, 13)
            n = [integer(v, 0, U32) for v in p[1:]]
            if n[0] == 0 or n[1] != 2 or n[2] == 0 or not (1 <= n[7] <= 500) or n[10] > 100:
                raise ProtocolError("INFO版本或范围无效")
            return InfoFrame(*n)
        if k == "CAL":
            count(p, 14)
            req = integer(p[1], 1, U32)
            values = [
                integer(p[2], -90000, 180000, na=True),
                integer(p[3], -90000, 180000, na=True),
                integer(p[4], -1, 1, na=True),
                integer(p[5], 0, 180000, na=True),
                integer(p[6], -90000, 180000, na=True),
                integer(p[7], 0, 180000, na=True),
                integer(p[8], 0, 180000, na=True),
                integer(p[9], 0, 180000, na=True),
                integer(p[10], 0, 180000, na=True),
                integer(p[11], 0, 60000, na=True),
                integer(p[12], 0, 60000, na=True),
                integer(p[13], 3000, 5000, na=True),
            ]
            if values[2] not in (-1, 1, None):
                raise ProtocolError("CAL方向无效")
            if values[7] is not None and values[8] is not None and values[7] > values[8]:
                raise ProtocolError("CAL小舵机范围无效")
            return CalFrame(req, *values)
        if k == "EXPERIMENT":
            count(p, 5)
            req, version, low, high = (integer(v, 0, U32) for v in p[1:])
            if not req or version != 1 or low != 20 or high != 2000:
                raise ProtocolError("实验固件能力无效")
            return ExperimentInfoFrame(req, version, low, high)
        if k == "IMU":
            count(p, 9)
            values = [integer(v, 0, U32) for v in p[1:]]
            if values[0] == 0 or values[6] > 3 or values[7] > 1000:
                raise ProtocolError("IMU诊断字段无效")
            return ImuDiagFrame(*values)
        if k == "CAPTURE":
            count(p, 3)
            integer(p[2], 1, 1)
            return CaptureInfoFrame(integer(p[1], 1, U32))
        if k == "CAPDATA":
            count(p, 11)
            body, _, checksum = payload.rpartition(b",")
            if not re.fullmatch(r"[0-9a-fA-F]{8}", p[-1]) or zlib.crc32(body) != int(checksum, 16):
                raise ProtocolError("高频数据分包 CRC 错误")
            counts = (integer(p[4], 0, 400), integer(p[5], 0, 200))
            total = counts[0] * CAPTURE_ACC_BYTES + counts[1] * CAPTURE_HX_BYTES
            page = integer(p[3], 0, max(0, (total - 1) // CAPTURE_PAGE_BYTES))
            try:
                data = b"" if p[9] == "-" else base64.b64decode(p[9], validate=True)
            except binascii.Error as exc:
                raise ProtocolError("高频分包编码无效") from exc
            if len(data) != min(CAPTURE_PAGE_BYTES, total - page * CAPTURE_PAGE_BYTES):
                raise ProtocolError("高频分包长度无效")
            return CapturePageFrame(integer(p[1], 1, U32), integer(p[2], 1, U32), page, *counts,
                                    integer(p[6], 0, U32), integer(p[7], 0, U32),
                                    integer(p[8], 0, 7), data)
        if k == "ACK":
            count(p, 4)
            verb = enum(p[2], VERBS)
            allowed = {"POSE": {"APPLIED"}, "MOVE": {"ACCEPTED"}, "XMOVE": {"ACCEPTED"}, "HOME": {"ACCEPTED"},
                       "SHAPE": {"ACCEPTED"}, "PREP": {"ACCEPTED"},
                       "PICK": {"ACCEPTED"}, "TRIGGER": {"ACCEPTED"}, "MODE": MODES,
                       "GRIP": {"APPLIED"}, "SERVO": {"APPLIED"},
                       "RECORD": {"ON", "OFF"}, "STOP": {"APPLIED"}}[verb]
            return AckFrame(integer(p[1], 1, U32), verb, enum(p[3], allowed))
        if k == "DONE":
            count(p, 4)
            return DoneFrame(integer(p[1], 1, U32), enum(p[2], {"MOVE", "XMOVE", "HOME", "SHAPE", "PREP", "PICK", "TRIGGER"}),
                             integer(p[3], -2147483648, 2147483647))
        if k == "MOTION":
            count(p, 7)
            return MotionFrame(integer(p[1], 1, U32),
                               enum(p[2], {"START", "ARRIVED", "END", "ABORT", "PREPARING", "OPEN_APPLIED", "READY", "CLOSE_APPLIED"}),
                               integer(p[3], 0, U32),
                               integer(p[4], -2147483648, 2147483647, na=True),
                               integer(p[5], -2147483648, 2147483647, na=True),
                               enum(p[6], ERRORS | {"NONE"}))
        if k == "HX8_DIAG":
            count(p, 8)
            return Hx8DiagFrame(
                integer(p[1], 0, U32), integer(p[2], 0, U32),
                integer(p[3], -2147483648, 2147483647, na=True),
                integer(p[4], 0, 65535), integer(p[5], -32768, 32767),
                integer(p[6], 0, 255),
                enum(p[7], {"NO_SAMPLE", "AGE", "ANGLE", "VOLT_LOW",
                            "VOLT_HIGH", "STATUS", "HX8_OK_OR_OTHER"}),
            )
        if k == "ERR":
            count(p, 3)
            return ErrFrame(integer(p[1], 0, U32), enum(p[2], ERRORS))
        if k == "PONG":
            count(p, 3)
            return PongFrame(integer(p[1], 1, U32), integer(p[2], 0, U32))
        if k in ("GYRO", "ATT"):
            count(p, 6)
            return OptionalImuFrame(k, integer(p[1], 0, U32), integer(p[2], 0, U32),
                                    tuple(integer(v, -2147483648, 2147483647) for v in p[3:6]))
        raise ProtocolError("未知帧类型")


class V1Encoder:
    @staticmethod
    def encode(verb: str, req_id: int, *values: str | int) -> bytes:
        integer(str(req_id), 1, U32)
        if verb in ("MOVE", "XMOVE"):
            if len(values) not in ((3, 4) if verb == "XMOVE" else (2,)):
                raise ProtocolError(f"{verb}参数数量错误")
            target = integer(str(values[0]), -90000, 180000)
            speed = integer(str(values[1]), 1, 500)
            if target % 100:
                raise ProtocolError(f"{verb}目标须为0.1°步进")
            fields = (target, speed, integer(str(values[2]), 20, 2000)) if verb == "XMOVE" else (target, speed)
            if len(values) == 4:
                fields += (integer(str(values[3]), 1, 1),)
        elif verb == "POSE":
            if len(values) != 2:
                raise ProtocolError("POSE参数数量错误")
            normal = integer(str(values[0]), -16000, 180000)
            folded = integer(str(values[1]), normal + 10000, 180000)
            if normal % 100 or folded % 100:
                raise ProtocolError("POSE目标须为0.1°步进")
            fields = (normal, folded)
        elif verb in ("HOME", "STOP", "PREP", "TRIGGER"):
            if values:
                raise ProtocolError("该命令不接受参数")
            fields = ()
        elif verb == "MODE":
            fields = (enum(str(values[0]), MODES),) if len(values) == 1 else None
        elif verb == "SHAPE":
            fields = (enum(str(values[0]), {"NORMAL", "FOLDED"}),) if len(values) == 1 else None
        elif verb == "GRIP":
            fields = (enum(str(values[0]), {"OPEN", "CLOSE"}),) if len(values) == 1 else None
        elif verb == "SERVO":
            if len(values) != 2 or str(values[0]) != "1":
                raise ProtocolError("SERVO仅支持通道1")
            angle = integer(str(values[1]), 0, 180000)
            if angle % 1000:
                raise ProtocolError("SERVO角度须为1°步进")
            fields = (1, angle)
        elif verb == "RECORD":
            fields = (integer(str(values[0]), 0, 1),) if len(values) == 1 else None
        elif verb == "GET":
            if len(values) == 3 and values[0] == "CAPTURE":
                fields = ("CAPTURE", integer(str(values[1]), 1, U32), integer(str(values[2]), 0, 65535))
            else:
                fields = (enum(str(values[0]), {"INFO", "CAL", "STATE", "EXPERIMENT", "CAPTURE", "IMU"}),) if len(values) == 1 else None
        elif verb == "PING":
            fields = () if not values else None
        elif verb == "PICK":
            if len(values) != 3:
                raise ProtocolError("PICK参数数量错误")
            target = integer(str(values[0]), -90000, 180000)
            speed = integer(str(values[1]), 1, 500)
            lead = integer(str(values[2]), 3000, 5000)
            if target % 100:
                raise ProtocolError("PICK目标须为0.1°步进")
            fields = (target, speed, lead)
        else:
            raise ProtocolError("未知指令")
        if fields is None:
            raise ProtocolError("参数数量错误")
        line = ",".join(str(v) for v in (verb, req_id, *fields))
        if len(line) > MAX_PAYLOAD:
            raise ProtocolError("指令超长")
        return (line + "\n").encode("ascii")

    @staticmethod
    def encode_visual(angle_deg: int, speed_dps: int, confidence: int) -> bytes:
        angle = integer(str(angle_deg), -90000, 180000)
        speed = integer(str(speed_dps), 1, 500)
        conf = integer(str(confidence), 0, 100)
        return f"V,{angle},{speed},{conf}\n".encode("ascii")

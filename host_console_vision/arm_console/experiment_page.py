"""Measured one-shot arm experiments. All acceleration figures are in g, gravity included."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
from datetime import datetime
from dataclasses import asdict
from functools import lru_cache
import json
from pathlib import Path
import random
import statistics
import struct
import time
import uuid

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (QComboBox, QFormLayout, QHBoxLayout, QLabel, QMessageBox,
                               QPushButton, QSpinBox, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget, QHeaderView)

from .v1_protocol import (CAPTURE_ACC_BYTES, CAPTURE_HX_BYTES, CAPTURE_PAGE_BYTES, AccFrame, CapturePageFrame,
                          MotionFrame, OptionalImuFrame, StateFrame)
from .v1_state import EXPERIMENT_SPEED_LEVELS, MAX_OPERATOR_SPEED_PERCENT, ExperimentController, WORKING_ZERO_MDEG


MAX_TRAVEL_DEG = 70.0
STRUCTURE_SPEEDS = {"低速": 25, "中速": 45, "高速": MAX_OPERATOR_SPEED_PERCENT}
GEAR_NAMES = ("1 最短", "2 较短", "3 中间", "4 较长", "5 最长")
VERTICAL_GEAR_POSITIONS = {
    "3 中间": (3, 3),
    "主控臂 3 挡 · 其他臂 4 挡": (3, 4),
    "主控臂 4 挡 · 其他臂 3 挡": (4, 3),
    "4 较长": (4, 4),
}
TRAVELS = {"短": round(MAX_TRAVEL_DEG / 3, 1),
           "中": round(MAX_TRAVEL_DEG * 2 / 3, 1), "长": MAX_TRAVEL_DEG}
SIGNAL_GEAR = "主控臂 3 挡 · 其他臂 4 挡"
SIGNAL_RAMP_TIMES = tuple(range(60, 201, 20))
SIGNAL_TIME_RATIOS = (40, 55, 70)
SIGNAL_ROUNDS = 4
SIGNAL_PLAN_VERSION = 1
SIGNAL_RETEST_STAGE = 4
HX8_POWER_PROTECTION = "HX8_POWER_PROTECTION"
HIGH_RATE_REVIEW = "HIGH_RATE_REVIEW"
DATA_PATH = Path(__file__).resolve().parent.parent / "runtime" / "experiment_records.json"
MANUAL_RETEST_LOG_PATH = DATA_PATH.with_name("experiment_manual_retest_diagnostics.jsonl")
VERTICAL_DATA_PATH = DATA_PATH.with_name("experiment_vertical_records.json")
BASELINE_SECONDS = 1.0
POST_SECONDS = 2.0
VERTICAL_START_TOLERANCE_MDEG = 2000
CAPTURE_PAGE_TIMEOUT = 0.8
CAPTURE_MAX_RETRIES = 4
CAPTURE_TRANSFER_TIMEOUT = 60.0
CAPTURE_MIN_ACC_HZ = 160
CAPTURE_MAX_ACC_GAP_MS = 20
BATCH_STABLE_MS = 1000
BATCH_ACC_LIMIT_MG = 6
BATCH_MAX_ACC_GAP_MS = 125
BATCH_RECOVERY_TIMEOUT = 6.0
BATCH_MAX_RETRIES = 2
BATCH_REARM_SECONDS = 0.75
TraceFrame = AccFrame | MotionFrame | OptionalImuFrame | StateFrame


def target_for_travel(travel_deg: float, normal_mdeg: int = WORKING_ZERO_MDEG) -> int:
    return normal_mdeg - round(travel_deg * 10) * 100


@dataclass(frozen=True)
class BatchCondition:
    name: str
    travel_deg: float
    speed_percent: int
    ramp_ms: int = 100
    time_ratios: tuple[int, ...] = ()
    signal_round: int | None = None

    @property
    def key(self) -> tuple[str, int, int]:
        return self.name, self.speed_percent, self.ramp_ms


@lru_cache(maxsize=16)
def signal_conditions(normal_mdeg: int, fold_mdeg: int, max_speed_dps: int,
                      round_number: int) -> tuple[BatchCondition, ...]:
    conditions = []
    for name, travel in TRAVELS.items():
        distance_mdeg = fold_mdeg-target_for_travel(travel, normal_mdeg)
        for speed in STRUCTURE_SPEEDS.values():
            velocity = max(1, round(max_speed_dps*speed/100))
            ramp_ratios: dict[int, list[int]] = {ramp: [] for ramp in SIGNAL_RAMP_TIMES}
            for ratio in SIGNAL_TIME_RATIOS:
                # t = (q / (2-q)) * (D/v). Round to the nearest 20 ms, ties upward.
                denominator = (200-ratio)*velocity
                ramp = ((distance_mdeg*ratio+10*denominator)//(20*denominator))*20
                if not 20 <= ramp <= 2000:
                    raise ValueError("校准角差和速度换算出的斜坡超出固件 20–2000 ms 范围")
                ramp_ratios.setdefault(ramp, []).append(ratio)
            conditions.extend(BatchCondition(name, travel, speed, ramp, tuple(ratios))
                              for ramp, ratios in sorted(ramp_ratios.items()))
    random.Random(round_number).shuffle(conditions)
    return tuple(conditions)


def batch_conditions(kind: int) -> tuple[BatchCondition, ...]:
    if kind == 2:
        return tuple(BatchCondition("长", MAX_TRAVEL_DEG, STRUCTURE_SPEEDS[name]) for name in ("中速", "高速"))
    return tuple(BatchCondition(name, travel, speed)
                 for name, travel in TRAVELS.items() for speed in STRUCTURE_SPEEDS.values())


@dataclass
class AutoBatch:
    id: str
    conditions: tuple[BatchCondition, ...]
    repeats: int
    boot_id: int
    gear: str | None
    normal_mdeg: int
    fold_mdeg: int
    max_speed_dps: int
    signal_round: int | None = None
    signal_retest: bool = False
    already_saved: int = 0
    phase: str = "route"
    running: bool = True
    index: int = 0
    valid: int = 0
    power_protected: bool = False
    capture_deferred: bool = False
    retries: int = 0
    rejected_moves: int = 0
    return_confirmed: bool = False
    return_corrected: bool = False
    recovery_used: bool = False
    resume_after_recovery: bool = False
    fault_ms: int | None = None
    rearm_at: float = 0.0
    await_state: int | None = None
    verify_after_ms: int | None = None
    stable_state_ms: int | None = None
    stable_pos_mdeg: int | None = None
    recovery_deadline: float | None = None
    acc: deque[tuple[float, AccFrame]] = field(default_factory=deque)

    @property
    def completed(self) -> int:
        return self.already_saved + self.index * self.repeats + self.valid + int(self.power_protected or self.capture_deferred)

    @property
    def condition_complete(self) -> bool:
        return self.valid == self.repeats or self.power_protected or self.capture_deferred

    @property
    def total(self) -> int:
        return self.already_saved + len(self.conditions) * self.repeats


@dataclass
class ManualStartStability:
    state_ms: int | None = None
    pos_mdeg: int | None = None
    acc: deque[tuple[float, AccFrame]] = field(default_factory=deque)


def batch_capture_valid(record: dict) -> bool:
    end = record["end_pos_mdeg"]
    if record["terminal"] not in ("END", "SETTLED") or end is None or abs(end-record["target_mdeg"]) > 3000:
        return False
    capture = record.get("high_rate_capture", {})
    if record.get("signal_round") is not None:
        if capture.get("status") != "ok" and not (
                capture.get("status") == "review" and capture.get("flags") == 1
                and capture.get("issues") == ["高频窗口达到板载容量或时限"]):
            return False
        acc = capture.get("acc", [])
        angles = capture.get("hx8", [])
        if not acc or not angles or angles[0]["t_rel_ms"] > 0:
            return False
        covered = [row for row in angles if 0 < row["t_rel_ms"] <= acc[-1]["t_rel_ms"]]
        tail = covered[-3:]
        # Confirm that both streams span the moving interval. The STOP/hold
        # notification may follow the end of actual movement by over a second.
        return (len(tail) == 3 and tail[-1]["t_rel_ms"]-tail[0]["t_rel_ms"] >= 15 and
                all(abs(row["pos_mdeg"]-end) <= 300 for row in tail))
    if capture.get("status") == "ok":
        return True
    if (capture.get("status") != "review" or capture.get("flags") != 1 or
            capture.get("issues") != ["高频窗口达到板载容量或时限"]):
        return False
    # The board may fill while the PC waits for STOP after the arm has settled.
    # Count this only when both streams cover the last 200 ms of verified arm stability.
    angle = capture["hx8"]
    acc = capture["acc"]
    if not angle or not acc:
        return False
    tail_end = min(angle[-1]["t_rel_ms"], acc[-1]["t_rel_ms"])
    tail = [row for row in angle if tail_end-220 <= row["t_rel_ms"] <= tail_end]
    return bool(len(tail) >= 15 and tail[-1]["t_rel_ms"]-tail[0]["t_rel_ms"] >= 200 and
                all(abs(row["pos_mdeg"]-end) <= 300 for row in tail))


@dataclass
class ActiveRun:
    req_id: int
    params: dict
    queued_at: float
    record: bool = True
    tx_at: float | None = None
    tx_wall: str | None = None
    end_at: float | None = None
    end_mcu_ms: int | None = None
    end_pos_mdeg: int | None = None
    samples: list[tuple[float, AccFrame]] = field(default_factory=list)
    terminal: str | None = None
    trace: list[tuple[float, TraceFrame]] | None = None
    post_until: float | None = None
    outcome: str | None = None
    high_rate: bool = False
    download: CaptureDownload | None = None
    high_rate_result: dict | None = None
    batch_id: str | None = None
    failure_code: str | None = None
    motion_started: bool = False
    motion_confirmed: bool = False
    imu_diag_before: dict | None = None


@dataclass
class CaptureDownload:
    started_at: float
    page: int = 0
    request_id: int | None = None
    sent_at: float = 0.0
    attempts: int = 0
    metadata: tuple[int, int, int, int, int] | None = None
    data: bytearray = field(default_factory=bytearray)
    request_log: list[dict] = field(default_factory=list)
    ignored_pages: int = 0

    def add(self, frame: CapturePageFrame) -> None:
        metadata = frame.acc_count, frame.hx_count, frame.start_ms, frame.end_ms, frame.flags
        if self.metadata is not None and self.metadata != metadata:
            raise ValueError("设备分包的采样元数据不一致")
        self.metadata = metadata
        self.data.extend(frame.data)
        self.page += 1
        self.attempts = 0

    def result(self) -> dict:
        acc_count, hx_count, start, end, flags = self.metadata
        acc_end = acc_count * CAPTURE_ACC_BYTES

        def relative(tick: int) -> int:
            return ((tick - start + 32768) % 65536) - 32768

        acc = [{"t_rel_ms": relative(t), "x_g": x/1000, "y_g": y/1000, "z_g": z/1000}
               for t, x, y, z in struct.iter_unpack("<Hhhh", self.data[:acc_end])]
        hx8 = [{"t_rel_ms": relative(t), "pos_mdeg": pos}
               for t, pos in struct.iter_unpack("<Hi", self.data[acc_end:])]

        def rate(rows: list[dict]) -> tuple[float | None, int | None]:
            ticks = [row["t_rel_ms"] for row in rows if row["t_rel_ms"] >= 0]
            intervals = [b-a for a, b in zip(ticks, ticks[1:])]
            if any(gap < 0 for gap in intervals):
                raise ValueError("设备采样时间顺序异常")
            duration = sum(intervals)
            return (1000*len(intervals)/duration if duration > 0 else None, max(intervals, default=None))

        acc_hz, acc_gap = rate(acc)
        hx_hz, hx_gap = rate(hx8)
        baseline_count = sum(row["t_rel_ms"] < 0 for row in acc)
        issues = []
        if flags & 1:
            issues.append("高频窗口达到板载容量或时限")
        if flags & 4:
            issues.append("出臂期间板端 IMU 断流")
        elif flags & 2:
            issues.append("采样中出现 IMU 校验或 UART 恢复")
        if baseline_count < 20:
            issues.append("高频静止基线不足")
        if acc_hz is None or acc_hz < CAPTURE_MIN_ACC_HZ or acc_gap > CAPTURE_MAX_ACC_GAP_MS:
            issues.append("加速度采样率偏低或存在长间隔")
        if hx_hz is None or hx_hz < 75 or hx_gap > 35:
            issues.append("HX8 回报率偏低或存在长间隔")
        return {"schema_version": 1, "source": "MCU buffered capture", "status": "review" if issues else "ok",
                "start_mcu_ms": start, "end_mcu_ms": end, "flags": flags, "issues": issues,
                "baseline_acc_count": baseline_count, "acc_hz": acc_hz, "hx8_hz": hx_hz,
                "max_acc_gap_ms": acc_gap, "max_hx8_gap_ms": hx_gap, "acc": acc, "hx8": hx8}


class ExperimentPage(QWidget):
    capture_changed = Signal()

    def __init__(self, controller: ExperimentController, *, minimum_voltage_mv: int = 10_600):
        super().__init__()
        self.controller = controller
        self.records: list[dict] = []
        self.vertical_records: list[dict] = []
        self._baseline: deque[tuple[float, TraceFrame]] = deque()
        self._records_load_error = ""
        self._vertical_load_error = ""
        self.active: ActiveRun | None = None
        self.batch: AutoBatch | None = None
        self.manual_stability = ManualStartStability()
        self._minimum_voltage_mv = minimum_voltage_mv
        self._completion_dialog: QMessageBox | None = None
        self._recovery_req_id: int | None = None
        self._recovery_query_id: int | None = None
        self._recovery_requested_at: float | None = None
        self._load()
        root = QVBoxLayout(self)
        root.addWidget(QLabel("实验模式｜仅记录出臂；回收只执行动作；补偿臂挡位仅用于记录，不改变控制信号"))
        self.start_hint = QLabel()
        self.start_hint.setWordWrap(True)
        root.addWidget(self.start_hint)
        self.form = form = QFormLayout()
        self.kind = QComboBox()
        self.kind.addItems(("结构测试", "信号测试", "结构测试 · 竖直起步"))
        form.addRow("实验类别", self.kind)
        self.gear = QComboBox()
        self.gear.addItems(GEAR_NAMES)
        form.addRow("补偿臂挡位（记录用）", self.gear)
        self.travel = QComboBox()
        for label, degrees in TRAVELS.items():
            self.travel.addItem(label, (label, degrees))
        form.addRow("行程", self.travel)
        self.structure_speed = QComboBox()
        for name, percent in STRUCTURE_SPEEDS.items():
            self.structure_speed.addItem(f"{name} · {percent}%", percent)
        form.addRow("结构测试速度", self.structure_speed)
        self.vertical_gear = QComboBox()
        self.vertical_gear.addItems(list(VERTICAL_GEAR_POSITIONS))
        form.addRow("补偿臂挡位组合（记录用）", self.vertical_gear)
        self.vertical_speed = QComboBox()
        for name in ("中速", "高速"):
            percent = STRUCTURE_SPEEDS[name]
            self.vertical_speed.addItem(f"{name} · {percent}%", percent)
        form.addRow("竖直起步速度", self.vertical_speed)
        self.signal_round = QComboBox()
        for number in range(1, SIGNAL_ROUNDS+1):
            self.signal_round.addItem(f"第 {number} 轮", number)
        self.signal_round.addItem("第四轮补测（第 1–4 轮所有待补测，排除功率过载）", 0)
        self.signal_round.addItem("手动补测（保留全部失败记录）", -1)
        form.addRow("信号测试轮次", self.signal_round)
        self.signal_progress = QLabel()
        form.addRow("本轮采集进度", self.signal_progress)
        self.manual_condition = QComboBox()
        form.addRow("手动补测工况", self.manual_condition)
        self.manual_ready_label = QLabel()
        form.addRow("起点停稳提示", self.manual_ready_label)
        root.addLayout(form)
        self.firmware_label = QLabel()
        root.addWidget(self.firmware_label)
        buttons = QHBoxLayout()
        self.extend_button = QPushButton("出臂并记录")
        self.auto_button = QPushButton("自动采集")
        self.auto_button.setToolTip("每批只测当前补偿臂挡位。请先手动调好并选择对应挡位；程序只切换出臂速度和行程。")
        self.auto_count = QSpinBox()
        self.auto_count.setRange(1, 100)
        self.auto_count.setValue(12)
        self.auto_count.setSuffix(" 次／工况")
        self.auto_end_button = QPushButton("结束批次")
        self.defer_capture_button = QPushButton("暂缓本工况")
        self.defer_capture_button.setToolTip("出臂到位但高频记录反复未通过时，列为待补测并继续本轮。")
        self.return_button = QPushButton("回收到折叠位")
        self.recover_button = QPushButton("故障恢复")
        self.recover_button.setToolTip("停止并保持，等待新鲜 STATE 确认故障清除；不自动运动或操作夹爪")
        buttons.addWidget(self.extend_button)
        buttons.addWidget(self.auto_button)
        buttons.addWidget(self.auto_count)
        buttons.addWidget(self.auto_end_button)
        buttons.addWidget(self.defer_capture_button)
        buttons.addWidget(self.return_button)
        buttons.addWidget(self.recover_button)
        buttons.addStretch()
        root.addLayout(buttons)
        self.fault_status = QLabel("设备故障：等待状态反馈")
        root.addWidget(self.fault_status)
        self.status = QLabel("等待真实设备反馈")
        self.status.setWordWrap(True)
        root.addWidget(self.status)
        root.addWidget(QLabel("出臂记录（g，含重力；X/Y/Z 均值保留方向；|X|+|Y|+|Z| 按帧求和后统计峰/均）"))
        self.table = QTableWidget(0, 10)
        self.table.setHorizontalHeaderLabels(("时间", "类别／挡位", "动作", "目标／速度／斜坡", "X 峰/均", "Y 峰/均", "Z 峰/均", "|X|+|Y|+|Z| 峰/均", "结果／样本", "删除"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        self.table.horizontalHeader().setStretchLastSection(True)
        root.addWidget(self.table, 1)
        self.kind.currentIndexChanged.connect(self._change_kind)
        self.signal_round.currentIndexChanged.connect(self._sync)
        self.manual_condition.currentIndexChanged.connect(self._sync)
        self.extend_button.clicked.connect(lambda: self._start("出臂"))
        self.auto_button.clicked.connect(self._toggle_batch)
        self.auto_end_button.clicked.connect(self._end_batch)
        self.defer_capture_button.clicked.connect(self._defer_signal_capture)
        self.return_button.clicked.connect(lambda: self._start("回收"))
        self.recover_button.clicked.connect(self._recover)
        controller.command_result.connect(self._on_result)
        controller.state_changed.connect(self._sync)
        controller.device_restarted.connect(self._device_restart)
        self._timer = QTimer(self)
        self._timer.setInterval(500)
        self._timer.timeout.connect(self._check_timeout)
        self._timer.start()
        self._render()
        self._sync()

    def _load(self) -> None:
        try:
            loaded = json.loads(DATA_PATH.read_text(encoding="utf-8"))
            if not isinstance(loaded, list):
                raise ValueError("记录文件应为数组")
            self.records = loaded
        except FileNotFoundError:
            self.records = []
        except (ValueError, OSError) as exc:
            self._records_load_error = f"实验记录无法读取，已暂停写入：{exc}"
        try:
            loaded = json.loads(VERTICAL_DATA_PATH.read_text(encoding="utf-8"))
            if not isinstance(loaded, list):
                raise ValueError("记录文件应为数组")
            self.vertical_records = loaded
        except FileNotFoundError:
            pass
        except (ValueError, OSError) as exc:
            self._vertical_load_error = f"竖直起步记录无法读取，已暂停写入：{exc}"

    def _save(self, vertical: bool = False) -> None:
        path = VERTICAL_DATA_PATH if vertical else DATA_PATH
        records = self.vertical_records if vertical else self.records
        path.parent.mkdir(parents=True, exist_ok=True)
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")
        temp.replace(path)

    @property
    def vertical(self) -> bool:
        return self.kind.currentIndex() == 2

    @property
    def signal_test(self) -> bool:
        return self.kind.currentIndex() == 1

    @property
    def capturing_trace(self) -> bool:
        return ((self.batch is not None and self.batch.running) or
                (self.active is not None and (self.active.trace is not None or self.active.high_rate)))

    @property
    def start_target_mdeg(self) -> int:
        return self.controller.working_zero_mdeg if self.vertical else self.controller.fold_target_mdeg

    @property
    def start_tolerance_mdeg(self) -> int:
        return VERTICAL_START_TOLERANCE_MDEG if self.vertical else 5000

    def _change_kind(self) -> None:
        self._baseline.clear()
        self.manual_stability = ManualStartStability()
        error = self._vertical_load_error if self.vertical else self._records_load_error
        self.status.setText(error or "等待出臂或回收请求")
        self._render()
        self._sync()

    def _signal_round_context(self) -> tuple[int, int, int, int, int]:
        info = self.controller.info
        return (self.signal_round.currentData(), SIGNAL_PLAN_VERSION, self.controller.working_zero_mdeg,
                self.controller.fold_target_mdeg, info.max_speed_dps if info is not None else 500)

    @property
    def signal_retest(self) -> bool:
        return self.signal_round.currentData() in (0, -1)

    @property
    def manual_retest(self) -> bool:
        return self.signal_test and self.signal_round.currentData() == -1

    def _signal_round_records(self, context: tuple[int, int, int, int, int]):
        return (row for row in self.records if (row.get("signal_round"), row.get("signal_plan_version"),
                row.get("normal_mdeg"), row.get("fold_mdeg"), row.get("signal_max_speed_dps")) == context)

    def _signal_round_plan(self, *, include_deferred: bool = False) -> tuple[tuple[BatchCondition, ...], int, int, int]:
        context = self._signal_round_context()
        round_number, _, normal, folded, speed = context
        if self.signal_retest:
            selected, rows = [], []
            for source_round in (1, 2, 3, 4):
                source_context = (source_round, *context[1:])
                source_rows = list(self._signal_round_records(source_context))
                existing_valid, existing_power, candidates = set(), set(), set()
                for row in source_rows:
                    key = row.get("travel"), row.get("speed_percent"), row.get("ramp_ms")
                    review = (row.get("signal_condition_review") or {}).get("code")
                    if row.get("signal_retest_stage") != SIGNAL_RETEST_STAGE and (row.get("automatic_batch") or {}).get("valid"):
                        existing_valid.add(key)
                    if row.get("signal_retest_stage") != SIGNAL_RETEST_STAGE and review == HX8_POWER_PROTECTION:
                        existing_power.add(key)
                    end = row.get("end_pos_mdeg")
                    capture = row.get("high_rate_capture") or {}
                    if review == HIGH_RATE_REVIEW or (
                            (row.get("automatic_batch") or {}).get("valid") is False and
                            capture.get("status") in ("review", "incomplete") and
                            row.get("terminal") in ("END", "SETTLED") and end is not None and
                            abs(end-row["target_mdeg"]) <= 3000):
                        candidates.add(key)
                candidates -= existing_valid | existing_power
                selected.extend(replace(condition, signal_round=source_round) for condition in
                                signal_conditions(normal, folded, speed, source_round) if condition.key in candidates)
                rows.extend(row for row in source_rows if row.get("signal_retest_stage") == SIGNAL_RETEST_STAGE)
            plan = tuple(selected)
            # Earlier review labels select the scope. Count only this stage's
            # attempts toward its progress; earlier valid/power rows were
            # already excluded from candidates above.
        else:
            plan = signal_conditions(normal, folded, speed, round_number)
            rows = list(self._signal_round_records(context))
        def row_key(row):
            key = row.get("travel"), row.get("speed_percent"), row.get("ramp_ms")
            return (row.get("signal_round"), *key) if self.signal_retest else key
        def condition_key(condition):
            return (condition.signal_round, *condition.key) if self.signal_retest else condition.key
        valid = set()
        protected = set()
        capture_review = set()
        for row in rows:
            key = row_key(row)
            review = (row.get("signal_condition_review") or {}).get("code")
            if (row.get("automatic_batch") or {}).get("valid"):
                valid.add(key)
            elif review == HX8_POWER_PROTECTION:
                protected.add(key)
            elif review == HIGH_RATE_REVIEW:
                capture_review.add(key)
        tested = valid | protected | (set() if include_deferred else capture_review)
        protected -= valid
        capture_review -= valid | protected
        remaining = tuple(condition for condition in plan if condition_key(condition) not in tested)
        return (remaining, len(plan), sum(condition_key(condition) in protected for condition in plan),
                sum(condition_key(condition) in capture_review for condition in plan))

    def _signal_capture_failures(self, condition: BatchCondition) -> list[dict]:
        failures = []
        context = self._signal_round_context()
        if self.signal_retest:
            context = (condition.signal_round, *context[1:])
        for row in self._signal_round_records(context):
            if self.signal_retest and row.get("signal_retest_stage") != SIGNAL_RETEST_STAGE:
                continue
            if not self.signal_retest and row.get("signal_retest"):
                continue
            if (row.get("travel"), row.get("speed_percent"), row.get("ramp_ms")) != condition.key:
                continue
            end = row.get("end_pos_mdeg")
            if ((row.get("automatic_batch") or {}).get("valid") is False and row.get("terminal") in ("END", "SETTLED")
                    and end is not None and abs(end-row["target_mdeg"]) <= 3000 and row.get("high_rate_capture")):
                failures.append(row)
        return failures

    @staticmethod
    def _manual_key(condition: BatchCondition) -> tuple[int, str, int, int]:
        return condition.signal_round, condition.name, condition.speed_percent, condition.ramp_ms

    def _manual_options(self) -> tuple[BatchCondition, ...]:
        pending, _, _, _ = self._signal_round_plan(include_deferred=True)
        travel_order = {name: index for index, name in enumerate(TRAVELS)}
        return tuple(sorted(pending, key=lambda c: (c.speed_percent, travel_order[c.name], c.ramp_ms,
                                                    c.signal_round)))

    def _sync_manual_options(self, options: tuple[BatchCondition, ...]) -> None:
        current = self.manual_condition.currentData()
        current_key = self._manual_key(current) if isinstance(current, BatchCondition) else None
        keys = tuple(self._manual_key(c) for c in options)
        if keys == tuple(self._manual_key(self.manual_condition.itemData(i))
                         for i in range(self.manual_condition.count())):
            return
        self.manual_condition.blockSignals(True)
        self.manual_condition.clear()
        for condition in options:
            self.manual_condition.addItem(
                f"第 {condition.signal_round} 轮 · {condition.name}行程 · "
                f"{condition.speed_percent}% · {condition.ramp_ms} ms／段", condition)
        if current_key in keys:
            self.manual_condition.setCurrentIndex(keys.index(current_key))
        self.manual_condition.blockSignals(False)

    @staticmethod
    def _imu_diag_snapshot(frame) -> dict | None:
        return asdict(frame) if frame is not None else None

    def _defer_signal_capture(self) -> None:
        if not self.defer_capture_button.isEnabled():
            return
        remaining, _, _, _ = self._signal_round_plan()
        if not remaining:
            return
        condition = remaining[0]
        failures = self._signal_capture_failures(condition)
        if len(failures) <= BATCH_MAX_RETRIES:
            return
        review = {"code": HIGH_RATE_REVIEW, "source": "operator_deferred",
                  "evidence": [{"req_id": row["req_id"]} for row in failures]}
        record = failures[-1]
        record["signal_condition_review"] = review
        try:
            self._save()
        except OSError as exc:
            del record["signal_condition_review"]
            self.status.setText(f"待补测标记保存失败：{exc}")
            return
        self.batch = None
        self.status.setText(f"{condition.name}行程 · {condition.speed_percent}% · {condition.ramp_ms} ms 已标为高频采样待补测")
        self._render()
        self._sync()
        self.capture_changed.emit()

    def _sync(self) -> None:
        vertical = self.vertical
        structure = self.kind.currentIndex() == 0
        signal = self.signal_test
        normal = self.controller.working_zero_mdeg
        destination = target_for_travel(MAX_TRAVEL_DEG, normal)
        self.start_hint.setText(
            f"竖直起步：以已校准正常位 {normal / 1000:+.1f}° 出臂和回收。"
            f"长行程 {MAX_TRAVEL_DEG:g}° · HX8 目标 {destination / 1000:+.1f}° · 加减速各 100 ms。"
            "出臂保存逐帧记录，结束后继续采集 2 秒。"
            if vertical else
            "手动补测：按速度、行程、斜坡排序选组；折叠起点停稳提示变绿后出臂，保存结果后手动回折叠位。"
            if self.manual_retest else
            "折叠起步 · 主控臂 3 挡／其他臂 4 挡。自动切换三种行程、三档速度和斜坡时间。" if signal else
            "折叠起步：短／中／长相对正常位前伸 23.3°／46.7°／70°。加减速各 100 ms。")
        self.return_button.setText("回收到正常位" if vertical else "回收到折叠位")
        self.extend_button.setText("出臂并记录逐帧数据" if vertical else "出臂并记录")
        manual = self.manual_retest
        self.extend_button.setVisible(not signal or manual)
        self.auto_button.setVisible(not manual)
        self.auto_count.setVisible(not signal)
        if structure:
            for index, (label, degrees) in enumerate(TRAVELS.items()):
                target = target_for_travel(degrees, normal)
                self.travel.setItemText(index, f"{label} · 相对正常位 {degrees:.1f}° · "
                                          f"折叠起点约 {(self.controller.fold_target_mdeg-target)/1000:.1f}°"
                                          f"（目标 {target/1000:+.1f}°）")
        if self.active is not None and not self.controller.link_online:
            self._baseline.clear()
            self._finish("链路断开，动作状态未确认", time.monotonic())
            return
        if not self.controller.link_online or self.controller.mode != "MANUAL":
            self._baseline.clear()
        if self._recovery_req_id is not None and not self.controller.link_online:
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
            self.status.setText("故障恢复中链路断开，恢复结果未确认")
        editable = self.active is None and self.batch is None
        self.kind.setEnabled(editable)
        for widget, visible in (
                (self.gear, structure), (self.travel, structure), (self.structure_speed, structure),
                (self.vertical_gear, vertical), (self.vertical_speed, vertical),
                (self.signal_round, signal), (self.signal_progress, signal),
                (self.manual_condition, manual), (self.manual_ready_label, manual)):
            self.form.setRowVisible(widget, visible)
            widget.setEnabled(editable and visible)
        supported = self.controller.experiment_info is not None
        self.firmware_label.setText(
            ("板载高频采样：出臂后无线回传" if self.controller.capture_supported else
             "实验固件 XMOVE：低频记录，当前固件未确认支持板载高频采样") if supported else
            "实验固件 XMOVE：设备离线，等待重连确认" if not self.controller.link_online else
            "实验固件 XMOVE：能力回复未收到，正在重试查询")
        state = self.controller.latest_state
        if not self.controller.link_online or state is None:
            self.fault_status.setText("设备故障：未知／等待在线反馈")
        elif state.fault_code == "NONE":
            self.fault_status.setText("设备故障：无")
        else:
            position = f"{state.pos_mdeg/1000:.1f}°" if state.pos_mdeg is not None else "未知"
            target = f"{state.target_mdeg/1000:.1f}°" if state.target_mdeg is not None else "未知"
            self.fault_status.setText(f"设备故障：{state.fault_code} · 实际 {position} · 目标 {target}")
        if (self._recovery_query_id is not None and state is not None and
                state.reply_to >= self._recovery_query_id):
            recovered = state.fault_code == "NONE" and state.motion == "IDLE"
            self.status.setText("故障恢复已确认：设备报告无故障" if recovered else
                                f"故障恢复未完成：设备仍报告 {state.fault_code}／{state.motion}")
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
            if self.batch is not None and self.batch.running and self.batch.phase == "recovering":
                if recovered:
                    batch = self.batch
                    batch.resume_after_recovery = False
                    batch.rejected_moves = 0
                    batch.return_confirmed = False
                    batch.return_corrected = False
                    self._batch_phase("route", time.monotonic(), verify=True, rearm=True)
                    self.status.setText("故障已清除，准备回当前模式起点并停稳后续采")
                else:
                    self._pause_batch("设备仍有故障", device_fault=True)
                    return
        auto_running = self.batch is not None and self.batch.running
        self.recover_button.setEnabled(bool(not self.controller.simulated and self.controller.link_online and
            self.controller.mode == "MANUAL" and state is not None and state.hx8_online and
            state.fault_code != "NONE" and self._recovery_req_id is None and
            not auto_running and not any(p.verb == "STOP" for p in self.controller._pending.values())))
        ready = (supported and not self.controller.voltage_locked and not self.controller.simulated and self.controller.link_online and
                 self.controller.info is not None and self.controller.cal is not None and
                 self.controller.mode == "MANUAL" and state is not None and state.hx8_online and
                 state.imu_online and state.imu_hz >= 60 and state.pos_mdeg is not None and
                 state.motion == "IDLE" and state.fault_code == "NONE" and not self.controller._pending and
                 self.active is None)
        tolerance = self.start_tolerance_mdeg
        calibrated = not vertical or self.controller.pose_calibration_saved
        load_error = self._vertical_load_error if vertical else self._records_load_error
        can_start = ready and calibrated and not load_error
        can_extend = bool(can_start
                          and abs(state.pos_mdeg-self.start_target_mdeg) <= tolerance)
        can_return = ready and calibrated and (
            abs(state.pos_mdeg - self.start_target_mdeg) > tolerance
            if vertical else target_for_travel(MAX_TRAVEL_DEG) - 10000 <= state.pos_mdeg < self.start_target_mdeg - tolerance
        )
        remaining = None
        total = 0
        if signal:
            try:
                remaining, total, protected, capture_review = self._signal_round_plan(
                    include_deferred=manual)
                tested = total-len(remaining)
                if manual:
                    options = self._manual_options()
                    self._sync_manual_options(options)
                    self.signal_progress.setText(
                        f"待手动补测 {len(options)} 组 · 已有有效 {total-len(options)-protected} 组"
                        f" · 功率保护排除 {protected} 组。建议按速度 25→45→78%、短→中→长、斜坡递增逐组测试。")
                else:
                    valid = tested-protected-capture_review
                    suffix = (f" · 功率保护待补测 {protected}" if protected else "")
                    if capture_review:
                        suffix += f" · 高频采样待补测 {capture_review}"
                    self.signal_progress.setText(
                        f"已测试 {tested} / {total} · 有效 {valid}{suffix}" if suffix else f"{tested} / {total} 次有效出臂")
            except ValueError as exc:
                self.signal_progress.setText(str(exc))
                remaining = ()
        manual_ready = self._manual_start_ready(time.monotonic()) if manual else False
        self.manual_ready_label.setText("起点已停稳，可手动出臂并记录" if manual_ready else
                                        "等待折叠位空闲、IMU ≥160 Hz 与三轴波动 ≤0.006 g 持续 1 秒")
        self.manual_ready_label.setStyleSheet("color: #167534; font-weight: bold;" if manual_ready else "")
        self.extend_button.setEnabled(bool((not signal and can_extend or manual and can_extend and manual_ready
                                            and self.manual_condition.count() > 0) and not auto_running))
        self.auto_button.setToolTip("依次补测第 2、3、4 轮高频不合格且尚无有效数据的工况，排除功率过载组。" if signal and self.signal_retest else
                                   "从所选轮次的首个未完成工况开始，完成后停在折叠位。" if signal else
                                   "每批只测当前补偿臂挡位，自动切换出臂速度和行程。")
        self.auto_button.setText("暂停自动采集" if auto_running else
                                 "继续自动采集" if self.batch is not None else
                                 "无待复测工况" if signal and self.signal_retest and not total else
                                 "本轮已完成" if signal and remaining == () and total else
                                 "启动本轮" if signal else "自动采集")
        can_auto_start = ((can_extend or can_return) and bool(remaining) if signal else can_extend)
        auto_ready = ready and state.imu_hz >= CAPTURE_MIN_ACC_HZ
        self.auto_button.setEnabled(not manual and (auto_running or bool(self.controller.capture_supported and calibrated and not load_error and
            auto_ready and (self.batch is not None or can_auto_start))))
        self.auto_end_button.setVisible(self.batch is not None and not auto_running)
        self.auto_end_button.setEnabled(self.active is None)
        self.defer_capture_button.setVisible(signal and not manual)
        self.defer_capture_button.setEnabled(bool(signal and remaining and not self._records_load_error
            and not auto_running and self.active is None and self._recovery_req_id is None
            and not self.controller.voltage_locked
            and len(self._signal_capture_failures(remaining[0])) > BATCH_MAX_RETRIES))
        self.auto_count.setEnabled(editable)
        self.return_button.setEnabled(bool(can_return and not auto_running))

    def _recover(self) -> bool:
        if self.controller.voltage_locked:
            return False
        if not self.controller.request_recover():
            self.status.setText(f"故障恢复未提交：{self.controller.command_status}")
            return False
        if self.active is not None:
            self._finish("故障恢复中止采样，动作未完成", time.monotonic())
        self._recovery_req_id = self.controller._next_req - 1
        self._recovery_query_id = None
        self._recovery_requested_at = time.monotonic()
        if self.batch is not None and not self.batch.running and self.batch.resume_after_recovery:
            self.batch.running = True
            self._batch_phase("recovering", self._recovery_requested_at)
            self.capture_changed.emit()
        self.status.setText("已发送停止并保持；等待设备 ACK 和新鲜 STATE 确认故障是否清除")
        self._sync()
        return True

    def _toggle_batch(self) -> None:
        if self.controller.voltage_locked or self.manual_retest:
            return
        if self.batch is not None and self.batch.running:
            self._pause_batch("已手动暂停", stop=True)
            return
        info = self.controller.info
        if not self.auto_button.isEnabled() or info is None:
            return
        if self.batch is None:
            if self.signal_test:
                options, total, _, _ = self._signal_round_plan()
                gear = SIGNAL_GEAR
                repeats = 1
                round_number = self._signal_round_context()[0]
                saved = total-len(options)
            else:
                options = batch_conditions(self.kind.currentIndex())
                gear = (self.vertical_gear if self.vertical else self.gear).currentText()
                repeats = self.auto_count.value()
                round_number = None
                saved = 0
            self.batch = AutoBatch(uuid.uuid4().hex, options, repeats, info.boot_id, gear,
                                   self.controller.working_zero_mdeg, self.controller.fold_target_mdeg,
                                   info.max_speed_dps, signal_round=round_number, already_saved=saved,
                                   signal_retest=self.signal_test and self.signal_retest)
        else:
            self.batch.running = True
            self.batch.boot_id = info.boot_id
            self.batch.recovery_used = False
            self.batch.retries = 0
            self.batch.rejected_moves = 0
            self.batch.return_confirmed = False
            self.batch.return_corrected = False
            self.batch.resume_after_recovery = False
        self._batch_phase("route", time.monotonic(), verify=True)
        self.status.setText("自动采集已启动，等待起点空闲并停稳")
        self._sync()
        self.capture_changed.emit()

    def _batch_phase(self, phase: str, now: float, *, verify: bool = False, rearm: bool = False) -> None:
        batch = self.batch
        batch.phase = phase
        batch.acc.clear()
        batch.recovery_deadline = now + BATCH_RECOVERY_TIMEOUT if phase in ("fault", "recovering") else None
        batch.rearm_at = now + BATCH_REARM_SECONDS if rearm else now
        batch.await_state = None
        state = self.controller.latest_state
        batch.verify_after_ms = state.t_ms if verify and state is not None else None
        if verify and self.controller.query("STATE"):
            batch.await_state = self.controller._query_ids["STATE"]

    def _pause_batch(self, reason: str, *, stop: bool = False, device_fault: bool = False) -> None:
        batch = self.batch
        if batch is None:
            return
        if not batch.running:
            if not device_fault:
                batch.resume_after_recovery = False
            return
        batch.running = False
        batch.resume_after_recovery = device_fault
        batch.acc.clear()
        run = self.active
        if stop and run is not None and run.end_at is None:
            if (self.controller.link_online and not self.controller.voltage_locked and
                    not any(p.verb == "STOP" for p in self.controller._pending.values())):
                self.controller.request_stop()
            self._finish("自动采集暂停，动作未确认完成", time.monotonic())
        progress = f"已测试 {batch.completed} 组" if batch.signal_round is not None else f"已保存 {batch.completed} 次有效出臂"
        self.status.setText(f"自动采集暂停（{progress}）｜{reason}")
        self._sync()
        self.capture_changed.emit()

    def pause_for_low_voltage(self) -> None:
        self._pause_batch("已触发全局低压报警", stop=True)
        if self.active is not None and self.active.end_at is None:
            self._finish("检测到低压，已中止动作与采样", time.monotonic())

    def _end_batch(self) -> None:
        if self.controller.voltage_locked or self.batch is None or self.batch.running or self.active is not None:
            return
        completed = self.batch.completed
        self.batch = None
        label = "组已测试结果" if self.signal_test else "次有效记录"
        self.status.setText(f"批次已结束，保留 {completed} {label}")
        self._render()
        self._sync()
        self.capture_changed.emit()

    def _complete_batch(self, batch: AutoBatch) -> None:
        self.batch = None
        if batch.signal_round is not None:
            _, total, protected, capture_review = self._signal_round_plan()
            label = "第四轮补测" if batch.signal_retest else f"第 {batch.signal_round} 轮"
            summary = (f"{label}已测试 {total} 个工况，有效 {total-protected-capture_review} 个，"
                       f"功率保护待补测 {protected} 个，高频采样待补测 {capture_review} 个，已回到折叠位并停稳。"
                       if protected or capture_review else
                       f"信号测试{label}已采齐 {total} 个工况，已回到折叠位并停稳。")
        else:
            summary = f"{batch.gear or '当前参数'} · {len(batch.conditions)} 个工况，各 {batch.repeats} 次有效出臂，已回到起点并停稳。"
        self.status.setText(f"自动采集完成｜{summary}")
        self._render()
        self._sync()
        self.capture_changed.emit()
        dialog = QMessageBox(QMessageBox.Icon.Information, "自动采集完成", summary,
                             QMessageBox.StandardButton.Ok, self)
        dialog.setModal(False)
        dialog.finished.connect(dialog.deleteLater)
        self._completion_dialog = dialog
        dialog.show()

    def _batch_at_home(self, state: StateFrame) -> bool:
        return state.pos_mdeg is not None and abs(state.pos_mdeg-self.start_target_mdeg) <= self.start_tolerance_mdeg

    def _batch_stable(self, batch: AutoBatch, now: float) -> bool:
        return self._acc_stable(batch.acc, now)

    @staticmethod
    def _acc_stable(samples: deque[tuple[float, AccFrame]], now: float) -> bool:
        if len(samples) < 16 or now-samples[-1][0] > 0.2:
            return False
        if (samples[-1][1].sample_ms-samples[0][1].sample_ms) % (2**32) < BATCH_STABLE_MS:
            return False
        for axis in ("ax", "ay", "az"):
            values = [round(getattr(frame.sample, axis)*1000) for _, frame in samples]
            if max(values)-min(values) > BATCH_ACC_LIMIT_MG:
                return False
        return True

    def _manual_start_ready(self, now: float) -> bool:
        stable = self.manual_stability
        state = self.controller.latest_state
        state_at = self.controller._last_state_received_at
        info = self.controller.info
        if (not self.manual_retest or self.batch is not None or self.active is not None or
                self.controller.voltage_locked or self.controller.simulated or
                not self.controller.link_online or self.controller.mode != "MANUAL" or
                not self.controller.capture_supported or info is None or state is None or
                state_at is None or now-state_at > 1.5 or self.controller._pending or
                not state.hx8_online or not state.imu_online or state.imu_hz < CAPTURE_MIN_ACC_HZ or
                state.motion != "IDLE" or state.fault_code != "NONE" or
                state.voltage_mv is None or state.voltage_mv < self._minimum_voltage_mv or
                not self._batch_at_home(state)):
            stable.acc.clear()
            stable.state_ms = None
            stable.pos_mdeg = None
            return False
        if stable.pos_mdeg is None or abs(state.pos_mdeg-stable.pos_mdeg) > 300:
            stable.acc.clear()
            stable.state_ms = state.t_ms
            stable.pos_mdeg = state.pos_mdeg
            return False
        return state.t_ms != stable.state_ms and self._acc_stable(stable.acc, now)

    def _vertical_baseline_ready(self, now: float) -> bool:
        acc_times = [t for t, frame in self._baseline if isinstance(frame, AccFrame) and t >= now-BASELINE_SECONDS]
        return bool(acc_times and acc_times[-1]-acc_times[0] >= 0.8 and now-acc_times[-1] <= 0.2)

    def _advance_batch(self, now: float) -> None:
        batch = self.batch
        if self.controller.voltage_locked or batch is None or not batch.running or self.active is not None:
            return
        info = self.controller.info
        if (not self.controller.link_online or info is None or info.boot_id != batch.boot_id
                or self.controller.mode != "MANUAL" or not self.controller.capture_supported):
            self._pause_batch("设备离线、重启或控制模式已变化")
            return
        if (batch.normal_mdeg, batch.fold_mdeg, batch.max_speed_dps) != (
                self.controller.working_zero_mdeg, self.controller.fold_target_mdeg, info.max_speed_dps):
            self._pause_batch("校准基准或设备速度上限已变化，请结束旧批次")
            return
        if batch.recovery_deadline is not None and now >= batch.recovery_deadline:
            self._pause_batch("等待故障恢复超时", device_fault=True)
            return
        state = self.controller.latest_state
        state_at = self.controller._last_state_received_at
        if (state is None or state_at is None or now-state_at > 1.5 or
                (batch.await_state is not None and state.reply_to < batch.await_state) or
                state.t_ms == batch.verify_after_ms):
            batch.acc.clear()
            return
        if batch.phase == "recovering":
            return
        if batch.phase == "fault":
            # The fault frame may arrive before STATE. The diagnostic must
            # belong to this fault and identify a transient feedback gap.
            diagnostic = self.controller.last_hx8_diag
            if state.fault_code == "NONE":
                return
            if (batch.recovery_used or state.fault_code != "FEEDBACK_STALE" or
                    diagnostic is None or (state.t_ms-diagnostic.t_ms) % (2**32) > 2500 or
                    batch.fault_ms is None or diagnostic.t_ms != batch.fault_ms or
                    diagnostic.cause not in ("AGE", "NO_SAMPLE", "HX8_OK_OR_OTHER")):
                self._pause_batch(f"请检查设备故障 {state.fault_code}", device_fault=True)
                return
            if (not state.hx8_online or state.voltage_mv is None or
                    state.voltage_mv < self._minimum_voltage_mv or self.controller._pending):
                return
            batch.recovery_used = True
            self._batch_phase("recovering", now)
            if not self._recover():
                self._pause_batch("自动故障恢复未提交", device_fault=True)
            return
        if state.fault_code != "NONE":
            self._pause_batch(f"请检查设备故障 {state.fault_code}", device_fault=True)
            return
        if not state.imu_online or state.imu_hz < CAPTURE_MIN_ACC_HZ:
            batch.acc.clear()
            self._baseline.clear()
            health = "板端 IMU 断流" if not state.imu_online else f"板端 IMU 采样率 {state.imu_hz} Hz"
            self.status.setText(f"自动采集 {batch.completed}/{batch.total} · {health} · 等待有效采样恢复")
            return
        if now < batch.rearm_at:
            return
        if (state.motion != "IDLE" or not state.hx8_online or state.pos_mdeg is None or self.controller._pending):
            batch.acc.clear()
            return
        if state.voltage_mv is None or state.voltage_mv < self._minimum_voltage_mv:
            self._pause_batch("起步前供电不足或电压反馈无效")
            return
        condition = batch.conditions[batch.index]
        if batch.phase == "route":
            if self._batch_at_home(state):
                self._batch_phase("stable", now)
                batch.stable_state_ms = state.t_ms
                batch.stable_pos_mdeg = state.pos_mdeg
            else:
                if batch.return_confirmed and batch.return_corrected:
                    self._pause_batch("补回一次后仍未进入起点容差，请检查机构")
                    return
                batch.return_corrected = batch.return_confirmed
                self._batch_phase("return", now)
                self._start("回收", condition)
                if self.active is None:
                    self._pause_batch(f"自动回收未提交：{self.status.text()}")
            return
        if batch.phase != "stable":
            return
        if not self._batch_at_home(state):
            self._batch_phase("route", now, verify=True)
            return
        if abs(state.pos_mdeg-batch.stable_pos_mdeg) > 300:
            batch.stable_state_ms = state.t_ms
            batch.stable_pos_mdeg = state.pos_mdeg
            batch.acc.clear()
        if state.t_ms == batch.stable_state_ms or not self._batch_stable(batch, now):
            self.status.setText(f"自动采集 {batch.completed}/{batch.total} · "
                                "等待起点三轴各自峰峰值 ≤0.006 g，持续 1 秒")
            return
        if batch.condition_complete and batch.index+1 == len(batch.conditions):
            self._complete_batch(batch)
            return
        if self.vertical and not self._vertical_baseline_ready(now):
            self.status.setText("起点已停稳，等待连续加速度数据作为出臂前基线")
            return
        if batch.condition_complete:
            batch.index += 1
            batch.valid = 0
            batch.power_protected = False
            batch.capture_deferred = False
            condition = batch.conditions[batch.index]
        self._batch_phase("extend", now)
        self._start("出臂", condition)
        if self.active is None:
            self._pause_batch(f"自动出臂未提交：{self.status.text()}")

    def _batch_run_finished(self, run: ActiveRun, valid: bool, now: float, issue: str = "",
                            *, power_protected: bool = False, capture_deferred: bool = False) -> None:
        batch = self.batch
        if batch is None or run.batch_id != batch.id:
            return
        if run.record and valid:
            batch.valid += 1
            batch.retries = 0
            batch.recovery_used = False
        if power_protected:
            batch.power_protected = True
            batch.retries = 0
            batch.recovery_used = False
            if batch.running:
                self._pause_batch("HX8 功率保护，本工况已列为待补测。请确认设备后点击故障恢复", device_fault=True)
            return
        if not batch.running:
            return
        if (run.failure_code in ("NOT_READY", "BUSY", "IMU_RATE_LOW", "IMU_OFFLINE") and
                not run.motion_started and run.terminal is None
                and batch.rejected_moves < BATCH_MAX_RETRIES):
            batch.rejected_moves += 1
            if not run.record and batch.return_confirmed:
                batch.return_corrected = False
            self._batch_phase("route", now, verify=True, rearm=True)
            self.status.setText(f"指令尚未起动（{run.failure_code}），等待设备就绪后重试")
            return
        if (run.terminal == "ABORT" and run.failure_code not in ("STOPPED", "LINK_LOST")) or (
                run.failure_code in ("FEEDBACK_STALE", "FAULT", "MOTION_TIMEOUT", "HX8_OFFLINE", "IMU_OFFLINE")):
            batch.fault_ms = run.end_mcu_ms if run.failure_code == "FEEDBACK_STALE" else None
            self._batch_phase("fault", now, verify=True)
            return
        if not run.motion_confirmed:
            self._pause_batch(run.failure_code or "动作结果未确认")
            return
        if capture_deferred:
            batch.capture_deferred = True
            batch.retries = 0
        batch.rejected_moves = 0
        batch.return_confirmed = not run.record
        if run.record:
            batch.return_corrected = False
        if run.record and not valid and not capture_deferred:
            batch.retries += 1
            if batch.retries > BATCH_MAX_RETRIES:
                self._pause_batch(f"连续三次高频数据未通过检查 · 最近一次：{issue}")
                return
        self._batch_phase("route", now, verify=True, rearm=run.terminal == "SETTLED")
        if capture_deferred:
            self.status.setText(f"高频采样待补测，本工况已自动跳过 · {issue} · 回起点停稳后继续下一工况")
        elif run.record and not valid:
            self.status.setText(f"本次未计入批次（{batch.retries}/3） · {issue} · 确认反馈及停稳后重测")

    def _start(self, action: str, condition: BatchCondition | None = None) -> None:
        manual = self.manual_retest and action == "出臂" and condition is None
        if (self.controller.voltage_locked or self.active is not None or
                (condition is None and ((self.batch is not None and self.batch.running)
                                        or (self.signal_test and action == "出臂" and not manual)))):
            return
        state = self.controller.latest_state
        info = self.controller.info
        if state is None or state.pos_mdeg is None or info is None:
            return
        vertical = self.vertical
        measured = vertical and action == "出臂"
        if vertical and (not self.controller.pose_calibration_saved or self.controller.calibration_position() is None):
            self.status.setText("请先在校准模式保存竖直正常位，并等待 HX8 的空闲角度反馈")
            return
        load_error = self._vertical_load_error if vertical else self._records_load_error
        if action == "出臂" and load_error:
            self.status.setText(load_error)
            return
        now = time.monotonic()
        if manual:
            if not self._manual_start_ready(now):
                self.status.setText("折叠起点尚未满足自动采集同款停稳判定，请等待绿色就绪提示")
                return
            condition = self.manual_condition.currentData()
            if not isinstance(condition, BatchCondition):
                self.status.setText("没有待补测工况")
                return
        baseline = [(t, f) for t, f in self._baseline if t >= now-BASELINE_SECONDS]
        if measured and not self._vertical_baseline_ready(now):
            self.status.setText("等待约 1 秒连续加速度数据作为起动前基线")
            return
        structure = self.kind.currentIndex() == 0
        if vertical:
            name, travel = "长", MAX_TRAVEL_DEG
            gear = self.vertical_gear.currentText()
            speed_percent = self.vertical_speed.currentData()
        elif structure:
            name, travel = self.travel.currentData()
            gear = self.gear.currentText()
            speed_percent = self.structure_speed.currentData()
        else:
            name, travel = "回收", 0.0
            gear = SIGNAL_GEAR
            speed_percent = STRUCTURE_SPEEDS["低速"]
        if condition is not None:
            name, travel, speed_percent = condition.name, condition.travel_deg, condition.speed_percent
            gear = self.batch.gear if self.batch is not None else SIGNAL_GEAR
            if action == "回收":
                speed_percent = STRUCTURE_SPEEDS["低速"]
        target = target_for_travel(travel, self.controller.working_zero_mdeg) if action == "出臂" else self.start_target_mdeg
        tolerance = self.start_tolerance_mdeg
        if action == "出臂" and abs(state.pos_mdeg-self.start_target_mdeg) > tolerance:
            self.status.setText("出臂前请回到已校准的正常位（±2°）" if vertical else "出臂必须先回到折叠位")
            return
        if speed_percent not in EXPERIMENT_SPEED_LEVELS:
            self.status.setText(f"请选择 5% 的整数倍或最高档 {MAX_OPERATOR_SPEED_PERCENT}%")
            return
        ramp_ms = condition.ramp_ms if condition is not None and action == "出臂" else 100
        params = {"test": self.kind.currentText(), "gear": gear,
                  "normal_mdeg": self.controller.working_zero_mdeg, "fold_mdeg": self.controller.fold_target_mdeg,
                  "action": action, "travel": name, "travel_deg": travel, "start_mdeg": state.pos_mdeg,
                  "target_mdeg": target, "commanded_delta_deg": abs(target-state.pos_mdeg)/1000,
                  "speed_percent": speed_percent,
                  "speed_dps": round(info.max_speed_dps*speed_percent/100), "ramp_ms": ramp_ms,
                  "ramp_input": f"每段 {ramp_ms} ms", "device_boot_id": info.boot_id}
        if vertical:
            control_arm_gear, other_arm_gear = VERTICAL_GEAR_POSITIONS[gear]
            params["control_arm_gear"] = control_arm_gear
            params["other_arm_gear"] = other_arm_gear
            params["start_reference"] = "calibrated_normal"
        if self.signal_test and action == "出臂":
            planned_distance = self.controller.fold_target_mdeg-target
            ramp_distance = ramp_ms*params["speed_dps"]
            params.update(signal_round=condition.signal_round if manual or self.batch.signal_retest else self.batch.signal_round,
                          signal_plan_version=SIGNAL_PLAN_VERSION,
                          signal_max_speed_dps=info.max_speed_dps, control_arm_gear=3, other_arm_gear=4,
                          signal_time_scan=ramp_ms in SIGNAL_RAMP_TIMES,
                          signal_time_ratio_targets_percent=list(condition.time_ratios),
                          signal_nominal_time_ratio_percent=round(200*ramp_distance/(planned_distance+ramp_distance), 2))
            if manual or self.batch.signal_retest:
                params["signal_retest"] = True
                params["signal_retest_stage"] = SIGNAL_RETEST_STAGE
            if manual:
                params["manual_retest"] = True
        high_rate = action == "出臂" and self.controller.capture_supported
        req = self.controller.request_experiment_move(
            target, speed_percent, ramp_ms, capture=high_rate,
            return_tolerance_mdeg=tolerance if condition is not None and action == "回收" else None)
        if req is None:
            self.status.setText(self.controller.command_status)
            return
        self.active = ActiveRun(req, params, now, record=action == "出臂", trace=baseline if measured else None,
                                high_rate=high_rate, batch_id=self.batch.id if self.batch is not None and condition is not None else None,
                                imu_diag_before=self._imu_diag_snapshot(self.controller.last_imu_diag) if manual else None)
        if manual:
            self.manual_stability.acc.clear()
            self.status.setText(f"手动补测第 {condition.signal_round} 轮 · {name}行程 · {speed_percent}% · "
                                f"{ramp_ms} ms／段；请求 {req} 已排队")
        elif condition is not None:
            batch = self.batch
            if batch.signal_round is not None:
                label = "第四轮补测" if batch.signal_retest else f"第 {batch.signal_round} 轮"
                self.status.setText(f"{label} · 已测试 {batch.completed}/{batch.total} 工况 · "
                                    f"{action} {name} · {speed_percent}% · {ramp_ms} ms／段")
            else:
                self.status.setText(f"自动采集 {batch.index+1}/{len(batch.conditions)} 工况 · "
                                    f"{condition.name}行程 {condition.speed_percent}% · "
                                    f"{batch.valid}/{batch.repeats} 次有效 · {action}")
        else:
            self.status.setText(f"请求 {req} 已排队；等待实际串口写出和设备结束回执")
        self._sync()
        self.capture_changed.emit()

    def on_wire_written(self, payload: bytes, success: bool, written_at: float) -> None:
        run = self.active
        if run is None or not payload.startswith(f"XMOVE,{run.req_id},".encode()):
            return
        if not success:
            self._finish("串口写入失败", written_at)
            return
        if run.tx_at is None:  # retries do not reset the measurement window
            run.tx_at = written_at
            if run.record:
                run.tx_wall = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.status.setText(f"请求 {run.req_id} 已写出，" + ("正在采样" if run.record else "正在回收（不记录）"))

    def on_acc(self, frame: AccFrame, received_at: float) -> None:
        self.on_trace_frame(frame, received_at)
        batch = self.batch
        stable = (batch if batch is not None and batch.running and batch.phase == "stable" else
                  self.manual_stability if self.manual_retest and batch is None and self.active is None else None)
        if stable is not None:
            state = self.controller.latest_state
            state_at = self.controller._last_state_received_at
            if (state is None or state_at is None or received_at-state_at > 1.5 or
                    state.motion != "IDLE" or state.fault_code != "NONE" or not self._batch_at_home(state)):
                stable.acc.clear()
            else:
                if stable.acc:
                    previous = stable.acc[-1][1]
                    if not 0 < (frame.sample_ms-previous.sample_ms) % (2**32) <= BATCH_MAX_ACC_GAP_MS:
                        stable.acc.clear()
                stable.acc.append((received_at, frame))
                while (len(stable.acc) > 1 and
                       (frame.sample_ms-stable.acc[1][1].sample_ms) % (2**32) >= BATCH_STABLE_MS):
                    stable.acc.popleft()
        run = self.active
        if (run is not None and run.record and run.tx_at is not None and
                received_at >= run.tx_at and run.terminal is None):
            run.samples.append((received_at, frame))

    def on_trace_frame(self, frame: TraceFrame, received_at: float) -> None:
        if not self.vertical or not self.controller.link_online or self.controller.mode != "MANUAL":
            return
        run = self.active
        if run is not None:
            if run.download is not None:
                return
            if run.trace is not None and (not isinstance(frame, MotionFrame) or frame.req_id == run.req_id):
                if run.post_until is None or received_at <= run.post_until:
                    run.trace.append((received_at, frame))
                if (run.post_until is not None and isinstance(frame, StateFrame)
                        and (run.end_mcu_ms is None or (frame.t_ms-run.end_mcu_ms) % (2**32) < 2**31)):
                    if frame.motion != "IDLE" or frame.fault_code != "NONE":
                        self._finish("结束后留窗中止：机构再次动作或出现故障", received_at)
            return
        if isinstance(frame, StateFrame) and (frame.motion != "IDLE" or frame.fault_code != "NONE"):
            self._baseline.clear()
            return
        state = self.controller.latest_state
        if state is None or state.motion != "IDLE" or state.fault_code != "NONE" or isinstance(frame, MotionFrame):
            return
        self._baseline.append((received_at, frame))
        while self._baseline and self._baseline[0][0] < received_at-BASELINE_SECONDS:
            self._baseline.popleft()

    def on_motion(self, frame: MotionFrame, received_at: float) -> None:
        run = self.active
        if run is None or frame.req_id != run.req_id:
            return
        if frame.stage in ("START", "ARRIVED"):
            run.motion_started = True
        if frame.stage in ("END", "ABORT") and run.end_at is None:
            run.end_at = received_at
            run.end_mcu_ms = frame.t_ms
            run.end_pos_mdeg = frame.pos_mdeg
            run.terminal = frame.stage
            if frame.stage == "ABORT":
                run.failure_code = frame.reason
                error = (abs(frame.pos_mdeg - frame.target_mdeg) / 1000
                         if frame.pos_mdeg is not None and frame.target_mdeg is not None else None)
                detail = (f"；实际 {frame.pos_mdeg / 1000:.1f}°，距目标 {error:.1f}°"
                          if error is not None else "")
                self._finish(f"中止：{frame.reason}{detail}", received_at)

    def _on_result(self, req_id: int, verb: str, result: str) -> None:
        if req_id == self._recovery_req_id and verb == "STOP":
            if result == "APPLIED":
                self._recovery_query_id = self.controller._query_ids.get("STATE")
                self.status.setText("STOP 已确认；等待新鲜 STATE 核对故障")
            else:
                self.status.setText(f"故障恢复失败：{result}")
                self._recovery_req_id = None
                self._recovery_query_id = None
                self._recovery_requested_at = None
                if self.batch is not None and self.batch.running and self.batch.phase == "recovering":
                    self._pause_batch(f"故障恢复失败：{result}", device_fault=True)
                    return
            self._sync()
            return
        if self.active is not None and self.active.req_id == req_id:
            if self.active.post_until is not None or self.active.download is not None:
                return
            if not self.active.record and result in ("DONE", "RETURN_STATE"):
                self.active.motion_confirmed = True
                outcome = "已通过位置反馈确认" if result == "RETURN_STATE" else "完成"
                self._finish(outcome, time.monotonic())
                return
            if result == "DONE":
                outcome = "完成" if self.active.end_at is not None else "完成，但未收到动作结束帧"
            elif result == "SETTLED":
                state = self.controller.latest_state
                self.active.end_at = time.monotonic()
                self.active.end_mcu_ms = state.t_ms if state is not None else None
                self.active.end_pos_mdeg = state.pos_mdeg if state is not None else None
                self.active.terminal = "SETTLED"
                error = (abs(state.pos_mdeg-self.active.params["target_mdeg"])/1000
                         if state is not None and state.pos_mdeg is not None else None)
                outcome = (f"近目标稳定保持：误差 {error:.1f}°（非精确到位）" if error is not None
                           else "近目标稳定保持（非精确到位）")
            else:
                outcome = f"失败：{result}"
                self.active.failure_code = result
            now = time.monotonic()
            self.active.motion_confirmed = (self.active.terminal in ("END", "SETTLED") and result in ("DONE", "SETTLED"))
            if self.active.trace is not None and self.active.terminal in ("END", "SETTLED") and result in ("DONE", "SETTLED"):
                self.active.outcome = outcome
                self.active.post_until = now + POST_SECONDS
                self.status.setText(f"{outcome}；继续记录停稳后数据 2 秒")
                return
            if self.active.high_rate and result in ("DONE", "SETTLED"):
                self.active.outcome = outcome
                self._start_download(self.active, now)
                return
            self._finish(outcome, now)

    def _start_download(self, run: ActiveRun, now: float) -> None:
        run.download = CaptureDownload(now)
        self.status.setText("出臂已结束，正在回传高频数据")
        self._request_capture_page(run, now)

    def _request_capture_page(self, run: ActiveRun, now: float) -> None:
        download = run.download
        if download.request_log and download.request_log[-1]["outcome"] == "waiting":
            download.request_log[-1]["outcome"] = "timeout"
        if (not self.controller.link_online or download.attempts >= CAPTURE_MAX_RETRIES or
                now - download.started_at >= CAPTURE_TRANSFER_TIMEOUT):
            reason = ("无线链路断开" if not self.controller.link_online else
                      "高频回传首包超时" if download.page == 0 else
                      f"高频回传第 {download.page} 包超时")
            self._capture_failed(run, reason, now)
            return
        download.request_id = self.controller.request_experiment_capture_page(run.req_id, download.page)
        download.request_log.append({"page": download.page, "req_id": download.request_id,
                                     "elapsed_ms": round(1000*(now-download.started_at)),
                                     "outcome": "waiting" if download.request_id is not None else "send_failed"})
        download.sent_at = now
        download.attempts += 1

    def on_capture_page(self, frame: CapturePageFrame, received_at: float) -> None:
        run = self.active
        if run is None or run.download is None:
            return
        download = run.download
        if (frame.move_id, frame.page, frame.req_id) != (run.req_id, download.page, download.request_id):
            download.ignored_pages += 1
            return
        try:
            download.add(frame)
            if download.request_log:
                download.request_log[-1]["outcome"] = "received"
                download.request_log[-1]["response_ms"] = round(1000*(received_at-download.sent_at))
            total = frame.acc_count * CAPTURE_ACC_BYTES + frame.hx_count * CAPTURE_HX_BYTES
            if len(download.data) == total:
                run.high_rate_result = download.result()
                self._finish(run.outcome or "动作结束", received_at)
                return
        except (ValueError, struct.error) as exc:
            self._capture_failed(run, str(exc), received_at)
            return
        page_count = (total+CAPTURE_PAGE_BYTES-1)//CAPTURE_PAGE_BYTES
        self.status.setText(f"出臂已结束，高频数据回传 {download.page}/{page_count} 包")
        self._request_capture_page(run, received_at)

    def _capture_failed(self, run: ActiveRun, reason: str, now: float) -> None:
        run.high_rate_result = {"schema_version": 1, "status": "incomplete", "reason": reason,
                                "received_pages": run.download.page}
        self._finish(run.outcome or "动作结束", now)

    def _check_timeout(self) -> None:
        if (self._recovery_req_id is not None and self._recovery_requested_at is not None and
                time.monotonic()-self._recovery_requested_at > 5):
            self.status.setText("故障恢复超时：未收到完整回执；请检查设备 STATE")
            self._recovery_req_id = None
            self._recovery_query_id = None
            self._recovery_requested_at = None
            if self.batch is not None and self.batch.running and self.batch.phase == "recovering":
                self._pause_batch("故障恢复超时", device_fault=True)
            self._sync()
        run = self.active
        now = time.monotonic()
        if run is not None:
            if run.download is not None:
                if now-run.download.sent_at >= CAPTURE_PAGE_TIMEOUT:
                    self._request_capture_page(run, now)
            elif run.post_until is not None:
                if now >= run.post_until:
                    if run.high_rate:
                        self._start_download(run, now)
                    else:
                        self._finish(run.outcome or "动作结束", now)
            elif now - run.queued_at > 15:
                self._finish("回执超时，动作状态未确认", now)
        self._advance_batch(now)
        if self.manual_retest and self.active is None:
            self._sync()

    def _device_restart(self) -> None:
        self._baseline.clear()
        self._pause_batch("设备已重启，请确认状态后继续")
        if self.active is not None:
            self._finish("设备重启，动作状态未确认", time.monotonic())

    def close_capture(self) -> None:
        self._pause_batch("上位机关闭", stop=True)
        if self.active is not None and self.capturing_trace:
            self._finish("上位机关闭，采样中止", time.monotonic())

    def _finish(self, result: str, now: float) -> None:
        run = self.active
        if run is None:
            return
        self.active = None
        self._baseline.clear()
        self.capture_changed.emit()
        if not run.record:
            self.status.setText(f"请求 {run.req_id}：回收{result}；未写入实验记录")
            self._batch_run_finished(run, False, now)
            self._sync()
            return
        end = run.end_at or now
        samples = [f for t, f in run.samples if run.tx_at is not None and t <= end]
        values = [[getattr(f.sample, axis) for f in samples] for axis in ("ax", "ay", "az")]
        stats = [{"peak_abs_g": max(map(abs, v)), "mean_g": statistics.fmean(v),
                  "mean_abs_g": statistics.fmean(map(abs, v))} if v else None for v in values]
        abs_axis_sums = [sum(abs(component) for component in components) for components in zip(*values)]
        abs_axis_sum_stats = ({"peak_g": max(abs_axis_sums), "mean_g": statistics.fmean(abs_axis_sums)}
                              if abs_axis_sums else None)
        record = {"id": str(uuid.uuid4()), "recorded_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                  "req_id": run.req_id, **run.params, "tx_at": run.tx_wall,
                  "duration_s": round(end-run.tx_at, 3) if run.tx_at is not None and run.end_at is not None else None,
                  "end_mcu_ms": run.end_mcu_ms, "end_pos_mdeg": run.end_pos_mdeg,
                  "actual_delta_deg": (abs(run.end_pos_mdeg-run.params["start_mdeg"])/1000
                                       if run.end_pos_mdeg is not None else None),
                  "comparable": result == "完成", "terminal": run.terminal,
                  "sample_count": len(samples),
                  "first_acc_mcu_ms": samples[0].sample_ms if samples else None,
                  "last_acc_mcu_ms": samples[-1].sample_ms if samples else None,
                  "axis_stats": dict(zip(("x", "y", "z"), stats)),
                  "abs_axis_sum_stats": abs_axis_sum_stats, "result": result,
                  "unit": "g including gravity",
                  "window": ("PC monotonic TX to verified STOP/STATE RX" if run.terminal == "SETTLED"
                             else "PC monotonic TX to MCU terminal RX")}
        vertical = run.trace is not None
        if vertical:
            self._add_trace(record, run, now)
        if run.high_rate:
            record["high_rate_capture"] = run.high_rate_result or {
                "schema_version": 1, "status": "incomplete", "reason": result,
            }
            if run.download is not None:
                record["capture_transfer"] = {
                    "requests": run.download.request_log,
                    "ignored_pages": run.download.ignored_pages,
                    "received_pages": run.download.page,
                    "link_online_at_save": self.controller.link_online,
                }
        manual = bool(run.params.get("manual_retest"))
        if manual:
            valid = run.motion_confirmed and batch_capture_valid(record)
            capture = record["high_rate_capture"]
            issue = ("" if valid else run.failure_code or capture.get("reason") or
                     "、".join(capture.get("issues") or ()) or
                     f"高频覆盖或目标位置未通过；状态 {capture.get('status')}")
            record["manual_retest_result"] = {"valid": valid, "reason": issue}
            record["automatic_batch"] = {"id": "manual_retest", "valid": valid,
                                         "reason": issue if not valid else ""}
            diag = self.controller.last_hx8_diag
            if (run.terminal == "ABORT" and run.failure_code == "FEEDBACK_STALE" and
                    diag is not None and diag.t_ms == run.end_mcu_ms and diag.cause == "STATUS" and
                    diag.last_status & 0x40):
                record["signal_condition_review"] = {
                    "code": HX8_POWER_PROTECTION,
                    "evidence": [{"req_id": run.req_id, "t_ms": diag.t_ms, "status": diag.last_status,
                                  "pos_mdeg": diag.last_pos_mdeg, "voltage_mv": diag.last_voltage_mv}],
                }
            record["diagnostics"] = {
                "imu_before": run.imu_diag_before,
                "imu_after": self._imu_diag_snapshot(self.controller.last_imu_diag),
                "imu_diag_note": "最近一次已收到的诊断快照，可能早于本次动作",
                "device_state_at_save": (asdict(self.controller.latest_state)
                                         if self.controller.latest_state is not None else None),
                "failure_code": run.failure_code,
                "capture_status": capture.get("status"),
                "capture_flags": capture.get("flags"),
                "capture_issues": capture.get("issues"),
                "capture_max_acc_gap_ms": capture.get("max_acc_gap_ms"),
                "capture_acc_hz": capture.get("acc_hz"),
                "capture_hx8_hz": capture.get("hx8_hz"),
                "received_pages": capture.get("received_pages"),
                "capture_transfer": record.get("capture_transfer"),
            }
        batch = self.batch
        batch_valid = False
        power_protected = False
        capture_deferred = False
        batch_issue = ""
        if batch is not None and run.batch_id == batch.id:
            batch_valid = run.motion_confirmed and batch_capture_valid(record)
            diag = self.controller.last_hx8_diag
            if (batch.signal_round is not None and run.terminal == "ABORT" and run.failure_code == "FEEDBACK_STALE" and
                    diag is not None and diag.t_ms == run.end_mcu_ms and diag.cause == "STATUS" and diag.last_status & 0x40):
                power_protected = True
                record["signal_condition_review"] = {
                    "code": HX8_POWER_PROTECTION,
                    "evidence": [{"req_id": run.req_id, "t_ms": diag.t_ms, "status": diag.last_status,
                                  "pos_mdeg": diag.last_pos_mdeg, "voltage_mv": diag.last_voltage_mv}],
                }
            record["automatic_batch"] = {"id": batch.id, "condition": batch.already_saved+batch.index+1,
                                         "repeat": batch.valid+1, "valid": batch_valid}
            if not batch_valid:
                capture = record.get("high_rate_capture", {})
                if not run.motion_confirmed:
                    batch_issue = f"HX8 功率保护，待补测 · {result}" if power_protected else result
                elif capture.get("status") == "incomplete":
                    batch_issue = f"高频回传不完整：{capture['reason']}"
                else:
                    issues = list(capture.get("issues") or ())
                    gap = capture.get("max_acc_gap_ms")
                    if gap is not None and gap > CAPTURE_MAX_ACC_GAP_MS:
                        issues.append(f"ACC 最大帧间隔 {gap} ms，超过 {CAPTURE_MAX_ACC_GAP_MS} ms")
                    if not issues or issues == ["高频窗口达到板载容量或时限"]:
                        issues.append("未确认高频数据覆盖完整运动或达到目标")
                    batch_issue = "、".join(issues)
                record["automatic_batch"]["reason"] = batch_issue
                # Persist deferral on the third consecutive acquisition failure.
                # Motion faults still follow the recovery/pause path.
                if (batch.signal_round is not None and batch.retries >= BATCH_MAX_RETRIES and run.motion_confirmed and
                        run.terminal in ("END", "SETTLED") and run.end_pos_mdeg is not None and
                        abs(run.end_pos_mdeg-run.params["target_mdeg"]) <= 3000 and not run.failure_code):
                    capture_deferred = True
                    record["signal_condition_review"] = {
                        "code": HIGH_RATE_REVIEW, "source": "automatic_deferred",
                        "evidence": [{"req_id": run.req_id}], "reason": batch_issue,
                        "consecutive_failures": batch.retries+1,
                    }
        records = self.vertical_records if vertical else self.records
        records.append(record)
        try:
            self._save(vertical)
        except OSError as exc:
            self.status.setText(f"请求 {run.req_id}：{result}；记录保存失败：{exc}")
            records.pop()
            if run.batch_id is not None:
                self._pause_batch(f"记录保存失败：{exc}")
            self._render()
            self._sync()
            return
        if manual:
            journal = {"id": record["id"], "recorded_at": record["recorded_at"],
                       "req_id": record["req_id"], "signal_round": record["signal_round"],
                       "travel": record["travel"], "speed_percent": record["speed_percent"],
                       "ramp_ms": record["ramp_ms"], "target_mdeg": record["target_mdeg"],
                       "end_pos_mdeg": record["end_pos_mdeg"], "terminal": record["terminal"],
                       "result": result, "valid": valid, "reason": issue,
                       "diagnostics": record["diagnostics"]}
            journal_error = None
            try:
                with MANUAL_RETEST_LOG_PATH.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(journal, ensure_ascii=False, separators=(",", ":")) + "\n")
            except OSError as exc:
                journal_error = str(exc)
        self._render()
        path = VERTICAL_DATA_PATH if vertical else DATA_PATH
        count = record["trace"]["acc_count"] if vertical else len(samples)
        rate = record["trace"]["acc_hz"] if vertical else None
        rate_text = f"（约 {rate:.1f} Hz）" if rate is not None else ""
        capture = record.get("high_rate_capture")
        high_rate_text = ""
        if capture is not None:
            if capture["status"] == "incomplete":
                high_rate_text = f"；高频数据未保存：{capture['reason']}"
            else:
                rates = (f"ACC {capture['acc_hz']:.0f} Hz / HX8 {capture['hx8_hz']:.0f} Hz"
                         if capture["acc_hz"] is not None and capture["hx8_hz"] is not None else "帧数不足")
                details = "，需复核：" + "、".join(capture["issues"]) if capture["issues"] else ""
                high_rate_text = f"；高频 {rates}{details}"
        self.status.setText(f"请求 {run.req_id}：{result}；采集 {count} 帧{rate_text}{high_rate_text}，已保存至 {path}")
        if manual:
            self.status.setText(self.status.text() + ("；本组有效，回收到折叠位后可选下一组" if valid else
                                                    f"；本组未通过：{issue}，可查看诊断索引并决定是否重试"))
            if journal_error:
                self.status.setText(self.status.text() + f"；诊断索引写入失败：{journal_error}")
        self._batch_run_finished(run, batch_valid, now, batch_issue, power_protected=power_protected,
                                 capture_deferred=capture_deferred)
        self._sync()

    @staticmethod
    def _add_trace(record: dict, run: ActiveRun, now: float) -> None:
        origin = run.tx_at if run.tx_at is not None else run.queued_at
        acc, states, events, optional = [], [], [], []
        for received_at, frame in run.trace or ():
            if received_at < origin - BASELINE_SECONDS:
                continue
            rx = round(received_at - origin, 6)
            if isinstance(frame, AccFrame):
                acc.append({"rx_s": rx, "mcu_ms": frame.sample_ms, "seq": frame.seq,
                            "x_g": frame.sample.ax, "y_g": frame.sample.ay, "z_g": frame.sample.az})
            elif isinstance(frame, StateFrame):
                states.append({"rx_s": rx, "mcu_ms": frame.t_ms, "pos_mdeg": frame.pos_mdeg,
                               "target_mdeg": frame.target_mdeg, "motion": frame.motion,
                               "fault": frame.fault_code, "voltage_mv": frame.voltage_mv})
            elif isinstance(frame, MotionFrame):
                events.append({"rx_s": rx, "mcu_ms": frame.t_ms, "stage": frame.stage,
                               "pos_mdeg": frame.pos_mdeg, "target_mdeg": frame.target_mdeg, "reason": frame.reason})
            else:
                optional.append({"rx_s": rx, "mcu_ms": frame.sample_ms, "seq": frame.seq,
                                 "kind": frame.kind, "xyz_raw": frame.xyz})
        gaps_ms = [(b["mcu_ms"]-a["mcu_ms"]) % (2**32) for a, b in zip(acc, acc[1:])]
        complete = (run.post_until is not None and now >= run.post_until and
                    record["result"] == run.outcome)
        record["trace"] = {"schema_version": 1, "rx_origin": "first_xmove_write" if run.tx_at is not None else "queued",
                           "baseline_acc_count": sum(a["rx_s"] < 0 for a in acc),
                           "post_acc_count": sum(run.end_at is not None and a["rx_s"] > run.end_at-origin for a in acc),
                           "acc_count": len(acc), "acc_hz": 1000 * len(gaps_ms) / sum(gaps_ms) if sum(gaps_ms) else None,
                           "max_acc_gap_ms": max(gaps_ms, default=None), "post_complete": complete,
                           "acc": acc, "state": states, "motion": events, "optional_imu": optional}

    def _delete(self, record_id: str) -> None:
        if self.controller.voltage_locked:
            return
        if self.batch is not None:
            rows = self.vertical_records if self.vertical else self.records
            if any(r.get("id") == record_id and self._record_in_current_batch(r) for r in rows):
                return
        if self.vertical:
            self.vertical_records = [r for r in self.vertical_records if r.get("id") != record_id]
        else:
            self.records = [r for r in self.records if r.get("id") != record_id]
        self._save(self.vertical)
        self._render()
        self._sync()

    def _record_in_current_batch(self, record: dict) -> bool:
        batch_record = record.get("automatic_batch", {})
        return bool(self.batch is not None and batch_record.get("valid") and
                    (batch_record.get("id") == self.batch.id or (self.batch.signal_round is not None
                     and record.get("signal_round") == self.batch.signal_round)))

    def _render(self) -> None:
        records = self.vertical_records if self.vertical else self.records
        self.table.setRowCount(len(records))
        for row, record in enumerate(records):
            axes = record.get("axis_stats", {})
            def fmt(axis: str) -> str:
                stat = axes.get(axis)
                if not stat:
                    return "无有效样本"
                return f"{stat['peak_abs_g']:.3f}/{stat.get('mean_g', stat['mean_abs_g']):+.3f}"
            aggregate = record.get("abs_axis_sum_stats")
            aggregate_text = (f"{aggregate['peak_g']:.3f}/{aggregate['mean_g']:.3f}" if aggregate else
                              "旧记录未计算" if "abs_axis_sum_stats" not in record and record.get("sample_count", 0) else
                              "无有效样本")
            capture = record.get("high_rate_capture")
            capture_text = {"ok": "高频已保存", "review": "高频需复核", "incomplete": "高频不完整"}
            suffix = f" · {capture_text.get(capture.get('status'), '高频')}" if capture else ""
            batch_record = record.get("automatic_batch") or {}
            review = (record.get("signal_condition_review") or {}).get("code")
            if review == HX8_POWER_PROTECTION:
                suffix += " · 已测试 · 功率保护待补测"
            elif review == HIGH_RATE_REVIEW:
                suffix += " · 已测试 · 高频采样待补测"
            elif batch_record and not record.get("manual_retest"):
                suffix += " · 批次有效" if batch_record.get("valid") else " · 未计入批次"
            if record.get("manual_retest"):
                suffix += " · 手动补测有效" if batch_record.get("valid") else " · 手动补测未通过"
            round_label = f" · 第 {record['signal_round']} 轮" if record.get("signal_round") is not None else ""
            if record.get("signal_retest"):
                round_label += "复测"
            values = (record.get("recorded_at", "")[:19].replace("T", " "),
                      f"{record.get('test','')} {record.get('gear') or ''}{round_label}", record.get("action", ""),
                      f"{record.get('target_mdeg',0)/1000:.1f}° / {record.get('speed_percent')}% / {record.get('ramp_ms')}ms",
                      fmt("x"), fmt("y"), fmt("z"), aggregate_text,
                      f"{record.get('result','')} / {record.get('sample_count',0)}{suffix}")
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col == 8 and capture:
                    diagnostic = record.get("diagnostics") or {}
                    item.setToolTip(batch_record.get("reason") or capture.get("reason") or "、".join(capture.get("issues", ())) or
                                    f"ACC {capture.get('acc_hz') or 0:.0f} Hz / HX8 {capture.get('hx8_hz') or 0:.0f} Hz"
                                    + (f"；故障 {diagnostic['failure_code']}" if diagnostic.get("failure_code") else ""))
                self.table.setItem(row, col, item)
            button = QPushButton("删除")
            button.setEnabled(not self._record_in_current_batch(record))
            button.clicked.connect(lambda checked=False, rid=record.get("id"): self._delete(rid))
            self.table.setCellWidget(row, 9, button)

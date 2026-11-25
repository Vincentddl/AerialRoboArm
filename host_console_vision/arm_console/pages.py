from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import time

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSpinBox,
    QScrollArea, QSlider, QVBoxLayout, QWidget,
)

from .models import VisionResult
from .black_blocks import BlackBlockConfig
from .foam_vision import FisheyeLutMapper, FoamObservation, FoamVisionConfig, yolo_available
from .position_recorder import CalibratedPositionLut
from .state import ExperimentController
from .v1_state import (
    GRIP_INSTALL_ANGLE_MDEG, GRIP_TEST_ANGLE_MDEG,
    MANUAL_EXTENSION_LIMIT_MDEG, MAX_OPERATOR_SPEED_PERCENT,
)
from .vision import VisionCanvas


SPEED_MIN, SPEED_MAX = 0.0, 100.0  # 最大实际速度由INFO给出
MANUAL_SPEED_MAX = float(MAX_OPERATOR_SPEED_PERCENT)
MANUAL_ANGLE_MAX = 70.0  # 模式一界面上限；协议保护范围保持不变
ANGLE_MIN, ANGLE_MAX = 0.0, MANUAL_EXTENSION_LIMIT_MDEG / 1000


class ManualPage(QWidget):
    def __init__(self, controller: ExperimentController, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.grab_flow = None
        self._manual_grab_started = False
        self._show_recovery_status = False
        root = QVBoxLayout(self)
        root.setSpacing(5)
        title = QLabel("手动模式")
        title.setObjectName("pageTitle")
        self.recover_button = QPushButton("故障恢复")
        self.recover_button.setToolTip("发送停止并保持，检查设备状态；不自动归零或操作夹爪")
        self.recover_button.setEnabled(False)
        title_row = QHBoxLayout()
        title_row.addWidget(title)
        title_row.addStretch()
        title_row.addWidget(self.recover_button)
        root.addLayout(title_row)
        grid = QGridLayout()
        self.speed_slider, self.speed_spin = self._add_control(grid, 0, "出臂速度", SPEED_MIN, MANUAL_SPEED_MAX, 50.0, "%（归一化）")
        self.angle_slider, self.angle_spin = self._add_control(
            grid, 1, "出臂角度（相对正常位）", ANGLE_MIN, MANUAL_ANGLE_MAX, 30.0, "°"
        )
        self.angle_spin.setToolTip(
            "相对已保存正常位的出臂行程"
        )
        root.addLayout(grid)
        buttons = QHBoxLayout()
        self.start_button = QPushButton("启动出臂")
        self.start_button.setObjectName("primaryButton")
        self.grab_button = QPushButton("按当前参数抓取")
        self.zero_button = QPushButton("归零")
        self.zero_button.setToolTip(
            "返回已保存的正常位"
        )
        self.open_button = QPushButton("夹爪张开")
        self.close_button = QPushButton("夹爪闭合")
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.grab_button)
        buttons.addWidget(self.zero_button)
        buttons.addWidget(self.open_button)
        buttons.addWidget(self.close_button)
        buttons.addStretch()
        root.addLayout(buttons)
        self.parameters = QLabel()
        self.operation = QLabel("最近操作：尚未提交")
        self.actual = QLabel("机械臂实际状态：未知／等待反馈")
        self.position_feedback = QLabel("HX8角度：目标未知 · 实际未知 · 误差未知")
        self.voltage_label = QLabel("HX8 电压：未知")
        self.fault_label = QLabel("故障：等待设备反馈")
        status_grid = QGridLayout()
        status_grid.setHorizontalSpacing(12)
        status_grid.setVerticalSpacing(3)
        status_grid.setColumnStretch(0, 1)
        status_grid.setColumnStretch(1, 1)
        for label in (self.parameters, self.operation, self.actual, self.position_feedback,
                      self.voltage_label, self.fault_label):
            label.setWordWrap(True)
        status_grid.addWidget(self.parameters, 0, 0)
        status_grid.addWidget(self.operation, 0, 1)
        status_grid.addWidget(self.actual, 1, 0)
        status_grid.addWidget(self.position_feedback, 1, 1)
        status_grid.addWidget(self.voltage_label, 2, 0)
        status_grid.addWidget(self.fault_label, 2, 1)
        root.addLayout(status_grid)
        self.video_canvas = VisionCanvas(empty_text="视频未打开\n请选择图传设备并点击打开视频")
        root.addWidget(self.video_canvas, 1)
        self.speed_spin.valueChanged.connect(self._update_parameters)
        self.angle_spin.valueChanged.connect(self._update_parameters)
        self.start_button.clicked.connect(self._start)
        self.grab_button.clicked.connect(self._grab)
        self.zero_button.clicked.connect(self._zero)
        self.open_button.clicked.connect(lambda: self._grip("OPEN"))
        self.close_button.clicked.connect(lambda: self._grip("CLOSE"))
        self.recover_button.clicked.connect(self._recover)
        controller.state_changed.connect(self._sync_state)
        self._update_parameters()

    def _add_control(self, grid, row, label, minimum, maximum, initial, unit):
        grid.addWidget(QLabel(label), row, 0)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(round(minimum * 10), round(maximum * 10))
        slider.setValue(round(initial * 10))
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(1)
        spin.setSingleStep(0.1)
        spin.setValue(initial)
        spin.setSuffix(" " + unit)
        slider.valueChanged.connect(lambda value: spin.setValue(value / 10))
        spin.valueChanged.connect(lambda value: slider.setValue(round(value * 10)))
        grid.addWidget(slider, row, 1)
        grid.addWidget(spin, row, 2)
        return slider, spin

    def _update_parameters(self) -> None:
        sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
        target = self.controller.working_zero_mdeg / 1000 + sign * self.angle_spin.value()
        maximum = self.controller.info.max_speed_dps if self.controller.info else None
        speed = (f"{round(maximum * self.speed_spin.value() / 100)}°/s（INFO 上限 {maximum}°/s）"
                 if maximum else "等待 INFO 速度上限")
        actual = self.controller.latest_state.pos_mdeg if self.controller.latest_state else None
        start = ("折叠位" if actual is not None and abs(actual - self.controller.fold_target_mdeg) <= 5_000 else
                 "正常位" if actual is not None and abs(actual - self.controller.working_zero_mdeg) <= 5_000 else "待确认")
        self.parameters.setText(
            f"待发送：速度 {self.speed_spin.value():.1f}% ≈ {speed} · 起点 {start} · "
            f"相对正常位 {self.angle_spin.value():.1f}° · 绝对目标 {target:.1f}°")

    def _start(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        speed, angle = self.speed_spin.value(), self.angle_spin.value()
        self.controller.request_extend(speed, angle)
        self.operation.setText(f"最近操作：出臂请求（速度 {speed:.1f}%，角度 {angle:.1f}°） · {self.controller.command_status}")

    def _grab(self) -> None:
        self._show_recovery_status = False
        speed, angle = self.speed_spin.value(), self.angle_spin.value()
        if self.grab_flow is None:
            self.operation.setText("最近操作：抓取流程未连接")
            return
        if self.grab_flow.start_manual_grab(speed, angle):
            self._manual_grab_started = True
            self.operation.setText(f"最近操作：抓取已提交 · 出臂 {speed:g}%／{angle:g}°，返回折叠位 10%")
        else:
            self.operation.setText(f"最近操作：抓取未启动 · {self.controller.command_status}")

    def _zero(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        self.controller.request_home()
        self.operation.setText(f"最近操作：归零 · {self.controller.command_status}")

    def _grip(self, action: str) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = False
        self.controller.request_grip(action)
        self.operation.setText(f"最近操作：夹爪{'张开' if action == 'OPEN' else '闭合'} · {self.controller.command_status}")

    def _recover(self) -> None:
        self._manual_grab_started = False
        self._show_recovery_status = True
        self.controller.request_recover()
        self.operation.setText(f"最近操作：故障恢复 · {self.controller.command_status}")

    def _sync_state(self) -> None:
        self._update_parameters()
        self.actual.setText(f"机械臂执行状态：{self.controller.execution_status}")
        idle = (not self.controller.control_busy and not self.controller.grip_ramp_active and
                (self.grab_flow is None or not self.grab_flow.running))
        self.start_button.setEnabled(idle)
        self.grab_button.setEnabled(idle and (self.grab_flow is None or not self.grab_flow.running))
        self.zero_button.setEnabled(idle)
        self.open_button.setEnabled(idle)
        self.close_button.setEnabled(idle)
        c = self.controller
        if self._manual_grab_started and self.grab_flow is not None:
            self.operation.setText(f"最近操作：手动抓取 · {self.grab_flow.flow_label.text()}")
        if self._show_recovery_status:
            self.operation.setText(f"最近操作：故障恢复 · {c.command_status}；以设备故障反馈为准")
        s = c.latest_state
        online = not c.simulated and c.link_online and s is not None
        if online and s.target_mdeg is not None and s.pos_mdeg is not None:
            error = (s.target_mdeg - s.pos_mdeg) / 1000
            self.position_feedback.setText(
                f"HX8角度（设备反馈）：目标 {s.target_mdeg / 1000:.1f}° · 实际 {s.pos_mdeg / 1000:.1f}° · 误差 {error:+.1f}°")
        else:
            self.position_feedback.setText("HX8角度：目标未知 · 实际未知 · 误差未知")
        self.voltage_label.setText(
            f"HX8 电压：{s.voltage_mv / 1000:.2f} V" if online and s.hx8_online and s.voltage_mv is not None
            else "HX8 电压：未知／无有效反馈")
        descriptions = {
            "MOTION_TIMEOUT": "运动超时，未在规定时间内满足到位条件",
            "HX8_OFFLINE": "HX8反馈离线", "FEEDBACK_STALE": "MCU中止：HX8反馈健康检查未通过；需检查原始反馈时间、电压和状态位",
            "IMU_OFFLINE": "IMU离线", "IMU_RATE_LOW": "IMU有效采样率不足",
            "NOT_READY": "执行器未就绪", "FAULT": "设备故障，请检查反馈和机构",
            "LINK_LOST": "控制会话失联", "STOPPED": "动作已中止",
        }
        if not online:
            self.fault_label.setText("故障：未知／等待设备在线反馈")
        elif s.fault_code == "NONE":
            self.fault_label.setText("故障：无（设备反馈）")
        else:
            detail = descriptions.get(s.fault_code, "设备报告故障")
            self.fault_label.setText(f"故障：{s.fault_code} · {detail}")
        self.recover_button.setEnabled(bool(online and c.mode == "MANUAL" and s.hx8_online
            and s.fault_code != "NONE" and not any(p.verb == "STOP" for p in c._pending.values())))


class VisionPage(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        root = QVBoxLayout(self)
        heading = QLabel(title)
        heading.setObjectName("pageTitle")
        root.addWidget(heading)
        self.canvas = VisionCanvas()
        root.addWidget(self.canvas, 1)
        self.source_label = QLabel("视觉源：未接入")
        self.target_label = QLabel("目标：未检测（无数据源）")
        self.coords_label = QLabel("图像坐标：—")
        self.coords_label.setWordWrap(True)
        root.addWidget(self.source_label)
        root.addWidget(self.target_label)
        root.addWidget(self.coords_label)
        self.result: VisionResult | None = None

    def set_frame(self, image, result: VisionResult) -> None:
        self.canvas.set_frame(image, result)
        self.result = result
        self.source_label.setText(f"视觉源：{result.source}{'（模拟）' if result.simulated else ''}")
        if result.black_blocks is not None:
            block = result.black_blocks[0] if result.black_blocks else None
            self.target_label.setText(
                f"黑色物块：检测到 1 个（最大合格轮廓） · {'已对齐' if result.black_block_aligned else '普通'}"
                if block else "未检测到黑色物块")
            self.coords_label.setText(
                f"视觉重心（原图像素质心）：({round(block.center_x)}, {round(block.center_y)}) px · "
                f"轮廓面积：{block.area_px:.0f} px²" if block else "图像坐标：—")
            return
        if result.detection is None:
            self.target_label.setText("目标：未检测到" if result.simulated else "目标：识别算法未接入")
            self.coords_label.setText("图像坐标：—")
        else:
            d = result.detection
            self.target_label.setText(f"目标：已检测 · 编号 {d.target_id} · 类别 {d.category} · 置信度 {d.confidence:.1%}")
            self.coords_label.setText(f"图像坐标：u={d.u:.1f} px, v={d.v:.1f} px")

    def clear_source(self) -> None:
        self.canvas.clear()
        self.result = None
        self.source_label.setText("视觉源：未接入")
        self.target_label.setText("目标：未检测（无数据源）")
        self.coords_label.setText("图像坐标：—")


class VisionDemoPage(VisionPage):
    parameters_changed = Signal()

    def __init__(self, parent=None):
        super().__init__("视觉演示模式", parent)
        root = self.layout()
        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(4, 4, 4, 4)
        for widget in (self.source_label, self.target_label, self.coords_label):
            root.removeWidget(widget)
        controls = QGridLayout()
        self.gray_threshold = QSpinBox()
        self.gray_threshold.setRange(0, 255)
        self.gray_threshold.setValue(70)
        self.min_area = QSpinBox()
        self.min_area.setRange(1, 100000)
        self.min_area.setValue(250)
        self.max_area_percent = QDoubleSpinBox()
        self.max_area_percent.setRange(0.1, 100.0)
        self.max_area_percent.setDecimals(1)
        self.max_area_percent.setValue(25.0)
        self.max_area_percent.setSuffix(" %")
        self.alignment_tolerance = QSpinBox()
        self.alignment_tolerance.setRange(0, 20)
        self.alignment_tolerance.setValue(5)
        self.alignment_tolerance.setSuffix(" px")
        for control in (self.gray_threshold, self.min_area, self.max_area_percent,
                        self.alignment_tolerance):
            control.setKeyboardTracking(False)
            control.valueChanged.connect(lambda _value: self.parameters_changed.emit())
        for column, (label, control) in enumerate((("黑色阈值 (0–255)", self.gray_threshold),
                                                   ("最小面积 (px²)", self.min_area),
                                                   ("最大画面占比", self.max_area_percent))):
            controls.addWidget(QLabel(label), 0, column)
            controls.addWidget(control, 1, column)
        controls.addWidget(QLabel("对齐容差（原图像素）"), 2, 0)
        controls.addWidget(self.alignment_tolerance, 3, 0)
        details_layout.addLayout(controls)
        self.tracking_label = QLabel("视觉跟踪：SEARCH · 等待视频帧")
        for widget in (self.source_label, self.target_label, self.coords_label):
            details_layout.addWidget(widget)
        details_layout.addWidget(QLabel("仅展示图传与识别叠加；此页面不提交机械臂运动命令。"))
        self.details_scroll = QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_scroll.setMinimumHeight(105)
        self.details_scroll.setMaximumHeight(135)
        self.details_scroll.setWidget(details)
        root.addWidget(self.details_scroll)
        root.addWidget(self.tracking_label)

    def set_frame(self, image, result: VisionResult) -> None:
        super().set_frame(image, result)
        if result.tracking_phase == "SEARCH":
            self.tracking_label.setText("视觉跟踪：SEARCH · 中央 50% 区域搜索 · 连续漏检 0 帧")
        elif result.tracking_phase == "TRACK" and result.black_blocks:
            self.tracking_label.setText("视觉跟踪：TRACK · 已锁定 · 连续漏检 0 帧")
        elif result.tracking_phase == "TRACK":
            self.tracking_label.setText(
                f"视觉跟踪：TRACK · 暂时丢失 · 连续漏检 {result.tracking_misses}/{result.tracking_miss_limit} 帧"
                + (" · 下一帧恢复 SEARCH" if result.tracking_misses >= result.tracking_miss_limit else ""))
        else:
            self.tracking_label.setText("视觉跟踪：等待识别")

    def clear_source(self) -> None:
        super().clear_source()
        self.tracking_label.setText("视觉跟踪：SEARCH · 等待视频帧")

    def detector_config(self) -> BlackBlockConfig:
        return BlackBlockConfig(self.gray_threshold.value(), self.min_area.value(),
                                self.max_area_percent.value())

    def show_pending_frame(self, image, source: str, frame_id: int) -> None:
        result = VisionResult(image.width(), image.height(), None, source, False,
                              frame_id=frame_id, roi_overlay=True)
        self.set_frame(image, result)
        self.target_label.setText("参数已更新，正在识别最新画面")
        self.coords_label.setText("图像坐标：—")


class VisionGrabPage(VisionPage):
    REGION_ANGLES = (30, 50, 70, 90, 110)  # bottom to top, travel from configured normal position
    STABLE_FRAMES = 2
    CENTER_TOLERANCE_PX = 12
    SETTLE_TOLERANCE_MDEG = 5_000
    PICK_RETURN_TOLERANCE_MDEG = 10_000  # Manual PICK only; never claims precise arrival.
    PICK_STILL_DELTA_MDEG = 1_000
    PICK_STILL_MIN_MS = 1_000
    SETTLE_DELTA_MDEG = 300
    SETTLE_SAMPLES = 3
    SETTLE_MIN_MS = 350
    SETTLE_FEEDBACK_TIMEOUT_SECONDS = 2.0
    RETURN_SETTLE_SECONDS = 0.45  # MCU resets its 300 ms HX8 stability gate on STOP.
    RETURN_DEADLINE_SECONDS = 5.0
    RETURN_MAX_ATTEMPTS = 4  # Retry only after an explicit NOT_READY rejection.
    PICK_REPLY_TIMEOUT_SECONDS = 12.0  # MCU motion deadline is 10 seconds.
    def __init__(self, controller: ExperimentController, parent=None):
        super().__init__("视觉抓取模式", parent)
        self.controller = controller
        self.running = False
        self.stage = 0
        self.request_generated = False
        self.command_sent = False
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self._start_submission_count = 0
        self._candidate_region = None
        self._stable_frames = 0
        self._active_req = None
        self._pick_started_at: float | None = None
        self._phase = "idle"
        self._settle_target = None
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._settle_stop_query_id = None
        self._settle_wait_since = None
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg: int | None = None
        self._action_speed_percent = 30.0
        self._return_speed_percent = 60.0
        self._manual_timed_grab = False
        self._manual_travel_seconds: float | None = None
        self._manual_close_deadline: float | None = None
        self._manual_done_at: float | None = None
        self._manual_pending_pick: tuple[float, float] | None = None
        self._return_confirm_after: float | None = None
        self._return_confirm_device_ms: int | None = None
        controls = QGridLayout()
        self.speed_slider, self.speed_spin = ManualPage._add_control(
            self, controls, 0, "出臂速度", 0.1, 100.0, 30.0, "%")
        self.layout().addLayout(controls)
        row = QHBoxLayout()
        self.start_button = QPushButton("启动视觉抓取")
        self.start_button.setObjectName("primaryButton")
        self.stop_button = QPushButton("停止新决策")
        self.stop_button.setEnabled(False)
        row.addWidget(self.start_button)
        row.addWidget(self.stop_button)
        row.addStretch()
        self.layout().addLayout(row)
        self.flow_label = QLabel("流程：待启动")
        self.request_label = QLabel("抓取请求：无")
        self.command_label = QLabel("控制命令：未发送")
        self.actual_label = QLabel("机械臂实际执行：未知／等待反馈")
        self.result_label = QLabel("抓取结果：未知／暂不可确认")
        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(4, 2, 4, 2)
        for widget in (self.source_label, self.target_label, self.coords_label):
            self.layout().removeWidget(widget)
            details_layout.addWidget(widget)
        for widget in (self.flow_label, self.request_label, self.command_label, self.actual_label, self.result_label):
            details_layout.addWidget(widget)
        self.details_scroll = QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_scroll.setMinimumHeight(85)
        self.details_scroll.setMaximumHeight(105)
        self.details_scroll.setWidget(details)
        self.layout().addWidget(self.details_scroll)
        self.timer = QTimer(self)
        self.timer.setInterval(700)
        self.timer.timeout.connect(self._advance_simulation)
        self.feedback_timer = QTimer(self)
        self.feedback_timer.setInterval(250)
        self.feedback_timer.timeout.connect(self._poll_motion)
        self.start_button.clicked.connect(self.start_flow)
        self.stop_button.clicked.connect(self.stop_flow)
        controller.state_changed.connect(self._sync_execution)
        controller.command_result.connect(self._on_command_result)
        controller.motion_event.connect(self._on_motion_event)

    def start_flow(self) -> None:
        if self.running:
            return
        self._action_speed_percent = self.speed_spin.value()
        self._return_speed_percent = 60.0
        if not self.controller.simulated and self.controller.mode != "MANUAL":
            if not self.controller.request_mode("MANUAL"):
                self.flow_label.setText(f"流程：{self.controller.command_status}")
                return
        elif not self.controller.simulated and not self.controller._guard(manual=True, motion=True):
            self.flow_label.setText(f"流程：{self.controller.command_status}")
            return
        self.running = True
        self._phase = "mode" if not self.controller.simulated and self.controller.mode != "MANUAL" else "watch"
        self._candidate_region = None
        self._stable_frames = 0
        self._active_req = None
        self._pick_started_at = None
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._settle_stop_query_id = None
        self._settle_wait_since = None
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg = None
        self.stage = 0
        self._manual_timed_grab = False
        self._manual_travel_seconds = None
        self._manual_close_deadline = None
        self._manual_done_at = None
        self._manual_pending_pick = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._return_confirm_after = None
        self._return_confirm_device_ms = None
        self.request_generated = False
        self.command_sent = False
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self._start_submission_count = self.controller.submitted_requests
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.flow_label.setText("流程：等待手动模式回执" if self._phase == "mode" else "流程：等待中心目标稳定")
        self.request_label.setText("抓取请求：无")
        self.command_label.setText("控制命令：未发送")
        self.actual_label.setText("机械臂实际执行：未知／等待反馈")
        self.result_label.setText("抓取结果：未知／暂不可确认")
        if self.controller.simulated:
            self.timer.start()
            self.controller.event.emit("模拟", "视觉抓取", "模拟流程已启动")
        else:
            self.controller.event.emit("视觉", "视觉抓取", "已开启；等待中心目标连续稳定")

    def start_manual_grab(self, speed: float, angle: float) -> bool:
        """Run the existing grab state machine with a manual parameter snapshot."""
        if self.running:
            return False
        if not self.controller.simulated and not self.controller._guard(manual=True, motion=True):
            return False
        self.start_flow()
        if not self.running:
            return False
        self._action_speed_percent = speed
        self._return_speed_percent = 10.0
        self._manual_timed_grab = False
        self._manual_pick = True
        if self.controller.simulated:
            self.request_label.setText(f"模拟手动抓取：出臂 {speed:g}%／{angle:g}°，返回折叠位 10%")
            return True
        # The previous PICK leaves the jaw closed. SERVO is accepted only
        # while HX8 is idle, so apply OPEN first; its physical movement can
        # overlap the following extension, without claiming position feedback.
        state = self.controller.latest_state
        if state is None or state.grip_cmd_mdeg != GRIP_TEST_ANGLE_MDEG["OPEN"]:
            if not self.controller.request_grip("OPEN"):
                self.stop_flow(send_stop=False)
                return False
            self._manual_pending_pick = (speed, angle)
            self._phase = "manual_open"
            self._active_req = self.controller._next_req - 1
            self.flow_label.setText("流程：张开夹爪PWM已提交；收到设备回执后立即出臂")
            self.request_label.setText(f"抓取请求：出臂 {speed:g}%／{angle:g}°；等待开爪回执")
            return True
        if self._submit_grab_motion(speed, angle, f"手动抓取：相对行程 {angle:g}°，速度 {speed:g}%"):
            return True
        self.stop_flow(send_stop=False)
        return False

    def _submit_grab_motion(self, speed: float, angle: float, label: str) -> bool:
        if not self.controller.request_extend(speed, angle, host_settle=False, linked_pick=True):
            self.flow_label.setText(f"流程：出臂请求被拦截：{self.controller.command_status}")
            return False
        self._phase = "pick_extend"
        self._active_req = self.controller._next_req - 1
        self._pick_started_at = time.monotonic()
        self._return_target_mdeg = (self.controller.fold_target_mdeg if self._manual_pick
                                    else self.controller.last_extend_start_mdeg)
        self._settle_target = self.controller.working_zero_mdeg + self.controller.cal.extension_sign * round(angle * 1000)
        self._settle_last = None
        self._settle_time = None
        self._settle_count = 0
        self._settle_anchor = None
        self._settle_since_ms = None
        self._pick_close_applied = False
        if self._manual_timed_grab:
            state = self.controller.latest_state
            if state is None or state.pos_mdeg is None or self.controller.info is None:
                return False
            speed_dps = max(1, round(self.controller.info.max_speed_dps * speed / 100))
            self._manual_travel_seconds = abs(self._settle_target - state.pos_mdeg) / (speed_dps * 1000)
            # The actual START event resets the clock after the queued write.
            self._manual_close_deadline = time.monotonic() + self._manual_travel_seconds
            self.controller.event.emit(
                "控制", "手动抓取闭爪时序",
                f"按本次行程和 {speed_dps}°/s 估算 {self._manual_travel_seconds:.3f}s；"
                "到时仍须确认 HX8 已停止、接近目标且无故障")
        self.feedback_timer.start()
        self.request_generated = True
        self.request_label.setText(f"抓取请求：{label}")
        self.flow_label.setText("流程：出臂中；MCU按实际位置在目标前4°应用闭爪PWM")
        return True

    def _on_motion_event(self, frame) -> None:
        if (self.running and self._phase == "pick_extend" and
                frame.req_id == self._active_req and frame.stage == "CLOSE_APPLIED"):
            self._pick_close_applied = True
        if (self.running and self._manual_timed_grab and self._phase == "extend" and
                frame.req_id == self._active_req and frame.stage == "START" and
                self._manual_travel_seconds is not None):
            self._manual_close_deadline = time.monotonic() + self._manual_travel_seconds

    def _close_or_wait_for_manual_time(self) -> None:
        if self._manual_timed_grab:
            self._phase = "timed_wait"
            self._active_req = None
            self._manual_done_at = time.monotonic()
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：HX8已停止，等待估算闭爪时刻和新鲜位置反馈")
            self._sync_execution()
        elif self.controller.request_grip("CLOSE"):
            self._phase = "close"
            self._active_req = self.controller._next_req - 1
            self.flow_label.setText("流程：HX8到位，等待闭爪PWM回执")
        else:
            self.stop_flow(send_stop=False)
            self.flow_label.setText(f"流程：闭爪被拦截：{self.controller.command_status}")

    def observe(self, result: VisionResult) -> None:
        if not self.running or self.controller.simulated or self._phase != "watch":
            return
        block = result.black_blocks[0] if result.black_blocks else None
        region = None
        if (block is not None and abs(block.center_x - result.width / 2) <= self.CENTER_TOLERANCE_PX
                and 0 <= block.center_y < result.height):
            region = 5 - min(4, int(block.center_y * 5 / result.height))
        self._stable_frames = self._stable_frames + 1 if region is not None and region == self._candidate_region else (1 if region else 0)
        self._candidate_region = region
        self.flow_label.setText(f"流程：区域{region}稳定 {self._stable_frames}/{self.STABLE_FRAMES} 帧" if region else "流程：等待中心目标")
        if self._stable_frames < self.STABLE_FRAMES:
            return
        angle = self.REGION_ANGLES[region - 1]
        if round(angle * 1000) > MANUAL_EXTENSION_LIMIT_MDEG:
            self.flow_label.setText(
                f"流程：区域{region}需出臂{angle}°，超过实测{ANGLE_MAX:g}°上限；未发送动作，请重新标定该区域")
            self._stable_frames = 0
            return
        if not self._submit_grab_motion(self._action_speed_percent, angle,
                                        f"区域{region}，相对行程{angle}°，速度{self._action_speed_percent:g}%"):
            self.flow_label.setText(f"流程：出臂请求被拦截：{self.controller.command_status}")
            self._candidate_region = None
            self._stable_frames = 0
            return

    def _on_command_result(self, req: int, verb: str, result: str) -> None:
        if not self.running or req != self._active_req:
            return
        if self._phase == "manual_open" and verb == "SERVO" and result == "APPLIED":
            snapshot = self._manual_pending_pick
            self._manual_pending_pick = None
            self._active_req = None
            self._phase = "manual_starting"
            if snapshot is not None and self._submit_grab_motion(
                    snapshot[0], snapshot[1],
                    f"手动抓取：相对行程 {snapshot[1]:g}°，速度 {snapshot[0]:g}%"):
                return
            reason = self.controller.command_status
            self.stop_flow(send_stop=False)
            self.flow_label.setText(f"流程：开爪后出臂未启动：{reason}")
            return
        if self._phase == "pick_extend" and verb == "PICK" and result == "DONE":
            self.feedback_timer.stop()
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_wait_since = now
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self._return_close_req = req
            self._return_attempts = 0
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：MCU已应用闭爪PWM并到位，等待新鲜状态后回位")
            return
        if self._phase == "extend" and verb == "MOVE" and result == "DONE":
            self.feedback_timer.stop()
            self._close_or_wait_for_manual_time()
            return
        elif self._phase == "settle_stop" and verb == "STOP" and result == "APPLIED":
            self._phase = "settle_wait"
            self._active_req = None
            self._settle_wait_since = time.monotonic()
            self.flow_label.setText("流程：已保持位置，等待无故障的空闲反馈")
            self.controller.query("STATE")
            self._settle_stop_query_id = self.controller._query_ids.get("STATE")
            self.feedback_timer.start()
            return
        elif self._phase == "pick_settle_stop" and verb == "STOP" and result == "APPLIED":
            self._phase = "pick_settle_wait"
            self._active_req = None
            self._settle_wait_since = time.monotonic()
            self.controller.query("STATE")
            self._settle_stop_query_id = self.controller._query_ids.get("STATE")
            self.feedback_timer.start()
            self.flow_label.setText("流程：闭爪后已停止并保持，等待无故障的空闲反馈后回位")
            return
        elif self._phase == "close" and verb == "SERVO" and result == "APPLIED":
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_wait_since = now
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self._return_close_req = req
            self._return_attempts = 0
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：闭爪PWM已应用，等待HX8稳定后回正")
            return
        elif self._phase == "return" and verb == "MOVE" and result == "NOT_READY":
            # STOP clears the MCU's 300 ms stability gate. An explicit ERR
            # proves this MOVE did not start, so a new request is safe later.
            now = time.monotonic()
            self._phase = "return_wait"
            self._active_req = None
            self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
            self._return_state_after = now
            self.feedback_timer.start()
            self.controller.query("STATE")
            self.flow_label.setText("流程：HX8尚未稳定，等待新反馈后重试回正")
            return
        elif self._phase == "return" and verb == "MOVE" and result in ("DONE", "SETTLED"):
            # DONE belongs to the return MOVE, but the cached STATE may still
            # describe the extension endpoint. Keep controls locked until a
            # fresh idle position confirms which preset the next PICK starts at.
            self._phase = "return_confirm"
            self._active_req = None
            self._return_confirm_after = time.monotonic()
            self._return_confirm_device_ms = (self.controller.latest_state.t_ms
                                              if self.controller.latest_state is not None else None)
            self.controller.query("STATE")
            self.feedback_timer.start()
            self.flow_label.setText("流程：回位动作已结束；等待新鲜位置反馈后允许再次抓取")
            return
        self.stop_flow(send_stop=False)
        self._phase = "aborted"
        self.flow_label.setText(f"流程：中止，{result}")
        self.controller.state_changed.emit()

    def _poll_motion(self) -> None:
        if not self.running or self._phase not in ("extend", "pick_extend", "pick_settle_stop", "settle_wait", "pick_settle_wait", "timed_wait", "return_wait", "return_confirm") or self.controller.simulated:
            self.feedback_timer.stop()
            return
        self.controller.query("STATE")
        if self._phase in ("pick_extend", "pick_settle_stop", "settle_wait", "pick_settle_wait", "timed_wait", "return_wait", "return_confirm"):
            self._sync_execution()  # Also enforce the deadline if no STATE arrives.

    def _try_close_after_hold(self) -> None:
        state = self.controller.latest_state
        if (state is None or state.motion != "IDLE" or state.fault_code != "NONE" or
                not state.hx8_online or state.pos_mdeg is None or self._settle_target is None or
                abs(state.pos_mdeg - self._settle_target) > self.SETTLE_TOLERANCE_MDEG or
                self.controller._pending or not self.controller.link_online or
                self._settle_stop_query_id is None or state.reply_to < self._settle_stop_query_id):
            return
        self.feedback_timer.stop()
        self._close_or_wait_for_manual_time()

    def _sync_execution(self) -> None:
        if not self.running or self.controller.simulated:
            return
        if self._phase == "return_confirm":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if not self.controller.link_online or (state is not None and state.fault_code != "NONE"):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：回位后的设备反馈异常，未开放再次抓取")
                return
            if (self._return_confirm_after is not None and received is not None and
                    received >= self._return_confirm_after and now - received < 1.0 and
                    state is not None and state.motion == "IDLE" and state.hx8_online and
                    (self._return_confirm_device_ms is None or state.t_ms > self._return_confirm_device_ms) and
                    state.pos_mdeg is not None and self._return_target_mdeg is not None and
                    abs(state.pos_mdeg - self._return_target_mdeg) <= self.SETTLE_TOLERANCE_MDEG):
                self.feedback_timer.stop()
                self._phase = "finished"
                self.running = False
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                position = "折叠位置" if self._return_target_mdeg == self.controller.fold_target_mdeg else "正常位置"
                self.flow_label.setText(f"流程：已返回{position}；闭爪PWM保持，可按当前参数再次抓取")
                self.result_label.setText("抓取结果：未知／无夹持反馈")
                self.controller.state_changed.emit()
            elif self._return_confirm_after is not None and now - self._return_confirm_after > 2.0:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：未取得回位后的新鲜位置反馈；请查看设备状态")
            return
        if self._phase == "pick_extend":
            state = self.controller.latest_state
            if state is not None and state.fault_code != "NONE":
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：MCU故障 {state.fault_code}，抓取已中止；不会发送回程")
                return
            if not self.controller.link_online:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：设备离线，抓取状态未知；不会继续自动动作")
                return
            received = self.controller._last_state_received_at
            if (self._manual_pick and self._pick_close_applied and
                    state is not None and state.pos_mdeg is not None and
                    self._settle_target is not None and received is not None and
                    time.monotonic() - received < 0.75 and state.t_ms != self._settle_time and
                    state.motion == "MOVING" and state.hx8_online and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"] and
                    self._active_req is not None and state.last_req_id >= self._active_req):
                near = abs(state.pos_mdeg - self._settle_target) <= self.PICK_RETURN_TOLERANCE_MDEG
                if (near and self._settle_anchor is not None and
                        abs(state.pos_mdeg - self._settle_anchor) <= self.PICK_STILL_DELTA_MDEG):
                    self._settle_count += 1
                else:
                    self._settle_count = 1 if near else 0
                    self._settle_anchor = state.pos_mdeg if near else None
                    self._settle_since_ms = state.t_ms if near else None
                self._settle_time = state.t_ms
                if (self._settle_count >= self.SETTLE_SAMPLES and self._settle_since_ms is not None and
                        state.t_ms - self._settle_since_ms >= self.PICK_STILL_MIN_MS):
                    pick_req = self._active_req
                    self._phase = "pick_settle_stop"
                    self._return_close_req = pick_req
                    self._settle_wait_since = time.monotonic()
                    self.feedback_timer.stop()
                    if self.controller.request_stop():
                        self._active_req = self.controller._next_req - 1
                        self.flow_label.setText(
                            f"流程：闭爪PWM已应用，HX8距目标{abs(state.pos_mdeg - self._settle_target) / 1000:.1f}°且位置稳定；停止后回位")
                    else:
                        self.stop_flow(send_stop=False)
                        self.flow_label.setText(f"流程：无法停止并保持：{self.controller.command_status}")
                    return
            if (self._pick_started_at is not None and
                    time.monotonic() - self._pick_started_at > self.PICK_REPLY_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=True)
                self.flow_label.setText("流程：抓取终态超时，已请求停止并保持；请查看设备反馈")
            return
        if self._phase == "pick_settle_stop":
            if (not self.controller.link_online or
                    (self._settle_wait_since is not None and
                     time.monotonic() - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS)):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：停止请求未确认，未自动回位；请查看设备状态")
            return
        if self._phase == "timed_wait":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if (state is not None and state.fault_code != "NONE") or not self.controller.link_online:
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：闭爪前设备故障或离线，已取消闭爪与回位")
                return
            if (self._manual_close_deadline is not None and self._manual_done_at is not None and
                    now - max(self._manual_close_deadline, self._manual_done_at) >
                    self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：未取得新鲜的近目标空闲反馈，未闭爪")
                return
            if (self._manual_close_deadline is not None and now >= self._manual_close_deadline and
                    self._manual_done_at is not None and received is not None and
                    received > self._manual_done_at and now - received < 1.0 and
                    state is not None and state.motion == "IDLE" and state.hx8_online and
                    state.pos_mdeg is not None and self._settle_target is not None and
                    abs(state.pos_mdeg - self._settle_target) <= self.SETTLE_TOLERANCE_MDEG and
                    not self.controller._pending):
                self.feedback_timer.stop()
                if self.controller.request_grip("CLOSE"):
                    self._phase = "close"
                    self._active_req = self.controller._next_req - 1
                    self.flow_label.setText("流程：估算时刻已到、HX8近目标空闲，等待闭爪PWM回执")
                else:
                    self.stop_flow(send_stop=False)
                    self.flow_label.setText(f"流程：闭爪被拦截：{self.controller.command_status}")
            return
        if self._phase == "settle_wait":
            if (self._settle_wait_since is not None and
                    time.monotonic() - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：STOP后未取得新鲜空闲反馈，未闭爪")
                return
            self._try_close_after_hold()
            return
        if self._phase == "pick_settle_wait":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            now = time.monotonic()
            if (not self.controller.link_online or state is None or state.fault_code != "NONE" or
                    self._settle_wait_since is None or
                    now - self._settle_wait_since > self.SETTLE_FEEDBACK_TIMEOUT_SECONDS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：闭爪后停止反馈异常，未自动回位；请查看设备状态")
                return
            if (received is not None and received >= self._settle_wait_since and now - received < 1.0 and
                    state.motion == "IDLE" and state.hx8_online and state.pos_mdeg is not None and
                    self._settle_target is not None and
                    abs(state.pos_mdeg - self._settle_target) <= self.PICK_RETURN_TOLERANCE_MDEG and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"] and
                    self._settle_stop_query_id is not None and
                    state.reply_to >= self._settle_stop_query_id and not self.controller._pending):
                self._phase = "return_wait"
                self._return_wait_since = now
                self._return_ready_after = now + self.RETURN_SETTLE_SECONDS
                self._return_state_after = now
                self._return_attempts = 0
                self.controller.query("STATE")
                self.flow_label.setText("流程：闭爪后近目标稳定保持，等待HX8就绪后回位")
            return
        if self._phase == "extend":
            state = self.controller.latest_state
            received = self.controller._last_state_received_at
            if state is not None and state.fault_code != "NONE":
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：设备故障 {state.fault_code}，未闭爪")
                return
            if (state is not None and state.pos_mdeg is not None and received is not None and
                    self._settle_target is not None and state.t_ms != self._settle_time and
                    time.monotonic() - received < 0.75 and state.motion == "MOVING" and
                    state.hx8_online):
                near = abs(state.pos_mdeg - self._settle_target) <= self.SETTLE_TOLERANCE_MDEG
                if (near and self._settle_anchor is not None and
                        abs(state.pos_mdeg - self._settle_anchor) <= self.SETTLE_DELTA_MDEG):
                    self._settle_count += 1
                else:
                    self._settle_count = 1 if near else 0
                    self._settle_anchor = state.pos_mdeg if near else None
                    self._settle_since_ms = state.t_ms if near else None
                self._settle_last = state.pos_mdeg
                self._settle_time = state.t_ms
                if (self._settle_count >= self.SETTLE_SAMPLES and self._settle_since_ms is not None and
                        state.t_ms - self._settle_since_ms >= self.SETTLE_MIN_MS):
                    self.feedback_timer.stop()
                    if self.controller.request_stop():
                        self._phase = "settle_stop"
                        self._active_req = self.controller._next_req - 1
                        self.flow_label.setText("流程：接近目标且位置稳定，正在停止并保持")
                    else:
                        self.stop_flow(send_stop=False)
                        self.flow_label.setText(f"流程：保持失败：{self.controller.command_status}")
                    return
        if self._phase == "return_wait":
            state = self.controller.latest_state
            now = time.monotonic()
            if (self._return_wait_since is None or
                    now - self._return_wait_since > self.RETURN_DEADLINE_SECONDS or
                    self._return_attempts >= self.RETURN_MAX_ATTEMPTS):
                self.stop_flow(send_stop=False)
                self.flow_label.setText("流程：回位等待超时；闭爪PWM保持，请检查HX8并手动选择正常／折叠位置")
                return
            if state is not None and state.fault_code != "NONE":
                self.feedback_timer.stop()
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：闭爪后设备故障 {state.fault_code}，回程未发送")
                self.controller.event.emit("视觉", "回程中止", self.flow_label.text())
                return
            if (state is not None and state.motion == "IDLE" and state.fault_code == "NONE"
                    and not self.controller._pending and self.controller.link_online and
                    self.controller._last_state_received_at is not None and
                    self._return_ready_after is not None and now >= self._return_ready_after and
                    self._return_state_after is not None and
                    self.controller._last_state_received_at > self._return_state_after and
                    now - self.controller._last_state_received_at < 1.0 and
                    state.grip_cmd_mdeg == GRIP_TEST_ANGLE_MDEG["CLOSE"]):
                # Set the phase before _submit emits state_changed, preventing
                # reentrant callbacks from submitting a second MOVE.
                self._phase = "return"
                if self.controller.request_visual_return(self._return_close_req,
                                                         target_mdeg=self._return_target_mdeg,
                                                         speed_percent=self._return_speed_percent):
                    self._return_attempts += 1
                    self._phase = "return"
                    self._active_req = self.controller._next_req - 1
                    position = "折叠位置" if self._return_target_mdeg == self.controller.fold_target_mdeg else "正常位置"
                    self.flow_label.setText(f"流程：闭爪PWM已应用，HX8返回{position}中")
                    self.command_label.setText(f"控制命令：已提交回程 MOVE +{self._return_target_mdeg / 1000:g}°")
                else:
                    self.stop_flow(send_stop=False)
                    self.flow_label.setText(f"流程：返回被拦截：{self.controller.command_status}")
            return
        if self._phase == "mode" and self.controller.mode == "MANUAL" and not self.controller._pending:
            self._phase = "watch"
            self.flow_label.setText("流程：等待中心目标稳定")
        self.actual_label.setText(f"机械臂实际执行：{self.controller.execution_status}")
        if (self.controller.last_request is not None and
                self.controller.last_request.action == "grab" and
                self.controller.submitted_requests > self._start_submission_count):
            self.request_label.setText("抓取请求：已产生一次")
            self.command_label.setText(f"控制命令：{self.controller.command_status}")
        if self._phase in ("watch", "extend", "pick_extend", "close", "return"):
            return
        if self.controller.ready_for_pick:
            self.flow_label.setText("流程：准备完成，等待视觉算法提供候选目标和单次触发")
        elif "闭爪命令" in self.controller.execution_status:
            self.flow_label.setText("流程：闭爪命令已应用，HX8继续出臂")
        elif "中止" in self.controller.execution_status:
            self.flow_label.setText("流程：已中止")

    def stop_flow(self, *, send_stop: bool = True) -> None:
        if not self.running:
            return
        self.timer.stop()
        self.feedback_timer.stop()
        self.running = False
        self._phase = "idle"
        self._active_req = None
        self._pick_started_at = None
        self._candidate_region = None
        self._stable_frames = 0
        self._return_wait_since = None
        self._return_ready_after = None
        self._return_state_after = None
        self._return_close_req = None
        self._return_attempts = 0
        self._return_target_mdeg = None
        self._manual_timed_grab = False
        self._manual_travel_seconds = None
        self._manual_close_deadline = None
        self._manual_done_at = None
        self._manual_pending_pick = None
        self._pick_close_applied = False
        self._manual_pick = False
        self._return_confirm_after = None
        self._return_confirm_device_ms = None
        self.controller.set_visual_target_enabled(False)
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        self.flow_label.setText("流程：已停止新决策")
        if send_stop and not self.controller.simulated:
            self.controller.request_stop()
        self.controller.event.emit("模拟" if self.controller.simulated else "视觉", "停止视觉抓取", "后续自动决策已停止；设备执行状态以回执为准")
        self.controller.state_changed.emit()

    def _advance_simulation(self) -> None:
        if not self.running or not self.controller.simulated:
            return
        self.stage += 1
        if self.stage == 1:
            self.flow_label.setText("流程：已识别模拟目标")
        elif self.stage == 2:
            self.flow_label.setText("流程：模拟目标满足示意条件")
        elif self.stage == 3:
            self.request_generated = True
            self.request_label.setText("抓取请求：已生成（模拟）")
            self.command_sent = self.controller.request_grab()
            self.command_label.setText("控制命令：模拟提交，未写入串口" if self.command_sent else "控制命令：未提交")
            self.flow_label.setText("流程：模拟抓取请求已提交")
        else:
            self.actual_execution = "模拟执行结束（非设备反馈）"
            self.grab_result = "模拟成功（非真实抓取结果）"
            self.actual_label.setText(f"机械臂实际执行：{self.actual_execution}")
            self.result_label.setText(f"抓取结果：{self.grab_result}")
            self.flow_label.setText("流程：模拟演示结束")
            self.timer.stop()
            self.running = False
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
            self.controller.event.emit("模拟", "视觉抓取", self.grab_result)

    def simulation_disabled(self) -> None:
        self.stop_flow()
        self.actual_execution = "未知／等待反馈"
        self.grab_result = "未知／暂不可确认"
        self.actual_label.setText("机械臂实际执行：未知／等待反馈")
        self.result_label.setText("抓取结果：未知／暂不可确认")
        self.command_label.setText("控制命令：未发送")
        self.request_label.setText("抓取请求：无")


def map_vertical_to_extension_deg(
    center_y: float,
    height: int,
    *,
    min_angle_deg: float,
    max_angle_deg: float,
    top_percent: float = 5.0,
    bottom_percent: float = 95.0,
    invert: bool = False,
) -> float:
    """Map vertical image coordinate (bottom -> top) linearly to relative extension angle."""
    if height <= 0:
        raise ValueError("图像高度无效")
    low_angle = min(min_angle_deg, max_angle_deg)
    high_angle = max(min_angle_deg, max_angle_deg)
    top_p = min(top_percent, bottom_percent)
    bottom_p = max(top_percent, bottom_percent)
    top_y = height * top_p / 100.0
    bottom_y = max(top_y + 1.0, height * bottom_p / 100.0)
    ratio = max(0.0, min(1.0, (bottom_y - center_y) / (bottom_y - top_y)))
    if invert:
        ratio = 1.0 - ratio
    return low_angle + ratio * (high_angle - low_angle)


class VisionFollowPage(VisionPage):
    """20 Hz fire-and-forget streaming follow (VISION_FOLLOW) with automatic step-by-step fallback.

    Strictly controls ONLY the HX8 main arm and NEVER actuates the SERVO1 gripper.
    """

    exposure_lock_toggled = Signal(bool)
    detector_mode_changed = Signal(str)
    record_lut_requested = Signal()

    STREAM_INTERVAL_SECONDS = 0.085  # ~12 Hz pacing matches HX8 100ms ramp & keeps 433MHz RF collision-free
    STREAM_REFRESH_SECONDS = 0.90    # Keepalive target refresh while holding off-zero extension
    SETTLE_COOLDOWN_SECONDS = 0.32   # Legacy step-fallback cooldown after STOP
    DONE_COOLDOWN_SECONDS = 0.31     # Legacy step-fallback cooldown after DONE
    RETRY_COOLDOWN_SECONDS = 0.05    # 50 ms retry interval on transient NOT_READY
    HX8_OFFLINE_GRACE_SECONDS = 1.80
    SIM_INTERVAL_SECONDS = 0.08

    def __init__(self, controller: ExperimentController, parent=None):
        super().__init__("视觉跟随模式", parent)
        self.controller = controller
        self.running = False
        self._phase = "idle"
        self._active_req: int | None = None
        self._mode_req: int | None = None
        self._legacy_step_fallback = False
        self._pending_return_normal = False
        self._desired_angle_deg: float | None = None
        self._filtered_angle_deg: float | None = None
        self._last_commanded_angle_deg: float | None = None
        self._pending_commanded_angle_deg: float | None = None
        self._last_stream_angle_deg: float | None = None
        self._last_stream_speed_dps: int = 120
        self._last_stream_sent_at = 0.0
        self._est_arm_ext_deg: float | None = None
        self._est_arm_updated_at = 0.0
        self._prev_raw_deg: float | None = None
        self._prev_raw_at = 0.0
        self._target_vel_dps = 0.0
        self._last_confidence_pct = 95
        self._confirm_count = 0
        self._lost_frames = 0
        self._cooldown_until = 0.0
        self._hx8_offline_since: float | None = None
        self._sim_current_angle_deg = 0.0
        self._sim_last_move_at = 0.0

        self._fisheye_mapper = None
        self._calibrated_lut: CalibratedPositionLut | None = None
        console_root = Path(__file__).resolve().parent.parent
        runtime_lut = console_root / "runtime" / "calibrated_position_lut.json"
        bundled_lut = console_root / "assets" / "calibrated_position_lut.json"
        self._calibrated_lut_path = runtime_lut if runtime_lut.is_file() else bundled_lut
        if self._calibrated_lut_path.is_file():
            try:
                self._calibrated_lut = CalibratedPositionLut.from_file(self._calibrated_lut_path)
            except Exception:
                self._calibrated_lut = None

        vision_bar = QHBoxLayout()
        vision_bar.setSpacing(8)
        vision_bar.addWidget(QLabel("识别引擎"))
        self.detector_combo = QComboBox()
        self._yolo_supported = yolo_available()
        if self._yolo_supported:
            self.detector_combo.addItem("YOLO26 专属黑块分割 (GPU·新训)", "yolo_v9")
        self.detector_combo.addItem("OpenCV 倾斜增强黑块（备选）", "opencv_black")
        vision_bar.addWidget(self.detector_combo)

        vision_bar.addWidget(QLabel("角度映射"))
        self.mapping_combo = QComboBox()
        if self._calibrated_lut is not None:
            self.mapping_combo.addItem("实测位置关系表 (30°~70°)", "calibrated_lut")
        self.mapping_combo.addItem("画面纵向线性映射（新版）", "linear")
        vision_bar.addWidget(self.mapping_combo)

        self.exposure_lock_checkbox = QCheckBox("防拖影锁曝光")
        self.exposure_lock_checkbox.setChecked(True)
        self.exposure_lock_checkbox.setToolTip("锁定摄像头低快门曝光 (exposure=-7, gain=20)，消除快速挥动时的泡沫拖影模糊")
        vision_bar.addWidget(self.exposure_lock_checkbox)

        self.adaptive_speed_checkbox = QCheckBox("速度自适应柔顺")
        self.adaptive_speed_checkbox.setChecked(True)
        self.adaptive_speed_checkbox.setToolTip("根据手部移动速度与角度差动态调节舵机速度与前瞻量，消除慢移时的‘猛冲-急刹’顿挫感")
        vision_bar.addWidget(self.adaptive_speed_checkbox)
        vision_bar.addStretch()
        self.layout().addLayout(vision_bar)

        speed_grid = QGridLayout()
        self.speed_slider, self.speed_spin = ManualPage._add_control(
            self, speed_grid, 0, "跟随速度", 10.0, 500.0, 300.0, "°/s")
        self.layout().addLayout(speed_grid)

        params = QGridLayout()
        params.setHorizontalSpacing(10)
        params.setVerticalSpacing(4)
        self.min_angle_spin = self._make_double_spin(0.0, MANUAL_ANGLE_MAX, 0.0, 1.0, " °")
        self.max_angle_spin = self._make_double_spin(0.0, ANGLE_MAX, 70.0, 1.0, " °")
        self.deadband_spin = self._make_double_spin(0.2, 25.0, 1.2, 0.2, " °")
        self.deadband_spin.setToolTip("流式跟随下的角度死区（默认 1.2°，抑制悬停微抖）；旧固件步进回退时自动按 ≥2.5° 防抖")
        self.smoothing_spin = self._make_double_spin(0.10, 1.00, 0.70, 0.05, "", decimals=2)
        self.smoothing_spin.setToolTip("自适应低通滤波基准权重（默认 0.70：静止时自动加强滤噪，快速移动时自动零延迟放行；1.00 为完全无滤波）")
        self.confirm_frames_spin = QSpinBox()
        self.confirm_frames_spin.setRange(1, 10)
        self.confirm_frames_spin.setValue(1)
        self.confirm_frames_spin.setSuffix(" 帧")
        self.confirm_frames_spin.setToolTip("连续检测到有效泡沫块的确认帧数（流式跟随默认 1 帧零延迟响应）")
        self.angle_trim_spin = self._make_double_spin(-45.0, 45.0, 0.0, 0.5, " °")
        self.angle_trim_spin.setToolTip("鱼眼+LUT 映射下的安装零点微调角（默认 0.0°）")
        self.corridor_half_width_spin = QSpinBox()
        self.corridor_half_width_spin.setRange(20, 240)
        self.corridor_half_width_spin.setSingleStep(5)
        self.corridor_half_width_spin.setValue(75)
        self.corridor_half_width_spin.setSuffix(" px")
        self.corridor_half_width_spin.setToolTip("中轴线左右 ±N 像素识别走廊（默认 ±75px 完整包住机械臂运动平面并屏蔽左侧暗门缝；近距贴脸跨中线自动放行）")
        self.yolo_conf_spin = self._make_double_spin(0.25, 0.95, 0.35, 0.05, "", decimals=2)
        self.yolo_conf_spin.setToolTip("YOLO26 专属黑块分割最低置信度阈值（默认 0.35，配合黑色表面校验实现 98%+ 近距检出与 0 杂物误检）")
        self.top_margin_spin = self._make_double_spin(0.0, 45.0, 5.0, 1.0, " %")
        self.bottom_margin_spin = self._make_double_spin(55.0, 100.0, 95.0, 1.0, " %")
        self.invert_checkbox = QCheckBox("上下反向映射")
        self.invert_checkbox.setToolTip("默认画面底部对应最小出臂角、顶部对应最大出臂角；勾选后反转")

        fields = (
            (0, 0, "最小出臂角", self.min_angle_spin),
            (0, 1, "最大出臂角", self.max_angle_spin),
            (0, 2, "触发死区", self.deadband_spin),
            (0, 3, "确认帧数", self.confirm_frames_spin),
            (0, 4, "中轴半宽", self.corridor_half_width_spin),
            (2, 0, "画面上边界", self.top_margin_spin),
            (2, 1, "画面下边界", self.bottom_margin_spin),
            (2, 2, "平滑系数", self.smoothing_spin),
            (2, 3, "YOLO置信度", self.yolo_conf_spin),
            (2, 4, "零点微调", self.angle_trim_spin),
        )
        for row_idx, col_idx, text, widget in fields:
            params.addWidget(QLabel(text), row_idx, col_idx)
            params.addWidget(widget, row_idx + 1, col_idx)
        self.layout().addLayout(params)
        vision_bar.addWidget(self.invert_checkbox)

        buttons = QHBoxLayout()
        self.start_button = QPushButton("启动视觉跟随")
        self.start_button.setObjectName("primaryButton")
        self.stop_button = QPushButton("停止跟随")
        self.stop_button.setEnabled(False)
        self.return_normal_button = QPushButton("回正常位（不动夹爪）")
        self.return_normal_button.setToolTip("跟随停止且空闲时，仅控制 HX8 回到已保存正常位，不改变夹爪状态")
        self.record_lut_button = QPushButton("录制位置关系表 (17点)")
        self.record_lut_button.setToolTip("弹出非模态录制窗口：按 R 录视频，Space 录 30°~70° 去程+回程每 5° 保持 3 秒")
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_button)
        buttons.addWidget(self.return_normal_button)
        buttons.addWidget(self.record_lut_button)
        buttons.addStretch()
        self.layout().addLayout(buttons)

        self.mapping_label = QLabel("映射目标：等待视频帧")
        self.mapping_label.setWordWrap(True)
        self.flow_label = QLabel("流程：待启动（仅预览映射角度，不发运动指令，全程不控制夹爪）")
        self.flow_label.setWordWrap(True)
        self.command_label = QLabel("控制命令：未发送")
        self.command_label.setWordWrap(True)
        self.actual_label = QLabel("机械臂实际状态：未知／等待反馈")
        self.actual_label.setWordWrap(True)
        hint_label = QLabel(
            "默认启用新机架实拍训练的 YOLO26n-seg 黑色物块分割（全尺度真实掩膜质心 + 近距贴脸防漏检 + 暗门缝/夹爪零误检）+ 17点实测位置关系表 + 20Hz 流式控制；绝不操作夹爪。"
        )
        hint_label.setWordWrap(True)

        details = QWidget()
        details_layout = QVBoxLayout(details)
        details_layout.setContentsMargins(4, 2, 4, 2)
        for widget in (self.source_label, self.target_label, self.coords_label):
            self.layout().removeWidget(widget)
            details_layout.addWidget(widget)
        for widget in (self.mapping_label, self.flow_label, self.command_label, self.actual_label, hint_label):
            details_layout.addWidget(widget)
        self.details_scroll = QScrollArea()
        self.details_scroll.setWidgetResizable(True)
        self.details_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_scroll.setMinimumHeight(95)
        self.details_scroll.setMaximumHeight(125)
        self.details_scroll.setWidget(details)
        self.layout().addWidget(self.details_scroll)

        self.feedback_timer = QTimer(self)
        self.feedback_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.feedback_timer.setInterval(50)
        self.feedback_timer.timeout.connect(self._poll_state)

        for spin in (self.min_angle_spin, self.max_angle_spin, self.deadband_spin,
                     self.smoothing_spin, self.confirm_frames_spin, self.angle_trim_spin,
                     self.corridor_half_width_spin, self.yolo_conf_spin,
                     self.top_margin_spin, self.bottom_margin_spin):
            spin.setKeyboardTracking(False)
            spin.valueChanged.connect(lambda _v: self._on_mapping_parameters_changed())
        self.corridor_half_width_spin.valueChanged.connect(
            lambda _v: self.detector_mode_changed.emit(self.detector_mode)
        )
        self.yolo_conf_spin.valueChanged.connect(
            lambda _v: self.detector_mode_changed.emit(self.detector_mode)
        )
        self.invert_checkbox.toggled.connect(lambda _v: self._on_mapping_parameters_changed())
        self.mapping_combo.currentIndexChanged.connect(lambda _i: self._on_mapping_mode_changed())
        self.detector_combo.currentIndexChanged.connect(
            lambda _i: self.detector_mode_changed.emit(self.detector_mode)
        )
        self.exposure_lock_checkbox.toggled.connect(self.exposure_lock_toggled.emit)
        self.start_button.clicked.connect(self.start_flow)
        self.stop_button.clicked.connect(lambda: self.stop_flow(send_stop=True))
        self.return_normal_button.clicked.connect(self._on_return_normal_clicked)
        self.record_lut_button.clicked.connect(self.record_lut_requested.emit)
        controller.state_changed.connect(self._sync_execution)
        controller.command_result.connect(self._on_command_result)
        self._on_mapping_mode_changed()
        self._sync_buttons()

    @property
    def detector_mode(self) -> str:
        return str(self.detector_combo.currentData() or "opencv_black")

    @property
    def mapping_mode(self) -> str:
        return str(self.mapping_combo.currentData() or "linear")

    def set_detector_mode(self, mode: str) -> None:
        idx = self.detector_combo.findData(mode)
        if idx >= 0:
            self.detector_combo.setCurrentIndex(idx)

    def set_mapping_mode(self, mode: str) -> None:
        idx = self.mapping_combo.findData(mode)
        if idx >= 0:
            self.mapping_combo.setCurrentIndex(idx)

    def reset_firmware_capability_cache(self) -> None:
        self._legacy_step_fallback = False

    def apply_calibrated_lut(self, lut: CalibratedPositionLut, path: str = "") -> None:
        self._calibrated_lut = lut
        idx = self.mapping_combo.findData("calibrated_lut")
        if idx < 0:
            self.mapping_combo.insertItem(0, "实测位置关系表 (30°~70°)", "calibrated_lut")
            idx = 0
        self.mapping_combo.setCurrentIndex(idx)
        self.controller.event.emit("视觉", "位置关系表", f"已加载实测 17 点位置关系表：{path or lut.name}")
        self._on_mapping_mode_changed()

    @staticmethod
    def _make_double_spin(minimum: float, maximum: float, initial: float,
                          step: float, suffix: str, *, decimals: int = 1) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setValue(initial)
        if suffix:
            spin.setSuffix(suffix)
        return spin

    @staticmethod
    def detector_config(base: BlackBlockConfig, *, corridor_half_width_px: int | None = None) -> BlackBlockConfig:
        """OpenCV fallback config: separated from >=64 cabinet strip (threshold 56), close-up up to 65% area."""
        return replace(
            base,
            gray_threshold=min(base.gray_threshold, 56),
            max_area_percent=65.0,
            min_fill_ratio=min(base.min_fill_ratio, 0.45),
            track_miss_limit=2,
            track_margin_px=160,
            track_growth_px=40,
            track_max_center_distance_px=160,
            instant_reacquire=True,
            use_rotated_fill_ratio=True,
            allow_roi_touch=True,
            center_corridor_half_width_px=corridor_half_width_px,
        )

    def current_black_detector_config(self, base: BlackBlockConfig) -> BlackBlockConfig:
        return self.detector_config(base, corridor_half_width_px=self.corridor_half_width_spin.value())

    def foam_vision_config(self) -> FoamVisionConfig:
        return FoamVisionConfig(
            conf_threshold=float(self.yolo_conf_spin.value()),
            center_half_width_px=int(self.corridor_half_width_spin.value()),
        )

    def _on_mapping_mode_changed(self) -> None:
        is_fisheye = self.mapping_mode == "fisheye_lut" and self._fisheye_mapper is not None
        is_linear = self.mapping_mode == "linear"
        self.angle_trim_spin.setEnabled(is_fisheye)
        self.top_margin_spin.setEnabled(is_linear)
        self.bottom_margin_spin.setEnabled(is_linear)
        self._on_mapping_parameters_changed()

    def _sync_buttons(self) -> None:
        self.start_button.setEnabled(not self.running)
        self.stop_button.setEnabled(self.running)
        self.return_normal_button.setEnabled(not self.running and not self.controller.control_busy)

    def _on_return_normal_clicked(self) -> None:
        if self.running:
            return
        if not self.controller.simulated and self.controller.mode != "MANUAL":
            self._pending_return_normal = True
            if not self.controller.request_mode("MANUAL"):
                self._pending_return_normal = False
                self.flow_label.setText(f"流程：{self.controller.command_status}")
            else:
                self.flow_label.setText("流程：正在切回手动模式后返回正常位（不操作夹爪）")
            return
        if not self.controller.request_follow_return_normal():
            self.flow_label.setText(f"流程：{self.controller.command_status}")
        else:
            self.flow_label.setText("流程：正在返回正常位（仅移动 HX8，夹爪保持原状）")

    def _on_mapping_parameters_changed(self) -> None:
        self._filtered_angle_deg = None
        self._last_commanded_angle_deg = None
        self._last_stream_angle_deg = None
        if self.result is not None:
            self._update_desired_from_result(self.result)
            if self.running:
                self._try_dispatch_follow()

    def set_yolo_observation(self, image, frame_id: int, width: int, height: int,
                             source_name: str, obs: FoamObservation) -> None:
        det = obs.detection
        blocks = (det.block,) if det is not None else ()
        if det is not None:
            self._last_confidence_pct = max(60, min(100, round(det.confidence * 100)))
        result = VisionResult(
            width=width,
            height=height,
            detection=None,
            source=source_name,
            simulated=False,
            black_blocks=blocks,
            frame_id=frame_id,
            roi_overlay=False,
            center_segments_overlay=True,
            corridor_half_width_px=self.corridor_half_width_spin.value(),
            tracking_phase="TRACK" if det is not None else ("SEARCH" if obs.missed_frames >= 2 else "TRACK"),
            tracking_misses=obs.missed_frames,
            tracking_miss_limit=8,
        )
        super().set_frame(image, result)
        if det is not None:
            anchor_zh = "分割掩膜质心" if det.center_source == "mask_centroid" else "检测框中心"
            self.target_label.setText(
                f"YOLO26 已锁定黑块中心 · 置信度 {det.confidence:.0%} · {anchor_zh} · {obs.device_label} {obs.inference_ms:.1f} ms"
            )
            self.coords_label.setText(
                f"中心坐标：({det.block.center_x:.1f}, {det.block.center_y:.1f}) px · 掩膜面积 {det.block.area_px:.0f} px²"
            )
        else:
            self.target_label.setText(
                f"YOLO26 未检测到黑色物块 · {obs.device_label} {obs.inference_ms:.1f} ms"
            )
            self.coords_label.setText("中心坐标：—")
        self._update_desired_from_result(result)

    def set_frame(self, image, result: VisionResult) -> None:
        self._last_confidence_pct = 95
        if not result.simulated:
            result = replace(
                result,
                roi_overlay=False,
                center_segments_overlay=True,
                corridor_half_width_px=self.corridor_half_width_spin.value(),
            )
        super().set_frame(image, result)
        self._update_desired_from_result(result)

    def clear_source(self) -> None:
        super().clear_source()
        self._desired_angle_deg = None
        self._filtered_angle_deg = None
        self._prev_raw_deg = None
        self._target_vel_dps = 0.0
        self._confirm_count = 0
        self._lost_frames = 0
        self.mapping_label.setText("映射目标：等待视频帧")

    def _extract_pixel_target(self, result: VisionResult) -> tuple[float, float, int, int] | None:
        if result.height <= 0 or result.width <= 0:
            return None
        if result.simulated:
            if result.detection is not None:
                return result.detection.u, result.detection.v, result.width, result.height
            return None
        block = result.black_blocks[0] if result.black_blocks else None
        if block is not None and 0 <= block.center_y <= result.height:
            return block.center_x, block.center_y, result.width, result.height
        return None

    def _update_desired_from_result(self, result: VisionResult) -> None:
        extracted = self._extract_pixel_target(result)
        if extracted is None:
            self._lost_frames += 1
            self._confirm_count = 0
            self._desired_angle_deg = None
            self._target_vel_dps *= 0.5
            if result.tracking_phase == "SEARCH" or self._lost_frames >= 2:
                self._filtered_angle_deg = None
                self._prev_raw_deg = None
                self._target_vel_dps = 0.0
            self.mapping_label.setText("映射目标：未检测到泡沫块（保持当前位置）")
            if self.running and self._phase in ("stream_follow", "step_follow") and self._active_req is None:
                mode_tag = "20Hz流式" if self._phase == "stream_follow" else "步进"
                self.flow_label.setText(f"流程：{mode_tag}跟随待命 · 未检测到泡沫块，机械臂平稳保持当前位置")
            return
        center_x, center_y, width, height = extracted
        self._lost_frames = 0
        self._confirm_count += 1
        use_calibrated = (
            not result.simulated
            and self.mapping_mode == "calibrated_lut"
            and self._calibrated_lut is not None
        )
        use_fisheye = (
            not result.simulated
            and self.mapping_mode == "fisheye_lut"
            and self._fisheye_mapper is not None
        )
        bearing_note = ""
        if use_calibrated:
            raw_deg = self._calibrated_lut.pixel_y_to_extension_deg(
                center_y,
                height,
                min_angle_deg=self.min_angle_spin.value(),
                max_angle_deg=self.max_angle_spin.value(),
                invert=self.invert_checkbox.isChecked(),
            )
            bearing_note = "实测表 → "
        elif use_fisheye:
            raw_deg, bearing = self._fisheye_mapper.pixel_to_extension_deg(
                center_x,
                center_y,
                width,
                height,
                min_ext_deg=self.min_angle_spin.value(),
                max_ext_deg=self.max_angle_spin.value(),
                angle_trim_deg=self.angle_trim_spin.value(),
                invert=self.invert_checkbox.isChecked(),
            )
            bearing_note = f"视线角 {bearing.optical_offset_deg:+.1f}° → "
        else:
            raw_deg = map_vertical_to_extension_deg(
                center_y,
                height,
                min_angle_deg=self.min_angle_spin.value(),
                max_angle_deg=self.max_angle_spin.value(),
                top_percent=self.top_margin_spin.value(),
                bottom_percent=self.bottom_margin_spin.value(),
                invert=self.invert_checkbox.isChecked(),
            )
        now = time.monotonic()
        base_alpha = self.smoothing_spin.value()
        # 1€-style velocity-adaptive filter: strong anti-jitter when still, crisp zero-lag when moving!
        if self._filtered_angle_deg is None:
            self._filtered_angle_deg = raw_deg
            self._target_vel_dps = 0.0
        else:
            delta_raw = abs(raw_deg - self._filtered_angle_deg)
            if base_alpha >= 0.99:
                eff_alpha = 1.0
            else:
                motion_ratio = min(1.0, max(0.0, (delta_raw - 0.5) / 4.0))
                min_alpha = max(0.18, base_alpha * 0.35)
                max_alpha = min(0.92, base_alpha + 0.20)
                eff_alpha = min_alpha + motion_ratio * (max_alpha - min_alpha)
            prev_filt = self._filtered_angle_deg
            self._filtered_angle_deg += eff_alpha * (raw_deg - self._filtered_angle_deg)
            dt = now - self._prev_raw_at
            if 0.012 <= dt <= 0.25:
                if self._prev_raw_deg is not None and abs(raw_deg - self._prev_raw_deg) < 0.35:
                    self._target_vel_dps *= 0.25
                else:
                    inst_vel = (self._filtered_angle_deg - prev_filt) / dt
                    self._target_vel_dps = 0.45 * self._target_vel_dps + 0.55 * inst_vel
            else:
                self._target_vel_dps = 0.0
        self._prev_raw_deg = raw_deg
        self._prev_raw_at = now
        desired = round(self._filtered_angle_deg, 1)
        self._desired_angle_deg = desired
        sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
        abs_target = self.controller.working_zero_mdeg / 1000 + sign * desired
        required = self.confirm_frames_spin.value()
        confirm_note = f"确认 {min(self._confirm_count, required)}/{required} 帧"
        self.mapping_label.setText(
            f"映射目标：y={center_y:.0f}/{height} px · {bearing_note}行程 {desired:.1f}°"
            f"（HX8 绝对 {abs_target:+.1f}°） · {confirm_note}"
        )

    def start_flow(self) -> None:
        if self.running:
            return
        self._pending_return_normal = False
        if self.controller.voltage_locked:
            self.flow_label.setText("流程：低压锁定，等待供电恢复")
            return
        if not self.controller.simulated:
            if not self.controller._require_link():
                self.flow_label.setText(f"流程：{self.controller.command_status}")
                return
            state = self.controller.latest_state
            if state is None or not state.hx8_online or state.fault_code not in ("NONE", "STOPPED"):
                self.flow_label.setText("流程：HX8离线或存在故障，请先在手动模式检查并恢复")
                return
            if not self._legacy_step_fallback:
                if self.controller.mode == "VISION_FOLLOW":
                    self._phase = "stream_follow"
                else:
                    if not self.controller.request_mode("VISION_FOLLOW"):
                        self.flow_label.setText(f"流程：{self.controller.command_status}")
                        return
                    self._phase = "enter_stream"
                    self._mode_req = self.controller._next_req - 1
            else:
                if self.controller.mode != "MANUAL":
                    if not self.controller.request_mode("MANUAL"):
                        self.flow_label.setText(f"流程：{self.controller.command_status}")
                        return
                    self._phase = "enter_step_manual"
                    self._mode_req = self.controller._next_req - 1
                else:
                    self._phase = "step_follow"
        else:
            self._phase = "sim_follow"
            self._sim_current_angle_deg = 0.0
            self._sim_last_move_at = 0.0

        self.running = True
        self._active_req = None
        self._last_commanded_angle_deg = None
        self._pending_commanded_angle_deg = None
        self._last_stream_angle_deg = None
        self._last_stream_sent_at = 0.0
        self._est_arm_ext_deg = None
        self._est_arm_updated_at = time.monotonic()
        self._cooldown_until = 0.0
        self._hx8_offline_since = None
        self._sync_buttons()
        self.command_label.setText("控制命令：等待泡沫块目标进入死区外")
        if self.controller.simulated:
            self.flow_label.setText("流程：模拟视觉跟随已启动（不控制夹爪）")
            self.controller.event.emit("模拟", "视觉跟随", "模拟视觉跟随已启动")
        else:
            self.feedback_timer.start()
            self.controller.query("STATE")
            if self._phase == "enter_stream":
                self.flow_label.setText("流程：正在请求进入 VISION_FOLLOW 20Hz 流式跟随模式…")
            elif self._phase == "stream_follow":
                self.flow_label.setText("流程：20Hz 丝滑流式跟随已开启（不控制夹爪）")
            elif self._phase == "enter_step_manual":
                self.flow_label.setText("流程：等待切换手动模式回执（兼容步进跟随）")
            else:
                self.flow_label.setText("流程：兼容步进跟随已开启；刷入新固件可启用 20Hz 无停顿流式跟随")
            self.controller.event.emit("视觉", "视觉跟随", "已启动视觉跟随（仅控制 HX8，不操作夹爪）")
        self.controller.state_changed.emit()
        if self.result is not None:
            self._update_desired_from_result(self.result)
            self._try_dispatch_follow()

    def observe(self, result: VisionResult) -> None:
        if not self.running:
            return
        self._try_dispatch_follow()

    def _poll_state(self) -> None:
        if not self.running or self.controller.simulated:
            self.feedback_timer.stop()
            return
        # In streaming mode, v1_state.py already queries STATE/PING every 1.0 s;
        # avoid high-rate GET STATE queries that collide with V packets on 433 MHz half-duplex air!
        now = time.monotonic()
        if self._phase != "stream_follow" or self.controller._last_state_received_at is None or (
            now - self.controller._last_state_received_at >= 0.85
        ):
            self.controller.query("STATE")
        self._try_dispatch_follow()

    def _sync_execution(self) -> None:
        self._sync_buttons()
        self.actual_label.setText(f"机械臂实际状态：{self.controller.execution_status}")
        if self._pending_return_normal and not self.running and self.controller.mode == "MANUAL" and not self.controller._pending:
            self._pending_return_normal = False
            self.controller.request_follow_return_normal()
            return
        if not self.running or self.controller.simulated:
            return
        if self.controller.voltage_locked:
            self.stop_flow(send_stop=False)
            self.flow_label.setText("流程：低压锁定，已停止视觉跟随")
            return
        if not self.controller.link_online:
            self.stop_flow(send_stop=False)
            self.flow_label.setText("流程：无线链路离线，已停止视觉跟随")
            return
        state = self.controller.latest_state
        now = time.monotonic()
        if state is not None and (state.fault_code == "HX8_OFFLINE" or not state.hx8_online):
            if self._hx8_offline_since is None:
                self._hx8_offline_since = now
            if now - self._hx8_offline_since < self.HX8_OFFLINE_GRACE_SECONDS:
                self.flow_label.setText("流程：HX8反馈短暂缺帧，等待自动恢复后继续跟随")
                return
            self.stop_flow(send_stop=False)
            self.flow_label.setText("流程：HX8持续离线，已停止视觉跟随；请检查舵机供电与接线")
            return
        self._hx8_offline_since = None
        if state is not None and state.fault_code not in ("NONE", "STOPPED"):
            self.stop_flow(send_stop=False)
            self.flow_label.setText(f"流程：设备报告故障 {state.fault_code}，已停止视觉跟随")
            return
        if self._phase == "enter_stream" and self.controller.mode == "VISION_FOLLOW":
            self._phase = "stream_follow"
            self._mode_req = None
            self.flow_label.setText("流程：已进入 VISION_FOLLOW 20Hz 无停顿流式跟随（不控制夹爪）")
        elif self._phase == "enter_step_manual" and self.controller.mode == "MANUAL" and not self.controller._pending:
            self._phase = "step_follow"
            self._mode_req = None
            self.flow_label.setText("流程：兼容模式跟随中（刷入新固件后可升级为 20Hz 丝滑流式跟随）")
        self._try_dispatch_follow()

    def _try_dispatch_follow(self) -> None:
        if not self.running:
            return
        if self.controller.simulated:
            if self._desired_angle_deg is None or self._confirm_count < self.confirm_frames_spin.value():
                return
            now = time.monotonic()
            deadband = self.deadband_spin.value()
            step_deg = abs(self._desired_angle_deg - self._sim_current_angle_deg)
            if now - self._sim_last_move_at >= self.SIM_INTERVAL_SECONDS and step_deg >= deadband:
                speed_dps = round(self.speed_spin.value())
                target_deg = self._desired_angle_deg
                if self.controller.submit_follow_stream_target(target_deg, speed_dps, self._last_confidence_pct):
                    self._sim_last_move_at = now
                    previous = self._sim_current_angle_deg
                    self._sim_current_angle_deg = target_deg
                    self.command_label.setText(
                        f"控制命令：模拟流式跟随 {previous:.1f}° → {target_deg:.1f}°（速度 {speed_dps}°/s，不动夹爪）"
                    )
                    self.flow_label.setText(f"流程：模拟 20Hz 流式跟随中 · 当前模拟行程 {target_deg:.1f}°")
            return

        if self._phase == "stream_follow":
            self._dispatch_stream_follow()
        elif self._phase == "step_follow":
            self._dispatch_step_fallback()

    def _dispatch_stream_follow(self) -> None:
        now = time.monotonic()
        if now - self._last_stream_sent_at < self.STREAM_INTERVAL_SECONDS:
            return
        if self._desired_angle_deg is None:
            return
        required = self.confirm_frames_spin.value()
        if self._confirm_count < required:
            self.flow_label.setText(f"流程：目标确认中 {self._confirm_count}/{required} 帧")
            return
        state = self.controller.latest_state
        if (state is None or not state.hx8_online or state.pos_mdeg is None or
                self.controller.cal is None or self.controller.cal.extension_sign is None):
            return
        sign = self.controller.cal.extension_sign
        actual_ext_deg = sign * (state.pos_mdeg - self.controller.working_zero_mdeg) / 1000.0

        # Advance smooth kinematic arm-position estimator between 1 Hz STATE frames
        if self._est_arm_ext_deg is None:
            self._est_arm_ext_deg = actual_ext_deg
            self._est_arm_updated_at = now
        else:
            dt_est = max(0.0, min(0.25, now - self._est_arm_updated_at))
            self._est_arm_updated_at = now
            if self._last_stream_angle_deg is not None and dt_est > 0:
                rem = self._last_stream_angle_deg - self._est_arm_ext_deg
                step = min(abs(rem), self._last_stream_speed_dps * dt_est)
                self._est_arm_ext_deg += math.copysign(step, rem) if rem != 0 else 0.0
            if self.controller._last_state_received_at is not None and (now - self.controller._last_state_received_at) < 0.15:
                self._est_arm_ext_deg = 0.65 * actual_ext_deg + 0.35 * self._est_arm_ext_deg

        low_lim = min(self.min_angle_spin.value(), self.max_angle_spin.value())
        high_lim = max(self.min_angle_spin.value(), self.max_angle_spin.value())
        desired = self._desired_angle_deg
        max_dps = self.controller.info.max_speed_dps if self.controller.info is not None else 500
        user_max_dps = min(max_dps, max(15, round(self.speed_spin.value())))

        vel = self._target_vel_dps
        if self.adaptive_speed_checkbox.isChecked():
            # Project target slightly ahead along smooth motion (85 ms lookahead) so HX8 stays in cruise
            # instead of entering its 100 ms braking ramp on every intermediate frame!
            lead_deg = max(-6.5, min(6.5, 0.085 * vel)) if abs(vel) >= 5.0 else 0.0
            cmd_deg = round(max(low_lim, min(high_lim, desired + lead_deg)), 1)
            tracking_err = abs(desired - self._est_arm_ext_deg)
            catchup_dps = 4.5 * tracking_err + 0.22 * (tracking_err ** 2)
            raw_speed = 1.15 * abs(vel) + catchup_dps + 22.0
            speed_dps = min(user_max_dps, max(35, round(raw_speed)))
        else:
            cmd_deg = desired
            speed_dps = user_max_dps

        deadband = self.deadband_spin.value()
        actively_moving = abs(vel) >= 8.0 or abs(desired - self._est_arm_ext_deg) >= 2.5
        effective_deadband = min(deadband, 0.35) if actively_moving else deadband
        if self._last_stream_angle_deg is not None:
            delta_cmd = abs(cmd_deg - self._last_stream_angle_deg)
            need_refresh = (
                now - self._last_stream_sent_at >= self.STREAM_REFRESH_SECONDS
                and abs(cmd_deg - actual_ext_deg) >= max(0.8, deadband)
            )
            if delta_cmd < effective_deadband and not need_refresh:
                self.flow_label.setText(
                    f"流程：柔顺锁定 · 目标行程 {desired:.1f}° · 实际 {actual_ext_deg:.1f}°"
                )
                return

        if self.controller.submit_follow_stream_target(cmd_deg, speed_dps, self._last_confidence_pct):
            self._last_stream_angle_deg = cmd_deg
            self._last_stream_speed_dps = speed_dps
            self._last_stream_sent_at = now
            abs_target = self.controller.working_zero_mdeg / 1000 + sign * cmd_deg
            cur_abs = state.pos_mdeg / 1000
            self.command_label.setText(
                f"控制命令：柔顺流式 → 绝对 {abs_target:+.1f}°（反馈 {cur_abs:+.1f}°，动态速度 {speed_dps}°/s）"
            )
            self.flow_label.setText(
                f"流程：柔顺流式跟随中 → 目标 {abs_target:+.1f}°（行程 {cmd_deg:.1f}°，速度 {speed_dps}°/s）"
            )

    def _dispatch_step_fallback(self) -> None:
        now = time.monotonic()
        if now < self._cooldown_until or self._active_req is not None or self.controller.control_busy:
            return
        if self._desired_angle_deg is None:
            return
        required = self.confirm_frames_spin.value()
        if self._confirm_count < required:
            self.flow_label.setText(f"流程：目标确认中 {self._confirm_count}/{required} 帧")
            return
        state = self.controller.latest_state
        if (state is None or not state.hx8_online or state.motion != "IDLE" or
                state.fault_code != "NONE" or state.pos_mdeg is None or
                self.controller._last_state_received_at is None or
                now - self.controller._last_state_received_at > 1.0 or
                not self.controller.link_online or self.controller.cal is None or
                self.controller.cal.extension_sign is None):
            return
        sign = self.controller.cal.extension_sign
        actual_ext_deg = sign * (state.pos_mdeg - self.controller.working_zero_mdeg) / 1000.0
        desired = self._desired_angle_deg
        deadband = max(2.5, self.deadband_spin.value())
        error_deg = abs(desired - actual_ext_deg)
        if error_deg < deadband:
            self.flow_label.setText(
                f"流程：兼容模式已对齐 · 实际行程 {actual_ext_deg:.1f}°（偏差 {error_deg:.1f}° < 死区 {deadband:.1f}°）"
            )
            return
        retrigger_band = max(1.2, round(deadband * 0.6, 1))
        if (self._last_commanded_angle_deg is not None and
                abs(desired - self._last_commanded_angle_deg) < retrigger_band):
            return
        max_dps = self.controller.info.max_speed_dps if self.controller.info is not None else 500
        speed_dps = min(max_dps, max(1, round(self.speed_spin.value())))
        speed_pct = round(speed_dps / max_dps * 100, 1)
        if self.controller.request_follow_move(speed_pct, desired, speed_dps=speed_dps):
            self._active_req = self.controller._next_req - 1
            self._pending_commanded_angle_deg = desired
            abs_target = self.controller.working_zero_mdeg / 1000 + sign * desired
            cur_abs = state.pos_mdeg / 1000
            self.command_label.setText(
                f"控制命令：[旧固件步进兼容] 转 {error_deg:.1f}°（{cur_abs:+.1f}° → {abs_target:+.1f}°，速度 {speed_dps}°/s）"
            )
            self.flow_label.setText(f"流程：兼容步进运动中 → 到 {abs_target:+.1f}°（重烧固件可消除停顿）")

    def _on_command_result(self, req: int, verb: str, result: str) -> None:
        if not self.running:
            return
        if req == self._mode_req and verb == "MODE":
            self._mode_req = None
            if result in ("BAD_FRAME", "UNSUPPORTED"):
                self._legacy_step_fallback = True
                self.controller.event.emit(
                    "视觉",
                    "固件兼容回退",
                    "当前单片机固件尚未包含 VISION_FOLLOW，已自动回退为步进跟随；请烧录 firmware_latest/FOC_DEMO2.hex 开启 20Hz 丝滑流式跟随",
                )
                if self.controller.mode == "MANUAL":
                    self._phase = "step_follow"
                    self.flow_label.setText("流程：检测到旧版单片机固件，已自动兼容切换至步进跟随")
                    self._try_dispatch_follow()
                elif self.controller.request_mode("MANUAL"):
                    self._phase = "enter_step_manual"
                    self._mode_req = self.controller._next_req - 1
                else:
                    self.stop_flow(send_stop=False)
                return
            if result in ("BUSY", "NOT_READY"):
                self.stop_flow(send_stop=False)
                self.flow_label.setText(f"流程：切换跟随模式被拒（{result}），请等机械臂空闲后重试")
                return
        if req != self._active_req:
            return
        self._active_req = None
        now = time.monotonic()
        if verb == "MOVE" and result in ("DONE", "SETTLED"):
            self._last_commanded_angle_deg = self._pending_commanded_angle_deg
            self._pending_commanded_angle_deg = None
            self._cooldown_until = now + (
                self.SETTLE_COOLDOWN_SECONDS if result == "SETTLED" else self.DONE_COOLDOWN_SECONDS
            )
            self.controller.query("STATE")
            self.flow_label.setText("流程：本步到位，等待HX8稳定门控后继续跟随")
            return
        if verb == "MOVE" and result in ("NOT_READY", "BUSY", "STOPPED", "HX8_OFFLINE"):
            self._pending_commanded_angle_deg = None
            self._cooldown_until = now + self.RETRY_COOLDOWN_SECONDS
            self.controller.query("STATE")
            self.flow_label.setText(f"流程：等待HX8稳定就绪（{result}），50ms后重试")
            return
        self._pending_commanded_angle_deg = None
        self.stop_flow(send_stop=False)
        self.flow_label.setText(f"流程：跟随中止，设备返回 {result}")

    def stop_flow(self, *, send_stop: bool = True) -> None:
        if not self.running:
            return
        was_streaming = self._phase in ("stream_follow", "enter_stream") or self.controller.mode == "VISION_FOLLOW"
        was_moving = was_streaming or self._active_req is not None or bool(self.controller._pending)
        self.feedback_timer.stop()
        self.running = False
        self._phase = "idle"
        self._active_req = None
        self._mode_req = None
        self._pending_commanded_angle_deg = None
        self._last_stream_angle_deg = None
        self._cooldown_until = 0.0
        self._hx8_offline_since = None
        self._sync_buttons()
        self.flow_label.setText("流程：已停止视觉跟随（保持当前位置，未触碰夹爪）")
        if send_stop and not self.controller.simulated:
            if was_moving:
                self.controller.request_stop()
            if self.controller.mode == "VISION_FOLLOW":
                self.controller.request_mode("MANUAL")
        self.controller.event.emit(
            "模拟" if self.controller.simulated else "视觉",
            "停止视觉跟随",
            "已停止自动跟随决策" + ("并保持当前位置" if send_stop and not self.controller.simulated and was_moving else ""),
        )
        self.controller.state_changed.emit()

    def simulation_disabled(self) -> None:
        self.stop_flow(send_stop=False)
        self.actual_label.setText("机械臂实际状态：未知／等待反馈")
        self.command_label.setText("控制命令：未发送")
        self.flow_label.setText("流程：待启动（仅预览映射角度，不发运动指令，全程不控制夹爪）")

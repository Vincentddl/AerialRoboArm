from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QFrame, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPlainTextEdit, QPushButton, QScrollArea, QSplitter, QStackedWidget, QVBoxLayout, QWidget,
)

from .models import ConnectionState
from .pages import ManualPage, VisionDemoPage, VisionFollowPage, VisionGrabPage
from .experiment_page import ExperimentPage
from .calibration_page import CalibrationPage
from .plots import AccelerationPanel
from .v1_protocol import AccFrame, CapturePageFrame, InvalidFrame, MotionFrame, OptionalImuFrame, StateFrame, V1Decoder
from .serial_service import SerialService, available_ports
from .simulation import SimulationService
from .state import ExperimentController
from .video_capture import VideoCaptureService, VideoDevice, VideoFrame, VideoStatus
from .black_blocks import BlackBlockAlignment, BlackBlockDetectionService
from .foam_vision import FoamVisionService
from .position_recorder import PositionSweepRecorderDialog


LOW_VOLTAGE_MV = 10_600
LOW_VOLTAGE_REARM_MV = 10_800
LOW_VOLTAGE_CONFIRM_S = 2.0
LOW_VOLTAGE_CONFIRM_SAMPLES = 3
LOW_VOLTAGE_MAX_SAMPLE_GAP_S = 2.0


class VoltageAlarmGate:
    """One warning per low-voltage episode, with recovery hysteresis."""

    def __init__(self) -> None:
        self.tripped = False
        self.low_since: float | None = None
        self.low_count = 0
        self.last_sample_at: float | None = None

    def reset(self) -> None:
        self.tripped = False
        self.low_since = None
        self.low_count = 0
        self.last_sample_at = None

    def observe(self, voltage_mv: int | None, *, valid: bool, received_at: float) -> bool:
        if not valid or voltage_mv is None or voltage_mv <= 0:
            self.low_since = None
            self.low_count = 0
            self.last_sample_at = None
            return False
        if voltage_mv >= LOW_VOLTAGE_REARM_MV:
            self.reset()
        elif voltage_mv >= LOW_VOLTAGE_MV:
            self.low_since = None
            self.low_count = 0
            self.last_sample_at = None
        elif not self.tripped:
            if (self.low_since is None or self.last_sample_at is None or
                    received_at - self.last_sample_at > LOW_VOLTAGE_MAX_SAMPLE_GAP_S):
                self.low_since = received_at
                self.low_count = 0
            self.low_count += 1
            self.last_sample_at = received_at
            if (self.low_count < LOW_VOLTAGE_CONFIRM_SAMPLES or
                    received_at - self.low_since < LOW_VOLTAGE_CONFIRM_S):
                return False
            self.tripped = True
            return True
        return False


class MainWindow(QMainWindow):
    def __init__(self, decoder: V1Decoder | None = None, video_service: VideoCaptureService | None = None):
        super().__init__()
        self._configure_font()
        self.setWindowTitle("机载机械臂实验上位机")
        self.resize(1440, 900)
        self.setMinimumSize(960, 640)
        self.serial = SerialService(self)
        self.decoder = decoder or V1Decoder()
        self.controller = ExperimentController(self.serial, Path(__file__).resolve().parent.parent / "runtime" / "pose_calibration.json")
        self.simulation = SimulationService(self)
        self.video = video_service or VideoCaptureService(self)
        self.black_detector = BlackBlockDetectionService(self)
        self.foam_detector = FoamVisionService(self)
        self.black_alignment = BlackBlockAlignment()
        self._last_v_tx_log_at = 0.0
        self._position_recorder: PositionSweepRecorderDialog | None = None
        self._active_video_device: VideoDevice | None = None
        self._voltage_alarm_gate = VoltageAlarmGate()
        self._voltage_alarm_dialog: QMessageBox | None = None
        self._last_voltage_state_ms: int | None = None
        self.received_total = 0
        self.last_received: float | None = None
        self.last_valid_acc: float | None = None
        self.discarded_imu_frames = 0
        self.raw_previews: deque[str] = deque(maxlen=60)
        self._raw_line = bytearray()
        self._raw_line_overlong = False

        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(12, 12, 12, 10)
        outer.setSpacing(8)
        outer.addLayout(self._build_top())
        self.vsplit = QSplitter(Qt.Orientation.Vertical)
        outer.addWidget(self.vsplit, 1)
        self.hsplit = QSplitter(Qt.Orientation.Horizontal)
        self.vsplit.addWidget(self.hsplit)
        self.hsplit.addWidget(self._build_sidebar())
        self.pages = QStackedWidget()
        self.manual_page = ManualPage(self.controller)
        self.demo_page = VisionDemoPage()
        self.grab_page = VisionGrabPage(self.controller)
        self.follow_page = VisionFollowPage(self.controller)
        self.experiment_page = ExperimentPage(self.controller, minimum_voltage_mv=LOW_VOLTAGE_MV)
        self.calibration_page = CalibrationPage(self.controller)
        self.manual_page.grab_flow = self.grab_page
        self.manual_scroll = QScrollArea()
        self.manual_scroll.setWidgetResizable(True)
        self.manual_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.manual_scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.manual_scroll.setWidget(self.manual_page)
        for page in (self.manual_scroll, self.demo_page, self.grab_page, self.follow_page,
                     self.experiment_page, self.calibration_page):
            self.pages.addWidget(page)
        self.mode_panel = QWidget()
        mode_layout = QVBoxLayout(self.mode_panel)
        mode_layout.setContentsMargins(0, 0, 0, 0)
        mode_layout.setSpacing(6)
        self.video_controls = self._build_video_controls()
        mode_layout.addWidget(self.video_controls)
        mode_layout.addWidget(self.pages, 1)
        self.hsplit.addWidget(self.mode_panel)
        self.plot_panel = AccelerationPanel()
        self.hsplit.addWidget(self.plot_panel)
        self.hsplit.setSizes([190, 790, 390])
        self.hsplit.setStretchFactor(1, 4)
        self.hsplit.setStretchFactor(2, 2)
        self.log_panel = self._build_log()
        self.vsplit.addWidget(self.log_panel)
        self.vsplit.setSizes([730, 130])

        self.serial.state_changed.connect(self._on_connection_state)
        self.serial.error.connect(lambda msg: self._log("串口", "错误", msg))
        self.serial.raw_received.connect(self._on_raw)
        self.serial.wire_written.connect(self._on_wire_written)
        self.controller.event.connect(self._log)
        self.controller.state_changed.connect(self._sync_shape)
        self.experiment_page.capture_changed.connect(self._sync_shape)
        self.controller.state_changed.connect(self._check_low_voltage)
        self.controller.device_restarted.connect(self._on_device_restarted)
        self.controller.motion_event.connect(lambda frame: self.experiment_page.on_motion(frame, time.monotonic()))
        self.simulation.acceleration.connect(self._on_simulated_sample)
        self.simulation.vision.frame_ready.connect(self._on_vision_frame)
        self.video.devices_changed.connect(self._on_video_devices)
        self.video.status_changed.connect(self._on_video_status)
        self.video.frame_ready.connect(self._on_real_video_frame)
        self.video.frame_cleared.connect(self._clear_real_video)
        self.video.event.connect(lambda message: self._log("视频", "采集", message))
        self.black_detector.result_ready.connect(self._on_black_blocks)
        self.black_detector.error.connect(lambda message: self._log("视觉", "识别", message))
        self.foam_detector.result_ready.connect(self._on_foam_observation)
        self.foam_detector.error.connect(lambda message: self._log("视觉", "YOLO识别", message))
        self.follow_page.exposure_lock_toggled.connect(self._on_follow_exposure_lock_toggled)
        self.follow_page.detector_mode_changed.connect(self._on_follow_detector_mode_changed)
        self.follow_page.record_lut_requested.connect(self._open_position_recorder)
        self.demo_page.parameters_changed.connect(self._on_demo_parameters_changed)
        self.port_button.clicked.connect(self.refresh_ports)
        self.connect_button.clicked.connect(self._toggle_connection)
        self.simulation_checkbox.toggled.connect(self._toggle_simulation)
        self.fold_button.clicked.connect(self.controller.request_fold)
        self.normal_button.clicked.connect(self.controller.request_restore)
        for index, button in enumerate(self.nav_buttons):
            button.clicked.connect(lambda checked=False, i=index: self.switch_page(i, request_mode=(i in (0, 4, 5))))
        self.log_toggle.clicked.connect(self._toggle_log)
        self.video_refresh.clicked.connect(self.video.refresh_devices)
        self.video_open.clicked.connect(self._open_video)
        self.video_close.clicked.connect(self.video.close_device)
        self.video_select.currentIndexChanged.connect(lambda: self._on_video_status(self.video.status))
        self.monitor_timer = QTimer(self)
        self.monitor_timer.setInterval(500)
        self.monitor_timer.timeout.connect(self._update_reception)
        self.monitor_timer.start()
        self.refresh_ports()
        self.video.refresh_devices()
        self._on_video_status(self.video.status)
        self._on_connection_state(ConnectionState.DISCONNECTED)
        self._sync_shape()
        self.switch_page(0)
        self._apply_style()

    def _build_top(self):
        row = QHBoxLayout()
        row.addWidget(QLabel("433 MHz 串口"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(210)
        row.addWidget(self.port_combo)
        self.port_button = QPushButton("刷新")
        row.addWidget(self.port_button)
        row.addWidget(QLabel("波特率"))
        self.baud_combo = QComboBox()
        for baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600):
            self.baud_combo.addItem(str(baud), baud)
        self.baud_combo.setCurrentText("115200")
        row.addWidget(self.baud_combo)
        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("primaryButton")
        row.addWidget(self.connect_button)
        self.connection_label = QLabel("未连接")
        row.addWidget(self.connection_label)
        row.addStretch()
        self.simulation_checkbox = QCheckBox("模拟演示")
        row.addWidget(self.simulation_checkbox)
        self.mode_badge = QLabel("真实模式")
        row.addWidget(self.mode_badge)
        return row

    def _build_sidebar(self):
        panel = QFrame()
        panel.setObjectName("sidebar")
        panel.setMinimumWidth(180)
        root = QVBoxLayout(panel)
        title = QLabel("全局形态")
        title.setObjectName("sectionTitle")
        root.addWidget(title)
        self.fold_button = QPushButton("折叠位置")
        self.fold_button.setToolTip(
            "MANUAL 模式下先张开夹爪，再移动到已保存的折叠位，"
            "速度为 INFO 最大速度的 40%；该目标需使用同步更新的 HC13 实验固件")
        self.normal_button = QPushButton("正常模式")
        self.normal_button.setToolTip(
            "MANUAL 模式下控制 HX8 以 INFO 最大速度的 60% 到已保存的正常位。"
            "到位后张开夹爪；物理标定零点仍为0°")
        root.addWidget(self.fold_button)
        root.addWidget(self.normal_button)
        self.shape_label = QLabel("实际形态：未知")
        self.shape_label.setWordWrap(True)
        self.shape_request_label = QLabel("最近请求：无")
        self.shape_request_label.setWordWrap(True)
        root.addWidget(self.shape_label)
        root.addWidget(self.shape_request_label)
        self.device_mode_label = QLabel("设备工作模式：未知")
        self.device_mode_label.setWordWrap(True)
        root.addWidget(self.device_mode_label)
        root.addSpacing(16)
        mode_label = QLabel("工作模式")
        mode_label.setObjectName("sectionTitle")
        root.addWidget(mode_label)
        self.nav_buttons = [
            QPushButton(text)
            for text in ("手动模式", "视觉演示模式", "视觉抓取模式", "视觉跟随模式", "实验模式", "校准模式")
        ]
        for button in self.nav_buttons:
            button.setCheckable(True)
            root.addWidget(button)
        root.addStretch()
        self.reception_label = QLabel("最近数据：尚未收到")
        self.reception_label.setWordWrap(True)
        root.addWidget(self.reception_label)
        root.addWidget(QLabel("串口打开 ≠ 机械臂在线"))
        return panel

    def _build_video_controls(self):
        panel = QFrame()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 4, 6, 4)
        row = QHBoxLayout()
        row.addWidget(QLabel("图传设备"))
        self.video_select = QComboBox()
        self.video_select.addItem("请选择视频设备", None)
        self.video_select.setMinimumWidth(160)
        row.addWidget(self.video_select, 1)
        self.video_refresh = QPushButton("刷新")
        self.video_open = QPushButton("打开视频")
        self.video_close = QPushButton("关闭视频")
        row.addWidget(self.video_refresh)
        row.addWidget(self.video_open)
        row.addWidget(self.video_close)
        layout.addLayout(row)
        self.video_status_label = QLabel("视频源：未打开")
        self.video_status_label.setWordWrap(True)
        layout.addWidget(self.video_status_label)
        return panel

    def _build_log(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 2, 0, 0)
        head = QHBoxLayout()
        head.addWidget(QLabel("操作日志与原始接收摘要"))
        head.addStretch()
        self.log_toggle = QPushButton("折叠")
        head.addWidget(self.log_toggle)
        layout.addLayout(head)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(500)
        layout.addWidget(self.log_view)
        return panel

    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow, QWidget { background: #eef3f8; color: #18314a; font-family: 'Noto Sans SC'; font-size: 13px; }
            QFrame#sidebar { background: #e3ebf4; border-radius: 8px; }
            QLabel#sectionTitle { font-size: 16px; font-weight: bold; }
            QLabel#pageTitle { font-size: 22px; font-weight: bold; padding: 5px 0; }
            QPushButton { background: #ffffff; border: 1px solid #b7c8d8; border-radius: 5px; padding: 7px 10px; }
            QPushButton:hover { background: #dceaf5; }
            QPushButton:checked, QPushButton#primaryButton { background: #176b9b; color: white; border-color: #176b9b; }
            QPushButton:disabled { color: #8795a4; background: #e5e9ed; }
            QComboBox, QDoubleSpinBox { background: #fff; border: 1px solid #b7c8d8; border-radius: 4px; padding: 4px; }
            QPlainTextEdit { background: #142336; color: #d4e1ee; border: 0; border-radius: 5px; }
        """)

    @staticmethod
    def _configure_font() -> None:
        # Offscreen/独立 Qt 运行时有时未列出 Windows 字体，显式载入中文字体。
        candidates = ("Noto Sans SC.ttf", "msyh.ttc", "simhei.ttf")
        for name in candidates:
            path = Path("C:/Windows/Fonts") / name
            if path.exists():
                font_id = QFontDatabase.addApplicationFont(str(path))
                families = QFontDatabase.applicationFontFamilies(font_id) if font_id >= 0 else []
                if families:
                    QApplication.instance().setFont(QFont(families[0], 10))
                    return

    def refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = available_ports()
        for device, description in ports:
            self.port_combo.addItem(f"{device} — {description}", device)
        if not ports:
            self.port_combo.addItem("无可用 COM 口", None)
        elif current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        if self.serial.state == ConnectionState.OPEN:
            if not self.controller.voltage_locked:
                self.serial.disconnect_port()
            return
        if self.serial.state in (ConnectionState.CONNECTING, ConnectionState.DISCONNECTING):
            return
        port = self.port_combo.currentData()
        if port is None:
            self._log("串口", "连接", "无可用 COM 口，请接入设备并刷新")
            self.connection_label.setText("连接失败：无可用 COM 口")
            return
        self.decoder.reset()
        self._raw_line.clear()
        self._raw_line_overlong = False
        self.serial.connect_port(port, self.baud_combo.currentData())

    def _on_connection_state(self, state: ConnectionState) -> None:
        if state != ConnectionState.OPEN:
            self._voltage_alarm_gate.observe(None, valid=False, received_at=time.monotonic())
            self._last_voltage_state_ms = None
            self.follow_page.reset_firmware_capability_cache()
        self._sync_connection_controls()
        if state == ConnectionState.OPEN:
            self._log("串口", "打开", "本机串口可用；等待设备数据，未确认机械臂在线")
        elif state == ConnectionState.DISCONNECTED:
            self._log("串口", "关闭", "串口已释放")

    def _sync_connection_controls(self) -> None:
        state = self.serial.state
        voltage_locked = self.controller.voltage_locked
        self.connection_label.setText(state.value + (" · 低压锁定，等待 ≥10.80 V" if voltage_locked else ""))
        self.simulation_checkbox.setEnabled(not voltage_locked)
        port_busy = state in (ConnectionState.CONNECTING, ConnectionState.OPEN, ConnectionState.DISCONNECTING)
        self.port_combo.setEnabled(not port_busy)
        self.port_button.setEnabled(not port_busy)
        self.baud_combo.setEnabled(not port_busy)
        self.connect_button.setEnabled(state not in (ConnectionState.CONNECTING, ConnectionState.DISCONNECTING)
                                       and not (voltage_locked and state == ConnectionState.OPEN))
        self.connect_button.setText("断开" if state == ConnectionState.OPEN else "连接")

    def _on_raw(self, chunk: bytes, received_at: float) -> None:
        self.received_total += len(chunk)
        self.last_received = received_at
        self.raw_previews.append(f"{len(chunk)} B · {chunk[:24].hex(' ').upper()}{' …' if len(chunk) > 24 else ''}")
        if self.controller.simulated:
            return  # 模拟画面与曲线绝不混入真实串口数据
        self._log_control_rx_lines(chunk, received_at)
        try:
            samples = self.decoder.feed(chunk, received_at)
        except Exception as exc:
            self._log("协议", "解析异常", str(exc))
            self.decoder.reset()
            return
        for frame in samples:
            if isinstance(frame, AccFrame):
                if self.controller.mode == "VISION_PICK":
                    self.discarded_imu_frames += 1
                    continue  # 保持同一串口接收控制回执；视觉抓取时不消费残余IMU帧
                self.plot_panel.add_sample(frame.sample)
                self.experiment_page.on_acc(frame, received_at)
                self.last_valid_acc = received_at
                self.controller.last_acc_received = received_at
            elif isinstance(frame, InvalidFrame):
                self._log("协议", "丢弃无效帧", frame.reason)
            elif isinstance(frame, CapturePageFrame):
                self.experiment_page.on_capture_page(frame, received_at)
            elif isinstance(frame, OptionalImuFrame):
                if self.controller.mode == "VISION_PICK":
                    self.discarded_imu_frames += 1
                else:
                    self.experiment_page.on_trace_frame(frame, received_at)
                # 可选GYRO/ATT不在本轮三轴加速度图内
            else:
                if isinstance(frame, (MotionFrame, StateFrame)):
                    self.experiment_page.on_trace_frame(frame, received_at)
                self.controller.handle_frame(frame)

    def _on_wire_written(self, payload: bytes, success: bool, detail: str, written_at: float) -> None:
        self.experiment_page.on_wire_written(payload, success, written_at)
        parts = payload.rstrip(b"\r\n").split(b",")
        if success and len(parts) == 5 and parts[0] == b"GET" and parts[2] == b"CAPTURE":
            return
        verb = parts[0].decode("ascii", "replace") if parts else "?"
        if verb == "V" and success:
            if written_at - self._last_v_tx_log_at < 1.0:
                return
            self._last_v_tx_log_at = written_at
        req_id = (parts[1].decode("ascii", "replace") if len(parts) > 1 and verb != "V" else "无")
        self._log("串口TX", f"{verb} req={req_id}",
                  f"{'实际写出' if success else '写入失败'} · 单调时钟 {written_at:.6f}s · "
                  f"{detail} · bytes={payload!r} · HEX={payload.hex(' ').upper()}")

    def _log_control_rx_lines(self, chunk: bytes, received_at: float) -> None:
        """只逐帧记录控制回执；ACC 高频原始流仍见有界摘要。"""
        for byte in chunk:
            if byte == 10:
                if not self._raw_line_overlong:
                    raw = bytes(self._raw_line) + b"\n"
                    line = raw.rstrip(b"\r\n")
                    if line and not line.startswith((b"ACC,", b"GYRO,", b"ATT,", b"CAPDATA,")):
                        self._log("串口RX", "原始帧",
                                  f"单调时钟 {received_at:.6f}s · bytes={raw!r} · HEX={raw.hex(' ').upper()}")
                        if line == b"ERROR":
                            self.controller.note_unexpected_wire_response(line)
                self._raw_line.clear()
                self._raw_line_overlong = False
            elif not self._raw_line_overlong:
                if len(self._raw_line) < 256:
                    self._raw_line.append(byte)
                else:
                    self._raw_line.clear()
                    self._raw_line_overlong = True

    def _on_device_restarted(self) -> None:
        self.decoder.reset()
        self.plot_panel.clear()
        self.last_valid_acc = None
        self.follow_page.reset_firmware_capability_cache()
        self._log("协议", "新会话", "设备时间轴已重置，旧加速度曲线已清空")

    def _update_reception(self) -> None:
        now = time.monotonic()
        raw_age = "尚未收到" if self.last_received is None else f"{now - self.last_received:.1f} 秒前"
        acc_age = "尚未收到" if self.last_valid_acc is None else f"{now - self.last_valid_acc:.1f} 秒前"
        imu_paused = not self.controller.simulated and self.controller.mode == "VISION_PICK"
        state = self.controller.latest_state
        stale_acc = self.last_valid_acc is not None and now - self.last_valid_acc > 3
        if imu_paused:
            imu_note = "视觉抓取期间暂停"
        elif self.controller.simulated:
            imu_note = "模拟数据"
        elif not self.controller.link_online or state is None:
            imu_note = "等待设备在线"
        elif not state.imu_online:
            imu_note = "板端 IMU 断流"
        elif state.imu_hz < 180 or state.imu_hz > 220:
            imu_note = f"板端 IMU 采样率异常（{state.imu_hz} Hz）"
        elif self.last_valid_acc is None:
            imu_note = "等待无线 ACC"
        elif stale_acc:
            imu_note = "无线 ACC 已过期"
        else:
            imu_note = "正常接收"
        imu_note = f"IMU无线流：{imu_note}"
        self.reception_label.setText(f"原始接收：{raw_age}\n有效ACC：{acc_age}\n{imu_note}\n累计 {self.received_total} 字节")
        if imu_paused:
            acc_tx = self.controller.latest_state.acc_tx if self.controller.latest_state is not None else None
            state_note = "设备STATE仍报ACC开启；本机已丢弃残余IMU帧" if acc_tx else "设备无线ACC已关闭" if acc_tx is False else "等待STATE确认无线ACC状态"
            self.plot_panel.status.setText(f"视觉抓取模式：IMU显示暂停 · {state_note}")
        elif self.controller.link_online and state is not None and not state.imu_online:
            self.plot_panel.status.setText("板端 IMU 无有效采样 · 固件空闲时尝试重连；请检查传感器供电和串口接线")
        elif self.controller.link_online and state is not None and (state.imu_hz < 180 or state.imu_hz > 220):
            self.plot_panel.status.setText(f"板端 IMU 采样率 {state.imu_hz} Hz · 等待恢复至 200 Hz")
        elif stale_acc and not self.controller.simulated:
            self.plot_panel.status.setText("加速度数据已过期 · 保留历史曲线")
        elif self.plot_panel.status.text().startswith(("视觉抓取模式：", "板端 IMU ")):
            self.plot_panel.status.setText("等待加速度数据恢复 · 保留历史曲线")
        if self.raw_previews:
            lines = list(self.raw_previews)
            self.raw_previews.clear()
            self._log("原始接收", "数据摘要", f"本轮 {len(lines)} 包；末包 {lines[-1]}")
        if self.discarded_imu_frames:
            self._log("协议", "视觉抓取IMU隔离", f"丢弃 {self.discarded_imu_frames} 个残余IMU帧；控制回执和STATE继续接收")
            self.discarded_imu_frames = 0

    def _toggle_simulation(self, enabled: bool) -> None:
        if self.controller.voltage_locked:
            return
        self._reset_voltage_alarm()
        self.black_detector.clear()
        self.foam_detector.clear()
        self.black_alignment.reset()
        self.grab_page.simulation_disabled()
        self.follow_page.simulation_disabled()
        self.controller.set_simulation(enabled)
        if enabled:
            self.video.close_device()
        self._on_video_status(self.video.status)
        self.plot_panel.clear()
        if enabled:
            self.mode_badge.setText("● 模拟演示 · 不发真实指令")
            self.mode_badge.setStyleSheet("color: #a45000; font-weight: bold")
            self.simulation.start()
        else:
            self.simulation.stop()
            self.mode_badge.setText("真实模式")
            self.mode_badge.setStyleSheet("")
            self.manual_page.video_canvas.clear()
            self.demo_page.clear_source()
            self.grab_page.clear_source()
            self.follow_page.clear_source()
            self.decoder.reset()

    def _on_simulated_sample(self, sample) -> None:
        if self.controller.simulated:
            self.plot_panel.add_sample(sample)

    def _on_vision_frame(self, image, result) -> None:
        if self.controller.simulated:
            from .models import VisionResult
            if self.pages.currentIndex() == 0:
                self.manual_page.video_canvas.set_frame(
                    image, VisionResult(result.width, result.height, None, result.source, True,
                                        center_segments_overlay=True))
            self.demo_page.set_frame(image, result)
            self.grab_page.set_frame(image, result)
            self.follow_page.set_frame(image, result)
            self.follow_page.observe(result)

    def _on_video_devices(self, devices: list[VideoDevice]) -> None:
        old = self.video_select.currentData()
        self.video_select.clear()
        self.video_select.addItem("请选择视频设备", None)
        for device in devices:
            self.video_select.addItem(device.label, device)
            self.video_select.setItemData(self.video_select.count() - 1, device.path or device.label,
                                          Qt.ItemDataRole.ToolTipRole)
        if old in devices:
            self.video_select.setCurrentIndex(devices.index(old) + 1)
        else:
            usb_video = [i for i, device in enumerate(devices, 1) if device.name.casefold() == "usb video"]
            if len(usb_video) == 1:
                self.video_select.setCurrentIndex(usb_video[0])
        self._on_video_status(self.video.status)

    def _open_video(self) -> None:
        device = self.video_select.currentData()
        if device is None:
            self._log("视频", "打开", "请先选择 USB Video 等明确的视频设备")
            self.video_status_label.setText("视频源：未选择设备；不会默认打开编号 0")
            return
        if self.video.open_device(device):
            self._active_video_device = device
        else:
            self._log("视频", "打开", "设备正在扫描、采集或关闭，请等待")

    def _on_video_status(self, status: VideoStatus) -> None:
        if self.controller.simulated:
            self.video_status_label.setText("视频源：模拟演示；真实采集已停用")
        elif status.phase == "采集中":
            self.video_status_label.setText(
                f"视频源：{status.device_name} · {status.phase} · 实际 {status.width}×{status.height} · "
                f"读取 {status.read_fps:.1f} fps · 电脑端接收时间（非曝光时间）")
        else:
            self.video_status_label.setText(f"视频源：{status.phase} · {status.detail}")
        busy = status.phase in ("扫描中", "打开中", "采集中", "无有效画面", "关闭中")
        self.video_select.setEnabled(not busy and not self.controller.simulated)
        self.video_refresh.setEnabled(not busy and not self.controller.simulated)
        self.video_open.setEnabled(not busy and not self.controller.simulated and self.video_select.currentData() is not None)
        self.video_close.setEnabled(status.phase in ("打开中", "采集中", "无有效画面") and not self.controller.simulated)

    def _on_real_video_frame(self, frame: VideoFrame) -> None:
        if self.controller.simulated:
            return
        from .models import VisionResult
        result = VisionResult(frame.width, frame.height, None, self.video.status.device_name, False,
                              center_segments_overlay=True)
        if self.pages.currentIndex() == 0:
            self.manual_page.video_canvas.set_frame(frame.image, result)
        elif self.pages.currentIndex() in (1, 2):
            self.black_detector.submit(frame, self.demo_page.detector_config())
        elif self.pages.currentIndex() == 3:
            if self.follow_page.detector_mode == "yolo_v9":
                self.foam_detector.submit(frame, self.follow_page.foam_vision_config())
            else:
                self.black_detector.submit(frame, self.follow_page.current_black_detector_config(self.demo_page.detector_config()))

    def _on_follow_exposure_lock_toggled(self, enabled: bool) -> None:
        if hasattr(self.video, "set_exposure_lock"):
            self.video.set_exposure_lock(enabled)

    def _on_follow_detector_mode_changed(self, _mode: str) -> None:
        self.black_detector.clear()
        self.foam_detector.clear()
        if not self.controller.simulated and self.pages.currentIndex() == 3:
            self._show_latest_video()

    def _open_position_recorder(self) -> None:
        if self.follow_page.running:
            self.follow_page.stop_flow(send_stop=True)
        if self._position_recorder is not None:
            self._position_recorder.show()
            self._position_recorder.raise_()
            self._position_recorder.activateWindow()
            return
        runtime_dir = Path(__file__).resolve().parent.parent / "runtime"
        dialog = PositionSweepRecorderDialog(
            self.controller,
            self.video,
            lambda: self.follow_page.current_black_detector_config(self.demo_page.detector_config()),
            runtime_dir,
            self,
        )
        dialog.lut_saved.connect(self.follow_page.apply_calibrated_lut)
        dialog.finished.connect(lambda _code: setattr(self, "_position_recorder", None))
        self._position_recorder = dialog
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

    def _on_foam_observation(self, frame: VideoFrame, observation) -> None:
        if (self.controller.simulated or self.pages.currentIndex() != 3 or
                self.follow_page.detector_mode != "yolo_v9" or
                self.video.status.phase != "采集中" or
                time.monotonic() - frame.received_monotonic_s > 1.0):
            return
        self.follow_page.set_yolo_observation(
            frame.image,
            frame.frame_id,
            frame.width,
            frame.height,
            self.video.status.device_name,
            observation,
        )
        if self.follow_page.result is not None:
            self.follow_page.observe(self.follow_page.result)

    def _on_demo_parameters_changed(self) -> None:
        # Retire in-flight results computed with the old immutable config.
        self.black_detector.clear()
        self.black_alignment.reset()
        if self.controller.simulated or self.pages.currentIndex() not in (1, 3):
            return
        frame = self.video.latest_frame
        if self.video.status.phase == "采集中" and frame is not None:
            if self.pages.currentIndex() == 1:
                self.demo_page.show_pending_frame(frame.image, self.video.status.device_name, frame.frame_id)
                self.black_detector.submit(frame, self.demo_page.detector_config())
            else:
                if self.follow_page.detector_mode == "yolo_v9":
                    self.foam_detector.submit(frame, self.follow_page.foam_vision_config())
                else:
                    self.black_detector.submit(frame, self.follow_page.current_black_detector_config(self.demo_page.detector_config()))

    def _on_black_blocks(self, frame: VideoFrame, observation) -> None:
        if (self.controller.simulated or self.pages.currentIndex() not in (1, 2, 3) or
                (self.pages.currentIndex() == 3 and self.follow_page.detector_mode != "opencv_black") or
                self.video.status.phase != "采集中" or
                time.monotonic() - frame.received_monotonic_s > 1.0):
            return
        from .models import VisionResult
        blocks = observation.blocks
        aligned = self.black_alignment.update(
            blocks[0] if blocks else None, frame.width,
            self.demo_page.alignment_tolerance.value())
        result = VisionResult(frame.width, frame.height, None,
                              self.video.status.device_name, False,
                              blocks, frame.frame_id, True, aligned,
                              observation.phase, observation.missed_frames,
                              observation.miss_limit)
        if self.pages.currentIndex() == 1:
            self.demo_page.set_frame(frame.image, result)
        elif self.pages.currentIndex() == 2:
            self.grab_page.set_frame(frame.image, result)
            self.grab_page.observe(result)
        elif self.pages.currentIndex() == 3:
            self.follow_page.set_frame(frame.image, result)
            self.follow_page.observe(result)

    def _clear_real_video(self) -> None:
        if self.grab_page.running and self.grab_page._phase in ("mode", "watch"):
            self.grab_page.stop_flow(send_stop=True)
        elif self.grab_page.running:
            self._log("视觉", "视频中断", "抓取已触发；停止新的识别，已提交的闭爪和回程继续依据设备回执")
        if self.follow_page.running:
            self.follow_page.stop_flow(send_stop=True)
        self.black_detector.clear()
        self.foam_detector.clear()
        self.black_alignment.reset()
        self._active_video_device = None
        if not self.controller.simulated:
            self.manual_page.video_canvas.clear()
            self.demo_page.clear_source()
            self.grab_page.clear_source()
            self.follow_page.clear_source()

    def _show_latest_video(self) -> None:
        if self.video.status.phase == "采集中" and self.video.latest_frame is not None:
            self._on_real_video_frame(self.video.latest_frame)

    def _sync_shape(self) -> None:
        self.shape_label.setText(f"实际形态：{self.controller.actual_state}")
        self.shape_request_label.setText(f"最近请求：{self.controller.requested_shape}\n{self.controller.command_status}")
        self.device_mode_label.setText(f"设备工作模式：{self.controller.mode} · {'在线' if self.controller.link_online else '未确认在线'}")
        capture = self.experiment_page.capturing_trace
        vision_active = self.grab_page.running or self.follow_page.running
        self.fold_button.setEnabled(not capture and not self.controller.control_busy and not vision_active)
        self.normal_button.setEnabled(not capture and not self.controller.control_busy and not vision_active)
        for button in self.nav_buttons:
            button.setEnabled(not capture)
        self.vsplit.setEnabled(not self.controller.voltage_locked)

    def _reset_voltage_alarm(self) -> None:
        self._voltage_alarm_gate.reset()
        self._last_voltage_state_ms = None
        if self._voltage_alarm_dialog is not None:
            self._voltage_alarm_dialog.close()
            self._voltage_alarm_dialog = None

    def _check_low_voltage(self) -> None:
        state = self.controller.latest_state
        received_at = self.controller._last_state_received_at
        valid = (not self.controller.simulated and self.serial.state == ConnectionState.OPEN
                 and self.controller.link_online and state is not None and state.hx8_online
                 and received_at is not None and time.monotonic() - received_at <= 2.0)
        if not valid:
            self._voltage_alarm_gate.observe(None, valid=False, received_at=time.monotonic())
            return
        if state.t_ms == self._last_voltage_state_ms:
            return
        self._last_voltage_state_ms = state.t_ms
        tripped = self._voltage_alarm_gate.observe(state.voltage_mv, valid=True, received_at=received_at)
        if self.controller.voltage_locked and not self._voltage_alarm_gate.tripped:
            self.controller.set_voltage_lock(False)
            self._sync_connection_controls()
            if self._voltage_alarm_dialog is not None:
                self._voltage_alarm_dialog.close()
                self._voltage_alarm_dialog = None
            self._log("电压", "供电恢复", f"HX8 已回报 {state.voltage_mv/1000:.2f} V，控制已解锁；暂停的批次可手动继续")
        if not tripped:
            return
        voltage = state.voltage_mv / 1000
        self.controller.set_voltage_lock(True)
        self._sync_connection_controls()
        self.grab_page.stop_flow(send_stop=False)
        self.follow_page.stop_flow(send_stop=False)
        self.experiment_page.pause_for_low_voltage()
        self._log("电压", "低压报警", f"HX8回报供电连续低于 {LOW_VOLTAGE_MV / 1000:.2f} V，当前 {voltage:.2f} V；请检查或更换电池，防止过放")
        if self._voltage_alarm_dialog is not None:
            self._voltage_alarm_dialog.close()
        dialog = QMessageBox(self)
        dialog.setIcon(QMessageBox.Icon.Warning)
        dialog.setWindowTitle("供电电压过低")
        dialog.setText(f"HX8 回报供电电压连续低于 {LOW_VOLTAGE_MV / 1000:.2f} V，当前 {voltage:.2f} V。")
        dialog.setInformativeText(f"控制已锁定，自动采集已暂停。请检查或更换电池。收到 HX8 新鲜反馈 ≥ "
                                  f"{LOW_VOLTAGE_REARM_MV/1000:.2f} V 后解锁，暂停的批次可手动继续。")
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.setModal(False)
        self._voltage_alarm_dialog = dialog
        dialog.show()

    def switch_page(self, index: int, *, request_mode: bool = False) -> None:
        if self.controller.voltage_locked:
            return
        if self.pages.currentIndex() == 0 and index != 0:
            self.controller.cancel_grip_ramp("离开手动页")
            if self.grab_page.running:
                self.grab_page.stop_flow(send_stop=True)
        if self.pages.currentIndex() == 1 and index != 1 and not self.controller.simulated:
            self.black_detector.clear()
            self.black_alignment.reset()
            self.demo_page.clear_source()
        if self.pages.currentIndex() == 2 and index != 2:
            self.grab_page.stop_flow(send_stop=True)
        if self.pages.currentIndex() == 3 and index != 3:
            self.follow_page.stop_flow(send_stop=True)
            if not self.controller.simulated:
                self.black_detector.clear()
                self.foam_detector.clear()
                self.black_alignment.reset()
                self.follow_page.clear_source()
        self.pages.setCurrentIndex(index)
        if index == 3 and hasattr(self.video, "set_exposure_lock") and self.follow_page.exposure_lock_checkbox.isChecked():
            self.video.set_exposure_lock(True)
        self.video_controls.setVisible(index in (0, 1, 2, 3))
        if not self.controller.simulated:
            self._show_latest_video()
        for i, button in enumerate(self.nav_buttons):
            button.setChecked(i == index)
        if request_mode:
            self.controller.request_mode(("MANUAL", "VISION_DEMO", "VISION_PICK", "MANUAL", "MANUAL", "MANUAL")[index])

    def _toggle_log(self) -> None:
        show = not self.log_view.isVisible()
        self.log_view.setVisible(show)
        self.log_toggle.setText("折叠" if show else "展开")

    def _log(self, source: str, event: str, result: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{timestamp}] [{source}] {event}：{result}")

    def closeEvent(self, event) -> None:
        self._reset_voltage_alarm()
        if self._position_recorder is not None:
            self._position_recorder.close()
            self._position_recorder = None
        self.experiment_page.close_capture()
        self.monitor_timer.stop()
        self.simulation.stop()
        self.grab_page.stop_flow()
        self.follow_page.stop_flow(send_stop=False)
        self.plot_panel.timer.stop()
        self.controller.shutdown()
        self.black_detector.shutdown()
        self.foam_detector.shutdown()
        if not self.video.shutdown():
            self._log("视频", "关闭", "采集驱动未在2秒内返回；进程退出时由系统释放设备")
        self.serial.shutdown()
        super().closeEvent(event)

"""Interactive 17-point (30°-70° outbound+return) position-table recorder and LUT fitter."""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import statistics
import time

import cv2
import numpy as np
from PySide6.QtCore import QPointF, QRectF, QTimer, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QKeyEvent, QPainter, QPen
from PySide6.QtWidgets import (
    QDialog, QDoubleSpinBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget,
)

from .black_blocks import BlackBlockConfig, detect_black_blocks, qimage_gray
from .foam_vision import qimage_bgr
from .v1_state import ExperimentController
from .video_capture import VideoCaptureService, VideoFrame

DEFAULT_SWEEP_ANGLES: tuple[float, ...] = (
    30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0,
    65.0, 60.0, 55.0, 50.0, 45.0, 40.0, 35.0, 30.0,
)
DEFAULT_HOLD_SECONDS = 3.0


@dataclass
class HoldRecord:
    step_index: int
    direction: str  # "outbound" or "return"
    guide_extension_deg: float
    start_frame: int
    end_frame: int
    started_at_s: float
    ended_at_s: float
    center_y_samples: list[float]
    hx8_pos_mdeg_samples: list[int]
    hx8_extension_deg_samples: list[float]
    frame_height: int

    def summary(self) -> dict:
        y_med = round(statistics.median(self.center_y_samples), 3) if self.center_y_samples else None
        pos_med = round(statistics.median(self.hx8_pos_mdeg_samples), 1) if self.hx8_pos_mdeg_samples else None
        ext_med = round(statistics.median(self.hx8_extension_deg_samples), 3) if self.hx8_extension_deg_samples else None
        return {
            "step_index": self.step_index,
            "direction": self.direction,
            "guide_extension_deg": self.guide_extension_deg,
            "start_frame": self.start_frame,
            "end_frame": self.end_frame,
            "frame_count": max(0, self.end_frame - self.start_frame + 1),
            "detected_count": len(self.center_y_samples),
            "center_y_median_px": y_med,
            "hx8_pos_mdeg_median": pos_med,
            "hx8_extension_deg_median": ext_med,
            "frame_height": self.frame_height,
        }


class CalibratedPositionLut:
    """Piecewise-linear pixel_y -> relative extension angle LUT fitted from current-rig sweep."""

    def __init__(self, data: dict):
        self.data = data
        self.name = str(data.get("name", "calibrated_position_lut"))
        self.frame_height = int(data.get("frame_height", 480))
        pts = data.get("points") or []
        if len(pts) < 2:
            raise ValueError("位置关系表至少需要 2 个有效标定点")
        sorted_pts = sorted(
            ((float(p["center_y_px"]), float(p["extension_deg"])) for p in pts),
            key=lambda item: item[0],
        )
        self.y_nodes = [item[0] for item in sorted_pts]
        self.ext_nodes = [item[1] for item in sorted_pts]

    @classmethod
    def from_file(cls, path: Path) -> "CalibratedPositionLut":
        return cls(json.loads(path.read_text(encoding="utf-8")))

    def pixel_y_to_extension_deg(
        self,
        center_y: float,
        height: int,
        *,
        min_angle_deg: float = 0.0,
        max_angle_deg: float = 70.0,
        invert: bool = False,
    ) -> float:
        scale_y = self.frame_height / max(1, int(height))
        y = float(center_y) * scale_y
        ys = self.y_nodes
        es = self.ext_nodes
        if y <= ys[0]:
            slope = (es[1] - es[0]) / (ys[1] - ys[0]) if ys[1] != ys[0] else 0.0
            ext = es[0] + slope * (y - ys[0])
        elif y >= ys[-1]:
            slope = (es[-1] - es[-2]) / (ys[-1] - ys[-2]) if ys[-1] != ys[-2] else 0.0
            ext = es[-1] + slope * (y - ys[-1])
        else:
            ext = es[-1]
            for idx in range(len(ys) - 1):
                y0, y1 = ys[idx], ys[idx + 1]
                if y0 <= y <= y1:
                    ratio = (y - y0) / (y1 - y0) if y1 != y0 else 0.0
                    ext = es[idx] + ratio * (es[idx + 1] - es[idx])
                    break
        low = min(float(min_angle_deg), float(max_angle_deg))
        high = max(float(min_angle_deg), float(max_angle_deg))
        if invert:
            ext = (low + high) - ext
        return max(low, min(high, ext))


def build_position_lut_from_holds(
    holds: list[HoldRecord],
    *,
    frame_height: int,
    working_zero_mdeg: int,
    extension_sign: int,
    video_path: str = "",
    events_path: str = "",
) -> dict:
    grouped: dict[float, list[HoldRecord]] = {}
    for hold in holds:
        if hold.center_y_samples:
            grouped.setdefault(float(hold.guide_extension_deg), []).append(hold)
    if len(grouped) < 2:
        raise ValueError("检测到有效黑块的角度节点不足 2 个，无法生成位置关系表")

    points: list[dict] = []
    y_hyst_values: list[float] = []
    enc_hyst_values: list[float] = []
    for guide_deg in sorted(grouped):
        group = grouped[guide_deg]
        outbound = [h for h in group if h.direction == "outbound"]
        ret = [h for h in group if h.direction == "return"]
        all_y = [y for h in group for y in h.center_y_samples]
        all_ext = [e for h in group for e in h.hx8_extension_deg_samples]
        y_med = round(statistics.median(all_y), 3)
        ext_med = round(statistics.median(all_ext), 3) if all_ext else round(guide_deg, 3)

        y_hyst = None
        if outbound and ret:
            y_out = statistics.median(outbound[-1].center_y_samples)
            y_ret = statistics.median(ret[-1].center_y_samples)
            y_hyst = round(abs(y_out - y_ret), 3)
            y_hyst_values.append(y_hyst)

        enc_hyst = None
        if outbound and ret and outbound[-1].hx8_extension_deg_samples and ret[-1].hx8_extension_deg_samples:
            e_out = statistics.median(outbound[-1].hx8_extension_deg_samples)
            e_ret = statistics.median(ret[-1].hx8_extension_deg_samples)
            enc_hyst = round(abs(e_out - e_ret), 3)
            enc_hyst_values.append(enc_hyst)

        points.append({
            "guide_extension_deg": round(guide_deg, 1),
            "extension_deg": ext_med,
            "center_y_px": y_med,
            "hold_count": len(group),
            "sample_count": len(all_y),
            "hysteresis_y_px": y_hyst,
            "hysteresis_encoder_deg": enc_hyst,
        })

    # Verify strict monotonicity of center_y across increasing extension_deg
    y_seq = [p["center_y_px"] for p in points]
    strictly_decreasing = all(b < a for a, b in zip(y_seq, y_seq[1:]))
    strictly_increasing = all(b > a for a, b in zip(y_seq, y_seq[1:]))
    monotonic = strictly_decreasing or strictly_increasing

    return {
        "type": "calibrated_position_lut",
        "name": f"rig_position_lut_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "frame_height": int(frame_height),
        "working_zero_mdeg": int(working_zero_mdeg),
        "extension_sign": int(extension_sign),
        "monotonic": monotonic,
        "points": points,
        "hysteresis_summary": {
            "max_y_px": round(max(y_hyst_values), 3) if y_hyst_values else None,
            "mean_y_px": round(statistics.mean(y_hyst_values), 3) if y_hyst_values else None,
            "max_encoder_deg": round(max(enc_hyst_values), 3) if enc_hyst_values else None,
            "mean_encoder_deg": round(statistics.mean(enc_hyst_values), 3) if enc_hyst_values else None,
        },
        "source": {
            "video": video_path,
            "events": events_path,
            "total_holds": len(holds),
        },
    }


class PositionSweepStateMachine:
    """R=start/stop video, SPACE=record 3s hold / advance next angle, B=previous angle."""

    def __init__(
        self,
        angles: tuple[float, ...] = DEFAULT_SWEEP_ANGLES,
        hold_seconds: float = DEFAULT_HOLD_SECONDS,
    ):
        self.angles = tuple(float(a) for a in angles)
        self.hold_seconds = float(hold_seconds)
        self.reset()

    def reset(self) -> None:
        self.phase = "IDLE"  # IDLE, WAIT_STABLE, HOLDING, CAPTURED, FINISHED
        self.index = 0
        self.hold_started_at = 0.0
        self.hold_start_frame = 0
        self.current_y_samples: list[float] = []
        self.current_pos_samples: list[int] = []
        self.current_ext_samples: list[float] = []
        self.current_frame_height = 480
        self.completed_holds: dict[int, HoldRecord] = {}

    @property
    def current_angle_deg(self) -> float:
        return self.angles[min(self.index, len(self.angles) - 1)]

    @property
    def current_direction(self) -> str:
        peak_idx = self.angles.index(max(self.angles))
        return "outbound" if self.index <= peak_idx else "return"

    @property
    def next_angle_deg(self) -> float | None:
        return self.angles[self.index + 1] if self.index + 1 < len(self.angles) else None

    def start_recording(self) -> bool:
        if self.phase != "IDLE":
            return False
        self.reset()
        self.phase = "WAIT_STABLE"
        return True

    def on_space(self, now_s: float, current_frame_index: int) -> str:
        if self.phase == "WAIT_STABLE":
            self.phase = "HOLDING"
            self.hold_started_at = now_s
            self.hold_start_frame = current_frame_index
            self.current_y_samples = []
            self.current_pos_samples = []
            self.current_ext_samples = []
            return "HOLD_STARTED"
        if self.phase == "CAPTURED":
            if self.index + 1 < len(self.angles):
                self.index += 1
                self.phase = "WAIT_STABLE"
                return "NEXT_ANGLE"
            self.phase = "FINISHED"
            return "SEQUENCE_FINISHED"
        return "IGNORED"

    def on_back(self) -> bool:
        if self.phase not in ("WAIT_STABLE", "CAPTURED"):
            return False
        if self.phase == "CAPTURED":
            # Re-record current angle
            self.completed_holds.pop(self.index, None)
            self.phase = "WAIT_STABLE"
            return True
        if self.index > 0:
            self.index -= 1
            self.completed_holds.pop(self.index, None)
            self.phase = "WAIT_STABLE"
            return True
        return False

    def feed_sample(
        self,
        now_s: float,
        frame_index: int,
        frame_height: int,
        center_y: float | None,
        hx8_pos_mdeg: int | None,
        hx8_ext_deg: float | None,
    ) -> tuple[float, HoldRecord | None]:
        if self.phase != "HOLDING":
            return 0.0, None
        self.current_frame_height = frame_height
        if center_y is not None:
            self.current_y_samples.append(float(center_y))
        if hx8_pos_mdeg is not None:
            self.current_pos_samples.append(int(hx8_pos_mdeg))
        if hx8_ext_deg is not None:
            self.current_ext_samples.append(float(hx8_ext_deg))
        elapsed = max(0.0, now_s - self.hold_started_at)
        if elapsed >= self.hold_seconds:
            record = HoldRecord(
                step_index=self.index,
                direction=self.current_direction,
                guide_extension_deg=self.current_angle_deg,
                start_frame=self.hold_start_frame,
                end_frame=frame_index,
                started_at_s=self.hold_started_at,
                ended_at_s=now_s,
                center_y_samples=list(self.current_y_samples),
                hx8_pos_mdeg_samples=list(self.current_pos_samples),
                hx8_extension_deg_samples=list(self.current_ext_samples),
                frame_height=frame_height,
            )
            self.completed_holds[self.index] = record
            self.phase = "CAPTURED"
            return self.hold_seconds, record
        return elapsed, None

    def ordered_holds(self) -> list[HoldRecord]:
        return [self.completed_holds[i] for i in sorted(self.completed_holds)]


class _RecorderCanvas(QWidget):
    """Live camera preview with center-axis corridor, detected block, and step banner."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(640, 480)
        self.image: QImage | None = None
        self.block_box: tuple[int, int, int, int] | None = None
        self.block_center: tuple[float, float] | None = None
        self.corridor_half_px = 55
        self.banner_title = "按 R 开始录制视频与位置表"
        self.banner_sub = "支持主界面手动调角，或直接按 G 驱动 HX8 到目标角"
        self.banner_color = QColor("#1c3554")

    def update_view(
        self,
        image: QImage,
        block_box: tuple[int, int, int, int] | None,
        block_center: tuple[float, float] | None,
        corridor_half_px: int,
        title: str,
        sub: str,
        color: QColor,
    ) -> None:
        self.image = image
        self.block_box = block_box
        self.block_center = block_center
        self.corridor_half_px = corridor_half_px
        self.banner_title = title
        self.banner_sub = sub
        self.banner_color = color
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#0e1726"))
        if self.image is None or self.image.isNull():
            painter.setPen(QColor("#b8cadb"))
            painter.setFont(QFont("Noto Sans SC", 13))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "等待图传视频帧…\n请先在主界面打开 USB Video")
            return
        w, h = self.image.width(), self.image.height()
        scale = min(self.width() / max(1, w), self.height() / max(1, h))
        ox = (self.width() - w * scale) / 2.0
        oy = (self.height() - h * scale) / 2.0
        painter.save()
        painter.translate(ox, oy)
        painter.scale(scale, scale)
        painter.drawImage(QRectF(0, 0, w, h), self.image)

        cx_mid = w / 2.0
        left = max(0.0, cx_mid - self.corridor_half_px)
        right = min(float(w), cx_mid + self.corridor_half_px)
        painter.fillRect(QRectF(0, 0, left, h), QColor(0, 0, 0, 110))
        painter.fillRect(QRectF(right, 0, w - right, h), QColor(0, 0, 0, 110))
        painter.setPen(QPen(QColor("#33bfd5"), 2, Qt.PenStyle.DashLine))
        painter.drawLine(QPointF(left, 0), QPointF(left, h))
        painter.drawLine(QPointF(right, 0), QPointF(right, h))
        painter.setPen(QPen(QColor("#ffd166"), 1, Qt.PenStyle.DashLine))
        painter.drawLine(QPointF(cx_mid, 0), QPointF(cx_mid, h))

        if self.block_box is not None and self.block_center is not None:
            bx, by, bw, bh = self.block_box
            cx, cy = self.block_center
            painter.setPen(QPen(QColor("#4ee6aa"), 2))
            painter.drawRect(QRectF(bx, by, bw, bh))
            painter.setPen(QPen(QColor("#ff5252"), 2))
            painter.drawEllipse(QPointF(cx, cy), 4.5, 4.5)
            painter.drawLine(QPointF(cx - 10, cy), QPointF(cx + 10, cy))
            painter.drawLine(QPointF(cx, cy - 10), QPointF(cx, cy + 10))
        painter.restore()

        # Top HUD banner (never written to the saved raw video!)
        painter.fillRect(QRectF(0, 0, self.width(), 64), QColor(self.banner_color.red(), self.banner_color.green(), self.banner_color.blue(), 215))
        painter.setPen(QColor("#ffffff"))
        painter.setFont(QFont("Noto Sans SC", 13, QFont.Weight.Bold))
        painter.drawText(QRectF(14, 6, self.width() - 28, 28), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.banner_title)
        painter.setFont(QFont("Noto Sans SC", 10))
        painter.setPen(QColor("#e5f1fb"))
        painter.drawText(QRectF(14, 34, self.width() - 28, 24), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, self.banner_sub)


class PositionSweepRecorderDialog(QDialog):
    """Non-modal floating popup for recording 17-point (30°-70° outbound+return) position LUT."""

    lut_saved = Signal(object, str)  # CalibratedPositionLut, path

    def __init__(
        self,
        controller: ExperimentController,
        video: VideoCaptureService,
        black_config_provider,
        runtime_dir: Path,
        parent=None,
    ):
        super().__init__(parent, Qt.WindowType.Window)
        self.setWindowTitle("新版位置关系表录制（30°~70° 去程+回程 17 点）")
        self.setModal(False)
        self.resize(820, 680)
        self.controller = controller
        self.video = video
        self._black_config_provider = black_config_provider
        self.runtime_dir = runtime_dir
        self.sm = PositionSweepStateMachine()

        self._writer: cv2.VideoWriter | None = None
        self._events_file = None
        self._session_dir: Path | None = None
        self._video_path: Path | None = None
        self._events_path: Path | None = None
        self._recorded_frames = 0
        self._last_elapsed = 0.0
        self._last_block = None

        root = QVBoxLayout(self)
        root.setSpacing(6)

        self.canvas = _RecorderCanvas(self)
        root.addWidget(self.canvas, 1)

        ctrl_row = QHBoxLayout()
        self.record_btn = QPushButton("开始录制视频 (R)")
        self.record_btn.setObjectName("primaryButton")
        self.space_btn = QPushButton("录制当前位置 3s / 下个角度 (Space)")
        self.space_btn.setEnabled(False)
        self.back_btn = QPushButton("重录上一点 (B)")
        self.back_btn.setEnabled(False)
        self.goto_btn = QPushButton("驱动 HX8 到当前目标角 (G)")
        self.goto_btn.setToolTip("以 60°/s 仅移动 HX8 到当前引导角度（支持任意中间位置起步，绝不触碰夹爪）")
        ctrl_row.addWidget(self.record_btn)
        ctrl_row.addWidget(self.space_btn)
        ctrl_row.addWidget(self.back_btn)
        ctrl_row.addWidget(self.goto_btn)
        root.addLayout(ctrl_row)

        manual_row = QHBoxLayout()
        manual_row.addWidget(QLabel("快捷手动调角（不动夹爪）："))
        self.manual_angle_spin = QDoubleSpinBox()
        self.manual_angle_spin.setRange(0.0, 74.0)
        self.manual_angle_spin.setSingleStep(5.0)
        self.manual_angle_spin.setDecimals(1)
        self.manual_angle_spin.setValue(30.0)
        self.manual_angle_spin.setSuffix(" °")
        self.manual_move_btn = QPushButton("移动 HX8 到该角度")
        self.manual_move_btn.setToolTip("弹窗为非模态窗口，您也可随时切回主窗口手动控制")
        manual_row.addWidget(self.manual_angle_spin)
        manual_row.addWidget(self.manual_move_btn)
        manual_row.addStretch()
        self.hx8_label = QLabel("HX8 实际：等待反馈")
        manual_row.addWidget(self.hx8_label)
        root.addLayout(manual_row)

        self.status_label = QLabel(
            "准备就绪：先打开 USB Video 与串口。按 R 开始录制原视频；把机械臂移到 30° 停稳后按 Space 录制 3 秒，录完再按 Space 切下个角度。"
        )
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)

        self.record_btn.clicked.connect(self._toggle_recording)
        self.space_btn.clicked.connect(self._trigger_space)
        self.back_btn.clicked.connect(self._trigger_back)
        self.goto_btn.clicked.connect(self._move_to_current_guide)
        self.manual_move_btn.clicked.connect(self._move_to_manual_angle)
        self.video.frame_ready.connect(self._on_video_frame)
        self.controller.state_changed.connect(self._sync_hx8_status)

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(200)
        self._poll_timer.timeout.connect(self._poll_hx8)
        self._poll_timer.start()
        self._sync_hx8_status()

    def keyPressEvent(self, event: QKeyEvent) -> None:
        key = event.key()
        if key == Qt.Key.Key_R:
            self._toggle_recording()
            event.accept()
            return
        if key == Qt.Key.Key_Space:
            self._trigger_space()
            event.accept()
            return
        if key == Qt.Key.Key_B:
            self._trigger_back()
            event.accept()
            return
        if key == Qt.Key.Key_G:
            self._move_to_current_guide()
            event.accept()
            return
        super().keyPressEvent(event)

    def _poll_hx8(self) -> None:
        if not self.controller.simulated and self.controller.link_online:
            self.controller.query("STATE")

    def _current_hx8_extension_deg(self) -> tuple[int | None, float | None, str]:
        state = self.controller.latest_state
        if state is None or not state.hx8_online or state.pos_mdeg is None:
            return None, None, "离线"
        sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
        ext_deg = sign * (state.pos_mdeg - self.controller.working_zero_mdeg) / 1000.0
        return state.pos_mdeg, ext_deg, state.motion

    def _sync_hx8_status(self) -> None:
        pos_mdeg, ext_deg, motion = self._current_hx8_extension_deg()
        if pos_mdeg is None or ext_deg is None:
            self.hx8_label.setText("HX8 实际：未连接或无反馈")
        else:
            settled_zh = "已停稳" if motion == "IDLE" else motion
            self.hx8_label.setText(
                f"HX8 实际：绝对 {pos_mdeg / 1000:+.1f}°（出臂 {ext_deg:.1f}° · {settled_zh}）"
            )

    def _move_hx8_angle(self, ext_deg: float) -> None:
        if self.controller.simulated:
            self.status_label.setText(f"模拟模式：引导目标出臂 {ext_deg:.1f}°")
            return
        if self.controller.mode != "MANUAL":
            self.controller.request_mode("MANUAL")
            self.status_label.setText("正在切回 MANUAL 模式，请 0.2 秒后再按 G 移动 HX8")
            return
        if not self.controller.request_follow_move(12.0, ext_deg, speed_dps=60):
            self.status_label.setText(f"移动被拦截：{self.controller.command_status}")
        else:
            sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
            abs_deg = self.controller.working_zero_mdeg / 1000.0 + sign * ext_deg
            self.status_label.setText(
                f"已下发 HX8 移动到出臂 {ext_deg:.1f}°（绝对 {abs_deg:+.1f}°，速度 60°/s，不操作夹爪）；请等停稳后按 Space"
            )

    def _move_to_current_guide(self) -> None:
        angle = self.sm.current_angle_deg
        self.manual_angle_spin.setValue(angle)
        self._move_hx8_angle(angle)

    def _move_to_manual_angle(self) -> None:
        self._move_hx8_angle(self.manual_angle_spin.value())

    def _log_event(self, payload: dict) -> None:
        if self._events_file is None:
            return
        payload = {"ts": datetime.now().astimezone().isoformat(timespec="milliseconds"), **payload}
        self._events_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self._events_file.flush()

    def _toggle_recording(self) -> None:
        if self.sm.phase == "IDLE":
            frame = self.video.latest_frame
            if frame is None:
                self.status_label.setText("尚未收到视频帧！请先在主窗口打开 USB Video 再按 R 开始录制。")
                return
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._session_dir = self.runtime_dir / "position_recordings" / f"sweep_{stamp}"
            self._session_dir.mkdir(parents=True, exist_ok=True)
            self._video_path = self._session_dir / f"sweep_{stamp}.avi"
            self._events_path = self._session_dir / f"sweep_{stamp}.angles.jsonl"
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._writer = cv2.VideoWriter(str(self._video_path), fourcc, 30.0, (frame.width, frame.height))
            if not self._writer.isOpened():
                self._writer = None
                self.status_label.setText("无法创建视频写入器，请检查磁盘权限。")
                return
            self._events_file = self._events_path.open("w", encoding="utf-8")
            self._recorded_frames = 0
            self.sm.start_recording()
            self._log_event({
                "event": "session_start",
                "angles": list(self.sm.angles),
                "hold_seconds": self.sm.hold_seconds,
                "frame_size": [frame.width, frame.height],
                "working_zero_mdeg": self.controller.working_zero_mdeg,
            })
            self.manual_angle_spin.setValue(self.sm.current_angle_deg)
            self.record_btn.setText("停止录制并保存 (R)")
            self.space_btn.setEnabled(True)
            self.back_btn.setEnabled(True)
            self.status_label.setText(
                f"已开始录制原始视频！当前第 1/{len(self.sm.angles)} 点：去程 {self.sm.current_angle_deg:g}°。"
                "请调到该角度（可按 G 或在主界面手动调），停稳后按 Space 录制 3 秒。"
            )
        else:
            self._finish_session(auto_completed=False)

    def _trigger_space(self) -> None:
        if self.sm.phase == "IDLE":
            self.status_label.setText("请先按 R 开始录制视频，再按 Space 记录当前角度！")
            return
        action = self.sm.on_space(time.monotonic(), self._recorded_frames)
        if action == "HOLD_STARTED":
            dir_zh = "去程" if self.sm.current_direction == "outbound" else "回程"
            self._log_event({
                "event": "hold_start",
                "step_index": self.sm.index,
                "direction": self.sm.current_direction,
                "guide_extension_deg": self.sm.current_angle_deg,
                "frame_index": self._recorded_frames,
            })
            self.status_label.setText(
                f"正在录制第 {self.sm.index + 1}/{len(self.sm.angles)} 点（{dir_zh} {self.sm.current_angle_deg:g}°）— 请保持静止 3.0 秒…"
            )
        elif action == "NEXT_ANGLE":
            dir_zh = "去程" if self.sm.current_direction == "outbound" else "回程"
            self.manual_angle_spin.setValue(self.sm.current_angle_deg)
            self.status_label.setText(
                f"已切换到第 {self.sm.index + 1}/{len(self.sm.angles)} 点：{dir_zh} {self.sm.current_angle_deg:g}°。"
                "请调到该角度（可按 G 或主界面手动调），停稳后按 Space 录制 3 秒。"
            )
        elif action == "SEQUENCE_FINISHED":
            self._finish_session(auto_completed=True)

    def _trigger_back(self) -> None:
        if self.sm.on_back():
            dir_zh = "去程" if self.sm.current_direction == "outbound" else "回程"
            self.manual_angle_spin.setValue(self.sm.current_angle_deg)
            self.status_label.setText(
                f"已回退到第 {self.sm.index + 1}/{len(self.sm.angles)} 点（{dir_zh} {self.sm.current_angle_deg:g}°），请重新停稳后按 Space 录制。"
            )

    def _finish_session(self, *, auto_completed: bool) -> None:
        holds = self.sm.ordered_holds()
        self._log_event({
            "event": "session_end",
            "auto_completed": auto_completed,
            "recorded_frames": self._recorded_frames,
            "completed_holds": len(holds),
        })
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self._events_file is not None:
            self._events_file.close()
            self._events_file = None

        self.record_btn.setText("开始录制视频 (R)")
        self.space_btn.setEnabled(False)
        self.back_btn.setEnabled(False)

        saved_msg = f"原始视频已保存：{self._video_path}" if self._video_path else "录制已结束"
        if len(holds) >= 2 and self._session_dir is not None:
            try:
                sign = self.controller.cal.extension_sign if self.controller.cal and self.controller.cal.extension_sign else -1
                lut_dict = build_position_lut_from_holds(
                    holds,
                    frame_height=holds[0].frame_height,
                    working_zero_mdeg=self.controller.working_zero_mdeg,
                    extension_sign=sign,
                    video_path=str(self._video_path or ""),
                    events_path=str(self._events_path or ""),
                )
                session_lut_path = self._session_dir / "position_lut.json"
                session_lut_path.write_text(json.dumps(lut_dict, ensure_ascii=False, indent=2), encoding="utf-8")
                active_lut_path = self.runtime_dir / "calibrated_position_lut.json"
                active_lut_path.write_text(json.dumps(lut_dict, ensure_ascii=False, indent=2), encoding="utf-8")
                lut = CalibratedPositionLut(lut_dict)
                hyst = lut_dict["hysteresis_summary"]
                hyst_note = (
                    f"最大视觉回差 {hyst['max_y_px']:.1f} px"
                    if hyst["max_y_px"] is not None else "单程无回差数据"
                )
                mono_note = "严格单调 ✓" if lut_dict["monotonic"] else "警告：存在非单调点，请核查视频"
                self.status_label.setText(
                    f"完成！已采集 {len(holds)}/17 点并自动生成新版位置关系表（{mono_note} · {hyst_note}），已同步加载到视觉跟随模式。\n{saved_msg}"
                )
                self.lut_saved.emit(lut, str(active_lut_path))
            except Exception as exc:
                self.status_label.setText(f"{saved_msg}\n位置表拟合提示：{exc}")
        else:
            self.status_label.setText(f"{saved_msg}（已录 {len(holds)} 点，未生成位置表）")
        self.sm.reset()

    def _on_video_frame(self, frame: VideoFrame) -> None:
        cfg: BlackBlockConfig = self._black_config_provider()
        gray = qimage_gray(frame.image)
        blocks = detect_black_blocks(gray, cfg)
        block = blocks[0] if blocks else None
        self._last_block = block

        if self._writer is not None:
            bgr = qimage_bgr(frame.image)
            self._writer.write(bgr)
            self._recorded_frames += 1

        pos_mdeg, ext_deg, _motion = self._current_hx8_extension_deg()
        elapsed, finished_hold = self.sm.feed_sample(
            time.monotonic(),
            self._recorded_frames,
            frame.height,
            block.center_y if block is not None else None,
            pos_mdeg,
            ext_deg,
        )
        self._last_elapsed = elapsed
        if finished_hold is not None:
            self._log_event({"event": "hold_complete", **finished_hold.summary()})
            nxt = self.sm.next_angle_deg
            if nxt is not None:
                self.status_label.setText(
                    f"第 {self.sm.index + 1}/17 点（{finished_hold.guide_extension_deg:g}°）采集完成"
                    f"（检出 {len(finished_hold.center_y_samples)} 帧）！请按 Space 切到下一个角度 {nxt:g}°。"
                )
            else:
                self.status_label.setText(
                    f"最后一点 17/17（{finished_hold.guide_extension_deg:g}°）采集完成！请按 Space 结束录制并生成位置关系表。"
                )

        # Build HUD banner
        phase = self.sm.phase
        dir_zh = "去程" if self.sm.current_direction == "outbound" else "回程"
        step_str = f"[{self.sm.index + 1}/{len(self.sm.angles)} {dir_zh} {self.sm.current_angle_deg:g}°]"
        if phase == "IDLE":
            title = "待开始录制 · 按 R 键开始录制原始视频"
            sub = "支持主界面手动调角，或按 G 键驱动 HX8 到当前目标角（30°~70° 去程+回程 17 点）"
            color = QColor("#1c3554")
        elif phase == "WAIT_STABLE":
            title = f"{step_str}  请移至出臂 {self.sm.current_angle_deg:g}°，停稳后按 Space 开始录 3 秒"
            sub = f"按 G 自动移至 {self.sm.current_angle_deg:g}° | 或切回主窗口手动控制 | 按 B 回退上一点 | 按 R 结束"
            color = QColor("#155e75")
        elif phase == "HOLDING":
            title = f"{step_str}  保持静止采集中…  {elapsed:.1f} / {self.sm.hold_seconds:.1f} s"
            sub = f"已采有效黑块 {len(self.sm.current_y_samples)} 帧 · 请勿移动机械臂或泡沫块"
            color = QColor("#b91c1c")
        else:
            nxt = self.sm.next_angle_deg
            nxt_txt = f"下一个角度 {nxt:g}°" if nxt is not None else "完成并生成位置关系表"
            title = f"{step_str}  3 秒采集完成！按 Space → {nxt_txt}"
            sub = "如对本点不满意可按 B 重录本点 · 按 R 提前停止保存"
            color = QColor("#15803d")

        corridor = cfg.center_corridor_half_width_px or 55
        self.canvas.update_view(
            frame.image,
            (block.x, block.y, block.width, block.height) if block is not None else None,
            (block.center_x, block.center_y) if block is not None else None,
            corridor,
            title,
            sub,
            color,
        )

    def closeEvent(self, event) -> None:
        self._poll_timer.stop()
        if self._writer is not None or self._events_file is not None:
            self._finish_session(auto_completed=False)
        try:
            self.video.frame_ready.disconnect(self._on_video_frame)
        except RuntimeError:
            pass
        try:
            self.controller.state_changed.disconnect(self._sync_hx8_status)
        except RuntimeError:
            pass
        super().closeEvent(event)

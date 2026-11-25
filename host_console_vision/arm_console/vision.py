from __future__ import annotations

import math
import time

from PySide6.QtCore import QObject, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from .models import BlackBlock, Detection, VisionResult


def demo_overlay_columns(width: int, blocks: tuple[BlackBlock, ...]) -> tuple[float, float, float, tuple[float, ...]]:
    """Line locations in raw image coordinates; paint uses this directly."""
    return width / 4, width / 2, 3 * width / 4, (blocks[0].center_x,) if blocks else ()


def demo_center_regions(height: int) -> tuple[tuple[int, float, float], ...]:
    """Five raw-image vertical regions, numbered 1 bottom through 5 top."""
    if height <= 0:
        return ()
    return tuple((5 - top_index, height * top_index / 5, height * (top_index + 1) / 5)
                 for top_index in range(5))


class VisionSource(QObject):
    frame_ready = Signal(object, object)  # QImage, VisionResult

    def start(self) -> None: ...
    def stop(self) -> None: ...


class SimulatedVisionSource(VisionSource):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.timer = QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self._tick)
        self.start_time = 0.0

    def start(self) -> None:
        self.start_time = time.monotonic()
        self.timer.start()
        self._tick()

    def stop(self) -> None:
        self.timer.stop()

    def _tick(self) -> None:
        width, height = 640, 360
        elapsed = time.monotonic() - self.start_time
        u = 320 + 110 * math.sin(elapsed * 0.8)
        v = 190 + 54 * math.cos(elapsed * 0.65)
        image = QImage(width, height, QImage.Format.Format_RGB32)
        image.fill(QColor("#152436"))
        painter = QPainter(image)
        painter.setPen(QPen(QColor("#264259"), 1))
        for x in range(0, width, 40):
            painter.drawLine(x, 0, x, height)
        for y in range(0, height, 40):
            painter.drawLine(0, y, width, y)
        painter.setBrush(QColor("#6b877b"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QRectF(u - 31, v - 23, 62, 46))
        painter.end()
        detection = Detection("SIM-01", "模拟目标", 0.93, u, v, (u - 43, v - 35, 86, 70))
        result = VisionResult(width, height, detection, "模拟视觉源", True)
        self.frame_ready.emit(image, result)


class VisionCanvas(QWidget):
    """视频帧与检测结果分离输入，使用源图像像素坐标绘制叠加层。"""
    def __init__(self, parent=None, *, empty_text: str = "视觉源未接入\n等待图传帧与识别结果"):
        super().__init__(parent)
        self.image: QImage | None = None
        self.result: VisionResult | None = None
        self.empty_text = empty_text
        self.setMinimumSize(360, 220)

    def set_frame(self, image: QImage, result: VisionResult) -> None:
        self.image = image.copy()
        self.result = result
        self.update()

    def clear(self) -> None:
        self.image = None
        self.result = None
        self.update()

    def image_transform(self) -> tuple[float, float, float] | None:
        if self.result is None or self.result.width <= 0 or self.result.height <= 0:
            return None
        scale = min(self.width() / self.result.width, self.height() / self.result.height)
        return scale, (self.width() - self.result.width * scale) / 2, (self.height() - self.result.height * scale) / 2

    def source_to_widget(self, u: float, v: float) -> tuple[float, float] | None:
        transform = self.image_transform()
        if transform is None:
            return None
        scale, ox, oy = transform
        return ox + u * scale, oy + v * scale

    def widget_to_source(self, x: float, y: float) -> tuple[float, float] | None:
        transform = self.image_transform()
        if transform is None:
            return None
        scale, ox, oy = transform
        u, v = (x - ox) / scale, (y - oy) / scale
        if not (0 <= u <= self.result.width and 0 <= v <= self.result.height):
            return None
        return u, v

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#101a2a"))
        if self.image is None or self.result is None:
            painter.setPen(QColor("#aebdcb"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.empty_text)
            return
        width, height = self.result.width, self.result.height
        scale, ox, oy = self.image_transform()
        painter.translate(ox, oy)
        painter.scale(scale, scale)
        painter.drawImage(QRectF(0, 0, width, height), self.image)
        if self.result.roi_overlay:
            left, center, right, centroids = demo_overlay_columns(
                width, self.result.black_blocks or ())
            painter.fillRect(QRectF(0, 0, left, height), QColor(0, 0, 0, 105))
            painter.fillRect(QRectF(right, 0, width - right, height), QColor(0, 0, 0, 105))
            painter.setPen(QPen(QColor("#33bfd5"), 2, Qt.PenStyle.DashLine))
            for boundary in (left, right):
                painter.drawLine(QPointF(boundary, 0), QPointF(boundary, height))
            painter.setFont(QFont("Noto Sans SC", 10, QFont.Weight.Bold))
            painter.drawText(int(left + 5), min(height - 6, 18), "ROI 中央 50%")
            self._draw_center_segments(painter, width, height, left, right)
            painter.setPen(QPen(QColor("#f368ef"), 2))
            for cx in centroids:
                painter.drawLine(QPointF(cx, 0), QPointF(cx, height))
                painter.drawText(int(cx + 5), max(18, height - 14), "目标视觉重心")
        elif self.result.corridor_half_width_px is not None and self.result.corridor_half_width_px > 0:
            half_w = float(self.result.corridor_half_width_px)
            left = max(0.0, width / 2.0 - half_w)
            right = min(float(width), width / 2.0 + half_w)
            painter.fillRect(QRectF(0, 0, left, height), QColor(0, 0, 0, 120))
            painter.fillRect(QRectF(right, 0, width - right, height), QColor(0, 0, 0, 120))
            painter.setPen(QPen(QColor("#33bfd5"), 2, Qt.PenStyle.DashLine))
            for boundary in (left, right):
                painter.drawLine(QPointF(boundary, 0), QPointF(boundary, height))
            painter.setFont(QFont("Noto Sans SC", 10, QFont.Weight.Bold))
            painter.drawText(int(left + 4), min(height - 6, 18), f"中轴带 ±{int(half_w)}px")
            if self.result.center_segments_overlay:
                self._draw_center_segments(painter, width, height, left, right)
        elif self.result.center_segments_overlay:
            self._draw_center_segments(painter, width, height)
        if self.result.simulated and not self.result.center_segments_overlay:
            # 未标定抓取带仅用于模拟演示；真实源无识别时显示原始画面。
            painter.setPen(QPen(QColor("#d4ae68"), 2, Qt.PenStyle.DashLine))
            painter.drawRect(QRectF(width * 0.35, height * 0.17, width * 0.30, height * 0.66))
            painter.setFont(QFont("Noto Sans SC", 10))
            painter.drawText(16, height - 14, "抓取带示意 · 未标定")
        det = self.result.detection
        if det is not None:
            painter.setPen(QPen(QColor("#5de0cf"), 3))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRect(QRectF(*det.box))
            painter.drawLine(int(det.u - 8), int(det.v), int(det.u + 8), int(det.v))
            painter.drawLine(int(det.u), int(det.v - 8), int(det.u), int(det.v + 8))
            painter.drawText(int(det.box[0]), max(18, int(det.box[1] - 8)),
                             f"{det.target_id}  {det.confidence:.0%}  ({det.u:.0f}, {det.v:.0f}) px")
        if self.result.black_blocks is not None:
            normal_color = QColor("#4ee6aa")
            box_color = QColor("#ff3030") if self.result.black_block_aligned else normal_color
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setFont(QFont("Noto Sans SC", 10, QFont.Weight.Bold))
            for block in self.result.black_blocks[:1]:
                if len(block.rotated_box) >= 3:
                    painter.setPen(QPen(QColor("#36d6ff"), 2))
                    painter.drawPolygon(QPolygonF([QPointF(px, py) for px, py in block.rotated_box]))
                painter.setPen(QPen(box_color, 3 if self.result.black_block_aligned else 2))
                painter.drawRect(QRectF(block.x, block.y, block.width, block.height))
                painter.setPen(QPen(normal_color, 2))
                cx, cy = round(block.center_x), round(block.center_y)
                painter.drawEllipse(QRectF(cx - 4, cy - 4, 8, 8))
                painter.drawLine(cx - 9, cy, cx + 9, cy)
                painter.drawLine(cx, cy - 9, cx, cy + 9)
                painter.drawText(block.x, max(16, block.y - 6),
                                 f"目标视觉重心 ({cx}, {cy}) px")
        if self.result.simulated:
            painter.setPen(QColor("#ffcf6a"))
            painter.setFont(QFont("Noto Sans SC", 16, QFont.Weight.Bold))
            painter.drawText(16, 30, "模拟画面")

    @staticmethod
    def _draw_center_segments(painter: QPainter, width: int, height: int,
                              left: float | None = None, right: float | None = None) -> None:
        """Draw the shared five segments without requiring an ROI on the manual page."""
        center = width / 2
        colors = ("#ffd166", "#f7b267", "#f79d65", "#f4845f", "#f27059")
        regions = demo_center_regions(height)
        painter.setFont(QFont("Noto Sans SC", 10, QFont.Weight.Bold))
        for number, top, bottom in regions:
            painter.setPen(QPen(QColor(colors[number - 1]), 3))
            painter.drawLine(QPointF(center, top), QPointF(center, bottom))
            painter.drawText(int(center + 7), int((top + bottom) / 2), f"区域{number}")
        painter.setPen(QPen(QColor("#ffbf47"), 1, Qt.PenStyle.DashLine))
        for _, top, _ in regions[1:]:
            if left is not None and right is not None:
                painter.drawLine(QPointF(left, top), QPointF(right, top))
            painter.drawLine(QPointF(center - 8, top), QPointF(center + 8, top))
        painter.drawText(int(center + 7), min(height - 6, 18), "画面中心 · 5段")


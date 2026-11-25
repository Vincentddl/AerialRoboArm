from __future__ import annotations

from collections import deque
import math

import pyqtgraph as pg
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .models import AccelerationSample


class AccelerationPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.samples: deque[AccelerationSample] = deque(maxlen=6000)
        self.paused = False
        self.latest: AccelerationSample | None = None
        self._dirty = True
        self._time_origin: float | None = None

        root = QVBoxLayout(self)
        header = QHBoxLayout()
        title = QLabel("XYZ 加速度")
        title.setObjectName("sectionTitle")
        header.addWidget(title)
        header.addStretch()
        self.pause_button = QPushButton("暂停显示")
        self.pause_button.clicked.connect(self.toggle_pause)
        header.addWidget(self.pause_button)
        clear = QPushButton("清空曲线")
        clear.clicked.connect(self.clear)
        header.addWidget(clear)
        self.window_choice = QComboBox()
        for seconds in (5, 10, 30):
            self.window_choice.addItem(f"最近 {seconds} 秒", seconds)
        self.window_choice.setCurrentIndex(1)
        self.window_choice.currentIndexChanged.connect(self._mark_dirty)
        header.addWidget(self.window_choice)
        root.addLayout(header)
        self.status = QLabel("等待加速度数据 · 单位待定 · 时间源：接收时间")
        self.status.setWordWrap(True)
        self.status.hide()  # 保留状态文本供日志/测试读取，界面为图传让出高度

        pg.setConfigOptions(antialias=False, background="#101a2a", foreground="#cbd5e1")
        self.graphs = []
        self.curves = []
        self.value_labels = []
        for axis, color in (("X", "#ef6c70"), ("Y", "#42b3a7"), ("Z", "#e6b65b")):
            label = QLabel(f"{axis}: 等待数据")
            label.setStyleSheet(f"color: {color}; font-weight: 600")
            root.addWidget(label)
            self.value_labels.append(label)
            graph = pg.PlotWidget()
            graph.setMinimumHeight(105)
            graph.showGrid(x=True, y=True, alpha=0.2)
            graph.setLabel("left", axis)
            graph.setLabel("bottom", "时间", units="s")
            graph.getAxis("bottom").enableAutoSIPrefix(False)
            graph.getAxis("left").enableAutoSIPrefix(False)
            graph.setMouseEnabled(x=False, y=False)
            if self.graphs:
                graph.setXLink(self.graphs[0])
            self.curves.append(graph.plot(pen=pg.mkPen(color, width=2)))
            self.graphs.append(graph)
            root.addWidget(graph, 1)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def add_sample(self, sample: AccelerationSample) -> None:
        if not all(math.isfinite(v) for v in (sample.time_s, sample.ax, sample.ay, sample.az)):
            return
        self.samples.append(sample)
        if self._time_origin is None:
            self._time_origin = sample.time_s
        self.latest = sample
        self._dirty = True

    def _mark_dirty(self) -> None:
        self._dirty = True

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_button.setText("继续显示" if self.paused else "暂停显示")
        if not self.paused:
            self._dirty = True
            self.refresh()
        else:
            self.status.setText(self.status.text() + " · 显示已暂停（接收继续）")

    def clear(self) -> None:
        self.samples.clear()
        self._time_origin = None
        self.latest = None
        self._dirty = True
        for curve in self.curves:
            curve.setData([], [])
        for axis, label in zip("XYZ", self.value_labels):
            label.setText(f"{axis}: 等待数据")
        self.status.setText("等待加速度数据 · 单位待定")

    def refresh(self) -> None:
        if self.paused or not self._dirty:
            return
        self._dirty = False
        if not self.samples:
            return
        latest = self.latest
        if latest is None:
            return
        span = self.window_choice.currentData()
        visible = [s for s in self.samples if s.time_s >= latest.time_s - span and s.source == latest.source]
        if not visible:
            return
        xs = [s.time_s - (self._time_origin or s.time_s) for s in visible]
        for idx, key in enumerate(("ax", "ay", "az")):
            self.curves[idx].setData(xs, [getattr(s, key) for s in visible])
            self.value_labels[idx].setText(f"{'XYZ'[idx]}: {getattr(latest, key):.4f} {latest.unit}")
        self.graphs[0].setXRange(max(xs[-1] - span, xs[0]), max(xs[-1], xs[0] + 0.01), padding=0)
        self.status.setText(f"来源：{latest.source} · 单位：{latest.unit} · 时间源：{latest.time_source}\n缓存 {len(self.samples)} 条")

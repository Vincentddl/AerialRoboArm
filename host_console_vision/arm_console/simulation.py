from __future__ import annotations

import math
import time

from PySide6.QtCore import QObject, QTimer, Signal

from .models import AccelerationSample
from .vision import SimulatedVisionSource


class SimulationService(QObject):
    acceleration = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self._tick)
        self.vision = SimulatedVisionSource(self)
        self.started = 0.0

    @property
    def active(self) -> bool:
        return self.timer.isActive()

    def start(self) -> None:
        if self.active:
            return
        self.started = time.monotonic()
        self.timer.start()
        self.vision.start()

    def stop(self) -> None:
        self.timer.stop()
        self.vision.stop()

    def _tick(self) -> None:
        now = time.monotonic()
        t = now - self.started
        self.acceleration.emit(AccelerationSample(
            now,
            0.24 * math.sin(2.2 * t) + 0.035 * math.sin(8.0 * t),
            0.16 * math.sin(1.5 * t + 1.1),
            0.32 * math.sin(2.8 * t + 0.4),
            "模拟单位", "模拟演示", "模拟时钟",
        ))

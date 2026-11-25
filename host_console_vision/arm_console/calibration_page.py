from PySide6.QtWidgets import QDoubleSpinBox, QFormLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .v1_state import ExperimentController


class CalibrationPage(QWidget):
    def __init__(self, controller: ExperimentController):
        super().__init__()
        self.controller = controller
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("校准模式"))
        hint = QLabel("以低速试移至实际正常位。到位后，用 HX8 实际反馈作为正常位保存。"
                      "折叠偏移为沿现有折叠方向的角度，可先试移到折叠目标，再回到正常位确认。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        form = QFormLayout()
        self.actual = QLabel()
        self.saved = QLabel()
        self.preview = QLabel()
        self.save_preview = QLabel()
        self.save_preview.setWordWrap(True)
        self.normal = QDoubleSpinBox()
        self.normal.setRange(-90, 180)
        self.normal.setDecimals(1)
        self.normal.setSingleStep(0.5)
        self.normal.setSuffix(" °")
        self.normal.setValue(controller.working_zero_mdeg / 1000)
        self.offset = QDoubleSpinBox()
        self.offset.setRange(10, 196)
        self.offset.setDecimals(1)
        self.offset.setSingleStep(0.5)
        self.offset.setSuffix(" °")
        self.offset.setValue(controller.fold_offset_mdeg / 1000)
        form.addRow("HX8 实际角度", self.actual)
        form.addRow("试移正常位（HX8 绝对角度）", self.normal)
        form.addRow("折叠偏移（相对正常位）", self.offset)
        form.addRow("试移折叠目标", self.preview)
        form.addRow("现在保存将得到", self.save_preview)
        form.addRow("当前已保存", self.saved)
        layout.addLayout(form)
        self.move_normal = QPushButton("低速试移到输入角度")
        self.move_fold = QPushButton("低速试移到计算折叠位")
        self.save = QPushButton("将当前实际角度设为正常位并保存")
        self.stop = QPushButton("停止运动")
        self.stop.clicked.connect(controller.request_stop)
        self.status = QLabel(controller.calibration_load_error)
        self.status.setWordWrap(True)
        self._save_error_context = None
        for widget in (self.move_normal, self.move_fold, self.stop, self.save, self.status):
            layout.addWidget(widget)
        layout.addStretch()
        self.move_normal.clicked.connect(lambda: self._move(self.normal.value()))
        self.move_fold.clicked.connect(lambda: self._move(self.normal.value() + self.offset.value()))
        self.save.clicked.connect(self._save)
        self.normal.valueChanged.connect(self._refresh)
        self.offset.valueChanged.connect(self._refresh)
        controller.state_changed.connect(self._refresh)
        self._refresh()

    def _refresh(self):
        c = self.controller
        current = c.calibration_position()
        if (self._save_error_context is not None
                and self._save_error_context != (current, self.offset.value())):
            self.status.clear()
            self._save_error_context = None
        self.actual.setText("等待空闲反馈" if current is None else f"{current / 1000:+.1f}°")
        self.preview.setText(f"{self.normal.value() + self.offset.value():+.1f}°")
        if current is None:
            self.save_preview.setText("等待 HX8 停稳后计算；输入的试移角度不会直接保存")
        else:
            folded = current / 1000 + self.offset.value()
            detail = f"正常位 {current / 1000:+.1f}°，折叠位 {folded:+.1f}°"
            if folded > 180:
                detail += "（超出固件 +180°；先低速试移到实际正常位）"
            self.save_preview.setText(detail)
        self.saved.setText(f"正常位 {c.working_zero_mdeg / 1000:+.1f}°，"
                           f"偏移 {c.fold_offset_mdeg / 1000:.1f}°，折叠位 {c.fold_target_mdeg / 1000:+.1f}°")
        if c.pose_calibration_saved:
            self.saved.setText(self.saved.text() + (" · 固件已同步" if c.pose_synced else " · 等待固件同步"))
        ready = current is not None and c.mode == "MANUAL" and not c.control_busy
        self.move_normal.setEnabled(ready)
        self.move_fold.setEnabled(ready and self.normal.value() + self.offset.value() <= 180)
        self.save.setEnabled(ready)
        self.stop.setEnabled(c.link_online and not c.simulated)

    def _move(self, degrees):
        self._save_error_context = None
        self.controller.move_for_calibration(round(degrees * 10) * 100)
        self.status.setText(self.controller.command_status)

    def _save(self):
        c = self.controller
        if c.save_pose_calibration(round(self.offset.value() * 10) * 100):
            self._save_error_context = None
            self.normal.setValue(c.working_zero_mdeg / 1000)
            self.status.setText("校准已保存，下次启动会自动加载。")
        else:
            self.status.setText(c.command_status)
            self._save_error_context = ((c.calibration_position(), self.offset.value())
                                        if c.command_status.startswith("当前实际角度") else None)

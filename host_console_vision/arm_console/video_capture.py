"""Windows DirectShow camera enumeration and one shared, bounded video stream."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Callable

import cv2
from cv2_enumerate_cameras import enumerate_cameras
from PySide6.QtCore import QObject, QTimer, Qt, Signal
from PySide6.QtGui import QImage


@dataclass(frozen=True)
class VideoDevice:
    name: str
    index: int
    backend: int
    path: str
    vid: int | None = None
    pid: int | None = None

    @property
    def label(self) -> str:
        identity = self.path.split("#")[1:3]
        suffix = " / ".join(identity) if identity else (f"{self.vid:04X}:{self.pid:04X}" if self.vid is not None and self.pid is not None else "无设备路径")
        return f"{self.name} · DirectShow {self.index} · {suffix}"


@dataclass(frozen=True)
class VideoFrame:
    image: QImage  # 自有像素内存；不引用 OpenCV / NumPy 的临时缓冲
    frame_id: int
    received_monotonic_s: float  # 电脑读取完成时间，非摄像头曝光时间
    width: int
    height: int


@dataclass(frozen=True)
class VideoStatus:
    phase: str
    detail: str
    device_name: str = ""
    width: int = 0
    height: int = 0
    read_fps: float = 0.0
    reported_fps: float = 0.0


def list_directshow_devices() -> list[VideoDevice]:
    """枚举值与 VideoCapture(index, CAP_DSHOW) 使用同一后端。"""
    return [VideoDevice(d.name, d.index, d.backend, d.path or "", d.vid, d.pid)
            for d in enumerate_cameras(cv2.CAP_DSHOW)]


def resolve_device(selected: VideoDevice, available: list[VideoDevice]) -> VideoDevice:
    if selected.path:
        matches = [d for d in available if d.path.lower() == selected.path.lower() and d.backend == selected.backend]
    else:
        matches = [d for d in available if d.name == selected.name and d.index == selected.index and d.backend == selected.backend]
    if len(matches) != 1:
        raise RuntimeError("所选视频设备已拔出或编号变化，请刷新设备列表后重选")
    return matches[0]


def make_owned_qimage(bgr) -> QImage:
    if bgr is None or bgr.ndim not in (2, 3) or bgr.shape[0] < 1 or bgr.shape[1] < 1:
        raise ValueError("空画面或图像维度无效")
    channels = 1 if bgr.ndim == 2 else bgr.shape[2]
    if channels == 1:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_GRAY2RGB)
    elif channels == 3:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    elif channels == 4:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGRA2RGB)
    else:
        raise ValueError(f"不支持的图像通道数：{channels}")
    height, width = rgb.shape[:2]
    image = QImage(rgb.data, width, height, rgb.strides[0], QImage.Format.Format_RGB888).copy()
    if image.isNull():
        raise ValueError("图像缓冲转换失败")
    return image


class VideoCaptureService(QObject):
    """采集和枚举在线程中执行；Qt 定时器仅提取最新帧到主线程。"""
    devices_changed = Signal(object)
    status_changed = Signal(object)
    frame_ready = Signal(object)
    frame_cleared = Signal()
    event = Signal(str)
    _notify = Signal()

    def __init__(self, parent=None, *, enumerator: Callable[[], list[VideoDevice]] = list_directshow_devices,
                 capture_factory: Callable = cv2.VideoCapture):
        super().__init__(parent)
        self._enumerator = enumerator
        self._capture_factory = capture_factory
        self._lock = threading.Lock()
        self._capture_thread: threading.Thread | None = None
        self._scan_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._latest: VideoFrame | None = None
        self._delivered_frame_id = 0
        self._generation = 0
        self._status = VideoStatus("未打开", "请选择设备并打开")
        self._shown_status = self._status
        self._scan_result: tuple[list[VideoDevice] | None, str | None] | None = None
        self._events: deque[str] = deque(maxlen=30)
        self._exposure_lock_enabled = False
        self._exposure_value = -7.0
        self._gain_value = 20.0
        self._exposure_dirty = False
        self.devices: list[VideoDevice] = []
        self._notify.connect(self.poll, Qt.ConnectionType.QueuedConnection)
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.setInterval(12)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    @property
    def status(self) -> VideoStatus:
        with self._lock:
            return self._status

    @property
    def latest_frame(self) -> VideoFrame | None:
        with self._lock:
            return self._latest

    @property
    def exposure_lock_enabled(self) -> bool:
        with self._lock:
            return self._exposure_lock_enabled

    def set_exposure_lock(self, enabled: bool, *, exposure: float = -7.0, gain: float = 20.0) -> None:
        with self._lock:
            changed = (
                self._exposure_lock_enabled != bool(enabled)
                or self._exposure_value != float(exposure)
                or self._gain_value != float(gain)
            )
            self._exposure_lock_enabled = bool(enabled)
            self._exposure_value = float(exposure)
            self._gain_value = float(gain)
            if changed:
                self._exposure_dirty = True

    def _apply_exposure_settings(self, capture, *, force_only_when_enabled: bool = False) -> None:
        if not hasattr(capture, "set"):
            return
        with self._lock:
            enabled = self._exposure_lock_enabled
            exposure = self._exposure_value
            gain = self._gain_value
            self._exposure_dirty = False
        if force_only_when_enabled and not enabled:
            return
        try:
            if enabled:
                capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
                capture.set(cv2.CAP_PROP_GAIN, gain)
                capture.set(cv2.CAP_PROP_AUTO_WB, 0.0)
                self._event(f"已应用防拖影低曝光锁定：exposure={exposure:g}, gain={gain:g}")
            else:
                capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
                capture.set(cv2.CAP_PROP_AUTO_WB, 1.0)
                self._event("已恢复摄像头自动曝光与自动白平衡")
        except Exception as exc:
            self._event(f"设置摄像头曝光参数未完全生效：{exc}")

    def refresh_devices(self) -> bool:
        if self._scan_thread is not None and self._scan_thread.is_alive():
            return False
        if self._capture_thread is not None and self._capture_thread.is_alive():
            return False
        self._set_status(VideoStatus("扫描中", "正在枚举 DirectShow 视频设备"))
        self._scan_thread = threading.Thread(target=self._scan_devices, name="video-device-scan", daemon=True)
        self._scan_thread.start()
        return True

    def _scan_devices(self) -> None:
        try:
            result = (self._enumerator(), None)
        except Exception as exc:
            result = (None, f"枚举视频设备失败：{exc}")
        with self._lock:
            self._scan_result = result

    def open_device(self, selected: VideoDevice) -> bool:
        if self._capture_thread is not None and self._capture_thread.is_alive():
            return False
        if self._scan_thread is not None and self._scan_thread.is_alive():
            return False
        self._stop = threading.Event()
        with self._lock:
            self._generation += 1
            generation = self._generation
            self._latest = None
            self._delivered_frame_id = 0
        self.frame_cleared.emit()
        self._set_status(VideoStatus("打开中", "正在打开设备并验证连续画面", selected.name))
        self._capture_thread = threading.Thread(target=self._capture, args=(selected, generation, self._stop),
                                                name="video-capture", daemon=True)
        self._capture_thread.start()
        return True

    def close_device(self) -> None:
        self._stop.set()
        with self._lock:
            self._generation += 1
            self._latest = None
        self.frame_cleared.emit()
        if self._capture_thread is not None and self._capture_thread.is_alive():
            self._set_status(VideoStatus("关闭中", "等待采集线程释放设备", self.status.device_name))
        else:
            self._set_status(VideoStatus("未打开", "视频设备已关闭"))

    def _set_status(self, status: VideoStatus) -> None:
        with self._lock:
            self._status = status

    def _event(self, message: str) -> None:
        with self._lock:
            self._events.append(message)

    def _capture(self, selected: VideoDevice, generation: int, stop: threading.Event) -> None:
        capture = None
        failed = False
        try:
            device = resolve_device(selected, self._enumerator())
            self._event(f"打开视频设备：{device.name}；索引={device.index}；后端={device.backend}；路径={device.path or '未提供'}")
            capture = self._capture_factory(device.index, device.backend)
            if not capture.isOpened():
                raise RuntimeError("DirectShow 打开失败；请检查 USB Video 是否接入，以及 OBS、相机软件是否占用")
            if hasattr(capture, "set"):
                try:
                    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                except Exception:
                    pass
            self._apply_exposure_settings(capture, force_only_when_enabled=True)
            reported_width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
            reported_height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
            reported_fps = capture.get(cv2.CAP_PROP_FPS)
            self._event(f"设备报告参数：{reported_width:g}×{reported_height:g}，标称帧率 {reported_fps:g} fps；未强制设置格式")
            verified = 0
            frame_id = 0
            started = time.monotonic()
            last_valid = started
            last_status = started
            frame_times: deque[float] = deque(maxlen=120)
            width = height = 0
            while not stop.is_set():
                with self._lock:
                    exposure_dirty = self._exposure_dirty
                if exposure_dirty:
                    self._apply_exposure_settings(capture)
                try:
                    ok, bgr = capture.read()
                    received_at = time.monotonic()
                    if stop.is_set():
                        break
                    if not ok:
                        raise ValueError("read() 未返回有效帧")
                    image = make_owned_qimage(bgr)
                except Exception as exc:
                    verified = 0 if frame_id == 0 else verified
                    now = time.monotonic()
                    if frame_id == 0 and now - started >= 5.0:
                        raise RuntimeError(f"设备已打开但未连续取得有效画面：{exc}") from exc
                    if frame_id > 0 and now - last_valid >= 1.0:
                        with self._lock:
                            if self._generation == generation:
                                self._latest = None
                        self._set_status(VideoStatus("无有效画面", "超过1秒没有新帧；已清除旧画面", selected.name, width, height))
                    if frame_id > 0 and now - last_valid >= 2.0:
                        raise RuntimeError(f"持续读取失败或设备拔出：{exc}") from exc
                    stop.wait(0.03)
                    continue
                now = received_at
                last_valid = now
                verified += 1
                if verified < 3:
                    continue
                frame_id += 1
                height, width = image.height(), image.width()
                frame_times.append(now)
                while frame_times and now - frame_times[0] > 2.0:
                    frame_times.popleft()
                read_fps = ((len(frame_times) - 1) / (frame_times[-1] - frame_times[0])
                            if len(frame_times) > 1 and frame_times[-1] > frame_times[0] else 0.0)
                with self._lock:
                    if self._generation == generation:
                        self._latest = VideoFrame(image, frame_id, now, width, height)
                        notify = True
                    else:
                        notify = False
                if frame_id == 1 or now - last_status >= 0.5:
                    self._set_status(VideoStatus("采集中", "已连续取得有效画面", selected.name,
                                                 width, height, read_fps, reported_fps))
                    last_status = now
                if notify:
                    self._notify.emit()
        except Exception as exc:
            failed = True
            self._event(f"视频采集失败：{exc}")
            if not stop.is_set():
                self._set_status(VideoStatus("失败", f"{exc}；请检查 OBS 或其他相机软件是否占用", selected.name))
                with self._lock:
                    if self._generation == generation:
                        self._latest = None
        finally:
            if capture is not None:
                try:
                    capture.release()
                except Exception as exc:
                    self._event(f"释放视频设备异常：{exc}")
            self._event(f"视频设备已释放：{selected.name}")
            if stop.is_set() or not failed:
                self._set_status(VideoStatus("未打开", "视频设备已关闭", selected.name))

    def poll(self) -> None:
        with self._lock:
            scan = self._scan_result
            self._scan_result = None
            events = list(self._events)
            self._events.clear()
            status = self._status
            frame = self._latest
            generation = self._generation
        if status.phase == "采集中" and frame is not None and time.monotonic() - frame.received_monotonic_s > 1.0:
            with self._lock:
                if self._generation == generation and self._latest is frame:
                    self._latest = None
                    self._status = VideoStatus("无有效画面", "超过1秒没有新帧；已清除旧画面",
                                               status.device_name, status.width, status.height)
                    frame = None
                    status = self._status
        if scan is not None:
            devices, error = scan
            if error:
                self.devices = []
                self.devices_changed.emit([])
                self._set_status(VideoStatus("失败", error))
                self.event.emit(error)
            else:
                self.devices = devices or []
                self.devices_changed.emit(self.devices)
                self._set_status(VideoStatus("未打开", f"发现 {len(self.devices)} 个 DirectShow 视频设备"))
            status = self.status
        for message in events:
            self.event.emit(message)
        if status != self._shown_status:
            self._shown_status = status
            self.status_changed.emit(status)
            if status.phase in ("失败", "无有效画面"):
                self.frame_cleared.emit()
        if frame is not None and frame.frame_id != self._delivered_frame_id:
            with self._lock:
                current = self._generation == generation
            if current and status.phase == "采集中":
                self._delivered_frame_id = frame.frame_id
                self.frame_ready.emit(frame)

    def shutdown(self) -> bool:
        self.timer.stop()
        self.close_device()
        capture = self._capture_thread
        scan = self._scan_thread
        if capture is not None:
            capture.join(timeout=2.0)
        if scan is not None:
            scan.join(timeout=1.0)
        return not ((capture is not None and capture.is_alive()) or (scan is not None and scan.is_alive()))

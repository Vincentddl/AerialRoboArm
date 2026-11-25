"""Central-half OpenCV black-block detection on shared, owned video frames."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading

import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, Qt, Signal
from PySide6.QtGui import QImage

from .models import BlackBlock
from .video_capture import VideoFrame


@dataclass(frozen=True)
class BlackBlockConfig:
    gray_threshold: int = 70
    min_area_px: int = 250
    max_area_percent: float = 25.0
    morphology_kernel: int = 3
    min_fill_ratio: float = 0.55
    track_miss_limit: int = 20
    track_margin_px: int = 70
    track_growth_px: int = 8
    track_max_center_distance_px: int = 60
    track_max_area_ratio: float = 3.0
    instant_reacquire: bool = False
    use_rotated_fill_ratio: bool = False
    allow_roi_touch: bool = False
    center_corridor_half_width_px: int | None = None


def roi_pixel_bounds(width: int) -> tuple[int, int]:
    """Integer pixel indices in [W/4, 3W/4); right bound is exclusive."""
    if width < 1:
        raise ValueError("图像宽度无效")
    return math.ceil(width / 4), math.ceil(3 * width / 4)


def _search_horizontal_bounds(width: int, config: BlackBlockConfig) -> tuple[int, int]:
    if config.center_corridor_half_width_px is not None:
        half = max(8, int(config.center_corridor_half_width_px))
        cx = width / 2.0
        return max(0, int(math.floor(cx - half))), min(width, int(math.ceil(cx + half)))
    return roi_pixel_bounds(width)


def _validate(gray: np.ndarray, config: BlackBlockConfig) -> None:
    if gray.ndim != 2 or gray.size == 0:
        raise ValueError("识别输入必须是非空灰度原图")
    if not 0 <= config.gray_threshold <= 255 or config.min_area_px < 1 or not 0 < config.max_area_percent <= 100:
        raise ValueError("黑块识别参数超出范围")
    if (config.track_miss_limit < 1 or config.track_margin_px < 1 or config.track_growth_px < 0 or
            config.track_max_center_distance_px < 1 or config.track_max_area_ratio < 1):
        raise ValueError("黑块跟踪参数超出范围")


def _find_candidates(gray: np.ndarray, config: BlackBlockConfig,
                     region: tuple[int, int, int, int], *, reject_clipped: bool = False
                     ) -> tuple[list[tuple[BlackBlock, np.ndarray]], np.ndarray]:
    left, top, right, bottom = region
    roi = gray[top:bottom, left:right]
    if roi.size == 0:
        return [], np.empty((0, 0), dtype=np.uint8)
    _, binary = cv2.threshold(roi, config.gray_threshold, 255, cv2.THRESH_BINARY_INV)
    k = max(1, int(config.morphology_kernel))
    if k > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = gray.shape[0] * gray.shape[1] * config.max_area_percent / 100.0
    candidates: list[tuple[BlackBlock, np.ndarray]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < config.min_area_px or area > max_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if (reject_clipped and not config.allow_roi_touch and
                (x == 0 or y == 0 or x + width >= roi.shape[1] or y + height >= roi.shape[0])):
            # A component cut by the search boundary is incomplete. In
            # particular, this prevents a dark object outside the central-half
            # ROI from being accepted through its small in-ROI fragment.
            continue
        if width < 4 or height < 4:
            continue
        fill_ratio = area / (width * height)
        if config.use_rotated_fill_ratio:
            (_, (rot_w, rot_h), _) = cv2.minAreaRect(contour)
            rot_area = float(rot_w) * float(rot_h)
            if rot_area >= 16.0:
                fill_ratio = max(fill_ratio, area / rot_area)
        if fill_ratio < config.min_fill_ratio:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        cx = moments["m10"] / moments["m00"] + left
        cy = moments["m01"] / moments["m00"] + top
        if config.center_corridor_half_width_px is not None:
            if abs(cx - gray.shape[1] / 2.0) > config.center_corridor_half_width_px:
                continue
            aspect = width / max(1.0, float(height))
            if not (0.28 <= aspect <= 3.5):
                continue
        candidate = BlackBlock(x + left, y + top, width, height, cx, cy, area)
        candidates.append((candidate, contour))
    return candidates, binary


def _selected_block(gray: np.ndarray, region: tuple[int, int, int, int], binary: np.ndarray,
                    candidates: list[tuple[BlackBlock, np.ndarray]], index: int) -> BlackBlock:
    largest, contour = candidates[index]
    left, top, right, _ = region
    largest = replace(largest, other_centers=tuple(
        (candidate.center_x, candidate.center_y)
        for i, (candidate, _) in enumerate(candidates) if i != index))
    # Check the selected connected black component, never merely its bounding box
    # or another dark region sharing the same image column.
    center = gray.shape[1] / 2
    center_columns = {math.floor(center), math.ceil(center) - 1}
    if any(largest.x <= column < largest.x + largest.width for column in center_columns):
        _, labels = cv2.connectedComponents(binary, connectivity=8)
        seed_x, seed_y = contour[0, 0]
        selected_label = labels[seed_y, seed_x]
        touching = selected_label != 0 and any(
            left <= column < right and np.any(labels[:, column - left] == selected_label)
            for column in center_columns)
        largest = replace(largest, touches_center_line=bool(touching))
    return largest


def _search_score(candidate: tuple[BlackBlock, np.ndarray], config: BlackBlockConfig) -> float:
    return candidate[0].area_px


def detect_black_blocks(gray: np.ndarray, config: BlackBlockConfig) -> tuple[BlackBlock, ...]:
    """SEARCH rule: largest qualified contour in the configured central corridor."""
    _validate(gray, config)
    left, right = _search_horizontal_bounds(gray.shape[1], config)
    region = (left, 0, right, gray.shape[0])
    candidates, binary = _find_candidates(gray, config, region, reject_clipped=True)
    if not candidates:
        return ()
    index = max(range(len(candidates)), key=lambda i: candidates[i][0].area_px)
    return (_selected_block(gray, region, binary, candidates, index),)


@dataclass(frozen=True)
class BlackBlockObservation:
    blocks: tuple[BlackBlock, ...]
    phase: str  # SEARCH or TRACK
    missed_frames: int
    miss_limit: int
    last_known: BlackBlock | None = None
    target_id: int | None = None
    diagnostic_image: QImage | None = None


def _box_iou(a: BlackBlock, b: BlackBlock) -> float:
    x_overlap = max(0, min(a.x + a.width, b.x + b.width) - max(a.x, b.x))
    y_overlap = max(0, min(a.y + a.height, b.y + b.height) - max(a.y, b.y))
    intersection = x_overlap * y_overlap
    union = a.width * a.height + b.width * b.height - intersection
    return intersection / union if union else 0.0


class BlackBlockTracker:
    """One target, updated only from successfully decoded video frames."""

    def __init__(self) -> None:
        self._next_target_id = 0
        self.reset()

    def reset(self) -> None:
        self.last: BlackBlock | None = None
        self.missed_frames = 0
        self.target_id: int | None = None

    def _local_region(self, width: int, height: int, config: BlackBlockConfig) -> tuple[int, int, int, int]:
        block = self.last
        margin = config.track_margin_px + self.missed_frames * config.track_growth_px
        roi_left, roi_right = _search_horizontal_bounds(width, config)
        return (max(roi_left, block.x - margin), max(0, block.y - margin),
                min(roi_right, block.x + block.width + margin),
                min(height, block.y + block.height + margin))

    def process(self, gray: np.ndarray | None, config: BlackBlockConfig) -> BlackBlockObservation | None:
        # An invalid camera read produces no observation and never increments misses.
        if gray is None or not isinstance(gray, np.ndarray) or gray.ndim != 2 or gray.size == 0:
            return None
        _validate(gray, config)
        width, height = gray.shape[1], gray.shape[0]
        if self.last is None:
            left, right = _search_horizontal_bounds(width, config)
            region = (left, 0, right, height)
            candidates, binary = _find_candidates(gray, config, region, reject_clipped=True)
            if not candidates:
                return BlackBlockObservation((), "SEARCH", 0, config.track_miss_limit)
            index = max(range(len(candidates)), key=lambda i: _search_score(candidates[i], config))
            selected = _selected_block(gray, region, binary, candidates, index)
            self.last = selected
            self.missed_frames = 0
            self._next_target_id += 1
            self.target_id = self._next_target_id
            return BlackBlockObservation((selected,), "TRACK", 0, config.track_miss_limit,
                                         selected, self.target_id)

        region = self._local_region(width, height, config)
        candidates, binary = _find_candidates(gray, config, region, reject_clipped=True)
        previous = self.last
        matches: list[tuple[float, int]] = []
        for index, (candidate, _) in enumerate(candidates):
            distance = math.hypot(candidate.center_x - previous.center_x,
                                  candidate.center_y - previous.center_y)
            overlap = _box_iou(previous, candidate)
            area_ratio = max(previous.area_px, candidate.area_px) / min(previous.area_px, candidate.area_px)
            if area_ratio > config.track_max_area_ratio:
                continue
            if distance > config.track_max_center_distance_px and overlap < 0.05:
                continue
            score = (distance / config.track_max_center_distance_px +
                     0.3 * (1 - overlap) + 0.25 * abs(math.log(area_ratio)))
            matches.append((score, index))
        if matches:
            index = min(matches)[1]
            selected = _selected_block(gray, region, binary, candidates, index)
            self.last = selected
            self.missed_frames = 0
            return BlackBlockObservation((selected,), "TRACK", 0, config.track_miss_limit,
                                         selected, self.target_id)

        if config.instant_reacquire:
            left, right = _search_horizontal_bounds(width, config)
            full_region = (left, 0, right, height)
            full_candidates, full_binary = _find_candidates(gray, config, full_region, reject_clipped=True)
            if full_candidates:
                index = max(range(len(full_candidates)), key=lambda i: _search_score(full_candidates[i], config))
                selected = _selected_block(gray, full_region, full_binary, full_candidates, index)
                self.last = selected
                self.missed_frames = 0
                self._next_target_id += 1
                self.target_id = self._next_target_id
                return BlackBlockObservation((selected,), "TRACK", 0, config.track_miss_limit,
                                             selected, self.target_id)

        self.missed_frames += 1
        missed = self.missed_frames
        target_id = self.target_id
        if missed >= config.track_miss_limit:
            self.reset()  # Next valid frame starts SEARCH in the central half.
        return BlackBlockObservation((), "TRACK", missed, config.track_miss_limit, previous, target_id)


class BlackBlockAlignment:
    """UI-side state for the currently selected target, independent of commands."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.previous: BlackBlock | None = None
        self.aligned = False

    @staticmethod
    def _same_target(previous: BlackBlock, current: BlackBlock, tolerance_px: int) -> bool:
        # A nearby moving component can cross the center between sampled frames;
        # a jump to a separate largest component starts a fresh alignment state.
        x_gap = max(0, current.x - (previous.x + previous.width),
                    previous.x - (current.x + current.width))
        y_gap = max(0, current.y - (previous.y + previous.height),
                    previous.y - (current.y + current.height))
        x_allowance = max(2 * tolerance_px, min(previous.width, current.width) / 2)
        y_allowance = max(2 * tolerance_px, min(previous.height, current.height) / 2)
        area_ratio = max(previous.area_px, current.area_px) / min(previous.area_px, current.area_px)
        return x_gap <= x_allowance and y_gap <= y_allowance and area_ratio <= 3

    def update(self, block: BlackBlock | None, image_width: int, tolerance_px: int) -> bool:
        if block is None:
            self.reset()
            return False
        if image_width < 1 or tolerance_px < 0:
            raise ValueError("对齐参数无效")
        previous = self.previous
        same_target = previous is not None and self._same_target(previous, block, tolerance_px)
        if same_target and block.other_centers:
            selected_distance = ((previous.center_x - block.center_x) ** 2 +
                                 (previous.center_y - block.center_y) ** 2)
            if any((previous.center_x - x) ** 2 + (previous.center_y - y) ** 2 < selected_distance
                   for x, y in block.other_centers):
                same_target = False  # Old selected block is still present, but no longer largest.
        center = image_width / 2
        near = abs(block.center_x - center) <= tolerance_px
        crossed = (same_target and
                   (previous.center_x - center) * (block.center_x - center) < 0)
        if not same_target:
            self.aligned = near  # New target cannot inherit the previous target's state.
        elif self.aligned:
            self.aligned = block.touches_center_line  # Release as soon as its black pixels leave.
        else:
            entered_tolerance = near and abs(previous.center_x - center) > tolerance_px
            self.aligned = bool(crossed or entered_tolerance)
        self.previous = block
        return self.aligned


def qimage_gray(image: QImage) -> np.ndarray:
    gray = image.convertToFormat(QImage.Format.Format_Grayscale8)
    if gray.isNull():
        raise ValueError("视频帧无法转换为灰度图")
    pixels = np.frombuffer(gray.bits(), dtype=np.uint8)
    return pixels.reshape(gray.height(), gray.bytesPerLine())[:, :gray.width()].copy()


def diagnostic_mask(gray: np.ndarray, config: BlackBlockConfig) -> QImage:
    """White segmented pixels; green outlines are area-qualified candidates."""
    left, right = roi_pixel_bounds(gray.shape[1])
    region = (left, 0, right, gray.shape[0])
    candidates, binary = _find_candidates(gray, config, region, reject_clipped=True)
    full = np.zeros_like(gray)
    full[:, left:right] = binary
    rgb = cv2.cvtColor(full, cv2.COLOR_GRAY2RGB)
    for _, contour in candidates:
        shifted = contour + np.array([[[left, 0]]], dtype=np.int32)
        cv2.drawContours(rgb, [shifted], -1, (0, 255, 0), 2)
    return QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format.Format_RGB888).copy()


class BlackBlockDetectionService(QObject):
    """One worker; pending input and completed output each hold at most one frame."""
    result_ready = Signal(object, object)  # VideoFrame, BlackBlockObservation
    error = Signal(str)
    _notify = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._generation = 0
        self._pending: tuple[int, VideoFrame, BlackBlockConfig, bool] | None = None
        self._completed: tuple[int, VideoFrame, BlackBlockObservation] | None = None
        self._error: str | None = None
        self._notify.connect(self.poll, Qt.ConnectionType.QueuedConnection)
        self._worker = threading.Thread(target=self._run, name="black-block-detection", daemon=True)
        self._worker.start()
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.setInterval(12)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    def submit(self, frame: VideoFrame, config: BlackBlockConfig, diagnostic: bool = False) -> None:
        with self._lock:
            self._pending = (self._generation, frame, config, diagnostic)
        self._wake.set()

    def clear(self) -> None:
        with self._lock:
            self._generation += 1
            self._pending = None
            self._completed = None
            self._error = None

    def _run(self) -> None:
        tracker = BlackBlockTracker()
        tracker_generation = -1
        last_frame_id: int | None = None
        while not self._stop.is_set():
            self._wake.wait(0.2)
            self._wake.clear()
            if self._stop.is_set():
                break
            with self._lock:
                pending, self._pending = self._pending, None
            if pending is None:
                continue
            generation, frame, config, diagnostic = pending
            if generation != tracker_generation:
                tracker.reset()
                tracker_generation = generation
                last_frame_id = None
            if frame.frame_id == last_frame_id:
                continue
            try:
                gray = qimage_gray(frame.image)
                observation = tracker.process(gray, config)
                if observation is None:
                    continue
                if diagnostic:
                    observation = replace(observation, diagnostic_image=diagnostic_mask(gray, config))
                last_frame_id = frame.frame_id
                with self._lock:
                    if generation == self._generation:
                        self._completed = (generation, frame, observation)
                        notify = True
                    else:
                        notify = False
                if notify:
                    self._notify.emit()
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._error = f"黑块识别失败：{exc}"
                        notify = True
                    else:
                        notify = False
                if notify:
                    self._notify.emit()

    def poll(self) -> None:
        with self._lock:
            completed, self._completed = self._completed, None
            error, self._error = self._error, None
            generation = self._generation
        if error:
            self.error.emit(error)
        if completed is not None and completed[0] == generation:
            self.result_ready.emit(completed[1], completed[2])

    def shutdown(self) -> bool:
        self.timer.stop()
        self.clear()
        self._stop.set()
        self._wake.set()
        self._worker.join(timeout=1.0)
        return not self._worker.is_alive()

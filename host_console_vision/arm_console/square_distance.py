"""Square-face geometry and per-camera monocular axial-distance calibration."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class SquareDistanceConfig:
    side_mm: float = 35.0
    polygon_epsilon_fraction: float = 0.018
    max_side_ratio: float = 1.16
    max_diagonal_ratio: float = 1.12
    max_right_angle_cosine: float = 0.14
    min_contour_quad_ratio: float = 0.92
    min_edge_px: float = 12.0
    calibration_frames: int = 15
    verification_frames: int = 7
    max_side_spread_fraction: float = 0.06
    collection_timeout_s: float = 15.0
    median_window: int = 3


SQUARE_DISTANCE_CONFIG = SquareDistanceConfig()


def square_side_from_contour(contour: np.ndarray, roi_shape: tuple[int, int],
                             config: SquareDistanceConfig = SQUARE_DISTANCE_CONFIG) -> float | None:
    """Return original-frame pixel edge length only for a complete near-fronto-parallel square."""
    height, width = roi_shape
    if contour is None or len(contour) < 4 or height < 1 or width < 1:
        return None
    x, y, box_width, box_height = cv2.boundingRect(contour)
    if x <= 0 or y <= 0 or x + box_width >= width or y + box_height >= height:
        return None  # ROI/frame clipping makes the physical face size unknowable.
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        return None
    corners = cv2.approxPolyDP(contour, config.polygon_epsilon_fraction * perimeter, True)
    if len(corners) != 4 or not cv2.isContourConvex(corners):
        return None
    area = cv2.contourArea(contour)
    quad_area = cv2.contourArea(corners)
    if (quad_area <= 0 or area / quad_area < config.min_contour_quad_ratio or
            area / quad_area > 1 / config.min_contour_quad_ratio):
        return None  # Reject missing corners / visible side faces / concave notches.
    points = corners.reshape(4, 2).astype(np.float64)
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    if np.min(lengths) < config.min_edge_px or np.max(lengths) / np.min(lengths) > config.max_side_ratio:
        return None
    diagonals = (np.linalg.norm(points[0] - points[2]),
                 np.linalg.norm(points[1] - points[3]))
    if min(diagonals) <= 0 or max(diagonals) / min(diagonals) > config.max_diagonal_ratio:
        return None
    for index in range(4):
        incoming = points[index - 1] - points[index]
        outgoing = points[(index + 1) % 4] - points[index]
        cosine = abs(float(np.dot(incoming, outgoing) /
                           (np.linalg.norm(incoming) * np.linalg.norm(outgoing))))
        if cosine > config.max_right_angle_cosine:
            return None
    return float(np.median(lengths))


def video_device_identity(device) -> str:
    """Prefer DirectShow path; use index only when the driver provides no stable path."""
    if device.path:
        return f"dshow:{device.backend}:{device.path.casefold()}"
    return f"dshow:{device.backend}:{device.name.casefold()}:{device.index}"


@dataclass(frozen=True)
class SquareCalibration:
    device_identity: str
    width: int
    height: int
    side_mm: float
    reference_distance_mm: float
    reference_side_px: float
    focal_px: float
    saved_at: str

    def distance_cm(self, side_px: float) -> float:
        if side_px <= 0:
            raise ValueError("目标像素边长必须为正")
        return self.focal_px * self.side_mm / side_px / 10.0


def default_calibration_path() -> Path:
    return Path(__file__).resolve().parents[1] / "calibrations" / "square_calibrations.json"


class SquareCalibrationStore:
    def __init__(self, path: Path | None = None):
        self.path = path or default_calibration_path()

    def get(self, identity: str, width: int, height: int,
            config: SquareDistanceConfig = SQUARE_DISTANCE_CONFIG) -> SquareCalibration | None:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            for item in payload.get("calibrations", []):
                candidate = SquareCalibration(**item)
                if (candidate.device_identity == identity and candidate.width == width and
                        candidate.height == height and candidate.side_mm == config.side_mm and
                        candidate.focal_px > 0 and candidate.reference_side_px > 0 and
                        candidate.reference_distance_mm > 0):
                    return candidate
        except (OSError, ValueError, TypeError, KeyError, AttributeError):
            return None
        return None

    def save(self, calibration: SquareCalibration) -> None:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
            items = payload.get("calibrations", [])
            if not isinstance(items, list):
                items = []
        except (OSError, ValueError, TypeError, AttributeError):
            items = []
        items = [item for item in items if not (
            isinstance(item, dict) and item.get("device_identity") == calibration.device_identity and
            item.get("width") == calibration.width and item.get("height") == calibration.height)]
        items.append(asdict(calibration))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        temporary.write_text(json.dumps({"version": 1, "calibrations": items}, ensure_ascii=False, indent=2),
                             encoding="utf-8")
        temporary.replace(self.path)


@dataclass(frozen=True)
class RangeDisplay:
    current_cm: float | None
    text: str
    calibration_text: str
    verification_text: str


@dataclass
class _Collection:
    kind: str
    reference_cm: float
    target_id: int
    started: float
    sides: list[float]
    last_frame_id: int | None = None


class SquareRangeManager:
    """UI-thread calibration and measurement state; never issues control commands."""

    def __init__(self, store: SquareCalibrationStore | None = None,
                 config: SquareDistanceConfig = SQUARE_DISTANCE_CONFIG):
        self.store = store or SquareCalibrationStore()
        self.config = config
        self.context: tuple[str, int, int] | None = None
        self.calibration: SquareCalibration | None = None
        self.reset_runtime(clear_context=False)

    def reset_runtime(self, *, clear_context: bool = False) -> None:
        self.collection: _Collection | None = None
        self.target_id: int | None = None
        self.latest_square_side: float | None = None
        self.last_measurement: tuple[float, str] | None = None
        self.recent_cm: deque[float] = deque(maxlen=self.config.median_window)
        self.verification_text = "验证：尚未进行"
        self.collection_text = ""
        if clear_context:
            self.context = None
            self.calibration = None

    def set_context(self, identity: str, width: int, height: int) -> None:
        context = (identity, width, height)
        if context != self.context:
            self.reset_runtime(clear_context=False)
            self.context = context
            self.calibration = self.store.get(identity, width, height, self.config)

    def _calibration_text(self) -> str:
        if self.context is None:
            return "标定：请先打开视频"
        if self.collection is not None:
            name = "标定" if self.collection.kind == "calibration" else "验证"
            required = (self.config.calibration_frames if self.collection.kind == "calibration"
                        else self.config.verification_frames)
            return f"{name}采集中：{len(self.collection.sides)}/{required} 帧"
        if self.collection_text:
            return self.collection_text
        if self.calibration is None:
            return "标定：当前视频设备与分辨率尚未标定"
        return (f"标定：已加载 · {self.context[1]}×{self.context[2]} · "
                f"参考 {self.calibration.reference_distance_mm / 10:.1f} cm / "
                f"{self.calibration.reference_side_px:.1f} px")

    def begin(self, kind: str, reference_cm: float) -> str:
        if kind not in ("calibration", "verification") or not math.isfinite(reference_cm) or reference_cm <= 0:
            return "请输入有效的相机至目标正面距离"
        if self.context is None or self.target_id is None or self.latest_square_side is None:
            return f"请先打开视频并让完整 {self.config.side_mm:g} mm 正方形正对相机"
        if kind == "verification" and self.calibration is None:
            return "当前设备与分辨率尚未标定，无法验证"
        self.collection = _Collection(kind, reference_cm, self.target_id, time.monotonic(), [])
        self.collection_text = ""
        if kind == "verification":
            self.verification_text = "验证：采集中"
        return f"{'标定' if kind == 'calibration' else '验证'}已开始，保持目标静止并正对相机"

    def observe(self, frame_id: int, observation) -> RangeDisplay:
        if self.collection is not None and time.monotonic() - self.collection.started > self.config.collection_timeout_s:
            self.collection = None
            self.collection_text = "标定/验证超时：请保持目标正对并重试"
        block = observation.blocks[0] if observation.blocks else None
        if block is None:
            self.latest_square_side = None
            if observation.phase == "SEARCH" or observation.missed_frames >= observation.miss_limit:
                self.target_id = None
                self.last_measurement = None
                self.recent_cm.clear()
                self.collection = None
                return self._display(None, "距离不可用/未检测到锁定目标")
            if self.last_measurement is not None:
                value, stamp = self.last_measurement
                return self._display(None, f"上次测量：{value:.1f} cm（上位机处理时间 {stamp}；本帧暂时丢失）")
            return self._display(None, "距离不可用/暂时丢失")

        if observation.target_id != self.target_id:
            self.target_id = observation.target_id
            self.last_measurement = None
            self.recent_cm.clear()
            if self.collection is not None:
                self.collection = None
                self.collection_text = "标定/验证已取消：锁定目标改变"
        side = block.square_side_px
        self.latest_square_side = side
        if side is None:
            self.last_measurement = None
            self.recent_cm.clear()
            return self._display(None, "距离不可用/姿态不符合（需完整、近似正对的正方形）")

        if self.collection is not None:
            collection = self.collection
            if time.monotonic() - collection.started > self.config.collection_timeout_s:
                self.collection = None
                self.collection_text = "标定/验证超时：请保持目标正对并重试"
            elif frame_id != collection.last_frame_id:
                collection.sides.append(side)
                collection.last_frame_id = frame_id
                required = (self.config.calibration_frames if collection.kind == "calibration"
                            else self.config.verification_frames)
                if len(collection.sides) >= required:
                    self._finish_collection(collection)

        if self.calibration is None:
            return self._display(None, f"距离不可用/未标定 · 原始像素边长 {side:.1f} px")
        raw_cm = self.calibration.distance_cm(side)
        self.recent_cm.append(raw_cm)
        shown_cm = statistics.median(self.recent_cm)
        self.last_measurement = (shown_cm, datetime.now().strftime("%H:%M:%S"))
        return self._display(shown_cm, f"相机轴向距离：{shown_cm:.1f} cm（估计） · "
                             f"原始测值 {raw_cm:.1f} cm · 原图边长 {side:.1f} px")

    def _finish_collection(self, collection: _Collection) -> None:
        median_side = statistics.median(collection.sides)
        if (max(collection.sides) - min(collection.sides)) / median_side > self.config.max_side_spread_fraction:
            self.collection_text = "标定/验证失败：目标像素边长波动过大，请固定目标重试"
        elif collection.kind == "calibration":
            identity, width, height = self.context
            calibration = SquareCalibration(
                identity, width, height, self.config.side_mm, collection.reference_cm * 10,
                median_side, median_side * collection.reference_cm * 10 / self.config.side_mm,
                datetime.now().isoformat(timespec="seconds"))
            try:
                self.store.save(calibration)
            except OSError as exc:
                self.collection_text = f"标定未保存：{exc}"
            else:
                self.calibration = calibration
                self.recent_cm.clear()
                self.collection_text = f"标定已保存：{median_side:.1f} px @ {collection.reference_cm:.1f} cm"
        else:
            estimated_cm = self.calibration.distance_cm(median_side)
            error_cm = estimated_cm - collection.reference_cm
            error_percent = error_cm / collection.reference_cm * 100
            self.verification_text = (f"验证：实测 {collection.reference_cm:.1f} cm · 估计 {estimated_cm:.1f} cm · "
                                      f"误差 {error_cm:+.1f} cm（{error_percent:+.1f}%）")
            self.collection_text = "验证完成"
        self.collection = None

    def _display(self, current_cm: float | None, text: str) -> RangeDisplay:
        return RangeDisplay(current_cm, text, self._calibration_text(), self.verification_text)

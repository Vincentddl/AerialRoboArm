"""YOLO11s-seg foam-center detection + 2.1 mm fisheye bearing & empirical LUT mapping."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import threading
import time

import cv2
import numpy as np
from PySide6.QtCore import QObject, QTimer, Qt, Signal
from PySide6.QtGui import QImage

from .models import BlackBlock
from .video_capture import VideoFrame

CONSOLE_ROOT = Path(__file__).resolve().parent.parent
ASSETS_DIR = CONSOLE_ROOT / "assets"
VISION_CORE_ROOT = Path(r"D:\Study_data\AerialRoboArm-Vision-Core")

DEFAULT_MODEL_NAME = "black_block_yolo26n_seg_rig_v1.pt"
FALLBACK_MODEL_NAME = "foam_center_v9_gripper_axis_normal_bg03_20260816_candidate.pt"
DEFAULT_FISHEYE_NAME = "camera_2p1mm_640x480_fisheye.json"
DEFAULT_LUT_NAME = "servo_to_optical_angle_foam_center_lut_20260816_v1.json"

# Under the current +90° normal working pose, the gripper tips rest near y ≈ 435..445
# (optical_offset ≈ +19° -> extrapolated LUT servo ≈ -38.5°).
DEFAULT_LUT_ZERO_SERVO_DEG = -38.5


def resolve_vision_asset(filename: str) -> Path:
    names = [filename]
    if filename == DEFAULT_MODEL_NAME:
        names.append(FALLBACK_MODEL_NAME)
    for name in names:
        candidates = [
            ASSETS_DIR / name,
            VISION_CORE_ROOT / "models" / name,
            VISION_CORE_ROOT / "configs" / name,
        ]
        for path in candidates:
            if path.is_file():
                return path
    raise FileNotFoundError(f"未找到视觉资产文件：{filename}")


def qimage_bgr(image: QImage) -> np.ndarray:
    rgb_img = image.convertToFormat(QImage.Format.Format_RGB888)
    if rgb_img.isNull():
        raise ValueError("视频帧无法转换为 RGB888 图像")
    raw = np.frombuffer(rgb_img.bits(), dtype=np.uint8)
    rgb = raw.reshape(rgb_img.height(), rgb_img.bytesPerLine())[:, : rgb_img.width() * 3]
    rgb = rgb.reshape(rgb_img.height(), rgb_img.width(), 3)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


@dataclass(frozen=True)
class FisheyeBearing:
    lateral_angle_deg: float
    optical_offset_deg: float
    lut_servo_deg: float


class FisheyeLutMapper:
    """2.1 mm fisheye ray undistortion + piecewise-linear servo-to-optical LUT."""

    def __init__(self, fisheye_path: Path | None = None, lut_path: Path | None = None):
        fisheye_file = fisheye_path or resolve_vision_asset(DEFAULT_FISHEYE_NAME)
        lut_file = lut_path or resolve_vision_asset(DEFAULT_LUT_NAME)
        fisheye_data = json.loads(fisheye_file.read_text(encoding="utf-8"))
        lut_data = json.loads(lut_file.read_text(encoding="utf-8"))

        if fisheye_data.get("model") != "fisheye":
            raise ValueError(f"不支持的相机模型：{fisheye_data.get('model')}")
        size = fisheye_data.get("image_size") or [640, 480]
        self.calib_width = int(size[0])
        self.calib_height = int(size[1])
        self.camera_matrix = np.asarray(fisheye_data["camera_matrix"], dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.asarray(fisheye_data["dist_coeffs"], dtype=np.float64).reshape(4, 1)

        raw_points = lut_data["model"]["points"]
        pairs = sorted(
            ((float(p["optical_offset_deg"]), float(p["servo_command_deg"])) for p in raw_points),
            key=lambda item: item[0],
        )
        if len(pairs) < 2:
            raise ValueError("鱼眼 LUT 标定点不足")
        self.optical_nodes = [item[0] for item in pairs]
        self.servo_nodes = [item[1] for item in pairs]

    def lut_servo_from_optical_offset(self, optical_offset_deg: float) -> float:
        xs = self.optical_nodes
        ys = self.servo_nodes
        x = float(optical_offset_deg)
        if x <= xs[0]:
            slope = (ys[1] - ys[0]) / (xs[1] - xs[0])
            return ys[0] + slope * (x - xs[0])
        if x >= xs[-1]:
            slope = (ys[-1] - ys[-2]) / (xs[-1] - xs[-2])
            return ys[-1] + slope * (x - xs[-1])
        for idx in range(len(xs) - 1):
            x0, x1 = xs[idx], xs[idx + 1]
            if x0 <= x <= x1:
                ratio = (x - x0) / (x1 - x0) if x1 != x0 else 0.0
                return ys[idx] + ratio * (ys[idx + 1] - ys[idx])
        return ys[-1]

    def pixel_to_bearing(self, pixel_x: float, pixel_y: float,
                         frame_width: int, frame_height: int) -> FisheyeBearing:
        scale_x = self.calib_width / max(1, int(frame_width))
        scale_y = self.calib_height / max(1, int(frame_height))
        pt = np.array([[[float(pixel_x) * scale_x, float(pixel_y) * scale_y]]], dtype=np.float64)
        undistorted = cv2.fisheye.undistortPoints(pt, self.camera_matrix, self.dist_coeffs)
        xn = float(undistorted[0, 0, 0])
        yn = float(undistorted[0, 0, 1])
        lateral_deg = math.degrees(math.atan(xn))
        optical_offset_deg = math.degrees(math.atan(yn))
        lut_servo_deg = self.lut_servo_from_optical_offset(optical_offset_deg)
        return FisheyeBearing(lateral_deg, optical_offset_deg, lut_servo_deg)

    def pixel_to_extension_deg(
        self,
        pixel_x: float,
        pixel_y: float,
        frame_width: int,
        frame_height: int,
        *,
        min_ext_deg: float = 0.0,
        max_ext_deg: float = 70.0,
        angle_trim_deg: float = 0.0,
        invert: bool = False,
        lut_zero_servo_deg: float = DEFAULT_LUT_ZERO_SERVO_DEG,
    ) -> tuple[float, FisheyeBearing]:
        bearing = self.pixel_to_bearing(pixel_x, pixel_y, frame_width, frame_height)
        low = min(float(min_ext_deg), float(max_ext_deg))
        high = max(float(min_ext_deg), float(max_ext_deg))
        raw_ext = (float(lut_zero_servo_deg) - bearing.lut_servo_deg) + float(angle_trim_deg)
        if invert:
            raw_ext = (low + high) - raw_ext
        clamped = max(low, min(high, raw_ext))
        return clamped, bearing


@dataclass(frozen=True)
class FoamVisionConfig:
    conf_threshold: float = 0.35
    min_area_ratio: float = 0.0012
    max_area_ratio: float = 0.68    # Supports holding block right in front of lens ('放跟前' up to 68% frame)
    min_aspect_ratio: float = 0.22
    max_aspect_ratio: float = 3.80
    mask_centroid_min_area_ratio: float = 0.0012
    mask_centroid_max_area_ratio: float = 0.68  # True mask area centroid across ALL scales
    central_roi_only: bool = True
    center_half_width_px: int = 75  # Covers x=[245..395] (arm axis ~307px) while blocking x<215 doorway
    require_dark_foam: bool = True
    max_foam_gray: float = 105.0    # p25/median dark-surface check resilient to close-up lens glare
    min_ring_contrast: float = 14.0


def verify_dark_foam_appearance(
    gray_frame: np.ndarray,
    bbox_xyxy: tuple[float, float, float, float],
    contour_pts: tuple[tuple[int, int], ...],
    *,
    max_foam_gray: float = 105.0,
    min_ring_contrast: float = 14.0,
) -> bool:
    """Reject bright/uniform background false positives while keeping glare-lit close-up black blocks."""
    h, w = gray_frame.shape[:2]
    x1 = max(0, min(w - 1, int(round(bbox_xyxy[0]))))
    y1 = max(0, min(h - 1, int(round(bbox_xyxy[1]))))
    x2 = max(x1 + 1, min(w, int(round(bbox_xyxy[2]))))
    y2 = max(y1 + 1, min(h, int(round(bbox_xyxy[3]))))
    box_area_ratio = ((x2 - x1) * (y2 - y1)) / float(max(1, w * h))
    pad = 14
    rx1 = max(0, x1 - pad)
    ry1 = max(0, y1 - pad)
    rx2 = min(w, x2 + pad)
    ry2 = min(h, y2 + pad)
    patch = gray_frame[ry1:ry2, rx1:rx2]
    if patch.size == 0:
        return False
    local_mask = np.zeros(patch.shape, dtype=np.uint8)
    if len(contour_pts) >= 3:
        pts = np.array([[[px - rx1, py - ry1]] for px, py in contour_pts], dtype=np.int32)
        cv2.fillPoly(local_mask, [pts], 255)
    if int(np.count_nonzero(local_mask)) < 12:
        local_mask[y1 - ry1 : y2 - ry1, x1 - rx1 : x2 - rx1] = 255
    fg_pixels = patch[local_mask > 0]
    if fg_pixels.size == 0:
        return False
    fg_p25 = float(np.percentile(fg_pixels, 25))
    fg_med = float(np.median(fg_pixels))
    # Even under strong close-up glare, the darker half of a black foam block stays dark
    if fg_p25 > max_foam_gray or fg_med > max_foam_gray + 18.0:
        return False
    # For small/medium objects (<18% frame), require local contrast against outer ring to reject flat wall shadows;
    # for huge close-up objects ('放跟前' >=18% frame), skip ring check because the block fills the central view.
    if box_area_ratio < 0.18:
        dilated = cv2.dilate(local_mask, np.ones((13, 13), np.uint8))
        ring_pixels = patch[(dilated > 0) & (local_mask == 0)]
        if ring_pixels.size >= 16:
            ring_hi = float(np.percentile(ring_pixels, 70))
            if ring_hi - fg_p25 < min_ring_contrast:
                return False
    return True


@dataclass(frozen=True)
class FoamDetection:
    block: BlackBlock
    confidence: float
    center_source: str  # "mask_centroid" or "box_center"
    contour_points: tuple[tuple[int, int], ...] = ()


@dataclass(frozen=True)
class FoamObservation:
    detection: FoamDetection | None
    inference_ms: float
    device_label: str
    missed_frames: int


def compute_hybrid_foam_center(
    bbox_xyxy: tuple[float, float, float, float],
    mask_2d: np.ndarray | None,
    frame_width: int,
    frame_height: int,
    *,
    min_mask_ratio: float = 0.01,
    max_mask_ratio: float = 0.13,
) -> tuple[float, float, str, float, tuple[tuple[int, int], ...]]:
    """Vision-Core hybrid rule: mask contour centroid for 1%-13% box area, else box center."""
    x1, y1, x2, y2 = bbox_xyxy
    bw = max(1.0, x2 - x1)
    bh = max(1.0, y2 - y1)
    box_area = bw * bh
    frame_area = float(max(1, frame_width * frame_height))
    box_ratio = box_area / frame_area
    box_cx = 0.5 * (x1 + x2)
    box_cy = 0.5 * (y1 + y2)

    contour_pts: tuple[tuple[int, int], ...] = ()
    mask_area = box_area
    if mask_2d is not None and mask_2d.size > 0:
        if mask_2d.shape[:2] != (frame_height, frame_width):
            resized = cv2.resize(
                mask_2d.astype(np.float32),
                (frame_width, frame_height),
                interpolation=cv2.INTER_LINEAR,
            )
            binary = (resized >= 0.5).astype(np.uint8) * 255
        else:
            binary = (mask_2d >= 0.5).astype(np.uint8) * 255
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(largest))
            if area >= 16.0:
                mask_area = area
                approx = cv2.approxPolyDP(largest, 2.0, True)
                contour_pts = tuple((int(pt[0][0]), int(pt[0][1])) for pt in approx)
                if min_mask_ratio <= box_ratio <= max_mask_ratio:
                    moments = cv2.moments(largest)
                    if moments["m00"] > 0:
                        cx = float(moments["m10"] / moments["m00"])
                        cy = float(moments["m01"] / moments["m00"])
                        return cx, cy, "mask_centroid", mask_area, contour_pts
    return box_cx, box_cy, "box_center", mask_area, contour_pts


class FoamYoloDetector:
    """Lazy-initialized YOLO11s-seg foam detector with GPU FP16 warmup."""

    def __init__(self, model_path: Path | None = None):
        self.model_path = model_path or resolve_vision_asset(DEFAULT_MODEL_NAME)
        self._model = None
        self._device = "cpu"
        self._use_half = False
        self.device_label = "未加载"
        self._last_center: tuple[float, float] | None = None
        self._missed = 0

    def reset_track(self) -> None:
        self._last_center = None
        self._missed = 0

    def ensure_loaded(self) -> None:
        if self._model is not None:
            return
        import torch
        from ultralytics import YOLO

        use_cuda = bool(torch.cuda.is_available())
        self._device = "0" if use_cuda else "cpu"
        self._use_half = use_cuda
        gpu_name = torch.cuda.get_device_name(0) if use_cuda else "CPU"
        self.device_label = f"GPU ({gpu_name})" if use_cuda else "CPU"
        model = YOLO(str(self.model_path))
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        model.predict(
            dummy,
            conf=0.5,
            imgsz=640,
            device=self._device,
            half=self._use_half,
            verbose=False,
        )
        self._model = model

    def detect(self, bgr: np.ndarray, config: FoamVisionConfig) -> FoamObservation:
        self.ensure_loaded()
        height, width = bgr.shape[:2]
        frame_area = float(max(1, width * height))
        t0 = time.perf_counter()
        results = self._model.predict(
            bgr,
            conf=float(config.conf_threshold),
            iou=0.70,
            max_det=6,
            imgsz=640,
            device=self._device,
            half=self._use_half,
            verbose=False,
        )
        infer_ms = (time.perf_counter() - t0) * 1000.0
        if not results:
            self._missed += 1
            return FoamObservation(None, infer_ms, self.device_label, self._missed)

        res = results[0]
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            self._missed += 1
            return FoamObservation(None, infer_ms, self.device_label, self._missed)

        xyxy_np = boxes.xyxy.detach().cpu().numpy()
        confs_np = boxes.conf.detach().cpu().numpy()
        masks_np = res.masks.data.detach().cpu().numpy() if res.masks is not None else None

        center_x_mid = width / 2.0
        half_corridor = max(10.0, float(config.center_half_width_px))
        gray_frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if config.require_dark_foam else None

        candidates: list[tuple[float, FoamDetection]] = []
        for idx in range(len(xyxy_np)):
            x1, y1, x2, y2 = (float(v) for v in xyxy_np[idx])
            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)
            area_ratio = (bw * bh) / frame_area
            aspect_ratio = bw / bh
            if area_ratio < config.min_area_ratio or area_ratio > config.max_area_ratio:
                continue
            if not (config.min_aspect_ratio <= aspect_ratio <= config.max_aspect_ratio):
                continue
            mask_2d = masks_np[idx] if masks_np is not None and idx < len(masks_np) else None
            cx, cy, source, area_px, contour_pts = compute_hybrid_foam_center(
                (x1, y1, x2, y2),
                mask_2d,
                width,
                height,
                min_mask_ratio=config.mask_centroid_min_area_ratio,
                max_mask_ratio=config.mask_centroid_max_area_ratio,
            )
            axis_offset = abs(cx - center_x_mid)
            touches_center = x1 <= center_x_mid <= x2
            # Close-up ('放跟前') large blocks are accepted whenever they span the center axis or lie inside the corridor!
            closeup_spanning_axis = area_ratio >= 0.06 and touches_center
            if config.central_roi_only and axis_offset > half_corridor and not closeup_spanning_axis:
                continue
            if gray_frame is not None and not verify_dark_foam_appearance(
                gray_frame,
                (x1, y1, x2, y2),
                contour_pts,
                max_foam_gray=config.max_foam_gray,
                min_ring_contrast=config.min_ring_contrast,
            ):
                continue
            conf = float(confs_np[idx])
            bx = max(0, int(round(x1)))
            by = max(0, int(round(y1)))
            bwidth = max(4, min(width - bx, int(round(bw))))
            bheight = max(4, min(height - by, int(round(bh))))
            touches_center = bx <= center_x_mid <= (bx + bwidth)
            poly_pts = tuple((float(px), float(py)) for px, py in contour_pts)
            block = BlackBlock(
                x=bx,
                y=by,
                width=bwidth,
                height=bheight,
                center_x=cx,
                center_y=cy,
                area_px=area_px,
                touches_center_line=touches_center,
                rotated_box=poly_pts,
            )
            det = FoamDetection(block, conf, source, contour_pts)
            axis_bonus = 0.16 * max(0.0, 1.0 - axis_offset / half_corridor)
            continuity_bonus = 0.0
            if self._last_center is not None:
                dist = math.hypot(cx - self._last_center[0], cy - self._last_center[1])
                continuity_bonus = max(0.0, 0.18 * (1.0 - dist / max(120.0, 0.35 * height)))
            candidates.append((conf + axis_bonus + continuity_bonus, det))

        if not candidates:
            self._missed += 1
            if self._missed >= 8:
                self._last_center = None
            return FoamObservation(None, infer_ms, self.device_label, self._missed)

        best = max(candidates, key=lambda item: item[0])[1]
        self._last_center = (best.block.center_x, best.block.center_y)
        self._missed = 0
        return FoamObservation(best, infer_ms, self.device_label, 0)


def yolo_available() -> bool:
    try:
        import importlib.util
        resolve_vision_asset(DEFAULT_MODEL_NAME)
        return importlib.util.find_spec("ultralytics") is not None
    except Exception:
        return False


class FoamVisionService(QObject):
    """Single-slot latest-frame worker for low-latency YOLO11s-seg foam tracking."""

    result_ready = Signal(object, object)  # VideoFrame, FoamObservation
    error = Signal(str)
    _notify = Signal()

    def __init__(self, parent=None, *, detector_factory=FoamYoloDetector):
        super().__init__(parent)
        self._detector_factory = detector_factory
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._generation = 0
        self._pending: tuple[int, VideoFrame, FoamVisionConfig] | None = None
        self._completed: tuple[int, VideoFrame, FoamObservation] | None = None
        self._error: str | None = None
        self._notify.connect(self.poll, Qt.ConnectionType.QueuedConnection)
        self._worker = threading.Thread(target=self._run, name="foam-yolo-vision", daemon=True)
        self._worker.start()
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.setInterval(12)
        self.timer.timeout.connect(self.poll)
        self.timer.start()

    def submit(self, frame: VideoFrame, config: FoamVisionConfig) -> None:
        with self._lock:
            self._pending = (self._generation, frame, config)
        self._wake.set()

    def clear(self) -> None:
        with self._lock:
            self._generation += 1
            self._pending = None
            self._completed = None
            self._error = None

    def _run(self) -> None:
        detector: FoamYoloDetector | None = None
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
            generation, frame, config = pending
            if generation != tracker_generation:
                if detector is not None:
                    detector.reset_track()
                tracker_generation = generation
                last_frame_id = None
            if frame.frame_id == last_frame_id:
                continue
            try:
                if detector is None:
                    detector = self._detector_factory()
                bgr = qimage_bgr(frame.image)
                obs = detector.detect(bgr, config)
                last_frame_id = frame.frame_id
                with self._lock:
                    if generation == self._generation:
                        self._completed = (generation, frame, obs)
                        notify = True
                    else:
                        notify = False
                if notify:
                    self._notify.emit()
            except Exception as exc:
                with self._lock:
                    if generation == self._generation:
                        self._error = f"YOLO 泡沫识别失败：{exc}"
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
        self._worker.join(timeout=2.0)
        return not self._worker.is_alive()

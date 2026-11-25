"""Single-point 35 mm sphere calibration, independent of legacy square calibration."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
import statistics

from .square_distance import video_device_identity


@dataclass(frozen=True)
class SphereConfig:
    diameter_mm: float = 35.0
    calibration_frames: int = 12
    verification_frames: int = 5
    median_window: int = 3


SPHERE_CONFIG = SphereConfig()


@dataclass(frozen=True)
class SphereCalibration:
    identity: str
    width: int
    height: int
    diameter_mm: float
    reference_cm: float
    reference_px: float
    saved_at: str

    def estimate_cm(self, pixels: float) -> float:
        return self.reference_cm * self.reference_px / pixels


@dataclass(frozen=True)
class RangeDisplay:
    current_cm: float | None
    text: str
    calibration_text: str
    verification_text: str


class SphereRangeManager:
    def __init__(self, path: Path | None = None, config: SphereConfig = SPHERE_CONFIG):
        self.path = path or Path(__file__).resolve().parents[1] / "calibrations" / "sphere_calibrations.json"
        self.config = config
        self.context = None
        self.calibration = None
        self.reset_runtime(clear_context=False)

    def reset_runtime(self, *, clear_context: bool = False) -> None:
        self.target_id = None
        self.latest_px = None
        self.last_measurement = None
        self.recent = deque(maxlen=self.config.median_window)
        self.collection = None
        self.verification_text = "验证：尚未进行"
        self.collection_text = ""
        if clear_context:
            self.context = None
            self.calibration = None

    def set_context(self, identity: str, width: int, height: int) -> None:
        context = (identity, width, height)
        if context == self.context:
            return
        self.reset_runtime()
        self.context = context
        self.calibration = None
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            for item in data.get("calibrations", []):
                c = SphereCalibration(**item)
                if (c.identity, c.width, c.height, c.diameter_mm) == (*context, self.config.diameter_mm):
                    self.calibration = c
                    break
        except (OSError, ValueError, TypeError, KeyError):
            pass

    def begin(self, kind: str, reference_cm: float) -> str:
        if kind not in ("calibration", "verification") or not 0 < reference_cm <= 1000:
            return "请输入有效的相机至球心距离（cm）"
        if self.context is None or self.latest_px is None or self.target_id is None:
            return "请先打开视频并稳定识别 35 mm 黑球"
        if kind == "verification" and self.calibration is None:
            return "请先完成一次球体标定"
        self.collection = [kind, float(reference_cm), self.target_id, [], None]
        self.collection_text = ""
        return f"{'标定' if kind == 'calibration' else '验证'}采集中：保持球体与相机静止"

    def observe(self, frame_id: int, observation) -> RangeDisplay:
        block = observation.blocks[0] if observation.blocks else None
        if block is None:
            self.latest_px = None
            if observation.phase == "SEARCH" or observation.missed_frames >= observation.miss_limit:
                self.target_id = None
                self.last_measurement = None
                self.recent.clear()
                self.collection = None
            if self.last_measurement:
                distance, stamp = self.last_measurement
                return self._display(None, f"上次距离：{distance:.1f} cm（{stamp}；本帧漏检）")
            return self._display(None, "待检测黑球")
        if observation.target_id != self.target_id:
            self.target_id = observation.target_id
            self.last_measurement = None
            self.recent.clear()
            self.collection = None
        px = block.circle_diameter_px
        self.latest_px = px if block.diameter_usable else None
        if self.latest_px is None:
            return self._display(None, f"像素直径 {px:.1f} px；距离不可用（轮廓截断或质量不足）" if px else "距离不可用")
        if self.collection and self.collection[2] == self.target_id and frame_id != self.collection[4]:
            self.collection[3].append(px)
            self.collection[4] = frame_id
            needed = self.config.calibration_frames if self.collection[0] == "calibration" else self.config.verification_frames
            if len(self.collection[3]) >= needed:
                self._finish()
        if self.calibration is None:
            return self._display(None, f"待标定 · 原图像素直径 {px:.1f} px")
        raw = self.calibration.estimate_cm(px)
        self.recent.append(raw)
        shown = statistics.median(self.recent)
        stamp = datetime.now().strftime("%H:%M:%S")
        self.last_measurement = (shown, stamp)
        return self._display(shown, f"相机至球心估计距离：{shown:.1f} cm · 原始 {raw:.1f} cm · {px:.1f} px")

    def _finish(self) -> None:
        kind, reference_cm, _, samples, _ = self.collection
        median_px = statistics.median(samples)
        if kind == "calibration":
            self.calibration = SphereCalibration(*self.context, self.config.diameter_mm,
                                                 reference_cm, median_px, datetime.now().isoformat(timespec="seconds"))
            try:
                try:
                    data = json.loads(self.path.read_text(encoding="utf-8"))
                    items = data.get("calibrations", [])
                except (OSError, ValueError, TypeError):
                    items = []
                items = [i for i in items if (i.get("identity"), i.get("width"), i.get("height")) != self.context]
                items.append(asdict(self.calibration))
                self.path.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.path.with_suffix(".tmp")
                tmp.write_text(json.dumps({"version": 1, "calibrations": items}, ensure_ascii=False, indent=2), encoding="utf-8")
                tmp.replace(self.path)
                self.collection_text = f"球体标定已保存：{median_px:.1f} px @ {reference_cm:.1f} cm"
            except OSError as exc:
                self.calibration = None
                self.collection_text = f"标定保存失败：{exc}"
        else:
            estimate = self.calibration.estimate_cm(median_px)
            error = estimate - reference_cm
            self.verification_text = f"验证：实测 {reference_cm:.1f} cm，估计 {estimate:.1f} cm，误差 {error:+.1f} cm（{error / reference_cm * 100:+.1f}%）"
        self.collection = None

    def _display(self, current: float | None, text: str) -> RangeDisplay:
        if self.collection:
            kind, _, _, samples, _ = self.collection
            total = self.config.calibration_frames if kind == "calibration" else self.config.verification_frames
            status = f"{'标定' if kind == 'calibration' else '验证'}采集中：{len(samples)}/{total} 帧"
        elif self.collection_text:
            status = self.collection_text
        elif self.calibration:
            status = f"球体标定已加载：{self.calibration.reference_px:.1f} px @ {self.calibration.reference_cm:.1f} cm"
        else:
            status = "球体测距：待标定"
        return RangeDisplay(current, text, status, self.verification_text)

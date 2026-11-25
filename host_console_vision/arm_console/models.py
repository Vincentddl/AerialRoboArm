from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class AccelerationSample:
    time_s: float
    ax: float
    ay: float
    az: float
    unit: str
    source: str
    time_source: str  # "设备时间" 或 "接收时间"


@dataclass(frozen=True)
class MotionRequest:
    action: str  # fold, restore, extend, grab, follow
    speed_percent: float | None = None
    angle_deg: float | None = None


class ConnectionState(str, Enum):
    DISCONNECTED = "未连接"
    CONNECTING = "连接中"
    OPEN = "串口已打开"
    DISCONNECTING = "断开中"
    FAILED = "连接失败"


@dataclass(frozen=True)
class Detection:
    target_id: str
    category: str
    confidence: float
    u: float
    v: float
    box: tuple[float, float, float, float]


@dataclass(frozen=True)
class BlackBlock:
    x: int
    y: int
    width: int
    height: int
    center_x: float
    center_y: float
    area_px: float
    touches_center_line: bool = False  # Selected binary component intersects x=W/2.
    other_centers: tuple[tuple[float, float], ...] = ()  # Qualified but unshown contours.
    square_side_px: float | None = None  # Four-corner face edge on the original image.
    circle_diameter_px: float | None = None
    diameter_usable: bool = False
    short_side_px: float | None = None
    width_usable: bool = False
    rotated_box: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True)
class VisionResult:
    width: int
    height: int
    detection: Detection | None
    source: str
    simulated: bool
    black_blocks: tuple[BlackBlock, ...] | None = None  # None=未运行黑块识别，()=已识别但无目标
    frame_id: int | None = None
    roi_overlay: bool = False
    black_block_aligned: bool = False
    tracking_phase: str | None = None
    tracking_misses: int = 0
    tracking_miss_limit: int = 20
    distance_cm: float | None = None
    center_segments_overlay: bool = False
    corridor_half_width_px: int | None = None

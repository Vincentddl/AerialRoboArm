"""Train YOLO26n-seg on black_block_rig_v1 and regenerate the 17-point LUT using its exact mask centroid."""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import shutil
import statistics

import cv2
import numpy as np
from ultralytics import YOLO

CONSOLE_ROOT = Path(__file__).resolve().parents[1]
DATASET_DIR = CONSOLE_ROOT / "runtime" / "datasets" / "black_block_rig_v1"
DATASET_YAML = DATASET_DIR / "dataset.yaml"
RUNS_DIR = CONSOLE_ROOT / "runtime" / "yolo26_runs"
ASSETS_DIR = CONSOLE_ROOT / "assets"
OUTPUT_MODEL_PATH = ASSETS_DIR / "black_block_yolo26n_seg_rig_v1.pt"

SWEEP_DIR = CONSOLE_ROOT / "runtime" / "position_recordings" / "sweep_20261123_200109"
VIDEO_PATH = SWEEP_DIR / "sweep_20261123_200109.avi"
EVENTS_PATH = SWEEP_DIR / "sweep_20261123_200109.angles.jsonl"
ACTIVE_LUT_PATH = CONSOLE_ROOT / "runtime" / "calibrated_position_lut.json"


def mask_centroid_from_result(res, width: int, height: int, *, center_corridor_px: float = 105.0, min_conf: float = 0.45):
    """Extract true visible mask area centroid (works from 0.15% tiny far block up to 65% close-up block)."""
    if res.boxes is None or len(res.boxes) == 0:
        return None
    xyxy = res.boxes.xyxy.detach().cpu().numpy()
    confs = res.boxes.conf.detach().cpu().numpy()
    masks = res.masks.data.detach().cpu().numpy() if res.masks is not None else None
    frame_area = float(max(1, width * height))
    mid_x = width / 2.0

    best = None
    best_score = -1.0
    for i in range(len(xyxy)):
        conf = float(confs[i])
        if conf < min_conf:
            continue
        x1, y1, x2, y2 = (float(v) for v in xyxy[i])
        bw = max(1.0, x2 - x1)
        bh = max(1.0, y2 - y1)
        area_ratio = (bw * bh) / frame_area
        if not (0.0012 <= area_ratio <= 0.68):
            continue
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        source = "box_center"
        if masks is not None and i < len(masks):
            m2d = masks[i]
            if m2d.shape[:2] != (height, width):
                m2d = cv2.resize(m2d.astype(np.float32), (width, height), interpolation=cv2.INTER_LINEAR)
            binary = (m2d >= 0.5).astype(np.uint8) * 255
            cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                largest = max(cnts, key=cv2.contourArea)
                M = cv2.moments(largest)
                if M["m00"] >= 16.0:
                    cx = float(M["m10"] / M["m00"])
                    cy = float(M["m01"] / M["m00"])
                    source = "mask_centroid"
        axis_dist = abs(cx - mid_x)
        if axis_dist > center_corridor_px:
            continue
        score = conf + 0.15 * max(0.0, 1.0 - axis_dist / center_corridor_px)
        if score > best_score:
            best_score = score
            best = (cx, cy, conf, source)
    return best


def main() -> None:
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    # Locate cached yolo26n-seg.pt or let Ultralytics resolve it
    base_weights = Path("yolo26n-seg.pt")
    if not base_weights.is_file():
        local_try = CONSOLE_ROOT / "yolo26n-seg.pt"
        if local_try.is_file():
            base_weights = local_try

    model = YOLO(str(base_weights))
    model.train(
        data=str(DATASET_YAML),
        epochs=28,
        imgsz=640,
        batch=16,
        device=0,
        workers=0,
        project=str(RUNS_DIR),
        name="black_block_rig_v1",
        exist_ok=True,
        degrees=35.0,
        scale=0.65,
        translate=0.15,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.4,
        hsv_v=0.45,
        mosaic=0.7,
        close_mosaic=8,
        patience=12,
        verbose=True,
    )

    best_pt = RUNS_DIR / "black_block_rig_v1" / "weights" / "best.pt"
    if not best_pt.is_file():
        raise FileNotFoundError(f"Training did not produce {best_pt}")
    shutil.copy2(best_pt, OUTPUT_MODEL_PATH)
    print(f"Saved trained YOLO26n-seg model to: {OUTPUT_MODEL_PATH}")

    trained = YOLO(str(OUTPUT_MODEL_PATH))

    # 1. Verify hard-negative FP rate & close-up recall on val split
    neg_imgs = sorted((DATASET_DIR / "images" / "val").glob("rig_empty_neg_*.jpg"))
    neg_fp = 0
    for p in neg_imgs:
        img = cv2.imread(str(p))
        r = trained.predict(img, conf=0.50, imgsz=640, device=0, half=True, verbose=False)[0]
        if mask_centroid_from_result(r, img.shape[1], img.shape[0], center_corridor_px=110.0, min_conf=0.50) is not None:
            neg_fp += 1

    closeup_imgs = sorted((DATASET_DIR / "images" / "val").glob("rig_closeup_aug_*.jpg"))
    closeup_hit = 0
    for p in closeup_imgs:
        img = cv2.imread(str(p))
        r = trained.predict(img, conf=0.45, imgsz=640, device=0, half=True, verbose=False)[0]
        if mask_centroid_from_result(r, img.shape[1], img.shape[0], center_corridor_px=320.0, min_conf=0.45) is not None:
            closeup_hit += 1

    print(f"Empty-gripper hard-negative FP: {neg_fp}/{len(neg_imgs)}")
    print(f"Close-up / tilted augmented recall: {closeup_hit}/{len(closeup_imgs)}")

    # 2. Re-evaluate ALL 17 holds in sweep_20261123_200109.avi using the new YOLO26n-seg mask centroid!
    events = [json.loads(line) for line in EVENTS_PATH.read_text(encoding="utf-8").splitlines() if line.strip()]
    raw_holds = [e for e in events if e.get("event") == "hold_complete"]
    # Keep the latest record per step_index (note step 8 was re-recorded once in the log)
    by_step: dict[int, dict] = {}
    for h in raw_holds:
        by_step[int(h["step_index"])] = h
    ordered_steps = [by_step[k] for k in sorted(by_step)]

    cap = cv2.VideoCapture(str(VIDEO_PATH))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    recomputed_holds: list[dict] = []
    total_hold_frames = 0
    total_detected_frames = 0
    for h in ordered_steps:
        f_start = int(h["start_frame"])
        f_end = int(h["end_frame"])
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, f_start - 1))
        ys: list[float] = []
        xs: list[float] = []
        n_frames = max(1, f_end - f_start + 1)
        for _ in range(n_frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            total_hold_frames += 1
            res = trained.predict(frame, conf=0.45, imgsz=640, device=0, half=True, verbose=False)[0]
            det = mask_centroid_from_result(res, width, height, center_corridor_px=105.0, min_conf=0.45)
            if det is not None:
                total_detected_frames += 1
                xs.append(det[0])
                ys.append(det[1])
        recomputed_holds.append({
            "step_index": int(h["step_index"]),
            "direction": h["direction"],
            "guide_extension_deg": float(h["guide_extension_deg"]),
            "hx8_extension_deg_median": float(h["hx8_extension_deg_median"]),
            "hx8_pos_mdeg_median": float(h["hx8_pos_mdeg_median"]),
            "frame_count": n_frames,
            "detected_count": len(ys),
            "center_x_median_px": round(statistics.median(xs), 3) if xs else None,
            "center_y_median_px": round(statistics.median(ys), 3) if ys else None,
            "y_samples": ys,
        })
        print(
            f"Step {h['step_index']:2d} ({h['direction']:8s} {h['guide_extension_deg']:4.1f}°): "
            f"detected {len(ys)}/{n_frames} frames, y_med={recomputed_holds[-1]['center_y_median_px']}"
        )
    cap.release()

    grouped: dict[float, list[dict]] = {}
    for rh in recomputed_holds:
        if rh["y_samples"]:
            grouped.setdefault(rh["guide_extension_deg"], []).append(rh)

    points: list[dict] = []
    y_hysts: list[float] = []
    enc_hysts: list[float] = []
    all_x_meds: list[float] = []
    for guide_deg in sorted(grouped):
        grp = grouped[guide_deg]
        outbound = [g for g in grp if g["direction"] == "outbound"]
        ret = [g for g in grp if g["direction"] == "return"]
        all_y = [y for g in grp for y in g["y_samples"]]
        ext_vals = [g["hx8_extension_deg_median"] for g in grp]
        x_vals = [g["center_x_median_px"] for g in grp if g["center_x_median_px"] is not None]
        if x_vals:
            all_x_meds.extend(x_vals)
        y_med = round(statistics.median(all_y), 3)
        ext_med = round(statistics.median(ext_vals), 3)

        y_hyst = None
        enc_hyst = None
        if outbound and ret:
            y_hyst = round(abs(outbound[-1]["center_y_median_px"] - ret[-1]["center_y_median_px"]), 3)
            enc_hyst = round(abs(outbound[-1]["hx8_extension_deg_median"] - ret[-1]["hx8_extension_deg_median"]), 3)
            y_hysts.append(y_hyst)
            enc_hysts.append(enc_hyst)

        points.append({
            "guide_extension_deg": round(guide_deg, 1),
            "extension_deg": ext_med,
            "center_y_px": y_med,
            "hold_count": len(grp),
            "sample_count": len(all_y),
            "hysteresis_y_px": y_hyst,
            "hysteresis_encoder_deg": enc_hyst,
        })

    y_seq = [p["center_y_px"] for p in points]
    monotonic = all(b < a for a, b in zip(y_seq, y_seq[1:])) or all(b > a for a, b in zip(y_seq, y_seq[1:]))
    arm_axis_x_px = round(statistics.median(all_x_meds), 1) if all_x_meds else 307.0

    lut_data = {
        "type": "calibrated_position_lut",
        "name": "rig_position_lut_yolo26n_seg_20261123",
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "detector_model": OUTPUT_MODEL_PATH.name,
        "detector_anchor": "mask_centroid",
        "frame_height": height,
        "frame_width": width,
        "arm_axis_center_x_px": arm_axis_x_px,
        "working_zero_mdeg": 90000,
        "extension_sign": -1,
        "monotonic": monotonic,
        "hold_detection_rate": round(total_detected_frames / max(1, total_hold_frames), 5),
        "validation": {
            "empty_gripper_false_positives": f"{neg_fp}/{len(neg_imgs)}",
            "closeup_augmented_recall": f"{closeup_hit}/{len(closeup_imgs)}",
            "hold_frames_detected": f"{total_detected_frames}/{total_hold_frames}",
        },
        "points": points,
        "hysteresis_summary": {
            "max_y_px": round(max(y_hysts), 3) if y_hysts else None,
            "mean_y_px": round(statistics.mean(y_hysts), 3) if y_hysts else None,
            "max_encoder_deg": round(max(enc_hysts), 3) if enc_hysts else None,
            "mean_encoder_deg": round(statistics.mean(enc_hysts), 3) if enc_hysts else None,
        },
        "source": {
            "video": str(VIDEO_PATH),
            "events": str(EVENTS_PATH),
            "total_holds": len(recomputed_holds),
        },
    }

    ACTIVE_LUT_PATH.write_text(json.dumps(lut_data, ensure_ascii=False, indent=2), encoding="utf-8")
    (SWEEP_DIR / "position_lut_yolo26.json").write_text(json.dumps(lut_data, ensure_ascii=False, indent=2), encoding="utf-8")
    print("=== REGENERATED LUT SUMMARY ===")
    print(json.dumps({
        "monotonic": monotonic,
        "arm_axis_center_x_px": arm_axis_x_px,
        "hold_frames_detected": f"{total_detected_frames}/{total_hold_frames}",
        "empty_gripper_fp": f"{neg_fp}/{len(neg_imgs)}",
        "closeup_recall": f"{closeup_hit}/{len(closeup_imgs)}",
        "hysteresis_summary": lut_data["hysteresis_summary"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

"""Build a high-recall, zero-clutter black-block segmentation dataset for the current rig.

Combines:
1. Real frames across all 17 holds + motion transitions of sweep_20261123_200109.avi
2. Inpainted current-room + empty-white-gripper hard negatives (doorway, cabinet strip, servo horn, tiles)
3. Close-up ('放跟前', up to 50% frame area), tilted, specular-glare, and edge-clipped black-block positives
4. Real hand-held black-foam positives & hand/arm negatives (polygon masks only; zero old LUT usage)
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import random
import shutil

import cv2
import numpy as np

CONSOLE_ROOT = Path(__file__).resolve().parents[1]
SWEEP_DIR = CONSOLE_ROOT / "runtime" / "position_recordings" / "sweep_20261123_200109"
VIDEO_PATH = SWEEP_DIR / "sweep_20261123_200109.avi"
EVENTS_PATH = SWEEP_DIR / "sweep_20261123_200109.angles.jsonl"
DATASET_DIR = CONSOLE_ROOT / "runtime" / "datasets" / "black_block_rig_v1"
OLD_V9_DIR = Path(r"D:\Study_data\AerialRoboArm-Vision-Core\data\training\foam_center_v9")


def extract_clean_foam_mask(frame_bgr: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Isolate the black foam block held in the gripper from background cabinet strips and doorway."""
    h, w = frame_bgr.shape[:2]
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    # In sweep_20261123_200109.avi, the foam block stays inside x=[250, 360], y=[90, 450]
    # with gray ~18..48, whereas cabinet trim >=64 and white jaws >=175.
    roi_mask = np.zeros((h, w), dtype=np.uint8)
    roi_mask[75:452, 248:362] = 255
    dark = ((gray <= 52) & (roi_mask > 0)).astype(np.uint8) * 255
    # Open with 5x5 to sever any 1-3 px contact, then close 7x7 to fill serrated jaw indentations slightly
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    cleaned = cv2.morphologyEx(dark, cv2.MORPH_OPEN, k_open)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, k_close)
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    best_area = 0.0
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if not (3500.0 <= area <= 22000.0):
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        aspect = bw / max(1.0, float(bh))
        # Reject the thin horizontal servo horn (~55x12, aspect > 3.2)
        if not (0.38 <= aspect <= 1.35):
            continue
        if area > best_area:
            best_area = area
            best_cnt = cnt
    if best_cnt is None:
        return None, None
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [best_cnt], -1, 255, thickness=-1)
    approx = cv2.approxPolyDP(best_cnt, 1.8, True)
    if len(approx) < 4:
        return None, None
    poly = approx.reshape(-1, 2)
    return mask, poly


def mask_to_yolo_line(mask: np.ndarray) -> str | None:
    h, w = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < 180.0:
        return None
    approx = cv2.approxPolyDP(cnt, 1.8, True)
    if len(approx) < 3:
        return None
    pts = approx.reshape(-1, 2)
    coords = []
    for px, py in pts:
        nx = min(1.0, max(0.0, float(px) / w))
        ny = min(1.0, max(0.0, float(py) / h))
        coords.append(f"{nx:.6f} {ny:.6f}")
    return "0 " + " ".join(coords)


def inpaint_foam_to_empty_gripper(frame_bgr: np.ndarray, foam_mask: np.ndarray) -> np.ndarray:
    """Create a realistic empty-gripper negative frame in the exact current room."""
    dilated = cv2.dilate(foam_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
    # Preserve white gripper pixels (bright pixels > 145) so white serrated teeth stay crisp!
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    inpaint_region = ((dilated > 0) & (gray < 145)).astype(np.uint8) * 255
    return cv2.inpaint(frame_bgr, inpaint_region, 9, cv2.INPAINT_TELEA)


def paste_closeup_augmented_block(
    bg_bgr: np.ndarray,
    block_bgr: np.ndarray,
    block_mask: np.ndarray,
    rng: random.Random,
) -> tuple[np.ndarray, np.ndarray]:
    """Synthesize close-up ('放跟前'), tilted, glare-lit, or edge-clipped black block on real background."""
    h, w = bg_bgr.shape[:2]
    bh, bw = block_bgr.shape[:2]

    # Sample scale heavily covering close-up ("放跟前": 1.3x to 3.4x -> up to ~48% of frame!) as well as far (0.65x)
    scale = rng.choice([
        rng.uniform(0.65, 1.15),
        rng.uniform(1.25, 2.15),
        rng.uniform(2.15, 3.35),
    ])
    angle = rng.uniform(-48.0, 48.0)

    pad = int(max(bh, bw) * 0.8)
    canvas_bgr = cv2.copyMakeBorder(block_bgr, pad, pad, pad, pad, cv2.BORDER_REPLICATE)
    canvas_msk = cv2.copyMakeBorder(block_mask, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=0)
    ch, cw = canvas_bgr.shape[:2]
    M = cv2.getRotationMatrix2D((cw / 2.0, ch / 2.0), angle, scale)
    rot_bgr = cv2.warpAffine(canvas_bgr, M, (cw, ch), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
    rot_msk = cv2.warpAffine(canvas_msk, M, (cw, ch), flags=cv2.INTER_NEAREST, borderValue=0)

    ys, xs = np.where(rot_msk > 127)
    if xs.size < 50:
        return bg_bgr, np.zeros((h, w), dtype=np.uint8)
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    crop_bgr = rot_bgr[y0:y1, x0:x1].copy().astype(np.float32)
    crop_msk = (rot_msk[y0:y1, x0:x1] > 127).astype(np.uint8) * 255
    ph, pw = crop_bgr.shape[:2]

    # Simulate close-range lens illumination / diffuse surface reflection (+0..+52 gray gradient)
    if rng.random() < 0.75:
        gx = np.linspace(rng.uniform(-0.5, 0.5), rng.uniform(0.5, 1.2), pw, dtype=np.float32)
        gy = np.linspace(rng.uniform(-0.5, 0.5), rng.uniform(0.5, 1.2), ph, dtype=np.float32)
        glare = np.clip(np.outer(gy, gx), 0.0, 1.0) * rng.uniform(12.0, 52.0)
        crop_bgr += glare[:, :, None]
    # Overall brightness shift (dark shadow down to 12, or bright close-up up to ~85)
    crop_bgr = np.clip(crop_bgr * rng.uniform(0.75, 1.55) + rng.uniform(-6.0, 22.0), 8.0, 118.0)

    # Optional simulated finger occlusion on one edge (10%-22%) when hand-holding the block
    if rng.random() < 0.35 and ph > 40 and pw > 40:
        occ_w = rng.randint(max(8, pw // 6), max(12, pw // 3))
        occ_h = rng.randint(max(8, ph // 5), max(14, ph // 2))
        ox = 0 if rng.random() < 0.5 else pw - occ_w
        oy = rng.randint(ph // 3, ph - occ_h)
        skin_color = np.array([rng.uniform(125, 165), rng.uniform(140, 185), rng.uniform(170, 215)], dtype=np.float32)
        crop_bgr[oy : oy + occ_h, ox : ox + occ_w] = skin_color
        crop_msk[oy : oy + occ_h, ox : ox + occ_w] = 0

    # Place mostly near central corridor, occasionally wider or slightly clipped at top/bottom ("放跟前")
    target_cx = int(rng.gauss(w * 0.50, w * 0.12))
    target_cx = max(pw // 4, min(w - pw // 4, target_cx))
    target_cy = int(rng.uniform(0.08 * h, 0.94 * h))

    dst_x0 = target_cx - pw // 2
    dst_y0 = target_cy - ph // 2
    src_x0 = max(0, -dst_x0)
    src_y0 = max(0, -dst_y0)
    dst_x1 = min(w, dst_x0 + pw)
    dst_y1 = min(h, dst_y0 + ph)
    dst_x0 = max(0, dst_x0)
    dst_y0 = max(0, dst_y0)
    copy_w = dst_x1 - dst_x0
    copy_h = dst_y1 - dst_y0
    if copy_w < 16 or copy_h < 16:
        return bg_bgr, np.zeros((h, w), dtype=np.uint8)

    sub_bgr = crop_bgr[src_y0 : src_y0 + copy_h, src_x0 : src_x0 + copy_w]
    sub_msk = crop_msk[src_y0 : src_y0 + copy_h, src_x0 : src_x0 + copy_w]
    feathered = cv2.GaussianBlur(sub_msk.astype(np.float32) / 255.0, (5, 5), 1.0)[:, :, None]

    out_bgr = bg_bgr.copy()
    roi = out_bgr[dst_y0:dst_y1, dst_x0:dst_x1].astype(np.float32)
    blended = roi * (1.0 - feathered) + sub_bgr * feathered
    out_bgr[dst_y0:dst_y1, dst_x0:dst_x1] = np.clip(blended, 0, 255).astype(np.uint8)

    full_mask = np.zeros((h, w), dtype=np.uint8)
    full_mask[dst_y0:dst_y1, dst_x0:dst_x1] = sub_msk
    return out_bgr, full_mask


def main() -> None:
    rng = random.Random(20261123)
    if DATASET_DIR.exists():
        shutil.rmtree(DATASET_DIR)
    for split in ("train", "val"):
        (DATASET_DIR / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET_DIR / "labels" / split).mkdir(parents=True, exist_ok=True)
    (DATASET_DIR / "debug_preview").mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(VIDEO_PATH))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # Outbound frames (< 4500) -> train; Return frames (>= 4500) -> split 65% train / 35% independent val segment
    # Specifically, reserve frames [6800..8797] strictly for val so train and val do not share adjacent frames!
    val_cutoff_frame = 7000

    sampled_indices = list(range(50, total_frames - 30, 22))
    real_pos_train = real_pos_val = 0
    neg_backgrounds_train: list[np.ndarray] = []
    neg_backgrounds_val: list[np.ndarray] = []
    cutouts: list[tuple[np.ndarray, np.ndarray]] = []

    for idx, fno in enumerate(sampled_indices):
        cap.set(cv2.CAP_PROP_POS_FRAMES, fno)
        ok, frame = cap.read()
        if not ok or frame is None:
            continue
        mask, poly = extract_clean_foam_mask(frame)
        if mask is None or poly is None:
            continue
        yolo_line = mask_to_yolo_line(mask)
        if yolo_line is None:
            continue
        split = "val" if fno >= val_cutoff_frame else "train"
        stem = f"rig_sweep_f{fno:05d}"
        cv2.imwrite(str(DATASET_DIR / "images" / split / f"{stem}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
        (DATASET_DIR / "labels" / split / f"{stem}.txt").write_text(yolo_line + "\n", encoding="utf-8")
        if split == "train":
            real_pos_train += 1
        else:
            real_pos_val += 1

        if idx % 3 == 0:
            empty_bg = inpaint_foam_to_empty_gripper(frame, mask)
            if split == "train":
                neg_backgrounds_train.append(empty_bg)
            else:
                neg_backgrounds_val.append(empty_bg)
        if split == "train" and idx % 6 == 0:
            ys, xs = np.where(mask > 0)
            x0, x1 = int(xs.min()), int(xs.max()) + 1
            y0, y1 = int(ys.min()), int(ys.max()) + 1
            cutouts.append((frame[y0:y1, x0:x1].copy(), mask[y0:y1, x0:x1].copy()))

        if idx < 8:
            dbg = frame.copy()
            cv2.polylines(dbg, [poly.reshape(-1, 1, 2)], True, (0, 255, 128), 2)
            cv2.imwrite(str(DATASET_DIR / "debug_preview" / f"{stem}.jpg"), dbg)
    cap.release()

    # Save hard-negative empty-gripper frames (teaches model: white gripper + doorway + cabinet strip != foam!)
    neg_train_count = neg_val_count = 0
    for i, bg in enumerate(neg_backgrounds_train):
        stem = f"rig_empty_neg_tr_{i:04d}"
        cv2.imwrite(str(DATASET_DIR / "images" / "train" / f"{stem}.jpg"), bg, [cv2.IMWRITE_JPEG_QUALITY, 93])
        (DATASET_DIR / "labels" / "train" / f"{stem}.txt").write_text("", encoding="utf-8")
        neg_train_count += 1
    for i, bg in enumerate(neg_backgrounds_val):
        stem = f"rig_empty_neg_va_{i:04d}"
        cv2.imwrite(str(DATASET_DIR / "images" / "val" / f"{stem}.jpg"), bg, [cv2.IMWRITE_JPEG_QUALITY, 93])
        (DATASET_DIR / "labels" / "val" / f"{stem}.txt").write_text("", encoding="utf-8")
        neg_val_count += 1

    # Generate close-up ("放跟前"), tilted, glare-lit & partially occluded synthetic positives on current room bg
    closeup_train = closeup_val = 0
    for i in range(340):
        split = "val" if i >= 280 else "train"
        bg_pool = neg_backgrounds_val if (split == "val" and neg_backgrounds_val) else neg_backgrounds_train
        bg = rng.choice(bg_pool)
        blk_bgr, blk_msk = rng.choice(cutouts)
        aug_bgr, aug_msk = paste_closeup_augmented_block(bg, blk_bgr, blk_msk, rng)
        line = mask_to_yolo_line(aug_msk)
        if line is None:
            continue
        stem = f"rig_closeup_aug_{i:04d}"
        cv2.imwrite(str(DATASET_DIR / "images" / split / f"{stem}.jpg"), aug_bgr, [cv2.IMWRITE_JPEG_QUALITY, 93])
        (DATASET_DIR / "labels" / split / f"{stem}.txt").write_text(line + "\n", encoding="utf-8")
        if split == "train":
            closeup_train += 1
            if closeup_train <= 8:
                dbg = aug_bgr.copy()
                cnts, _ = cv2.findContours(aug_msk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(dbg, cnts, -1, (255, 220, 0), 2)
                cv2.imwrite(str(DATASET_DIR / "debug_preview" / f"{stem}.jpg"), dbg)
        else:
            closeup_val += 1

    # Add real hand-held black-foam images (no white jaws, real hand/string/motion/dim-light) + real hand negatives
    hand_pos = hand_neg = 0
    if OLD_V9_DIR.is_dir():
        v9_train_lbls = sorted((OLD_V9_DIR / "labels" / "train").glob("*.txt"))
        rng.shuffle(v9_train_lbls)
        for lbl_file in v9_train_lbls:
            # Skip old gripper_axis frames so old gripper geometry never conflicts; keep pure hand-held foam & negatives!
            if "gripper" in lbl_file.stem:
                continue
            img_file = OLD_V9_DIR / "images" / "train" / f"{lbl_file.stem}.jpg"
            if not img_file.is_file():
                continue
            txt = lbl_file.read_text(encoding="utf-8").strip()
            if txt and hand_pos < 240:
                shutil.copy2(img_file, DATASET_DIR / "images" / "train" / f"hand_{img_file.name}")
                (DATASET_DIR / "labels" / "train" / f"hand_{lbl_file.name}").write_text(txt + "\n", encoding="utf-8")
                hand_pos += 1
            elif not txt and hand_neg < 80:
                shutil.copy2(img_file, DATASET_DIR / "images" / "train" / f"handneg_{img_file.name}")
                (DATASET_DIR / "labels" / "train" / f"handneg_{lbl_file.name}").write_text("", encoding="utf-8")
                hand_neg += 1
            if hand_pos >= 240 and hand_neg >= 80:
                break

    yaml_text = (
        f"path: {DATASET_DIR.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: black_block\n"
    )
    (DATASET_DIR / "dataset.yaml").write_text(yaml_text, encoding="utf-8")

    summary = {
        "dataset_dir": str(DATASET_DIR),
        "real_sweep_train": real_pos_train,
        "real_sweep_val": real_pos_val,
        "empty_gripper_neg_train": neg_train_count,
        "empty_gripper_neg_val": neg_val_count,
        "closeup_aug_train": closeup_train,
        "closeup_aug_val": closeup_val,
        "handheld_pos_train": hand_pos,
        "handheld_neg_train": hand_neg,
    }
    (DATASET_DIR / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

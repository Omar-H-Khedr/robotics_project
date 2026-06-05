"""Shared 74-dim context-vector computation utilities.

Used by both the offline v2_12 context_vector_extractor (which
reads the multimodal_observation_log.csv) and the live
live_v2_14_inference_node (which computes the context vector on
the fly from ROS messages). Centralizing the computation here
guarantees the live and offline pipelines produce identical
features.

Context vector layout (v2_12, length = CONTEXT_DIM = 74):
    [ 0:48]  rgb: 8x6 grayscale, normalized [0, 1]
    [48:54]  depth: (w, h, min_m, max_m, roi_min_m, roi_max_m)
             bad values (NaN / inf) replaced with 0.0
    [54:60]  wrench: (fx, fy, fz, tx, ty, tz)
    [60:66]  joint position: (j1..j6) in rad
    [66:72]  joint velocity: (j1..j6) in rad/s; NaN replaced with 0.0
    [ 72 ]   phase_int: enum encoding task_phase
    [ 73 ]   safety_int: enum encoding safety_status
"""
from __future__ import annotations

import base64
import io
from typing import Tuple

import numpy as np
from PIL import Image as PILImage

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - cv_bridge is a ROS dep
    CvBridge = None


RGB_TARGET_W = 8
RGB_TARGET_H = 6
RGB_DIMS = RGB_TARGET_W * RGB_TARGET_H
DEPTH_DIMS = 6
WRENCH_DIMS = 6
JOINT_POS_DIMS = 6
JOINT_VEL_DIMS = 6
PHASE_DIMS = 1
SAFETY_DIMS = 1
CONTEXT_DIM = (
    RGB_DIMS + DEPTH_DIMS + WRENCH_DIMS
    + JOINT_POS_DIMS + JOINT_VEL_DIMS + PHASE_DIMS + SAFETY_DIMS
)

PHASE_ENUM = {
    "UNKNOWN": 0, "IDLE": 0,
    "MOVE_TO_START": 1, "MOVING_TO_START": 1,
    "APPROACH": 2, "CHECK_ALIGNMENT": 2,
    "SEARCH": 3,
    "HOVER_ABOVE_HOLE": 4,
    "INSERT": 5, "INSERTING": 5,
    "RETREAT": 6, "INSERTED": 6,
    "DONE": 7,
    "ABORT": 8, "COMPLETE": 8,
}
SAFETY_ENUM = {
    "UNKNOWN": 0,
    "OK": 1, "SAFE": 1,
    "WARNING": 2, "WARN": 2,
    "ABORT": 3,
}

DEPTH_DIM_INDICES = (48, 49)
DEPTH_VALUE_INDICES = (50, 51, 52, 53)
DEPTH_CLIP_VALUE = 5.0
NUM_PHASE_CLASSES = 9


def decode_rgb_b64_png(b64_png: str, target_w: int = RGB_TARGET_W,
                       target_h: int = RGB_TARGET_H) -> np.ndarray:
    if not isinstance(b64_png, str) or len(b64_png) == 0:
        return np.zeros(target_h * target_w, dtype=np.float32)
    raw = base64.b64decode(b64_png.encode("ascii"))
    with PILImage.open(io.BytesIO(raw)) as img:
        img = img.convert("L")
        img = img.resize((target_w, target_h), PILImage.BILINEAR)
        arr = np.asarray(img, dtype=np.float32) / 255.0
    return arr.reshape(-1)


def encode_rgb_to_48(msg, bridge) -> Tuple[int, int, np.ndarray]:
    if CvBridge is None or bridge is None:
        return 0, 0, np.zeros(RGB_DIMS, dtype=np.float32)
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:
        return 0, 0, np.zeros(RGB_DIMS, dtype=np.float32)
    h, w = img.shape[:2]
    if h == 0 or w == 0:
        return 0, 0, np.zeros(RGB_DIMS, dtype=np.float32)
    scale_w = RGB_TARGET_W / w
    scale_h = RGB_TARGET_H / h
    new_w = RGB_TARGET_W
    new_h = RGB_TARGET_H
    small = img[:: max(1, h // new_h), :: max(1, w // new_w)][:new_h, :new_w]
    if small.size == 0:
        return 0, 0, np.zeros(RGB_DIMS, dtype=np.float32)
    try:
        pil = PILImage.fromarray(small[:, :, ::-1])
        buf = io.BytesIO()
        pil.save(buf, format="PNG", optimize=True)
        rgb = decode_rgb_b64_png(
            base64.b64encode(buf.getvalue()).decode("ascii"),
            RGB_TARGET_W, RGB_TARGET_H,
        )
    except Exception:
        rgb = np.zeros(RGB_DIMS, dtype=np.float32)
    return new_w, new_h, rgb


def summarize_depth_msg(msg, bridge) -> Tuple[int, int, float, float, float, float]:
    if CvBridge is None or bridge is None:
        return 0, 0, 0.0, 0.0, 0.0, 0.0
    try:
        depth = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception:
        return 0, 0, 0.0, 0.0, 0.0, 0.0
    depth = np.asarray(depth, dtype=np.float32)
    if depth.size == 0:
        return 0, 0, 0.0, 0.0, 0.0, 0.0
    valid = depth[depth > 0]
    if valid.size == 0:
        d_min = 0.0
        d_max = 0.0
    else:
        d_min = float(np.min(valid))
        d_max = float(np.max(valid))
    h, w = depth.shape[:2]
    rx, ry = 0, 0
    rw, rh = w, h
    rx = max(0, min(w - 1, rx))
    ry = max(0, min(h - 1, ry))
    rw = max(1, min(w - rx, rw))
    rh = max(1, min(h - ry, rh))
    roi = depth[ry : ry + rh, rx : rx + rw]
    roi_valid = roi[roi > 0]
    if roi_valid.size == 0:
        roi_min = 0.0
        roi_max = 0.0
    else:
        roi_min = float(np.min(roi_valid))
        roi_max = float(np.max(roi_valid))
    return w, h, d_min, d_max, roi_min, roi_max


def summarize_depth_csv_row(w, h, d_min, d_max, roi_min, roi_max) -> np.ndarray:
    def _safe(x):
        x = float(x) if x is not None else 0.0
        if not np.isfinite(x):
            return 0.0
        return x
    return np.array([
        float(w or 0), float(h or 0),
        _safe(d_min), _safe(d_max), _safe(roi_min), _safe(roi_max),
    ], dtype=np.float32)


def phase_to_int(s: str) -> int:
    if s is None:
        return 0
    s = str(s).strip()
    if s.startswith("{"):
        import json
        try:
            obj = json.loads(s)
            if isinstance(obj, dict):
                for k in ("phase", "status", "name", "code"):
                    if k in obj and obj[k] is not None:
                        v = str(obj[k]).strip().upper()
                        if v in PHASE_ENUM:
                            return PHASE_ENUM[v]
        except (ValueError, TypeError):
            pass
        return 0
    s = s.upper()
    return PHASE_ENUM.get(s, 0)


def safety_to_int(s: str) -> int:
    if s is None:
        return 0
    s = str(s).strip()
    if s.startswith("{"):
        import json
        try:
            obj = json.loads(s)
            if isinstance(obj, dict):
                if "level" in obj and obj["level"] is not None:
                    v = str(obj["level"]).strip().upper()
                    if v in SAFETY_ENUM:
                        return SAFETY_ENUM[v]
                for k in ("status", "name", "code"):
                    if k in obj and obj[k] is not None:
                        v = str(obj[k]).strip().upper()
                        if v in SAFETY_ENUM:
                            return SAFETY_ENUM[v]
        except (ValueError, TypeError):
            pass
        return 0
    s = s.upper()
    return SAFETY_ENUM.get(s, 0)

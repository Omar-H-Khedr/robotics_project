"""Offline context vector extraction from multimodal_observation_log.csv.

Reads a multimodal observation CSV produced by
multimodal_observation_logger and writes a parquet file with one
fixed-length context vector per row. Used to build the training
corpus for v2_14 context encoder and v2_15 context-conditioned action.

Context vector layout (v2_13, length = CONTEXT_DIM = 68):
    [ 0:48]  rgb: decode rgb_b64_png (64x36 PNG) -> resize to 8x6 -> flatten
             -> normalize to [0, 1] (uint8 / 255)
    [48:54]  depth: (w, h, min_m, max_m, roi_min_m, roi_max_m)
             bad values (NaN / inf) are replaced with 0.0
    [54:60]  joint position: (j1..j6) in rad
    [60:66]  joint velocity: (j1..j6) in rad/s; NaN replaced with 0.0
    [ 66 ]   phase_int: enum encoding task_phase
    [ 67 ]   safety_int: enum encoding safety_status

Wrench features (F/T sensor) are excluded because the ft_sensor_bridge
crashes with SIGSEGV at startup. The admittance_insertion_node still
receives F/T through gz_ros_control, but the perception logger's
/ft_sensor_wrench subscription gets nothing.

A bad-frame mask is not stored; the convention is that bad (NaN, inf)
values are replaced with 0.0 and the row stays in the parquet. Filtering
on phase_int == 0 or safety_int == 0 at training time is the user's
responsibility.
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image

RGB_TARGET_W = 8
RGB_TARGET_H = 6
RGB_DIMS = RGB_TARGET_W * RGB_TARGET_H  # 48
DEPTH_DIMS = 6
JOINT_POS_DIMS = 6
JOINT_VEL_DIMS = 6
PHASE_DIMS = 1
SAFETY_DIMS = 1
CONTEXT_DIM = (
    RGB_DIMS + DEPTH_DIMS
    + JOINT_POS_DIMS + JOINT_VEL_DIMS + PHASE_DIMS + SAFETY_DIMS
)

PHASE_ENUM = {
    "UNKNOWN": 0,
    "IDLE": 0,
    "MOVE_TO_START": 1,
    "MOVING_TO_START": 1,
    "APPROACH": 2,
    "CHECK_ALIGNMENT": 2,
    "SEARCH": 3,
    "HOVER_ABOVE_HOLE": 4,
    "INSERT": 5,
    "INSERTING": 5,
    "RETREAT": 6,
    "INSERTED": 6,
    "DONE": 7,
    "ABORT": 8,
    "COMPLETE": 8,
}
SAFETY_ENUM = {
    "UNKNOWN": 0,
    "OK": 1,
    "SAFE": 1,
    "WARNING": 2,
    "WARN": 2,
    "ABORT": 3,
}

CONTEXT_SPEC = [
    f"rgb_{i}" for i in range(RGB_DIMS)
] + [
    "depth_w", "depth_h", "depth_min_m", "depth_max_m",
    "depth_roi_min_m", "depth_roi_max_m",
] + [
    f"joint_{j}_pos_rad" for j in range(1, 7)
] + [
    f"joint_{j}_vel_rad_s" for j in range(1, 7)
] + ["phase_int", "safety_int"]


def _decode_rgb(b64_png: str, target_w: int, target_h: int) -> np.ndarray:
    """Decode base64 PNG, resize to (target_h, target_w) grayscale, normalize."""
    if not isinstance(b64_png, str) or len(b64_png) == 0:
        return np.zeros(target_h * target_w, dtype=np.float32)
    raw = base64.b64decode(b64_png.encode("ascii"))
    with Image.open(io.BytesIO(raw)) as img:
        img = img.convert("L")
        img = img.resize((target_w, target_h), Image.BILINEAR)
        arr = np.asarray(img, dtype=np.float32) / 255.0
    return arr.reshape(-1)


def _safe_float_series(df: pd.DataFrame, col: str) -> np.ndarray:
    if col not in df.columns:
        return np.zeros(len(df), dtype=np.float32)
    arr = pd.to_numeric(df[col], errors="coerce").to_numpy(dtype=np.float32)
    arr = np.nan_to_num(arr, nan=0.0, posinf=1e6, neginf=-1e6)
    return arr


def _normalize_status_string(v, enum_map: dict) -> str:
    """Extract the upper-case status token from a string or a JSON object.

    The safety_monitor publishes statuses as
    `{"level": "OK|WARNING|ABORT", "code": "joint_states_valid", ...}`
    JSON; the admittance_insertion_node publishes phases as plain
    strings like `MOVING_TO_START`. Both forms must map cleanly into
    the enums.

    For safety_status, prefer the `level` field (OK / WARNING / ABORT)
    over `code` (which is a fine-grained descriptor like
    `joint_states_valid`).
    """
    if v is None:
        return ""
    s = str(v).strip()
    if s.startswith("{"):
        try:
            obj = json.loads(s)
        except (ValueError, TypeError):
            return s.upper()
        if isinstance(obj, dict):
            for key in ("level", "code", "phase", "status", "name"):
                if key in obj and obj[key] is not None:
                    return str(obj[key]).strip().upper()
            return s.upper()
    return s.upper()


def _safe_int_array(values, enum_map: dict, default: int) -> np.ndarray:
    out = np.empty(len(values), dtype=np.int32)
    for i, v in enumerate(values):
        if v is None or (isinstance(v, float) and np.isnan(v)):
            out[i] = default
            continue
        s = _normalize_status_string(v, enum_map)
        out[i] = enum_map.get(s, default)
    return out


def extract(csv_path: Path, parquet_path: Path) -> dict:
    """Read the CSV, build a (N, CONTEXT_DIM) float32 array, write parquet."""
    df = pd.read_csv(csv_path)
    n = len(df)
    if n == 0:
        raise RuntimeError(f"empty CSV: {csv_path}")

    rgb_col = df["rgb_b64_png"] if "rgb_b64_png" in df.columns else pd.Series([""] * n)
    rgb_block = np.stack(
        [_decode_rgb(v, RGB_TARGET_W, RGB_TARGET_H) for v in rgb_col]
    ).astype(np.float32)

    depth_block = np.stack([
        _safe_float_series(df, "depth_w"),
        _safe_float_series(df, "depth_h"),
        _safe_float_series(df, "depth_min_m"),
        _safe_float_series(df, "depth_max_m"),
        _safe_float_series(df, "depth_roi_min_m"),
        _safe_float_series(df, "depth_roi_max_m"),
    ], axis=1).astype(np.float32)

    pos_block = np.stack([
        _safe_float_series(df, f"joint_{j}_pos_rad") for j in range(1, 7)
    ], axis=1).astype(np.float32)
    vel_block = np.stack([
        _safe_float_series(df, f"joint_{j}_vel_rad_s") for j in range(1, 7)
    ], axis=1).astype(np.float32)

    phase_block = _safe_int_array(
        df["task_phase"].tolist() if "task_phase" in df.columns else [None] * n,
        PHASE_ENUM, 0,
    ).reshape(-1, 1).astype(np.int32)
    safety_block = _safe_int_array(
        df["safety_status"].tolist() if "safety_status" in df.columns else [None] * n,
        SAFETY_ENUM, 0,
    ).reshape(-1, 1).astype(np.int32)

    context = np.concatenate([
        rgb_block, depth_block, pos_block, vel_block,
        phase_block, safety_block,
    ], axis=1).astype(np.float32)

    if context.shape[1] != CONTEXT_DIM:
        raise RuntimeError(
            f"context width mismatch: got {context.shape[1]}, expected {CONTEXT_DIM}"
        )

    stamp = pd.to_numeric(df["stamp_s"], errors="coerce").fillna(0.0).astype(np.float64).to_numpy()
    tick = pd.to_numeric(df["tick_index"], errors="coerce").fillna(0).astype(np.int64).to_numpy()

    out_df = pd.DataFrame({
        "stamp_s": stamp,
        "tick_index": tick,
        "context_vec": [row.astype(np.float32).tolist() for row in context],
        "context_dim": np.full(n, CONTEXT_DIM, dtype=np.int32),
        "phase_int": phase_block.reshape(-1).astype(np.int32),
        "safety_int": safety_block.reshape(-1).astype(np.int32),
    })
    spec_json = json.dumps({
        "version": "v2_13",
        "context_dim": CONTEXT_DIM,
        "spec": CONTEXT_SPEC,
        "phase_enum": PHASE_ENUM,
        "safety_enum": SAFETY_ENUM,
        "rgb_target": [RGB_TARGET_W, RGB_TARGET_H],
        "note": "Wrench features excluded (ft_sensor_bridge crashes with SIGSEGV)",
    })
    out_df.attrs["context_spec_json"] = spec_json

    table = pa.Table.from_pandas(out_df, preserve_index=False)
    table = table.replace_schema_metadata({
        b"context_spec_json": spec_json.encode("utf-8"),
    })
    parquet_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, parquet_path)

    return {
        "rows": int(n),
        "context_dim": int(CONTEXT_DIM),
        "rgb_block_min": float(rgb_block.min()),
        "rgb_block_max": float(rgb_block.max()),
        "depth_block_min": float(depth_block.min()),
        "depth_block_max": float(depth_block.max()),
        "pos_block_min": float(pos_block.min()),
        "pos_block_max": float(pos_block.max()),
        "vel_block_min": float(vel_block.min()),
        "vel_block_max": float(vel_block.max()),
        "phase_unique": sorted(set(phase_block.reshape(-1).tolist())),
        "safety_unique": sorted(set(safety_block.reshape(-1).tolist())),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Extract a fixed-length context vector per tick from a "
                    "multimodal_observation_log.csv into a parquet file."
    )
    parser.add_argument(
        "--input-csv", required=True,
        help="Path to multimodal_observation_log.csv (input).",
    )
    parser.add_argument(
        "--output-parquet", required=True,
        help="Path to write context_log.parquet (output).",
    )
    args = parser.parse_args(argv)

    csv_path = Path(args.input_csv)
    parquet_path = Path(args.output_parquet)
    if not csv_path.exists():
        print(f"ERROR: input CSV not found: {csv_path}", file=sys.stderr)
        return 2

    print(
        f"context_vector_extractor: reading {csv_path} "
        f"({csv_path.stat().st_size} bytes)"
    )
    summary = extract(csv_path, parquet_path)
    print(
        f"context_vector_extractor: wrote {parquet_path} "
        f"({parquet_path.stat().st_size} bytes) "
        f"rows={summary['rows']} context_dim={summary['context_dim']}"
    )
    print(
        f"  rgb    : min={summary['rgb_block_min']:.4f} max={summary['rgb_block_max']:.4f}"
    )
    print(
        f"  depth  : min={summary['depth_block_min']:.4f} max={summary['depth_block_max']:.4f}"
    )
    print(
        f"  pos    : min={summary['pos_block_min']:.4f} max={summary['pos_block_max']:.4f}"
    )
    print(
        f"  vel    : min={summary['vel_block_min']:.4f} max={summary['vel_block_max']:.4f}"
    )
    print(f"  phase  : unique={summary['phase_unique']}")
    print(f"  safety : unique={summary['safety_unique']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

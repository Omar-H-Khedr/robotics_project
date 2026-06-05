"""Offline analyzer for live v2_14 trial CSVs.

Reads the live_v2_14_inference_log.csv (per-tick ground truth
phase + v2_14 predicted phase + 6-dim target joint pose) and
the multimodal_observation_log.csv (raw sensor data) and reports:

  - total ticks, ticks with valid ground truth (phase_int != 0)
  - phase classification accuracy, per-class precision/recall
  - 9x9 confusion matrix
  - per-phase mean absolute error of the predicted target joint
    pose vs the per-phase mean (the regression baseline)
  - target joint MSE per phase
  - JSON summary: live_v2_14_ablation_summary.json
  - PNG: live_v2_14_confusion_matrix.png (if matplotlib)
  - PNG: live_v2_14_per_phase_target_mse.png (if matplotlib)
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np


PHASE_INT_TO_NAME = {
    0: "UNKNOWN", 1: "MOVE_TO_START", 2: "APPROACH", 3: "SEARCH",
    4: "HOVER_ABOVE_HOLE", 5: "INSERT", 6: "INSERTED",
    7: "DONE", 8: "ABORT",
}
PHASE_NAME_TO_INT = {v: k for k, v in PHASE_INT_TO_NAME.items()}
PHASE_NAME_TO_INT["RETREAT"] = 6
PHASE_NAME_TO_INT["DONE"] = 7


def _coerce_gt_phase(raw: str) -> int:
    raw = raw.strip()
    if raw.isdigit():
        return int(raw)
    if raw in PHASE_NAME_TO_INT:
        return PHASE_NAME_TO_INT[raw]
    if raw == "MOVING_TO_START":
        return 1
    if raw == "INSERTING":
        return 5
    if raw == "IDLE":
        return 0
    if raw == "WARN":
        return 2
    if raw == "COMPLETE":
        return 8
    if raw in ("", "None", "nan"):
        return 0
    raise ValueError(f"unknown ground_truth_phase string: {raw!r}")


def _read_inference_log(path: Path) -> dict:
    gt_phases = []
    pred_phases = []
    targets = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            gt_phases.append(_coerce_gt_phase(row["ground_truth_phase"]))
            pred_phases.append(int(row["predicted_phase_int"]))
            targets.append([
                float(row["target_joint_1"]),
                float(row["target_joint_2"]),
                float(row["target_joint_3"]),
                float(row["target_joint_4"]),
                float(row["target_joint_5"]),
                float(row["target_joint_6"]),
            ])
    return {
        "gt_phases": np.array(gt_phases, dtype=np.int64),
        "pred_phases": np.array(pred_phases, dtype=np.int64),
        "targets": np.array(targets, dtype=np.float32),
    }


def _confusion_matrix(y_true: np.ndarray, y_pred: np.ndarray, n: int) -> np.ndarray:
    cm = np.zeros((n, n), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def _per_phase_target_stats(targets: np.ndarray, gt_phases: np.ndarray) -> dict:
    per_phase_mean = {}
    per_phase_mse = {}
    for c in range(9):
        mask = gt_phases == c
        if not mask.any():
            continue
        mean = targets[mask].mean(axis=0)
        per_phase_mean[str(c)] = mean.tolist()
        if mask.sum() > 1:
            mse = float(((targets[mask] - mean) ** 2).mean())
        else:
            mse = 0.0
        per_phase_mse[str(c)] = {
            "support": int(mask.sum()),
            "mean_target_joint_pose": mean.tolist(),
            "intra_phase_mse": mse,
        }
    return per_phase_mean, per_phase_mse


def _per_class_metrics(cm: np.ndarray) -> dict:
    out = {}
    for c in range(9):
        support = int(cm[c].sum())
        pred_count = int(cm[:, c].sum())
        tp = int(cm[c, c])
        precision = tp / pred_count if pred_count > 0 else 0.0
        recall = tp / support if support > 0 else 0.0
        out[str(c)] = {
            "support": support,
            "predicted_count": pred_count,
            "true_positive": tp,
            "precision": precision,
            "recall": recall,
        }
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Live v2_14 ablation analyzer")
    parser.add_argument("--inference-csv", required=True,
                        help="Path to live_v2_14_inference_log.csv (from the live node).")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    inference_path = Path(args.inference_csv)
    print(f"live_v2_14_ablation_analyzer: inference_csv = {inference_path}")
    data = _read_inference_log(inference_path)
    gt = data["gt_phases"]
    pred = data["pred_phases"]
    targets = data["targets"]
    n_total = len(gt)
    n_valid = int((gt != 0).sum())
    print(f"  total ticks = {n_total}, valid (gt != 0) = {n_valid}")

    cm = _confusion_matrix(gt, pred, 9)
    acc = float((pred == gt).mean())
    acc_valid = float((pred[gt != 0] == gt[gt != 0]).mean()) if n_valid > 0 else 0.0
    print(f"  overall accuracy = {acc:.3f}, valid-only accuracy = {acc_valid:.3f}")
    print(f"  confusion matrix:")
    for i in range(9):
        if cm[i].sum() == 0:
            continue
        row = "    " + PHASE_INT_TO_NAME[i].ljust(20) + " ".join(f"{cm[i, j]:3d}" for j in range(9))
        print(row)

    per_phase_mean, per_phase_mse = _per_phase_target_stats(targets, gt)
    per_class = _per_class_metrics(cm)

    summary = {
        "version": "live_v2_14_ablation",
        "inference_csv": str(inference_path),
        "n_total_ticks": n_total,
        "n_valid_ticks": n_valid,
        "overall_accuracy": acc,
        "valid_only_accuracy": acc_valid,
        "per_class_metrics": per_class,
        "confusion_matrix": cm.tolist(),
        "per_phase_target_stats": per_phase_mse,
        "phase_distribution": {
            PHASE_INT_TO_NAME[int(k)]: int(v)
            for k, v in zip(*np.unique(gt, return_counts=True))
        },
    }
    summary_path = output_dir / "live_v2_14_ablation_summary.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"live_v2_14_ablation_analyzer: wrote {summary_path}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 5))
        im = ax.imshow(cm, cmap="Blues")
        ax.set_xlabel("predicted phase")
        ax.set_ylabel("ground-truth phase")
        ax.set_title("live v2_14 phase classifier (live trial)")
        for i in range(9):
            for j in range(9):
                if cm[i, j] > 0:
                    ax.text(j, i, str(cm[i, j]), ha="center", va="center",
                            color="white" if cm[i, j] > cm.max() / 2 else "black",
                            fontsize=7)
        fig.colorbar(im, ax=ax)
        fig.tight_layout()
        cm_path = output_dir / "live_v2_14_confusion_matrix.png"
        fig.savefig(cm_path, dpi=110)
        plt.close(fig)
        print(f"live_v2_14_ablation_analyzer: wrote {cm_path}")

        phases_with_data = sorted([
            int(k) for k, v in per_phase_mse.items() if v["support"] > 0
        ])
        if phases_with_data:
            labels = [PHASE_INT_TO_NAME[c] for c in phases_with_data]
            mses = [per_phase_mse[str(c)]["intra_phase_mse"] for c in phases_with_data]
            fig, ax = plt.subplots(figsize=(7, 4))
            ax.bar(range(len(labels)), mses)
            ax.set_xticks(range(len(labels)))
            ax.set_xticklabels(labels, rotation=30, ha="right")
            ax.set_ylabel("intra-phase MSE of target joint pose (rad^2)")
            ax.set_title("live v2_14 per-phase target joint pose spread")
            fig.tight_layout()
            mse_path = output_dir / "live_v2_14_per_phase_target_mse.png"
            fig.savefig(mse_path, dpi=110)
            plt.close(fig)
            print(f"live_v2_14_ablation_analyzer: wrote {mse_path}")
    except ImportError:
        print("live_v2_14_ablation_analyzer: matplotlib not available, skipped PNGs")

    print("live_v2_14_ablation_analyzer: done")
    return 0


if __name__ == "__main__":
    sys.exit(main())

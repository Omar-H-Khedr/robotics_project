#!/usr/bin/env python3
"""Offline smoke test for v2_14 safety-gated action interface.

Tests the SafetyGatedActionInterface on the 6-phase dataset and validates:
- Per-class precision/recall/F1
- Confidence thresholding
- Safety gate behavior
- Fallback reasons
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def main(args: list[str] | None = None) -> None:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parquet", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--confidence-threshold", type=float, default=0.85)
    a = parser.parse_args(args)

    sys.path.insert(0, str(Path(__file__).parent))
    from v2_14_safety_gated_action import SafetyGatedActionInterface, PHASE_NAMES

    iface = SafetyGatedActionInterface(
        model_path=a.model, confidence_threshold=a.confidence_threshold,
    )

    df = pd.read_parquet(a.parquet)
    vecs = np.stack([np.array(v, dtype=np.float32) for v in df["context_vec"]])
    phases = df["phase_int"].to_numpy(dtype=np.int64)

    valid = (vecs[:, :48].sum(axis=1) > 1.0) & (vecs[:, 48] > 0) & (vecs[:, 49] > 0)
    vecs = vecs[valid]
    phases = phases[valid]

    preds = []
    confidences = []
    fallbacks = []
    fallback_reasons = []

    for i in range(len(vecs)):
        cmd = iface.predict(vecs[i])
        preds.append(cmd.phase)
        confidences.append(cmd.confidence)
        fallbacks.append(cmd.use_fallback)
        fallback_reasons.append(cmd.fallback_reason)

    preds = np.array(preds)
    confidences = np.array(confidences)
    fallbacks = np.array(fallbacks)

    unique_phases = sorted(set(phases))
    per_class = {}
    for p in unique_phases:
        mask_true = phases == p
        mask_pred = preds == p
        tp = int((mask_true & mask_pred).sum())
        fp = int((~mask_true & mask_pred).sum())
        fn = int((mask_true & ~mask_pred).sum())
        support = int(mask_true.sum())
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, tp + fn)
        f1 = 2 * precision * recall / max(1e-8, precision + recall)
        per_class[PHASE_NAMES.get(p, str(p))] = {
            "support": support, "precision": round(precision, 4),
            "recall": round(recall, 4), "f1": round(f1, 4),
        }

    accuracy = float((preds == phases).mean())
    fallback_rate = float(fallbacks.mean())
    high_conf_mask = confidences >= a.confidence_threshold
    low_conf_mask = ~high_conf_mask
    ml_decision_rate = float((~fallbacks).mean())

    summary = {
        "accuracy": round(accuracy, 4),
        "n_samples": len(phases),
        "confidence_threshold": a.confidence_threshold,
        "mean_confidence": round(float(confidences.mean()), 4),
        "fallback_rate": round(fallback_rate, 4),
        "ml_decision_rate": round(ml_decision_rate, 4),
        "high_confidence_fraction": round(float(high_conf_mask.mean()), 4),
        "per_class": per_class,
        "fallback_reasons": {
            r: int(c) for r, c in
            zip(fallback_reasons, [1] * len(fallback_reasons))
            if r
        },
    }

    Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    Path(a.output).write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"Accuracy: {accuracy:.4f}")
    print(f"Fallback rate: {fallback_rate:.4f}")
    print(f"ML decision rate: {ml_decision_rate:.4f}")
    for name, metrics in per_class.items():
        print(f"  {name}: P={metrics['precision']:.3f} R={metrics['recall']:.3f} F1={metrics['f1']:.3f} (n={metrics['support']})")
    print(f"Results: {a.output}")


if __name__ == "__main__":
    main()

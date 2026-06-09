#!/usr/bin/env python3
"""v2_14 safety-gated action interface.

Trains a raw 68-dim context classifier and defines the safety-gated action
interface for real-time control pipeline integration.

Input: 68-dim context vector (joint_pos, joint_vel, rgb, depth, phase_int, safety_int)
Output: ActionCommand(phase, confidence, target_joints, use_fallback)

Safety rules:
- ML never overrides safety gates
- If confidence < CONFIDENCE_THRESHOLD: fallback to deterministic controller
- If phase is safety-critical and ML confidence is borderline: fallback
- INSERT phase always uses deterministic controller
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field, asdict
from enum import IntEnum
from pathlib import Path
from typing import Any

import numpy as np

try:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset
except Exception as _e:
    import sys
    print(f"WARNING: torch import failed: {_e}", file=sys.stderr)
    torch = None
    nn = None

CONTEXT_DIM = 68
NUM_CLASSES = 7
JOINT_DIM = 6
CONFIDENCE_THRESHOLD = 0.85
SAFETY_CRITICAL_PHASES = frozenset({3, 5, 6, 7})  # SEARCH, INSERT, RETREAT, DONE


class PhaseID(IntEnum):
    UNKNOWN = 0
    MOVING_TO_START = 1
    APPROACH = 2
    SEARCH = 3
    INSERT = 5
    RETREAT = 6
    DONE = 7


PHASE_NAMES = {v.value: v.name for v in PhaseID}


@dataclass
class ActionCommand:
    phase: int = 0
    phase_name: str = "UNKNOWN"
    confidence: float = 0.0
    target_joints: list[float] = field(default_factory=lambda: [0.0] * 6)
    use_fallback: bool = True
    fallback_reason: str = "init"
    raw_logits: list[float] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


if nn is not None:
    class RawContextClassifier(nn.Module):
        """Raw 68-dim input classifier for all 7 phases."""

        def __init__(self, input_dim: int = CONTEXT_DIM, hidden: int = 128,
                     num_classes: int = NUM_CLASSES, dropout: float = 0.2):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(input_dim, hidden),
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden, num_classes),
            )

        def forward(self, x: torch.Tensor) -> torch.Tensor:
            return self.net(x)
else:
    class RawContextClassifier:
        pass


class SafetyGatedActionInterface:
    """Safety-gated action interface for real-time control integration.

    Uses raw 68-dim context as input, outputs ActionCommand with confidence
    and fallback flag. ML never overrides safety gates.
    """

    def __init__(self, model_path: str | Path | None = None,
                 confidence_threshold: float = CONFIDENCE_THRESHOLD):
        self.confidence_threshold = confidence_threshold
        self._model: RawContextClassifier | None = None
        self._device = "cpu"
        self._phase_names = PHASE_NAMES
        self._target_poses: dict[int, list[float]] = {}
        self._scaler_min: np.ndarray | None = None
        self._scaler_max: np.ndarray | None = None
        self._scaler_range: np.ndarray | None = None

        if model_path and torch is not None:
            self.load_model(model_path)

    def load_model(self, model_path: str | Path) -> None:
        model_path = Path(model_path)
        ckpt = torch.load(model_path, map_location="cpu", weights_only=False)
        state_dict = ckpt.get("model_state_dict", ckpt)
        meta = ckpt.get("metadata", {})
        self._model = RawContextClassifier(
            input_dim=meta.get("input_dim", CONTEXT_DIM),
            hidden=meta.get("hidden", 128),
            num_classes=meta.get("num_classes", NUM_CLASSES),
        )
        self._model.load_state_dict(state_dict)
        self._model.eval()
        self._target_poses = {
            int(k): v for k, v in meta.get("target_poses", {}).items()
        }
        if "scaler_min" in meta:
            self._scaler_min = np.array(meta["scaler_min"], dtype=np.float32)
            self._scaler_max = np.array(meta["scaler_max"], dtype=np.float32)
            self._scaler_range = np.array(meta["scaler_range"], dtype=np.float32)

    def predict(self, context: np.ndarray | list[float]) -> ActionCommand:
        """Run inference on a 68-dim context vector and apply safety gating."""
        ctx = np.asarray(context, dtype=np.float32)
        if ctx.shape != (CONTEXT_DIM,):
            raise ValueError(f"context must be ({CONTEXT_DIM},), got {ctx.shape}")

        if self._model is None:
            return ActionCommand(
                phase=0, phase_name="UNKNOWN", confidence=0.0,
                target_joints=[0.0] * 6, use_fallback=True,
                fallback_reason="no_model_loaded",
            )

        if self._scaler_min is not None and self._scaler_range is not None:
            ctx = ctx.copy()
            for i in range(48, 54):
                ctx[i] = min(ctx[i], 10.0)
                ctx[i] = np.log1p(max(0.0, ctx[i]))
            ctx = (ctx - self._scaler_min) / self._scaler_range

        with torch.no_grad():
            x = torch.tensor(ctx, dtype=torch.float32).unsqueeze(0)
            logits = self._model(x)
            probs = torch.softmax(logits, dim=-1).squeeze(0)

        confidence, pred_class = probs.max(dim=-1)
        confidence = confidence.item()
        pred_class = pred_class.item()
        raw_logits = probs.tolist()

        phase_id = self._class_to_phase(pred_class)
        phase_name = self._phase_names.get(phase_id, f"UNKNOWN_{phase_id}")

        target_joints = self._target_poses.get(pred_class, [0.0] * 6)

        fallback, reason = self._check_safety_gate(
            phase_id, confidence, raw_logits
        )

        return ActionCommand(
            phase=phase_id,
            phase_name=phase_name,
            confidence=round(confidence, 4),
            target_joints=target_joints,
            use_fallback=fallback,
            fallback_reason=reason,
            raw_logits=[round(x, 6) for x in raw_logits],
        )

    def _class_to_phase(self, class_idx: int) -> int:
        mapping = {0: 0, 1: 1, 2: 2, 3: 3, 4: 5, 5: 6, 6: 7}
        return mapping.get(class_idx, 0)

    def _check_safety_gate(
        self, phase_id: int, confidence: float, logits: list[float]
    ) -> tuple[bool, str]:
        if confidence < self.confidence_threshold:
            return True, f"low_confidence={confidence:.3f}"

        if phase_id in SAFETY_CRITICAL_PHASES:
            sorted_logits = sorted(logits, reverse=True)
            margin = sorted_logits[0] - sorted_logits[1]
            if margin < 0.3:
                return True, f"safety_critical_low_margin={margin:.3f}"

        if phase_id == PhaseID.INSERT:
            return True, "insert_phase_defers_to_deterministic"

        return False, ""


def train_raw_context_classifier(
    parquet_path: str | Path,
    output_dir: str | Path,
    epochs: int = 100,
    hidden: int = 128,
    dropout: float = 0.2,
    lr: float = 0.001,
    seed: int = 0,
) -> dict[str, Any]:
    """Train a raw 68-dim context classifier on 6-phase data."""
    import pandas as pd
    from sklearn.metrics import classification_report, confusion_matrix

    torch.manual_seed(seed)
    np.random.seed(seed)

    df = pd.read_parquet(parquet_path)

    if "context_vec" in df.columns:
        context = np.stack(df["context_vec"].values).astype(np.float32)
        phases = df["phase_int"].values.astype(int)
    elif "task_phase" in df.columns:
        phases = df["task_phase"].values.astype(int)
        exclude = {"joint_1_pos_rad", "joint_2_pos_rad", "joint_3_pos_rad",
                   "joint_4_pos_rad", "joint_5_pos_rad", "joint_6_pos_rad",
                   "joint_1_vel_rad_s", "joint_2_vel_rad_s", "joint_3_vel_rad_s",
                   "joint_4_vel_rad_s", "joint_5_vel_rad_s", "joint_6_vel_rad_s",
                   "task_phase", "safety_status", "stamp_s", "tick_index"}
        feat_cols = [c for c in df.columns if c not in exclude]
        context = df[feat_cols].values.astype(np.float32)
        context = context[:, :CONTEXT_DIM]
    else:
        raise ValueError(f"Unknown parquet format: columns={list(df.columns)}")

    unique_phases = sorted(set(phases))
    phase_to_class = {p: i for i, p in enumerate(unique_phases)}
    class_to_phase = {i: p for p, i in phase_to_class.items()}
    y = np.array([phase_to_class[p] for p in phases], dtype=np.int64)
    num_classes = len(unique_phases)

    context = context.astype(np.float32)
    depth_indices = list(range(48, 54))
    for i in depth_indices:
        context[:, i] = np.clip(context[:, i], 0.0, 10.0)
        context[:, i] = np.log1p(context[:, i])
    mn = context.min(axis=0)
    mx = context.max(axis=0)
    rng = mx - mn
    rng[rng < 1e-8] = 1.0
    context = (context - mn) / rng

    n_total = len(context)
    n_train = int(n_total * 0.8)
    indices = np.random.permutation(n_total)
    train_idx = indices[:n_train]
    test_idx = indices[n_train:]

    X_train = torch.tensor(context[train_idx], dtype=torch.float32)
    y_train = torch.tensor(y[train_idx], dtype=torch.long)
    X_test = torch.tensor(context[test_idx], dtype=torch.float32)
    y_test = torch.tensor(y[test_idx], dtype=torch.long)

    train_ds = TensorDataset(X_train, y_train)
    train_loader = DataLoader(train_ds, batch_size=64, shuffle=True)

    model = RawContextClassifier(
        input_dim=CONTEXT_DIM, hidden=hidden, num_classes=num_classes, dropout=dropout,
    )
    class_counts = np.bincount(y, minlength=num_classes).astype(np.float64)
    class_weights = len(y) / (num_classes * np.maximum(class_counts, 1))
    class_weights_tensor = torch.tensor(class_weights, dtype=torch.float32)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    criterion = nn.CrossEntropyLoss(weight=class_weights_tensor)

    history = []
    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        for xb, yb in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(xb), yb)
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * len(xb)
        model.eval()
        with torch.no_grad():
            train_logits = model(X_train)
            train_pred = train_logits.argmax(dim=-1)
            train_acc = (train_pred == y_train).float().mean().item()
            test_logits = model(X_test)
            test_pred = test_logits.argmax(dim=-1)
            test_acc = (test_pred == y_test).float().mean().item()
            test_ce = criterion(test_logits, y_test).item()
        history.append({
            "epoch": epoch + 1, "train_acc": train_acc,
            "test_acc": test_acc, "test_ce": test_ce,
        })

    model.eval()
    with torch.no_grad():
        test_logits = model(X_test)
        test_pred = test_logits.argmax(dim=-1).numpy()

    y_test_np = y_test.numpy()
    target_names = [class_to_phase.get(i, f"class_{i}") for i in range(num_classes)]
    report = classification_report(y_test_np, test_pred, target_names=target_names, output_dict=True)
    cm = confusion_matrix(y_test_np, test_pred).tolist()

    target_poses = {}
    for cls_idx in range(num_classes):
        phase_id = class_to_phase.get(cls_idx, 0)
        mask = phases == phase_id
        if mask.any():
            target_poses[cls_idx] = context[mask, 54:60].mean(axis=0).tolist()

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = output_dir / "raw_context_classifier.pt"
    torch.save({
        "model_state_dict": model.state_dict(),
        "metadata": {
            "input_dim": CONTEXT_DIM, "hidden": hidden,
            "num_classes": num_classes,
            "class_to_phase": {int(k): int(v) for k, v in class_to_phase.items()},
            "phase_to_class": {int(k): int(v) for k, v in phase_to_class.items()},
            "target_poses": {int(k): v for k, v in target_poses.items()},
            "scaler_min": mn.tolist(),
            "scaler_max": mx.tolist(),
            "scaler_range": rng.tolist(),
            "epochs": epochs,
            "final_test_acc": report["accuracy"],
            "phase_counts": {str(p): int((phases == p).sum()) for p in unique_phases},
        },
    }, model_path)

    metadata = {
        "model_type": "raw_context_classifier",
        "input_dim": CONTEXT_DIM, "hidden": hidden,
        "num_classes": num_classes, "accuracy": float(report["accuracy"]),
        "confusion_matrix": cm,
        "class_to_phase": {str(k): int(v) for k, v in class_to_phase.items()},
        "phase_to_class": {str(k): int(v) for k, v in phase_to_class.items()},
        "model_path": str(model_path),
        "target_poses": {str(k): [float(x) for x in v] for k, v in target_poses.items()},
    }
    (output_dir / "raw_classifier_metadata.json").write_text(
        json.dumps(metadata, indent=2, default=str), encoding="utf-8"
    )
    print(f"Trained raw context classifier: test_acc={report['accuracy']:.4f}")
    print(f"Model: {model_path}")
    print(f"Metadata: {output_dir / 'raw_classifier_metadata.json'}")
    return metadata


def main(args: list[str] | None = None) -> None:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-parquet", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--hidden", type=int, default=128)
    parser.add_argument("--dropout", type=float, default=0.2)
    parser.add_argument("--lr", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=0)
    a = parser.parse_args(args)

    train_raw_context_classifier(
        parquet_path=a.input_parquet, output_dir=a.output_dir,
        epochs=a.epochs, hidden=a.hidden, dropout=a.dropout,
        lr=a.lr, seed=a.seed,
    )


if __name__ == "__main__":
    main()

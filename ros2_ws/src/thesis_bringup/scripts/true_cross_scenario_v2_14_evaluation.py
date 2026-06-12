#!/usr/bin/env python3
"""Comprehensive cross-scenario v2_14 evaluation script.

Evaluates the v2_14 context classifier across 7 geometry/tolerance scenarios
using row-level 68-dim context vectors from multi-scenario Stage C trials.

Evaluation modes:
  A. Mixed-scenario random split (80/20 train/test)
  B. Held-out scenario evaluation (leave-one-scenario-out)
  C. Feasibility classifier (geometry-only, 3-class envelope prediction)
  D. Safety-gated v2_14 advisory mode (ML + feasibility + deterministic rules)

The v2_14 classifier is a 68 -> 128 -> 128 -> 7 neural network that predicts
task phase from raw context vectors. The feasibility classifier predicts
in-envelope / marginal / out-of-envelope from geometry parameters only.

SAFETY CONSTRAINT: This classifier is advisory-only. INSERT is always deferred
to the deterministic controller. DONE is never trusted from ML.
"""
import json
import sys
import time
import warnings
from collections import defaultdict
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

try:
    from sklearn.metrics import (
        accuracy_score,
        classification_report,
        confusion_matrix,
        f1_score,
        precision_recall_fscore_support,
    )
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False

warnings.filterwarnings("ignore", category=FutureWarning)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DATASET_PATH = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "src"
    / "diagnostics"
    / "multi_scenario_row_level_dataset"
    / "multi_scenario_context_vectors.parquet"
)
OUTPUT_DIR = (
    Path(__file__).resolve().parent.parent.parent.parent
    / "diagnostics"
    / "multi_scenario_row_level_dataset"
)
OUTPUT_FILE = OUTPUT_DIR / "cross_scenario_v2_14_evaluation.json"

CONTEXT_DIM = 68
HIDDEN_DIM = 128
OUTPUT_CLASSES = 7
EPOCHS = 15
LEARNING_RATE = 2e-3
BATCH_SIZE = 512
TRAIN_SPLIT = 0.8
DEVICE = "cpu"

# Phase mapping: phase_name -> integer label
PHASE_MAP = {
    "UNKNOWN": 0,
    "MOVING_TO_START": 1,
    "APPROACH": 2,
    "SEARCH": 3,
    "INSERT": 5,
    "RETREAT": 6,
    "DONE": 7,
}
PHASE_NAMES = {v: k for k, v in PHASE_MAP.items()}
PHASE_LIST = [0, 1, 2, 3, 5, 6, 7]

# Geometry-only feasibility: clearance / noise thresholds
NOISE_FLOOR_MM = 0.5
FEASIBILITY_CLASSES = {"robust": 0, "marginal": 1, "out_of_envelope": 2}
FEASIBILITY_NAMES = {0: "in_envelope", 1: "marginal", 2: "out_of_envelope"}


# ---------------------------------------------------------------------------
# Neural network (v2_14 architecture)
# ---------------------------------------------------------------------------
class V2_14Classifier(nn.Module):
    """68 -> 128 -> 128 -> 7 classifier matching v2_14 architecture."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(CONTEXT_DIM, HIDDEN_DIM),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(HIDDEN_DIM, HIDDEN_DIM),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(HIDDEN_DIM, OUTPUT_CLASSES),
        )

    def forward(self, x):
        return self.net(x)

    def predict_proba_tensor(self, x):
        self.eval()
        with torch.no_grad():
            logits = self.forward(x)
            return torch.softmax(logits, dim=-1)


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_dataset(path: Path) -> pd.DataFrame:
    """Load multi-scenario row-level parquet dataset."""
    df = pd.read_parquet(path)
    print(f"Loaded {len(df)} rows from {path.name}")
    print(f"Scenarios: {sorted(df['scenario_id'].unique())}")
    print(f"Phase distribution:")
    for phase, count in df["phase_name"].value_counts().sort_index().items():
        print(f"  {phase}: {count}")
    return df


def extract_arrays(df: pd.DataFrame):
    """Extract numpy arrays from DataFrame.

    Returns:
        X: (N, 68) float32 array of context vectors
        y_phase: (N,) int array of phase labels
        y_feasibility: (N,) int array of feasibility labels
        meta: DataFrame with scenario_id, trial_id, geometry params
    """
    # Convert context_vec column to proper numpy array
    context_list = []
    for idx in range(len(df)):
        vec = df["context_vec"].iloc[idx]
        if isinstance(vec, np.ndarray):
            context_list.append(vec.astype(np.float32))
        else:
            context_list.append(np.array(vec, dtype=np.float32))
    X = np.stack(context_list, axis=0)

    # Phase labels
    y_phase = np.array(
        [PHASE_MAP.get(name, -1) for name in df["phase_name"]],
        dtype=np.int64,
    )

    # Feasibility labels from envelope_zone
    y_feasibility = np.array(
        [FEASIBILITY_CLASSES.get(zone, -1) for zone in df["envelope_zone"]],
        dtype=np.int64,
    )

    meta = df[
        [
            "scenario_id",
            "trial_id",
            "peg_diameter_mm",
            "hole_diameter_mm",
            "radial_clearance_mm",
            "initial_xy_offset_mm",
            "envelope_zone",
            "feasibility_label",
            "trial_success",
            "phase_name",
        ]
    ].copy()

    return X, y_phase, y_feasibility, meta


# ---------------------------------------------------------------------------
# Training helpers
# ---------------------------------------------------------------------------
def compute_class_weights(y: np.ndarray, num_classes: int) -> torch.Tensor:
    """Compute inverse-frequency class weights for cross-entropy loss."""
    counts = np.bincount(y, minlength=num_classes).astype(np.float64)
    counts[counts == 0] = 1.0
    weights = 1.0 / counts
    weights = weights / weights.sum() * num_classes
    return torch.tensor(weights, dtype=torch.float32, device=DEVICE)


def train_v2_14(X_train, y_train, X_test, y_test, num_classes=7):
    """Train v2_14 classifier using PyTorch.

    Returns:
        model: trained model
        train_history: list of epoch losses
        test_accs: list of per-epoch test accuracies
    """
    model = V2_14Classifier().to(DEVICE)

    # Filter to valid classes only
    valid_mask_train = np.isin(y_train, PHASE_LIST)
    valid_mask_test = np.isin(y_test, PHASE_LIST)

    X_train_t = torch.tensor(X_train[valid_mask_train], dtype=torch.float32, device=DEVICE)
    y_train_t = torch.tensor(y_train[valid_mask_train], dtype=torch.long, device=DEVICE)
    X_test_t = torch.tensor(X_test[valid_mask_test], dtype=torch.float32, device=DEVICE)
    y_test_t = torch.tensor(y_test[valid_mask_test], dtype=torch.long, device=DEVICE)

    # Remap labels to contiguous indices for loss computation
    label_remap = {old: i for i, old in enumerate(PHASE_LIST)}
    y_train_mapped = torch.tensor(
        [label_remap[v.item()] for v in y_train_t], dtype=torch.long, device=DEVICE
    )
    y_test_mapped = torch.tensor(
        [label_remap[v.item()] for v in y_test_t], dtype=torch.long, device=DEVICE
    )

    # Class weights
    class_weights = compute_class_weights(y_train_mapped.cpu().numpy(), num_classes)
    criterion = nn.CrossEntropyLoss(weight=class_weights)
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE, weight_decay=1e-4)

    dataset = TensorDataset(X_train_t, y_train_mapped)
    loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

    train_history = []
    test_accs = []

    for epoch in range(EPOCHS):
        model.train()
        epoch_loss = 0.0
        n_batches = 0
        for xb, yb in loader:
            optimizer.zero_grad()
            logits = model(xb)
            loss = criterion(logits, yb)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item()
            n_batches += 1
        avg_loss = epoch_loss / max(n_batches, 1)
        train_history.append(avg_loss)

        # Evaluate
        model.eval()
        with torch.no_grad():
            test_logits = model(X_test_t)
            test_preds = test_logits.argmax(dim=1)
            test_acc = (test_preds == y_test_mapped).float().mean().item()
            test_accs.append(test_acc)

        if (epoch + 1) % 10 == 0:
            print(
                f"  Epoch {epoch+1:3d}/{EPOCHS}  loss={avg_loss:.4f}  "
                f"test_acc={test_acc:.4f}"
            )

    return model, label_remap, train_history, test_accs


def train_sklearn_fallback(X_train, y_train, X_test, y_test):
    """Fallback training using sklearn MLPClassifier when torch is unavailable."""
    valid_mask = np.isin(y_train, PHASE_LIST)
    X_tr = X_train[valid_mask]
    y_tr = y_train[valid_mask]

    scaler = StandardScaler()
    X_tr_s = scaler.fit_transform(X_tr)
    X_te_s = scaler.transform(X_test)

    clf = MLPClassifier(
        hidden_layer_sizes=(128, 128),
        activation="relu",
        max_iter=EPOCHS,
        random_state=42,
        early_stopping=True,
        validation_fraction=0.15,
    )
    clf.fit(X_tr_s, y_tr)
    return clf, scaler


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------
def compute_metrics(y_true, y_pred, phase_names_map=PHASE_NAMES, labels=None):
    """Compute comprehensive classification metrics.

    Returns dict with accuracy, macro F1, weighted F1, per-class report,
    confusion matrix, and key recall values.
    """
    if labels is None:
        labels = sorted(set(y_true) | set(y_pred))

    acc = accuracy_score(y_true, y_pred)
    macro_f1 = f1_score(y_true, y_pred, average="macro", zero_division=0)
    weighted_f1 = f1_score(y_true, y_pred, average="weighted", zero_division=0)

    # Per-class precision/recall/f1
    prec, rec, f1, sup = precision_recall_fscore_support(
        y_true, y_pred, labels=labels, zero_division=0
    )
    per_class = {}
    for i, lab in enumerate(labels):
        name = phase_names_map.get(lab, str(lab))
        per_class[name] = {
            "precision": round(float(prec[i]), 4),
            "recall": round(float(rec[i]), 4),
            "f1": round(float(f1[i]), 4),
            "support": int(sup[i]),
        }

    # Confusion matrix
    cm = confusion_matrix(y_true, y_pred, labels=labels)
    cm_list = cm.tolist()

    # Key recalls
    search_recall = per_class.get("SEARCH", {}).get("recall", 0.0)
    retreat_recall = per_class.get("RETREAT", {}).get("recall", 0.0)
    done_recall = per_class.get("DONE", {}).get("recall", 0.0)

    return {
        "accuracy": round(float(acc), 4),
        "macro_f1": round(float(macro_f1), 4),
        "weighted_f1": round(float(weighted_f1), 4),
        "per_class": per_class,
        "confusion_matrix": cm_list,
        "labels": [phase_names_map.get(l, str(l)) for l in labels],
        "search_recall": round(float(search_recall), 4),
        "retreat_recall": round(float(retreat_recall), 4),
        "done_recall": round(float(done_recall), 4),
    }


# ---------------------------------------------------------------------------
# Mode A: Mixed-scenario random split
# ---------------------------------------------------------------------------
def run_mode_a(X, y_phase, meta, scenarios):
    """Mixed-scenario random split evaluation (80/20)."""
    print("\n" + "=" * 70)
    print("MODE A: Mixed-Scenario Random Split (80/20 train/test)")
    print("=" * 70)

    np.random.seed(42)
    n = len(X)
    indices = np.random.permutation(n)
    split = int(n * TRAIN_SPLIT)
    train_idx = indices[:split]
    test_idx = indices[split:]

    X_train, X_test = X[train_idx], X[test_idx]
    y_train, y_test = y_phase[train_idx], y_phase[test_idx]

    print(f"Train: {len(X_train)} rows  |  Test: {len(X_test)} rows")
    print(f"Train scenarios: {sorted(meta.iloc[train_idx]['scenario_id'].unique())}")
    print(f"Test scenarios:  {sorted(meta.iloc[test_idx]['scenario_id'].unique())}")

    if HAS_TORCH:
        print("\nTraining v2_14 (PyTorch)...")
        model, label_remap, history, test_accs = train_v2_14(
            X_train, y_train, X_test, y_test, num_classes=len(PHASE_LIST)
        )

        # Predict on test set (only valid-phase rows)
        valid_mask_test = np.isin(y_test, PHASE_LIST)
        X_test_t = torch.tensor(X_test[valid_mask_test], dtype=torch.float32, device=DEVICE)
        y_test_valid = y_test[valid_mask_test]
        model.eval()
        with torch.no_grad():
            preds_logits = model(X_test_t)
            preds_mapped = preds_logits.argmax(dim=1).cpu().numpy()
        inv_remap = {i: v for v, i in label_remap.items()}
        y_pred = np.array([inv_remap[p] for p in preds_mapped], dtype=np.int64)

        metrics = compute_metrics(y_test_valid, y_pred)
        metrics["training_loss_curve"] = [round(h, 4) for h in history]
        metrics["test_accuracy_per_epoch"] = [round(a, 4) for a in test_accs]
        print(f"\nFinal test accuracy: {metrics['accuracy']:.4f}")
        print(f"Macro F1:            {metrics['macro_f1']:.4f}")
        print(f"Weighted F1:         {metrics['weighted_f1']:.4f}")
        print(f"SEARCH recall:       {metrics['search_recall']:.4f}")
        print(f"RETREAT recall:      {metrics['retreat_recall']:.4f}")
        print(f"DONE recall:         {metrics['done_recall']:.4f}")
    elif HAS_SKLEARN:
        print("\nTraining fallback (sklearn MLPClassifier)...")
        clf, scaler = train_sklearn_fallback(X_train, y_train, X_test, y_test)
        y_pred = clf.predict(scaler.transform(X_test))
        valid_mask = np.isin(y_test, PHASE_LIST)
        metrics = compute_metrics(y_test[valid_mask], y_pred[valid_mask])
        print(f"\nFinal test accuracy: {metrics['accuracy']:.4f}")
        print(f"Macro F1:            {metrics['macro_f1']:.4f}")
    else:
        print("\nERROR: Neither torch nor sklearn available. Cannot train.")
        return {}

    # Per-scenario breakdown
    test_meta = meta.iloc[test_idx].iloc[valid_mask_test].copy()
    test_meta = test_meta.reset_index(drop=True)
    per_scenario = {}
    for sc in sorted(test_meta["scenario_id"].unique()):
        mask = test_meta["scenario_id"].values == sc
        sc_acc = accuracy_score(y_test_valid[mask], y_pred[mask])
        per_scenario[sc] = {
            "accuracy": round(float(sc_acc), 4),
            "n_rows": int(mask.sum()),
        }
        print(f"  {sc}: accuracy={sc_acc:.4f}  (n={mask.sum()})")

    metrics["per_scenario_accuracy"] = per_scenario
    metrics["train_rows"] = len(X_train)
    metrics["test_rows"] = len(X_test)
    return metrics


# ---------------------------------------------------------------------------
# Mode B: Held-out scenario evaluation
# ---------------------------------------------------------------------------
def run_mode_b(X, y_phase, meta, scenarios):
    """Held-out scenario evaluation (leave-one-out)."""
    print("\n" + "=" * 70)
    print("MODE B: Held-Out Scenario Evaluation")
    print("=" * 70)

    results = {}
    for held_out in scenarios:
        print(f"\n--- Held out: {held_out} ---")
        train_mask = meta["scenario_id"].values != held_out
        test_mask = meta["scenario_id"].values == held_out

        X_train, X_test = X[train_mask], X[test_mask]
        y_train, y_test = y_phase[train_mask], y_phase[test_mask]

        print(
            f"Train: {len(X_train)} rows ({len(scenarios)-1} scenarios)  |  "
            f"Test: {len(X_test)} rows"
        )

        if HAS_TORCH:
            model, label_remap, history, test_accs = train_v2_14(
                X_train, y_train, X_test, y_test, num_classes=len(PHASE_LIST)
            )
            valid_mask_test = np.isin(y_test, PHASE_LIST)
            X_test_t = torch.tensor(
                X_test[valid_mask_test], dtype=torch.float32, device=DEVICE
            )
            y_test_valid = y_test[valid_mask_test]
            model.eval()
            with torch.no_grad():
                preds_mapped = model(X_test_t).argmax(dim=1).cpu().numpy()
            inv_remap = {i: v for v, i in label_remap.items()}
            y_pred = np.array([inv_remap[p] for p in preds_mapped], dtype=np.int64)
            metrics = compute_metrics(y_test_valid, y_pred)
        elif HAS_SKLEARN:
            clf, scaler = train_sklearn_fallback(X_train, y_train, X_test, y_test)
            y_pred = clf.predict(scaler.transform(X_test))
            valid_mask = np.isin(y_test, PHASE_LIST)
            metrics = compute_metrics(y_test[valid_mask], y_pred[valid_mask])
        else:
            metrics = {}

        results[held_out] = metrics
        if metrics:
            print(f"  Accuracy:     {metrics['accuracy']:.4f}")
            print(f"  Macro F1:     {metrics['macro_f1']:.4f}")
            print(f"  Weighted F1:  {metrics['weighted_f1']:.4f}")
            print(f"  SEARCH recall: {metrics['search_recall']:.4f}")
            print(f"  RETREAT recall: {metrics['retreat_recall']:.4f}")
            print(f"  DONE recall:   {metrics['done_recall']:.4f}")

    # Aggregate held-out metrics
    if results:
        accs = [r["accuracy"] for r in results.values() if "accuracy" in r]
        f1s = [r["macro_f1"] for r in results.values() if "macro_f1" in r]
        summary = {
            "mean_accuracy": round(float(np.mean(accs)), 4) if accs else 0.0,
            "std_accuracy": round(float(np.std(accs)), 4) if accs else 0.0,
            "min_accuracy": round(float(np.min(accs)), 4) if accs else 0.0,
            "max_accuracy": round(float(np.max(accs)), 4) if accs else 0.0,
            "mean_macro_f1": round(float(np.mean(f1s)), 4) if f1s else 0.0,
            "worst_scenario": min(results, key=lambda k: results[k].get("accuracy", 1.0)),
        }
        results["_summary"] = summary
        print(f"\n--- Held-Out Summary ---")
        print(f"  Mean accuracy:   {summary['mean_accuracy']:.4f} +/- {summary['std_accuracy']:.4f}")
        print(f"  Min accuracy:    {summary['min_accuracy']:.4f} ({summary['worst_scenario']})")
        print(f"  Max accuracy:    {summary['max_accuracy']:.4f}")
        print(f"  Mean macro F1:   {summary['mean_macro_f1']:.4f}")

    return results


# ---------------------------------------------------------------------------
# Mode C: Feasibility classifier (geometry-only)
# ---------------------------------------------------------------------------
def run_mode_c(X, y_feasibility, meta, scenarios):
    """Feasibility classifier using geometry-only features."""
    print("\n" + "=" * 70)
    print("MODE C: Feasibility Classifier (Geometry-Only)")
    print("=" * 70)

    # Extract geometry features: clearance_mm, offset_mm
    clearance = meta["radial_clearance_mm"].values.astype(np.float32)
    offset = meta["initial_xy_offset_mm"].values.astype(np.float32)
    X_geom = np.column_stack([clearance, offset])

    y_true = y_feasibility

    # Rule-based classifier (same logic as existing FeasibilityClassifier)
    y_pred = np.zeros(len(X_geom), dtype=np.int64)
    for i in range(len(X_geom)):
        c = X_geom[i, 0]
        o = X_geom[i, 1]
        ratio = c / NOISE_FLOOR_MM
        adjusted = ratio - (o / NOISE_FLOOR_MM)
        if adjusted >= 2.0:
            y_pred[i] = 0  # robust -> in_envelope
        elif adjusted >= 1.0:
            y_pred[i] = 1  # marginal
        else:
            y_pred[i] = 2  # out_of_envelope

    labels = [0, 1, 2]
    acc = accuracy_score(y_true, y_pred)
    prec, rec, f1, sup = precision_recall_fscore_support(
        y_true, y_pred, labels=labels, zero_division=0
    )
    cm = confusion_matrix(y_true, y_pred, labels=labels)

    # False-safe rate: predicted in_envelope (0) but actually out_of_envelope (2)
    false_safe = int(np.sum((y_pred == 0) & (y_true == 2)))
    false_safe_rate = false_safe / max(int(np.sum(y_true == 2)), 1)

    # Fail-closed detection: predicted out_of_envelope (2) and actually out_of_envelope (2)
    fail_closed = int(np.sum((y_pred == 2) & (y_true == 2)))
    fail_closed_detection = fail_closed / max(int(np.sum(y_true == 2)), 1)

    # False-block rate: predicted out_of_envelope but actually in_envelope
    false_block = int(np.sum((y_pred == 2) & (y_true == 0)))
    false_block_rate = false_block / max(int(np.sum(y_true == 0)), 1)

    # Also train sklearn MLPClassifier on geometry features for comparison
    if HAS_SKLEARN:
        np.random.seed(42)
        n = len(X_geom)
        indices = np.random.permutation(n)
        split = int(n * TRAIN_SPLIT)
        train_idx = indices[:split]
        test_idx = indices[split:]

        scaler = StandardScaler()
        X_tr = scaler.fit_transform(X_geom[train_idx])
        X_te = scaler.transform(X_geom[test_idx])

        clf = MLPClassifier(
            hidden_layer_sizes=(32, 32),
            activation="relu",
            max_iter=500,
            random_state=42,
        )
        clf.fit(X_tr, y_true[train_idx])
        y_pred_ml = clf.predict(X_te)
        ml_acc = accuracy_score(y_true[test_idx], y_pred_ml)
        ml_prec, ml_rec, ml_f1, _ = precision_recall_fscore_support(
            y_true[test_idx], y_pred_ml, labels=labels, zero_division=0
        )
        ml_cm = confusion_matrix(y_true[test_idx], y_pred_ml, labels=labels)
        ml_false_safe = int(np.sum((y_pred_ml == 0) & (y_true[test_idx] == 2)))
        ml_fail_closed = int(np.sum((y_pred_ml == 2) & (y_true[test_idx] == 2)))
    else:
        ml_acc = 0.0
        ml_cm = []

    per_class = {}
    for i, lab in enumerate(labels):
        per_class[FEASIBILITY_NAMES[lab]] = {
            "precision": round(float(prec[i]), 4),
            "recall": round(float(rec[i]), 4),
            "f1": round(float(f1[i]), 4),
            "support": int(sup[i]),
        }

    results = {
        "rule_based": {
            "accuracy": round(float(acc), 4),
            "per_class": per_class,
            "confusion_matrix": cm.tolist(),
            "false_safe_count": false_safe,
            "false_safe_rate": round(float(false_safe_rate), 4),
            "fail_closed_detection_count": fail_closed,
            "fail_closed_detection_rate": round(float(fail_closed_detection), 4),
            "false_block_count": false_block,
            "false_block_rate": round(float(false_block_rate), 4),
        },
    }

    if HAS_SKLEARN:
        ml_per_class = {}
        for i, lab in enumerate(labels):
            ml_per_class[FEASIBILITY_NAMES[lab]] = {
                "precision": round(float(ml_prec[i]), 4),
                "recall": round(float(ml_rec[i]), 4),
                "f1": round(float(ml_f1[i]), 4),
            }
        results["ml_classifier"] = {
            "accuracy": round(float(ml_acc), 4),
            "per_class": ml_per_class,
            "confusion_matrix": ml_cm.tolist(),
            "false_safe_count": ml_false_safe,
            "fail_closed_detection_count": ml_fail_closed,
        }

    # Per-scenario breakdown
    per_scenario = {}
    for sc in sorted(scenarios):
        mask = meta["scenario_id"].values == sc
        sc_acc = accuracy_score(y_true[mask], y_pred[mask])
        sc_fs = int(np.sum((y_pred[mask] == 0) & (y_true[mask] == 2)))
        per_scenario[sc] = {
            "accuracy": round(float(sc_acc), 4),
            "false_safe_count": sc_fs,
            "n_rows": int(mask.sum()),
        }

    results["per_scenario"] = per_scenario
    results["total_rows"] = len(y_true)

    print(f"\nRule-based classifier:")
    print(f"  Accuracy:            {results['rule_based']['accuracy']:.4f}")
    print(f"  False-safe rate:     {results['rule_based']['false_safe_rate']:.4f} ({false_safe} cases)")
    print(f"  Fail-closed detect:  {results['rule_based']['fail_closed_detection_rate']:.4f}")
    print(f"  False-block rate:    {results['rule_based']['false_block_rate']:.4f}")
    for name, pc in per_class.items():
        print(f"  {name:20s} P={pc['precision']:.3f}  R={pc['recall']:.3f}  F1={pc['f1']:.3f}")

    if HAS_SKLEARN:
        print(f"\nML classifier (sklearn MLPClassifier):")
        print(f"  Accuracy:            {results['ml_classifier']['accuracy']:.4f}")
        print(f"  False-safe count:    {results['ml_classifier']['false_safe_count']}")
        print(f"  Fail-closed detect:  {results['ml_classifier']['fail_closed_detection_count']}")

    print(f"\nPer-scenario accuracy:")
    for sc, data in per_scenario.items():
        print(f"  {sc:30s} acc={data['accuracy']:.4f}  fs={data['false_safe_count']}  n={data['n_rows']}")

    return results


# ---------------------------------------------------------------------------
# Mode D: Safety-gated v2_14 advisory mode
# ---------------------------------------------------------------------------
def run_mode_d(X, y_phase, y_feasibility, meta, scenarios):
    """Safety-gated advisory mode: v2_14 + feasibility + deterministic rules.

    Rules:
    - INSERT (phase 5) is ALWAYS deferred to the deterministic controller
    - DONE (phase 7) is NEVER trusted from ML prediction
    - If feasibility classifier says out_of_envelope, override to RETREAT
    - If ML confidence < threshold, fall back to deterministic
    """
    print("\n" + "=" * 70)
    print("MODE D: Safety-Gated v2_14 Advisory Mode")
    print("=" * 70)

    np.random.seed(42)
    n = len(X)
    indices = np.random.permutation(n)
    split = int(n * TRAIN_SPLIT)
    train_idx = indices[:split]
    test_idx = indices[split:]

    X_train, X_test = X[train_idx], X[test_idx]
    y_train, y_test = y_phase[train_idx], y_phase[test_idx]
    feas_test = y_feasibility[test_idx]
    meta_test = meta.iloc[test_idx].reset_index(drop=True)

    confidence_threshold = 0.7

    if HAS_TORCH:
        model, label_remap, history, test_accs = train_v2_14(
            X_train, y_train, X_test, y_test, num_classes=len(PHASE_LIST)
        )

        valid_mask_test = np.isin(y_test, PHASE_LIST)
        X_test_t = torch.tensor(
            X_test[valid_mask_test], dtype=torch.float32, device=DEVICE
        )
        y_test_valid = y_test[valid_mask_test]
        feas_valid = feas_test[valid_mask_test]
        meta_valid = meta_test.iloc[valid_mask_test].reset_index(drop=True)

        model.eval()
        with torch.no_grad():
            probs = model.predict_proba_tensor(X_test_t).cpu().numpy()
            raw_preds = probs.argmax(axis=1)
            confs = probs.max(axis=1)

        inv_remap = {i: v for v, i in label_remap.items()}
        y_raw = np.array([inv_remap[p] for p in raw_preds], dtype=np.int64)

        # Apply safety gates (rules applied in priority order, each row modified at most once)
        y_advisory = y_raw.copy()
        insert_deferred = 0
        done_blocked = 0
        feasibility_override = 0
        low_confidence_fallback = 0

        for i in range(len(y_advisory)):
            phase = y_raw[i]
            conf = confs[i]
            feas = feas_valid[i]

            # Rule 1 (highest priority): INSERT always deferred to deterministic
            if phase == 5:
                y_advisory[i] = 1  # Defer to MOVING_TO_START, deterministic handles INSERT
                insert_deferred += 1

            # Rule 2: DONE never trusted from ML
            elif phase == 7:
                y_advisory[i] = 6  # Override to RETREAT, deterministic decides actual DONE
                done_blocked += 1

            # Rule 3: Out-of-envelope forces RETREAT
            elif feas == 2:
                y_advisory[i] = 6  # Force RETREAT for out-of-envelope geometry
                feasibility_override += 1

            # Rule 4: Low confidence triggers deterministic fallback
            elif conf < confidence_threshold:
                y_advisory[i] = 1  # Fall back to MOVING_TO_START
                low_confidence_fallback += 1

        # Count unique rows that were modified from raw ML prediction
        fallback_count = int(np.sum(y_advisory != y_raw))

        # Compute metrics on advisory predictions
        metrics = compute_metrics(y_test_valid, y_advisory)
        metrics["safety_gating"] = {
            "confidence_threshold": confidence_threshold,
            "fallback_count": int(fallback_count),
            "fallback_rate": round(float(fallback_count / len(y_advisory)), 4),
            "insert_deferred_to_deterministic": int(insert_deferred),
            "done_blocked_from_ml": int(done_blocked),
            "feasibility_override_to_retreat": int(feasibility_override),
            "low_confidence_fallback": int(low_confidence_fallback),
            "unsafe_advice_blocked": int(insert_deferred + done_blocked),
        }
        metrics["raw_ml_accuracy"] = round(float(accuracy_score(y_test_valid, y_raw)), 4)
        metrics["advisory_accuracy"] = metrics["accuracy"]
        metrics["training_loss_curve"] = [round(h, 4) for h in history]
        metrics["test_accuracy_per_epoch"] = [round(a, 4) for a in test_accs]

        print(f"\nRaw ML accuracy (no safety gates): {metrics['raw_ml_accuracy']:.4f}")
        print(f"Advisory accuracy (with safety gates): {metrics['advisory_accuracy']:.4f}")
        print(f"\nSafety gating summary:")
        print(f"  Total fallbacks:            {fallback_count}")
        print(f"  Fallback rate:              {metrics['safety_gating']['fallback_rate']:.4f}")
        print(f"  INSERT deferred to determ.: {insert_deferred}")
        print(f"  DONE blocked from ML:       {done_blocked}")
        print(f"  Feasibility override:       {feasibility_override}")
        print(f"  Low confidence fallback:    {low_confidence_fallback}")
        print(f"  Unsafe advice blocked:      {metrics['safety_gating']['unsafe_advice_blocked']}")
        print(f"\nPost-gating metrics:")
        print(f"  Accuracy:     {metrics['accuracy']:.4f}")
        print(f"  Macro F1:     {metrics['macro_f1']:.4f}")
        print(f"  Weighted F1:  {metrics['weighted_f1']:.4f}")
        print(f"  SEARCH recall: {metrics['search_recall']:.4f}")
        print(f"  RETREAT recall: {metrics['retreat_recall']:.4f}")
        print(f"  DONE recall:   {metrics['done_recall']:.4f}")

        return metrics

    elif HAS_SKLEARN:
        clf, scaler = train_sklearn_fallback(X_train, y_train, X_test, y_test)
        valid_mask = np.isin(y_test, PHASE_LIST)
        X_te_s = scaler.transform(X_test[valid_mask])
        y_raw = clf.predict(X_te_s)
        probs = clf.predict_proba(X_te_s)
        confs = probs.max(axis=1)
        y_test_valid = y_test[valid_mask]
        feas_valid = feas_test[valid_mask]
        meta_valid = meta_test.iloc[valid_mask].reset_index(drop=True)

        y_advisory = y_raw.copy()
        insert_deferred = 0
        done_blocked = 0
        feasibility_override = 0
        low_confidence_fallback = 0

        for i in range(len(y_advisory)):
            phase = y_raw[i]
            conf = confs[i]
            feas = feas_valid[i]

            if phase == 5:
                y_advisory[i] = 1
                insert_deferred += 1
            elif phase == 7:
                y_advisory[i] = 6
                done_blocked += 1
            elif feas == 2:
                y_advisory[i] = 6
                feasibility_override += 1
            elif conf < confidence_threshold:
                y_advisory[i] = 1
                low_confidence_fallback += 1

        fallback_count = int(np.sum(y_advisory != y_raw))

        metrics = compute_metrics(y_test_valid, y_advisory)
        metrics["safety_gating"] = {
            "confidence_threshold": confidence_threshold,
            "fallback_count": int(fallback_count),
            "fallback_rate": round(float(fallback_count / len(y_advisory)), 4),
            "insert_deferred_to_deterministic": int(insert_deferred),
            "done_blocked_from_ml": int(done_blocked),
            "feasibility_override_to_retreat": int(feasibility_override),
            "low_confidence_fallback": int(low_confidence_fallback),
            "unsafe_advice_blocked": int(insert_deferred + done_blocked),
        }
        metrics["raw_ml_accuracy"] = round(float(accuracy_score(y_test_valid, y_raw)), 4)
        metrics["advisory_accuracy"] = metrics["accuracy"]

        print(f"\nRaw ML accuracy (no safety gates): {metrics['raw_ml_accuracy']:.4f}")
        print(f"Advisory accuracy (with safety gates): {metrics['advisory_accuracy']:.4f}")
        print(f"  Fallback rate:  {metrics['safety_gating']['fallback_rate']:.4f}")
        print(f"  Unsafe blocked: {metrics['safety_gating']['unsafe_advice_blocked']}")

        return metrics

    else:
        print("\nERROR: Neither torch nor sklearn available.")
        return {}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    t0 = time.time()
    print("=" * 70)
    print("Cross-Scenario v2_14 Comprehensive Evaluation")
    print("=" * 70)
    print(f"PyTorch available: {HAS_TORCH}")
    print(f"scikit-learn available: {HAS_SKLEARN}")
    print(f"Device: {DEVICE}")

    # Load data
    if not DATASET_PATH.exists():
        print(f"\nERROR: Dataset not found at {DATASET_PATH}")
        sys.exit(1)

    df = load_dataset(DATASET_PATH)
    X, y_phase, y_feasibility, meta = extract_arrays(df)
    scenarios = sorted(df["scenario_id"].unique())

    print(f"\nExtracted arrays:")
    print(f"  X shape:            {X.shape}")
    print(f"  y_phase shape:      {y_phase.shape}")
    print(f"  y_feasibility shape: {y_feasibility.shape}")
    print(f"  Unique phases:      {np.unique(y_phase)}")
    print(f"  Unique feasibility: {np.unique(y_feasibility)}")

    # Run all four evaluation modes
    results = {
        "metadata": {
            "dataset": str(DATASET_PATH),
            "total_rows": len(df),
            "context_dim": CONTEXT_DIM,
            "n_scenarios": len(scenarios),
            "scenarios": scenarios,
            "phase_map": PHASE_MAP,
            "feasibility_map": FEASIBILITY_NAMES,
            "pytorch_available": HAS_TORCH,
            "sklearn_available": HAS_SKLEARN,
            "device": DEVICE,
        },
    }

    # Mode A
    results["mode_a_mixed_split"] = run_mode_a(X, y_phase, meta, scenarios)

    # Mode B
    results["mode_b_held_out"] = run_mode_b(X, y_phase, meta, scenarios)

    # Mode C
    results["mode_c_feasibility"] = run_mode_c(X, y_feasibility, meta, scenarios)

    # Mode D
    results["mode_d_advisory"] = run_mode_d(X, y_phase, y_feasibility, meta, scenarios)

    # Save results
    elapsed = time.time() - t0
    results["execution_time_seconds"] = round(elapsed, 2)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_FILE, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\nResults saved to: {OUTPUT_FILE}")

    # -----------------------------------------------------------------------
    # Comprehensive summary
    # -----------------------------------------------------------------------
    print("\n" + "=" * 70)
    print("COMPREHENSIVE EVALUATION SUMMARY")
    print("=" * 70)

    # Mode A summary
    ma = results.get("mode_a_mixed_split", {})
    if ma:
        print(f"\n[Mode A] Mixed-Scenario Random Split:")
        print(f"  Accuracy:     {ma.get('accuracy', 0):.4f}")
        print(f"  Macro F1:     {ma.get('macro_f1', 0):.4f}")
        print(f"  Weighted F1:  {ma.get('weighted_f1', 0):.4f}")
        print(f"  SEARCH recall: {ma.get('search_recall', 0):.4f}")
        print(f"  RETREAT recall: {ma.get('retreat_recall', 0):.4f}")
        print(f"  DONE recall:   {ma.get('done_recall', 0):.4f}")
        per_sc = ma.get("per_scenario_accuracy", {})
        if per_sc:
            print(f"  Per-scenario accuracy:")
            for sc, data in sorted(per_sc.items()):
                print(f"    {sc}: {data['accuracy']:.4f} (n={data['n_rows']})")

    # Mode B summary
    mb = results.get("mode_b_held_out", {})
    if mb and "_summary" in mb:
        s = mb["_summary"]
        print(f"\n[Mode B] Held-Out Scenario Evaluation:")
        print(f"  Mean accuracy:   {s['mean_accuracy']:.4f} +/- {s['std_accuracy']:.4f}")
        print(f"  Min accuracy:    {s['min_accuracy']:.4f} ({s['worst_scenario']})")
        print(f"  Max accuracy:    {s['max_accuracy']:.4f}")
        print(f"  Mean macro F1:   {s['mean_macro_f1']:.4f}")
        for sc in scenarios:
            if sc in mb and "accuracy" in mb[sc]:
                print(f"    {sc:30s} acc={mb[sc]['accuracy']:.4f}")

    # Mode C summary
    mc = results.get("mode_c_feasibility", {})
    if mc and "rule_based" in mc:
        rb = mc["rule_based"]
        print(f"\n[Mode C] Feasibility Classifier (Geometry-Only):")
        print(f"  Accuracy:            {rb['accuracy']:.4f}")
        print(f"  False-safe rate:     {rb['false_safe_rate']:.4f} ({rb['false_safe_count']} cases)")
        print(f"  Fail-closed detect:  {rb['fail_closed_detection_rate']:.4f}")
        print(f"  False-block rate:    {rb['false_block_rate']:.4f}")
        if "ml_classifier" in mc:
            ml = mc["ml_classifier"]
            print(f"  ML classifier accuracy: {ml['accuracy']:.4f}")

    # Mode D summary
    md = results.get("mode_d_advisory", {})
    if md and "safety_gating" in md:
        sg = md["safety_gating"]
        print(f"\n[Mode D] Safety-Gated Advisory Mode:")
        print(f"  Raw ML accuracy:     {md.get('raw_ml_accuracy', 0):.4f}")
        print(f"  Advisory accuracy:   {md.get('advisory_accuracy', 0):.4f}")
        print(f"  Fallback rate:       {sg['fallback_rate']:.4f}")
        print(f"  INSERT deferred:     {sg['insert_deferred_to_deterministic']}")
        print(f"  DONE blocked:        {sg['done_blocked_from_ml']}")
        print(f"  Feasibility override: {sg['feasibility_override_to_retreat']}")
        print(f"  Low conf fallback:   {sg['low_confidence_fallback']}")
        print(f"  Unsafe advice blocked: {sg['unsafe_advice_blocked']}")
        print(f"  SEARCH recall: {md.get('search_recall', 0):.4f}")
        print(f"  RETREAT recall: {md.get('retreat_recall', 0):.4f}")
        print(f"  DONE recall:   {md.get('done_recall', 0):.4f}")

    print(f"\nExecution time: {elapsed:.1f}s")
    print("=" * 70)
    print("SAFETY NOTICE: All ML outputs are advisory-only.")
    print("INSERT is always deferred to the deterministic controller.")
    print("DONE is never trusted from ML prediction.")
    print("Feasibility classifier provides envelope-aware fail-closed behavior.")
    print("=" * 70)

    return results


if __name__ == "__main__":
    main()

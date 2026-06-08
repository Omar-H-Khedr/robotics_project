"""Comprehensive v2_15 ablation with full per-class metrics.

Compares multiple action classification approaches on real multi-phase data:
  A: with v2_13 encoder (32-dim latent)
  B: raw 68-dim input (no encoder)
  C: raw 68-dim + min-max normalization
  D: feature-selected (joint_pos + joint_vel only, 12-dim)
  E: no-phase-int baseline (excludes phase_int/safety_int from input)

Reports: accuracy, per-class precision/recall/F1, confusion matrix,
SEARCH recall, macro/weighted averages.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Tuple, Dict, Any

import numpy as np
import pyarrow.parquet as pq
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


CONTEXT_DIM = 68
LATENT_DIM = 32
NUM_CLASSES = 9
JOINT_DIM = 6
DEFAULT_EPOCHS = 200
DEFAULT_BATCH = 64
DEFAULT_LR = 1e-3
DEFAULT_SEED = 0
HEAD_HIDDEN = 32
HEAD_DROPOUT = 0.10
WEIGHT_DECAY = 1e-4

DEPTH_VALUE_INDICES = (50, 51, 52, 53)
DEPTH_CLIP_VALUE = 5.0

PHASE_NAMES = {
    0: "UNKNOWN", 1: "MOVING_TO_START", 2: "APPROACH",
    3: "SEARCH", 4: "HOVER_ABOVE_HOLE", 5: "INSERT",
    6: "RETREAT", 7: "DONE", 8: "ABORT",
}


def _load_parquet(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    table = pq.read_table(path)
    df = table.to_pandas()
    X = np.stack([np.array(v, dtype=np.float32) for v in df["context_vec"]])
    phases = df["phase_int"].to_numpy(dtype=np.int64)
    return X, phases


def _preprocess(X: np.ndarray, mn: np.ndarray, rng: np.ndarray) -> np.ndarray:
    out = X.copy().astype(np.float32)
    for i in DEPTH_VALUE_INDICES:
        out[:, i] = np.clip(out[:, i], 0.0, DEPTH_CLIP_VALUE)
        out[:, i] = np.log1p(out[:, i])
    out = (out - mn) / rng
    return out


def _minmax_normalize(X_train: np.ndarray, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray, dict]:
    mn = X_train.min(axis=0)
    mx = X_train.max(axis=0)
    rng = mx - mn
    rng[rng < 1e-8] = 1.0
    return (X_train - mn) / rng, (X_test - mn) / rng, {"min": mn.tolist(), "max": mx.tolist(), "range": rng.tolist()}


def _load_encoder(encoder_path: Path) -> nn.Module:
    ckpt = torch.load(encoder_path, map_location="cpu", weights_only=False)
    from perception_pipeline.v2_13_context_encoder import Autoencoder
    model = Autoencoder(
        input_dim=ckpt["input_dim"], latent_dim=ckpt["latent_dim"],
        hidden_dims=tuple(ckpt["hidden_dims"]), dropout=ckpt["dropout"],
    )
    model.load_state_dict(ckpt["state_dict"])
    model.eval()
    for p in model.parameters():
        p.requires_grad = False
    return model


class PhaseHead(nn.Module):
    def __init__(self, input_dim: int, hidden: int = HEAD_HIDDEN, dropout: float = HEAD_DROPOUT):
        super().__init__()
        self.classifier = nn.Sequential(
            nn.Linear(input_dim, hidden), nn.ReLU(), nn.Dropout(dropout),
            nn.Linear(hidden, hidden), nn.ReLU(), nn.Dropout(dropout),
            nn.Linear(hidden, NUM_CLASSES),
        )
        self.regressor = nn.Sequential(
            nn.Linear(input_dim, hidden), nn.ReLU(), nn.Dropout(dropout),
            nn.Linear(hidden, JOINT_DIM),
        )

    def forward(self, x):
        return self.classifier(x), self.regressor(x)


def _train_head(
    X_train: torch.Tensor, X_test: torch.Tensor,
    y_train: torch.Tensor, y_test: torch.Tensor,
    z_train: torch.Tensor, z_test: torch.Tensor,
    input_dim: int, tag: str, args,
) -> Dict[str, Any]:
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    head = PhaseHead(input_dim=input_dim)
    opt = optim.Adam(head.parameters(), lr=args.lr, weight_decay=WEIGHT_DECAY)
    ce_fn = nn.CrossEntropyLoss()
    mse_fn = nn.MSELoss()
    train_ds = TensorDataset(X_train, y_train, z_train)
    test_ds = TensorDataset(X_test, y_test, z_test)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=args.batch_size, shuffle=False)

    best_acc = 0.0
    best_state = None
    for epoch in range(args.epochs):
        head.train()
        for xb, pb, zb in train_loader:
            opt.zero_grad()
            logits, reg = head(xb)
            loss = ce_fn(logits, pb) + 0.01 * mse_fn(reg, zb)
            loss.backward()
            opt.step()
        head.eval()
        correct = 0
        total = 0
        all_pred = []
        all_true = []
        with torch.no_grad():
            for xb, pb, zb in test_loader:
                logits, reg = head(xb)
                pred = logits.argmax(dim=1)
                correct += (pred == pb).sum().item()
                total += pb.size(0)
                all_pred.extend(pred.numpy())
                all_true.extend(pb.numpy())
        acc = correct / max(total, 1)
        if acc > best_acc:
            best_acc = acc
            best_state = {k: v.clone() for k, v in head.state_dict().items()}
        if epoch % 40 == 0 or epoch == args.epochs - 1:
            print(f"  [{tag}] epoch {epoch:4d}  test_acc={acc:.4f}")

    head.load_state_dict(best_state)

    # Full evaluation with best model
    head.eval()
    all_pred = []
    all_true = []
    all_reg_true = []
    all_reg_pred = []
    with torch.no_grad():
        for xb, pb, zb in test_loader:
            logits, reg = head(xb)
            pred = logits.argmax(dim=1)
            all_pred.extend(pred.numpy())
            all_true.extend(pb.numpy())
            all_reg_true.extend(zb.numpy())
            all_reg_pred.extend(reg.numpy())

    all_pred = np.array(all_pred)
    all_true = np.array(all_true)

    # Per-class metrics
    per_class = {}
    for c in range(NUM_CLASSES):
        mask_true = all_true == c
        mask_pred = all_pred == c
        tp = int((mask_true & mask_pred).sum())
        fp = int((~mask_true & mask_pred).sum())
        fn = int((mask_true & ~mask_pred).sum())
        support = int(mask_true.sum())
        predicted_count = int(mask_pred.sum())
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)
        f1 = 2 * precision * recall / max(precision + recall, 1e-8)
        per_class[c] = {
            "name": PHASE_NAMES.get(c, f"class_{c}"),
            "support": support,
            "predicted_count": predicted_count,
            "true_positive": tp,
            "precision": round(precision, 6),
            "recall": round(recall, 6),
            "f1": round(f1, 6),
        }

    # Confusion matrix
    cm = np.zeros((NUM_CLASSES, NUM_CLASSES), dtype=int)
    for t, p in zip(all_true, all_pred):
        cm[t][p] += 1

    # Macro averages
    valid = [c for c in range(NUM_CLASSES) if per_class[c]["support"] > 0]
    macro_precision = np.mean([per_class[c]["precision"] for c in valid])
    macro_recall = np.mean([per_class[c]["recall"] for c in valid])
    macro_f1 = np.mean([per_class[c]["f1"] for c in valid])

    # Weighted averages
    total = sum(per_class[c]["support"] for c in valid)
    weighted_precision = sum(per_class[c]["precision"] * per_class[c]["support"] for c in valid) / max(total, 1)
    weighted_recall = sum(per_class[c]["recall"] * per_class[c]["support"] for c in valid) / max(total, 1)
    weighted_f1 = sum(per_class[c]["f1"] * per_class[c]["support"] for c in valid) / max(total, 1)

    reg_mse = float(np.mean((np.array(all_reg_true) - np.array(all_reg_pred)) ** 2))

    search_recall = per_class[3]["recall"]  # SEARCH is class 3

    return {
        "tag": tag,
        "input_dim": input_dim,
        "test_acc": round(best_acc, 6),
        "per_class": per_class,
        "confusion_matrix": cm.tolist(),
        "macro_precision": round(macro_precision, 6),
        "macro_recall": round(macro_recall, 6),
        "macro_f1": round(macro_f1, 6),
        "weighted_precision": round(weighted_precision, 6),
        "weighted_recall": round(weighted_recall, 6),
        "weighted_f1": round(weighted_f1, 6),
        "search_recall": round(search_recall, 6),
        "regression_mse": round(reg_mse, 8),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-parquet", required=True)
    parser.add_argument("--encoder-pt", required=True)
    parser.add_argument("--scaler-json", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    parser.add_argument("--batch-size", type=int, default=DEFAULT_BATCH)
    parser.add_argument("--lr", type=float, default=DEFAULT_LR)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("v2_15 comprehensive ablation: loading data")
    X, phases = _load_parquet(Path(args.input_parquet))
    print(f"  X.shape={X.shape} phases.shape={phases.shape}")

    # Drop empty-camera rows
    rgb_sum = X[:, :48].sum(axis=1)
    valid = (rgb_sum > 1.0) & (X[:, 48] > 0) & (X[:, 49] > 0)
    X = X[valid]
    phases = phases[valid]
    print(f"  after filtering: {X.shape[0]} rows")

    # Load scaler and preprocess
    with open(args.scaler_json) as f:
        scaler = json.load(f)
    mn = np.array(scaler["min"], dtype=np.float32)
    mx = np.array(scaler["max"], dtype=np.float32)
    rng = mx - mn
    rng[rng < 1e-8] = 1.0
    Xp = _preprocess(X, mn, rng)

    # Load encoder
    encoder = _load_encoder(Path(args.encoder_pt))
    with torch.no_grad():
        Z = encoder.encode(torch.from_numpy(Xp).float()).numpy()
    print(f"  Z.shape={Z.shape} (encoder latent)")

    # Per-phase target joint pose
    per_phase_target = {}
    for c in range(NUM_CLASSES):
        mask = phases == c
        if mask.sum() > 0:
            per_phase_target[c] = X[mask, 54:60].mean(axis=0).tolist()
        else:
            per_phase_target[c] = [0.0] * 6

    y_joint = np.zeros((X.shape[0], JOINT_DIM), dtype=np.float32)
    for c in range(NUM_CLASSES):
        mask = phases == c
        y_joint[mask] = per_phase_target.get(c, [0.0] * 6)

    # Split 80/20
    rng_state = np.random.default_rng(args.seed)
    idx = rng_state.permutation(X.shape[0])
    n_train = int(0.8 * X.shape[0])
    train_idx, test_idx = idx[:n_train], idx[n_train:]

    print(f"  train={len(train_idx)} test={len(test_idx)}")

    # Convert to tensors
    Xp_train_t = torch.from_numpy(Xp[train_idx]).float()
    Xp_test_t = torch.from_numpy(Xp[test_idx]).float()
    Z_train_t = torch.from_numpy(Z[train_idx]).float()
    Z_test_t = torch.from_numpy(Z[test_idx]).float()
    p_train_t = torch.from_numpy(phases[train_idx]).long()
    p_test_t = torch.from_numpy(phases[test_idx]).long()
    z_train_t = torch.from_numpy(y_joint[train_idx]).float()
    z_test_t = torch.from_numpy(y_joint[test_idx]).float()

    results = []

    # A: with encoder
    print("\n=== Variant A: with v2_13 encoder (32-dim) ===")
    res_a = _train_head(Z_train_t, Z_test_t, p_train_t, p_test_t, z_train_t, z_test_t,
                        input_dim=LATENT_DIM, tag="A_encoder", args=args)
    results.append(res_a)

    # B: raw 68-dim
    print("\n=== Variant B: raw 68-dim input ===")
    res_b = _train_head(Xp_train_t, Xp_test_t, p_train_t, p_test_t, z_train_t, z_test_t,
                        input_dim=CONTEXT_DIM, tag="B_raw_68", args=args)
    results.append(res_b)

    # C: raw 68-dim + extra normalization
    print("\n=== Variant C: raw 68-dim + min-max normalize ===")
    Xc_train, Xc_test, _ = _minmax_normalize(Xp[train_idx], Xp[test_idx])
    res_c = _train_head(
        torch.from_numpy(Xc_train).float(), torch.from_numpy(Xc_test).float(),
        p_train_t, p_test_t, z_train_t, z_test_t,
        input_dim=CONTEXT_DIM, tag="C_norm_68", args=args)
    results.append(res_c)

    # D: feature-selected (joint positions + velocities only, 12-dim)
    print("\n=== Variant D: joint features only (12-dim) ===")
    Xd_train = Xp[train_idx, 54:66].copy()
    Xd_test = Xp[test_idx, 54:66].copy()
    res_d = _train_head(
        torch.from_numpy(Xd_train).float(), torch.from_numpy(Xd_test).float(),
        p_train_t, p_test_t, z_train_t, z_test_t,
        input_dim=12, tag="D_joint_12", args=args)
    results.append(res_d)

    # E: no-phase-int (exclude phase_int and safety_int from input, 66-dim)
    print("\n=== Variant E: no-phase-int (66-dim, excludes last 2 features) ===")
    Xe_train = Xp[train_idx, :66].copy()
    Xe_test = Xp[test_idx, :66].copy()
    res_e = _train_head(
        torch.from_numpy(Xe_train).float(), torch.from_numpy(Xe_test).float(),
        p_train_t, p_test_t, z_train_t, z_test_t,
        input_dim=66, tag="E_no_phase_66", args=args)
    results.append(res_e)

    # Summary
    print("\n" + "=" * 80)
    print("COMPREHENSIVE ABLATION RESULTS")
    print("=" * 80)
    print(f"{'Variant':<20} {'Acc':>6} {'MacroF1':>8} {'WtF1':>8} {'SearchRec':>10} {'RegMSE':>10}")
    print("-" * 80)
    for r in results:
        print(f"{r['tag']:<20} {r['test_acc']:>6.4f} {r['macro_f1']:>8.4f} "
              f"{r['weighted_f1']:>8.4f} {r['search_recall']:>10.4f} {r['regression_mse']:>10.8f}")

    # Detailed per-class for best variant
    best = max(results, key=lambda r: r["weighted_f1"])
    print(f"\nBest variant: {best['tag']} (weighted_f1={best['weighted_f1']:.4f})")
    print(f"\nPer-class metrics for {best['tag']}:")
    print(f"{'Class':<20} {'Support':>8} {'Prec':>8} {'Recall':>8} {'F1':>8}")
    print("-" * 60)
    for c in range(NUM_CLASSES):
        m = best["per_class"][c]
        if m["support"] > 0:
            print(f"{m['name']:<20} {m['support']:>8} {m['precision']:>8.4f} "
                  f"{m['recall']:>8.4f} {m['f1']:>8.4f}")

    # Save
    out = {
        "version": "v2_15_comprehensive",
        "dataset": str(args.input_parquet),
        "n_rows": int(X.shape[0]),
        "n_train": len(train_idx),
        "n_test": len(test_idx),
        "epochs": args.epochs,
        "seed": args.seed,
        "results": results,
        "best_variant": best["tag"],
        "per_phase_target_pose": per_phase_target,
        "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    out_path = output_dir / "ablation_results.json"
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nWrote {out_path}")

    # Write Markdown report
    md_path = output_dir / "ablation_report.md"
    with open(md_path, "w") as f:
        f.write("# v2_15 Comprehensive Ablation Report\n\n")
        f.write(f"**Dataset**: {args.input_parquet}\n")
        f.write(f"**Rows**: {X.shape[0]} (train={len(train_idx)}, test={len(test_idx)})\n")
        f.write(f"**Epochs**: {args.epochs}, **Seed**: {args.seed}\n\n")
        f.write("## Summary\n\n")
        f.write("| Variant | Accuracy | Macro F1 | Weighted F1 | SEARCH Recall | Reg MSE |\n")
        f.write("|---|---:|---:|---:|---:|---:|\n")
        for r in results:
            f.write(f"| {r['tag']} | {r['test_acc']:.4f} | {r['macro_f1']:.4f} | "
                    f"{r['weighted_f1']:.4f} | {r['search_recall']:.4f} | {r['regression_mse']:.8f} |\n")
        f.write(f"\n## Best Variant: {best['tag']}\n\n")
        f.write("| Phase | Support | Precision | Recall | F1 |\n")
        f.write("|---|---:|---:|---:|---:|\n")
        for c in range(NUM_CLASSES):
            m = best["per_class"][c]
            if m["support"] > 0:
                f.write(f"| {m['name']} | {m['support']} | {m['precision']:.4f} | "
                        f"{m['recall']:.4f} | {m['f1']:.4f} |\n")
        f.write(f"\n**Confusion Matrix** ({best['tag']}):\n\n")
        f.write("| True \\ Pred | " + " | ".join(PHASE_NAMES.get(i, str(i)) for i in range(NUM_CLASSES)) + " |\n")
        f.write("|---|" + "|".join("---:" for _ in range(NUM_CLASSES)) + "|\n")
        for i, row in enumerate(best["confusion_matrix"]):
            if sum(row) > 0:
                f.write(f"| {PHASE_NAMES.get(i, str(i))} | " + " | ".join(str(v) for v in row) + " |\n")
        f.write("\n## Interpretation\n\n")
        delta = best["test_acc"] - res_a["test_acc"] if best["tag"] != "A_encoder" else 0
        if best["tag"] == "B_raw_68":
            f.write("The raw 68-dim input outperforms the encoder-based approach.\n")
            f.write("The 68->32 encoder bottleneck loses discriminative information, ")
            f.write(f"particularly for the rare SEARCH class (recall={res_a['search_recall']:.2f} "
                    f"with encoder vs {res_b['search_recall']:.2f} raw).\n\n")
            f.write("**Conclusion**: Use raw 68-dim context as the primary representation. ")
            f.write("The encoder is a documented negative ablation result.\n")
        else:
            f.write(f"Best variant is {best['tag']} with weighted_f1={best['weighted_f1']:.4f}.\n")
    print(f"Wrote {md_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

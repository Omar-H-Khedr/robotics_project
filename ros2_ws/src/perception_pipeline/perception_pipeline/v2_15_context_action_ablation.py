"""v2_15 ablation: with v2_13 encoder pre-training vs raw input.

A clean paired comparison of two action classifiers on the same data:
  A: with encoder
       N -> [frozen v2_13 encoder] -> 32 -> [head] -> 9 (CE) + 6 (MSE)
  B: no encoder
       N -> [Linear 32 + ReLU + Dropout] -> 32 -> [head] -> 9 (CE) + 6 (MSE)

Where N is the context vector dimension (68 for v2_13, auto-detected from
parquet metadata).
       The "Linear 32" is trained from scratch with the same loss.

If the v2_13 pre-training is useful, A should be at least comparable
to B (and ideally better in low-data regimes or in robustness to
depth outliers). If the v2_13 pre-training is harmful, A will be
worse (the encoder bottleneck throws away too much information).

Outputs in --output-dir:
    ablation_results.json     A and B test_acc, test_ce, test_mse,
                              per_class_metrics, confusion matrices
    head_with_encoder.pt      the v2_14 head (architecture A) checkpoint
    head_baseline.pt          the baseline head (architecture B) checkpoint
    history_with_encoder.json
    history_baseline.json
    confusion_with_encoder.png
    confusion_baseline.png
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Tuple

import numpy as np
import pyarrow.parquet as pq
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset


CONTEXT_DIM = 68
LATENT_DIM = 32
NUM_PHASE_CLASSES = 9
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


def _load_parquet(path: Path) -> Tuple[np.ndarray, np.ndarray, int]:
    table = pq.read_table(path)
    df = table.to_pandas()
    X = np.stack([np.array(v, dtype=np.float32) for v in df["context_vec"]])
    phases = df["phase_int"].to_numpy(dtype=np.int64)
    return X, phases, X.shape[1]


def _preprocess(X: np.ndarray, mn: np.ndarray, rng: np.ndarray) -> np.ndarray:
    out = X.copy().astype(np.float32)
    for i in DEPTH_VALUE_INDICES:
        out[:, i] = np.clip(out[:, i], 0.0, DEPTH_CLIP_VALUE)
        out[:, i] = np.log1p(out[:, i])
    out = (out - mn) / rng
    return out


def _load_encoder(encoder_path: Path) -> nn.Module:
    ckpt = torch.load(encoder_path, map_location="cpu", weights_only=False)

    class _Enc(nn.Module):
        def __init__(self, input_dim, latent_dim, hidden_dims, dropout):
            super().__init__()
            layers = []
            prev = input_dim
            for h in hidden_dims:
                layers.extend([nn.Linear(prev, h), nn.ReLU(), nn.Dropout(dropout)])
                prev = h
            self.encoder_body = nn.Sequential(*layers)
            self.encoder_head = nn.Linear(prev, latent_dim)

        def encode(self, x):
            return self.encoder_head(self.encoder_body(x))

    enc = _Enc(
        input_dim=ckpt["input_dim"],
        latent_dim=ckpt["latent_dim"],
        hidden_dims=tuple(ckpt["hidden_dims"]),
        dropout=ckpt["dropout"],
    )
    full_state = ckpt["state_dict"]
    encoder_state = {k: v for k, v in full_state.items() if k.startswith("encoder_")}
    enc.load_state_dict(encoder_state)
    enc.eval()
    return enc


class PhaseHead(nn.Module):
    def __init__(self, input_dim: int, hidden: int = HEAD_HIDDEN,
                 num_classes: int = NUM_PHASE_CLASSES, dropout: float = HEAD_DROPOUT):
        super().__init__()
        self.body = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
        )
        self.classifier = nn.Linear(hidden, num_classes)
        self.regressor = nn.Linear(hidden, JOINT_DIM)

    def forward(self, z: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        h = self.body(z)
        return self.classifier(h), self.regressor(h)


def _confusion_matrix(y_true: np.ndarray, y_pred: np.ndarray, n: int) -> np.ndarray:
    cm = np.zeros((n, n), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def _train_eval(
    X_train: torch.Tensor, X_test: torch.Tensor,
    p_train: torch.Tensor, p_test: torch.Tensor,
    y_train: torch.Tensor, y_test: torch.Tensor,
    input_dim: int, head_tag: str, args, history_path: Path,
) -> dict:
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    head = PhaseHead(input_dim=input_dim)
    opt = optim.Adam(head.parameters(), lr=args.lr, weight_decay=WEIGHT_DECAY)
    ce = nn.CrossEntropyLoss()
    mse = nn.MSELoss()
    train_ds = TensorDataset(X_train, p_train, y_train)
    test_ds = TensorDataset(X_test, p_test, y_test)
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=args.batch_size, shuffle=False)
    history = []
    for epoch in range(args.epochs):
        head.train()
        cl, rl, n = 0.0, 0.0, 0
        for zb, pb, yb in train_loader:
            opt.zero_grad()
            logits, reg = head(zb)
            lc = ce(logits, pb)
            lr_ = mse(reg, yb)
            loss = lc + lr_
            loss.backward()
            opt.step()
            cl += float(lc.item()) * zb.size(0)
            rl += float(lr_.item()) * zb.size(0)
            n += zb.size(0)
        cl /= max(n, 1)
        rl /= max(n, 1)
        head.eval()
        cl_t, rl_t, n = 0.0, 0.0, 0
        all_pred, all_true = [], []
        with torch.no_grad():
            for zb, pb, yb in test_loader:
                logits, reg = head(zb)
                lc = ce(logits, pb)
                lr_ = mse(reg, yb)
                cl_t += float(lc.item()) * zb.size(0)
                rl_t += float(lr_.item()) * zb.size(0)
                n += zb.size(0)
                all_pred.append(logits.argmax(dim=1).cpu().numpy())
                all_true.append(pb.cpu().numpy())
        cl_t /= max(n, 1)
        rl_t /= max(n, 1)
        y_pred = np.concatenate(all_pred)
        y_true = np.concatenate(all_true)
        acc = float((y_pred == y_true).mean())
        history.append({
            "epoch": epoch,
            "train_ce": cl, "train_mse": rl,
            "test_ce": cl_t, "test_mse": rl_t,
            "test_acc": acc,
        })
        if epoch % max(1, args.epochs // 10) == 0 or epoch == args.epochs - 1:
            print(
                f"  [{head_tag}] epoch {epoch:4d}  train_ce={cl:.4f} train_mse={rl:.4f}  "
                f"test_ce={cl_t:.4f} test_mse={rl_t:.4f}  test_acc={acc:.3f}"
            )
    with open(history_path, "w") as f:
        json.dump(history, f, indent=2)
    return {
        "head": head,
        "history": history,
        "y_pred": np.concatenate(all_pred),
        "y_true": np.concatenate(all_true),
    }


def _plot_cm(cm: np.ndarray, path: Path, title: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 5))
        im = ax.imshow(cm, cmap="Blues")
        ax.set_xlabel("predicted")
        ax.set_ylabel("true")
        ax.set_title(title)
        for i in range(NUM_PHASE_CLASSES):
            for j in range(NUM_PHASE_CLASSES):
                if cm[i, j] > 0:
                    ax.text(j, i, str(cm[i, j]), ha="center", va="center",
                            color="white" if cm[i, j] > cm.max() / 2 else "black",
                            fontsize=7)
        fig.colorbar(im, ax=ax)
        fig.tight_layout()
        fig.savefig(path, dpi=110)
        plt.close(fig)
        print(f"v2_15_ablation: wrote {path}")
    except ImportError:
        print("v2_15_ablation: matplotlib not available, skipped")


def _per_class(cm: np.ndarray) -> dict:
    out = {}
    for c in range(NUM_PHASE_CLASSES):
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
    parser = argparse.ArgumentParser(description="v2_15 ablation training")
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
    print(f"v2_15_ablation: output_dir = {output_dir}")

    X, phases, context_dim = _load_parquet(Path(args.input_parquet))
    print(f"v2_15_ablation: X.shape = {X.shape} phases.shape = {phases.shape}")
    rgb_sum = X[:, :48].sum(axis=1)
    valid = (rgb_sum > 1.0) & (X[:, 48] > 0) & (X[:, 49] > 0)
    X = X[valid]
    phases = phases[valid]
    print(f"  dropped {(~valid).sum()} empty rows, kept {X.shape[0]}")

    with open(args.scaler_json) as f:
        scaler = json.load(f)
    mn = np.array(scaler["min"], dtype=np.float32)
    mx = np.array(scaler["max"], dtype=np.float32)
    rng = mx - mn
    rng[rng < 1e-8] = 1.0
    Xp = _preprocess(X, mn, rng)

    encoder = _load_encoder(Path(args.encoder_pt))
    for p in encoder.parameters():
        p.requires_grad = False
    encoder.eval()
    with torch.no_grad():
        Z = encoder.encode(torch.from_numpy(Xp).float()).numpy()
    print(f"v2_15_ablation: Z.shape = {Z.shape}")

    per_phase_target = {}
    for phase_int in sorted(set(phases.tolist())):
        mask = phases == phase_int
        if mask.any():
            per_phase_target[str(int(phase_int))] = X[mask, 60:66].mean(axis=0).tolist()
    y_target = np.zeros((X.shape[0], JOINT_DIM), dtype=np.float32)
    for i, p in enumerate(phases):
        y_target[i] = per_phase_target[str(int(p))]

    rng2 = np.random.default_rng(args.seed)
    idx = rng2.permutation(X.shape[0])
    n_train = int(0.8 * X.shape[0])
    tr_idx, te_idx = idx[:n_train], idx[n_train:]

    Z_train, Z_test = Z[tr_idx], Z[te_idx]
    Xp_train, Xp_test = Xp[tr_idx], Xp[te_idx]
    p_train, p_test = phases[tr_idx], phases[te_idx]
    y_train, y_test = y_target[tr_idx], y_target[te_idx]

    print("v2_15_ablation: training A (with v2_13 encoder, input_dim=32)")
    res_a = _train_eval(
        torch.from_numpy(Z_train).float(),
        torch.from_numpy(Z_test).float(),
        torch.from_numpy(p_train).long(),
        torch.from_numpy(p_test).long(),
        torch.from_numpy(y_train).float(),
        torch.from_numpy(y_test).float(),
        input_dim=LATENT_DIM,
        head_tag="A_with_encoder",
        args=args,
        history_path=output_dir / "history_with_encoder.json",
    )
    cm_a = _confusion_matrix(res_a["y_true"], res_a["y_pred"], NUM_PHASE_CLASSES)
    _plot_cm(cm_a, output_dir / "confusion_with_encoder.png",
             "v2_15 phase classifier with v2_13 encoder")

    print(f"v2_15_ablation: training B (no encoder, input_dim={CONTEXT_DIM})")
    res_b = _train_eval(
        torch.from_numpy(Xp_train).float(),
        torch.from_numpy(Xp_test).float(),
        torch.from_numpy(p_train).long(),
        torch.from_numpy(p_test).long(),
        torch.from_numpy(y_train).float(),
        torch.from_numpy(y_test).float(),
        input_dim=CONTEXT_DIM,
        head_tag="B_baseline",
        args=args,
        history_path=output_dir / "history_baseline.json",
    )
    cm_b = _confusion_matrix(res_b["y_true"], res_b["y_pred"], NUM_PHASE_CLASSES)
    _plot_cm(cm_b, output_dir / "confusion_baseline.png",
             "v2_15 phase classifier baseline (no encoder)")

    head_a_path = output_dir / "head_with_encoder.pt"
    torch.save({
        "state_dict": res_a["head"].state_dict(),
        "input_dim": LATENT_DIM,
        "num_classes": NUM_PHASE_CLASSES,
        "joint_dim": JOINT_DIM,
        "hidden": HEAD_HIDDEN,
        "dropout": HEAD_DROPOUT,
    }, head_a_path)
    head_b_path = output_dir / "head_baseline.pt"
    torch.save({
        "state_dict": res_b["head"].state_dict(),
        "input_dim": CONTEXT_DIM,
        "num_classes": NUM_PHASE_CLASSES,
        "joint_dim": JOINT_DIM,
        "hidden": HEAD_HIDDEN,
        "dropout": HEAD_DROPOUT,
    }, head_b_path)
    print(f"v2_15_ablation: wrote {head_a_path}")
    print(f"v2_15_ablation: wrote {head_b_path}")

    final_a = res_a["history"][-1]
    final_b = res_b["history"][-1]

    summary = {
        "version": "v2_15",
        "ablation": f"with_v2_13_encoder vs raw_{CONTEXT_DIM}_dim_input",
        "dataset": str(args.input_parquet),
        "encoder_pt": str(args.encoder_pt),
        "scaler_json": str(args.scaler_json),
        "n_rows": int(X.shape[0]),
        "n_train": int(Z_train.shape[0]),
        "n_test": int(Z_test.shape[0]),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "seed": args.seed,
        "with_encoder": {
            "input_dim": LATENT_DIM,
            "final_test_acc": final_a["test_acc"],
            "final_test_ce": final_a["test_ce"],
            "final_test_mse": final_a["test_mse"],
            "per_class_metrics": _per_class(cm_a),
            "confusion_matrix": cm_a.tolist(),
        },
        "baseline": {
            "input_dim": CONTEXT_DIM,
            "final_test_acc": final_b["test_acc"],
            "final_test_ce": final_b["test_ce"],
            "final_test_mse": final_b["test_mse"],
            "per_class_metrics": _per_class(cm_b),
            "confusion_matrix": cm_b.tolist(),
        },
        "delta_test_acc": final_a["test_acc"] - final_b["test_acc"],
        "delta_test_ce": final_a["test_ce"] - final_b["test_ce"],
        "delta_test_mse": final_a["test_mse"] - final_b["test_mse"],
        "created_utc": int(time.time()),
    }
    summary_path = output_dir / "ablation_results.json"
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"v2_15_ablation: wrote {summary_path}")
    print(f"v2_15_ablation: with_encoder test_acc={final_a['test_acc']:.3f}  "
          f"baseline test_acc={final_b['test_acc']:.3f}  "
          f"delta={summary['delta_test_acc']:+.3f}")
    print("v2_15_ablation: done")
    return 0


if __name__ == "__main__":
    sys.exit(main())

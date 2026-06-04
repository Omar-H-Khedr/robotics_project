"""v2_13 self-supervised context encoder.

Trains a 74 -> 32 -> 74 autoencoder on a context_log.parquet produced
by the v2_12 context_vector_extractor. The encoder learns a 32-dim
latent that v2_14 (context-conditioned action) reuses for the
context -> action mapping.

This is offline training only; it does not need ROS. The script is
registered as a console script so it can be run with `ros2 run
perception_pipeline v2_13_context_encoder -- ...` for symmetry with
the other perception_pipeline tools, but it imports nothing from
rclpy and has no live node.

Data validation is performed before training and written to a
report alongside the artifacts. If the dataset is single-phase or
no-motion, the autoencoder still trains reconstruction but the
report calls that out so a downstream reader does not assume the
encoder can solve the original phase-learning problem alone.

Outputs in --output-dir:
    autoencoder.pt           full autoencoder weights (encoder + decoder)
    encoder.pt              encoder-only weights (for v2_14)
    scaler.json              per-feature mean / std used for normalization
    metadata.json            dimensions, hyperparams, MSE, dataset, timestamp
    data_validation_report.json  input shape, NaN/Inf, variance, class dist
    training_curve.png       loss vs epoch (if matplotlib is available)
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

CONTEXT_DIM = 74
LATENT_DIM = 32
DEFAULT_EPOCHS = 200
DEFAULT_BATCH = 64
DEFAULT_LR = 1e-3
DEFAULT_SEED = 0
HIDDEN_DIMS = (32, 32)
DROPOUT = 0.05
WEIGHT_DECAY = 1e-4

DEPTH_DIM_INDICES = (48, 49)
DEPTH_VALUE_INDICES = (50, 51, 52, 53)
DEPTH_CLIP_VALUE = 5.0
ZERO_VAR_FEATURES = (54, 55, 56, 57, 58, 59, 66, 67, 68, 69, 70, 71)


def _load_parquet(path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    table = pq.read_table(path)
    df = table.to_pandas()
    if "context_vec" not in df.columns:
        raise RuntimeError(f"parquet is missing 'context_vec' column: {path}")
    X = np.stack([np.array(v, dtype=np.float32) for v in df["context_vec"]])
    phases = df["phase_int"].to_numpy(dtype=np.int64) if "phase_int" in df.columns else None
    safety = df["safety_int"].to_numpy(dtype=np.int64) if "safety_int" in df.columns else None
    spec = {}
    if table.schema.metadata and b"context_spec_json" in table.schema.metadata:
        spec = json.loads(table.schema.metadata[b"context_spec_json"].decode())
    if X.shape[1] != CONTEXT_DIM:
        raise RuntimeError(
            f"expected context_dim={CONTEXT_DIM}, got {X.shape[1]}"
        )
    return X, phases, safety, spec


def _validate(X: np.ndarray, phases: np.ndarray, spec: dict) -> dict:
    report = {
        "n_rows": int(X.shape[0]),
        "n_features": int(X.shape[1]),
        "any_nan": bool(np.isnan(X).any()),
        "any_inf": bool(np.isinf(X).any()),
        "min": float(X.min()),
        "max": float(X.max()),
        "per_feature_variance_zero": int((X.var(axis=0) < 1e-8).sum()),
        "per_feature_variance_max": float(X.var(axis=0).max()),
        "per_feature_variance_mean": float(X.var(axis=0).mean()),
        "per_feature_min": X.min(axis=0).tolist(),
        "per_feature_max": X.max(axis=0).tolist(),
        "phase_unique": sorted(set(phases.tolist())) if phases is not None else None,
        "phase_counts": (
            {int(k): int(v) for k, v in zip(*np.unique(phases, return_counts=True))}
            if phases is not None else None
        ),
        "n_phase_classes": (
            int(len(np.unique(phases))) if phases is not None else None
        ),
        "spec_version": spec.get("version"),
        "spec_dim": spec.get("context_dim"),
    }
    if phases is not None:
        report["is_single_phase"] = int(len(np.unique(phases))) <= 1
    else:
        report["is_single_phase"] = None
    return report


def _normalize(X_train: np.ndarray, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray, dict]:
    mn = X_train.min(axis=0)
    mx = X_train.max(axis=0)
    rng = mx - mn
    rng[rng < 1e-8] = 1.0
    scaler = {
        "min": mn.tolist(),
        "max": mx.tolist(),
        "range": rng.tolist(),
    }
    return (X_train - mn) / rng, (X_test - mn) / rng, scaler


class Autoencoder(nn.Module):
    def __init__(self, input_dim: int = CONTEXT_DIM, latent_dim: int = LATENT_DIM,
                 hidden_dims=HIDDEN_DIMS, dropout: float = DROPOUT):
        super().__init__()
        layers = []
        prev = input_dim
        for h in hidden_dims:
            layers.extend([nn.Linear(prev, h), nn.ReLU(), nn.Dropout(dropout)])
            prev = h
        self.encoder_body = nn.Sequential(*layers)
        self.encoder_head = nn.Linear(prev, latent_dim)
        rev_hidden = list(reversed(hidden_dims))
        dec_layers = []
        prev = latent_dim
        for h in rev_hidden:
            dec_layers.extend([nn.Linear(prev, h), nn.ReLU(), nn.Dropout(dropout)])
            prev = h
        self.decoder_body = nn.Sequential(*dec_layers)
        self.decoder_head = nn.Linear(prev, input_dim)

    def encode(self, x: torch.Tensor) -> torch.Tensor:
        return self.encoder_head(self.encoder_body(x))

    def decode(self, z: torch.Tensor) -> torch.Tensor:
        return self.decoder_head(self.decoder_body(z))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.decode(self.encode(x))


def train(
    X_train: np.ndarray, X_test: np.ndarray,
    epochs: int, batch_size: int, lr: float, seed: int,
) -> Tuple[Autoencoder, list, list]:
    torch.manual_seed(seed)
    np.random.seed(seed)
    device = torch.device("cpu")
    model = Autoencoder().to(device)
    opt = optim.Adam(model.parameters(), lr=lr, weight_decay=WEIGHT_DECAY)
    loss_fn = nn.MSELoss()
    train_ds = TensorDataset(torch.from_numpy(X_train).float())
    test_ds = TensorDataset(torch.from_numpy(X_test).float())
    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True)
    test_loader = DataLoader(test_ds, batch_size=batch_size, shuffle=False)
    history = []
    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        n = 0
        for (xb,) in train_loader:
            xb = xb.to(device)
            opt.zero_grad()
            xh = model(xb)
            loss = loss_fn(xh, xb)
            loss.backward()
            opt.step()
            train_loss += float(loss.item()) * xb.size(0)
            n += xb.size(0)
        train_loss /= max(n, 1)
        model.eval()
        test_loss = 0.0
        n = 0
        with torch.no_grad():
            for (xb,) in test_loader:
                xb = xb.to(device)
                xh = model(xb)
                loss = loss_fn(xh, xb)
                test_loss += float(loss.item()) * xb.size(0)
                n += xb.size(0)
        test_loss /= max(n, 1)
        history.append({"epoch": epoch, "train_mse": train_loss, "test_mse": test_loss})
        if epoch % max(1, epochs // 10) == 0 or epoch == epochs - 1:
            print(
                f"  epoch {epoch:4d}  train_mse={train_loss:.6f}  test_mse={test_loss:.6f}",
                flush=True,
            )
    return model, history, [h["train_mse"] for h in history], [h["test_mse"] for h in history]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Train a 74 -> 32 -> 74 autoencoder on a context_log.parquet "
            "produced by the v2_12 context_vector_extractor."
        )
    )
    parser.add_argument("--input-parquet", required=True,
                        help="Path to context_log.parquet (input).")
    parser.add_argument("--output-dir", required=True,
                        help="Directory to write encoder.pt, autoencoder.pt, "
                             "scaler.json, metadata.json, "
                             "data_validation_report.json, training_curve.png.")
    parser.add_argument("--epochs", type=int, default=DEFAULT_EPOCHS)
    parser.add_argument("--batch-size", type=int, default=DEFAULT_BATCH)
    parser.add_argument("--lr", type=float, default=DEFAULT_LR)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument("--latent-dim", type=int, default=LATENT_DIM)
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"v2_13_context_encoder: output_dir = {output_dir}")

    print(f"v2_13_context_encoder: loading {args.input_parquet}")
    X, phases, safety, spec = _load_parquet(Path(args.input_parquet))
    print(f"v2_13_context_encoder: X.shape = {X.shape}")

    print("v2_13_context_encoder: validating data")
    report = _validate(X, phases, spec)
    report["input_path"] = str(args.input_parquet)
    report_path = output_dir / "data_validation_report.json"
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"v2_13_context_encoder: wrote {report_path}")
    print(f"  n_rows={report['n_rows']} n_features={report['n_features']}")
    print(f"  any_nan={report['any_nan']} any_inf={report['any_inf']}")
    print(f"  per_feature_variance_zero={report['per_feature_variance_zero']}")
    print(f"  per_feature_variance_max={report['per_feature_variance_max']:.4e}")
    print(f"  n_phase_classes={report['n_phase_classes']} is_single_phase={report['is_single_phase']}")
    if report["is_single_phase"]:
        print(
            "  WARNING: dataset is single-phase. The autoencoder can train "
            "reconstruction but cannot solve the original phase-learning "
            "problem alone. See data_validation_report.json."
        )

    print(f"v2_13_context_encoder: clipping depth values "
          f"{DEPTH_VALUE_INDICES} to [0, {DEPTH_CLIP_VALUE}] m "
          f"and applying log1p (skipping dimensions {DEPTH_DIM_INDICES})")
    for i in DEPTH_VALUE_INDICES:
        X[:, i] = np.clip(X[:, i], 0.0, DEPTH_CLIP_VALUE)
        X[:, i] = np.log1p(X[:, i])

    print("v2_13_context_encoder: dropping rows with empty-camera data "
          "(rgb_sum<1 or depth_w==0 or depth_h==0)")
    rgb_sum = X[:, :48].sum(axis=1)
    valid = (rgb_sum > 1.0) & (X[:, 48] > 0) & (X[:, 49] > 0)
    n_dropped = int((~valid).sum())
    if n_dropped > 0:
        if phases is not None:
            phases = phases[valid]
        X = X[valid]
        print(f"  dropped {n_dropped} rows, kept {X.shape[0]}")

    print(f"v2_13_context_encoder: splitting 80/20 with seed={args.seed}")
    rng = np.random.default_rng(args.seed)
    idx = rng.permutation(X.shape[0])
    n_train = int(0.8 * X.shape[0])
    train_idx, test_idx = idx[:n_train], idx[n_train:]
    X_train, X_test = X[train_idx], X[test_idx]
    print(f"  train={X_train.shape[0]} test={X_test.shape[0]}")

    print("v2_13_context_encoder: standardizing (train stats only)")
    X_train, X_test, scaler = _normalize(X_train, X_test)
    scaler_path = output_dir / "scaler.json"
    with open(scaler_path, "w") as f:
        json.dump(scaler, f, indent=2)
    print(f"v2_13_context_encoder: wrote {scaler_path}")

    print(f"v2_13_context_encoder: training {args.epochs} epochs "
          f"(latent_dim={args.latent_dim}, batch={args.batch_size}, lr={args.lr})")
    model, history, train_curve, test_curve = train(
        X_train, X_test,
        epochs=args.epochs, batch_size=args.batch_size,
        lr=args.lr, seed=args.seed,
    )
    train_mse = float(train_curve[-1])
    test_mse = float(test_curve[-1])
    print(f"v2_13_context_encoder: final train_mse={train_mse:.6f} test_mse={test_mse:.6f}")

    autoencoder_path = output_dir / "autoencoder.pt"
    torch.save(model.state_dict(), autoencoder_path)
    print(f"v2_13_context_encoder: wrote {autoencoder_path}")

    encoder = Autoencoder(input_dim=CONTEXT_DIM, latent_dim=args.latent_dim)
    encoder.load_state_dict(model.state_dict())
    encoder_path = output_dir / "encoder.pt"
    torch.save({
        "state_dict": encoder.state_dict(),
        "input_dim": CONTEXT_DIM,
        "latent_dim": args.latent_dim,
        "hidden_dims": list(HIDDEN_DIMS),
        "dropout": DROPOUT,
    }, encoder_path)
    print(f"v2_13_context_encoder: wrote {encoder_path}")

    print("v2_13_context_encoder: smoke test (reload encoder, run 1 batch)")
    ckpt = torch.load(encoder_path, map_location="cpu", weights_only=False)
    smoke_model = Autoencoder(input_dim=ckpt["input_dim"], latent_dim=ckpt["latent_dim"],
                              hidden_dims=tuple(ckpt["hidden_dims"]), dropout=ckpt["dropout"])
    smoke_model.load_state_dict(ckpt["state_dict"])
    smoke_model.eval()
    with torch.no_grad():
        sample = torch.from_numpy(X_test[:8]).float()
        z = smoke_model.encode(sample)
        xh = smoke_model(sample)
    assert z.shape == (8, args.latent_dim), f"latent shape mismatch: {z.shape}"
    assert xh.shape == (8, CONTEXT_DIM), f"recon shape mismatch: {xh.shape}"
    print(f"  latent shape {tuple(z.shape)} = OK (32-dim expected)")
    print(f"  recon   shape {tuple(xh.shape)} = OK (74-dim expected)")

    metadata = {
        "version": "v2_13",
        "model_type": "mlp_autoencoder",
        "input_dim": CONTEXT_DIM,
        "latent_dim": args.latent_dim,
        "hidden_dims": list(HIDDEN_DIMS),
        "dropout": DROPOUT,
        "weight_decay": WEIGHT_DECAY,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "seed": args.seed,
        "train_mse_final": train_mse,
        "test_mse_final": test_mse,
        "train_curve": train_curve,
        "test_curve": test_curve,
        "dataset_path": str(args.input_parquet),
        "n_rows": int(X.shape[0]),
        "n_train": int(X_train.shape[0]),
        "n_test": int(X_test.shape[0]),
        "depth_clip_value_m": DEPTH_CLIP_VALUE,
        "depth_dim_indices": list(DEPTH_DIM_INDICES),
        "depth_value_indices": list(DEPTH_VALUE_INDICES),
        "depth_transform": "clip(0,5)+log1p",
        "phase_classes": report["n_phase_classes"],
        "is_single_phase_dataset": report["is_single_phase"],
        "spec_version": spec.get("version"),
        "created_utc": int(time.time()),
    }
    metadata_path = output_dir / "metadata.json"
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"v2_13_context_encoder: wrote {metadata_path}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot([h["epoch"] for h in history], train_curve, label="train")
        ax.plot([h["epoch"] for h in history], test_curve, label="test")
        ax.set_xlabel("epoch")
        ax.set_ylabel("MSE")
        ax.set_yscale("log")
        ax.set_title("v2_13 context autoencoder loss")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig_path = output_dir / "training_curve.png"
        fig.savefig(fig_path, dpi=110)
        plt.close(fig)
        print(f"v2_13_context_encoder: wrote {fig_path}")
    except ImportError:
        print("v2_13_context_encoder: matplotlib not available, skipped training_curve.png")

    print("v2_13_context_encoder: done")
    return 0


if __name__ == "__main__":
    sys.exit(main())

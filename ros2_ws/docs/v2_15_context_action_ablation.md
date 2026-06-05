# v2_15 Context-Action Ablation

A clean A/B ablation of the v2_14 task (phase classification +
per-phase target joint regression) on the same multi-phase
synthetic dataset, comparing:

  A. **with v2_13 encoder** (input_dim=32; the v2_13_v2 frozen
     encoder compresses the 74-dim context to 32-dim, then a
     small PhaseHead MLP is trained on top)
  B. **baseline** (input_dim=74; the PhaseHead MLP is trained
     from scratch on the raw 74-dim context)

Same dataset, same head architecture, same hyperparameters, same
seed. The only difference is whether the input is the 32-dim
encoder latent or the 74-dim raw context.

## Why an ablation

The v2_13 encoder is a self-supervised 74 -> 32 -> 74 autoencoder
trained on the same data. The pre-training throws away 42 of 74
features; it could be too aggressive (information loss) or
helpful (denoising, regularization). v2_15 measures the actual
effect on the downstream task.

This is offline training only; it does not need ROS. The script is
registered as a console script
(`ros2 run perception_pipeline v2_15_context_action_ablation -- ...`).

## Pipeline position

```
v2_14_context_conditioned_action  (v2_14, single architecture)
        |
        v
v2_15_context_action_ablation     (v2_15, A vs B)  <-- you are here
        |
        v
[final thesis ablation chapter]
```

## Inputs

- `--input-parquet` path to multi-phase `context_log.parquet`.
- `--encoder-pt` path to v2_13_v2 encoder checkpoint (used by A only).
- `--scaler-json` path to v2_13_v2 scaler.json (used by both A and B).

## Outputs (in `--output-dir`)

| File | Purpose |
|---|---|
| `ablation_results.json` | full summary: A and B test_acc, test_ce, test_mse, per-class metrics, confusion matrices, deltas |
| `head_with_encoder.pt` | A's head checkpoint |
| `head_baseline.pt` | B's head checkpoint |
| `history_with_encoder.json` | A's full per-epoch loss curve |
| `history_baseline.json` | B's full per-epoch loss curve |
| `confusion_with_encoder.png` | 9x9 confusion matrix for A |
| `confusion_baseline.png` | 9x9 confusion matrix for B |

## Preprocessing

A: encoder.pt is loaded, applied to the preprocessed (log1p+minmax)
   74-dim context, latent (32-dim) feeds the head.
B: log1p+minmax applied to the 74-dim context, raw 74-dim feeds the
   head (no encoder bottleneck).

Both arms share the same 80/20 split (seed=0) on the SAME row
indices, so any difference is the input transform, not the split.

## Reproducing the v2_15_v1 baseline

```
source /opt/ros/jazzy/setup.bash
source /tmp/pos_controllers_setup.bash
source install/setup.bash
ros2 run perception_pipeline v2_15_context_action_ablation -- \
    --input-parquet diagnostics/perception_pipeline_synthetic_multiphase_v1/context_log.parquet \
    --encoder-pt diagnostics/perception_pipeline_v2_13_encoder_v2/encoder.pt \
    --scaler-json diagnostics/perception_pipeline_v2_13_encoder_v2/scaler.json \
    --output-dir diagnostics/perception_pipeline_v2_15_ablation \
    --epochs 200 --batch-size 64 --lr 0.001 --seed 0
```

Expected outputs:

```
with_encoder test_acc=1.000  baseline test_acc=1.000  delta=+0.000
```

Both A and B reach 100% test accuracy on the multi-phase dataset.
A's test_ce is 0.0156, B's is 0.0118 (B is slightly lower CE).
The difference is not statistically meaningful: with only 7 phase
classes and 4303 valid rows, the dataset is easy for either
architecture.

## Interpretation

The fact that the encoder pre-training does not help here is
expected on the synthetic multi-phase dataset, because:

  1. The `phase_int` (index 72) and `safety_int` (index 73) are
     directly in the 74-dim context vector; the model can read
     them off without needing the encoder bottleneck.
  2. The dataset is small (4303 rows) and structured (7 phase
     windows in time order), so the 74-dim input is already
     linearly separable with a small MLP.

A real ablation would require a multi-phase dataset where the
phase is IMPLICIT in the sensor data, not declared by the
publisher. That dataset is not reachable in the current
simulation (the working JTC's 1mm/2mm precision ceiling blocks
real SEARCH/INSERT/ABORT trials), so v2_15 reports a null
result: the encoder pre-training is at parity with the raw-input
baseline on this dataset.

## Files added

- `src/perception_pipeline/perception_pipeline/v2_15_context_action_ablation.py`
- `src/perception_pipeline/launch/v2_15_context_action_ablation.launch.py`
- `src/perception_pipeline/setup.py` (entry point)
- `docs/v2_15_context_action_ablation.md` (this file)
- `README.md` (v2_15 row)
- `diagnostics/perception_pipeline_v2_15_ablation/` (artifacts)

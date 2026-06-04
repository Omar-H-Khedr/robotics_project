# v2_13 Context Encoder

A self-supervised MLP autoencoder (74 -> 32 -> 74) trained offline on
the 74-dim context vectors produced by the v2_12
`context_vector_extractor`. The encoder's 32-dim latent representation
is the input that v2_14 (context-conditioned action) consumes.

This is an offline training step: it does not need ROS, but it is
registered as a console script
(`ros2 run perception_pipeline v2_13_context_encoder -- ...`) for
symmetry with the rest of the perception_pipeline toolchain.

## Why an autoencoder, not a classifier

The 1mm/2mm cartesian precision ceiling in the working JTC means
SEARCH / INSERT / ABORT labeled trials are unreachable. v2_13 is
therefore re-scoped from "phase classifier" to a self-supervised
encoder: it learns a 32-dim compression of the 74-dim context vector
without requiring phase labels, and v2_14 reuses the encoder latent
as the conditioning signal for a context -> action MLP.

## Pipeline position

```
multimodal_observation_logger  (v2_11, online)
        |
        |  multimodal_observation_log.csv
        v
context_vector_extractor        (v2_12, offline)
        |
        |  context_log.parquet  (X: N x 74, phase_int, safety_int)
        v
v2_13_context_encoder           (v2_13, offline)  <-- you are here
        |
        |  encoder.pt  +  scaler.json  +  metadata.json
        v
v2_14_context_conditioned_action  (v2_14, offline training +
                                    online ROS node)
```

## Inputs

- `--input-parquet` path to `context_log.parquet` produced by
  v2_12. Must contain the `context_vec` column (list<float> of length
  74), `phase_int`, `safety_int`, and the `context_spec_json` schema
  metadata.

## Outputs (in `--output-dir`)

| File | Purpose |
|---|---|
| `autoencoder.pt` | full encoder+decoder state dict, for offline analysis |
| `encoder.pt` | encoder-only checkpoint, dict with `state_dict`, `input_dim`, `latent_dim`, `hidden_dims`, `dropout`. v2_14 loads this directly |
| `scaler.json` | per-feature `min` and `max` from the train split. v2_14 re-applies the same (X - min) / range transform before inference |
| `metadata.json` | dimensions, hyperparams, final train/test MSE, full loss curve, dataset path, timestamp |
| `data_validation_report.json` | input shape, NaN/Inf, per-feature variance, zero-var feature count, phase class distribution, single-phase flag, spec version |
| `training_curve.png` | train + test MSE vs epoch on log-y axis (if matplotlib is available) |

## Preprocessing (committed in the encoder, applied identically at inference)

1. **Drop empty-camera rows** (`rgb_sum<1.0` or `depth_w==0` or
   `depth_h==0`). D405 publishes one empty frame at the start of
   every trial; keeping it in test while train is all non-empty
   blows up test MSE. v2_12 does not pre-filter; v2_13 does.
2. **Depth values** (indices 50-53: `depth_min_m`, `depth_max_m`,
   `depth_roi_min_m`, `depth_roi_max_m`) are clipped to
   `[0, 5.0] m` then `log1p` is applied. The clip removes
   `inf`/`nan` proxy values from the v2_12 NaN->0, inf->1e6
   conversion. The log1p compresses the 0.05-5m range so the
   autoencoder loss is not dominated by depth_max outliers.
   Indices 48-49 (image dimensions `depth_w`, `depth_h`) are NOT
   clipped or log-transformed.
3. **Min-max scaling** (per feature, train stats only) into `[0, 1]`.
   Min-max was chosen over z-score because depth_max has heavy
   tail even after clip+log1p, and a min-max scaler is robust to
   the remaining outliers in the same way that the test split
   benefits.

## Model

- Input: 74 (post-preprocessing)
- Hidden: 32 -> 32 (ReLU + Dropout 0.05 between each)
- Latent: 32
- Output: 74 (linear, MSE loss against input)
- ~4K parameters
- Optimizer: Adam, lr 1e-3, weight_decay 1e-4
- Batch size 64, 200 epochs (default; tunable)
- Determinism: `torch.manual_seed(seed)` and `np.random.seed(seed)`
  at the start of `train`, plus an `np.random.default_rng(seed)`
  for the 80/20 split. Re-running with the same `--seed` reproduces
  the same artifacts.

## Reproducing the v2_13_v1 baseline

```
source /opt/ros/jazzy/setup.bash
source /tmp/pos_controllers_setup.bash
source install/setup.bash
ros2 run perception_pipeline v2_13_context_encoder -- \
    --input-parquet diagnostics/perception_pipeline_motion_trial_v3/context_log.parquet \
    --output-dir diagnostics/perception_pipeline_v2_13_encoder \
    --epochs 200 --batch-size 64 --lr 0.001 \
    --seed 0 --latent-dim 32
```

Expected outputs:
- `train_mse_final ~= 0.0035` (in normalized [0,1] space)
- `test_mse_final  ~= 0.0024`
- smoke test prints `latent shape (8, 32)` and `recon shape (8, 74)`

The v3 trial is single-phase (54 UNKNOWN + 3277 MOVING_TO_START,
2 phase classes) so the autoencoder is reconstructing a single
trajectory, not learning to discriminate phases. The encoder is
therefore best treated as a 32-dim compression of a single
operating mode. v2_14 will use it as a feature; downstream
generalization to multiple phases requires re-training on a
multi-phase trial (see thesis chapter 5 / 6 roadmap).

## v2_14 compatibility path

v2_14 (the context-conditioned action network) needs to apply the
exact same preprocessing and load the exact same `encoder.pt`
checkpoint. The integration pattern:

```python
import json
import numpy as np
import torch
import torch.nn as nn

CONTEXT_DIM, LATENT_DIM = 74, 32

class Autoencoder(nn.Module):
    def __init__(self, input_dim=CONTEXT_DIM, latent_dim=LATENT_DIM,
                 hidden_dims=(32, 32), dropout=0.05):
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

ckpt = torch.load("diagnostics/perception_pipeline_v2_13_encoder/encoder.pt",
                  map_location="cpu", weights_only=False)
encoder = Autoencoder(input_dim=ckpt["input_dim"],
                      latent_dim=ckpt["latent_dim"],
                      hidden_dims=tuple(ckpt["hidden_dims"]),
                      dropout=ckpt["dropout"])
encoder.load_state_dict(ckpt["state_dict"])
encoder.eval()

scaler = json.load(open("diagnostics/perception_pipeline_v2_13_encoder/scaler.json"))
mn = np.array(scaler["min"])
mx = np.array(scaler["max"])
rng = mx - mn
rng[rng < 1e-8] = 1.0

def preprocess(x_74):
    x = x_74.copy()
    rgb_sum = x[:48].sum()
    if rgb_sum < 1.0 or x[48] == 0 or x[49] == 0:
        return None  # empty-camera frame
    for i in (50, 51, 52, 53):
        x[i] = np.log1p(min(max(x[i], 0.0), 5.0))
    return (x - mn) / rng

# In a live ROS callback (v2_14):
x = preprocess(context_vec)
if x is None:
    return
z = encoder.encode(torch.from_numpy(x).float().unsqueeze(0))  # (1, 32)
```

## Files added

- `src/perception_pipeline/perception_pipeline/v2_13_context_encoder.py`
- `src/perception_pipeline/launch/v2_13_context_encoder.launch.py`
- `src/perception_pipeline/setup.py` (entry point + torch dep)
- `docs/v2_13_context_encoder.md` (this file)
- `README.md` (v2_13 row)
- `diagnostics/perception_pipeline_v2_13_encoder/` (artifacts)

## Limitations

- Single-phase training set means the encoder compresses a single
  operating mode, not a phase-discriminative representation.
  Multi-phase data is required to train a phase-aware encoder.
- The depth features are clipped to 5.0 m. A workspace that
  occasionally sees the camera's far plane (>5 m) will be
  indistinguishable from the clipped value, which the model
  treats as a constant.
- 12 of the 74 features are zero-variance in this trial
  (wrench=0, joint velocity=0, depth_w=848, depth_h=480, etc.).
  They contribute nothing to the loss but are still passed through
  the network. v2_14 can drop them at inference if it wants.

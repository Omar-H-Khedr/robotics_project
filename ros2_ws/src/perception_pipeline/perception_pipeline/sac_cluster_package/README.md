# SAC / Meta-RL Cluster Training Package

Cluster-ready Soft Actor-Critic training package for scenario-randomized insertion tasks.

**Status:** Package only. No training has been performed. All results referenced are expected/projected baselines, not actual outcomes.

---

## Overview

This package provides:

- `sac_training_script.py` -- SAC agent training with scenario randomization, safety-gated rewards, and checkpointing
- `sac_evaluation_script.py` -- Checkpoint evaluation with baseline comparison
- `config/cluster_training_config.yaml` -- Training hyperparameters and scenario configurations
- `config/cluster_run_config.yaml` -- SLURM cluster settings, GPU requirements, storage paths
- `run_cluster.sh` -- SLURM submission script (auto-detects cluster vs local)

### Environment Contract

The training script expects an environment conforming to `sac_environment_contract.json`:

| Property       | Value        |
|---------------|--------------|
| Observation   | 72-dim       |
| Action        | 6-DOF        |
| Reward source | `SafetyAwareReward.compute()` |
| Scenario      | `ScenarioRandomizer.sample()`  |

The environment module `sac_scenario_randomization.py` must be in the parent directory of this package.

---

## Prerequisites

### Hardware

- GPU: NVIDIA A100 40GB minimum (tested with 1x A100)
- RAM: 32GB minimum
- Storage: ~500MB for checkpoints + logs per training run

### Software

- Python 3.10+
- PyTorch 2.0+ with CUDA 11.8
- NumPy
- PyYAML
- TensorBoard (for log visualization)

Install dependencies:

```bash
pip install torch numpy pyyaml tensorboard
```

---

## Directory Structure

```
sac_cluster_package/
  sac_training_script.py
  sac_evaluation_script.py
  config/
    cluster_training_config.yaml
    cluster_run_config.yaml
  run_cluster.sh
  README.md
```

---

## How to Run Locally (Smoke Test)

Verify the package runs without errors:

```bash
cd sac_cluster_package

# Smoke test: 5K steps, CPU
python sac_training_script.py \
    --total_steps 5000 \
    --device cpu \
    --checkpoint_dir /tmp/sac_test_ckpt \
    --log_dir /tmp/sac_test_logs

# Smoke test: evaluation
python sac_evaluation_script.py \
    --checkpoint /tmp/sac_test_ckpt/sac_final.pt \
    --episodes 5 \
    --output /tmp/sac_test_eval.json
```

### Local Run (1M steps)

```bash
# Requires GPU, ~4 hours on A100
python sac_training_script.py \
    --total_steps 1000000 \
    --scenario_mode uniform \
    --device cuda \
    --checkpoint_dir checkpoints/local_run \
    --log_dir logs/local_run
```

---

## How to Run on Cluster

### Submit to SLURM

```bash
# Default: 1M steps
sbatch run_cluster.sh

# Custom steps
TOTAL_STEPS=5000000 sbatch run_cluster.sh

# Check job status
squeue -u $USER

# View logs
tail -f logs/sac_train-*.out
```

### Override Configuration

All parameters can be overridden via environment variables:

| Variable          | Default              | Description                    |
|-------------------|----------------------|--------------------------------|
| `TOTAL_STEPS`     | `1000000`            | Training steps                 |
| `SCENARIO_MODE`   | `uniform`            | `uniform`, `weighted`, `fixed` |
| `SEED`            | `42`                 | Random seed                    |
| `CHECKPOINT_DIR`  | `/scratch/$USER/...` | Checkpoint output path         |
| `LOG_DIR`         | `/scratch/$USER/...` | TensorBoard log path           |
| `JOB_NAME`        | `sac_train`          | SLURM job name                 |
| `PARTITION`       | `gpu`                | SLURM partition                |
| `TIME_LIMIT`      | `24:00:00`           | Max wall clock time            |

### Monitor Training

```bash
# TensorBoard
tensorboard --logdir logs/scenario_randomized

# Watch checkpoint directory
watch -n 60 'ls -lh checkpoints/scenario_randomized/'
```

---

## Expected Results

All results below are **projected baselines** based on the environment design and SAC hyperparameters. No training has been performed.

### Projected Performance

| Metric                      | Value    |
|----------------------------|----------|
| Deterministic baseline     | 87.5%    |
| Advisory v2.14 target      | 90.0%    |
| Projected SAC (uniform)    | 88-92%   |
| Fail-closed rate (target)  | < 1%     |
| Unsafe rate (target)       | 0%       |

### Runtime Estimates

| Steps       | A100 Time   | Checkpoints |
|------------|-------------|-------------|
| 1M         | ~4 hours    | ~100        |
| 5M         | ~20 hours   | ~500        |
| 10M        | ~40 hours   | ~1000       |

### Scenario Randomization Impact

- Uniform sampling over peg/hole/clearance/offset ranges
- Expected to improve generalization by 5-10% over fixed-scenario training
- Held-out scenario evaluation validates out-of-distribution robustness

---

## Comparison with Baselines

| Approach               | Success Rate | Fail-Closed | Unsafe | Notes                    |
|------------------------|-------------|-------------|--------|--------------------------|
| Deterministic policy   | 87.5%       | N/A         | N/A    | No scenario variation    |
| Advisory v2.14         | 90.0%       | < 1%        | 0%     | Conservative, slower     |
| SAC (projected)        | 88-92%      | < 1%        | 0%     | Adaptive, faster at test |

The SAC agent is expected to match or exceed the deterministic baseline while maintaining safety constraints through the fail-closed reward structure.

---

## Configuration Reference

### Training Modes

- **uniform**: Uniform random sampling over full scenario ranges
- **weighted**: Weighted sampling toward nominal scenarios
- **fixed**: Single fixed scenario (no randomization)

### Safety Reward Structure

| Condition                          | Reward |
|-----------------------------------|--------|
| Success inside safety envelope    | +100   |
| Fail-closed outside envelope      | +50    |
| Unsafe insertion attempt          | -100   |

### Hyperparameters

| Parameter       | Value   | Notes                          |
|----------------|---------|--------------------------------|
| Hidden layers  | 2       |                                |
| Hidden units   | 256     |                                |
| Batch size     | 256     |                                |
| Learning rate  | 3e-4    | Adam optimizer                 |
| Gamma          | 0.99    | Discount factor                |
| Tau            | 0.005   | Soft target update             |
| Replay buffer  | 100K    |                                |
| Auto alpha     | True    | Adaptive entropy tuning        |
| Target entropy | -72     | -obs_dim (conservative)        |

---

## Known Limitations

1. **Environment dependency**: Requires `sac_scenario_randomization.py` in the parent directory. Package will not run without it.
2. **Single GPU**: No multi-GPU or distributed training support.
3. **Fixed observation space**: Hardcoded to 72-dim. Changing observation space requires code modification.
4. **No curriculum learning**: Scenario ranges are sampled uniformly from the start.
5. **Checkpoint size**: Each checkpoint is ~25MB. Long runs produce large checkpoint directories.
6. **Projected results only**: No actual training has been performed. All performance numbers are estimates.

---

## File Descriptions

### sac_training_script.py

Main SAC implementation with:
- Actor (Gaussian policy with tanh squashing)
- Twin critics (clipped double-Q)
- Auto-tuned entropy coefficient
- Replay buffer with ring buffer implementation
- Scenario-randomized environment wrapper
- TensorBoard logging
- Periodic checkpointing

### sac_evaluation_script.py

Evaluation utilities:
- Checkpoint loading
- Deterministic policy evaluation
- Held-out scenario evaluation
- Baseline comparison (87.5% deterministic, 90% advisory)
- JSON report generation

### run_cluster.sh

Cluster submission script:
- Auto-detects SLURM environment
- Falls back to local execution
- Module loading (python, cuda)
- Virtual environment activation
- Configurable via environment variables

---

## References

- Haarnoja et al., "Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor", 2018
- sac_environment_contract.json -- Environment interface specification
- sac_scenario_randomization.py -- Scenario randomization and safety rewards

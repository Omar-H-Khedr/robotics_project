# Reproducibility Guide

Last updated: 2026-06-09

## Environment

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic (via gz_ros2_control)
- **Python**: 3.12
- **PyTorch**: CPU-only
- **Robot**: KUKA LBR iisy 6 R1300 (simulated)

## Exact Launch Commands

### Single trial (production configuration)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false \
  control_rate:=25.0 \
  position_gain:=3000.0 \
  position_derivative_gain:=10.0 \
  joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  search_recenter_duration_s:=8.0 \
  search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 \
  search_entry_threshold_m:=0.0 \
  enable_perception_logging:=true \
  perception_log_dir:=diagnostics/<trial_dir> \
  exit_on_done:=true \
  shutdown_on_task_exit:=true
```

### Multi-trial validation

```bash
python3 -m experiment_manager.research_baseline_repeat_validator \
  --trials 20 \
  --timeout-s 600.0 \
  --output-dir diagnostics/research_baseline_production_search_v20_600s \
  --per-trial-tracking-logs \
  --extra-launch-arg "control_rate:=25.0" \
  --extra-launch-arg "position_gain:=3000.0" \
  --extra-launch-arg "position_derivative_gain:=10.0" \
  --extra-launch-arg "joint_damping_scale:=10.0" \
  --extra-launch-arg "inject_velocity_state:=true" \
  --extra-launch-arg "velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml" \
  --extra-launch-arg "search_recenter_duration_s:=8.0" \
  --extra-launch-arg "search_settle_duration_s:=9.0" \
  --extra-launch-arg "insert_handoff_timeout_s:=12.0" \
  --extra-launch-arg "search_entry_threshold_m:=0.0" \
  --extra-launch-arg "exit_on_done:=true" \
  --extra-launch-arg "shutdown_on_task_exit:=true"
```

## Perception Pipeline Training

### v2_13 autoencoder (6-phase)

```bash
python3 -m perception_pipeline.v2_13_context_encoder \
  --input-csv diagnostics/multi_trial_dataset_v3/merged_multimodal_observation_log.csv \
  --output-dir diagnostics/v2_13_encoder_v4_6phase \
  --epochs 200 --latent-dim 32 --seed 0
```

### v2_14 safety-gated classifier (raw 68-dim, 6-phase)

```bash
python3 -m perception_pipeline.v2_14_safety_gated_action \
  --input-parquet diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet \
  --output-dir diagnostics/v2_14_raw_safety_gated_v4 \
  --epochs 100 --hidden 128 --dropout 0.2 --seed 0
```

### v2_15 comprehensive ablation (6-phase)

```bash
python3 -m perception_pipeline.v2_15_comprehensive_ablation \
  --input-parquet diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet \
  --encoder-pt diagnostics/v2_13_encoder_v4_6phase/encoder.pt \
  --scaler-json diagnostics/v2_13_encoder_v4_6phase/scaler.json \
  --output-dir diagnostics/v2_15_ablation_v5_6phase \
  --epochs 30 --seed 0
```

### v2_14 offline validation

```bash
python3 -m perception_pipeline.test_v2_14_safety_gated_offline \
  --parquet diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet \
  --model diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt \
  --output diagnostics/v2_14_raw_safety_gated_v4/offline_test_results.json \
  --confidence-threshold 0.85
```

## Dataset Paths

| Dataset | Path | Rows | Phases |
|---|---|---:|---|
| 6-phase multi-trial CSV | `diagnostics/multi_trial_dataset_v3/merged_multimodal_observation_log.csv` | 22,083 | ALL 6 |
| 6-phase context vectors | `diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet` | 22,083 | ALL 6 |
| 20-trial outcomes | `diagnostics/research_baseline_production_search_v20_600s/` | 20 trials | N/A |

## Model Artifacts

| Model | Path | Metric |
|---|---|---|
| v2_13 encoder | `diagnostics/v2_13_encoder_v4_6phase/encoder.pt` | test_mse=0.003670 |
| v2_13 scaler | `diagnostics/v2_13_encoder_v4_6phase/scaler.json` | min/max per feature |
| v2_14 raw classifier | `diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt` | acc=99.98% |
| v2_14 classifier metadata | `diagnostics/v2_14_raw_safety_gated_v4/raw_classifier_metadata.json` | per-class metrics |
| v2_15 ablation results | `diagnostics/v2_15_ablation_v5_6phase/ablation_results.json` | 5 variants |

## Physical Validation Evidence

| Run | Trials | Successes | Rate | Path |
|---|---:|---:|---:|---|
| Production 10-trial | 10 | 10 | 100% | `diagnostics/research_baseline_search_entered_500hz_v1_10trial/` |
| Post-fix 10-trial | 10 | 9 | 90% | `diagnostics/research_baseline_search_confirmation_v1/` |
| 20-trial confirmation | 20 | 18 | 90% | `diagnostics/research_baseline_production_search_v20_600s/` |
| **Combined** | **40** | **37** | **92.5%** | All runs |

## Perception Pipeline Evidence

| Model | Accuracy | F1 (all 7 classes) | Path |
|---|---:|---:|---|
| v2_15 raw 68-dim | 99.91% | 99.91% | `diagnostics/v2_15_ablation_v5_6phase/` |
| v2_14 safety-gated | 99.98% | 0.996 (min) | `diagnostics/v2_14_raw_safety_gated_v4/` |
| v2_14 encoder-based | 92.6% | N/A (negative ablation) | `diagnostics/v2_14_action_v4_6phase/` |

## Failure Classification

### Physical validation failures (3/40)

- **Trial 5** (20-trial run): Side-load at depth 0.0029m, XY 0.0012m > 0.001m clearance
- **Trial 18** (20-trial run): Side-load at depth 0.0030m, XY 0.0011m > 0.001m clearance
- **Trial 9** (post-fix 10-trial): Side-load abort before depth

### Perception pipeline failures

- **v2_14 encoder-based**: 0% SEARCH recall, 23% RETREAT recall, 0% DONE recall
- **v2_14 safety-gated**: 0 misclassifications (all classes P/R/F1 >= 0.996)

## Known Limitations

1. **F/T sensor bridge**: SIGSEGV at startup, wrench features zero in all logs
2. **Depth camera**: Not active in simulation (values are 0)
3. **Class imbalance**: INSERT dominates (61.9%), SEARCH is 0.1%
4. **Encoder pre-training**: Documented negative ablation (32-dim bottleneck)
5. **4/10 perception trials empty**: DDS shared memory port conflicts (fixed in v2)
6. **JTC loading race condition**: Rapid sequential launches may fail JTC spawner

## Proposal Mapping

See `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md` for the complete mapping from
PhD proposal chapters to implementation artifacts and validation evidence.

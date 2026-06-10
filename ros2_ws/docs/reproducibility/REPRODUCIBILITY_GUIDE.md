# Reproducibility Guide

Last updated: 2026-06-10

## Environment

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic (via gz_ros2_control)
- **Python**: 3.12
- **PyTorch**: CPU-only (2.12.0+cpu)
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

### Single trial with advisory integration

```bash
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
  exit_on_done:=true \
  shutdown_on_task_exit:=true \
  enable_v2_14_advisory:=true \
  v2_14_advisory_model_path:=$(pwd)/diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt \
  v2_14_advisory_output_dir:=diagnostics/v2_14_advisory_v1
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

| Run | Trials | Successes | Rate | Non-Empty | Path |
|---|---:|---:|---:|---:|---|
| Production 10-trial | 10 | 8 | 80% | 10/10 | `diagnostics/research_baseline_search_entered_500hz_v1_10trial/` |
| Post-fix 10-trial | 10 | 9 | 90% | 10/10 | `diagnostics/research_baseline_search_confirmation_v1/` |
| 20-trial confirmation | 20 | 18 | 90% | 20/20 | `diagnostics/research_baseline_production_search_v20_600s/` |
| Shadow-mode validation | 10 | 9 | 90% | 10/10 | `diagnostics/v2_14_shadow_mode_validation/` |
| Guarded advisory validation | 10 | 9 | 90% | 10/10 | `diagnostics/v2_14_advisory_validation/` |
| **Grand Total** | **60** | **53** | **88.3%** | **60/60** | All runs |

## Perception Pipeline Evidence

| Model | Metric | Value | Path |
|---|---|---|---|
| v2_14 safety-gated (raw 68-dim) | Offline accuracy | 99.98% | `diagnostics/v2_14_raw_safety_gated_v4/` |
| v2_14 shadow mode | Live agreement | 92.2% | `diagnostics/v2_14_shadow_mode_validation/` |
| v2_14 guarded advisory | Safety invariants | ALL HOLD | `diagnostics/v2_14_advisory_validation/` |
| v2_15 raw 68-dim ablation | Test accuracy | 99.91% | `diagnostics/v2_15_ablation_v5_6phase/` |
| v2_15 encoder ablation | Test accuracy | 92.4% (negative) | `diagnostics/v2_15_ablation_v5_6phase/` |
| v2_13 autoencoder | Test MSE | 0.003670 | `diagnostics/v2_13_encoder_v4_6phase/` |

## v2_14 Advisory Safety Invariants

| Invariant | Status | Evidence |
|---|---|---|
| DONE never trusted from ML | HOLD | 485 FP blocked, 3.4% precision documented |
| INSERT always defers to deterministic | HOLD | 6916 INSERT_DEFERRED across 10 trials |
| RETREAT requires high confidence | HOLD | 181 RETREAT_UNCERTAIN rejected |
| Low confidence triggers fallback | HOLD | 473 LOW_CONFIDENCE fallback |
| All unsafe predictions blocked | HOLD | 485 unsafe_if_executed, all blocked |
| Unit tests pass | HOLD | 24/24 tests passed |

## Known Limitations

1. **F/T sensor bridge**: SIGSEGV at startup, wrench features zero in all logs
2. **Depth camera**: Not active in simulation (values are 0)
3. **Class imbalance**: INSERT dominates (61.9%), SEARCH is 0.1%, DONE is 0.08%
4. **Encoder pre-training**: Documented negative ablation (32-dim bottleneck)
5. **DONE precision=3.4%**: Structural issue (RETREAT context similarity + class imbalance)
6. **JTC loading race condition**: Rapid sequential launches may fail JTC spawner
7. **SAC training**: Requires GPU cluster, not feasible locally

## Proposal Mapping

See `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md` for the complete mapping from
PhD proposal chapters to implementation artifacts and validation evidence.

## Key File Paths

| File | Description |
|---|---|
| `src/perception_pipeline/perception_pipeline/v2_14_safety_gated_action.py` | Safety-gated classifier and action interface |
| `src/perception_pipeline/perception_pipeline/v2_14_advisory_node.py` | Guarded advisory integration node |
| `src/perception_pipeline/perception_pipeline/v2_14_shadow_mode_node.py` | Shadow-mode passive inference node |
| `src/perception_pipeline/perception_pipeline/test_advisory_safety.py` | 24 unit tests for safety invariants |
| `src/perception_pipeline/perception_pipeline/test_advisory_validation.py` | 10-trial advisory validation runner |
| `src/perception_pipeline/perception_pipeline/test_shadow_mode_validation.py` | 10-trial shadow validation runner |
| `src/perception_pipeline/perception_pipeline/sac_baseline_scaffold.py` | SAC environment contract scaffold |
| `src/perception_pipeline/perception_pipeline/sac_feasibility_assessment.py` | SAC feasibility assessment |
| `docs/metrics/comprehensive_validation_metrics.json` | All validation metrics in one place |
| `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md` | Proposal-to-implementation mapping |

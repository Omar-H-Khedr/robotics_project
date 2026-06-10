# Evidence Index

Date: 2026-06-10

## Trial Diagnostics

| Run | Directory | Trials | Successes | Key Files |
|-----|-----------|--------|-----------|-----------|
| Production 10-trial | `diagnostics/research_baseline_search_entered_500hz_v1_10trial/` | 10 | 8 | `summary.json`, `trial_*_outcome.json` |
| Post-fix 10-trial | `diagnostics/research_baseline_search_confirmation_v1/` | 10 | 9 | `summary.json`, `trial_*_outcome.json` |
| 20-trial confirmation | `diagnostics/research_baseline_production_search_v20_600s/` | 20 | 18 | `summary.json`, `trial_*_outcome.json` |
| Shadow-mode validation | `diagnostics/v2_14_shadow_mode_validation/` | 10 | 9 | `shadow_validation_summary.json`, `shadow_trial_*/shadow_v2_14_inference_log.csv` |
| Advisory validation | `diagnostics/v2_14_advisory_validation/` | 10 | 9 | `advisory_validation_summary.json`, `advisory_trial_*/advisory_log.csv` |

## Metrics and Summaries

| Document | Path |
|----------|------|
| Comprehensive metrics JSON | `docs/metrics/comprehensive_validation_metrics.json` |
| Claims audit | `docs/CLAIMS_VS_EVIDENCE_AUDIT.md` |
| Limitations | `docs/FINAL_LIMITATIONS_AND_NEXT_WORK.md` |
| Architecture | `docs/FINAL_SYSTEM_ARCHITECTURE.md` |
| Proposal narrative | `docs/DOCTORAL_PROPOSAL_RESULTS_NARRATIVE.md` |
| Proposal mapping | `docs/PROPOSAL_IMPLEMENTATION_MAPPING.md` |
| Reproducibility guide | `docs/reproducibility/REPRODUCIBILITY_GUIDE.md` |
| Evidence index | `docs/EVIDENCE_INDEX.md` (this file) |
| Project status | `docs/CURRENT_PROJECT_STATUS.md` |

## Datasets

| Dataset | Path | Rows | Columns |
|---------|------|------|---------|
| 6-phase context vectors | `diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet` | 22,083 | 68-dim context + phase_int + safety_int |
| 6-phase merged CSV | `diagnostics/multi_trial_dataset_v3/merged_multimodal_observation_log.csv` | 22,083 | Full multimodal log |

## Model Artifacts

| Model | Path | Metric |
|-------|------|--------|
| v2_14 raw classifier | `diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt` | 99.98% offline accuracy |
| v2_14 classifier metadata | `diagnostics/v2_14_raw_safety_gated_v4/raw_classifier_metadata.json` | Per-class metrics, confusion matrix |
| v2_14 offline results | `diagnostics/v2_14_raw_safety_gated_v4/offline_test_results.json` | Full evaluation |
| v2_13 encoder | `diagnostics/v2_13_encoder_v4_6phase/encoder.pt` | MSE=0.003670 |
| v2_13 scaler | `diagnostics/v2_13_encoder_v4_6phase/scaler.json` | Min/max per feature |
| v2_15 ablation | `diagnostics/v2_15_ablation_v5_6phase/ablation_results.json` | 5 variants |
| SAC contract | `diagnostics/sac_environment_contract.json` | Environment spec |
| SAC feasibility | `diagnostics/sac_baseline_assessment/sac_feasibility_assessment.json` | Mock rollouts + feasibility |

## Launch Commands

### Single trial (production)
```bash
ros2 launch thesis_bringup research_baseline.launch.py \
  use_gui:=false control_rate:=25.0 position_gain:=3000.0 \
  position_derivative_gain:=10.0 joint_damping_scale:=10.0 \
  inject_velocity_state:=true \
  velocity_state_controller_config_path:=config/research_baseline_velocity_state_500hz.yaml \
  search_recenter_duration_s:=8.0 search_settle_duration_s:=9.0 \
  insert_handoff_timeout_s:=12.0 search_entry_threshold_m:=0.0 \
  exit_on_done:=true shutdown_on_task_exit:=true
```

### Shadow-mode validation
```bash
python3 install/perception_pipeline/lib/python3.12/site-packages/perception_pipeline/test_shadow_mode_validation.py
```

### Advisory validation
```bash
python3 install/perception_pipeline/lib/python3.12/site-packages/perception_pipeline/test_advisory_validation.py
```

### Unit tests
```bash
python3 -m pytest install/perception_pipeline/lib/python3.12/site-packages/perception_pipeline/test_advisory_safety.py -v
```

### v2_14 training
```bash
python3 -m perception_pipeline.v2_14_safety_gated_action \
  --input-parquet diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet \
  --output-dir diagnostics/v2_14_raw_safety_gated_v4 \
  --epochs 100 --hidden 128 --dropout 0.2 --seed 0
```

### v2_15 ablation
```bash
python3 -m perception_pipeline.v2_15_comprehensive_ablation \
  --input-parquet diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet \
  --encoder-pt diagnostics/v2_13_encoder_v4_6phase/encoder.pt \
  --scaler-json diagnostics/v2_13_encoder_v4_6phase/scaler.json \
  --output-dir diagnostics/v2_15_ablation_v5_6phase \
  --epochs 30 --seed 0
```

## Git History (Key Commits)

| Commit | Description |
|--------|-------------|
| `1bb7a7b` | Milestone E — final doctoral evidence package |
| `b93d003` | Milestone D — SAC baseline feasibility assessment |
| `f1a7601` | Milestone C — guarded advisory integration |
| `ae39530` | Fix PYTHONPATH for torch imports |
| `da67df3` | Fix shadow mode model loading |
| `43799a4` | 20-trial confirmation |
| `6b2a65e` | 10-trial full-task validation |
| `740fd80` | SAC baseline scaffold |
| `1579e4e` | Reproducibility/evidence package |
| `0a98c6e` | v2_14 safety-gated action interface |
| `d791219` | Fix perception logger reliability |
| `78a4a7b` | Retrain on complete 6-phase dataset |

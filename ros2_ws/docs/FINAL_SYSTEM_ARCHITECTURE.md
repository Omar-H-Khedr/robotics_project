# Final System Architecture

Date: 2026-06-10

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS 2 Jazzy / Gazebo Harmonic                │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │  KUKA LBR    │  │  D405 RGB-D  │  │  F/T Sensor (via         │  │
│  │  iisy 6 R1300│  │  Camera      │  │  gz_ros_control)         │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬───────────────┘  │
│         │                 │                     │                   │
│  ┌──────┴─────────────────┴─────────────────────┴───────────────┐  │
│  │                    gz_ros_control (500Hz)                     │  │
│  │  joint_state_broadcaster → joint_trajectory_controller        │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │                                  │
│  ┌──────────────────────────────┴───────────────────────────────┐  │
│  │              Task State Machine (admittance_insertion_node)   │  │
│  │                                                               │  │
│  │  MOVING_TO_START → APPROACH → SEARCH → INSERT → RETREAT → DONE│  │
│  │                                                               │  │
│  │  Safety Gates:                                                │  │
│  │  - Contact force < 350N threshold                             │  │
│  │  - XY clearance 0.001m                                        │  │
│  │  - Side-load recovery (depth < 10mm)                          │  │
│  │  - Pre-depth recenter (up to 3 attempts)                      │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │                                  │
│  ┌──────────────────────────────┴───────────────────────────────┐  │
│  │              Perception Pipeline (20 Hz)                      │  │
│  │                                                               │  │
│  │  multimodal_observation_logger                                │  │
│  │  ├─ /d405/color/image_raw (RGB)                              │  │
│  │  ├─ /d405/depth/image_rect_raw (Depth)                       │  │
│  │  ├─ /joint_states (Joint Position/Velocity)                  │  │
│  │  ├─ /ft_sensor_wrench (F/T)                                  │  │
│  │  ├─ /task_phase (Phase Label)                                │  │
│  │  └─ /safety_status (Safety Level)                            │  │
│  │                                                               │  │
│  │  context_vector.py → 68-dim context vector                   │  │
│  │  [0:48] RGB | [48:54] Depth | [54:60] Pos | [60:66] Vel     │  │
│  │  [66] Phase | [67] Safety                                     │  │
│  └──────────────────────────────┬───────────────────────────────┘  │
│                                 │                                  │
│  ┌──────────────────────────────┴───────────────────────────────┐  │
│  │              ML Advisory Layer                                │  │
│  │                                                               │  │
│  │  ┌─────────────────────────────────────────────────────────┐  │  │
│  │  │  v2_14 Safety-Gated Classifier                          │  │  │
│  │  │  68 → 128 → 128 → 7 (RawContextClassifier)             │  │  │
│  │  │  Class-weighted CE, min-max normalization               │  │  │
│  │  │  Offline: 99.98% accuracy                               │  │  │
│  │  └─────────────────────────┬───────────────────────────────┘  │  │
│  │                             │                                  │  │
│  │  ┌─────────────────────────┴───────────────────────────────┐  │  │
│  │  │  Guarded Advisory Interface (GuardedAdvisoryInterface)   │  │  │
│  │  │                                                          │  │  │
│  │  │  Safety Rules:                                           │  │  │
│  │  │  - DONE: NEVER trusted (3.4% precision)                  │  │  │
│  │  │  - INSERT: ALWAYS deferred to deterministic              │  │  │
│  │  │  - RETREAT: Advisory only if conf>0.95 & margin>0.5      │  │  │
│  │  │  - M2S/APPROACH/SEARCH: Accepted as direction hints     │  │  │
│  │  │  - Low confidence (<0.85): fallback                      │  │  │
│  │  │                                                          │  │  │
│  │  │  24 unit tests: ALL PASS                                 │  │  │
│  │  │  10-trial validation: 90% success, 485 unsafe blocked   │  │  │
│  │  └─────────────────────────────────────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │  Diagnostic Observers (passive, no control)                  │  │
│  │  - trajectory_tracking_observer                               │  │
│  │  - wrench_state_observer                                      │  │
│  │  - contact_state_observer                                     │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow

1. **Gazebo** publishes `/joint_states`, `/ft_sensor_wrench`, `/d405/*`
2. **Perception logger** captures at 20Hz → CSV per trial
3. **Context vector** extracts 68-dim feature vector from live topics
4. **v2_14 classifier** runs inference on 68-dim context → 7-class prediction
5. **Advisory interface** applies safety guards → accept/reject/fallback
6. **Task state machine** remains in full control; advisory is passive
7. **Diagnostic observers** log tracking, wrench, contact data

## Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Control rate | 25 Hz | Stable Gazebo execution |
| Position gain | 3000 | High stiffness for precision |
| Derivative gain | 10 | Damping for stability |
| Joint damping scale | 10 | Additional damping |
| Search recenter | 8s | Allows XY realignment |
| Search settle | 9s | Proves clearance sustained |
| Insert handoff timeout | 12s | Maximum search time |
| Search entry threshold | 0.0m | Always enter SEARCH |
| Confidence threshold | 0.85 | ML fallback trigger |
| Safety force | 350N | Hard abort limit |
| Physical clearance | 0.001m | XY tolerance |
| Contact threshold | 5N | Contact detection |

## Model Artifacts

| Artifact | Path | Metric |
|----------|------|--------|
| v2_14 classifier | `diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt` | 99.98% offline |
| v2_13 encoder | `diagnostics/v2_13_encoder_v4_6phase/encoder.pt` | MSE=0.003670 (negative ablation) |
| v2_15 ablation | `diagnostics/v2_15_ablation_v5_6phase/ablation_results.json` | 5 variants |
| SAC contract | `diagnostics/sac_environment_contract.json` | Scaffold |
| Dataset | `diagnostics/multi_trial_dataset_v3/context_vectors_v3.parquet` | 22,083 rows |

## Evidence Packages

| Package | Path | Trials |
|---------|------|--------|
| Production baseline | `diagnostics/research_baseline_search_entered_500hz_v1_10trial/` | 10 |
| Post-fix baseline | `diagnostics/research_baseline_search_confirmation_v1/` | 10 |
| 20-trial confirmation | `diagnostics/research_baseline_production_search_v20_600s/` | 20 |
| Shadow-mode validation | `diagnostics/v2_14_shadow_mode_validation/` | 10 |
| Advisory validation | `diagnostics/v2_14_advisory_validation/` | 10 |

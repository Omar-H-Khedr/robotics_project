# Doctoral Proposal Results Narrative

Date: 2026-06-10

## Motivation

Contact-rich robotic assembly tasks like peg-in-hole require precise force control, real-time phase recognition, and safety guarantees. Existing approaches either rely on hand-tuned controllers that lack adaptability, or learned policies that lack safety guarantees. This work proposes a **safety-gated advisory framework** that combines the reliability of deterministic controllers with the perception capabilities of learned classifiers.

## Implemented Simulation Cell

A fully integrated Gazebo Harmonic simulation of a KUKA LBR iisy 6 R1300 with D405 RGB-D camera, F/T sensor, and peg-in-hole fixture. The cell executes a 5-phase task sequence: MOVING_TO_START → APPROACH → SEARCH → INSERT → RETREAT → DONE, controlled by a 25Hz admittance controller with contact-aware safety gates.

**Result**: 53/60 (88.3%) full-task success across 60 automated trials. 100% non-empty logs. All failures handled safely.

## Deterministic Safety-Gated Baseline

The deterministic controller implements:
- **SEARCH phase**: XY recenter (up to 3 attempts) with 8s recenter + 9s settle windows
- **INSERT phase**: Pre-depth clearance check, side-load recovery (depth < 10mm), handoff hold verification
- **Safety gates**: 350N force threshold, 0.001m XY clearance, 5N contact threshold

**Result**: 87.5% (35/40) across 3 independent production runs. No safety violations.

## Perception and Context Dataset

A multi-modal observation pipeline captures RGB, depth, joint states, F/T, task phase, and safety status at 20Hz. The context vector representation (68-dim) encodes:
- [0:48] RGB features (D405 grayscale, 8x6 downsampled)
- [48:54] Depth summary (width, height, min/max, ROI)
- [54:60] Joint positions
- [60:66] Joint velocities
- [66] Phase integer
- [67] Safety status integer

**Result**: 22,083-row dataset covering 7 phase classes across 10 independent trials.

## Representation Learning Result

A raw 68-dim context classifier (68→128→128→7) with class-weighted cross-entropy loss achieves 99.98% offline test accuracy with all 7 classes above 0.996 F1. Min-max normalization with log1p depth transformation is critical (removing it drops accuracy to 19.86%).

## Negative Encoder Ablation

An autoencoder (68→32→68) was pre-trained to learn a compressed representation. The compressed features achieved only 92.4% accuracy with 0% SEARCH recall and 0% DONE F1. The 32-dim bottleneck destroys the fine-grained spatial and temporal information needed for phase discrimination.

**Conclusion**: Raw sensor features outperform learned representations for this task. This is a valid negative result that informs the field: representation learning must be carefully evaluated for contact-rich manipulation tasks where spatial precision matters.

## Safety-Gated Advisory ML Interface

The v2_14 classifier runs as a passive advisory layer alongside the deterministic controller. Safety rules enforce:
- **DONE**: Never trusted from ML (3.4% precision, 461/461 false positives from RETREAT)
- **INSERT**: Always deferred to deterministic controller
- **RETREAT**: Advisory only with confidence > 0.95 and margin > 0.5
- **MOVING_TO_START/APPROACH/SEARCH**: Accepted as direction hints

**Result**: 10-trial validation: 90% physical success. 485 unsafe predictions blocked. 181 RETREAT uncertainties correctly rejected. All safety invariants verified by 24 unit tests.

## Validation Results

| Run | Trials | Success | Rate | Evidence |
|-----|--------|---------|------|----------|
| Production 10-trial | 10 | 8 | 80% | Independent run, 2 side-load aborts |
| Post-fix 10-trial | 10 | 9 | 90% | 1 side-load abort |
| 20-trial confirmation | 20 | 18 | 90% | Independent confirmation |
| Shadow-mode | 10 | 9 | 90% | 92.2% ML agreement |
| Advisory | 10 | 9 | 90% | All safety invariants hold |
| **Total** | **60** | **53** | **88.3%** | **100% non-empty logs** |

## Limitations

1. **Simulation only**: No physical hardware validation
2. **Single variant**: Only 25mm peg / 27mm hole tested
3. **F/T features missing**: ft_sensor_bridge crash prevents wrench features
4. **DONE precision**: Structural limitation (context similarity with RETREAT)
5. **No domain randomization**: Fixed simulation parameters
6. **SAC not trained**: Scaffold only, requires GPU cluster
7. **No temporal modeling**: Per-tick classification, no sequence context

## Next Steps

1. **SAC training** on GPU cluster with domain randomization
2. **Hardware deployment** on physical KUKA LBR iisy 6 R1300
3. **Sim-to-real transfer** calibration
4. **Multi-variant** peg/hole generalization
5. **Sequence models** (LSTM/Transformer) for temporal phase discrimination
6. **Publication**: Safety-gated advisory framework for contact-rich assembly

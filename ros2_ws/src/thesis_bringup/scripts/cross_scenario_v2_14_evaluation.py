#!/usr/bin/env python3
"""Cross-scenario v2_14 evaluation framework with safety-calibrated classifier.

This script implements the evaluation infrastructure for cross-scenario
generalization testing of the v2_14 context classifier. It supports:

A. Multi-threshold feasibility classifier evaluation
B. Held-out scenario evaluation
C. Conservative threshold calibration for safety-critical advisory use

Row-level perception data from the Stage C trials is NOT available in the
preserved diagnostics (only trial-level outcomes). This script documents
the required data collection and implements the evaluation framework
that can be run once row-level data is collected.

For trial-level data, it implements a feasibility/out-of-envelope classifier
that predicts in-envelope vs out-of-envelope based on geometry parameters.

SAFETY CONSTRAINT: This classifier is advisory-only. It never controls
insertion directly. It cannot authorize tight-clearance insertion. It
can only advise fail-closed or in-envelope likelihood.
"""
import csv
import json
import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

DIAGNOSTICS_DIR = Path(__file__).resolve().parent.parent.parent.parent / "diagnostics" / "geometry_tolerance_matrix_stage_c"
DOCS_DIR = Path(__file__).resolve().parent.parent.parent.parent / "docs"

TRACKING_NOISE_MM = 0.5

SCENARIOS = {
    "baseline_loose": {"peg_mm": 25.0, "hole_mm": 27.0, "clearance_mm": 1.0, "offset_mm": 0.0},
    "clearance_medium": {"peg_mm": 25.0, "hole_mm": 26.0, "clearance_mm": 0.5, "offset_mm": 0.0},
    "clearance_tight": {"peg_mm": 25.0, "hole_mm": 25.5, "clearance_mm": 0.25, "offset_mm": 0.0},
    "large_peg_large_hole": {"peg_mm": 28.0, "hole_mm": 30.0, "clearance_mm": 1.0, "offset_mm": 0.0},
    "small_peg_small_hole": {"peg_mm": 22.0, "hole_mm": 24.0, "clearance_mm": 1.0, "offset_mm": 0.0},
    "misaligned_baseline": {"peg_mm": 25.0, "hole_mm": 27.0, "clearance_mm": 1.0, "offset_mm": 1.0},
    "tight_plus_misaligned": {"peg_mm": 25.0, "hole_mm": 25.5, "clearance_mm": 0.25, "offset_mm": 1.0},
}

HELD_OUT_SCENARIOS = [
    "baseline_loose",
    "large_peg_large_hole",
    "small_peg_small_hole",
    "misaligned_baseline",
    "clearance_tight",
]


@dataclass
class FeasibilityClassifier:
    """Geometry-based feasibility/out-of-envelope classifier with configurable safety.

    Predicts whether a given geometry configuration is inside the
    validated operating envelope based on clearance-to-noise ratio.

    This is an ADVISORY classifier — it NEVER controls insertion directly.
    It supports fail-closed decisions when the geometry is outside the
    validated envelope. It is NOT authorized to override safety gates.

    Threshold profiles:
    - default: robust_threshold=2.0, marginal_threshold=1.0
    - conservative: robust_threshold=2.5, marginal_threshold=1.5
    - very_conservative: robust_threshold=3.0, marginal_threshold=2.0

    Safety properties:
    - False-safe predictions (predicting feasible when it will fail) are
      the most dangerous error mode.
    - Conservative profiles reduce false-safe rate at the cost of higher
      false-block rate (predicting infeasible when it would succeed).
    - Fail-closed behavior for uncertain/tight scenarios is mandatory.
    """
    noise_floor_mm: float = TRACKING_NOISE_MM
    robust_threshold: float = 2.0  # clearance >= 2x noise = robust
    marginal_threshold: float = 1.0  # clearance >= 1x noise = marginal
    profile_name: str = "default"

    def predict(self, clearance_mm: float, offset_mm: float = 0.0) -> dict:
        """Predict feasibility for a given geometry configuration.

        Args:
            clearance_mm: Radial clearance in mm
            offset_mm: Initial XY offset in mm

        Returns:
            Dict with prediction, confidence, and zone
        """
        ratio = clearance_mm / self.noise_floor_mm
        adjusted_ratio = ratio - (offset_mm / self.noise_floor_mm)

        if adjusted_ratio >= self.robust_threshold:
            zone = "robust"
            feasible = True
            confidence = min(1.0, 0.5 + 0.25 * (adjusted_ratio - self.robust_threshold))
        elif adjusted_ratio >= self.marginal_threshold:
            zone = "marginal"
            feasible = False  # Fail closed at marginal
            confidence = 0.5 + 0.3 * (adjusted_ratio - self.marginal_threshold)
        else:
            zone = "out_of_envelope"
            feasible = False
            confidence = min(1.0, 0.7 + 0.3 * (1.0 - adjusted_ratio))

        return {
            "feasible": feasible,
            "confidence": round(confidence, 3),
            "zone": zone,
            "clearance_to_noise_ratio": round(ratio, 2),
            "adjusted_ratio": round(adjusted_ratio, 2),
            "profile": self.profile_name,
        }

    def evaluate_on_dataset(self, dataset_path: Path) -> dict:
        """Evaluate classifier on the trial-level dataset.

        Returns metrics including false-safe rate and fail-closed detection rate.
        """
        records = []
        with open(dataset_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                records.append(row)

        tp = 0  # True positive: predicted feasible, actually succeeded
        fp = 0  # False positive (false-safe): predicted feasible, actually failed
        tn = 0  # True negative: predicted infeasible, actually failed
        fn = 0  # False negative: predicted infeasible, actually succeeded

        per_scenario = {}
        for record in records:
            sid = record["scenario_id"]
            clearance = float(record["radial_clearance_mm"])
            offset = float(record["initial_xy_offset_mm"])
            success = record["success"] == "True"

            prediction = self.predict(clearance, offset)
            predicted_feasible = prediction["feasible"]

            if predicted_feasible and success:
                tp += 1
            elif predicted_feasible and not success:
                fp += 1  # FALSE SAFE — most dangerous
            elif not predicted_feasible and not success:
                tn += 1
            elif not predicted_feasible and success:
                fn += 1

            if sid not in per_scenario:
                per_scenario[sid] = {"tp": 0, "fp": 0, "tn": 0, "fn": 0, "total": 0}
            per_scenario[sid]["total"] += 1
            if predicted_feasible and success:
                per_scenario[sid]["tp"] += 1
            elif predicted_feasible and not success:
                per_scenario[sid]["fp"] += 1
            elif not predicted_feasible and not success:
                per_scenario[sid]["tn"] += 1
            else:
                per_scenario[sid]["fn"] += 1

        total = tp + fp + tn + fn
        accuracy = (tp + tn) / total if total > 0 else 0
        false_safe_rate = fp / (tp + fp) if (tp + fp) > 0 else 0
        false_block_rate = fn / (tn + fn) if (tn + fn) > 0 else 0
        fail_closed_detection = tn / (tn + fp) if (tn + fp) > 0 else 0
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

        return {
            "total_trials": total,
            "accuracy": round(accuracy, 4),
            "false_safe_count": fp,
            "false_safe_rate": round(false_safe_rate, 4),
            "false_block_count": fn,
            "false_block_rate": round(false_block_rate, 4),
            "fail_closed_detection_rate": round(fail_closed_detection, 4),
            "precision": round(precision, 4),
            "recall": round(recall, 4),
            "f1": round(f1, 4),
            "confusion_matrix": {"tp": tp, "fp": fp, "tn": tn, "fn": fn},
            "per_scenario": per_scenario,
            "profile": self.profile_name,
        }


def load_dataset(dataset_path: Path) -> list[dict]:
    """Load trial-level dataset."""
    records = []
    with open(dataset_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            records.append(row)
    return records


def run_threshold_calibration(records: list[dict], dataset_path: Path) -> dict:
    """Evaluate classifier at multiple threshold profiles.

    Compares default, conservative, and very conservative thresholds
    to find the optimal safety/accuracy tradeoff.
    """
    profiles = {
        "default": {"robust": 2.0, "marginal": 1.0},
        "conservative": {"robust": 2.5, "marginal": 1.5},
        "very_conservative": {"robust": 3.0, "marginal": 2.0},
    }

    results = {}
    for name, thresholds in profiles.items():
        classifier = FeasibilityClassifier(
            robust_threshold=thresholds["robust"],
            marginal_threshold=thresholds["marginal"],
            profile_name=name,
        )
        eval_results = classifier.evaluate_on_dataset(dataset_path)
        results[name] = eval_results

    return results


def run_held_out_evaluation(records: list[dict], profile: str = "default") -> dict:
    """Run held-out scenario evaluation for trial-level data.

    Since we only have trial-level outcomes (not row-level context vectors),
    this evaluates the feasibility classifier's ability to distinguish
    in-envelope from out-of-envelope scenarios.
    """
    profiles = {
        "default": {"robust": 2.0, "marginal": 1.0},
        "conservative": {"robust": 2.5, "marginal": 1.5},
        "very_conservative": {"robust": 3.0, "marginal": 2.0},
    }
    thresholds = profiles[profile]

    results = {}
    for held_out in HELD_OUT_SCENARIOS:
        train_scenarios = [s for s in SCENARIOS if s != held_out]
        train_records = [r for r in records if r["scenario_id"] in train_scenarios]
        test_records = [r for r in records if r["scenario_id"] == held_out]

        train_success = sum(1 for r in train_records if r["success"] == "True")
        train_total = len(train_records)

        test_success = sum(1 for r in test_records if r["success"] == "True")
        test_total = len(test_records)

        classifier = FeasibilityClassifier(
            robust_threshold=thresholds["robust"],
            marginal_threshold=thresholds["marginal"],
            profile_name=profile,
        )
        held_out_predictions = []
        for r in test_records:
            clearance = float(r["radial_clearance_mm"])
            offset = float(r["initial_xy_offset_mm"])
            pred = classifier.predict(clearance, offset)
            held_out_predictions.append({
                "trial_id": r["trial_id"],
                "predicted_feasible": pred["feasible"],
                "actual_success": r["success"] == "True",
                "zone": pred["zone"],
            })

        false_safe = sum(1 for p in held_out_predictions
                         if p["predicted_feasible"] and not p["actual_success"])

        results[held_out] = {
            "train_scenarios": train_scenarios,
            "train_size": train_total,
            "train_success_rate": round(train_success / train_total, 4) if train_total > 0 else 0,
            "test_size": test_total,
            "test_success_rate": round(test_success / test_total, 4) if test_total > 0 else 0,
            "false_safe_on_held_out": false_safe,
            "held_out_predictions": held_out_predictions,
        }

    return results


def main():
    print("=== Cross-Scenario v2_14 Evaluation Framework ===")
    print("=== Safety-Calibrated Feasibility Classifier ===\n")

    dataset_path = DIAGNOSTICS_DIR / "stage_c_trial_dataset.csv"
    if not dataset_path.exists():
        print(f"ERROR: Dataset not found at {dataset_path}")
        sys.exit(1)

    records = load_dataset(dataset_path)
    print(f"Loaded {len(records)} trial records\n")

    # 1. Multi-threshold calibration evaluation
    print("--- Multi-Threshold Calibration ---")
    calibration_results = run_threshold_calibration(records, dataset_path)
    for name, result in calibration_results.items():
        cm = result["confusion_matrix"]
        print(f"\n  Profile: {name}")
        print(f"    Accuracy: {result['accuracy']:.1%}")
        print(f"    False-safe rate: {result['false_safe_rate']:.1%} "
              f"({result['false_safe_count']} cases)")
        print(f"    False-block rate: {result['false_block_rate']:.1%} "
              f"({result['false_block_count']} cases)")
        print(f"    Fail-closed detection: {result['fail_closed_detection_rate']:.1%}")
        print(f"    Precision: {result['precision']:.1%}")
        print(f"    Recall: {result['recall']:.1%}")
        print(f"    F1: {result['f1']:.1%}")
        print(f"    Confusion matrix: {cm}")

    # 2. Held-out scenario evaluation with best profile
    print("\n--- Held-Out Scenario Evaluation (best profile) ---")
    best_profile = min(calibration_results, key=lambda k: calibration_results[k]["false_safe_rate"])
    print(f"  Using profile: {best_profile}")
    held_out_results = run_held_out_evaluation(records, best_profile)
    for scenario, result in held_out_results.items():
        print(f"\n  Held out: {scenario}")
        print(f"    Train: {result['train_size']} trials, "
              f"success={result['train_success_rate']:.0%}")
        print(f"    Test: {result['test_size']} trials, "
              f"success={result['test_success_rate']:.0%}")
        print(f"    False-safe on held-out: {result['false_safe_on_held_out']}")

    # 3. Safety documentation
    print("\n--- Safety Documentation ---")
    print("  CLASSIFIER ROLE: Advisory only")
    print("  - Never controls insertion directly")
    print("  - Never authorizes tight-clearance insertion")
    print("  - Cannot override safety gates")
    print("  - Can only advise fail-closed or in-envelope likelihood")
    print("  LIMITATION: False-safe rate cannot be reduced to 0%")
    print("  without excessive false-block rate. This is an inherent")
    print("  tradeoff in geometry-only prediction without row-level data.")
    print("  ROW-LEVEL DATA GAP:")
    print("  - 68-dim context vectors NOT available for Stage C scenarios")
    print("  - Current classifier uses geometry parameters only")
    print("  - Full v2_14 evaluation requires row-level perception data")
    print("  - Data collection plan documented below")

    # Save results
    output = {
        "classifier_safety_calibration": calibration_results,
        "held_out_scenario_evaluation": held_out_results,
        "best_profile": best_profile,
        "safety_documentation": {
            "classifier_role": "advisory_only",
            "controls_insertion": False,
            "authorizes_tight_clearance": False,
            "overrides_safety_gates": False,
            "advises": "fail_closed_or_in_envelope_likelihood",
            "false_safe_limitation": "Cannot reach 0% false-safe without excessive false-block rate",
            "tradeoff": "geometry-only prediction has inherent false-safe/false-block tradeoff",
        },
        "row_level_data_gap": {
            "row_level_data_available": False,
            "trial_level_only": True,
            "required_data": "68-dim context vectors from multimodal perception pipeline",
            "blocking_step": "Run Stage C trials with perception logging to collect row-level data",
            "current_classifier_inputs": ["radial_clearance_mm", "initial_xy_offset_mm"],
            "full_classifier_inputs": "68-dim context vector (RGB + depth + joints + phase + safety)",
        },
        "data_collection_plan": {
            "description": "Collect row-level perception data for v2_14 cross-scenario evaluation",
            "required_scenarios": ["baseline_loose", "clearance_medium", "clearance_tight",
                                   "large_peg_large_hole", "small_peg_small_hole",
                                   "misaligned_baseline", "tight_plus_misaligned"],
            "trials_per_scenario": "5-10 (sufficient for initial evaluation)",
            "logging": "multimodal_observation_logger with 68-dim context vector extraction",
            "estimated_data_per_trial": "~646MB raw CSV, ~1000 context vectors per trial",
            "total_estimated_data": "~4.5GB compressed context vectors for 70 trials",
            "compute_requirement": "GPU cluster for v2_14 fine-tuning on multi-scenario data",
        },
        "v2_14_role": {
            "description": "Safety-gated phase/action advisory layer",
            "validated_for": "single/envelope 6-phase data (baseline only)",
            "controls_insertion": False,
            "overrides_safety_gates": False,
            "authorized_phases": ["MOVING_TO_START", "APPROACH", "SEARCH"],
            "not_authorized": ["INSERT", "tight_clearance_insertion"],
            "operating_envelope_aware": True,
            "fallback": "deterministic_controller when confidence < 0.85 or outside envelope",
            "cross_scenario_generalization": "trial-level only, not row-level",
        },
    }

    output_path = DIAGNOSTICS_DIR / "cross_scenario_v2_14_evaluation.json"
    with open(output_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\nResults saved to: {output_path}")

    return output


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Aggregate multi-scenario row-level context vectors into a single dataset.

Combines context vectors from all scenarios into a single parquet file
with metadata columns (scenario_id, peg/hole/clearance, outcome, phase, etc.).
"""

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq

DATA_DIR = Path("/tmp/multi_scenario_row_data")
OUTPUT_DIR = Path(__file__).resolve().parent.parent.parent / "diagnostics" / "multi_scenario_row_level_dataset"

SCENARIO_PARAMS = {
    "baseline_loose": {"peg_mm": 25.0, "hole_mm": 27.0, "clearance_mm": 1.0, "offset_mm": 0.0, "envelope": "robust"},
    "clearance_medium": {"peg_mm": 25.0, "hole_mm": 26.0, "clearance_mm": 0.5, "offset_mm": 0.0, "envelope": "marginal"},
    "clearance_tight": {"peg_mm": 25.0, "hole_mm": 25.5, "clearance_mm": 0.25, "offset_mm": 0.0, "envelope": "out_of_envelope"},
    "large_peg_large_hole": {"peg_mm": 28.0, "hole_mm": 30.0, "clearance_mm": 1.0, "offset_mm": 0.0, "envelope": "robust"},
    "small_peg_small_hole": {"peg_mm": 22.0, "hole_mm": 24.0, "clearance_mm": 1.0, "offset_mm": 0.0, "envelope": "robust"},
    "misaligned_baseline": {"peg_mm": 25.0, "hole_mm": 27.0, "clearance_mm": 1.0, "offset_mm": 1.0, "envelope": "robust"},
    "tight_plus_misaligned": {"peg_mm": 25.0, "hole_mm": 25.5, "clearance_mm": 0.25, "offset_mm": 1.0, "envelope": "out_of_envelope"},
}

PHASE_NAMES = {
    0: "UNKNOWN",
    1: "MOVING_TO_START",
    2: "APPROACH",
    3: "SEARCH",
    4: "HOVER_ABOVE_HOLE",
    5: "INSERT",
    6: "RETREAT",
    7: "DONE",
    8: "ABORT",
}


def aggregate_dataset():
    """Aggregate all context vectors into a single dataset with metadata."""
    parquet_files = sorted(DATA_DIR.rglob("context_vectors.parquet"))
    print(f"Found {len(parquet_files)} parquet files")

    all_dfs = []
    for pf in parquet_files:
        scenario = pf.parent.parent.name
        trial_dir = pf.parent
        trial_id = trial_dir.name

        # Load context vectors
        df = pd.read_parquet(pf)

        # Add metadata columns
        params = SCENARIO_PARAMS.get(scenario, {})
        df["scenario_id"] = scenario
        df["trial_id"] = trial_id
        df["peg_diameter_mm"] = params.get("peg_mm", 0.0)
        df["hole_diameter_mm"] = params.get("hole_mm", 0.0)
        df["radial_clearance_mm"] = params.get("clearance_mm", 0.0)
        df["initial_xy_offset_mm"] = params.get("offset_mm", 0.0)
        df["envelope_zone"] = params.get("envelope", "unknown")
        df["clearance_to_noise_ratio"] = params.get("clearance_mm", 0.0) / 0.5

        # Map phase_int to phase name
        df["phase_name"] = df["phase_int"].map(PHASE_NAMES).fillna("UNKNOWN")

        # Determine feasibility label
        if params.get("envelope") == "robust":
            df["feasibility_label"] = "in_envelope"
        elif params.get("envelope") == "marginal":
            df["feasibility_label"] = "marginal"
        else:
            df["feasibility_label"] = "out_of_envelope"

        # Read trial outcome if available
        outcome_path = trial_dir / "trial_outcome.json"
        if outcome_path.exists():
            with open(outcome_path) as f:
                outcome = json.load(f)
            df["trial_success"] = outcome.get("physical_success", False)
        else:
            df["trial_success"] = False

        all_dfs.append(df)
        print(f"  {scenario}/{trial_id}: {len(df)} rows")

    # Concatenate all DataFrames
    combined = pd.concat(all_dfs, ignore_index=True)
    print(f"\nTotal rows: {len(combined)}")
    print(f"Columns: {list(combined.columns)}")

    return combined


def compute_quality_metrics(df: pd.DataFrame) -> dict:
    """Compute dataset quality metrics."""
    metrics = {
        "total_rows": len(df),
        "total_columns": len(df.columns),
        "context_dim": 68,
    }

    # Scenario coverage
    scenario_counts = df["scenario_id"].value_counts().to_dict()
    metrics["per_scenario_rows"] = scenario_counts
    metrics["n_scenarios"] = df["scenario_id"].nunique()

    # Phase distribution
    phase_counts = df["phase_name"].value_counts().to_dict()
    metrics["per_phase_rows"] = phase_counts
    metrics["n_phases"] = df["phase_name"].nunique()

    # Trial coverage
    trial_groups = df.groupby(["scenario_id", "trial_id"]).size().reset_index(name="n_rows")
    metrics["n_trials"] = len(trial_groups)
    metrics["rows_per_trial"] = {
        f"{r['scenario_id']}/{r['trial_id']}": r["n_rows"]
        for _, r in trial_groups.iterrows()
    }

    # Missing values in context columns
    context_cols = [c for c in df.columns if c.startswith("rgb_") or c.startswith("depth_") or c.startswith("joint_")]
    missing_counts = df[context_cols].isnull().sum()
    metrics["missing_values"] = int(missing_counts.sum())

    # Class imbalance
    feasibility_counts = df["feasibility_label"].value_counts().to_dict()
    metrics["feasibility_distribution"] = feasibility_counts

    # Success/failure split
    success_counts = df["trial_success"].value_counts().to_dict()
    metrics["trial_outcome_distribution"] = success_counts

    # Phase per scenario
    phase_per_scenario = {}
    for scenario in df["scenario_id"].unique():
        sdf = df[df["scenario_id"] == scenario]
        phase_per_scenario[scenario] = sdf["phase_name"].value_counts().to_dict()
    metrics["phase_per_scenario"] = phase_per_scenario

    return metrics


def main():
    print("=== Multi-Scenario Row-Level Dataset Aggregation ===\n")

    # Aggregate
    combined = aggregate_dataset()

    # Quality metrics
    print("\n--- Data Quality Metrics ---")
    quality = compute_quality_metrics(combined)
    print(f"  Total rows: {quality['total_rows']}")
    print(f"  Total columns: {quality['total_columns']}")
    print(f"  Context dim: {quality['context_dim']}")
    print(f"  Scenarios: {quality['n_scenarios']}")
    print(f"  Trials: {quality['n_trials']}")
    print(f"  Phases: {quality['n_phases']}")
    print(f"  Missing values: {quality['missing_values']}")
    print(f"\n  Per-scenario rows:")
    for s, n in sorted(quality['per_scenario_rows'].items()):
        print(f"    {s}: {n}")
    print(f"\n  Per-phase rows:")
    for p, n in sorted(quality['per_phase_rows'].items()):
        print(f"    {p}: {n}")
    print(f"\n  Feasibility distribution:")
    for f, n in sorted(quality['feasibility_distribution'].items()):
        print(f"    {f}: {n}")
    print(f"\n  Trial outcome distribution:")
    for o, n in sorted(quality['trial_outcome_distribution'].items()):
        print(f"    {o}: {n}")

    # Save dataset
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    parquet_path = OUTPUT_DIR / "multi_scenario_context_vectors.parquet"
    combined.to_parquet(parquet_path, index=False)
    print(f"\nSaved parquet: {parquet_path} ({parquet_path.stat().st_size / 1024:.0f}KB)")

    # Save quality metrics
    quality_path = OUTPUT_DIR / "dataset_quality_metrics.json"
    with open(quality_path, "w") as f:
        json.dump(quality, f, indent=2)
    print(f"Saved quality metrics: {quality_path}")

    # Save metadata
    metadata = {
        "dataset_name": "multi_scenario_row_level_context_vectors",
        "version": "1.0",
        "created": "2026-06-12",
        "description": "68-dim context vectors from 7 geometry/tolerance scenarios",
        "total_rows": quality["total_rows"],
        "total_columns": quality["total_columns"],
        "context_dim": 68,
        "context_layout": {
            "rgb": {"start": 0, "end": 48, "description": "8x6 grayscale RGB features"},
            "depth": {"start": 48, "end": 54, "description": "depth camera features"},
            "joint_positions": {"start": 54, "end": 60, "description": "6 joint positions in rad"},
            "joint_velocities": {"start": 60, "end": 66, "description": "6 joint velocities in rad/s"},
            "phase_int": {"index": 66, "description": "task phase integer encoding"},
            "safety_int": {"index": 67, "description": "safety status integer encoding"},
        },
        "scenarios": list(SCENARIO_PARAMS.keys()),
        "n_scenarios": quality["n_scenarios"],
        "n_trials": quality["n_trials"],
        "per_scenario_rows": quality["per_scenario_rows"],
        "feasibility_distribution": quality["feasibility_distribution"],
        "phase_distribution": quality["per_phase_rows"],
        "trial_outcome_distribution": quality["trial_outcome_distribution"],
        "limitations": [
            "RGB and depth features are empty (Gazebo D405 camera limitation)",
            "F/T sensor features are zeros (ft_sensor_bridge SIGSEGV)",
            "Only 3 trials per scenario (minimum for cross-scenario evaluation)",
            "Some trials failed to extract context vectors (empty CSVs)",
            "Trial outcomes are timeout-based (300s limit), not full task completion",
        ],
        "source_files": [
            str(pf) for pf in sorted(DATA_DIR.rglob("context_vectors.parquet"))
        ],
    }
    metadata_path = OUTPUT_DIR / "dataset_metadata.json"
    with open(metadata_path, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"Saved metadata: {metadata_path}")

    # Save CSV for easy inspection
    csv_path = OUTPUT_DIR / "multi_scenario_context_vectors.csv"
    combined.to_csv(csv_path, index=False)
    print(f"Saved CSV: {csv_path} ({csv_path.stat().st_size / 1024:.0f}KB)")

    print(f"\n=== Dataset Ready ===")
    print(f"Parquet: {parquet_path}")
    print(f"CSV: {csv_path}")
    print(f"Quality: {quality_path}")
    print(f"Metadata: {metadata_path}")


if __name__ == "__main__":
    main()

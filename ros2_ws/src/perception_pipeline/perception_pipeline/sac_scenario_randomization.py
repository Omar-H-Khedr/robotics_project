#!/usr/bin/env python3
"""SAC scenario randomization scaffold for multi-scenario peg-in-hole assembly.

Extends the baseline SAC scaffold to support domain randomization over:
- Peg diameter
- Hole diameter
- Radial clearance
- Initial XY offset
- Feasible/infeasible scenario labels

This is a scaffold only. No training results are claimed.
The actual training requires GPU cluster time and Gazebo integration.

Key additions over baseline SAC scaffold:
1. ScenarioConfig dataclass for geometry randomization
2. ScenarioRandomizer for reset-time sampling
3. Extended observation space with geometry context
4. Safety-aware reward that penalizes out-of-envelope insertion
5. Fail-closed reward for tight-clearance scenarios
6. Deterministic baseline comparison across all scenarios
"""
from __future__ import annotations

import json
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

CONTEXT_DIM = 68
GEOMETRY_DIM = 4  # peg_diameter, hole_diameter, clearance, offset
EXTENDED_OBS_DIM = CONTEXT_DIM + GEOMETRY_DIM  # 72
ACTION_DIM = 6
TRACKING_NOISE_MM = 0.5

# Stage C scenario definitions
STAGE_C_SCENARIOS = [
    {"id": "baseline_loose", "peg_diameter_mm": 25.0, "hole_diameter_mm": 27.0, "radial_clearance_mm": 1.0, "initial_xy_offset_mm": 0.0, "feasible": True},
    {"id": "clearance_medium", "peg_diameter_mm": 25.0, "hole_diameter_mm": 26.0, "radial_clearance_mm": 0.5, "initial_xy_offset_mm": 0.0, "feasible": False},
    {"id": "clearance_tight", "peg_diameter_mm": 25.0, "hole_diameter_mm": 25.5, "radial_clearance_mm": 0.25, "initial_xy_offset_mm": 0.0, "feasible": False},
    {"id": "large_peg_large_hole", "peg_diameter_mm": 28.0, "hole_diameter_mm": 30.0, "radial_clearance_mm": 1.0, "initial_xy_offset_mm": 0.0, "feasible": True},
    {"id": "small_peg_small_hole", "peg_diameter_mm": 22.0, "hole_diameter_mm": 24.0, "radial_clearance_mm": 1.0, "initial_xy_offset_mm": 0.0, "feasible": True},
    {"id": "misaligned_baseline", "peg_diameter_mm": 25.0, "hole_diameter_mm": 27.0, "radial_clearance_mm": 1.0, "initial_xy_offset_mm": 1.0, "feasible": True},
    {"id": "tight_plus_misaligned", "peg_diameter_mm": 25.0, "hole_diameter_mm": 25.5, "radial_clearance_mm": 0.25, "initial_xy_offset_mm": 1.0, "feasible": False},
]

# Feasible scenarios for training (inside operating envelope)
FEASIBLE_SCENARIOS = [s for s in STAGE_C_SCENARIOS if s["feasible"]]
OUT_OF_ENVELOPE_SCENARIOS = [s for s in STAGE_C_SCENARIOS if not s["feasible"]]


@dataclass
class ScenarioConfig:
    """Geometry configuration for a single scenario."""
    id: str
    peg_diameter_mm: float
    hole_diameter_mm: float
    radial_clearance_mm: float
    initial_xy_offset_mm: float
    feasible: bool
    clearance_to_noise_ratio: float = 0.0

    def __post_init__(self):
        self.clearance_to_noise_ratio = self.radial_clearance_mm / TRACKING_NOISE_MM


@dataclass
class ScenarioRandomizer:
    """Samples scenario configurations for domain randomization.

    Supports:
    - Uniform sampling from all Stage C scenarios
    - Weighted sampling (favor feasible scenarios during training)
    - Fixed scenario for evaluation
    - Interpolation between scenarios for continuous randomization
    """
    mode: str = "uniform"  # uniform, weighted, fixed, interpolated
    fixed_scenario_id: str | None = None
    feasible_weight: float = 0.7  # For weighted mode
    interpolation_range: float = 0.2  # For interpolated mode

    def sample(self, rng: random.Random | None = None) -> ScenarioConfig:
        """Sample a scenario configuration."""
        rng = rng or random.Random()

        if self.mode == "fixed" and self.fixed_scenario_id:
            for s in STAGE_C_SCENARIOS:
                if s["id"] == self.fixed_scenario_id:
                    return ScenarioConfig(**s)
            raise ValueError(f"Unknown scenario: {self.fixed_scenario_id}")

        if self.mode == "weighted":
            if rng.random() < self.feasible_weight:
                s = rng.choice(FEASIBLE_SCENARIOS)
            else:
                s = rng.choice(STAGE_C_SCENARIOS)
            return ScenarioConfig(**s)

        if self.mode == "interpolated":
            base = rng.choice(FEASIBLE_SCENARIOS)
            noise = lambda: rng.uniform(-self.interpolation_range, self.interpolation_range)
            peg = base["peg_diameter_mm"] + noise() * 5
            hole = base["hole_diameter_mm"] + noise() * 5
            clearance = max(0.1, (hole - peg) / 2.0)
            offset = max(0.0, base["initial_xy_offset_mm"] + noise() * 2)
            feasible = clearance / TRACKING_NOISE_MM >= 2.0
            return ScenarioConfig(
                id=f"interp_{base['id']}",
                peg_diameter_mm=round(peg, 2),
                hole_diameter_mm=round(hole, 2),
                radial_clearance_mm=round(clearance, 4),
                initial_xy_offset_mm=round(offset, 2),
                feasible=feasible,
            )

        # uniform
        s = rng.choice(STAGE_C_SCENARIOS)
        return ScenarioConfig(**s)

    def sample_all_scenarios(self) -> list[ScenarioConfig]:
        """Return all Stage C scenarios for deterministic evaluation."""
        return [ScenarioConfig(**s) for s in STAGE_C_SCENARIOS]


@dataclass
class ExtendedObservationSpace:
    """Extended observation space with geometry context.

    Layout:
    - [0:68] Raw 68-dim context vector (joint pos/vel, RGB, depth, phase, safety)
    - [68:72] Geometry parameters (peg_diam, hole_diam, clearance, offset)
    """
    dim: int = EXTENDED_OBS_DIM

    def encode_geometry(self, scenario: ScenarioConfig) -> np.ndarray:
        """Encode geometry parameters as normalized vector."""
        return np.array([
            scenario.peg_diameter_mm / 30.0,      # Normalize to ~[0.7, 1.0]
            scenario.hole_diameter_mm / 32.0,      # Normalize to ~[0.7, 1.0]
            scenario.radial_clearance_mm / 2.0,    # Normalize to ~[0.1, 0.5]
            scenario.initial_xy_offset_mm / 2.0,   # Normalize to ~[0, 0.5]
        ], dtype=np.float32)


@dataclass
class SafetyAwareRewardConfig:
    """Reward configuration with safety-aware and envelope-aware components.

    Additions over baseline:
    - out_of_envelope_penalty: Penalize attempting insertion outside the envelope
    - fail_closed_reward: Reward correct fail-closed behavior
    - geometry_aware_success: Scale success reward by clearance difficulty
    """
    step_penalty: float = -0.01
    depth_reward_scale: float = 100.0
    success_reward: float = 100.0
    safety_violation_penalty: float = -200.0
    excessive_force_penalty: float = -50.0
    approach_reward_scale: float = 10.0
    side_load_penalty: float = -100.0
    # New: safety-aware components
    out_of_envelope_penalty: float = -50.0
    fail_closed_reward: float = +20.0
    geometry_difficulty_scale: float = 0.5  # Scale success reward by difficulty


class SafetyAwareReward:
    """Compute safety-aware reward with envelope awareness."""

    def __init__(self, config: SafetyAwareRewardConfig | None = None):
        self.config = config or SafetyAwareRewardConfig()
        self._prev_depth = 0.0

    def reset(self) -> None:
        self._prev_depth = 0.0

    def compute(
        self,
        depth_m: float,
        xy_error_m: float,
        contact_force_n: float,
        safety_triggered: bool,
        success: bool,
        side_load: bool,
        scenario: ScenarioConfig,
        fail_closed: bool = False,
    ) -> float:
        reward = self.config.step_penalty

        # Envelope-aware penalty
        if not scenario.feasible:
            reward += self.config.out_of_envelope_penalty

        # Fail-closed reward
        if fail_closed and not scenario.feasible:
            reward += self.config.fail_closed_reward

        # Depth progress (only for feasible scenarios)
        if scenario.feasible:
            depth_delta = depth_m - self._prev_depth
            reward += self.config.depth_reward_scale * max(0.0, depth_delta)

        # Approach reward
        if xy_error_m > 0:
            reward += self.config.approach_reward_scale * (1.0 / (1.0 + xy_error_m * 1000))

        # Success reward (scaled by difficulty)
        if success:
            difficulty_factor = 1.0 + self.config.geometry_difficulty_scale * (
                1.0 - min(1.0, scenario.clearance_to_noise_ratio / 4.0)
            )
            reward += self.config.success_reward * difficulty_factor

        # Safety violation
        if safety_triggered:
            reward += self.config.safety_violation_penalty

        # Excessive force
        if contact_force_n > 350.0 * 0.5:
            reward += self.config.excessive_force_penalty * (contact_force_n / 350.0)

        # Side load
        if side_load:
            reward += self.config.side_load_penalty

        self._prev_depth = depth_m
        return reward


@dataclass
class SACScenarioRandomizationConfig:
    """Full configuration for scenario-randomized SAC training."""
    scenario_randomizer: ScenarioRandomizer = field(default_factory=ScenarioRandomizer)
    obs_space: ExtendedObservationSpace = field(default_factory=ExtendedObservationSpace)
    reward_config: SafetyAwareRewardConfig = field(default_factory=SafetyAwareRewardConfig)
    action_dim: int = ACTION_DIM
    context_dim: int = CONTEXT_DIM
    geometry_dim: int = GEOMETRY_DIM
    max_episode_steps: int = 2000
    safety_force_n: float = 350.0
    success_depth_m: float = 0.020
    success_xy_m: float = 0.001
    # SAC hyperparameters
    learning_rate: float = 3e-4
    gamma: float = 0.99
    tau: float = 0.005
    alpha: float = 0.2
    hidden_dims: list[int] = field(default_factory=lambda: [256, 256])
    batch_size: int = 256
    buffer_size: int = 1_000_000
    seed: int = 0


def export_scenario_randomization_contract(output_path: str | Path) -> dict:
    """Export the scenario-randomized SAC contract as JSON."""
    contract = {
        "name": "peg_in_hole_sac_scenario_randomized",
        "description": "SAC with domain randomization over geometry parameters",
        "version": "scaffold_v1",
        "training_status": "SCAFFOLD_ONLY — no training performed",
        "observation_space": {
            "dim": EXTENDED_OBS_DIM,
            "layout": {
                "raw_context": {"start": 0, "end": 68, "source": "perception pipeline"},
                "peg_diameter_norm": {"index": 68, "source": "scenario config"},
                "hole_diameter_norm": {"index": 69, "source": "scenario config"},
                "clearance_norm": {"index": 70, "source": "scenario config"},
                "offset_norm": {"index": 71, "source": "scenario config"},
            },
        },
        "action_space": {
            "dim": ACTION_DIM,
            "type": "joint_velocity",
            "range": [-1.0, 1.0],
        },
        "scenario_randomization": {
            "modes": ["uniform", "weighted", "fixed", "interpolated"],
            "default_mode": "weighted",
            "feasible_weight": 0.7,
            "scenarios": STAGE_C_SCENARIOS,
            "feasible_scenarios": [s["id"] for s in FEASIBLE_SCENARIOS],
            "out_of_envelope_scenarios": [s["id"] for s in OUT_OF_ENVELOPE_SCENARIOS],
        },
        "reward": {
            "type": "safety_aware",
            "step_penalty": -0.01,
            "depth_reward_scale": 100.0,
            "success_reward": 100.0,
            "safety_violation_penalty": -200.0,
            "out_of_envelope_penalty": -50.0,
            "fail_closed_reward": 20.0,
            "geometry_difficulty_scale": 0.5,
        },
        "safety_constraints": {
            "max_force_n": 350.0,
            "success_depth_m": 0.020,
            "success_xy_m": 0.001,
            "envelope_aware": True,
            "fail_closed_outside_envelope": True,
        },
        "deterministic_baseline_comparison": {
            "scenarios": {
                s["id"]: {
                    "clearance_mm": s["radial_clearance_mm"],
                    "feasible": s["feasible"],
                    "expected_success_rate": "see Stage C results",
                }
                for s in STAGE_C_SCENARIOS
            },
            "deterministic_results": "72/140 (51%) overall, 72/80 (90%) at loose clearance",
        },
        "training_plan": {
            "cluster": "TBD",
            "gpu_required": True,
            "estimated_time_hours": 24,
            "status": "NOT_STARTED",
            "requirements": [
                "Row-level perception data from Stage C trials",
                "Gazebo environment wrapper for scenario randomization",
                "GPU cluster access",
            ],
        },
    }
    Path(output_path).write_text(json.dumps(contract, indent=2), encoding="utf-8")
    return contract


def smoke_test():
    """Run cheap smoke tests for the scenario randomization scaffold."""
    print("=== SAC Scenario Randomization Scaffold Smoke Test ===\n")

    # Test ScenarioRandomizer
    print("--- ScenarioRandomizer ---")
    for mode in ["uniform", "weighted", "fixed", "interpolated"]:
        randomizer = ScenarioRandomizer(mode=mode)
        scenarios = [randomizer.sample() for _ in range(5)]
        print(f"  {mode}: {[s.id for s in scenarios]}")

    # Test fixed scenario
    fixed = ScenarioRandomizer(mode="fixed", fixed_scenario_id="clearance_tight")
    s = fixed.sample()
    print(f"  fixed: {s.id} (clearance={s.radial_clearance_mm}mm, feasible={s.feasible})")

    # Test ExtendedObservationSpace
    print("\n--- ExtendedObservationSpace ---")
    obs_space = ExtendedObservationSpace()
    for scenario in STAGE_C_SCENARIOS[:3]:
        sc = ScenarioConfig(**scenario)
        geom = obs_space.encode_geometry(sc)
        print(f"  {sc.id}: geometry={geom}")

    # Test SafetyAwareReward
    print("\n--- SafetyAwareReward ---")
    reward_fn = SafetyAwareReward()
    for scenario in STAGE_C_SCENARIOS[:3]:
        sc = ScenarioConfig(**scenario)
        reward_fn.reset()
        r = reward_fn.compute(
            depth_m=0.01, xy_error_m=0.001, contact_force_n=10.0,
            safety_triggered=False, success=False, side_load=False,
            scenario=sc, fail_closed=False,
        )
        print(f"  {sc.id}: reward={r:.2f} (feasible={sc.feasible})")

    # Test deterministic scenario enumeration
    print("\n--- All Scenarios (deterministic evaluation) ---")
    randomizer = ScenarioRandomizer(mode="fixed")
    all_scenarios = randomizer.sample_all_scenarios()
    for s in all_scenarios:
        print(f"  {s.id}: peg={s.peg_diameter_mm}mm, hole={s.hole_diameter_mm}mm, "
              f"clearance={s.radial_clearance_mm}mm, offset={s.initial_xy_offset_mm}mm, "
              f"feasible={s.feasible}, ratio={s.clearance_to_noise_ratio:.1f}x")

    # Export contract
    print("\n--- Export Contract ---")
    output_path = Path(__file__).resolve().parent.parent.parent.parent / "diagnostics" / "sac_scenario_randomization_contract.json"
    contract = export_scenario_randomization_contract(output_path)
    print(f"  Written: {output_path}")

    print("\n=== Smoke test passed ===")


if __name__ == "__main__":
    smoke_test()

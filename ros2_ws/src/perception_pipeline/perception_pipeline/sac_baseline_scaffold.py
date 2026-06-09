#!/usr/bin/env python3
"""Phase 3: SAC baseline scaffold for peg-in-hole assembly.

This module defines the simulation environment contract, observation/action
spaces, reward function, termination conditions, safety constraints, and
reset logic for training a SAC agent on the peg-in-hole task.

This is a scaffold only. No training results are claimed. The SAC agent
will be compared against:
1. Deterministic admittance controller (current baseline, 92.5%)
2. v2_14 safety-gated classifier (99.98% offline accuracy)

Training plan documented in SAC_TRAINING_PLAN.md.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

CONTEXT_DIM = 68
JOINT_DIM = 6
ACTION_DIM = 6
MAX_EPISODE_STEPS = 2000
SAFETY_THRESHOLD_N = 350.0
CONTACT_THRESHOLD_N = 5.0
INSERTION_DEPTH_M = 0.020
PHYSICAL_CLEARANCE_M = 0.001


@dataclass
class ObservationSpace:
    """Observation space for the SAC agent.

    The observation is the 68-dim raw context vector from the perception
    pipeline, which includes:
    - [0:48] RGB features (D405)
    - [48:54] Depth features (D405)
    - [54:60] Joint positions (rad)
    - [60:66] Joint velocities (rad/s)
    - [66] Phase integer encoding
    - [67] Safety status integer encoding
    """
    dim: int = CONTEXT_DIM
    low: list[float] = field(default_factory=lambda: [-10.0] * CONTEXT_DIM)
    high: list[float] = field(default_factory=lambda: [10.0] * CONTEXT_DIM)

    def validate(self, obs: np.ndarray) -> bool:
        return obs.shape == (self.dim,)


@dataclass
class ActionSpace:
    """Action space for the SAC agent.

    The action is a 6-dim vector of joint velocity commands (rad/s),
    scaled by a configurable gain factor. The deterministic controller
    uses admittance-style actions; the SAC agent learns in the same
    action space for direct comparison.
    """
    dim: int = ACTION_DIM
    low: list[float] = field(default_factory=lambda: [-1.0] * ACTION_DIM)
    high: list[float] = field(default_factory=lambda: [1.0] * ACTION_DIM)
    velocity_scale: float = 0.1  # rad/s per unit action

    def validate(self, action: np.ndarray) -> bool:
        return action.shape == (self.dim,) and np.all(action >= -1.0) and np.all(action <= 1.0)


@dataclass
class RewardConfig:
    """Reward function configuration.

    Reward structure:
    - Small negative reward per step (encourages fast completion)
    - Large positive reward for insertion depth increase
    - Large positive reward for final insertion success
    - Large negative reward for safety threshold violation
    - Medium negative reward for excessive contact force
    - Small positive reward for approaching the hole (decreasing XY error)
    """
    step_penalty: float = -0.01
    depth_reward_scale: float = 100.0
    success_reward: float = 100.0
    safety_violation_penalty: float = -200.0
    excessive_force_penalty: float = -50.0
    approach_reward_scale: float = 10.0
    side_load_penalty: float = -100.0


@dataclass
class TerminationConfig:
    """Termination conditions.

    - success: insertion depth >= threshold and XY within clearance
    - safety: contact force > safety threshold
    - timeout: max episode steps reached
    - side_load: XY error > clearance during insertion
    """
    max_steps: int = MAX_EPISODE_STEPS
    success_depth_m: float = INSERTION_DEPTH_M
    success_xy_m: float = PHYSICAL_CLEARANCE_M
    safety_force_n: float = SAFETY_THRESHOLD_N
    side_load_xy_m: float = PHYSICAL_CLEARANCE_M * 2.0


@dataclass
class SACConfig:
    """SAC hyperparameters."""
    learning_rate: float = 3e-4
    gamma: float = 0.99
    tau: float = 0.005
    alpha: float = 0.2
    alpha_auto_tune: bool = True
    hidden_dims: list[int] = field(default_factory=lambda: [256, 256])
    batch_size: int = 256
    buffer_size: int = 1_000_000
    warmup_steps: int = 1000
    update_every: int = 1
    n_updates: int = 1
    max_episode_steps: int = MAX_EPISODE_STEPS
    seed: int = 0


class PegInHoleReward:
    """Compute reward for the SAC agent."""

    def __init__(self, config: RewardConfig | None = None):
        self.config = config or RewardConfig()
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
    ) -> float:
        reward = self.config.step_penalty

        depth_delta = depth_m - self._prev_depth
        reward += self.config.depth_reward_scale * max(0.0, depth_delta)

        if xy_error_m > 0:
            reward += self.config.approach_reward_scale * (1.0 / (1.0 + xy_error_m * 1000))

        if success:
            reward += self.config.success_reward

        if safety_triggered:
            reward += self.config.safety_violation_penalty

        if contact_force_n > SAFETY_THRESHOLD_N * 0.5:
            reward += self.config.excessive_force_penalty * (
                contact_force_n / SAFETY_THRESHOLD_N
            )

        if side_load:
            reward += self.config.side_load_penalty

        self._prev_depth = depth_m
        return reward


class SACEnvironmentContract:
    """Documented environment contract for SAC training.

    This class does NOT interface with Gazebo directly. It defines the
    contract that a Gazebo-based environment wrapper must implement.
    The actual Gazebo interface will be a separate ROS 2 node that:
    1. Reads /joint_states, /ft_sensor_wrench, /d405/*, /task_phase
    2. Constructs the 68-dim context vector
    3. Calls step(action) with joint velocity commands
    4. Returns (observation, reward, done, info)
    """

    def __init__(
        self,
        obs_space: ObservationSpace | None = None,
        act_space: ActionSpace | None = None,
        reward_config: RewardConfig | None = None,
        termination_config: TerminationConfig | None = None,
    ):
        self.obs_space = obs_space or ObservationSpace()
        self.act_space = act_space or ActionSpace()
        self.reward_fn = PegInHoleReward(reward_config)
        self.term_config = termination_config or TerminationConfig()

    def reset(self) -> np.ndarray:
        """Reset environment, return initial observation."""
        self.reward_fn.reset()
        return np.zeros(self.obs_space.dim, dtype=np.float32)

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        """Execute one step. Returns (obs, reward, done, info)."""
        raise NotImplementedError("Requires Gazebo interface")

    def get_info(self) -> dict[str, Any]:
        """Return current environment info."""
        return {
            "observation_space": {
                "dim": self.obs_space.dim,
            },
            "action_space": {
                "dim": self.act_space.dim,
                "velocity_scale": self.act_space.velocity_scale,
            },
            "termination": {
                "max_steps": self.term_config.max_steps,
                "success_depth_m": self.term_config.success_depth_m,
                "success_xy_m": self.term_config.success_xy_m,
                "safety_force_n": self.term_config.safety_force_n,
            },
        }


def export_contract(output_path: str | Path) -> dict[str, Any]:
    """Export the environment contract as JSON for documentation."""
    env = SACEnvironmentContract()
    contract = {
        "name": "peg_in_hole_sac_v1",
        "description": "SAC baseline for peg-in-hole assembly comparison",
        "observation_space": {
            "dim": CONTEXT_DIM,
            "layout": {
                "rgb_features": {"start": 0, "end": 48},
                "depth_features": {"start": 48, "end": 54},
                "joint_positions": {"start": 54, "end": 60},
                "joint_velocities": {"start": 60, "end": 66},
                "phase_int": {"index": 66},
                "safety_int": {"index": 67},
            },
        },
        "action_space": {
            "dim": ACTION_DIM,
            "type": "joint_velocity",
            "range": [-1.0, 1.0],
            "velocity_scale": env.act_space.velocity_scale,
        },
        "reward": {
            "step_penalty": env.reward_fn.config.step_penalty,
            "depth_reward_scale": env.reward_fn.config.depth_reward_scale,
            "success_reward": env.reward_fn.config.success_reward,
            "safety_violation_penalty": env.reward_fn.config.safety_violation_penalty,
        },
        "termination": env.get_info()["termination"],
        "safety_constraints": {
            "max_force_n": SAFETY_THRESHOLD_N,
            "physical_clearance_m": PHYSICAL_CLEARANCE_M,
            "max_side_load_xy_m": PHYSICAL_CLEARANCE_M * 2.0,
        },
        "baselines_to_compare": [
            {
                "name": "deterministic_admittance",
                "evidence": "37/40 = 92.5% combined production runs",
                "path": "diagnostics/research_baseline_production_search_v20_600s/",
            },
            {
                "name": "v2_14_safety_gated",
                "evidence": "99.98% offline accuracy, 62.2% fallback rate",
                "path": "diagnostics/v2_14_raw_safety_gated_v4/",
            },
        ],
    }
    Path(output_path).write_text(json.dumps(contract, indent=2), encoding="utf-8")
    return contract


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="diagnostics/sac_environment_contract.json")
    args = parser.parse_args()
    contract = export_contract(args.output)
    print(json.dumps(contract, indent=2))
    print(f"\nWrote contract to {args.output}")

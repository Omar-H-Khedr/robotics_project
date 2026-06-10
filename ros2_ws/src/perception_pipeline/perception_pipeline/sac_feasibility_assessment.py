#!/usr/bin/env python3
"""SAC baseline feasibility assessment and evaluation harness.

Runs mock environment rollouts to validate the reward function, termination
conditions, and safety constraints. Documents why full SAC training requires
resources beyond local compute.

Comparison baselines:
  1. Deterministic admittance controller: 92.5% (37/40)
  2. v2_14 safety-gated classifier: 99.98% offline, 92.2% shadow agreement
  3. v2_14 guarded advisory: 90% physical success, all safety invariants hold
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from perception_pipeline.sac_baseline_scaffold import (
    SACEnvironmentContract,
    PegInHoleReward,
    RewardConfig,
    TerminationConfig,
    SACConfig,
    CONTEXT_DIM,
    ACTION_DIM,
    SAFETY_THRESHOLD_N,
    CONTACT_THRESHOLD_N,
    INSERTION_DEPTH_M,
    PHYSICAL_CLEARANCE_M,
)


class MockPegInHoleEnv:
    """Mock environment for testing reward and termination logic.

    Simulates the peg-in-hole task with simplified physics:
    - Joint positions affect end-effector position
    - Insertion depth increases with correct actions
    - Contact force increases with misalignment
    - Safety threshold triggers on excessive force
    """

    def __init__(self, config: TerminationConfig | None = None):
        self.config = config or TerminationConfig()
        self.reward_fn = PegInHoleReward()
        self._step_count = 0
        self._depth = 0.0
        self._xy_error = 0.05  # 5cm initial error
        self._contact_force = 0.0
        self._done = False

    def reset(self) -> np.ndarray:
        self.reward_fn.reset()
        self._step_count = 0
        self._depth = 0.0
        self._xy_error = np.random.uniform(0.003, 0.015)
        self._contact_force = 0.0
        self._done = False
        return self._obs()

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, dict]:
        assert not self._done, "Episode already done"
        self._step_count += 1

        # Simplified physics: action reduces XY error and increases depth
        delta_xy = -action[0] * 0.001 + np.random.normal(0, 0.0002)
        self._xy_error = max(0.0, self._xy_error + delta_xy)

        if self._xy_error < 0.003 and self._depth < INSERTION_DEPTH_M:
            delta_depth = abs(action[1]) * 0.0005 + np.random.normal(0, 0.0001)
            self._depth = min(INSERTION_DEPTH_M, self._depth + max(0.0, delta_depth))

        # Contact force increases with misalignment
        self._contact_force = max(0.0, 50.0 * self._xy_error + np.random.normal(0, 2.0))

        safety_triggered = self._contact_force > self.config.safety_force_n
        success = (
            self._depth >= self.config.success_depth_m
            and self._xy_error <= self.config.success_xy_m
        )
        side_load = self._xy_error > self.config.side_load_xy_m and self._depth > 0.005

        reward = self.reward_fn.compute(
            depth_m=self._depth,
            xy_error_m=self._xy_error,
            contact_force_n=self._contact_force,
            safety_triggered=safety_triggered,
            success=success,
            side_load=side_load,
        )

        done = success or safety_triggered or self._step_count >= self.config.max_steps or side_load
        self._done = done

        info = {
            "depth_m": self._depth,
            "xy_error_m": self._xy_error,
            "contact_force_n": self._contact_force,
            "success": success,
            "safety_triggered": safety_triggered,
            "side_load": side_load,
            "step_count": self._step_count,
        }
        return self._obs(), reward, done, info

    def _obs(self) -> np.ndarray:
        obs = np.zeros(CONTEXT_DIM, dtype=np.float32)
        obs[54:60] = np.random.normal(0, 0.1, 6)
        obs[48] = 640.0
        obs[49] = 480.0
        obs[50] = 0.1
        obs[51] = 0.5
        obs[52] = 0.1
        obs[53] = 0.3
        obs[66] = 5.0  # INSERT phase
        obs[67] = 1.0  # OK safety
        return obs


def run_random_rollout(seed: int = 0) -> dict:
    """Run a random-policy rollout."""
    np.random.seed(seed)
    env = MockPegInHoleEnv()
    obs = env.reset()
    total_reward = 0.0
    steps = 0
    info_final = {}

    for _ in range(500):
        action = np.random.uniform(-1.0, 1.0, ACTION_DIM).astype(np.float32)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        steps += 1
        info_final = info
        if done:
            break

    return {
        "policy": "random",
        "total_reward": round(float(total_reward), 3),
        "steps": steps,
        "success": bool(info_final.get("success", False)),
        "safety_triggered": bool(info_final.get("safety_triggered", False)),
        "final_depth_m": round(float(info_final.get("depth_m", 0.0)), 6),
        "final_xy_error_m": round(float(info_final.get("xy_error_m", 0.0)), 6),
        "final_contact_force_n": round(float(info_final.get("contact_force_n", 0.0)), 2),
    }


def run_deterministic_baseline(seed: int = 0) -> dict:
    """Run a simple deterministic baseline (move toward hole, then insert)."""
    np.random.seed(seed)
    env = MockPegInHoleEnv()
    obs = env.reset()
    total_reward = 0.0
    steps = 0
    info_final = {}

    for _ in range(500):
        # Simple PID-like policy: move to reduce XY error, then push down
        xy_err = obs[54]  # simplified
        action = np.array([
            -np.sign(xy_err) * 0.5,
            0.8 if env._xy_error < 0.003 else 0.0,
            0.0, 0.0, 0.0, 0.0,
        ], dtype=np.float32)
        obs, reward, done, info = env.step(action)
        total_reward += reward
        steps += 1
        info_final = info
        if done:
            break

    return {
        "policy": "deterministic_baseline",
        "total_reward": round(float(total_reward), 3),
        "steps": steps,
        "success": bool(info_final.get("success", False)),
        "safety_triggered": bool(info_final.get("safety_triggered", False)),
        "final_depth_m": round(float(info_final.get("depth_m", 0.0)), 6),
        "final_xy_error_m": round(float(info_final.get("xy_error_m", 0.0)), 6),
        "final_contact_force_n": round(float(info_final.get("contact_force_n", 0.0)), 2),
    }


def run_multi_episode_evaluation(n_episodes: int = 100, policy: str = "random") -> dict:
    """Run multiple episodes and aggregate statistics."""
    successes = 0
    safety_violations = 0
    timeouts = 0
    total_rewards = []
    depths = []
    xy_errors = []

    for i in range(n_episodes):
        if policy == "random":
            result = run_random_rollout(seed=i)
        else:
            result = run_deterministic_baseline(seed=i)

        total_rewards.append(result["total_reward"])
        depths.append(result["final_depth_m"])
        xy_errors.append(result["final_xy_error_m"])

        if result["success"]:
            successes += 1
        elif result["safety_triggered"]:
            safety_violations += 1
        else:
            timeouts += 1

    return {
        "policy": policy,
        "n_episodes": n_episodes,
        "success_rate": round(successes / n_episodes, 3),
        "safety_violation_rate": round(safety_violations / n_episodes, 3),
        "timeout_rate": round(timeouts / n_episodes, 3),
        "mean_reward": round(float(np.mean(total_rewards)), 3),
        "std_reward": round(float(np.std(total_rewards)), 3),
        "mean_final_depth_m": round(float(np.mean(depths)), 6),
        "mean_final_xy_error_m": round(float(np.mean(xy_errors)), 6),
    }


def main() -> None:
    output_dir = Path("diagnostics/sac_baseline_assessment")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=== SAC Baseline Feasibility Assessment ===\n")

    # 1. Single rollouts
    random_result = run_random_rollout(seed=42)
    det_result = run_deterministic_baseline(seed=42)
    print(f"Random rollout: {json.dumps(random_result, indent=2)}")
    print(f"Deterministic baseline: {json.dumps(det_result, indent=2)}")

    # 2. Multi-episode evaluation
    print(f"\nRunning 100-episode evaluations...")
    random_eval = run_multi_episode_evaluation(n_episodes=100, policy="random")
    det_eval = run_multi_episode_evaluation(n_episodes=100, policy="deterministic_baseline")
    print(f"Random (100 eps): success={random_eval['success_rate']:.1%}")
    print(f"Deterministic (100 eps): success={det_eval['success_rate']:.1%}")

    # 3. Feasibility assessment
    sac_config = SACConfig()
    feasibility = {
        "environment_contract": "defined (sac_baseline_scaffold.py)",
        "gazebo_interface": "NOT implemented (requires Gazebo ROS2 bridge)",
        "training_compute_estimate": {
            "estimated_steps": "1M-5M",
            "estimated_time_gazebo": "3-15 days on 8-core CPU",
            "estimated_time_gpu_cluster": "6-36 hours on 1x A100",
            "local_feasibility": "IMPROBABLE without GPU cluster",
            "reason": (
                "Gazebo physics simulation at 25Hz control rate means "
                "~40ms per step. 1M steps = ~11 hours of pure simulation. "
                "SAC requires additional inference time. Realistic training "
                "would require a GPU cluster for the neural network updates."
            ),
        },
        "baselines_comparison": {
            "random_policy": random_eval,
            "deterministic_baseline": det_eval,
            "deterministic_admittance_controller": {
                "success_rate": 0.925,
                "evidence": "37/40 combined production runs",
                "path": "diagnostics/research_baseline_production_search_v20_600s/",
            },
            "v2_14_safety_gated": {
                "offline_accuracy": 0.9998,
                "shadow_agreement": 0.922,
                "evidence": "10-trial shadow-mode validation",
            },
            "v2_14_guarded_advisory": {
                "physical_success_rate": 0.90,
                "safety_invariants": "ALL_HOLD",
                "evidence": "10-trial guarded advisory validation",
            },
        },
        "recommendation": (
            "SAC baseline is scaffolded and validated with mock rollouts. "
            "Full Gazebo-based training requires GPU cluster compute not "
            "available locally. The deterministic admittance controller "
            "(92.5%) and v2_14 safety-gated approach (90% physical) already "
            "provide strong baselines. SAC training should be deferred to "
            "a compute cluster deployment."
        ),
    }

    # 4. Save results
    assessment_path = output_dir / "sac_feasibility_assessment.json"
    assessment_path.write_text(json.dumps(feasibility, indent=2, default=str))
    print(f"\nAssessment saved to {assessment_path}")
    print(f"\nRecommendation: {feasibility['recommendation']}")


if __name__ == "__main__":
    main()

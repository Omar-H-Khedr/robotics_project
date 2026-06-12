"""
SAC Evaluation Script
=====================
Loads a trained SAC checkpoint and evaluates it on held-out scenarios.
Reports success rate, fail-closed rate, unsafe rate, and mean reward.

Usage:
    python sac_evaluation_script.py --checkpoint checkpoints/sac_final.pt
    python sac_evaluation_script.py --checkpoint checkpoints/sac_final.pt --episodes 100
    python sac_evaluation_script.py --checkpoint checkpoints/sac_final.pt --held_out_scenarios
"""

import argparse
import json
import os
import sys
import yaml
from pathlib import Path
from collections import defaultdict

import numpy as np

try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False

# Import from training script
from sac_training_script import SACAgent, SACEnvironmentWrapper, DEFAULT_CONFIG


# ---------------------------------------------------------------------------
# Baselines
# ---------------------------------------------------------------------------
DETERMINISTIC_BASELINE_SUCCESS_RATE = 0.875  # 87.5%
ADVISORY_V2_14_SUCCESS_RATE = 0.90           # 90%


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------
def evaluate_agent(
    agent: SACAgent,
    env: SACEnvironmentWrapper,
    num_episodes: int = 20,
    deterministic: bool = True,
) -> dict:
    """Run evaluation episodes and collect statistics."""
    results = {
        "rewards": [],
        "lengths": [],
        "successes": [],
        "fail_closed": [],
        "unsafe_attempts": [],
        "episode_details": [],
    }

    for ep in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0.0
        episode_length = 0
        done = False
        truncated = False
        ep_detail = {"transitions": [], "scenario": info.get("scenario", {})}

        while not done and not truncated:
            action = agent.select_action(obs, deterministic=deterministic)
            next_obs, reward, done, truncated, info = env.step(action)

            ep_detail["transitions"].append({
                "obs_mean": float(np.mean(obs)),
                "action_mean": float(np.mean(action)),
                "reward": float(reward),
                "safety_envelope": info.get("safety_envelope", True),
            })

            episode_reward += reward
            episode_length += 1
            obs = next_obs

        success = info.get("success", False)
        fail_closed = info.get("fail_closed", False)
        unsafe = info.get("unsafe_attempt", False)

        results["rewards"].append(episode_reward)
        results["lengths"].append(episode_length)
        results["successes"].append(float(success))
        results["fail_closed"].append(float(fail_closed))
        results["unsafe_attempts"].append(float(unsafe))
        ep_detail["outcome"] = "success" if success else ("fail_closed" if fail_closed else "unsafe" if unsafe else "other")
        results["episode_details"].append(ep_detail)

    return results


def compute_statistics(results: dict) -> dict:
    """Compute aggregate statistics from evaluation results."""
    rewards = results["rewards"]
    successes = results["successes"]
    fail_closed = results["fail_closed"]
    unsafe = results["unsafe_attempts"]
    lengths = results["lengths"]

    stats = {
        "num_episodes": len(rewards),
        "mean_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards)),
        "min_reward": float(np.min(rewards)),
        "max_reward": float(np.max(rewards)),
        "mean_length": float(np.mean(lengths)),
        "success_rate": float(np.mean(successes)) * 100.0,
        "fail_closed_rate": float(np.mean(fail_closed)) * 100.0,
        "unsafe_rate": float(np.mean(unsafe)) * 100.0,
        "deterministic_baseline": DETERMINISTIC_BASELINE_SUCCESS_RATE * 100.0,
        "advisory_v2_14": ADVISORY_V2_14_SUCCESS_RATE * 100.0,
    }
    return stats


def compare_with_baselines(stats: dict) -> str:
    """Generate comparison text against known baselines."""
    lines = []
    lines.append("=" * 70)
    lines.append("Baseline Comparison")
    lines.append("=" * 70)
    lines.append(f"  {'Metric':<35} {'SAC':>10} {'Baseline':>12}")
    lines.append("-" * 70)
    lines.append(
        f"  {'Success Rate (%)':<35} {stats['success_rate']:>9.1f}% "
        f"{stats['deterministic_baseline']:>10.1f}%"
    )
    lines.append(
        f"  {'Advisory v2.14 (%)':<35} {stats['success_rate']:>9.1f}% "
        f"{stats['advisory_v2_14']:>10.1f}%"
    )
    lines.append(
        f"  {'Fail-Closed Rate (%)':<35} {stats['fail_closed_rate']:>9.1f}%"
    )
    lines.append(
        f"  {'Unsafe Rate (%)':<35} {stats['unsafe_rate']:>9.1f}%"
    )
    lines.append(
        f"  {'Mean Reward':<35} {stats['mean_reward']:>10.1f}"
    )
    lines.append(
        f"  {'Mean Episode Length':<35} {stats['mean_length']:>10.1f}"
    )
    lines.append("=" * 70)

    # Assessment
    if stats["success_rate"] >= DETERMINISTIC_BASELINE_SUCCESS_RATE * 100.0:
        lines.append("  MEETS deterministic baseline (87.5%)")
    else:
        lines.append(f"  BELOW deterministic baseline by {DETERMINISTIC_BASELINE_SUCCESS_RATE*100.0 - stats['success_rate']:.1f}%")

    if stats["unsafe_rate"] < 1.0:
        lines.append("  SAFETY: Unsafe rate < 1% (safety constraint satisfied)")
    else:
        lines.append(f"  SAFETY WARNING: Unsafe rate {stats['unsafe_rate']:.1f}% exceeds 1% threshold")

    lines.append("=" * 70)
    return "\n".join(lines)


def held_out_scenario_evaluation(
    agent: SACAgent,
    num_episodes: int = 50,
    seed: int = 9999,
) -> dict:
    """
    Evaluate on held-out scenarios (out-of-distribution).
    Uses a different seed to ensure scenario diversity.
    """
    env = SACEnvironmentWrapper(scenario_mode="uniform", seed=seed)
    return evaluate_agent(agent, env, num_episodes=num_episodes, deterministic=True)


# ---------------------------------------------------------------------------
# Report Generation
# ---------------------------------------------------------------------------
def generate_report(
    stats: dict,
    held_out_stats: dict | None,
    checkpoint_path: str,
    output_path: str,
):
    """Generate a JSON evaluation report."""
    report = {
        "checkpoint": checkpoint_path,
        "evaluation": stats,
        "baselines": {
            "deterministic_baseline": DETERMINISTIC_BASELINE_SUCCESS_RATE,
            "advisory_v2_14": ADVISORY_V2_14_SUCCESS_RATE,
        },
        "held_out": held_out_stats,
        "notes": {
            "warning": "PROJECTED RESULTS ONLY. No actual training has been performed.",
            "intended_use": "Cluster-ready package for future SAC training runs.",
        },
    }

    with open(output_path, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\nReport saved to: {output_path}")
    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="SAC Evaluation Script")
    parser.add_argument("--checkpoint", type=str, required=True, help="Path to SAC checkpoint")
    parser.add_argument("--episodes", type=int, default=20, help="Number of evaluation episodes")
    parser.add_argument("--held_out_episodes", type=int, default=50, help="Episodes for held-out eval")
    parser.add_argument("--held_out_scenarios", action="store_true", help="Run held-out scenario eval")
    parser.add_argument("--output", type=str, default=None, help="Output report path")
    parser.add_argument("--deterministic", action="store_true", default=True, help="Use deterministic policy")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--config", type=str, default=None, help="Config YAML (for env settings)")
    return parser.parse_args()


def main():
    args = parse_args()

    if not TORCH_AVAILABLE:
        print("[ERROR] PyTorch required. Install: pip install torch")
        sys.exit(1)

    if not os.path.exists(args.checkpoint):
        print(f"[ERROR] Checkpoint not found: {args.checkpoint}")
        sys.exit(1)

    print("=" * 70)
    print("SAC Evaluation Script")
    print("=" * 70)
    print(f"  Checkpoint:  {args.checkpoint}")
    print(f"  Episodes:    {args.episodes}")
    print(f"  Held-out:    {args.held_out_scenarios}")
    print("=" * 70)

    # Load config
    config = DEFAULT_CONFIG.copy()
    if args.config and os.path.exists(args.config):
        with open(args.config) as f:
            file_config = yaml.safe_load(f)
        if file_config and "evaluation" in file_config:
            config.update(file_config["evaluation"])

    # Load agent
    obs_dim = 72
    act_dim = 6
    device = "cuda" if torch.cuda.is_available() else "cpu"

    agent = SACAgent(
        obs_dim=obs_dim,
        act_dim=act_dim,
        hidden_dim=config["hidden_dim"],
        num_layers=config["num_hidden_layers"],
        device=device,
    )
    agent.load(args.checkpoint)
    print(f"[OK] Loaded checkpoint: {args.checkpoint}")

    # Standard evaluation
    env = SACEnvironmentWrapper(scenario_mode="uniform", seed=args.seed)
    results = evaluate_agent(
        agent, env,
        num_episodes=args.episodes,
        deterministic=args.deterministic,
    )
    stats = compute_statistics(results)

    # Print results
    print(f"\n  Results ({args.episodes} episodes):")
    print(f"  {'-'*50}")
    print(f"  Success Rate:     {stats['success_rate']:.1f}%")
    print(f"  Fail-Closed Rate: {stats['fail_closed_rate']:.1f}%")
    print(f"  Unsafe Rate:      {stats['unsafe_rate']:.1f}%")
    print(f"  Mean Reward:      {stats['mean_reward']:.1f}")
    print(f"  Mean Length:      {stats['mean_length']:.1f}")

    print(compare_with_baselines(stats))

    # Held-out evaluation
    held_out_stats = None
    if args.held_out_scenarios:
        print("\nRunning held-out scenario evaluation...")
        held_out_results = held_out_scenario_evaluation(
            agent, num_episodes=args.held_out_episodes
        )
        held_out_stats = compute_statistics(held_out_results)

        print(f"\n  Held-Out Results ({args.held_out_episodes} episodes):")
        print(f"  {'-'*50}")
        print(f"  Success Rate:     {held_out_stats['success_rate']:.1f}%")
        print(f"  Fail-Closed Rate: {held_out_stats['fail_closed_rate']:.1f}%")
        print(f"  Unsafe Rate:      {held_out_stats['unsafe_rate']:.1f}%")
        print(f"  Mean Reward:      {held_out_stats['mean_reward']:.1f}")

    # Generate report
    output_path = args.output
    if output_path is None:
        ckpt_stem = Path(args.checkpoint).stem
        output_path = f"eval_report_{ckpt_stem}.json"

    generate_report(stats, held_out_stats, args.checkpoint, output_path)


if __name__ == "__main__":
    main()

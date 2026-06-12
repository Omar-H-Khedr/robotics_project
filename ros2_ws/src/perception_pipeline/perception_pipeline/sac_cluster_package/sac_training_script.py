"""
SAC Training Script for Cluster Deployment
==========================================
Trains a Soft Actor-Critic agent on the scenario-randomized insertion environment.
Designed for SLURM cluster execution with checkpointing and tensorboard logging.

Usage:
    python sac_training_script.py --config config/cluster_training_config.yaml
    python sac_training_script.py --total_steps 1000000 --scenario_mode uniform
"""

import argparse
import json
import os
import sys
import time
import yaml
from pathlib import Path
from collections import deque
import random

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    import torch.nn.functional as F
    from torch.utils.tensorboard import SummaryWriter
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("[WARNING] PyTorch not available. Install: pip install torch")


# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------
DEFAULT_CONFIG = {
    "total_steps": 1_000_000,
    "warmup_steps": 1000,
    "batch_size": 256,
    "lr": 3e-4,
    "gamma": 0.99,
    "tau": 0.005,
    "alpha": 0.2,
    "auto_alpha": True,
    "target_entropy": -72,
    "hidden_dim": 256,
    "num_hidden_layers": 2,
    "replay_buffer_size": 100_000,
    "checkpoint_interval": 10_000,
    "log_interval": 1_000,
    "eval_interval": 50_000,
    "eval_episodes": 20,
    "scenario_mode": "uniform",
    "seed": 42,
    "device": "cuda",
    "checkpoint_dir": "checkpoints",
    "log_dir": "logs",
}


# ---------------------------------------------------------------------------
# Simple Replay Buffer
# ---------------------------------------------------------------------------
class ReplayBuffer:
    """Fixed-size ring buffer for off-policy learning."""

    def __init__(self, capacity: int, obs_dim: int, act_dim: int, device: str = "cpu"):
        self.capacity = capacity
        self.device = device
        self.ptr = 0
        self.size = 0

        self.obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.act = np.zeros((capacity, act_dim), dtype=np.float32)
        self.rew = np.zeros(capacity, dtype=np.float32)
        self.next_obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.done = np.zeros(capacity, dtype=np.float32)

    def add(self, obs, act, rew, next_obs, done):
        self.obs[self.ptr] = obs
        self.act[self.ptr] = act
        self.rew[self.ptr] = rew
        self.next_obs[self.ptr] = next_obs
        self.done[self.ptr] = done
        self.ptr = (self.ptr + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int):
        idx = np.random.randint(0, self.size, size=batch_size)
        return (
            torch.FloatTensor(self.obs[idx]).to(self.device),
            torch.FloatTensor(self.act[idx]).to(self.device),
            torch.FloatTensor(self.rew[idx]).to(self.device),
            torch.FloatTensor(self.next_obs[idx]).to(self.device),
            torch.FloatTensor(self.done[idx]).to(self.device),
        )


# ---------------------------------------------------------------------------
# SAC Networks
# ---------------------------------------------------------------------------
LOG_SIG_MAX = 2
LOG_SIG_MIN = -20
EPSILON = 1e-6


def fanin_init(layer):
    """Fan-in initialization (common for ReLU networks)."""
    bias = 1.0 / np.sqrt(layer.weight.shape[1])
    nn.init.uniform_(layer.weight, -bias, bias)
    nn.init.constant_(layer.bias, 0.0)


class Actor(nn.Module):
    """Gaussian policy with tanh squashing."""

    def __init__(self, obs_dim: int, act_dim: int, hidden_dim: int = 256, num_layers: int = 2):
        super().__init__()
        layers = []
        in_dim = obs_dim
        for _ in range(num_layers):
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.ReLU())
            in_dim = hidden_dim
        self.backbone = nn.Sequential(*layers)
        self.mean_head = nn.Linear(hidden_dim, act_dim)
        self.log_std_head = nn.Linear(hidden_dim, act_dim)

        # Initialize output layers with small weights
        fanin_init(self.mean_head)
        fanin_init(self.log_std_head)

    def forward(self, obs):
        h = self.backbone(obs)
        mean = self.mean_head(h)
        log_std = self.log_std_head(h)
        log_std = torch.clamp(log_std, LOG_SIG_MIN, LOG_SIG_MAX)
        return mean, log_std

    def sample(self, obs):
        mean, log_std = self.forward(obs)
        std = log_std.exp()
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        action = torch.tanh(x_t)

        # Log-probability with tanh squashing correction
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1 - action.pow(2) + EPSILON)
        log_prob = log_prob.sum(dim=-1, keepdim=True)

        return action, log_prob, mean


class Critic(nn.Module):
    """Q-network: takes (obs, action) -> Q-value."""

    def __init__(self, obs_dim: int, act_dim: int, hidden_dim: int = 256, num_layers: int = 2):
        super().__init__()
        layers = []
        in_dim = obs_dim + act_dim
        for _ in range(num_layers):
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.ReLU())
            in_dim = hidden_dim
        layers.append(nn.Linear(hidden_dim, 1))
        self.net = nn.Sequential(*layers)

    def forward(self, obs, action):
        x = torch.cat([obs, action], dim=-1)
        return self.net(x)


# ---------------------------------------------------------------------------
# SAC Agent
# ---------------------------------------------------------------------------
class SACAgent:
    """Soft Actor-Critic agent with auto-tuned alpha."""

    def __init__(
        self,
        obs_dim: int,
        act_dim: int,
        hidden_dim: int = 256,
        num_layers: int = 2,
        lr: float = 3e-4,
        gamma: float = 0.99,
        tau: float = 0.005,
        alpha: float = 0.2,
        auto_alpha: bool = True,
        target_entropy: float = -72.0,
        device: str = "cuda",
    ):
        self.gamma = gamma
        self.tau = tau
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.auto_alpha = auto_alpha

        # Actor
        self.actor = Actor(obs_dim, act_dim, hidden_dim, num_layers).to(self.device)
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)

        # Twin critics
        self.critic1 = Critic(obs_dim, act_dim, hidden_dim, num_layers).to(self.device)
        self.critic2 = Critic(obs_dim, act_dim, hidden_dim, num_layers).to(self.device)
        self.critic1_target = Critic(obs_dim, act_dim, hidden_dim, num_layers).to(self.device)
        self.critic2_target = Critic(obs_dim, act_dim, hidden_dim, num_layers).to(self.device)
        self.critic1_target.load_state_dict(self.critic1.state_dict())
        self.critic2_target.load_state_dict(self.critic2.state_dict())

        self.critic1_optimizer = optim.Adam(self.critic1.parameters(), lr=lr)
        self.critic2_optimizer = optim.Adam(self.critic2.parameters(), lr=lr)

        # Log-alpha
        if auto_alpha:
            self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr)
        else:
            self.log_alpha = torch.tensor([np.log(alpha)], device=self.device)

        self.target_entropy = target_entropy

    @property
    def alpha(self):
        return self.log_alpha.exp().item()

    def select_action(self, obs: np.ndarray, deterministic: bool = False) -> np.ndarray:
        """Select action for a single observation."""
        obs_t = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
        if deterministic:
            _, _, mean = self.actor.sample(obs_t)
            action = torch.tanh(mean)
        else:
            action, _, _ = self.actor.sample(obs_t)
        return action.detach().cpu().numpy().flatten()

    def update(self, batch):
        obs, act, rew, next_obs, done = batch

        # --- Critic update ---
        with torch.no_grad():
            next_action, next_log_prob, _ = self.actor.sample(next_obs)
            q1_next = self.critic1_target(next_obs, next_action)
            q2_next = self.critic2_target(next_obs, next_action)
            q_next = torch.min(q1_next, q2_next) - self.alpha * next_log_prob
            q_target = rew + self.gamma * (1.0 - done) * q_next

        q1 = self.critic1(obs, act)
        q2 = self.critic2(obs, act)
        critic1_loss = F.mse_loss(q1, q_target)
        critic2_loss = F.mse_loss(q2, q_target)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        # --- Actor update ---
        new_action, log_prob, _ = self.actor.sample(obs)
        q1_new = self.critic1(obs, new_action)
        q2_new = self.critic2(obs, new_action)
        q_new = torch.min(q1_new, q2_new)

        actor_loss = (self.alpha * log_prob - q_new).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # --- Alpha update ---
        alpha_loss = 0.0
        if self.auto_alpha:
            alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            alpha_loss = alpha_loss.item()

        # --- Soft target update ---
        self._soft_update(self.critic1, self.critic1_target)
        self._soft_update(self.critic2, self.critic2_target)

        return {
            "critic1_loss": critic1_loss.item(),
            "critic2_loss": critic2_loss.item(),
            "actor_loss": actor_loss.item(),
            "alpha": self.alpha,
            "alpha_loss": alpha_loss,
            "q_mean": q_new.mean().item(),
        }

    def _soft_update(self, source, target):
        for sp, tp in zip(source.parameters(), target.parameters()):
            tp.data.copy_(self.tau * sp.data + (1.0 - self.tau) * tp.data)

    def save(self, path: str):
        torch.save({
            "actor": self.actor.state_dict(),
            "critic1": self.critic1.state_dict(),
            "critic2": self.critic2.state_dict(),
            "critic1_target": self.critic1_target.state_dict(),
            "critic2_target": self.critic2_target.state_dict(),
            "actor_optimizer": self.actor_optimizer.state_dict(),
            "critic1_optimizer": self.critic1_optimizer.state_dict(),
            "critic2_optimizer": self.critic2_optimizer.state_dict(),
            "log_alpha": self.log_alpha,
        }, path)

    def load(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.actor.load_state_dict(ckpt["actor"])
        self.critic1.load_state_dict(ckpt["critic1"])
        self.critic2.load_state_dict(ckpt["critic2"])
        self.critic1_target.load_state_dict(ckpt["critic1_target"])
        self.critic2_target.load_state_dict(ckpt["critic2_target"])
        self.actor_optimizer.load_state_dict(ckpt["actor_optimizer"])
        self.critic1_optimizer.load_state_dict(ckpt["critic1_optimizer"])
        self.critic2_optimizer.load_state_dict(ckpt["critic2_optimizer"])
        self.log_alpha = ckpt["log_alpha"]


# ---------------------------------------------------------------------------
# Environment Wrapper (lazy import)
# ---------------------------------------------------------------------------
class SACEnvironmentWrapper:
    """
    Wraps the scenario-randomized SAC environment.
    Lazy-imports to allow config inspection without full env.
    """

    def __init__(self, scenario_mode: str = "uniform", seed: int = 42):
        self.scenario_mode = scenario_mode
        self.seed = seed
        self.env = None

    def _lazy_import(self):
        if self.env is not None:
            return
        try:
            sys.path.insert(
                0,
                str(Path(__file__).resolve().parent.parent),
            )
            from sac_scenario_randomization import (
                ScenarioRandomizer,
                SafetyAwareReward,
                ExtendedObservationSpace,
            )
            self.env_cls = ScenarioRandomizer
            self.reward_cls = SafetyAwareReward
            self.obs_cls = ExtendedObservationSpace
            print("[ENV] Successfully imported scenario randomization module")
        except ImportError as e:
            print(f"[ENV] ERROR: Cannot import sac_scenario_randomization: {e}")
            print("[ENV] Falling back to mock environment for smoke testing")
            self.env_cls = None

    def reset(self):
        self._lazy_import()
        if self.env_cls is not None:
            # Real environment
            randomizer = self.env_cls(mode=self.scenario_mode, seed=self.seed)
            scenario = randomizer.sample()
            obs, info = self.env_cls.reset(scenario)
            return obs, info
        else:
            # Mock environment for testing
            obs = np.random.randn(72).astype(np.float32)
            info = {"scenario": {}, "safety_envelope": True}
            return obs, info

    def step(self, action):
        if self.env_cls is not None:
            return self.env_cls.step(action)
        else:
            # Mock step
            obs = np.random.randn(72).astype(np.float32)
            reward = float(np.random.randn())
            done = bool(np.random.random() < 0.05)
            truncated = False
            info = {"safety_envelope": True, "success": False}
            return obs, reward, done, truncated, info


# ---------------------------------------------------------------------------
# Training Loop
# ---------------------------------------------------------------------------
def train(config: dict):
    """Main training loop."""
    if not TORCH_AVAILABLE:
        print("[ERROR] PyTorch required. Install: pip install torch")
        sys.exit(1)

    # Setup directories
    ckpt_dir = Path(config["checkpoint_dir"])
    log_dir = Path(config["log_dir"])
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)

    # Seed
    seed = config.get("seed", 42)
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)

    # Environment
    obs_dim = 72  # ExtendedObservationSpace
    act_dim = 6   # 6-DOF insertion action

    env = SACEnvironmentWrapper(
        scenario_mode=config["scenario_mode"],
        seed=seed,
    )

    # Agent
    agent = SACAgent(
        obs_dim=obs_dim,
        act_dim=act_dim,
        hidden_dim=config["hidden_dim"],
        num_layers=config["num_hidden_layers"],
        lr=config["lr"],
        gamma=config["gamma"],
        tau=config["tau"],
        alpha=config["alpha"],
        auto_alpha=config["auto_alpha"],
        target_entropy=config.get("target_entropy", -obs_dim),
        device=config["device"],
    )

    # Replay buffer
    buffer = ReplayBuffer(
        capacity=config["replay_buffer_size"],
        obs_dim=obs_dim,
        act_dim=act_dim,
        device="cpu",
    )

    # Tensorboard
    writer = SummaryWriter(log_dir=str(log_dir))

    # Training state
    total_steps = config["total_steps"]
    warmup_steps = config["warmup_steps"]
    batch_size = config["batch_size"]
    update_every = 1
    updates_done = 0

    print("=" * 70)
    print("SAC Training Configuration")
    print("=" * 70)
    print(f"  Total steps:       {total_steps:>12,}")
    print(f"  Warmup steps:      {warmup_steps:>12,}")
    print(f"  Batch size:        {batch_size:>12}")
    print(f"  Learning rate:     {config['lr']}")
    print(f"  Gamma:             {config['gamma']}")
    print(f"  Tau:               {config['tau']}")
    print(f"  Alpha:             {config['alpha']}")
    print(f"  Auto alpha:        {config['auto_alpha']}")
    print(f"  Replay buffer:     {config['replay_buffer_size']:>12,}")
    print(f"  Scenario mode:     {config['scenario_mode']}")
    print(f"  Checkpoint every:  {config['checkpoint_interval']:>12,} steps")
    print(f"  Device:            {config['device']}")
    print(f"  Seed:              {seed}")
    print("=" * 70)

    obs, info = env.reset()
    episode_reward = 0.0
    episode_length = 0
    episode_count = 0
    episode_rewards = deque(maxlen=100)
    episode_lengths = deque(maxlen=100)
    episode_successes = deque(maxlen=100)
    start_time = time.time()

    for step in range(1, total_steps + 1):
        # Select action
        if step <= warmup_steps:
            action = np.random.uniform(-1, 1, size=act_dim).astype(np.float32)
        else:
            action = agent.select_action(obs)

        # Step environment
        next_obs, reward, done, truncated, info = env.step(action)
        episode_reward += reward
        episode_length += 1

        # Store transition
        buffer.add(obs, action, reward, next_obs, float(done or truncated))

        obs = next_obs

        # Episode boundary
        if done or truncated:
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            success = info.get("success", False)
            episode_successes.append(float(success))

            writer.add_scalar("episode/reward", episode_reward, step)
            writer.add_scalar("episode/length", episode_length, step)
            writer.add_scalar("episode/success", float(success), step)

            if len(episode_rewards) > 0:
                writer.add_scalar("episode/mean_reward_100", np.mean(episode_rewards), step)
                writer.add_scalar("episode/mean_length_100", np.mean(episode_lengths), step)
                writer.add_scalar("episode/success_rate_100", np.mean(episode_successes), step)

            if episode_count % 10 == 0:
                elapsed = time.time() - start_time
                fps = step / max(elapsed, 1.0)
                print(
                    f"  Step {step:>10,}/{total_steps:,} | "
                    f"Ep {episode_count:>4} | "
                    f"Rew {episode_reward:>7.1f} | "
                    f"Len {episode_length:>4} | "
                    f"Success {success} | "
                    f"FPS {fps:.1f}"
                )

            episode_reward = 0.0
            episode_length = 0
            episode_count += 1
            obs, info = env.reset()

        # Update policy
        if step > warmup_steps and buffer.size >= batch_size:
            for _ in range(update_every):
                batch = buffer.sample(batch_size)
                update_info = agent.update(batch)
                updates_done += 1

                if updates_done % config["log_interval"] == 0:
                    for k, v in update_info.items():
                        writer.add_scalar(f"train/{k}", v, step)

        # Checkpoint
        if step % config["checkpoint_interval"] == 0:
            ckpt_path = ckpt_dir / f"sac_step_{step:08d}.pt"
            agent.save(str(ckpt_path))
            print(f"  [CHECKPOINT] Saved to {ckpt_path}")

            # Save training state
            state = {
                "step": step,
                "episode_count": episode_count,
                "seed": seed,
                "config": config,
            }
            with open(ckpt_dir / "training_state.json", "w") as f:
                json.dump(state, f, indent=2)

    # Final checkpoint
    agent.save(str(ckpt_dir / "sac_final.pt"))
    writer.close()

    elapsed = time.time() - start_time
    print("=" * 70)
    print("Training Complete")
    print("=" * 70)
    print(f"  Total steps:    {total_steps:>12,}")
    print(f"  Total episodes: {episode_count:>12}")
    print(f"  Wall time:      {elapsed:>12.1f}s ({elapsed/3600:.1f}h)")
    print(f"  FPS:            {total_steps/max(elapsed,1):>12.1f}")
    if episode_rewards:
        print(f"  Mean reward:    {np.mean(episode_rewards):>12.1f}")
        print(f"  Success rate:   {np.mean(episode_successes)*100:>11.1f}%")
    print("=" * 70)

    return agent


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(description="SAC Training Script")
    parser.add_argument("--config", type=str, default=None, help="Path to YAML config")
    parser.add_argument("--total_steps", type=int, default=None)
    parser.add_argument("--scenario_mode", type=str, default=None, choices=["uniform", "weighted", "fixed"])
    parser.add_argument("--checkpoint_dir", type=str, default=None)
    parser.add_argument("--log_dir", type=str, default=None)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--device", type=str, default=None)
    return parser.parse_args()


def main():
    args = parse_args()

    # Load config
    config = DEFAULT_CONFIG.copy()
    if args.config and os.path.exists(args.config):
        with open(args.config) as f:
            file_config = yaml.safe_load(f)
        if file_config and "training" in file_config:
            config.update(file_config["training"])
        elif file_config:
            config.update(file_config)

    # Override with CLI args
    cli_overrides = {
        "total_steps": args.total_steps,
        "scenario_mode": args.scenario_mode,
        "checkpoint_dir": args.checkpoint_dir,
        "log_dir": args.log_dir,
        "seed": args.seed,
        "device": args.device,
    }
    for k, v in cli_overrides.items():
        if v is not None:
            config[k] = v

    train(config)


if __name__ == "__main__":
    main()

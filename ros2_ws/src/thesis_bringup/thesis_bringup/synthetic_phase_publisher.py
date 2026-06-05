"""Synthetic /task_phase publisher for offline multi-phase dataset generation.

The 1mm/2mm cartesian precision ceiling with the working JTC means
SEARCH / INSERT / ABORT labeled trials are not reachable in the live
controller stack. To still produce a multi-phase labeled dataset for
v2_14 (context-conditioned action) and v2_15 (ablation) training, this
node publishes /task_phase on a scripted schedule.

It is intended to be enabled in research_baseline.launch.py with
`enable_synthetic_phases:=true` and combined with the existing
multimodal_observation_logger (`enable_perception_logging:=true`).
The resulting CSV records real D405 RGB-D, real joint states, real
force/torque, and synthetic phase labels per tick. The dataset is
explicitly a synthetic-phase proxy for offline training; it is not
a real motor-actuated SEARCH/INSERT trial.

Two schedule modes are supported:
  1. Built-in: a single short schedule that cycles through all
     phases (used when no `schedule_path` is provided).
  2. Custom: a YAML file with a list of {phase, duration_s} entries
     (used when `schedule_path` is provided). The YAML must contain
     a top-level `schedule:` key with a list of dicts.

The node is a passive publisher; it does not subscribe to any topic.
It uses sim time when `use_sim_time:=true` so its clock matches the
trial clock. It auto-stops after the schedule ends (the timer is
cancelled) so the trial can finish cleanly.

This is offline training infrastructure, not a control input to the
admittance node. The admittance_insertion_node should be DISABLED
when this node is enabled (the launch file controls this with
`enable_admittance:=false` in the synthetic mode).
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import List, Tuple

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_BUILTIN_SCHEDULE: List[Tuple[str, float]] = [
    ("MOVE_TO_START", 30.0),
    ("APPROACH", 15.0),
    ("SEARCH", 30.0),
    ("INSERT", 20.0),
    ("INSERTED", 5.0),
    ("ABORT", 5.0),
]


def _load_schedule(path: str) -> List[Tuple[str, float]]:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"schedule_path does not exist: {path}")
    with open(p) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "schedule" not in data:
        raise ValueError(
            f"schedule_path must be a YAML file with a top-level 'schedule:' key, got: {path}"
        )
    out: List[Tuple[str, float]] = []
    for entry in data["schedule"]:
        if not isinstance(entry, dict) or "phase" not in entry or "duration_s" not in entry:
            raise ValueError(
                f"each schedule entry must be a dict with 'phase' and 'duration_s', got: {entry}"
            )
        out.append((str(entry["phase"]), float(entry["duration_s"])))
    if not out:
        raise ValueError(f"schedule is empty in {path}")
    return out


class SyntheticPhasePublisher(Node):
    """Publish /task_phase on a scripted schedule, optionally loaded from YAML."""

    def __init__(self) -> None:
        super().__init__("synthetic_phase_publisher")
        self.declare_parameter("schedule_path", "")
        try:
            self._use_sim_time = bool(
                self.get_parameter("use_sim_time").get_parameter_value().bool_value
            )
        except Exception:
            self.declare_parameter("use_sim_time", True)
            self._use_sim_time = bool(
                self.get_parameter("use_sim_time").get_parameter_value().bool_value
            )
        self.declare_parameter("start_offset_s", 0.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("topic", "/task_phase")
        self.declare_parameter("autostop", True)

        schedule_path = (
            self.get_parameter("schedule_path").get_parameter_value().string_value
        )
        if schedule_path:
            self._schedule = _load_schedule(schedule_path)
            self.get_logger().info(
                f"loaded {len(self._schedule)}-step schedule from {schedule_path}"
            )
        else:
            self._schedule = DEFAULT_BUILTIN_SCHEDULE
            self.get_logger().info(
                f"using default {len(self._schedule)}-step builtin schedule"
            )

        self._start_offset_s = float(
            self.get_parameter("start_offset_s").get_parameter_value().double_value
        )
        self._publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        )
        self._loop = bool(
            self.get_parameter("loop").get_parameter_value().bool_value
        )
        self._topic = str(
            self.get_parameter("topic").get_parameter_value().string_value
        )
        self._autostop = bool(
            self.get_parameter("autostop").get_parameter_value().bool_value
        )

        self._pub = self.create_publisher(String, self._topic, 10)

        self._total_duration_s = sum(d for _, d in self._schedule)
        self._start_wall_time_s: float | None = None
        self._start_sim_time_s: float | None = None
        self._current_phase: str = ""
        self._current_index: int = -1
        self._done = False

        period_s = 1.0 / max(self._publish_rate_hz, 0.1)
        self._timer = self.create_timer(period_s, self._tick)
        self.get_logger().info(
            f"schedule: {self._schedule} total {self._total_duration_s:.1f}s "
            f"publish={self._publish_rate_hz:.1f}Hz on {self._topic} loop={self._loop} autostop={self._autostop}"
        )

    def _now_s(self) -> float:
        if self._use_sim_time:
            now_msg = self.get_clock().now()
            return now_msg.nanoseconds * 1e-9
        return time.time()

    def _tick(self) -> None:
        if self._done:
            return
        if self._start_sim_time_s is None:
            self._start_sim_time_s = self._now_s()
            self._start_wall_time_s = time.time()
        elapsed = self._now_s() - self._start_sim_time_s - self._start_offset_s
        if elapsed < 0.0:
            phase = self._schedule[0][0]
        else:
            acc = 0.0
            phase: str | None = None
            for i, (p, d) in enumerate(self._schedule):
                if elapsed < acc + d:
                    phase = p
                    self._current_index = i
                    break
                acc += d
            if phase is None:
                if self._loop:
                    cycle = self._total_duration_s
                    elapsed_mod = elapsed - math.floor(elapsed / cycle) * cycle
                    acc = 0.0
                    for i, (p, d) in enumerate(self._schedule):
                        if elapsed_mod < acc + d:
                            phase = p
                            self._current_index = i
                            break
                        acc += d
                else:
                    phase = self._schedule[-1][0]
                    self._current_index = len(self._schedule) - 1
                    if self._autostop:
                        self.get_logger().info(
                            f"schedule complete after {elapsed:.2f}s "
                            f"(total {self._total_duration_s:.2f}s); stopping"
                        )
                        self._done = True
                        self._timer.cancel()

        if phase != self._current_phase:
            self.get_logger().info(
                f"phase change: {self._current_phase!r} -> {phase!r} "
                f"at t={elapsed:.2f}s"
            )
            self._current_phase = phase
        msg = String()
        msg.data = phase
        self._pub.publish(msg)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SyntheticPhasePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

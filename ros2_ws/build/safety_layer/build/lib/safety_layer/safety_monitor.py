"""Monitor-only safety layer for the KUKA peg-in-hole research baseline."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class SafetyMonitor(Node):
    """Publish safety status from joint-state validity, soft limits, and timing."""

    TERMINAL_TRIAL_STATUSES = {
        "completed",
        "failed",
        "guarded_stop",
        "guarded_contact_stop",
    }

    JOINT_NAMES = (
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    )

    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("config_path", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("trial_status_topic", "/trial_status")
        self.declare_parameter("safety_status_topic", "/safety_status")
        self.declare_parameter("monitor_period_sec", 0.2)
        self.declare_parameter("warning_throttle_sec", 2.0)

        self._config_path = Path(
            self.get_parameter("config_path").get_parameter_value().string_value
        )
        self._joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self._task_phase_topic = (
            self.get_parameter("task_phase_topic").get_parameter_value().string_value
        )
        self._trial_status_topic = (
            self.get_parameter("trial_status_topic").get_parameter_value().string_value
        )
        self._safety_status_topic = (
            self.get_parameter("safety_status_topic").get_parameter_value().string_value
        )
        self._monitor_period_sec = (
            self.get_parameter("monitor_period_sec").get_parameter_value().double_value
        )
        self._warning_throttle_sec = (
            self.get_parameter("warning_throttle_sec").get_parameter_value().double_value
        )

        config = self._load_config(self._config_path)
        self._safety_enabled = bool(config["safety_enabled"])
        self._joint_limits = config["joint_soft_limits"]
        self._joint_state_timeout_sec = float(config["joint_state_timeout_sec"])
        self._max_phase_duration_sec = float(config["max_phase_duration_sec"])

        self._last_joint_state_time = None
        self._last_phase_time = None
        self._current_phase = "uninitialized"
        self._trial_status = "idle"
        self._terminal_trial_status: str | None = None
        self._terminal_status_published = False
        self._last_log_status = ""
        self._last_publish_by_key: dict[tuple[str, str, str], rclpy.time.Time] = {}

        self._status_publisher = self.create_publisher(
            String,
            self._safety_status_topic,
            10,
        )
        self.create_subscription(
            JointState,
            self._joint_states_topic,
            self._on_joint_state,
            20,
        )
        self.create_subscription(
            String,
            self._task_phase_topic,
            self._on_task_phase,
            20,
        )
        self.create_subscription(
            String,
            self._trial_status_topic,
            self._on_trial_status,
            20,
        )
        self.create_timer(self._monitor_period_sec, self._publish_monitor_status)

        self.get_logger().info(f"Loaded safety limits from {self._config_path}")
        self.get_logger().info(
            f"Monitoring {self._joint_states_topic}, {self._task_phase_topic}, and "
            f"{self._trial_status_topic}; "
            f"publishing {self._safety_status_topic}"
        )

    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_state_time = self.get_clock().now()
        if self._terminal_trial_status is not None:
            return
        if not self._safety_enabled:
            self._publish_status(
                "OK",
                "joint_states_valid",
                "safety monitor disabled by configuration",
                throttle=True,
            )
            return

        positions_by_name = {
            name: position for name, position in zip(message.name, message.position)
        }
        violations: list[str] = []
        warnings: list[str] = []

        for joint_name in self.JOINT_NAMES:
            if joint_name not in positions_by_name:
                violations.append(f"missing {joint_name}")
                continue

            position = positions_by_name[joint_name]
            if not math.isfinite(position):
                violations.append(f"{joint_name} is NaN/Inf")
                continue

            limits = self._joint_limits[joint_name]
            lower = float(limits["min"])
            upper = float(limits["max"])
            if position < lower or position > upper:
                violations.append(
                    f"{joint_name}={position:.4f} outside [{lower:.4f}, {upper:.4f}]"
                )
                continue

            margin = min(position - lower, upper - position)
            warning_band = 0.10 * (upper - lower)
            if margin <= warning_band:
                warnings.append(
                    f"{joint_name}={position:.4f} near soft limit "
                    f"[{lower:.4f}, {upper:.4f}]"
                )

        if violations:
            self._publish_status(
                "VIOLATION",
                "joint_limit_violation",
                "; ".join(violations),
                throttle=True,
            )
        elif warnings:
            self._publish_status(
                "WARNING",
                "joint_limit_warning",
                "; ".join(warnings),
                throttle=True,
            )
        else:
            self._publish_status(
                "OK",
                "joint_states_valid",
                f"joint states valid during phase {self._current_phase}",
                throttle=True,
            )

    def _on_task_phase(self, message: String) -> None:
        phase = message.data.strip() or "empty_phase"
        self._current_phase = phase
        self._last_phase_time = self.get_clock().now()
        self.get_logger().info(f"Task phase updated: {phase}")

    def _on_trial_status(self, message: String) -> None:
        status = message.data.strip() or "empty_status"
        self._trial_status = status
        if status in self.TERMINAL_TRIAL_STATUSES:
            self._terminal_trial_status = status
            self._publish_terminal_status(status)

    def _publish_monitor_status(self) -> None:
        if self._terminal_trial_status is not None:
            self._publish_terminal_status(self._terminal_trial_status)
            return

        if not self._safety_enabled:
            self._publish_status(
                "OK",
                "joint_states_valid",
                "safety monitor disabled by configuration",
                throttle=True,
            )
            return

        now = self.get_clock().now()
        if self._last_joint_state_time is None:
            self._publish_status(
                "WARNING",
                "waiting_for_joint_states",
                "waiting for first /joint_states message",
                throttle=True,
            )
            return

        joint_age = self._age_sec(now, self._last_joint_state_time)
        if joint_age > self._joint_state_timeout_sec:
            self._publish_status(
                "VIOLATION",
                "joint_state_timeout",
                f"missing joint states for {joint_age:.2f}s",
                throttle=True,
            )
            return

        if self._last_phase_time is None:
            self._publish_status(
                "WARNING",
                "waiting_for_task_phase",
                "waiting for first /task_phase message",
                throttle=True,
            )
            return

        phase_age = self._age_sec(now, self._last_phase_time)
        if phase_age > self._max_phase_duration_sec:
            self._publish_status(
                "WARNING",
                "waiting_for_task_phase",
                f"phase {self._current_phase} active for {phase_age:.2f}s; "
                "phase timeout policy is monitor-only in v0.2",
                throttle=True,
            )

    def _publish_terminal_status(self, status: str) -> None:
        if self._terminal_status_published:
            return

        level = "OK" if status in {"completed", "guarded_contact_stop"} else "WARNING"
        self._publish_status(
            level,
            "trial_terminal",
            f"trial reached terminal status {status}",
        )
        self._terminal_status_published = True

    def _publish_status(
        self,
        level: str,
        code: str,
        detail: str,
        *,
        throttle: bool = False,
    ) -> None:
        if throttle and not self._should_publish(level, code, detail):
            return

        payload = {
            "timestamp_ros_sec": self._now_sec(),
            "level": level,
            "code": code,
            "phase": self._current_phase,
            "message": detail,
        }
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._status_publisher.publish(message)

        log_status = f"{level}: {code}: {detail}"
        if log_status == self._last_log_status:
            return
        self._last_log_status = log_status
        if level == "OK":
            self.get_logger().info(log_status)
        elif level == "WARNING":
            self.get_logger().warning(log_status)
        else:
            self.get_logger().error(log_status)

    def _should_publish(self, level: str, code: str, detail: str) -> bool:
        now = self.get_clock().now()
        key = (level, code, self._current_phase)
        last_publish = self._last_publish_by_key.get(key)
        if last_publish is not None:
            age_sec = self._age_sec(now, last_publish)
            if age_sec < self._warning_throttle_sec:
                return False
        self._last_publish_by_key[key] = now
        return True

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _age_sec(now: rclpy.time.Time, then: rclpy.time.Time) -> float:
        return (now - then).nanoseconds / 1_000_000_000.0

    @classmethod
    def _load_config(cls, config_path: Path) -> dict[str, Any]:
        if str(config_path) == ".":
            raise ValueError("Parameter 'config_path' must point to a YAML config file.")
        if not config_path.is_file():
            raise FileNotFoundError(f"Safety config does not exist: {config_path}")

        with config_path.open("r", encoding="utf-8") as config_file:
            loaded = yaml.safe_load(config_file)

        if not isinstance(loaded, dict):
            raise ValueError(f"Safety config must be a YAML mapping: {config_path}")

        config = loaded.get("safety_monitor", {}).get("ros__parameters", loaded)
        cls._validate_config(config)
        return config

    @classmethod
    def _validate_config(cls, config: Any) -> None:
        if not isinstance(config, dict):
            raise ValueError("Safety config parameters must be a YAML mapping.")

        required = (
            "joint_soft_limits",
            "joint_state_timeout_sec",
            "max_phase_duration_sec",
            "safety_enabled",
        )
        missing = [key for key in required if key not in config]
        if missing:
            raise ValueError(f"Missing safety config keys: {missing}")

        limits = config["joint_soft_limits"]
        if not isinstance(limits, dict):
            raise ValueError("'joint_soft_limits' must be a mapping.")

        for joint_name in cls.JOINT_NAMES:
            if joint_name not in limits:
                raise ValueError(f"Missing soft limits for {joint_name}.")
            joint_limits = limits[joint_name]
            if not isinstance(joint_limits, dict):
                raise ValueError(f"Soft limits for {joint_name} must be a mapping.")
            lower = joint_limits.get("min")
            upper = joint_limits.get("max")
            if not isinstance(lower, (float, int)) or not isinstance(upper, (float, int)):
                raise ValueError(f"Soft limits for {joint_name} must be numeric.")
            if lower >= upper:
                raise ValueError(f"Soft limit min must be less than max for {joint_name}.")

        for key in ("joint_state_timeout_sec", "max_phase_duration_sec"):
            value = config[key]
            if not isinstance(value, (float, int)) or value <= 0.0:
                raise ValueError(f"'{key}' must be a positive number.")

        if not isinstance(config["safety_enabled"], bool):
            raise ValueError("'safety_enabled' must be true or false.")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SafetyMonitor | None = None

    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - top-level node failure logging.
        if node is not None:
            node.get_logger().error(f"Safety monitor failed: {exc}")
        else:
            print(f"Safety monitor failed during startup: {exc}")
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

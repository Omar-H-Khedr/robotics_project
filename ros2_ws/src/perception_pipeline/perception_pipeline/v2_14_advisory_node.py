#!/usr/bin/env python3
"""v2_14 guarded advisory integration node.

Runs v2_14 as a safety-gated advisory layer. The deterministic controller
remains in full authority. This node ONLY suggests phase/action hints for
non-safety-critical transitions.

Advisory policy:
  - MOVING_TO_START, APPROACH, SEARCH: advisory may suggest direction hints
  - INSERT: always defers to deterministic controller (NEVER advisory-controlled)
  - RETREAT: advisory only if confidence > 0.95 AND margin > 0.5
  - DONE: NEVER accepted from ML (3.4% precision, 461/461 FP from RETREAT)
  - Low confidence: fallback
  - Inconsistent phase prediction: fallback

Logging fields:
  timestamp, det_phase, pred_phase, confidence, proposed_action,
  accepted, rejected, fallback, fallback_reason, safety_gate_status,
  unsafe_if_executed, done_false_positive_flag, retreat_uncertainty_flag

Usage:
  ros2 run perception_pipeline v2_14_advisory_node -- \
      --ros-args -p model_path:=... -p output_dir:=...
"""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, asdict
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, String

try:
    from geometry_msgs.msg import WrenchStamped
except ImportError:
    WrenchStamped = None

from perception_pipeline.context_vector import (
    CONTEXT_DIM,
    encode_rgb_to_48,
    phase_to_int,
    safety_to_int,
    summarize_depth_msg,
)
from perception_pipeline.v2_14_safety_gated_action import (
    SafetyGatedActionInterface,
    PhaseID,
    CONFIDENCE_THRESHOLD,
)

PHASE_INT_TO_NAME = {
    0: "UNKNOWN", 1: "MOVING_TO_START", 2: "APPROACH", 3: "SEARCH",
    5: "INSERT", 6: "RETREAT", 7: "DONE",
}

DONE_PRECISION_KNOWN_ISSUE = (
    "DONE precision=3.4% (461/462 FP from RETREAT). "
    "DONE prediction NEVER trusted from ML alone."
)

RETREAT_UNCERTAINTY_NOTE = (
    "RETREAT recall=78.1%. Some RETREAT ticks misclassified as DONE. "
    "Advisory requires high confidence (>0.95) and margin (>0.5)."
)


@dataclass
class AdvisoryDecision:
    """Single advisory decision record for logging."""
    stamp_s: float = 0.0
    tick_index: int = 0
    det_phase: str = "UNKNOWN"
    det_phase_int: int = 0
    det_safety_level: str = "UNKNOWN"
    pred_phase_int: int = 0
    pred_phase_name: str = "UNKNOWN"
    confidence: float = 0.0
    margin: float = 0.0
    proposed_action: str = "none"
    accepted: bool = False
    rejected: bool = False
    fallback: bool = True
    fallback_reason: str = ""
    safety_gate_status: str = ""
    unsafe_if_executed: bool = False
    done_false_positive_flag: bool = False
    retreat_uncertainty_flag: bool = False

    def to_row(self) -> list:
        return [
            f"{self.stamp_s:.6f}",
            self.tick_index,
            self.det_phase, self.det_phase_int, self.det_safety_level,
            self.pred_phase_int, self.pred_phase_name,
            f"{self.confidence:.4f}", f"{self.margin:.4f}",
            self.proposed_action,
            self.accepted, self.rejected,
            self.fallback, self.fallback_reason,
            self.safety_gate_status,
            self.unsafe_if_executed,
            self.done_false_positive_flag,
            self.retreat_uncertainty_flag,
        ]


CSV_HEADER = [
    "stamp_s", "tick_index",
    "det_phase", "det_phase_int", "det_safety_level",
    "pred_phase_int", "pred_phase_name", "confidence", "margin",
    "proposed_action", "accepted", "rejected",
    "fallback", "fallback_reason", "safety_gate_status",
    "unsafe_if_executed", "done_false_positive_flag", "retreat_uncertainty_flag",
]


class GuardedAdvisoryInterface:
    """Computes advisory decisions from ML predictions with strict safety guards.

    This is the core advisory logic. It takes an ML prediction and the
    current deterministic phase, and decides whether to accept/reject
    the advisory.
    """

    def __init__(self, confidence_threshold: float = CONFIDENCE_THRESHOLD):
        self.confidence_threshold = confidence_threshold

    def decide(
        self,
        pred_phase: int,
        pred_phase_name: str,
        confidence: float,
        margin: float,
        det_phase: str,
        det_phase_int: int,
        logits: list[float],
    ) -> AdvisoryDecision:
        """Produce an advisory decision from ML prediction + deterministic state."""
        decision = AdvisoryDecision(
            pred_phase_int=pred_phase,
            pred_phase_name=pred_phase_name,
            confidence=confidence,
            margin=margin,
            det_phase=det_phase,
            det_phase_int=det_phase_int,
        )

        # 1. DONE: NEVER accepted from ML (3.4% precision, FP from RETREAT)
        if pred_phase == PhaseID.DONE:
            decision.rejected = True
            decision.fallback = True
            decision.fallback_reason = "done_never_trusted_from_ml"
            decision.done_false_positive_flag = True
            decision.unsafe_if_executed = True
            decision.safety_gate_status = "DONE_BLOCKED"
            decision.proposed_action = "none"
            return decision

        # 2. INSERT: always defers to deterministic controller
        if pred_phase == PhaseID.INSERT:
            decision.rejected = True
            decision.fallback = True
            decision.fallback_reason = "insert_defers_to_deterministic"
            decision.safety_gate_status = "INSERT_DEFERRED"
            decision.proposed_action = "none"
            return decision

        # 3. RETREAT: advisory only with high confidence + high margin
        if pred_phase == PhaseID.RETREAT:
            decision.retreat_uncertainty_flag = True
            if confidence < 0.95 or margin < 0.5:
                decision.rejected = True
                decision.fallback = True
                decision.fallback_reason = (
                    f"retreat_low_certainty: conf={confidence:.3f}, margin={margin:.3f}"
                )
                decision.safety_gate_status = "RETREAT_UNCERTAIN"
                decision.proposed_action = "none"
                return decision
            # RETREAT with high certainty: accept as advisory
            decision.accepted = True
            decision.fallback = False
            decision.safety_gate_status = "RETREAT_ACCEPTED"
            decision.proposed_action = "suggest_retreat"
            return decision

        # 4. Low confidence: fallback
        if confidence < self.confidence_threshold:
            decision.rejected = True
            decision.fallback = True
            decision.fallback_reason = f"low_confidence={confidence:.3f}"
            decision.safety_gate_status = "LOW_CONFIDENCE"
            decision.proposed_action = "none"
            return decision

        # 5. Safety-critical phases with low margin: fallback
        if pred_phase in (PhaseID.SEARCH,):
            if margin < 0.3:
                decision.rejected = True
                decision.fallback = True
                decision.fallback_reason = f"search_low_margin={margin:.3f}"
                decision.safety_gate_status = "SEARCH_UNCERTAIN"
                decision.proposed_action = "none"
                return decision

        # 6. MOVING_TO_START, APPROACH: accept as advisory for direction hints
        if pred_phase in (PhaseID.MOVING_TO_START, PhaseID.APPROACH, PhaseID.SEARCH):
            decision.accepted = True
            decision.fallback = False
            decision.safety_gate_status = "ADVISORY_ACCEPTED"
            decision.proposed_action = f"suggest_{pred_phase_name.lower()}"
            return decision

        # 7. UNKNOWN or anything else: reject
        decision.rejected = True
        decision.fallback = True
        decision.fallback_reason = f"unrecognized_phase={pred_phase_name}"
        decision.safety_gate_status = "UNKNOWN_PHASE"
        decision.proposed_action = "none"
        return decision


class V2_14AdvisoryNode(Node):
    """ROS2 node for guarded advisory integration."""

    def __init__(self) -> None:
        super().__init__("v2_14_advisory_node")

        self.declare_parameter(
            "model_path",
            "diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt",
        )
        self.declare_parameter("output_dir", "diagnostics/v2_14_advisory_v1")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("confidence_threshold", 0.85)
        self.declare_parameter("rgb_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_rect_raw")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ft_topic", "/ft_sensor_wrench")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("safety_status_topic", "/safety_status")

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_absolute():
            model_path = Path.cwd() / model_path
        model_path = model_path.resolve()
        self.get_logger().info(
            f"Loading v2_14 model from {model_path} (exists={model_path.exists()})"
        )

        try:
            self._interface = SafetyGatedActionInterface(
                model_path=str(model_path),
                confidence_threshold=float(
                    self.get_parameter("confidence_threshold").value
                ),
            )
            self.get_logger().info(
                f"v2_14 model loaded: {self._interface._model is not None}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to load v2_14 model: {exc}")
            self._interface = SafetyGatedActionInterface(model_path=None)

        self._advisory = GuardedAdvisoryInterface(
            confidence_threshold=float(
                self.get_parameter("confidence_threshold").value
            )
        )

        output_dir = Path(str(self.get_parameter("output_dir").value))
        output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = output_dir / "advisory_log.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(CSV_HEADER)
        self._csv_file.flush()

        self._bridge = CvBridge()
        self._tick_index = 0
        self._accepted_count = 0
        self._rejected_count = 0
        self._done_fp_count = 0
        self._retreat_uncertain_count = 0
        self._unsafe_count = 0

        self._joint_state: Optional[JointState] = None
        self._ft = np.zeros(6)
        self._latest_rgb: Optional[Image] = None
        self._latest_depth: Optional[Image] = None
        self._task_phase = "UNKNOWN"
        self._safety_status = "UNKNOWN"

        self.create_subscription(
            JointState, str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state, 10,
        )
        self.create_subscription(
            WrenchStamped, str(self.get_parameter("ft_topic").value),
            self._on_ft, 10,
        )
        self.create_subscription(
            Image, str(self.get_parameter("rgb_topic").value),
            self._on_rgb, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value),
            self._on_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            String, str(self.get_parameter("task_phase_topic").value),
            lambda msg: setattr(self, "_task_phase", msg.data), 10,
        )
        self.create_subscription(
            String, str(self.get_parameter("safety_status_topic").value),
            lambda msg: setattr(self, "_safety_status", msg.data), 10,
        )

        self._pub_advisory_phase = self.create_publisher(
            String, "/v2_14_advisory/predicted_phase", 10,
        )
        self._pub_advisory_conf = self.create_publisher(
            Float64, "/v2_14_advisory/confidence", 10,
        )
        self._pub_advisory_accepted = self.create_publisher(
            Bool, "/v2_14_advisory/accepted", 10,
        )
        self._pub_advisory_fallback = self.create_publisher(
            Bool, "/v2_14_advisory/fallback", 10,
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(1.0 / max(1e-3, rate_hz), self._on_tick)
        self.get_logger().info(
            f"v2_14_advisory_node writing to {self._csv_path} at {rate_hz} Hz"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg

    def _on_ft(self, msg) -> None:
        if WrenchStamped is not None and isinstance(msg, WrenchStamped):
            self._ft = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ])
        else:
            self._ft = np.array([msg.x, msg.y, msg.z, 0.0, 0.0, 0.0])

    def _on_rgb(self, msg: Image) -> None:
        self._latest_rgb = msg

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg

    def _compute_context_68(self) -> Optional[np.ndarray]:
        js = self._joint_state
        if js is None or len(js.position) < 6:
            return None

        pos = list(js.position[:6]) + [0.0] * (6 - len(js.position[:6]))
        pos = [float(p) if np.isfinite(p) else 0.0 for p in pos]

        vel = list(js.velocity[:6]) if js.velocity and len(js.velocity) >= 6 else [0.0] * 6
        vel = [float(v) if np.isfinite(v) else 0.0 for v in vel]
        vel = vel + [0.0] * (6 - len(vel))

        rgb_w, rgb_h, rgb_vec = encode_rgb_to_48(self._latest_rgb, self._bridge)

        if self._latest_depth is not None:
            d = summarize_depth_msg(self._latest_depth, self._bridge)
        else:
            d = (0, 0, 0.0, 0.0, 0.0, 0.0)
        depth_vec = np.array([float(x) for x in d], dtype=np.float32)
        depth_vec[~np.isfinite(depth_vec)] = 0.0

        phase_int = float(phase_to_int(self._task_phase))
        safety_int = float(safety_to_int(self._safety_status))

        c = np.zeros(68, dtype=np.float32)
        c[0:48] = rgb_vec
        c[48:54] = depth_vec
        c[54:60] = pos
        c[60:66] = vel
        c[66] = phase_int
        c[67] = safety_int
        return c

    def _on_tick(self) -> None:
        ctx = self._compute_context_68()
        if ctx is None:
            return
        if ctx[:48].sum() < 1.0 or ctx[48] == 0.0 or ctx[49] == 0.0:
            return

        cmd = self._interface.predict(ctx)

        gt_phase = self._task_phase
        gt_phase_int = phase_to_int(gt_phase)
        try:
            sj = json.loads(self._safety_status) if self._safety_status else {}
            if not isinstance(sj, dict):
                sj = {}
        except Exception:
            sj = {}
        gt_safety_level = str(sj.get("level", "UNKNOWN"))

        margin = 0.0
        if cmd.raw_logits and len(cmd.raw_logits) >= 2:
            sorted_l = sorted(cmd.raw_logits, reverse=True)
            margin = sorted_l[0] - sorted_l[1]

        decision = self._advisory.decide(
            pred_phase=cmd.phase,
            pred_phase_name=cmd.phase_name,
            confidence=cmd.confidence,
            margin=margin,
            det_phase=gt_phase,
            det_phase_int=gt_phase_int,
            logits=cmd.raw_logits,
        )
        decision.stamp_s = time.time()
        decision.tick_index = self._tick_index

        if decision.accepted:
            self._accepted_count += 1
        if decision.rejected:
            self._rejected_count += 1
        if decision.done_false_positive_flag:
            self._done_fp_count += 1
        if decision.retreat_uncertainty_flag:
            self._retreat_uncertain_count += 1
        if decision.unsafe_if_executed:
            self._unsafe_count += 1

        phase_msg = String()
        phase_msg.data = cmd.phase_name
        self._pub_advisory_phase.publish(phase_msg)

        conf_msg = Float64()
        conf_msg.data = cmd.confidence
        self._pub_advisory_conf.publish(conf_msg)

        acc_msg = Bool()
        acc_msg.data = decision.accepted
        self._pub_advisory_accepted.publish(acc_msg)

        fb_msg = Bool()
        fb_msg.data = decision.fallback
        self._pub_advisory_fallback.publish(fb_msg)

        self._writer.writerow(decision.to_row())
        self._csv_file.flush()
        self._tick_index += 1

    def _log_summary(self) -> dict:
        return {
            "total_ticks": self._tick_index,
            "accepted": self._accepted_count,
            "rejected": self._rejected_count,
            "done_false_positives": self._done_fp_count,
            "retreat_uncertain": self._retreat_uncertain_count,
            "unsafe_if_executed": self._unsafe_count,
            "acceptance_rate": (
                self._accepted_count / max(1, self._tick_index)
            ),
        }

    def destroy_node(self) -> None:
        try:
            summary = self._log_summary()
            summary_path = self._csv_path.parent / "advisory_summary.json"
            summary_path.write_text(json.dumps(summary, indent=2))
            self.get_logger().info(f"Advisory summary: {json.dumps(summary)}")
        except Exception:
            pass
        try:
            if hasattr(self, "_csv_file") and not self._csv_file.closed:
                self._csv_file.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = V2_14AdvisoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

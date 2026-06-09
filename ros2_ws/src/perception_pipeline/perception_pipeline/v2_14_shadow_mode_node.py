#!/usr/bin/env python3
"""v2_14 shadow-mode inference node.

Passive shadow-mode node for validating the v2_14 raw 68-dim safety-gated
classifier during live full-task trials.  The deterministic controller
remains in charge; this node only observes and logs.

Subscribes to:
  /joint_states, /ft_sensor_wrench, /task_phase, /safety_status,
  /d405/color/image_raw, /d405/depth/image_rect_raw

Publishes:
  /v2_14_shadow/predicted_phase   (String)
  /v2_14_shadow/confidence        (Float64)
  /v2_14_shadow/use_fallback      (Bool)

Logs:
  shadow_v2_14_inference_log.csv  with per-tick predictions and ground truth.

Usage:
  ros2 run perception_pipeline v2_14_shadow_mode_node -- \
      --ros-args -p model_path:=diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt \
                 -p output_dir:=diagnostics/v2_14_shadow_mode_v1
"""

from __future__ import annotations

import csv
import json
import time
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
    DEPTH_VALUE_INDICES,
    DEPTH_CLIP_VALUE,
    encode_rgb_to_48,
    phase_to_int,
    safety_to_int,
    summarize_depth_msg,
)
from perception_pipeline.v2_14_safety_gated_action import (
    SafetyGatedActionInterface,
    PhaseID,
)


PHASE_INT_TO_NAME = {
    0: "UNKNOWN", 1: "MOVING_TO_START", 2: "APPROACH", 3: "SEARCH",
    5: "INSERT", 6: "RETREAT", 7: "DONE",
}


class V2_14ShadowModeNode(Node):
    """Passive shadow-mode v2_14 inference for live trial validation."""

    def __init__(self) -> None:
        super().__init__("v2_14_shadow_mode_node")

        self.declare_parameter(
            "model_path",
            "diagnostics/v2_14_raw_safety_gated_v4/raw_context_classifier.pt",
        )
        self.declare_parameter("output_dir", "diagnostics/v2_14_shadow_mode_v1")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("confidence_threshold", 0.85)
        self.declare_parameter("rgb_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_rect_raw")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ft_topic", "/ft_sensor_wrench")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("safety_status_topic", "/safety_status")

        model_path = Path(str(self.get_parameter("model_path").value))
        self.get_logger().info(f"Loading v2_14 model from {model_path}")

        self._interface = SafetyGatedActionInterface(
            model_path=str(model_path),
            confidence_threshold=float(
                self.get_parameter("confidence_threshold").value
            ),
        )

        output_dir = Path(str(self.get_parameter("output_dir").value))
        output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = output_dir / "shadow_v2_14_inference_log.csv"
        self._csv_file = open(self._csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            "stamp_s", "tick_index",
            "gt_phase", "gt_phase_int", "gt_safety_level",
            "pred_phase_int", "pred_phase_name", "confidence",
            "use_fallback", "fallback_reason",
            "agreement", "margin",
        ] + [f"logit_{i}" for i in range(7)])
        self._csv_file.flush()

        self._bridge = CvBridge()
        self._tick_index = 0
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

        self._pub_phase = self.create_publisher(
            String, "/v2_14_shadow/predicted_phase", 10,
        )
        self._pub_conf = self.create_publisher(
            Float64, "/v2_14_shadow/confidence", 10,
        )
        self._pub_fallback = self.create_publisher(
            Bool, "/v2_14_shadow/use_fallback", 10,
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._timer = self.create_timer(
            1.0 / max(1e-3, rate_hz), self._on_tick
        )
        self.get_logger().info(
            f"v2_14_shadow_mode_node writing to {self._csv_path} at {rate_hz} Hz"
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
        """Compute 68-dim context vector from live topics."""
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

        gt_phase_mapped = gt_phase_int
        pred_phase_mapped = cmd.phase
        agreement = (gt_phase_mapped == pred_phase_mapped)

        margin = 0.0
        if cmd.raw_logits and len(cmd.raw_logits) >= 2:
            sorted_l = sorted(cmd.raw_logits, reverse=True)
            margin = sorted_l[0] - sorted_l[1]

        phase_msg = String()
        phase_msg.data = cmd.phase_name
        self._pub_phase.publish(phase_msg)

        conf_msg = Float64()
        conf_msg.data = cmd.confidence
        self._pub_conf.publish(conf_msg)

        fb_msg = Bool()
        fb_msg.data = cmd.use_fallback
        self._pub_fallback.publish(fb_msg)

        self._writer.writerow([
            f"{time.time():.6f}",
            self._tick_index,
            gt_phase, gt_phase_int, gt_safety_level,
            cmd.phase, cmd.phase_name, f"{cmd.confidence:.4f}",
            cmd.use_fallback, cmd.fallback_reason,
            agreement, f"{margin:.4f}",
            *[f"{x:.6f}" for x in cmd.raw_logits],
        ])
        self._csv_file.flush()
        self._tick_index += 1

    def destroy_node(self) -> None:
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
    node = V2_14ShadowModeNode()
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

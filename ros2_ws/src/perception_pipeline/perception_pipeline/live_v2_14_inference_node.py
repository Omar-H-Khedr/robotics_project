"""Live v2_14 inference node.

Subscribes to the same topics as multimodal_observation_logger,
computes the 74-dim context vector on the fly, runs the v2_13_v2
frozen encoder + v2_14 PhaseHead MLP, and publishes:

  /v2_14/predicted_phase   std_msgs/String  ("MOVE_TO_START" etc.)
  /v2_14/target_joint_pose std_msgs/Float64MultiArray (6 floats, rad)
  /v2_14/latent            std_msgs/Float64MultiArray (32 floats)
  /v2_14/prediction_log    CSV row per inference (for offline ablation)

This is a passive inference node: it does NOT publish any
/JointTrajectory corrections. The "action" output is published
as a topic; integrating it with the JTC is a follow-up milestone.

The node is designed to be enabled in research_baseline.launch.py
with `enable_v2_14_live_inference:=true` (or run standalone via
`ros2 run perception_pipeline live_v2_14_inference_node -- ...`).
"""
from __future__ import annotations

import csv
import json
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import torch
import torch.nn as nn
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
try:
    from geometry_msgs.msg import WrenchStamped
except ImportError:  # pragma: no cover
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


PHASE_INT_TO_NAME = {
    0: "UNKNOWN", 1: "MOVE_TO_START", 2: "APPROACH", 3: "SEARCH",
    4: "HOVER_ABOVE_HOLE", 5: "INSERT", 6: "INSERTED",
    7: "DONE", 8: "ABORT",
}


class _Encoder(nn.Module):
    def __init__(self, input_dim: int, latent_dim: int, hidden_dims, dropout: float):
        super().__init__()
        layers = []
        prev = input_dim
        for h in hidden_dims:
            layers.extend([nn.Linear(prev, h), nn.ReLU(), nn.Dropout(dropout)])
            prev = h
        self.encoder_body = nn.Sequential(*layers)
        self.encoder_head = nn.Linear(prev, latent_dim)

    def encode(self, x):
        return self.encoder_head(self.encoder_body(x))


class _PhaseHead(nn.Module):
    def __init__(self, input_dim: int, hidden: int, num_classes: int,
                 joint_dim: int, dropout: float):
        super().__init__()
        self.classifier = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, num_classes),
        )
        self.regressor = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, joint_dim),
        )

    def forward(self, z):
        return self.classifier(z), self.regressor(z)


class LiveV2_14InferenceNode(Node):
    def __init__(self) -> None:
        super().__init__("live_v2_14_inference_node")
        self.declare_parameter("encoder_pt",
                               "diagnostics/perception_pipeline_v2_13_encoder_v2/encoder.pt")
        self.declare_parameter("scaler_json",
                               "diagnostics/perception_pipeline_v2_13_encoder_v2/scaler.json")
        self.declare_parameter("action_classifier_pt",
                               "diagnostics/perception_pipeline_v2_14_action/action_classifier.pt")
        self.declare_parameter("output_dir",
                               "diagnostics/perception_pipeline_live_v2_14_v1")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("rgb_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_rect_raw")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ft_topic", "/ft_sensor_wrench")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("safety_status_topic", "/safety_status")
        self.declare_parameter("predicted_phase_topic", "/v2_14/predicted_phase")
        self.declare_parameter("target_joint_pose_topic", "/v2_14/target_joint_pose")
        self.declare_parameter("latent_topic", "/v2_14/latent")
        self.declare_parameter("csv_basename", "live_v2_14_inference_log.csv")

        enc_path = Path(self.get_parameter("encoder_pt").value)
        scaler_path = Path(self.get_parameter("scaler_json").value)
        head_path = Path(self.get_parameter("action_classifier_pt").value)
        self.get_logger().info(f"loading encoder {enc_path}")
        self.get_logger().info(f"loading scaler {scaler_path}")
        self.get_logger().info(f"loading head {head_path}")
        with open(scaler_path) as f:
            scaler = json.load(f)
        self._mn = np.array(scaler["min"], dtype=np.float32)
        self._mx = np.array(scaler["max"], dtype=np.float32)
        self._rng = (self._mx - self._mn).astype(np.float32)
        self._rng[self._rng < 1e-8] = 1.0

        enc_ckpt = torch.load(enc_path, map_location="cpu", weights_only=False)
        self._encoder = _Encoder(
            input_dim=enc_ckpt["input_dim"],
            latent_dim=enc_ckpt["latent_dim"],
            hidden_dims=tuple(enc_ckpt["hidden_dims"]),
            dropout=enc_ckpt["dropout"],
        )
        full_state = enc_ckpt["state_dict"]
        encoder_state = {k: v for k, v in full_state.items() if k.startswith("encoder_")}
        self._encoder.load_state_dict(encoder_state)
        self._encoder.eval()

        head_ckpt = torch.load(head_path, map_location="cpu", weights_only=False)
        self._head = _PhaseHead(
            input_dim=head_ckpt["input_dim"],
            hidden=head_ckpt["hidden"],
            num_classes=head_ckpt["num_classes"],
            joint_dim=head_ckpt["joint_dim"],
            dropout=head_ckpt["dropout"],
        )
        self._head.load_state_dict(head_ckpt["state_dict"])
        self._head.eval()
        self._num_classes = int(head_ckpt["num_classes"])
        self._joint_dim = int(head_ckpt["joint_dim"])
        self.get_logger().info(
            f"loaded encoder (74->{enc_ckpt['latent_dim']}) + head "
            f"({head_ckpt['input_dim']}->{head_ckpt['num_classes']} + {head_ckpt['joint_dim']})"
        )

        self._output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._csv_path = self._output_dir / self.get_parameter("csv_basename").value
        self._csv_file = open(self._csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            "stamp_s", "tick_index",
            "ground_truth_phase", "ground_truth_safety",
            "predicted_phase_int", "predicted_phase_name",
            "target_joint_1", "target_joint_2", "target_joint_3",
            "target_joint_4", "target_joint_5", "target_joint_6",
        ])
        self._csv_file.flush()
        self._tick_index = 0

        self._bridge = CvBridge()
        self._joint_state_msg: Optional[JointState] = None
        self._ft = np.zeros(6)
        self._latest_rgb: Optional[Image] = None
        self._latest_depth: Optional[Image] = None
        self._task_phase = "UNKNOWN"
        self._safety_status = "UNKNOWN"

        self.create_subscription(
            JointState, self.get_parameter("joint_state_topic").value,
            self._on_joint_state, 10,
        )
        self.create_subscription(
            WrenchStamped, self.get_parameter("ft_topic").value,
            self._on_ft, 10,
        )
        self.create_subscription(
            Image, self.get_parameter("rgb_topic").value,
            self._on_rgb, qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self._on_depth, qos_profile_sensor_data,
        )
        self.create_subscription(
            String, self.get_parameter("task_phase_topic").value,
            lambda msg: setattr(self, "_task_phase", msg.data), 10,
        )
        self.create_subscription(
            String, self.get_parameter("safety_status_topic").value,
            lambda msg: setattr(self, "_safety_status", msg.data), 10,
        )

        self._pub_phase = self.create_publisher(
            String, self.get_parameter("predicted_phase_topic").value, 10,
        )
        self._pub_target = self.create_publisher(
            Float64MultiArray, self.get_parameter("target_joint_pose_topic").value, 10,
        )
        self._pub_latent = self.create_publisher(
            Float64MultiArray, self.get_parameter("latent_topic").value, 10,
        )

        rate_hz = float(self.get_parameter("rate_hz").value)
        period_s = 1.0 / max(1e-3, rate_hz)
        self._timer = self.create_timer(period_s, self._on_tick)
        self.get_logger().info(
            f"live_v2_14_inference_node writing to {self._csv_path} at {rate_hz} Hz"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state_msg = msg

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

    def _compute_context(self) -> Optional[np.ndarray]:
        js = self._joint_state_msg
        if js is None or len(js.position) < 6:
            return None
        pos = list(js.position[:6]) + [0.0] * (6 - len(js.position[:6]))
        pos = [float(p) if np.isfinite(p) else 0.0 for p in pos]
        if js.velocity and len(js.velocity) >= 6:
            vel = [float(v) if np.isfinite(v) else 0.0 for v in js.velocity[:6]]
        else:
            vel = [0.0] * 6
        vel = vel + [0.0] * (6 - len(vel))

        rgb_w, rgb_h, rgb_vec = encode_rgb_to_48(self._latest_rgb, self._bridge)
        if self._latest_depth is not None:
            d = summarize_depth_msg(self._latest_depth, self._bridge)
        else:
            d = (0, 0, 0.0, 0.0, 0.0, 0.0)
        depth_vec = np.array([float(x) for x in d], dtype=np.float32)
        depth_vec[~np.isfinite(depth_vec)] = 0.0

        ft = self._ft.copy()
        ft[~np.isfinite(ft)] = 0.0

        phase_int = phase_to_int(self._task_phase)
        safety_int = safety_to_int(self._safety_status)

        c = np.zeros(CONTEXT_DIM, dtype=np.float32)
        c[0:48] = rgb_vec
        c[48:54] = depth_vec
        c[54:60] = ft
        c[60:66] = pos
        c[66:72] = vel
        c[72] = float(phase_int)
        c[73] = float(safety_int)
        return c

    def _preprocess(self, c: np.ndarray) -> np.ndarray:
        out = c.copy()
        for i in DEPTH_VALUE_INDICES:
            if not np.isfinite(out[i]):
                out[i] = 0.0
            out[i] = min(max(out[i], 0.0), DEPTH_CLIP_VALUE)
            out[i] = float(np.log1p(out[i]))
        out[~np.isfinite(out)] = 0.0
        out = (out - self._mn) / self._rng
        out[~np.isfinite(out)] = 0.0
        return out

    def _on_tick(self) -> None:
        c = self._compute_context()
        if c is None:
            return
        if c[:48].sum() < 1.0 or c[48] == 0.0 or c[49] == 0.0:
            return
        x = self._preprocess(c)
        with torch.no_grad():
            z = self._encoder.encode(torch.from_numpy(x).float().unsqueeze(0))
            logits, reg = self._head(z)
            pred = int(logits.argmax(dim=1).item())
            target = reg.squeeze(0).cpu().numpy()
            z_np = z.squeeze(0).cpu().numpy()

        phase_msg = String()
        phase_msg.data = PHASE_INT_TO_NAME.get(pred, "UNKNOWN")
        self._pub_phase.publish(phase_msg)
        target_msg = Float64MultiArray()
        target_msg.data = [float(v) for v in target]
        self._pub_target.publish(target_msg)
        latent_msg = Float64MultiArray()
        latent_msg.data = [float(v) for v in z_np]
        self._pub_latent.publish(latent_msg)

        gt_phase_int = phase_to_int(self._task_phase)
        gt_safety_int = safety_to_int(self._safety_status)
        try:
            sj = json.loads(self._safety_status) if self._safety_status else {}
            if not isinstance(sj, dict):
                sj = {}
        except Exception:
            sj = {}
        safety_level_str = str(sj.get("level", "UNKNOWN"))
        self._writer.writerow([
            f"{time.time():.6f}",
            self._tick_index,
            self._task_phase, safety_level_str,
            pred, phase_msg.data,
            *[f"{v:.6f}" for v in target],
        ])
        self._csv_file.flush()
        self._tick_index += 1

    def destroy_node(self) -> None:
        try:
            if not self._csv_file.closed:
                self._csv_file.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveV2_14InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Multi-modal synchronized observation logger for peg-in-hole experiments.

Subscribes to the canonical D405 RGB-D topics, the joint-state broadcaster,
and the F/T bridge topic, and writes one synchronized sample row per tick to
``multimodal_observation_log.csv`` in the configured output directory. The
RGB-D image is downsampled and serialized as a base64 PNG to keep the CSV
self-contained for offline learning.

The logger does not publish any commands. It is a passive observer; it does
not alter the JTC, the contact sensor, or the safety monitor.

Output schema (one row per tick):
    stamp_s, tick_index,
    joint_1_pos, ..., joint_6_pos,
    joint_1_vel, ..., joint_6_vel,
    ft_x, ft_y, ft_z, ft_rx, ft_ry, ft_rz,
    rgb_w, rgb_h, rgb_b64_png, depth_w, depth_h, depth_min_m, depth_max_m,
    depth_roi_min_m, depth_roi_max_m,
    task_phase, safety_status
"""

from __future__ import annotations

import base64
import csv
import io
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String

try:
    from geometry_msgs.msg import WrenchStamped
except ImportError:  # pragma: no cover
    WrenchStamped = None


RGB_W_H = 64  # downsample to 64x48 for compact logging
DEPTH_W_H = 64
DEPTH_ROI_XY = (0, 0)  # region of interest for the peg/hole workspace
DEPTH_ROI_WH = (848, 480)


class MultimodalObservationLogger(Node):
    """Synchronized multi-modal observation logger (passive, read-only)."""

    def __init__(self) -> None:
        super().__init__("multimodal_observation_logger")
        self.declare_parameter("output_dir", "diagnostics/multimodal_observation_log")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("log_rgb", True)
        self.declare_parameter("log_depth", True)
        self.declare_parameter("rgb_topic", "/d405/color/image_raw")
        self.declare_parameter("depth_topic", "/d405/depth/image_rect_raw")
        self.declare_parameter("rgb_info_topic", "/d405/color/camera_info")
        self.declare_parameter("depth_info_topic", "/d405/depth/camera_info")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("ft_topic", "/ft_sensor_wrench")
        self.declare_parameter("task_phase_topic", "/task_phase")
        self.declare_parameter("safety_status_topic", "/safety_status")
        self.declare_parameter("csv_basename", "multimodal_observation_log.csv")

        self._output_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        ).expanduser()
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._log_rgb = bool(self.get_parameter("log_rgb").value)
        self._log_depth = bool(self.get_parameter("log_depth").value)
        self._csv_path = self._output_dir / (
            self.get_parameter("csv_basename").get_parameter_value().string_value
        )

        self._bridge = CvBridge()
        self._joint_state: JointState | None = None
        self._ft: np.ndarray = np.zeros(6)
        self._latest_rgb: Image | None = None
        self._latest_depth: Image | None = None
        self._task_phase: str = "UNKNOWN"
        self._safety_status: str = "UNKNOWN"

        self.create_subscription(
            JointState, self.get_parameter("joint_state_topic").value,
            self._on_joint_state, 10,
        )
        self.create_subscription(
            WrenchStamped, self.get_parameter("ft_topic").value,
            self._on_ft, 10,
        )
        if self._log_rgb:
            self.create_subscription(
                Image, self.get_parameter("rgb_topic").value,
                self._on_rgb, qos_profile_sensor_data,
            )
        if self._log_depth:
            self.create_subscription(
                Image, self.get_parameter("depth_topic").value,
                self._on_depth, qos_profile_sensor_data,
            )
        self.create_subscription(
            String, self.get_parameter("task_phase_topic").value,
            self._on_task_phase, 10,
        )
        self.create_subscription(
            String, self.get_parameter("safety_status_topic").value,
            lambda msg: setattr(self, "_safety_status", msg.data), 10,
        )
        self._last_forced_phase: str = ""

        self._tick_index = 0
        self._start_time = self.get_clock().now()
        self._wall_start_s = time.time()
        self._first_joint_state_s: float | None = None
        self._first_task_phase_s: float | None = None
        self._first_rgb_s: float | None = None
        self._first_depth_s: float | None = None
        self._first_tick_s: float | None = None
        self._last_tick_s: float | None = None

        self._csv_file = open(self._csv_path, "w", newline="")
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(self._csv_header())
        self._csv_file.flush()

        period_s = 1.0 / max(1e-3, self._rate_hz)
        self._timer = self.create_timer(period_s, self._on_tick)
        self.get_logger().info(
            f"multimodal_observation_logger writing to {self._csv_path} at "
            f"{self._rate_hz} Hz"
        )

    def _csv_header(self) -> list[str]:
        cols: list[str] = ["stamp_s", "tick_index"]
        for i in range(1, 7):
            cols.append(f"joint_{i}_pos_rad")
        for i in range(1, 7):
            cols.append(f"joint_{i}_vel_rad_s")
        cols += ["ft_x_n", "ft_y_n", "ft_z_n", "ft_rx_nm", "ft_ry_nm", "ft_rz_nm"]
        if self._log_rgb:
            cols += ["rgb_w", "rgb_h", "rgb_b64_png"]
        if self._log_depth:
            cols += [
                "depth_w", "depth_h",
                "depth_min_m", "depth_max_m",
                "depth_roi_min_m", "depth_roi_max_m",
            ]
        cols += ["task_phase", "safety_status"]
        return cols

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_state = msg
        if self._first_joint_state_s is None:
            self._first_joint_state_s = time.time()

    def _on_ft(self, msg: Any) -> None:
        if WrenchStamped is not None and isinstance(msg, WrenchStamped):
            self._ft = np.array([
                msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
            ])
        else:
            self._ft = np.array([msg.x, msg.y, msg.z, 0.0, 0.0, 0.0])

    def _on_rgb(self, msg: Image) -> None:
        self._latest_rgb = msg
        if self._first_rgb_s is None:
            self._first_rgb_s = time.time()

    def _on_depth(self, msg: Image) -> None:
        self._latest_depth = msg
        if self._first_depth_s is None:
            self._first_depth_s = time.time()

    def _encode_rgb(self, msg: Image) -> tuple[int, int, str]:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return (0, 0, "")
        h, w = img.shape[:2]
        scale = min(RGB_W_H / w, RGB_W_H / h)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        small = img[:: max(1, h // new_h), :: max(1, w // new_w)][:new_h, :new_w]
        try:
            from PIL import Image as PILImage
            pil = PILImage.fromarray(small[:, :, ::-1])
            buf = io.BytesIO()
            pil.save(buf, format="PNG", optimize=True)
            return (new_w, new_h, base64.b64encode(buf.getvalue()).decode("ascii"))
        except Exception:
            return (new_w, new_h, "")

    def _summarize_depth(self, msg: Image) -> tuple[int, int, float, float, float, float]:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:
            return (0, 0, 0.0, 0.0, 0.0, 0.0)
        depth = np.asarray(depth, dtype=np.float32)
        if depth.size == 0:
            return (0, 0, 0.0, 0.0, 0.0, 0.0)
        valid = depth[depth > 0]
        if valid.size == 0:
            d_min = 0.0
            d_max = 0.0
        else:
            d_min = float(np.min(valid))
            d_max = float(np.max(valid))
        h, w = depth.shape[:2]
        rx, ry = DEPTH_ROI_XY
        rw, rh = DEPTH_ROI_WH
        rx = max(0, min(w - 1, rx))
        ry = max(0, min(h - 1, ry))
        rw = max(1, min(w - rx, rw))
        rh = max(1, min(h - ry, rh))
        roi = depth[ry : ry + rh, rx : rx + rw]
        roi_valid = roi[roi > 0]
        if roi_valid.size == 0:
            roi_min = 0.0
            roi_max = 0.0
        else:
            roi_min = float(np.min(roi_valid))
            roi_max = float(np.max(roi_valid))
        return (w, h, d_min, d_max, roi_min, roi_max)

    def _on_task_phase(self, msg: Any) -> None:
        new_phase = str(msg.data)
        old_phase = self._task_phase
        self._task_phase = new_phase
        if self._first_task_phase_s is None:
            self._first_task_phase_s = time.time()
        if new_phase in ('RETREAT', 'DONE', 'ABORT') and new_phase != old_phase:
            if new_phase != self._last_forced_phase:
                self._last_forced_phase = new_phase
                self._on_tick()

    def _on_tick(self) -> None:
        js = self._joint_state
        if js is None or len(js.position) < 6:
            return
        now = time.time()
        if self._first_tick_s is None:
            self._first_tick_s = now
        self._last_tick_s = now
        pos = list(js.position[:6]) + [0.0] * (6 - len(js.position[:6]))
        vel = list(js.velocity[:6]) if js.velocity else [0.0] * 6
        vel = vel + [0.0] * (6 - len(vel))
        ft = self._ft
        row: list[Any] = [
            time.time() - self._start_time.nanoseconds * 1e-9,
            self._tick_index,
        ]
        row += [f"{v:.6f}" for v in pos]
        row += [f"{v:.6f}" for v in vel]
        row += [f"{v:.6f}" for v in ft]
        if self._log_rgb:
            if self._latest_rgb is not None:
                w, h, b64 = self._encode_rgb(self._latest_rgb)
                row += [w, h, b64]
            else:
                row += [0, 0, ""]
        if self._log_depth:
            if self._latest_depth is not None:
                row += list(self._summarize_depth(self._latest_depth))
            else:
                row += [0, 0, 0.0, 0.0, 0.0, 0.0]
        row += [self._task_phase, self._safety_status]
        self._writer.writerow(row)
        self._csv_file.flush()
        self._tick_index += 1

    def _write_diagnostic_json(self) -> None:
        """Write a companion diagnostic JSON with startup/shutdown status."""
        wall_now = time.time()
        diag = {
            "csv_path": str(self._csv_path),
            "total_rows_written": self._tick_index,
            "empty_log": self._tick_index <= 1,
            "wall_start_s": self._wall_start_s,
            "wall_end_s": wall_now,
            "wall_elapsed_s": round(wall_now - self._wall_start_s, 3),
            "first_joint_state_s": (
                round(self._first_joint_state_s - self._wall_start_s, 3)
                if self._first_joint_state_s else None
            ),
            "first_task_phase_s": (
                round(self._first_task_phase_s - self._wall_start_s, 3)
                if self._first_task_phase_s else None
            ),
            "first_rgb_s": (
                round(self._first_rgb_s - self._wall_start_s, 3)
                if self._first_rgb_s else None
            ),
            "first_depth_s": (
                round(self._first_depth_s - self._wall_start_s, 3)
                if self._first_depth_s else None
            ),
            "first_tick_s": (
                round(self._first_tick_s - self._wall_start_s, 3)
                if self._first_tick_s else None
            ),
            "last_tick_s": (
                round(self._last_tick_s - self._wall_start_s, 3)
                if self._last_tick_s else None
            ),
            "subscriber_status": {
                "joint_state_received": self._first_joint_state_s is not None,
                "task_phase_received": self._first_task_phase_s is not None,
                "rgb_received": self._first_rgb_s is not None,
                "depth_received": self._first_depth_s is not None,
            },
        }
        diag_path = self._output_dir / "logger_diagnostic.json"
        try:
            diag_path.write_text(json.dumps(diag, indent=2), encoding="utf-8")
        except Exception as exc:
            self.get_logger().warning(f"Failed to write diagnostic JSON: {exc}")

    def destroy_node(self) -> None:
        try:
            self._write_diagnostic_json()
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


def main(args: list[str] | None = None) -> None:
    import signal
    import sys
    rclpy.init(args=args)
    node = MultimodalObservationLogger()

    def _shutdown_handler(signum, frame):
        node.get_logger().info("Shutdown signal received; flushing CSV.")
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _shutdown_handler)
    signal.signal(signal.SIGINT, _shutdown_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

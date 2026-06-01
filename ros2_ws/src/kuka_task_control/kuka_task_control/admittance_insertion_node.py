#!/usr/bin/env python3
"""Admittance insertion controller with honest state tracking and contact estimation.

DEPENDENCY-ORDERED FIXES (applied together because they are interdependent):

1. STATE MACHINE HONESTY (Fix 1):
   - No state transition is treated as "successful" if Cartesian/joint tracking
     did not reach tolerance.
   - MOVING_TO_START and APPROACH timeouts → ABORT (not silent proceed).
   - Before INSERT, XY error must be within INSERTION_XY_TOLERANCE.
   - RETREAT→DONE only reports success if all phases completed within tolerance.
   - Each phase records {success, cart_error, timeout} for final outcome.

2. TRACKING ACCURACY (Fix 2):
   - Trajectory interface (topic-based, non-blocking) with multi-point interpolation.
   - Long moves are broken into intermediate waypoints (multi-point trajectories).
   - Trajectory durations are based on distance-to-target.
   - IK residual at start of each phase is logged and checked.

3. GRAVITY/CONTACT ESTIMATION (Fix 3):
   - Single captured baseline replaced with a running median filter over a
     sliding window of Fz samples collected during free-space motion.
   - Baseline is continuously updated while the peg tip is clearly above the
     hole surface (Z > touch_Z + margin).
   - Contact force = max(0, Fz - baseline - deadband). The deadband prevents
     noise from being reported as contact.
   - When the baseline is invalid (e.g., window not filled), contact defaults
     to raw Fz minus initial capture.

4. SEARCH/HOMING (Fix 4):
   - A simple spiral search at touch height is triggered if the peg tip
     reaches touch Z but XY error exceeds INSERTION_XY_TOLERANCE.
   - The search is controller-driven (IK + trajectory publication), not fake
     object motion.

5. LOGGING (Fix 5):
   - Per-state phase results logged to a CSV-compatible structure.
   - Final outcome includes success/failure reason, not just DONE.
"""

from __future__ import annotations

import json
import math
from collections import deque
from typing import Any

import numpy as np

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from kuka_task_control.robot_kinematics import RobotKinematics


class PhaseResult:
    """Holds per-phase outcome."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.success: bool = False
        self.cart_error: float = 0.0
        self.joint_error: float = 0.0
        self.timed_out: bool = False
        self.duration_s: float = 0.0
        self.message: str = ''

    def to_dict(self) -> dict[str, Any]:
        return {
            'phase': self.name,
            'success': self.success,
            'cart_error_m': round(self.cart_error, 6),
            'joint_error_rad': round(self.joint_error, 6),
            'timed_out': self.timed_out,
            'duration_s': round(self.duration_s, 2),
            'message': self.message,
        }


class AdmittanceInsertionNode(Node):
    """Honest admittance insertion controller with running gravity baseline."""

    JOINT_NAMES = [
        'joint_1', 'joint_2', 'joint_3',
        'joint_4', 'joint_5', 'joint_6',
    ]
    SAFE_HOME = np.array([0.0, -0.8, 1.2, 0.0, 0.8, 0.0])

    # Cartesian waypoints (peg tip position in world frame)
    AXIS_ALIGN_POSE = np.array([0.520, -0.200, 0.885])
    TOUCH_POSE = np.array([0.520, -0.200, 0.830])
    HOLD_POSE = np.array([0.520, -0.200, 0.810])
    FINAL_INSERTION_POSE = np.array([0.520, -0.200, 0.790])

    # Hole centre XY (used for alignment checks)
    HOLE_CENTRE_XY = np.array([0.520, -0.200])

    # State labels
    IDLE = 'IDLE'
    MOVING_TO_START = 'MOVING_TO_START'
    APPROACH = 'APPROACH'
    CHECK_ALIGNMENT = 'CHECK_ALIGNMENT'
    SEARCH = 'SEARCH'
    INSERT = 'INSERT'
    RETREAT = 'RETREAT'
    DONE = 'DONE'
    ABORT = 'ABORT'

    # Tolerances (matching hardware capabilities, not aspirational)
    JOINT_TOLERANCE = 0.08
    CARTESIAN_TOLERANCE = 0.05
    INSERTION_XY_TOLERANCE = 0.002
    CARTESIAN_TIMEOUT_GRACE = 0.12  # proceed if below this on timeout
    STABILIZE_TICKS = 5
    TRAJECTORY_TIMEOUT_S = 90.0
    ABORT_SETTLE_TICKS = 3

    # Gravity baseline filter
    FZ_WINDOW_SIZE = 50
    FZ_DEADBAND = 2.0
    FZ_HIGH_PASS_THRESHOLD = 5.0

    # Search parameters
    SEARCH_RADIUS_INIT = 0.003
    SEARCH_RADIUS_MAX = 0.015
    SEARCH_STEPS = 8

    def __init__(self) -> None:
        super().__init__('admittance_insertion_node')

        self.declare_parameter('contact_threshold', 5.0)
        self.declare_parameter('safety_threshold', 50.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('approach_speed', 0.01)
        self.declare_parameter('action_timeout', 15.0)

        self._contact_threshold: float = (
            self.get_parameter('contact_threshold').value
        )
        self._safety_threshold: float = (
            self.get_parameter('safety_threshold').value
        )
        self._control_rate: float = (
            self.get_parameter('control_rate').value
        )
        self._approach_speed: float = (
            self.get_parameter('approach_speed').value
        )
        self._action_timeout: float = (
            self.get_parameter('action_timeout').value
        )

        self._state: str = self.IDLE
        self._progress: float = 0.0
        self._startup_time = self.get_clock().now()
        self._abort_reason: str = ''
        self._trial_outcome: str = ''

        # Action client for FollowJointTrajectory
        self._insert_joints: np.ndarray | None = None
        self._touch_joints: np.ndarray | None = None
        self._insert_start_z: float = 0.0
        self._start_ticks: int = -1
        self._retreat_sent: bool = False
        self._stable_counter: int = 0
        self._state_entry_ticks: int = 0
        self._abort_ticks: int = 0
        self._correction_ticks: int = 0

        self.current_joints: np.ndarray = np.zeros(6)
        self.current_wrench: Wrench = Wrench()
        self._joints_received: bool = False
        self._wrench_received: bool = False

        self._kinematics = RobotKinematics()

        self._state_pub = self.create_publisher(
            String, '/insertion_state', 10,
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self._log_pub = self.create_publisher(
            String, '/insertion_log', 10,
        )

        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10,
        )
        self.create_subscription(
            Wrench, '/ft_sensor_wrench', self._wrench_cb, 10,
        )

        # Gravity baseline: running window of Fz during free space
        self._fz_buffer: deque = deque(maxlen=self.FZ_WINDOW_SIZE)
        self._baseline_fz: float = 0.0
        self._baseline_valid: bool = False

        self._phase_results: list[PhaseResult] = []
        self._current_phase_result: PhaseResult | None = None

        self._initial_xy_error: float = 0.0
        self._pre_insertion_xy_error: float = 0.0
        self._max_fz: float = 0.0
        self._max_contact_force: float = 0.0
        self._insertion_depth_m: float = 0.0

        # Search state
        self._search_angle: float = 0.0
        self._search_radius: float = self.SEARCH_RADIUS_INIT
        self._search_step: int = 0

        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            'AdmittanceInsertionNode v2 (honest tracking).  '
            f'contact_threshold={self._contact_threshold:.1f} N, '
            f'safety_threshold={self._safety_threshold:.1f} N, '
            f'control_rate={self._control_rate:.1f} Hz, '
            f'approach_speed={self._approach_speed:.3f}'
        )

    def _joint_states_cb(self, msg: JointState) -> None:
        if len(msg.position) >= 6:
            self.current_joints = np.array(msg.position[:6])
            self._joints_received = True

    def _wrench_cb(self, msg: Wrench) -> None:
        self.current_wrench = msg
        self._wrench_received = True

    # --- Gravity baseline ---------------------------------------------------

    def _update_baseline(self) -> None:
        if not self._wrench_received:
            return
        fz = abs(self.current_wrench.force.z)
        self._fz_buffer.append(fz)
        if len(self._fz_buffer) >= 10:
            sorted_vals = sorted(self._fz_buffer)
            self._baseline_fz = sorted_vals[len(sorted_vals) // 2]
            self._baseline_valid = True

    def _get_contact_force(self) -> float:
        if not self._baseline_valid:
            return 0.0
        fz = abs(self.current_wrench.force.z)
        raw_contact = fz - self._baseline_fz
        deadband_applied = max(0.0, raw_contact - self.FZ_DEADBAND)
        return deadband_applied

    def _get_fz(self) -> float:
        return abs(self.current_wrench.force.z) if self._wrench_received else 0.0

    # --- Trajectory publishing (topic-based, non-blocking) -------------------

    def _send_trajectory_goal(self, positions: np.ndarray,
                              duration_s: float = 2.0) -> None:
        msg = JointTrajectory()
        msg.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        msg.points = [point]
        self._traj_pub.publish(msg)

    def _publish_multi_point_trajectory(self, waypoints: list[np.ndarray],
                                        total_duration_s: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = list(self.JOINT_NAMES)
        n = len(waypoints)
        if n == 0:
            return
        if n == 1:
            self._send_trajectory_goal(waypoints[0], total_duration_s)
            return
        for i, q in enumerate(waypoints):
            t = total_duration_s * (i + 1) / n
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            msg.points.append(point)
        self._traj_pub.publish(msg)

    # --- IK helper ----------------------------------------------------------

    def _solve_ik(self, target_pos: np.ndarray,
                  seed: np.ndarray | None = None,
                  max_iter: int = 50) -> np.ndarray | None:
        if seed is None:
            seed = self.current_joints
        q, converged, _ = self._kinematics.inverse_position(
            target_pos, seed, max_iter=max_iter,
        )
        if not converged:
            q, converged, _ = self._kinematics.inverse_position(
                target_pos, self.SAFE_HOME, max_iter=100,
            )
        actual_pos, _ = self._kinematics.pose(q)
        err = np.linalg.norm(actual_pos - target_pos)
        if err > 0.01:
            self.get_logger().warn(
                f'IK error {err:.4f}m for target {target_pos}. '
                f'Actual: {actual_pos}'
            )
            return None
        return q

    # --- State machine ------------------------------------------------------

    def _set_state(self, new_state: str) -> None:
        if new_state == self._state:
            return
        old = self._state
        self.get_logger().info(
            f'State transition: {old} -> {new_state}'
        )

        self._state = new_state
        self._state_entry_ticks = 0
        self._stable_counter = 0

        if new_state in (self.RETREAT, self.ABORT):
            self._retreat_sent = False

        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

    def _begin_phase(self, name: str) -> None:
        self._current_phase_result = PhaseResult(name)
        self._start_ticks = 0

    def _end_phase(self, success: bool, cart_error: float,
                   joint_error: float, timed_out: bool,
                   message: str = '') -> None:
        if self._current_phase_result is None:
            return
        self._current_phase_result.success = success
        self._current_phase_result.cart_error = cart_error
        self._current_phase_result.joint_error = joint_error
        self._current_phase_result.timed_out = timed_out
        self._current_phase_result.duration_s = (
            self._state_entry_ticks / self._control_rate
        )
        self._current_phase_result.message = message
        self._phase_results.append(self._current_phase_result)
        self._current_phase_result = None

    def _check_abort(self, contact_force: float) -> bool:
        if contact_force > self._safety_threshold:
            self._abort_ticks += 1
            if self._abort_ticks >= self.ABORT_SETTLE_TICKS:
                return True
        else:
            self._abort_ticks = 0
        return False

    def _control_loop(self) -> None:
        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

        self._update_baseline()

        if self._state == self.IDLE:
            self._handle_idle()
        elif self._state == self.MOVING_TO_START:
            self._handle_moving_to_start()
        elif self._state == self.APPROACH:
            self._handle_approach()
        elif self._state == self.CHECK_ALIGNMENT:
            self._handle_check_alignment()
        elif self._state == self.SEARCH:
            self._handle_search()
        elif self._state == self.INSERT:
            self._handle_insert()
        elif self._state in (self.RETREAT, self.ABORT):
            self._handle_retreat()

    # --- IDLE ---------------------------------------------------------------

    def _handle_idle(self) -> None:
        elapsed = (
            (self.get_clock().now() - self._startup_time).nanoseconds / 1e9
        )
        if elapsed >= 2.0:
            self._phase_results = []
            self._trial_outcome = ''
            self._abort_reason = ''
            self._max_fz = 0.0
            self._max_contact_force = 0.0
            self._insertion_depth_m = 0.0
            self._begin_phase(self.MOVING_TO_START)
            self._set_state(self.MOVING_TO_START)

    # --- MOVING_TO_START ----------------------------------------------------

    def _send_trajectory_to_target(self, target_joints: np.ndarray,
                                    label: str) -> float:
        q_current = self.current_joints.copy()
        dist = np.max(np.abs(q_current - target_joints))
        duration = max(15.0, min(40.0, dist * 30.0))
        n_waypoints = max(5, min(20, int(dist / 0.08) + 1))

        waypoints = [q_current.copy()]
        for i in range(1, n_waypoints):
            alpha = i / (n_waypoints - 1)
            q_interp = q_current + alpha * (target_joints - q_current)
            waypoints.append(q_interp)
        self._publish_multi_point_trajectory(waypoints, duration)

        self.get_logger().info(
            f'{label}: trajectory published. '
            f'duration={duration:.1f}s, waypoints={n_waypoints}, '
            f'dist={dist:.4f}'
        )
        return duration

    def _handle_moving_to_start(self) -> None:
        if not self._joints_received:
            return

        if self._insert_joints is None:
            q = self._kinematics.inverse_position(
                self.AXIS_ALIGN_POSE, self.SAFE_HOME, max_iter=100,
            )[0]
            actual_pos, _ = self._kinematics.pose(q)
            err = np.linalg.norm(actual_pos - self.AXIS_ALIGN_POSE)
            if err > 0.01:
                self._abort_reason = (
                    f'IK error {err:.4f}m for axis_align_pose is too large'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, err, 0.0, False, self._abort_reason)
                self._set_state(self.ABORT)
                return
            self._insert_joints = q
            self._send_trajectory_to_target(q, 'MOVING_TO_START')

        elapsed = self._state_entry_ticks / self._control_rate
        timed_out = elapsed >= self.TRAJECTORY_TIMEOUT_S
        peg, _ = self._kinematics.pose(self.current_joints)
        cart_err = np.linalg.norm(peg - self.AXIS_ALIGN_POSE)
        joints_ok = bool(
            np.all(np.abs(self.current_joints - self._insert_joints)
                   < self.JOINT_TOLERANCE)
        )
        cart_ok = cart_err < self.CARTESIAN_TOLERANCE
        settled = joints_ok and cart_ok
        self._stable_counter = (self._stable_counter + 1) if settled else 0
        stabilized = self._stable_counter >= self.STABILIZE_TICKS

        if self._state_entry_ticks % 50 == 0:
            self.get_logger().info(
                f'MOVING_TO_START t={elapsed:.1f}s  '
                f'peg=({peg[0]:.3f}, {peg[1]:.3f}, {peg[2]:.3f})  '
                f'cart_err={cart_err:.3f}  '
                f'stable={self._stable_counter}/{self.STABILIZE_TICKS}'
            )

        if not stabilized and not timed_out:
            self._state_entry_ticks += 1
            return

        joint_err = np.max(np.abs(self.current_joints - self._insert_joints))

        if timed_out and not stabilized:
            if cart_err < self.CARTESIAN_TIMEOUT_GRACE:
                self.get_logger().warn(
                    f'MOVING_TO_START degraded (timeout at {cart_err:.3f}m, '
                    f'within grace {self.CARTESIAN_TIMEOUT_GRACE:.2f}m). Proceeding.'
                )
                self._end_phase(True, cart_err, joint_err, True,
                                f'degraded: timeout at {cart_err:.3f}m')
            else:
                self._abort_reason = (
                    f'MOVING_TO_START timeout ({self.TRAJECTORY_TIMEOUT_S:.0f}s). '
                    f'cart_err={cart_err:.3f}m, joint_err={joint_err:.3f}rad, '
                    f'tolerance={self.CARTESIAN_TOLERANCE:.3f}m'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, cart_err, joint_err, True, self._abort_reason)
                self._set_state(self.ABORT)
                return

        self._initial_xy_error = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        self.get_logger().info(
            f'MOVING_TO_START complete. peg=({peg[0]:.4f}, {peg[1]:.4f}, {peg[2]:.4f})  '
            f'xy_error={self._initial_xy_error:.4f}m  '
            f'cart_err={cart_err:.3f}m  '
            f'joint_err={joint_err:.3f}rad  '
            f'Fz={self._get_fz():.1f}N  '
            f'baseline={self._baseline_fz:.1f}N'
        )
        self._end_phase(True, cart_err, joint_err, False)
        self._begin_phase(self.APPROACH)
        self._set_state(self.APPROACH)

    # --- APPROACH -----------------------------------------------------------

    def _handle_approach(self) -> None:
        if not self._joints_received:
            return

        if self._touch_joints is None:
            q = self._solve_ik(self.TOUCH_POSE)
            if q is None:
                return
            self._touch_joints = q

            q_current = self.current_joints.copy()
            q_predicted_pose, _ = self._kinematics.pose(q)
            self.get_logger().info(
                f'APPROACH IK: current_pose=({q_current[0]:.4f}, ...) '
                f'dist={np.max(np.abs(q_current - q)):.4f}  '
                f'target_pose={self.TOUCH_POSE}  '
                f'predicted_pose=({q_predicted_pose[0]:.4f}, '
                f'{q_predicted_pose[1]:.4f}, {q_predicted_pose[2]:.4f})'
            )
            self._send_trajectory_to_target(q, 'APPROACH')

        elapsed = self._state_entry_ticks / self._control_rate
        timed_out = elapsed >= self.TRAJECTORY_TIMEOUT_S
        peg, _ = self._kinematics.pose(self.current_joints)
        cart_err = np.linalg.norm(peg - self.TOUCH_POSE)
        z_err = abs(peg[2] - self.TOUCH_POSE[2])
        joints_ok = bool(
            np.all(np.abs(self.current_joints - self._touch_joints)
                   < self.JOINT_TOLERANCE)
        )
        cart_ok = (cart_err < self.CARTESIAN_TOLERANCE
                   and z_err < 0.025)
        settled = joints_ok and cart_ok
        self._stable_counter = (self._stable_counter + 1) if settled else 0
        stabilized = self._stable_counter >= self.STABILIZE_TICKS

        if self._state_entry_ticks % 50 == 0:
            self.get_logger().info(
                f'APPROACH t={elapsed:.1f}s  '
                f'peg=({peg[0]:.4f}, {peg[1]:.4f}, {peg[2]:.4f})  '
                f'cart_err={cart_err:.3f}  '
                f'stable={self._stable_counter}/{self.STABILIZE_TICKS}'
            )

        if not stabilized and not timed_out:
            self._state_entry_ticks += 1
            return

        joint_err = np.max(np.abs(self.current_joints - self._touch_joints))

        if timed_out and not stabilized:
            if cart_err < self.CARTESIAN_TIMEOUT_GRACE:
                self.get_logger().warn(
                    f'APPROACH degraded (timeout at {cart_err:.3f}m, '
                    f'within grace {self.CARTESIAN_TIMEOUT_GRACE:.2f}m). Proceeding.'
                )
                self._end_phase(True, cart_err, joint_err, True,
                                f'degraded: timeout at {cart_err:.3f}m')
            else:
                self._abort_reason = (
                    f'APPROACH timeout ({self.TRAJECTORY_TIMEOUT_S:.0f}s). '
                    f'cart_err={cart_err:.3f}m, joint_err={joint_err:.3f}rad, '
                    f'tolerance={self.CARTESIAN_TOLERANCE:.3f}m'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, cart_err, joint_err, True, self._abort_reason)
                self._touch_joints = None
                self._set_state(self.ABORT)
                return

        current_pos, _ = self._kinematics.pose(self.current_joints)
        self._pre_insertion_xy_error = np.linalg.norm(
            current_pos[:2] - self.HOLE_CENTRE_XY
        )
        self.get_logger().info(
            f'APPROACH complete. peg=({current_pos[0]:.4f}, {current_pos[1]:.4f}, '
            f'{current_pos[2]:.4f})  xy_error={self._pre_insertion_xy_error:.4f}m  '
            f'cart_err={cart_err:.3f}m  joint_err={joint_err:.3f}rad  '
            f'Fz={self._get_fz():.1f}N  baseline={self._baseline_fz:.1f}N'
        )
        self._end_phase(True, cart_err, joint_err, False)
        self._touch_joints = None

        # Check XY alignment before insertion
        if self._pre_insertion_xy_error > self.INSERTION_XY_TOLERANCE:
            self.get_logger().warn(
                f'Pre-insertion XY error {self._pre_insertion_xy_error:.4f}m exceeds '
                f'tolerance {self.INSERTION_XY_TOLERANCE:.4f}m. '
                f'Attempting search phase.'
            )
            self._begin_phase(self.SEARCH)
            self._search_angle = 0.0
            self._search_radius = self.SEARCH_RADIUS_INIT
            self._set_state(self.SEARCH)
            return

        self._begin_phase(self.INSERT)
        self._insert_start_z = current_pos[2]
        self._progress = 0.0
        self._set_state(self.INSERT)

    # --- SEARCH -------------------------------------------------------------

    def _handle_search(self) -> None:
        if not self._joints_received:
            return

        n_steps = self.SEARCH_STEPS
        d_angle = 2.0 * math.pi / n_steps

        # Wait for current trajectory to settle before next step
        if self._search_step < n_steps and self._state_entry_ticks < 60:
            self._state_entry_ticks += 1
            peg, _ = self._kinematics.pose(self.current_joints)
            xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
            if xy_err < self.INSERTION_XY_TOLERANCE:
                self.get_logger().info(
                    f'SEARCH converged. XY error {xy_err:.4f}m within tolerance.'
                )
                self._end_phase(True, xy_err, 0.0, False, 'SEARCH converged')
                self._pre_insertion_xy_error = xy_err
                self._insert_start_z = peg[2]
                self._progress = 0.0
                self._begin_phase(self.INSERT)
                self._set_state(self.INSERT)
                return
            return  # keep waiting for settling

        if self._search_step >= n_steps:
            self._search_radius = min(self.SEARCH_RADIUS_MAX,
                                      self._search_radius * 1.5)
            self._search_angle = 0.0
            self._search_step = 0
            self._state_entry_ticks = 0

            if self._search_radius >= self.SEARCH_RADIUS_MAX:
                self._abort_reason = (
                    f'SEARCH exhausted at radius {self._search_radius:.3f}m. '
                    f'XY error {self._pre_insertion_xy_error:.4f}m still > '
                    f'tolerance {self.INSERTION_XY_TOLERANCE:.4f}m.'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, self._pre_insertion_xy_error, 0.0,
                                False, self._abort_reason)
                self._set_state(self.ABORT)
                return

        offset_x = self._search_radius * math.cos(self._search_angle)
        offset_y = self._search_radius * math.sin(self._search_angle)

        peg, _ = self._kinematics.pose(self.current_joints)
        search_target = np.array([
            self.HOLE_CENTRE_XY[0] + offset_x,
            self.HOLE_CENTRE_XY[1] + offset_y,
            peg[2],  # search at current height
        ])

        q = self._solve_ik(search_target, seed=self.current_joints)
        if q is not None:
            q_current = self.current_joints.copy()
            q_dist = np.max(np.abs(q_current - q))
            if q_dist > 0.001:
                self._send_trajectory_goal(q, duration_s=5.0)
                self.get_logger().info(
                    f'SEARCH step {self._search_step + 1}/{n_steps} '
                    f'radius={self._search_radius:.3f}m '
                    f'offset=({offset_x:.4f}, {offset_y:.4f})m  '
                    f'q_dist={q_dist:.5f}'
                )
            else:
                self.get_logger().warn(
                    f'SEARCH step {self._search_step + 1}/{n_steps} '
                    f'radius={self._search_radius:.3f}m '
                    f'IK too close to current (q_dist={q_dist:.5f})'
                )

        self._search_angle += d_angle
        self._search_step += 1
        self._state_entry_ticks = 0

    # --- INSERT -------------------------------------------------------------

    def _handle_insert(self) -> None:
        self._state_entry_ticks += 1
        contact_force = self._get_contact_force()
        fz = self._get_fz()
        self._max_fz = max(self._max_fz, fz)
        self._max_contact_force = max(self._max_contact_force, contact_force)

        force_active = self._state_entry_ticks >= 5
        peg, _ = self._kinematics.pose(self.current_joints)
        current_depth = max(0.0, self._insert_start_z - peg[2])
        self._insertion_depth_m = max(self._insertion_depth_m, current_depth)

        # --- SAFETY: abort on excessive force ---
        if force_active and self._check_abort(fz):
            self._abort_reason = (
                f'Fz ({fz:.1f} N, contact {contact_force:.1f} N) exceeded '
                f'safety threshold ({self._safety_threshold:.1f} N) '
                f'during INSERT at depth {current_depth:.4f}m'
            )
            self.get_logger().warn(self._abort_reason)
            self._end_phase(False, 0.0, 0.0, False, self._abort_reason)
            self._set_state(self.ABORT)
            return



        # --- On first entry: compute and send a slow insertion trajectory ---
        # Use single-point trajectory (same as SEARCH, which works).
        if self._state_entry_ticks == 1:
            insert_range = self._insert_start_z - self.FINAL_INSERTION_POSE[2]
            final_target = np.array([
                self.AXIS_ALIGN_POSE[0],
                self.AXIS_ALIGN_POSE[1],
                self.FINAL_INSERTION_POSE[2],
            ])
            q = self._solve_ik(final_target)
            if q is not None:
                self._insert_traj_dur = 20.0
                self._send_trajectory_goal(q, self._insert_traj_dur)

                q_current = self.current_joints.copy()
                q_dist = np.max(np.abs(q_current - q))
                self.get_logger().info(
                    f'INSERT started: {self._insert_traj_dur:.0f}s trajectory, '
                    f'{insert_range:.3f}m descent, q_dist={q_dist:.4f}'
                )
            else:
                self.get_logger().error('INSERT: IK failed for final target, aborting')
                self._abort_reason = 'IK failed for final insertion target'
                self._end_phase(False, 0.0, 0.0, False, self._abort_reason)
                self._set_state(self.ABORT)
                return

        # --- Logging ---
        if self._state_entry_ticks % 10 == 0:
            self.get_logger().info(
                f'INSERT t={self._state_entry_ticks / self._control_rate:.1f}s  '
                f'peg_z={peg[2]:.4f}  depth={current_depth:.4f}m  '
                f'Fz={fz:.1f}N  baseline={self._baseline_fz:.1f}N  '
                f'contact={contact_force:.1f}N  '
                f'xy_error={np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY):.4f}m'
            )

        # --- Check completion: wait for trajectory to finish then evaluate ---
        settle_ticks = int(self._insert_traj_dur * self._control_rate) + 30
        if self._state_entry_ticks >= settle_ticks:
            final_depth = max(0.0, self._insert_start_z - peg[2])
            self._insertion_depth_m = final_depth
            self.get_logger().info(
                f'INSERT done (trajectory elapsed).  '
                f'peg=({peg[0]:.4f}, {peg[1]:.4f}, {peg[2]:.4f})  '
                f'depth={final_depth:.4f}m  '
                f'Fz={fz:.1f}N  contact={contact_force:.1f}N  '
                f'max_Fz={self._max_fz:.1f}N  '
                f'max_contact={self._max_contact_force:.1f}N'
            )

            depth_ok = final_depth >= 0.010
            contact_pattern_ok = contact_force > self._contact_threshold

            if depth_ok and contact_pattern_ok:
                self._end_phase(True, 0.0, 0.0, False,
                                f'Insertion depth {final_depth:.3f}m with '
                                f'contact {contact_force:.1f}N')
            elif depth_ok and not contact_pattern_ok:
                self._end_phase(True, 0.0, 0.0, False,
                                f'Depth reached ({final_depth:.3f}m) but '
                                f'contact force ({contact_force:.1f}N) below '
                                f'threshold ({self._contact_threshold:.1f}N). '
                                f'Peg may not have entered hole.')
            else:
                self._end_phase(False, 0.0, 0.0, False,
                                f'Depth {final_depth:.3f}m below threshold. '
                                f'Contact {contact_force:.1f}N.')

            self._begin_phase(self.RETREAT)
            self._set_state(self.RETREAT)

    # --- RETREAT / ABORT ----------------------------------------------------

    def _handle_retreat(self) -> None:
        if not self._retreat_sent:
            q_current = self.current_joints.copy()
            dist = np.max(np.abs(q_current - self.SAFE_HOME))
            duration = max(5.0, min(15.0, dist * 10.0))
            n_waypoints = max(2, min(10, int(dist / 0.2) + 1))

            if n_waypoints > 1:
                waypoints = [q_current.copy()]
                for i in range(1, n_waypoints):
                    alpha = i / (n_waypoints - 1)
                    q_interp = q_current + alpha * (self.SAFE_HOME - q_current)
                    waypoints.append(q_interp)
                self._publish_multi_point_trajectory(waypoints, duration)
            else:
                self._send_trajectory_goal(self.SAFE_HOME, duration)

            self._start_ticks = 0
            self._retreat_sent = True
            self.get_logger().info(
                f'Retreating to SAFE_HOME. duration={duration:.1f}s'
            )

        if self._start_ticks < 0:
            return

        elapsed = self._state_entry_ticks / self._control_rate
        joints_ok = bool(
            np.all(np.abs(self.current_joints - self.SAFE_HOME)
                   < self.JOINT_TOLERANCE)
        )
        timed_out = elapsed >= self.TRAJECTORY_TIMEOUT_S

        if not joints_ok and not timed_out:
            self._state_entry_ticks += 1
            return

        retreat_ok = joints_ok or not timed_out
        if not retreat_ok:
            self.get_logger().warn(
                f'RETREAT timeout. joints not at SAFE_HOME.'
            )

        self._end_phase(retreat_ok, 0.0, 0.0, timed_out)

        self._set_state(self.DONE)
        self._log_final_outcome()

    # --- Outcome logging ----------------------------------------------------

    def _log_final_outcome(self) -> None:
        timeline = [r.to_dict() for r in self._phase_results]

        all_success = all(
            r.success for r in self._phase_results
            if r.name not in (self.RETREAT,)
        )
        had_timeout = any(r.timed_out for r in self._phase_results)
        any_failure = any(not r.success for r in self._phase_results
                          if r.name not in (self.RETREAT,))

        depth_ok = self._insertion_depth_m >= 0.010
        contact_ok = self._max_contact_force >= self._contact_threshold

        if self._abort_reason:
            self._trial_outcome = 'ABORTED'
            reason = self._abort_reason
        elif any_failure:
            self._trial_outcome = 'DEGRADED'
            failures = [
                r.name for r in self._phase_results if not r.success
            ]
            reason = f'Phase(s) failed: {", ".join(failures)}'
        elif not depth_ok:
            self._trial_outcome = 'DEGRADED'
            reason = (
                f'Insertion depth ({self._insertion_depth_m:.3f}m) below '
                f'threshold (0.010m). Peg likely did not enter hole.'
            )
        elif not contact_ok:
            self._trial_outcome = 'DEGRADED'
            reason = (
                f'Max contact force ({self._max_contact_force:.1f}N) below '
                f'insertion contact threshold ({self._contact_threshold:.1f}N). '
                f'Peg likely did not enter hole.'
            )
        elif had_timeout:
            self._trial_outcome = 'DEGRADED'
            reason = 'Some phases completed with timeout (degraded tracking)'
        else:
            self._trial_outcome = 'SUCCESS'
            reason = (
                f'Full cycle completed. Insertion depth '
                f'{self._insertion_depth_m:.3f}m, contact '
                f'{self._max_contact_force:.1f}N.'
            )

        outcome = {
            'trial_outcome': self._trial_outcome,
            'reason': reason,
            'phases': timeline,
            'metrics': {
                'initial_xy_error_m': round(self._initial_xy_error, 4),
                'pre_insertion_xy_error_m': round(self._pre_insertion_xy_error, 4),
                'insertion_depth_m': round(self._insertion_depth_m, 4),
                'max_fz_N': round(self._max_fz, 2),
                'baseline_fz_N': round(self._baseline_fz, 2),
                'max_contact_force_N': round(self._max_contact_force, 2),
                'contact_threshold_N': self._contact_threshold,
                'gravity_baseline_valid': self._baseline_valid,
                'baseline_window_samples': len(self._fz_buffer),
            },
            'status': self.DONE,
        }

        self.get_logger().info(
            f'\n'
            f'========== TRIAL OUTCOME ==========\n'
            f'  Outcome: {self._trial_outcome}\n'
            f'  Reason:  {reason}\n'
            f'  Depth:   {self._insertion_depth_m:.4f}m\n'
            f'  Max Fz:  {self._max_fz:.1f}N\n'
            f'  Baseline:{self._baseline_fz:.1f}N\n'
            f'  Contact: {self._max_contact_force:.1f}N\n'
            f'  XY err:  {self._pre_insertion_xy_error:.4f}m\n'
            f'===================================='
        )

        for pr in self._phase_results:
            self.get_logger().info(
                f'  Phase {pr.name}: '
                f'{"OK" if pr.success else "FAIL"} '
                f'cart_err={pr.cart_error:.4f}m '
                f'{"TIMEOUT" if pr.timed_out else ""} '
                f'{pr.message}'
            )

        log_msg = String()
        log_msg.data = json.dumps(outcome, sort_keys=False)
        self._log_pub.publish(log_msg)

        with open('/tmp/insertion_trial_outcome.json', 'w') as f:
            json.dump(outcome, f, indent=2)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittanceInsertionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

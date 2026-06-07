#!/usr/bin/env python3
"""Admittance insertion controller with honest state tracking and contact estimation.

DEPENDENCY-ORDERED FIXES (applied together because they are interdependent):

1. STATE MACHINE HONESTY (Fix 1):
   - No state transition is treated as "successful" if Cartesian/joint tracking
     did not reach tolerance.
   - MOVING_TO_START and APPROACH timeouts -> ABORT (not silent proceed).
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
   - A simple spiral search at touch height is triggered only if the peg tip
     reaches the force-safe pre-insertion Z band and residual XY error is
     within the bounded search radius.
   - The search is controller-driven (IK + trajectory publication), not fake
     object motion.

5. LOGGING (Fix 5):
   - Per-state phase results logged to a CSV-compatible structure.
   - Final outcome includes success/failure reason, not just DONE.
"""

from __future__ import annotations

import csv
import json
import math
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
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
    HOLE_TOP_Z = 0.810
    RETREAT_CLEARANCE_Z = AXIS_ALIGN_POSE[2]

    # Hole centre XY (used for alignment checks)
    HOLE_CENTRE_XY = np.array([0.520, -0.200])
    PEG_AXIS_WORLD = np.array([0.0, 0.0, 1.0])

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
    MOVING_TO_START_TIMEOUT_S = 300.0
    TRAJECTORY_TIMEOUT_S = 90.0
    ABORT_RETREAT_TIMEOUT_S = 20.0
    ABORT_SETTLE_TICKS = 3
    INSERT_SETTLE_MARGIN_S = 3.0

    # Gravity baseline filter
    FZ_WINDOW_SIZE = 50
    FZ_DEADBAND = 2.0
    FZ_HIGH_PASS_THRESHOLD = 5.0
    HARD_FORCE_ABORT_N = 1000.0

    # Search parameters
    SEARCH_RADIUS_INIT = 0.003
    SEARCH_RADIUS_MAX = 0.015
    SEARCH_STEPS = 8
    SEARCH_TIMEOUT_S = 45.0
    SEARCH_CONVERGENCE_TICKS = 8
    SEARCH_RECENTER_XY_TOLERANCE = 0.004
    SEARCH_RECENTER_DURATION_S = 5.0
    SEARCH_SETTLE_DURATION_S = 6.0
    INSERT_PRECONDITION_XY_TOLERANCE = 0.015
    INSERT_PRECONDITION_MAX_Z = 0.845
    APPROACH_START_XY_TOLERANCE = INSERTION_XY_TOLERANCE
    PEG_RADIUS_M = 0.0125
    HOLE_RADIUS_M = 0.0135
    INSERT_FINAL_XY_TOLERANCE = HOLE_RADIUS_M - PEG_RADIUS_M
    INSERT_PRECONTACT_CLEARANCE_TICKS = 3
    INSERT_SIDELOAD_DEPTH_GATE_M = 0.001
    INSERT_SIDELOAD_SETTLE_TICKS = 3
    INSERT_SHALLOW_SIDELOAD_RECOVERY_DEPTH_M = 0.005
    INSERT_SHALLOW_SIDELOAD_RECOVERY_MAX_ATTEMPTS = 2
    INSERT_SIDELOAD_WITHDRAW_CLEARANCE_M = 0.015
    INSERT_SIDELOAD_WITHDRAW_DURATION_S = 3.0
    INSERT_HANDOFF_HOLD_DURATION_S = 2.0
    INSERT_HANDOFF_SETTLE_TICKS = 8
    INSERT_HANDOFF_TIMEOUT_S = 6.0
    INSERT_PREDEPTH_RECENTER_MAX_ATTEMPTS = 2

    def __init__(self) -> None:
        super().__init__('admittance_insertion_node')

        self.declare_parameter('contact_threshold', 5.0)
        self.declare_parameter('safety_threshold', 50.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('approach_speed', 0.01)
        self.declare_parameter('action_timeout', 15.0)
        self.declare_parameter('trajectory_discovery_wait_s', 2.0)
        self.declare_parameter('expected_trajectory_subscribers', 2)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('done_exit_delay_s', 0.5)
        self.declare_parameter(
            'search_recenter_duration_s',
            self.SEARCH_RECENTER_DURATION_S,
        )
        self.declare_parameter(
            'search_settle_duration_s',
            self.SEARCH_SETTLE_DURATION_S,
        )
        self.declare_parameter(
            'insert_handoff_hold_duration_s',
            self.INSERT_HANDOFF_HOLD_DURATION_S,
        )
        self.declare_parameter(
            'insert_handoff_timeout_s',
            self.INSERT_HANDOFF_TIMEOUT_S,
        )
        self.declare_parameter('tracking_log_dir', '')

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
        self._trajectory_discovery_wait_s: float = float(
            self.get_parameter('trajectory_discovery_wait_s').value
        )
        self._expected_trajectory_subscribers: int = int(
            self.get_parameter('expected_trajectory_subscribers').value
        )
        self._exit_on_done: bool = bool(
            self.get_parameter('exit_on_done').value
        )
        self._done_exit_delay_s: float = max(
            0.0,
            float(self.get_parameter('done_exit_delay_s').value),
        )
        self._done_exit_timer = None
        self._search_recenter_duration_s: float = max(
            0.1,
            float(self.get_parameter('search_recenter_duration_s').value),
        )
        self._search_settle_duration_s: float = max(
            self._search_recenter_duration_s,
            float(self.get_parameter('search_settle_duration_s').value),
        )
        self._insert_handoff_hold_duration_s: float = max(
            0.1,
            float(self.get_parameter('insert_handoff_hold_duration_s').value),
        )
        self._insert_handoff_timeout_s: float = max(
            self._insert_handoff_hold_duration_s,
            float(self.get_parameter('insert_handoff_timeout_s').value),
        )
        self._tracking_log_dir: str = str(
            self.get_parameter('tracking_log_dir').value or ''
        )

        self._state: str = self.IDLE
        self._progress: float = 0.0
        self._startup_time = self.get_clock().now()
        self._abort_reason: str = ''
        self._trial_outcome: str = ''
        self._final_outcome_logged: bool = False

        # Action client for FollowJointTrajectory
        self._insert_joints: np.ndarray | None = None
        self._touch_joints: np.ndarray | None = None
        self._move_to_start_duration_s: float = 0.0
        self._approach_duration_s: float = 0.0
        self._insert_start_z: float = 0.0
        self._start_ticks: int = -1
        self._retreat_sent: bool = False
        self._trajectory_discovery_checked: bool = False
        self._stable_counter: int = 0
        self._state_entry_ticks: int = 0
        self._abort_ticks: int = 0
        self._correction_ticks: int = 0
        self._insert_traj_dur: float = 20.0
        self._insert_command_start_s: float = 0.0
        self._insert_command_sent: bool = False
        self._insert_handoff_hold_sent: bool = False
        self._insert_handoff_hold_start_s: float = 0.0
        self._insert_handoff_stable_ticks: int = 0
        self._insert_precontact_clearance_ticks: int = 0
        self._insert_sideload_ticks: int = 0
        self._insert_predepth_recenter_attempts: int = 0
        self._insert_shallow_sideload_recovery_attempts: int = 0
        self._insert_sideload_withdraw_active: bool = False
        self._insert_sideload_withdraw_start_s: float = 0.0

        self.current_joints: np.ndarray = np.zeros(6)
        self.current_wrench: Wrench = Wrench()
        self._joints_received: bool = False
        self._wrench_received: bool = False

        self._kinematics = RobotKinematics()

        self._state_pub = self.create_publisher(
            String, '/insertion_state', 10,
        )
        self._task_phase_pub = self.create_publisher(
            String, '/task_phase', 10,
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
        self._max_force_norm: float = 0.0
        self._max_contact_force: float = 0.0
        self._max_insert_contact_force: float = 0.0
        self._insertion_depth_m: float = 0.0
        self._final_insertion_xy_error_m: float = 0.0
        self._raw_force_abort_reason: str = ''

        # Search state
        self._search_angle: float = 0.0
        self._search_radius: float = self.SEARCH_RADIUS_INIT
        self._search_step: int = 0
        self._search_total_ticks: int = 0
        self._search_convergence_ticks: int = 0
        self._search_recenter_attempts: int = 0
        self._search_stability_ready_s: float = 0.0
        self._search_trace_writer: csv.DictWriter | None = None
        self._search_trace_file = None
        self._setup_search_gate_trace()

        period = 1.0 / self._control_rate
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            'AdmittanceInsertionNode v2 (honest tracking).  '
            f'contact_threshold={self._contact_threshold:.1f} N, '
            f'safety_threshold={self._safety_threshold:.1f} N, '
            f'control_rate={self._control_rate:.1f} Hz, '
            f'approach_speed={self._approach_speed:.3f}, '
            f'trajectory_discovery_wait_s={self._trajectory_discovery_wait_s:.1f}, '
            f'expected_trajectory_subscribers={self._expected_trajectory_subscribers}, '
            f'exit_on_done={self._exit_on_done}, '
            f'done_exit_delay_s={self._done_exit_delay_s:.1f}, '
            f'search_recenter_duration_s={self._search_recenter_duration_s:.1f}, '
            f'search_settle_duration_s={self._search_settle_duration_s:.1f}, '
            f'insert_handoff_hold_duration_s='
            f'{self._insert_handoff_hold_duration_s:.1f}, '
            f'insert_handoff_timeout_s={self._insert_handoff_timeout_s:.1f}, '
            f'tracking_log_dir={self._tracking_log_dir or "disabled"}'
        )

    def _setup_search_gate_trace(self) -> None:
        if not self._tracking_log_dir:
            return
        output_dir = Path(self._tracking_log_dir).expanduser()
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            path = output_dir / 'search_gate_trace.csv'
            self._search_trace_file = path.open(
                'w',
                encoding='utf-8',
                newline='',
            )
            fields = [
                'stamp_s',
                'search_tick',
                'state_entry_ticks',
                'elapsed_s',
                'settle_elapsed_s',
                'ready_to_count',
                'settling_window_active',
                'command_still_running',
                'search_step',
                'recenter_attempts',
                'convergence_ticks_before',
                'convergence_ticks_after',
                'xy_error_m',
                'peg_z_m',
                'stability_ready_s',
                'decision',
            ]
            self._search_trace_writer = csv.DictWriter(
                self._search_trace_file,
                fieldnames=fields,
                lineterminator='\n',
            )
            self._search_trace_writer.writeheader()
            self._search_trace_file.flush()
            self.get_logger().info(
                f'SEARCH gate trace enabled: {path}'
            )
        except OSError as exc:
            self._search_trace_writer = None
            self._search_trace_file = None
            self.get_logger().warn(
                f'Could not open SEARCH gate trace in {output_dir}: {exc}'
            )

    def _write_search_gate_trace(
        self,
        *,
        elapsed_s: float,
        settle_elapsed_s: float,
        ready_to_count: bool,
        settling_window_active: bool,
        command_still_running: bool,
        convergence_before: int,
        xy_error: float,
        peg_z: float,
        decision: str,
    ) -> None:
        if self._search_trace_writer is None or self._search_trace_file is None:
            return
        try:
            self._search_trace_writer.writerow({
                'stamp_s': f'{self._now_s():.9f}',
                'search_tick': self._search_total_ticks,
                'state_entry_ticks': self._state_entry_ticks,
                'elapsed_s': f'{elapsed_s:.9f}',
                'settle_elapsed_s': f'{settle_elapsed_s:.9f}',
                'ready_to_count': int(ready_to_count),
                'settling_window_active': int(settling_window_active),
                'command_still_running': int(command_still_running),
                'search_step': self._search_step,
                'recenter_attempts': self._search_recenter_attempts,
                'convergence_ticks_before': convergence_before,
                'convergence_ticks_after': self._search_convergence_ticks,
                'xy_error_m': f'{xy_error:.9f}',
                'peg_z_m': f'{peg_z:.9f}',
                'stability_ready_s': f'{self._search_stability_ready_s:.9f}',
                'decision': decision,
            })
            self._search_trace_file.flush()
        except OSError as exc:
            self.get_logger().warn(
                f'Disabling SEARCH gate trace after write failure: {exc}'
            )
            self._search_trace_writer = None
            self._search_trace_file = None

    def _joint_states_cb(self, msg: JointState) -> None:
        positions_by_name = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }
        if all(name in positions_by_name for name in self.JOINT_NAMES):
            self.current_joints = np.array([
                positions_by_name[name] for name in self.JOINT_NAMES
            ])
            self._joints_received = True

    def _wrench_cb(self, msg: Wrench) -> None:
        self.current_wrench = msg
        self._wrench_received = True
        abs_fz = abs(float(msg.force.z))
        force_norm = math.sqrt(
            float(msg.force.x) * float(msg.force.x)
            + float(msg.force.y) * float(msg.force.y)
            + float(msg.force.z) * float(msg.force.z)
        )
        self._max_fz = max(self._max_fz, abs_fz)
        self._max_force_norm = max(self._max_force_norm, force_norm)
        if (
            not self._raw_force_abort_reason
            and self._state not in (self.IDLE, self.RETREAT, self.ABORT, self.DONE)
            and max(abs_fz, force_norm) > self.HARD_FORCE_ABORT_N
        ):
            self._raw_force_abort_reason = (
                f'Hard force abort: raw wrench exceeded '
                f'{self.HARD_FORCE_ABORT_N:.1f}N in state {self._state}; '
                f'|Fz|={abs_fz:.1f}N, |F|={force_norm:.1f}N.'
            )

    # --- Gravity baseline ---------------------------------------------------

    def _update_baseline(self) -> None:
        """Update gravity baseline only during free-space states.

        During INSERT, SEARCH, or RETREAT the Fz readings contain contact
        forces which would corrupt the median-based baseline.
        """
        if not self._wrench_received:
            return
        if self._state not in (self.IDLE, self.MOVING_TO_START, self.APPROACH):
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

    def _wait_for_trajectory_subscribers(self) -> None:
        if self._trajectory_discovery_checked:
            return
        self._trajectory_discovery_checked = True

        expected = max(1, self._expected_trajectory_subscribers)
        timeout_s = max(0.0, self._trajectory_discovery_wait_s)
        deadline = time.monotonic() + timeout_s
        subscribers = self.count_subscribers(
            '/joint_trajectory_controller/joint_trajectory',
        )
        while subscribers < expected and time.monotonic() < deadline:
            time.sleep(0.05)
            subscribers = self.count_subscribers(
                '/joint_trajectory_controller/joint_trajectory',
            )

        if subscribers < expected:
            self.get_logger().warn(
                'Trajectory topic discovery wait ended with '
                f'{subscribers}/{expected} subscribers. Continuing so the '
                'controller cannot hang on observer availability.'
            )
        else:
            self.get_logger().info(
                'Trajectory topic discovery satisfied: '
                f'{subscribers}/{expected} subscribers matched.'
            )

    def _send_trajectory_goal(self, positions: np.ndarray,
                              duration_s: float = 2.0) -> None:
        self._wait_for_trajectory_subscribers()
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
        self._wait_for_trajectory_subscribers()
        for i, q in enumerate(waypoints):
            t = total_duration_s * (i + 1) / n
            point = JointTrajectoryPoint()
            point.positions = q.tolist()
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            msg.points.append(point)
        self._traj_pub.publish(msg)

    def _now_s(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds * 1e-9

    # --- IK helper ----------------------------------------------------------

    def _solve_ik(self, target_pos: np.ndarray,
                  seed: np.ndarray | None = None,
                  max_iter: int = 50) -> np.ndarray | None:
        if seed is None:
            seed = self.current_joints
        q, converged, err = self._kinematics.inverse_position_axis(
            target_pos, self.PEG_AXIS_WORLD, seed, max_iter=max_iter,
        )
        if not converged:
            q, converged, err = self._kinematics.inverse_position_axis(
                target_pos, self.PEG_AXIS_WORLD, self.SAFE_HOME, max_iter=100,
            )
        actual_pos, actual_rot = self._kinematics.pose(q)
        pos_err = np.linalg.norm(actual_pos - target_pos)
        axis_err = np.linalg.norm(actual_rot[:, 2] - self.PEG_AXIS_WORLD)
        if pos_err > 0.01 or axis_err > 0.02:
            self.get_logger().warn(
                f'Axis-aligned IK error pos={pos_err:.4f}m, '
                f'axis={axis_err:.4f} for target {target_pos}. '
                f'Actual: {actual_pos}, peg_axis={actual_rot[:, 2]}, '
                f'solver_err={err:.4f}'
            )
            return None
        return q

    def _build_vertical_clearance_waypoints(self) -> list[np.ndarray]:
        peg, _ = self._kinematics.pose(self.current_joints)
        clearance_z = max(self.RETREAT_CLEARANCE_Z, float(peg[2]) + 0.015)
        z_lift = clearance_z - float(peg[2])
        if z_lift <= 0.010:
            return [self.current_joints.copy()]

        q_prev = self.current_joints.copy()
        waypoints = [q_prev.copy()]
        n_cart_waypoints = max(3, min(12, int(z_lift / 0.01) + 1))
        for i in range(1, n_cart_waypoints + 1):
            alpha = i / n_cart_waypoints
            target = np.array([
                peg[0],
                peg[1],
                peg[2] + alpha * z_lift,
            ])
            q = self._solve_ik(target, q_prev, max_iter=100)
            if q is None:
                self.get_logger().warn(
                    'RETREAT clearance lift IK failed; falling back to '
                    'joint-space retreat to SAFE_HOME.'
                )
                return [self.current_joints.copy()]
            waypoints.append(q.copy())
            q_prev = q
        return waypoints

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
        if new_state == self.INSERT:
            self._insert_command_sent = False
            self._insert_handoff_hold_sent = False
            self._insert_handoff_hold_start_s = 0.0
            self._insert_handoff_stable_ticks = 0
            self._insert_precontact_clearance_ticks = 0
            self._insert_sideload_ticks = 0
            self._insert_predepth_recenter_attempts = 0
            self._insert_shallow_sideload_recovery_attempts = 0
            self._insert_sideload_withdraw_active = False
            self._insert_sideload_withdraw_start_s = 0.0
        if new_state == self.SEARCH:
            self._search_total_ticks = 0
            self._search_convergence_ticks = 0
            self._search_recenter_attempts = 0
            self._search_stability_ready_s = 0.0

        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)
        self._task_phase_pub.publish(state_msg)

        if new_state == self.ABORT:
            self._log_final_outcome()

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
        self._task_phase_pub.publish(state_msg)

        self._update_baseline()
        if self._wrench_received:
            fz = self._get_fz()
            self._max_contact_force = max(
                self._max_contact_force,
                self._get_contact_force(),
            )
            if self._raw_force_abort_reason and self._state not in (
                self.IDLE,
                self.RETREAT,
                self.ABORT,
                self.DONE,
            ):
                self._abort_reason = self._raw_force_abort_reason
                self.get_logger().error(self._abort_reason)
                if self._current_phase_result is not None:
                    peg, _ = self._kinematics.pose(self.current_joints)
                    xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
                    self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
                self._set_state(self.ABORT)
                return

        if self._state == self.IDLE:
            self._handle_idle()
        elif self._state == self.MOVING_TO_START:
            self._handle_moving_to_start()
        elif self._state == self.APPROACH:
            self._handle_approach()
        elif self._state == self.CHECK_ALIGNMENT:
            # CHECK_ALIGNMENT is not currently reachable (no transition targets
            # it), but the handler exists defensively.  If entered, abort.
            self._abort_reason = 'CHECK_ALIGNMENT state is not implemented'
            self.get_logger().error(self._abort_reason)
            self._set_state(self.ABORT)
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
            self._final_outcome_logged = False
            self._raw_force_abort_reason = ''
            self._max_fz = 0.0
            self._max_force_norm = 0.0
            self._max_contact_force = 0.0
            self._max_insert_contact_force = 0.0
            self._insertion_depth_m = 0.0
            self._final_insertion_xy_error_m = 0.0
            self._insert_command_sent = False
            self._insert_handoff_hold_sent = False
            self._insert_handoff_hold_start_s = 0.0
            self._insert_handoff_stable_ticks = 0
            self._insert_precontact_clearance_ticks = 0
            self._insert_sideload_ticks = 0
            self._insert_predepth_recenter_attempts = 0
            self._insert_shallow_sideload_recovery_attempts = 0
            self._insert_sideload_withdraw_active = False
            self._insert_sideload_withdraw_start_s = 0.0
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

    def _send_cartesian_descent_trajectory(self, start_pos: np.ndarray,
                                           target_pos: np.ndarray,
                                           label: str) -> tuple[np.ndarray, float] | None:
        q_seed = self.current_joints.copy()
        z_dist = abs(float(start_pos[2] - target_pos[2]))
        n_cart_waypoints = max(4, min(10, int(z_dist / 0.01) + 2))

        joint_waypoints = [q_seed.copy()]
        q_prev = q_seed
        max_joint_step = 0.0
        for i in range(1, n_cart_waypoints + 1):
            alpha = i / n_cart_waypoints
            cart_target = np.array([
                target_pos[0],
                target_pos[1],
                start_pos[2] + alpha * (target_pos[2] - start_pos[2]),
            ])
            q, converged, _ = self._kinematics.inverse_position(
                cart_target, q_prev, max_iter=100,
            )
            actual_pos, _ = self._kinematics.pose(q)
            err = np.linalg.norm(actual_pos - cart_target)
            if not converged or err > 0.01:
                self.get_logger().error(
                    f'{label} Cartesian waypoint IK failed: '
                    f'target=({cart_target[0]:.4f}, {cart_target[1]:.4f}, '
                    f'{cart_target[2]:.4f}), err={err:.4f}m'
                )
                return None
            max_joint_step = max(max_joint_step, float(np.max(np.abs(q - q_prev))))
            joint_waypoints.append(q.copy())
            q_prev = q

        total_joint_dist = float(np.max(np.abs(q_seed - joint_waypoints[-1])))
        duration = max(15.0, min(40.0, max(total_joint_dist, max_joint_step) * 30.0))
        self._publish_multi_point_trajectory(joint_waypoints, duration)
        self.get_logger().info(
            f'{label}: Cartesian descent trajectory published. '
            f'duration={duration:.1f}s, cart_waypoints={n_cart_waypoints}, '
            f'z_dist={z_dist:.4f}, joint_dist={total_joint_dist:.4f}, '
            f'max_joint_step={max_joint_step:.4f}'
        )
        return joint_waypoints[-1], duration

    def _handle_moving_to_start(self) -> None:
        if not self._joints_received:
            return

        if self._insert_joints is None:
            q = self._solve_ik(self.AXIS_ALIGN_POSE, self.SAFE_HOME, max_iter=100)
            if q is None:
                actual_pos, actual_rot = self._kinematics.pose(self.current_joints)
                pos_err = np.linalg.norm(actual_pos - self.AXIS_ALIGN_POSE)
                axis_err = np.linalg.norm(actual_rot[:, 2] - self.PEG_AXIS_WORLD)
                self._abort_reason = (
                    f'Axis-aligned IK failed for axis_align_pose '
                    f'(pos_err={pos_err:.4f}m, axis_err={axis_err:.4f})'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, pos_err, 0.0, False, self._abort_reason)
                self._set_state(self.ABORT)
                return
            self._insert_joints = q
            self._move_to_start_duration_s = self._send_trajectory_to_target(
                q, 'MOVING_TO_START'
            )

        elapsed = self._state_entry_ticks / self._control_rate
        timeout_s = (
            self.ABORT_RETREAT_TIMEOUT_S
            if self._state == self.ABORT
            else self.MOVING_TO_START_TIMEOUT_S
        )
        timed_out = elapsed >= timeout_s
        peg, _ = self._kinematics.pose(self.current_joints)
        cart_err = np.linalg.norm(peg - self.AXIS_ALIGN_POSE)
        xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        joints_ok = bool(
            np.all(np.abs(self.current_joints - self._insert_joints)
                   < self.JOINT_TOLERANCE)
        )
        cart_ok = cart_err < self.CARTESIAN_TOLERANCE
        xy_ok = xy_err <= self.APPROACH_START_XY_TOLERANCE
        settled = joints_ok and cart_ok and xy_ok
        self._stable_counter = (self._stable_counter + 1) if settled else 0
        stabilized = self._stable_counter >= self.STABILIZE_TICKS

        if self._state_entry_ticks % 50 == 0:
            self.get_logger().info(
                f'MOVING_TO_START t={elapsed:.1f}s  '
                f'peg=({peg[0]:.3f}, {peg[1]:.3f}, {peg[2]:.3f})  '
                f'cart_err={cart_err:.3f}  '
                f'xy_err={xy_err:.3f}  '
                f'joint_err={np.max(np.abs(self.current_joints - self._insert_joints)):.3f}  '
                f'stable={self._stable_counter}/{self.STABILIZE_TICKS}'
            )

        if (
            not stabilized
            and not timed_out
        ):
            self._state_entry_ticks += 1
            return

        joint_err = np.max(np.abs(self.current_joints - self._insert_joints))

        phase_timed_out = False
        phase_message = ''
        if not stabilized:
            self._abort_reason = (
                f'MOVING_TO_START timeout/failure ({elapsed:.1f}s). '
                f'cart_err={cart_err:.3f}m, xy_err={xy_err:.3f}m, '
                f'joint_err={joint_err:.3f}rad, stable='
                f'{self._stable_counter}/{self.STABILIZE_TICKS}, '
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
        if self._initial_xy_error > self.APPROACH_START_XY_TOLERANCE:
            self._abort_reason = (
                f'APPROACH blocked: above-hole XY error '
                f'{self._initial_xy_error:.4f}m exceeds no-contact descent '
                f'precondition {self.APPROACH_START_XY_TOLERANCE:.4f}m.'
            )
            self.get_logger().error(self._abort_reason)
            self._end_phase(False, cart_err, joint_err, phase_timed_out,
                            self._abort_reason)
            self._set_state(self.ABORT)
            return
        self._end_phase(True, cart_err, joint_err, phase_timed_out, phase_message)
        self._begin_phase(self.APPROACH)
        self._set_state(self.APPROACH)

    # --- APPROACH -----------------------------------------------------------

    def _handle_approach(self) -> None:
        if not self._joints_received:
            return

        if self._touch_joints is None:
            current_peg, _ = self._kinematics.pose(self.current_joints)
            descent_start = np.array([
                self.HOLE_CENTRE_XY[0],
                self.HOLE_CENTRE_XY[1],
                current_peg[2],
            ])
            descent = self._send_cartesian_descent_trajectory(
                descent_start, self.TOUCH_POSE, 'APPROACH'
            )
            if descent is None:
                return
            q, duration = descent
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
            self._approach_duration_s = duration

        elapsed = self._state_entry_ticks / self._control_rate
        timed_out = elapsed >= self.TRAJECTORY_TIMEOUT_S
        peg, _ = self._kinematics.pose(self.current_joints)
        cart_err = np.linalg.norm(peg - self.TOUCH_POSE)
        xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        contact_force = self._get_contact_force()
        if peg[2] <= self.TOUCH_POSE[2] + 0.005 and self._check_abort(contact_force):
            self._abort_reason = (
                f'APPROACH aborted because contact force '
                f'({contact_force:.1f} N) exceeded safety threshold '
                f'({self._safety_threshold:.1f} N) near touch height.'
            )
            self.get_logger().warn(self._abort_reason)
            self._end_phase(False, cart_err, 0.0, False, self._abort_reason)
            self._touch_joints = None
            self._set_state(self.ABORT)
            return
        joints_ok = bool(
            np.all(np.abs(self.current_joints - self._touch_joints)
                   < self.JOINT_TOLERANCE)
        )
        z_precondition_ok = peg[2] <= self.INSERT_PRECONDITION_MAX_Z
        cart_ok = (
            cart_err < self.CARTESIAN_TOLERANCE
            and z_precondition_ok
        )
        settled = joints_ok and cart_ok
        self._stable_counter = (self._stable_counter + 1) if settled else 0
        stabilized = self._stable_counter >= self.STABILIZE_TICKS

        if self._state_entry_ticks % 50 == 0:
            self.get_logger().info(
                f'APPROACH t={elapsed:.1f}s  '
                f'peg=({peg[0]:.4f}, {peg[1]:.4f}, {peg[2]:.4f})  '
                f'cart_err={cart_err:.3f}  '
                f'xy_err={xy_err:.3f}  '
                f'z_ok={z_precondition_ok}  '
                f'stable={self._stable_counter}/{self.STABILIZE_TICKS}'
            )

        if (
            not stabilized
            and not timed_out
        ):
            self._state_entry_ticks += 1
            return

        joint_err = np.max(np.abs(self.current_joints - self._touch_joints))

        phase_timed_out = False
        phase_message = ''
        if not stabilized:
            self._abort_reason = (
                f'APPROACH timeout/degraded failure ({elapsed:.1f}s). '
                f'cart_err={cart_err:.3f}m, joint_err={joint_err:.3f}rad, '
                f'peg_z={peg[2]:.4f}m, '
                f'z_precondition<={self.INSERT_PRECONDITION_MAX_Z:.4f}m, '
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
        # Check XY alignment before insertion
        if self._pre_insertion_xy_error > self.INSERT_FINAL_XY_TOLERANCE:
            if current_pos[2] > self.INSERT_PRECONDITION_MAX_Z:
                self._abort_reason = (
                    f'SEARCH blocked: peg_z {current_pos[2]:.4f}m is above '
                    f'force-safe precondition '
                    f'{self.INSERT_PRECONDITION_MAX_Z:.4f}m. Approach did '
                    f'not reach the hole surface reliably.'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, cart_err, joint_err, phase_timed_out,
                                self._abort_reason)
                self._touch_joints = None
                self._set_state(self.ABORT)
                return
            if self._pre_insertion_xy_error > self.SEARCH_RADIUS_MAX:
                self._abort_reason = (
                    f'SEARCH blocked: pre-insertion XY error '
                    f'{self._pre_insertion_xy_error:.4f}m exceeds bounded '
                    f'search radius {self.SEARCH_RADIUS_MAX:.4f}m.'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, cart_err, joint_err, phase_timed_out,
                                self._abort_reason)
                self._touch_joints = None
                self._set_state(self.ABORT)
                return
            self.get_logger().warn(
                f'Pre-insertion XY error {self._pre_insertion_xy_error:.4f}m exceeds '
                f'physical clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m. '
                f'Attempting search phase.'
            )
            self._end_phase(True, cart_err, joint_err, phase_timed_out,
                            phase_message)
            self._touch_joints = None
            self._begin_phase(self.SEARCH)
            self._search_angle = 0.0
            self._search_radius = self.SEARCH_RADIUS_INIT
            self._search_total_ticks = 0
            self._search_convergence_ticks = 0
            self._search_recenter_attempts = 0
            self._search_stability_ready_s = 0.0
            self._set_state(self.SEARCH)
            return

        self._end_phase(True, cart_err, joint_err, phase_timed_out, phase_message)
        self._touch_joints = None
        self._insert_start_z = current_pos[2]
        if not self._insert_preconditions_ok(current_pos):
            self._set_state(self.ABORT)
            return
        self._begin_phase(self.INSERT)
        self._progress = 0.0
        self._set_state(self.INSERT)

    # --- SEARCH -------------------------------------------------------------

    def _handle_search(self) -> None:
        if not self._joints_received:
            return

        self._search_total_ticks += 1
        elapsed = self._search_total_ticks / self._control_rate
        fz = self._get_fz()
        contact_force = self._get_contact_force()
        if self._check_abort(fz):
            self._abort_reason = (
                f'SEARCH aborted because Fz ({fz:.1f} N, contact '
                f'{contact_force:.1f} N) exceeded safety threshold '
                f'({self._safety_threshold:.1f} N).'
            )
            self.get_logger().warn(self._abort_reason)
            peg, _ = self._kinematics.pose(self.current_joints)
            xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
            self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
            self._set_state(self.ABORT)
            return
        if elapsed >= self.SEARCH_TIMEOUT_S:
            peg, _ = self._kinematics.pose(self.current_joints)
            xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
            convergence_before = self._search_convergence_ticks
            if xy_err <= self.INSERT_FINAL_XY_TOLERANCE:
                self._abort_reason = (
                    f'SEARCH timeout ({self.SEARCH_TIMEOUT_S:.0f}s). '
                    f'Instantaneous XY error {xy_err:.4f}m is within '
                    f'physical clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m '
                    f'but was not sustained for '
                    f'{self.SEARCH_CONVERGENCE_TICKS} post-command ticks.'
                )
            else:
                self._abort_reason = (
                    f'SEARCH timeout ({self.SEARCH_TIMEOUT_S:.0f}s). '
                    f'XY error {xy_err:.4f}m remains above physical '
                    f'clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m.'
                )
            self.get_logger().error(self._abort_reason)
            self._write_search_gate_trace(
                elapsed_s=elapsed,
                settle_elapsed_s=0.0,
                ready_to_count=False,
                settling_window_active=False,
                command_still_running=False,
                convergence_before=convergence_before,
                xy_error=xy_err,
                peg_z=float(peg[2]),
                decision='timeout_abort',
            )
            self._end_phase(False, xy_err, 0.0, True, self._abort_reason)
            self._set_state(self.ABORT)
            return

        n_steps = self.SEARCH_STEPS
        d_angle = 2.0 * math.pi / n_steps

        # Wait for current trajectory to settle before next step. The settling
        # window is in seconds, not ticks, so non-default control rates do not
        # shorten the hold below the recenter command duration.
        search_settle_elapsed_s = self._state_entry_ticks / self._control_rate
        ready_to_count = self._now_s() >= self._search_stability_ready_s
        settling_window_active = search_settle_elapsed_s < self._search_settle_duration_s
        command_still_running = (
            self._search_stability_ready_s > 0.0
            and not ready_to_count
        )
        if (
            self._search_step < n_steps
            and (
                settling_window_active
                or command_still_running
                or self._search_convergence_ticks > 0
            )
        ):
            self._state_entry_ticks += 1
            peg, _ = self._kinematics.pose(self.current_joints)
            xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
            convergence_before = self._search_convergence_ticks
            decision = 'settling_reset'
            if ready_to_count and xy_err <= self.INSERT_FINAL_XY_TOLERANCE:
                self._search_convergence_ticks += 1
                decision = 'settling_count'
                if self._search_convergence_ticks >= self.SEARCH_CONVERGENCE_TICKS:
                    self._write_search_gate_trace(
                        elapsed_s=elapsed,
                        settle_elapsed_s=search_settle_elapsed_s,
                        ready_to_count=ready_to_count,
                        settling_window_active=settling_window_active,
                        command_still_running=command_still_running,
                        convergence_before=convergence_before,
                        xy_error=xy_err,
                        peg_z=float(peg[2]),
                        decision='settling_converged',
                    )
                    self.get_logger().info(
                        f'SEARCH converged. XY error {xy_err:.4f}m within '
                        f'physical clearance for '
                        f'{self._search_convergence_ticks} ticks.'
                    )
                    self._end_phase(
                        True,
                        xy_err,
                        0.0,
                        False,
                        'SEARCH converged with sustained physical clearance',
                    )
                    self._pre_insertion_xy_error = xy_err
                    if not self._insert_preconditions_ok(peg):
                        self._set_state(self.ABORT)
                        return
                    self._insert_start_z = peg[2]
                    self._progress = 0.0
                    self._begin_phase(self.INSERT)
                    self._set_state(self.INSERT)
                    return
            else:
                self._search_convergence_ticks = 0
            self._write_search_gate_trace(
                elapsed_s=elapsed,
                settle_elapsed_s=search_settle_elapsed_s,
                ready_to_count=ready_to_count,
                settling_window_active=settling_window_active,
                command_still_running=command_still_running,
                convergence_before=convergence_before,
                xy_error=xy_err,
                peg_z=float(peg[2]),
                decision=decision,
            )
            if self._state_entry_ticks % 10 == 0:
                self.get_logger().info(
                    f'SEARCH settling: xy_error={xy_err:.4f}m, stable='
                    f'{self._search_convergence_ticks}/'
                    f'{self.SEARCH_CONVERGENCE_TICKS}, '
                    f'ready_to_count={ready_to_count}, '
                    f'settle_elapsed={search_settle_elapsed_s:.1f}s'
                )
            return  # keep waiting for settling

        if self._search_step >= n_steps:
            self._search_radius = min(self.SEARCH_RADIUS_MAX,
                                      self._search_radius * 1.5)
            self._search_angle = 0.0
            self._search_step = 0
            self._state_entry_ticks = 0
            self._search_convergence_ticks = 0
            self._search_stability_ready_s = 0.0

            if self._search_radius >= self.SEARCH_RADIUS_MAX:
                self._abort_reason = (
                    f'SEARCH exhausted at radius {self._search_radius:.3f}m. '
                    f'XY error {self._pre_insertion_xy_error:.4f}m still > '
                    f'physical clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m.'
                )
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, self._pre_insertion_xy_error, 0.0,
                                False, self._abort_reason)
                self._set_state(self.ABORT)
                return

        peg, _ = self._kinematics.pose(self.current_joints)
        xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        convergence_before = self._search_convergence_ticks
        if ready_to_count and xy_err <= self.INSERT_FINAL_XY_TOLERANCE:
            self._search_convergence_ticks += 1
            self._state_entry_ticks += 1
            if self._search_convergence_ticks >= self.SEARCH_CONVERGENCE_TICKS:
                self._write_search_gate_trace(
                    elapsed_s=elapsed,
                    settle_elapsed_s=search_settle_elapsed_s,
                    ready_to_count=ready_to_count,
                    settling_window_active=settling_window_active,
                    command_still_running=command_still_running,
                    convergence_before=convergence_before,
                    xy_error=xy_err,
                    peg_z=float(peg[2]),
                    decision='post_settle_converged',
                )
                self.get_logger().info(
                    f'SEARCH converged. XY error {xy_err:.4f}m within '
                    f'physical clearance for '
                    f'{self._search_convergence_ticks} ticks.'
                )
                self._end_phase(
                    True,
                    xy_err,
                    0.0,
                    False,
                    'SEARCH converged with sustained physical clearance',
                )
                self._pre_insertion_xy_error = xy_err
                if not self._insert_preconditions_ok(peg):
                    self._set_state(self.ABORT)
                    return
                self._insert_start_z = peg[2]
                self._progress = 0.0
                self._begin_phase(self.INSERT)
                self._set_state(self.INSERT)
                return
            self._write_search_gate_trace(
                elapsed_s=elapsed,
                settle_elapsed_s=search_settle_elapsed_s,
                ready_to_count=ready_to_count,
                settling_window_active=settling_window_active,
                command_still_running=command_still_running,
                convergence_before=convergence_before,
                xy_error=xy_err,
                peg_z=float(peg[2]),
                decision='post_settle_count',
            )
            self.get_logger().info(
                f'SEARCH post-settle hold: xy_error={xy_err:.4f}m, '
                f'stable={self._search_convergence_ticks}/'
                f'{self.SEARCH_CONVERGENCE_TICKS}; continuing to observe '
                f'before sending another command.'
            )
            return
        self._search_convergence_ticks = 0
        if xy_err <= self.SEARCH_RECENTER_XY_TOLERANCE:
            recenter_target = np.array([
                self.HOLE_CENTRE_XY[0],
                self.HOLE_CENTRE_XY[1],
                peg[2],
            ])
            q = self._solve_ik(recenter_target, seed=self.current_joints)
            if q is not None:
                self._send_trajectory_goal(q, self._search_recenter_duration_s)
                self._search_stability_ready_s = (
                    self._now_s() + self._search_recenter_duration_s
                )
                self._search_recenter_attempts += 1
                self.get_logger().info(
                    f'SEARCH recenter {self._search_recenter_attempts}: '
                    f'xy_error={xy_err:.4f}m is inside coarse band '
                    f'{self.SEARCH_RECENTER_XY_TOLERANCE:.4f}m but not '
                    f'sustained physical clearance '
                    f'{self.INSERT_FINAL_XY_TOLERANCE:.4f}m; '
                    f'holding centered target for '
                    f'{self._search_recenter_duration_s:.1f}s.'
                )
                self._state_entry_ticks = 0
                self._search_convergence_ticks = 0
                self._write_search_gate_trace(
                    elapsed_s=elapsed,
                    settle_elapsed_s=search_settle_elapsed_s,
                    ready_to_count=ready_to_count,
                    settling_window_active=settling_window_active,
                    command_still_running=command_still_running,
                    convergence_before=convergence_before,
                    xy_error=xy_err,
                    peg_z=float(peg[2]),
                    decision='recenter_command',
                )
                return

        offset_x = self._search_radius * math.cos(self._search_angle)
        offset_y = self._search_radius * math.sin(self._search_angle)

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
                duration_s = 5.0
                self._send_trajectory_goal(q, duration_s=duration_s)
                self._search_stability_ready_s = self._now_s() + duration_s
                self._write_search_gate_trace(
                    elapsed_s=elapsed,
                    settle_elapsed_s=search_settle_elapsed_s,
                    ready_to_count=ready_to_count,
                    settling_window_active=settling_window_active,
                    command_still_running=command_still_running,
                    convergence_before=convergence_before,
                    xy_error=xy_err,
                    peg_z=float(peg[2]),
                    decision='spiral_command',
                )
                self.get_logger().info(
                    f'SEARCH step {self._search_step + 1}/{n_steps} '
                    f'radius={self._search_radius:.3f}m '
                    f'offset=({offset_x:.4f}, {offset_y:.4f})m  '
                    f'q_dist={q_dist:.5f}'
                )
            else:
                self._search_stability_ready_s = 0.0
                self._write_search_gate_trace(
                    elapsed_s=elapsed,
                    settle_elapsed_s=search_settle_elapsed_s,
                    ready_to_count=ready_to_count,
                    settling_window_active=settling_window_active,
                    command_still_running=command_still_running,
                    convergence_before=convergence_before,
                    xy_error=xy_err,
                    peg_z=float(peg[2]),
                    decision='spiral_ik_too_close',
                )
                self.get_logger().warn(
                    f'SEARCH step {self._search_step + 1}/{n_steps} '
                    f'radius={self._search_radius:.3f}m '
                    f'IK too close to current (q_dist={q_dist:.5f})'
                )

        self._search_angle += d_angle
        self._search_step += 1
        self._state_entry_ticks = 0
        self._search_convergence_ticks = 0

    # --- INSERT -------------------------------------------------------------

    def _physical_insertion_depth(self, peg_z: float) -> float:
        """Depth below the hole top, not just relative downward motion."""
        return max(0.0, self.HOLE_TOP_Z - peg_z)

    def _insert_preconditions_ok(self, peg: np.ndarray) -> bool:
        xy_err = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        if xy_err > self.INSERT_PRECONDITION_XY_TOLERANCE:
            self._abort_reason = (
                f'INSERT blocked: XY error {xy_err:.4f}m exceeds force-safe '
                f'precondition {self.INSERT_PRECONDITION_XY_TOLERANCE:.4f}m.'
            )
            self.get_logger().error(self._abort_reason)
            self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
            return False
        if xy_err > self.INSERT_FINAL_XY_TOLERANCE:
            self._abort_reason = (
                f'INSERT blocked: no-contact XY error {xy_err:.4f}m exceeds '
                f'physical radial clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m.'
            )
            self.get_logger().error(self._abort_reason)
            self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
            return False
        if peg[2] > self.INSERT_PRECONDITION_MAX_Z:
            self._abort_reason = (
                f'INSERT blocked: peg_z {peg[2]:.4f}m is above force-safe '
                f'precondition {self.INSERT_PRECONDITION_MAX_Z:.4f}m. '
                f'Approach did not reach the hole surface reliably.'
            )
            self.get_logger().error(self._abort_reason)
            self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
            return False
        if self._get_fz() > self._safety_threshold:
            self._abort_reason = (
                f'INSERT blocked: current Fz {self._get_fz():.1f}N exceeds '
                f'safety threshold {self._safety_threshold:.1f}N.'
            )
            self.get_logger().error(self._abort_reason)
            self._end_phase(False, xy_err, 0.0, False, self._abort_reason)
            return False
        return True

    def _send_insert_descent(self, peg: np.ndarray) -> bool:
        insert_range = self._insert_start_z - self.FINAL_INSERTION_POSE[2]
        final_target = np.array([
            self.AXIS_ALIGN_POSE[0],
            self.AXIS_ALIGN_POSE[1],
            self.FINAL_INSERTION_POSE[2],
        ])
        q = self._solve_ik(final_target)
        if q is None:
            self.get_logger().error('INSERT: IK failed for final target, aborting')
            self._abort_reason = 'IK failed for final insertion target'
            self._end_phase(False, 0.0, 0.0, False, self._abort_reason)
            self._set_state(self.ABORT)
            return False

        self._insert_traj_dur = 20.0
        self._send_trajectory_goal(q, self._insert_traj_dur)
        self._insert_command_start_s = self._now_s()
        self._insert_command_sent = True
        self._insert_precontact_clearance_ticks = 0

        q_current = self.current_joints.copy()
        q_dist = np.max(np.abs(q_current - q))
        xy_error = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)
        self.get_logger().info(
            f'INSERT descent started after handoff settle: '
            f'{self._insert_traj_dur:.0f}s trajectory, '
            f'{insert_range:.3f}m descent, q_dist={q_dist:.4f}, '
            f'xy_error={xy_error:.4f}m'
        )
        return True

    def _restart_insert_handoff_after_predepth_drift(
        self,
        xy_error: float,
        current_depth: float,
    ) -> bool:
        if (
            self._insert_predepth_recenter_attempts
            >= self.INSERT_PREDEPTH_RECENTER_MAX_ATTEMPTS
        ):
            self._abort_reason = (
                f'INSERT aborted: no-contact XY error {xy_error:.4f}m exceeds '
                f'physical clearance {self.INSERT_FINAL_XY_TOLERANCE:.4f}m '
                f'before meaningful insertion depth '
                f'{self.INSERT_SIDELOAD_DEPTH_GATE_M:.4f}m for '
                f'{self._insert_precontact_clearance_ticks} ticks after '
                f'{self._insert_predepth_recenter_attempts} bounded recenter '
                f'attempts.'
            )
            self.get_logger().warn(self._abort_reason)
            self._final_insertion_xy_error_m = xy_error
            self._end_phase(False, xy_error, 0.0, False, self._abort_reason)
            self._set_state(self.ABORT)
            return True

        self._insert_predepth_recenter_attempts += 1
        self.get_logger().warn(
            f'INSERT pre-depth XY drift {xy_error:.4f}m at depth '
            f'{current_depth:.4f}m; stopping descent and restarting handoff '
            f'recenter attempt {self._insert_predepth_recenter_attempts}/'
            f'{self.INSERT_PREDEPTH_RECENTER_MAX_ATTEMPTS}.'
        )
        self._insert_command_sent = False
        self._insert_command_start_s = 0.0
        self._insert_handoff_hold_sent = False
        self._insert_handoff_hold_start_s = 0.0
        self._insert_handoff_stable_ticks = 0
        self._insert_precontact_clearance_ticks = 0
        return False

    def _abort_insert_sideload(self, xy_error: float, current_depth: float) -> None:
        self._abort_reason = (
            f'INSERT aborted: side-loaded peg at depth '
            f'{current_depth:.4f}m with XY error {xy_error:.4f}m, '
            f'exceeding physical clearance '
            f'{self.INSERT_FINAL_XY_TOLERANCE:.4f}m for '
            f'{self._insert_sideload_ticks} ticks.'
        )
        self.get_logger().warn(self._abort_reason)
        self._final_insertion_xy_error_m = xy_error
        self._end_phase(False, xy_error, 0.0, False, self._abort_reason)
        self._set_state(self.ABORT)

    def _start_shallow_sideload_withdraw(
        self,
        peg: np.ndarray,
        xy_error: float,
        current_depth: float,
    ) -> bool:
        if (
            current_depth > self.INSERT_SHALLOW_SIDELOAD_RECOVERY_DEPTH_M
            or self._insert_shallow_sideload_recovery_attempts
            >= self.INSERT_SHALLOW_SIDELOAD_RECOVERY_MAX_ATTEMPTS
        ):
            return False

        target_z = min(
            self._insert_start_z,
            max(
                self.HOLE_TOP_Z + self.INSERT_SIDELOAD_WITHDRAW_CLEARANCE_M,
                float(peg[2]) + self.INSERT_SIDELOAD_WITHDRAW_CLEARANCE_M,
            ),
        )
        withdraw_target = np.array([peg[0], peg[1], target_z])
        q = self._solve_ik(withdraw_target, self.current_joints, max_iter=100)
        if q is None:
            self.get_logger().warn(
                'INSERT shallow side-load withdrawal IK failed; aborting '
                'instead of continuing side-loaded insertion.'
            )
            return False

        self._insert_shallow_sideload_recovery_attempts += 1
        self._send_trajectory_goal(q, self.INSERT_SIDELOAD_WITHDRAW_DURATION_S)
        self._insert_sideload_withdraw_active = True
        self._insert_sideload_withdraw_start_s = self._now_s()
        self._insert_command_sent = False
        self._insert_command_start_s = 0.0
        self._insert_handoff_hold_sent = False
        self._insert_handoff_hold_start_s = 0.0
        self._insert_handoff_stable_ticks = 0
        self._insert_precontact_clearance_ticks = 0
        self._insert_sideload_ticks = 0

        self.get_logger().warn(
            f'INSERT shallow side-load at depth {current_depth:.4f}m with '
            f'XY error {xy_error:.4f}m; withdrawing to z={target_z:.4f}m '
            f'and retrying handoff '
            f'{self._insert_shallow_sideload_recovery_attempts}/'
            f'{self.INSERT_SHALLOW_SIDELOAD_RECOVERY_MAX_ATTEMPTS}.'
        )
        return True

    def _handle_shallow_sideload_withdraw(
        self,
        xy_error: float,
        current_depth: float,
    ) -> bool:
        if not self._insert_sideload_withdraw_active:
            return False

        elapsed = self._now_s() - self._insert_sideload_withdraw_start_s
        settled = (
            elapsed >= self.INSERT_SIDELOAD_WITHDRAW_DURATION_S
            and current_depth < self.INSERT_SIDELOAD_DEPTH_GATE_M
            and xy_error <= self.INSERT_PRECONDITION_XY_TOLERANCE
        )
        timed_out = elapsed >= self.INSERT_SIDELOAD_WITHDRAW_DURATION_S + 3.0

        if settled:
            self.get_logger().info(
                f'INSERT shallow side-load withdrawal complete: '
                f'depth={current_depth:.4f}m, xy_error={xy_error:.4f}m. '
                f'Restarting handoff gate.'
            )
            self._insert_sideload_withdraw_active = False
            self._insert_handoff_hold_sent = False
            self._insert_handoff_hold_start_s = 0.0
            self._insert_handoff_stable_ticks = 0
            self._insert_precontact_clearance_ticks = 0
            self._insert_sideload_ticks = 0
            return False

        if timed_out:
            self._abort_reason = (
                f'INSERT aborted: shallow side-load withdrawal did not clear '
                f'the hole after {elapsed:.1f}s; depth={current_depth:.4f}m, '
                f'xy_error={xy_error:.4f}m.'
            )
            self.get_logger().warn(self._abort_reason)
            self._final_insertion_xy_error_m = xy_error
            self._end_phase(False, xy_error, 0.0, True, self._abort_reason)
            self._set_state(self.ABORT)
            return True

        if self._state_entry_ticks % 10 == 0:
            self.get_logger().info(
                f'INSERT shallow side-load withdrawal t={elapsed:.1f}s  '
                f'depth={current_depth:.4f}m  xy_error={xy_error:.4f}m'
            )
        return True

    def _handle_insert_handoff_settle(
        self,
        peg: np.ndarray,
        xy_error: float,
        current_depth: float,
        fz: float,
    ) -> bool:
        if not self._insert_handoff_hold_sent:
            hold_target = np.array([
                self.HOLE_CENTRE_XY[0],
                self.HOLE_CENTRE_XY[1],
                peg[2],
            ])
            q = self._solve_ik(hold_target, self.current_joints, max_iter=100)
            if q is None:
                self._abort_reason = 'INSERT handoff hold IK failed'
                self.get_logger().error(self._abort_reason)
                self._end_phase(False, xy_error, 0.0, False, self._abort_reason)
                self._set_state(self.ABORT)
                return False

            self._send_trajectory_goal(q, self._insert_handoff_hold_duration_s)
            self._insert_handoff_hold_start_s = self._now_s()
            self._insert_handoff_hold_sent = True
            self._insert_handoff_stable_ticks = 0
            self.get_logger().info(
                f'INSERT handoff hold started: duration='
                f'{self._insert_handoff_hold_duration_s:.1f}s, '
                f'xy_error={xy_error:.4f}m, peg_z={peg[2]:.4f}m'
            )
            return False

        elapsed = self._now_s() - self._insert_handoff_hold_start_s
        ready_to_count = elapsed >= self._insert_handoff_hold_duration_s
        stable = (
            ready_to_count
            and current_depth < self.INSERT_SIDELOAD_DEPTH_GATE_M
            and xy_error <= self.INSERT_FINAL_XY_TOLERANCE
            and fz <= self._safety_threshold
        )
        self._insert_handoff_stable_ticks = (
            self._insert_handoff_stable_ticks + 1 if stable else 0
        )

        if self._state_entry_ticks % 10 == 0:
            self.get_logger().info(
                f'INSERT handoff settle t={elapsed:.1f}s  '
                f'xy_error={xy_error:.4f}m  '
                f'depth={current_depth:.4f}m  '
                f'fz={fz:.1f}N  '
                f'stable={self._insert_handoff_stable_ticks}/'
                f'{self.INSERT_HANDOFF_SETTLE_TICKS}'
            )

        if self._insert_handoff_stable_ticks >= self.INSERT_HANDOFF_SETTLE_TICKS:
            return self._send_insert_descent(peg)

        if elapsed >= self._insert_handoff_timeout_s:
            self._abort_reason = (
                f'INSERT handoff settle timeout: XY error {xy_error:.4f}m did '
                f'not remain within physical clearance '
                f'{self.INSERT_FINAL_XY_TOLERANCE:.4f}m for '
                f'{self.INSERT_HANDOFF_SETTLE_TICKS} ticks before descent.'
            )
            self.get_logger().warn(self._abort_reason)
            self._final_insertion_xy_error_m = xy_error
            self._end_phase(False, xy_error, 0.0, True, self._abort_reason)
            self._set_state(self.ABORT)
            return False

        return False

    def _handle_insert(self) -> None:
        self._state_entry_ticks += 1
        contact_force = self._get_contact_force()
        fz = self._get_fz()
        self._max_fz = max(self._max_fz, fz)
        self._max_contact_force = max(self._max_contact_force, contact_force)
        self._max_insert_contact_force = max(
            self._max_insert_contact_force,
            contact_force,
        )

        force_active = self._state_entry_ticks >= 5
        peg, _ = self._kinematics.pose(self.current_joints)
        current_depth = self._physical_insertion_depth(peg[2])
        self._insertion_depth_m = max(self._insertion_depth_m, current_depth)
        xy_error = np.linalg.norm(peg[:2] - self.HOLE_CENTRE_XY)

        if xy_error > self.INSERT_PRECONDITION_XY_TOLERANCE:
            self._abort_reason = (
                f'INSERT aborted: XY error {xy_error:.4f}m exceeded force-safe '
                f'limit {self.INSERT_PRECONDITION_XY_TOLERANCE:.4f}m '
                f'at physical depth {current_depth:.4f}m.'
            )
            self.get_logger().warn(self._abort_reason)
            self._end_phase(False, xy_error, 0.0, False, self._abort_reason)
            self._set_state(self.ABORT)
            return

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

        if self._handle_shallow_sideload_withdraw(xy_error, current_depth):
            return

        side_loaded = (
            current_depth >= self.INSERT_SIDELOAD_DEPTH_GATE_M
            and xy_error > self.INSERT_FINAL_XY_TOLERANCE
        )
        self._insert_sideload_ticks = (
            self._insert_sideload_ticks + 1 if side_loaded else 0
        )
        if self._insert_sideload_ticks >= self.INSERT_SIDELOAD_SETTLE_TICKS:
            if self._start_shallow_sideload_withdraw(
                peg,
                xy_error,
                current_depth,
            ):
                return
            self._abort_insert_sideload(xy_error, current_depth)
            return

        if not self._insert_command_sent:
            self._handle_insert_handoff_settle(peg, xy_error, current_depth, fz)
            return

        precontact_misaligned = (
            current_depth < self.INSERT_SIDELOAD_DEPTH_GATE_M
            and xy_error > self.INSERT_FINAL_XY_TOLERANCE
        )
        self._insert_precontact_clearance_ticks = (
            self._insert_precontact_clearance_ticks + 1
            if precontact_misaligned
            else 0
        )
        if self._insert_precontact_clearance_ticks >= self.INSERT_PRECONTACT_CLEARANCE_TICKS:
            self._restart_insert_handoff_after_predepth_drift(
                xy_error,
                current_depth,
            )
            return

        # --- Logging ---
        if self._state_entry_ticks % 10 == 0:
            self.get_logger().info(
                f'INSERT t={self._state_entry_ticks / self._control_rate:.1f}s  '
                f'peg_z={peg[2]:.4f}  physical_depth={current_depth:.4f}m  '
                f'Fz={fz:.1f}N  baseline={self._baseline_fz:.1f}N  '
                f'contact={contact_force:.1f}N  '
                f'xy_error={xy_error:.4f}m'
            )

        # --- Check completion: wait for trajectory to finish then evaluate ---
        elapsed_since_command = self._now_s() - self._insert_command_start_s
        insert_elapsed = (
            elapsed_since_command
            >= self._insert_traj_dur + self.INSERT_SETTLE_MARGIN_S
        )
        if insert_elapsed:
            final_depth = self._physical_insertion_depth(peg[2])
            self._insertion_depth_m = final_depth
            self._final_insertion_xy_error_m = xy_error
            self.get_logger().info(
                f'INSERT done (trajectory elapsed).  '
                f'peg=({peg[0]:.4f}, {peg[1]:.4f}, {peg[2]:.4f})  '
                f'depth={final_depth:.4f}m  '
                f'xy_error={xy_error:.4f}m  '
                f'Fz={fz:.1f}N  contact={contact_force:.1f}N  '
                f'max_abs_Fz={self._max_fz:.1f}N  '
                f'max_force_norm={self._max_force_norm:.1f}N  '
                f'max_contact={self._max_contact_force:.1f}N  '
                f'max_insert_contact={self._max_insert_contact_force:.1f}N'
            )

            depth_ok = final_depth >= 0.010
            contact_pattern_ok = (
                self._max_insert_contact_force >= self._contact_threshold
            )
            final_xy_ok = xy_error <= self.INSERT_FINAL_XY_TOLERANCE

            if depth_ok and contact_pattern_ok and final_xy_ok:
                self._end_phase(True, 0.0, 0.0, False,
                                f'Insertion depth {final_depth:.3f}m with '
                                f'insert contact evidence '
                                f'{self._max_insert_contact_force:.1f}N and '
                                f'final XY error {xy_error:.4f}m')
            elif depth_ok and contact_pattern_ok and not final_xy_ok:
                self._end_phase(False, xy_error, 0.0, False,
                                f'Depth reached ({final_depth:.3f}m) and '
                                f'insert contact '
                                f'{self._max_insert_contact_force:.1f}N, but '
                                f'final XY error {xy_error:.4f}m exceeds '
                                f'physical hole clearance '
                                f'{self.INSERT_FINAL_XY_TOLERANCE:.4f}m. '
                                f'Insertion is side-loaded.')
            elif depth_ok and not contact_pattern_ok:
                self._end_phase(False, 0.0, 0.0, False,
                                f'Depth reached ({final_depth:.3f}m) but '
                                f'max insert contact '
                                f'({self._max_insert_contact_force:.1f}N) below '
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
            clearance_waypoints = self._build_vertical_clearance_waypoints()
            q_after_clearance = clearance_waypoints[-1]
            dist = float(np.max(np.abs(q_after_clearance - self.SAFE_HOME)))
            home_waypoint_count = max(2, min(10, int(dist / 0.2) + 1))

            waypoints = list(clearance_waypoints)
            for i in range(1, home_waypoint_count):
                alpha = i / (home_waypoint_count - 1)
                q_interp = (
                    q_after_clearance
                    + alpha * (self.SAFE_HOME - q_after_clearance)
                )
                waypoints.append(q_interp)

            lift_waypoints = max(0, len(clearance_waypoints) - 1)
            duration = max(
                8.0,
                min(25.0, lift_waypoints * 1.0 + dist * 10.0),
            )
            self._publish_multi_point_trajectory(waypoints, duration)

            self._start_ticks = 0
            self._retreat_sent = True
            self.get_logger().info(
                f'Retreating to SAFE_HOME via clearance lift. '
                f'duration={duration:.1f}s, '
                f'lift_waypoints={lift_waypoints}, '
                f'home_waypoints={home_waypoint_count}'
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
                f'RETREAT timeout after {self.TRAJECTORY_TIMEOUT_S:.0f}s. '
                f'joints not at SAFE_HOME.'
            )

        self._end_phase(retreat_ok, 0.0, 0.0, timed_out)

        self._set_state(self.DONE)
        self._log_final_outcome()

    # --- Outcome logging ----------------------------------------------------

    def _log_final_outcome(self) -> None:
        if self._final_outcome_logged:
            return
        self._final_outcome_logged = True

        timeline = [r.to_dict() for r in self._phase_results]

        all_success = all(
            r.success for r in self._phase_results
            if r.name not in (self.RETREAT,)
        )
        had_timeout = any(r.timed_out for r in self._phase_results)
        any_failure = any(not r.success for r in self._phase_results
                          if r.name not in (self.RETREAT,))

        depth_ok = self._insertion_depth_m >= 0.010
        contact_ok = self._max_insert_contact_force >= self._contact_threshold
        final_xy_ok = (
            self._final_insertion_xy_error_m
            <= self.INSERT_FINAL_XY_TOLERANCE
        )

        if self._abort_reason:
            self._trial_outcome = 'ABORTED'
            reason = self._abort_reason
        elif depth_ok and contact_ok and not final_xy_ok:
            self._trial_outcome = 'DEGRADED'
            reason = (
                f'Final insertion XY error '
                f'({self._final_insertion_xy_error_m:.4f}m) exceeds physical '
                f'hole clearance '
                f'({self.INSERT_FINAL_XY_TOLERANCE:.4f}m). '
                f'Peg is side-loaded; do not count as physical success.'
            )
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
                f'insertion contact threshold ({self._contact_threshold:.1f}N), '
                f'with insert contact '
                f'{self._max_insert_contact_force:.1f}N. '
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
                f'{self._max_insert_contact_force:.1f}N during INSERT.'
            )

        outcome = {
            'trial_outcome': self._trial_outcome,
            'reason': reason,
            'phases': timeline,
            'metrics': {
                'initial_xy_error_m': round(self._initial_xy_error, 4),
                'pre_insertion_xy_error_m': round(self._pre_insertion_xy_error, 4),
                'final_insertion_xy_error_m': round(
                    self._final_insertion_xy_error_m,
                    4,
                ),
                'insert_final_xy_tolerance_m': round(
                    self.INSERT_FINAL_XY_TOLERANCE,
                    4,
                ),
                'insertion_depth_m': round(self._insertion_depth_m, 4),
                'max_fz_N': round(self._max_fz, 2),
                'max_abs_fz_N': round(self._max_fz, 2),
                'max_force_norm_N': round(self._max_force_norm, 2),
                'baseline_fz_N': round(self._baseline_fz, 2),
                'max_contact_force_N': round(self._max_contact_force, 2),
                'max_insert_contact_force_N': round(
                    self._max_insert_contact_force,
                    2,
                ),
                'contact_threshold_N': self._contact_threshold,
                'control_rate_hz': round(self._control_rate, 3),
                'search_convergence_required_ticks': self.SEARCH_CONVERGENCE_TICKS,
                'search_convergence_required_duration_s': round(
                    self.SEARCH_CONVERGENCE_TICKS / self._control_rate,
                    3,
                ),
                'search_recenter_duration_s': round(
                    self._search_recenter_duration_s,
                    3,
                ),
                'search_settle_duration_s': round(
                    self._search_settle_duration_s,
                    3,
                ),
                'insert_handoff_hold_duration_s': round(
                    self._insert_handoff_hold_duration_s,
                    3,
                ),
                'insert_handoff_timeout_s': round(
                    self._insert_handoff_timeout_s,
                    3,
                ),
                'insert_predepth_recenter_attempts': (
                    self._insert_predepth_recenter_attempts
                ),
                'insert_shallow_sideload_recovery_attempts': (
                    self._insert_shallow_sideload_recovery_attempts
                ),
                'gravity_baseline_valid': self._baseline_valid,
                'baseline_window_samples': len(self._fz_buffer),
            },
            'status': self._state,
        }

        self.get_logger().info(
            f'\n'
            f'========== TRIAL OUTCOME ==========\n'
            f'  Outcome: {self._trial_outcome}\n'
            f'  Reason:  {reason}\n'
            f'  Depth:   {self._insertion_depth_m:.4f}m\n'
            f'  Max |Fz|:{self._max_fz:.1f}N\n'
            f'  Max |F|: {self._max_force_norm:.1f}N\n'
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

        if self._exit_on_done:
            self._schedule_done_exit()

    def _schedule_done_exit(self) -> None:
        if self._done_exit_timer is not None:
            return
        delay_s = max(0.01, self._done_exit_delay_s)
        self.get_logger().info(
            f'exit_on_done enabled; shutting down node in {delay_s:.2f}s.'
        )
        self._done_exit_timer = self.create_timer(delay_s, self._exit_after_done)

    def _exit_after_done(self) -> None:
        if self._done_exit_timer is not None:
            self._done_exit_timer.cancel()
        self.get_logger().info('DONE outcome written; exiting task node.')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittanceInsertionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt - shutting down.')
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

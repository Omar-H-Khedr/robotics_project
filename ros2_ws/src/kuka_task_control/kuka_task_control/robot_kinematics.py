"""Forward/inverse kinematics for KUKA LBR iisy6 R1300 with peg tip.

Uses finite-difference Jacobian and damped least-squares IK.
All joint transforms from the URDF macro:
  src/external/kuka_robot_descriptions/kuka_lbr_iisy_support/urdf/lbr_iisy6_r1300_macro.xacro
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

# Type alias for a 4x4 homogeneous transform matrix
Transform4 = np.ndarray  # shape (4, 4)


@dataclass
class JointTransform:
    """URDF joint definition for the kinematic chain."""

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Tuple[float, float, float] = (0.0, 0.0, 1.0)


def _rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Extrinsic XYZ (roll, pitch, yaw) → 3x3 rotation matrix."""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _rotation_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues' formula: rotation about *axis* by *angle* (rad)."""
    ax = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0, -ax[2], ax[1]],
                  [ax[2], 0, -ax[0]],
                  [-ax[1], ax[0], 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _make_transform(xyz: Tuple[float, float, float],
                    rpy: Tuple[float, float, float]) -> Transform4:
    """Build 4x4 homogeneous transform from xyz + extrinsic XYZ rpy."""
    R = _rotation_from_rpy(*rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def _rotation_error(R_current: np.ndarray, R_target: np.ndarray) -> np.ndarray:
    """Axis-angle orientation error from current to target rotation."""
    R_err = R_current.T @ R_target
    trace = R_err[0, 0] + R_err[1, 1] + R_err[2, 2]
    cos_theta = min(1.0, max(-1.0, (trace - 1.0) / 2.0))
    theta = np.arccos(cos_theta)
    if theta < 1e-10:
        return np.zeros(3)
    sin_theta = np.sin(theta)
    w = np.array([R_err[2, 1] - R_err[1, 2],
                  R_err[0, 2] - R_err[2, 0],
                  R_err[1, 0] - R_err[0, 1]]) / (2.0 * sin_theta)
    return w * theta


class RobotKinematics:
    """Forward and inverse kinematics for LBR iisy6 R1300 + peg tip.

    Chain: world → base_link → joint_1→...→joint_6 → link_6 → peg_tip
    """

    # Joint transforms from URDF (origin xyz, rpy).  All joints have
    # axis=(0,0,1) in the parent frame and are revolute.
    _JOINT_TRANSFORMS = [
        JointTransform(xyz=(0.0, 0.0, 0.1845), rpy=(np.pi, 0.0, 0.0)),
        JointTransform(xyz=(0.0, 0.1011, -0.1155), rpy=(np.pi / 2, 0.0, 0.0)),
        JointTransform(xyz=(0.59, 0.0, 0.0237), rpy=(0.0, 0.0, 0.0)),
        JointTransform(xyz=(0.1139, 0.0, 0.0774), rpy=(np.pi / 2, 0.0, -np.pi / 2)),
        JointTransform(xyz=(0.0, 0.0507, -0.4181), rpy=(0.0, np.pi / 2, np.pi / 2)),
        JointTransform(xyz=(0.0837, 0.0, -0.0507), rpy=(np.pi / 2, 0.0, -np.pi / 2)),
    ]

    # Fixed transform link_6 → peg_tip:
    #   link6-flange:      xyz=(0,0,-0.0943),  rpy=(0, pi/2, 0)
    #   flange-ft_sensor:  xyz=(0,0, 0.005),   rpy=(0,0,0)
    #   ft_sensor-palm:    xyz=(0,0, 0),       rpy=(0,0,0)
    #   palm-peg_tip:      xyz=(0,0, 0.015),   rpy=(0,0,0)
    # Combined:
    _LINK6_TO_PEGTIP = (
        _make_transform((0.0, 0.0, -0.0943), (0.0, np.pi / 2, 0.0))
        @ _make_transform((0.0, 0.0, 0.02), (0.0, 0.0, 0.0))
    )

    def __init__(self, base_xyz=(0.80, -0.75, 0.735), base_rpy=(0, 0, np.pi / 2)):
        """Initialise with the world→base_link transform."""
        self._T_world_base = _make_transform(base_xyz, base_rpy)
        self._dls_lambda = 0.01  # damping factor
        self._eps = 1e-6  # finite-difference perturbation

    def forward(self, joints: np.ndarray) -> Transform4:
        """Compute world→peg_tip transform at given joint angles (6,)."""
        T = np.eye(4)
        q_idx = 0
        for jt in self._JOINT_TRANSFORMS:
            T_origin = _make_transform(jt.xyz, jt.rpy)
            # URDF: axis is in the joint frame (after T_origin).
            # All joints have axis=(0,0,1) → RotZ about the local Z.
            R_joint = np.eye(4)
            R_joint[:3, :3] = _rot_z(joints[q_idx])
            T = T @ T_origin @ R_joint
            q_idx += 1
        T = T @ self._LINK6_TO_PEGTIP
        T = self._T_world_base @ T
        return T

    def _pose_from_transform(self, T: Transform4
                             ) -> Tuple[np.ndarray, np.ndarray]:
        """Extract position (3,) and rotation matrix (3,3)."""
        return T[:3, 3], T[:3, :3]

    def pose(self, joints: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return (position, rotation_matrix) of peg_tip in world frame."""
        return self._pose_from_transform(self.forward(joints))

    def jacobian(self, joints: np.ndarray) -> np.ndarray:
        """Finite-difference geometric Jacobian (6×6).

        Columns map joint velocity to [v; ω] at the peg tip in world frame.
        """
        pos0, rot0 = self.pose(joints)
        J = np.zeros((6, len(self._JOINT_TRANSFORMS)))
        for i in range(len(self._JOINT_TRANSFORMS)):
            q_pert = joints.copy()
            q_pert[i] += self._eps
            pos_i, rot_i = self.pose(q_pert)
            # Translational: (p_i - p0) / eps
            J[:3, i] = (pos_i - pos0) / self._eps
            # Rotational: orientation error between rot_i and rot0
            # Use skew of (R_i * R0^T) to get rotation axis
            R_rel = rot_i @ rot0.T
            trace = R_rel[0, 0] + R_rel[1, 1] + R_rel[2, 2]
            cos_a = min(1.0, max(-1.0, (trace - 1.0) / 2.0))
            if cos_a > 0.999999:
                w = np.zeros(3)
            else:
                theta = np.arccos(cos_a)
                sin_t = np.sin(theta)
                w = np.array([R_rel[2, 1] - R_rel[1, 2],
                              R_rel[0, 2] - R_rel[2, 0],
                              R_rel[1, 0] - R_rel[0, 1]]) / (2.0 * sin_t)
                w *= (theta / self._eps)
            J[3:, i] = w
        return J

    @staticmethod
    def _pinv(A: np.ndarray, damp: float = 1e-4) -> np.ndarray:
        """Damped pseudo-inverse: A^T (A A^T + damp² I)^{-1}."""
        m = A.shape[0]
        return A.T @ np.linalg.solve(A @ A.T + damp ** 2 * np.eye(m), np.eye(m))

    def inverse_position(self,
                         target_pos: np.ndarray,
                         initial_joints: np.ndarray,
                         max_iter: int = 50,
                         tol: float = 1e-6) -> Tuple[np.ndarray, bool, float]:
        """Position-only IK — ignore orientation entirely."""
        q = initial_joints.copy()
        for _ in range(max_iter):
            pos, _ = self.pose(q)
            e = target_pos - pos
            en = np.linalg.norm(e)
            if en < tol:
                return q, True, en
            J_pos = self.jacobian(q)[:3, :]
            dq = self._pinv(J_pos, self._dls_lambda) @ e
            q = q + dq
        pos, _ = self.pose(q)
        return q, False, np.linalg.norm(target_pos - pos)

    def inverse(self,
                target_pos: np.ndarray,
                target_rot: np.ndarray,
                initial_joints: np.ndarray,
                max_iter: int = 50,
                tol: float = 1e-6) -> Tuple[np.ndarray, bool, float]:
        """Task-priority IK: position primary, orientation secondary.

        Position is satisfied first.  Orientation is then optimised in
        the null-space of the position task.
        """
        q = initial_joints.copy()
        for _ in range(max_iter):
            pos, rot = self.pose(q)
            err_pos = target_pos - pos
            err_rot = _rotation_error(rot, target_rot)

            J = self.jacobian(q)
            J_pos = J[:3, :]   # 3×6

            # Primary: position
            dq_pos = self._pinv(J_pos, self._dls_lambda) @ err_pos

            # Secondary: orientation in null-space of J_pos
            N = np.eye(6) - self._pinv(J_pos, 1e-6) @ J_pos
            J_rot = J[3:, :]   # 3×6
            J_rot_n = J_rot @ N
            dq_rot = N @ self._pinv(J_rot_n, self._dls_lambda) @ err_rot

            q = q + dq_pos + dq_rot

            err_norm = np.linalg.norm(np.concatenate([err_pos, err_rot]))
            if err_norm < tol:
                return q, True, err_norm

        pos, rot = self.pose(q)
        err_pos = np.linalg.norm(target_pos - pos)
        err_rot = np.linalg.norm(_rotation_error(rot, target_rot))
        return q, False, err_pos + err_rot


def _test_fk() -> None:
    """Quick FK sanity check — peg tip should be above-ish the hole at SAFE_HOME."""
    kin = RobotKinematics()
    joints = np.array([0.0, -0.8, 1.2, 0.0, 0.8, 0.0])
    pos, rot = kin.pose(joints)
    print(f"SAFE_HOME peg tip: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
    # Hole centre: (0.520, -0.200, 0.810)
    print(f"Hole distance XY: {np.linalg.norm(pos[:2] - [0.520, -0.200]):.3f} m")
    print(f"Hole distance Z:  {pos[2] - 0.810:.3f} m (positive = above)")


if __name__ == '__main__':
    _test_fk()

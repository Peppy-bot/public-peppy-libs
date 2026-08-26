"""FK/IK over lerobot's placo-based RobotKinematics, radians at this boundary.

lerobot's solver speaks degrees and 4x4 matrices; everything above this module
speaks joint_link radians and wire poses. IK is verified by FK before it is
trusted: placo returns its best effort even far from the target, and an
unreached pose must fail the caller instead of moving the arm somewhere else.
"""

from __future__ import annotations

import math

import numpy as np

from so101_description.model import END_EFFECTOR_FRAME
from so101_description.transforms import matrix_from_pose, pose_from_matrix
from so101_description.units import JOINT_NAMES

# Positional acceptance for a verified point-to-point solution.
IK_POSITION_TOLERANCE_M = 0.01
# lerobot's inverse_kinematics runs one QP step, sized for streaming small
# deltas; a point-to-point solve iterates it until the verified error
# converges. A near seed exits on the first pass.
IK_MAX_ITERATIONS = 100
# The streamed path is best effort: a few steps bound the per-tick cost, and
# an out-of-reach pose tracks the workspace boundary instead of freezing.
IK_STREAM_ITERATIONS = 3


class Kinematics:
    def __init__(self, urdf_path: str):
        from lerobot.model.kinematics import RobotKinematics

        self._solver = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name=END_EFFECTOR_FRAME,
            joint_names=list(JOINT_NAMES),
        )

    def forward_kinematics(self, positions_rad: tuple[float, ...]):
        """(position m, quaternion xyzw) of the end effector."""
        matrix = self._solver.forward_kinematics(np.degrees(positions_rad))
        return pose_from_matrix(matrix)

    def inverse_kinematics(
        self, seed_rad: tuple[float, ...], position, orientation
    ) -> tuple[float, ...] | None:
        """Joint radians reaching the pose, or None when no verified solution
        lands within tolerance of the target position."""
        target_matrix = matrix_from_pose(position, orientation)
        target_position = tuple(position)
        joints_deg = np.degrees(seed_rad)
        for _ in range(IK_MAX_ITERATIONS):
            joints_deg = self._step(joints_deg, target_matrix)
            solution_rad = self._as_radians(joints_deg)
            reached, _ = self.forward_kinematics(solution_rad)
            if math.dist(reached, target_position) <= IK_POSITION_TOLERANCE_M:
                return solution_rad
        return None

    def inverse_kinematics_streaming(
        self, seed_rad: tuple[float, ...], position, orientation
    ) -> tuple[float, ...] | None:
        """Best-effort step toward the pose for the streamed path: a bounded
        few QP iterations, unverified, so an out-of-reach target tracks the
        workspace boundary instead of freezing the arm. None only for a
        solution the solver corrupted (non-finite)."""
        target_matrix = matrix_from_pose(position, orientation)
        joints_deg = np.degrees(seed_rad)
        for _ in range(IK_STREAM_ITERATIONS):
            joints_deg = self._step(joints_deg, target_matrix)
        solution_rad = self._as_radians(joints_deg)
        if not all(math.isfinite(v) for v in solution_rad):
            return None
        return solution_rad

    def _step(self, joints_deg, target_matrix):
        return self._solver.inverse_kinematics(joints_deg, target_matrix)

    @staticmethod
    def _as_radians(joints_deg) -> tuple[float, ...]:
        return tuple(float(v) for v in np.radians(joints_deg[: len(JOINT_NAMES)]))

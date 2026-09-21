"""FK/IK over placo's kinematics solver, radians at this boundary.

placo speaks 4x4 matrices; everything above this module speaks joint_link
radians and wire poses. IK is verified by FK before it is trusted: placo
returns its best effort even far from the target, and an unreached pose must
fail the caller instead of moving the arm somewhere else.

What that verification costs, measured on poses reachable by construction (a
random in-limits joint vector, its FK taken as the target): 14% of targets are
refused when seeded from the ready posture and 39% when seeded from an
arbitrary one, concentrated near the base (25% within 0.23 m, 7% beyond
0.45 m). The cause is structural rather than a tuning miss. A solve is a
single linearised QP step, so iterating it is a local descent from the seed
and reaches only what the seed's branch reaches; a refusal means this branch
did not arrive, not that the pose is unreachable. Callers that need the wider
workspace re-seed and ask again.
"""

from __future__ import annotations

import math

import numpy as np

from so101_description.model import END_EFFECTOR_FRAME
from so101_description.transforms import (
    matrix_from_pose,
    pose_from_matrix,
    relative_rotation_rad,
)
from so101_description.units import JOINT_NAMES

# Positional acceptance for a verified point-to-point solution.
IK_POSITION_TOLERANCE_M = 0.01
# A solve runs one QP step, sized for streaming small deltas; a
# point-to-point solve iterates it until FK verifies the position.
IK_MAX_ITERATIONS = 100
# The streamed path is best effort: a few steps bound the per-tick cost, and
# an out-of-reach pose tracks the workspace boundary instead of freezing.
IK_STREAM_ITERATIONS = 3

# Position and orientation are both soft objectives of one QP, minimising
# `position_weight * |dp|^2 + orientation_weight * |dtheta|^2`. The terms are
# metres against radians, so the weight ratio is not the trade: at 0.01 a 20
# degree miss outweighs a 7 mm miss by more than twenty to one, and the
# solver spends position buying orientation. Five joints
# underactuate three rotational degrees of freedom, so on this arm that
# purchase is frequently impossible and the position is spent for nothing.
#
# Weighted this low the orientation objective still resolves the wrist onto
# the pose it was given, reaching a reachable orientation within a third of a
# degree, while an unreachable one stops taking the position with it. Zero is
# not the answer; it abandons reachable orientations entirely.
ORIENTATION_WEIGHT = 1e-4
# The position objective's weight, the unit the orientation weight is
# measured against.
POSITION_WEIGHT = 1.0


def _bar(value: float | None, default: float) -> float:
    """A caller's acceptance bar, or the default when unstated. A bar that is
    not a usable positive distance is a caller error, not a reason to fall
    back to the default."""
    if value is None:
        return default
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"tolerance must be finite and positive, got {value}")
    return value

class Kinematics:
    """The arm's model loaded once, with the QP that solves the end effector's
    pose over it. The import is deferred to here so hardware-only consumers
    of this package never load the solver stack."""

    def __init__(self, urdf_path: str):
        import placo  # pylint: disable=C0415

        self._robot = placo.RobotWrapper(urdf_path)
        self._solver = placo.KinematicsSolver(self._robot)
        self._solver.mask_fbase(True)
        self._tip = self._solver.add_frame_task(END_EFFECTOR_FRAME, np.eye(4))

    def _set_joints(self, positions_rad) -> None:
        for name, value in zip(JOINT_NAMES, positions_rad, strict=True):
            self._robot.set_joint(name, float(value))
        self._robot.update_kinematics()

    def _joints(self) -> tuple[float, ...]:
        return tuple(float(self._robot.get_joint(name)) for name in JOINT_NAMES)

    def jacobian(self, positions_rad: tuple[float, ...]):
        """The end-effector Jacobian at these joint positions: 6 rows by one
        column per arm joint in wire order, linear velocity in rows 0..2 and
        angular in rows 3..5, both in world-aligned axes.

        `J @ dq` is the twist a joint step produces, and it is linear in that
        step, which is what a speed cap needs: scaling a step by s scales its
        end-effector speed by exactly s. Measuring the same thing by moving
        the arm and differencing forward kinematics does not have that
        property, because forward kinematics is not linear.

        Local-world-aligned, not world: Pinocchio's world Jacobian gives the
        twist of the body frame referred to the world origin, whose linear
        part is not the tool point's velocity. Against a finite difference the
        two disagree by tens of percent at every step size."""
        # The QP carries state between solves, and moving the model out from
        # under it diverges a streaming solve by radians. This is a query, so
        # it puts the configuration back before returning.
        restore = self._joints()
        try:
            self._set_joints(positions_rad)
            columns = [self._robot.get_joint_v_offset(name) for name in JOINT_NAMES]
            return np.asarray(
                self._robot.frame_jacobian(END_EFFECTOR_FRAME, "local_world_aligned")
            )[:, columns]
        finally:
            self._set_joints(restore)

    def forward_kinematics(self, positions_rad: tuple[float, ...]):
        """(position m, quaternion xyzw) of the end effector."""
        self._set_joints(positions_rad)
        return pose_from_matrix(self._robot.get_T_world_frame(END_EFFECTOR_FRAME))

    def inverse_kinematics(
        self,
        seed_rad: tuple[float, ...],
        position,
        orientation,
        *,
        position_tolerance_m: float | None = None,
        orientation_tolerance_rad: float | None = None,
    ) -> tuple[float, ...] | None:
        """Joint radians reaching the pose, or None when no verified solution
        meets the caller's bars.

        `position_tolerance_m` defaults to IK_POSITION_TOLERANCE_M.
        `orientation_tolerance_rad` defaults to no gate at all: five joints
        underactuate the three orientation degrees of freedom, so the solver's
        best orientation is taken rather than gated behind a bar almost no
        reachable pose could meet. A caller that does care states one and gets
        a refusal instead of a quiet approximation."""
        position_bar = _bar(position_tolerance_m, IK_POSITION_TOLERANCE_M)
        orientation_bar = _bar(orientation_tolerance_rad, math.inf)
        target_matrix = matrix_from_pose(position, orientation)
        target_position = tuple(position)
        target_orientation = tuple(orientation)
        solution_rad = tuple(seed_rad)
        for _ in range(IK_MAX_ITERATIONS):
            solution_rad = self._step(solution_rad, target_matrix)
            reached, reached_orientation = self.forward_kinematics(solution_rad)
            if math.dist(reached, target_position) > position_bar:
                continue
            if relative_rotation_rad(reached_orientation, target_orientation) <= orientation_bar:
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
        solution_rad = tuple(seed_rad)
        for _ in range(IK_STREAM_ITERATIONS):
            solution_rad = self._step(solution_rad, target_matrix)
        if not all(math.isfinite(v) for v in solution_rad):
            return None
        return solution_rad

    def _step(self, seed_rad, target_matrix) -> tuple[float, ...]:
        """One QP step from the seed toward the pose, both objectives soft
        at the weights above: the joint radians it lands on. The seed is
        written to the model without updating its kinematics: the QP
        linearises at the configuration the last query left, which within
        a solve is the seed itself, and on the first step of a solve is the
        pose queried last."""
        for name, value in zip(JOINT_NAMES, seed_rad, strict=True):
            self._robot.set_joint(name, float(value))
        self._tip.T_world_frame = target_matrix
        self._tip.configure(END_EFFECTOR_FRAME, "soft", POSITION_WEIGHT, ORIENTATION_WEIGHT)
        self._solver.solve(True)
        self._robot.update_kinematics()
        return self._joints()

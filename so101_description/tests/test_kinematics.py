"""Real placo FK/IK against the mini URDF: the solver wiring, unit
conversions, and the verified-solution gate."""

import math
from pathlib import Path

import numpy as np
import pytest

from so101_description.kinematics import Kinematics
from so101_description.transforms import relative_rotation_rad

URDF = str(Path(__file__).parent / "assets" / "mini_so101.urdf")
HOME = (0.0, 0.0, 0.0, 0.0, 0.0)


@pytest.fixture(scope="module")
def kinematics():
    return Kinematics(URDF)


def test_fk_at_home_matches_link_stack(kinematics):
    position, orientation = kinematics.forward_kinematics(HOME)
    # The chain is a straight stack of z offsets at home.
    assert position == pytest.approx((0.0, 0.0, 0.05 + 0.03 + 0.11 + 0.10 + 0.05 + 0.08))
    assert orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_fk_varies_with_each_positioning_joint(kinematics):
    # Evaluated at a bent posture: at home the chain is coaxial, so the two
    # z-axis joints (shoulder_pan, wrist_roll) would spin the EE in place.
    bent_home = (0.2, -0.4, 0.6, 0.3, 0.0)
    reference, _ = kinematics.forward_kinematics(bent_home)
    for j in range(4):  # wrist_roll only reorients the EE frame on this chain
        nudged = tuple(q + (0.3 if i == j else 0.0) for i, q in enumerate(bent_home))
        position, _ = kinematics.forward_kinematics(nudged)
        assert not all(
            math.isclose(a, b, abs_tol=1e-9) for a, b in zip(position, reference)
        ), f"joint {j} did not move the end effector"


def test_ik_reaches_a_known_fk_pose(kinematics):
    target_joints = (0.3, -0.4, 0.6, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(target_joints)
    solution = kinematics.inverse_kinematics(HOME, position, orientation)
    assert solution is not None
    reached, _ = kinematics.forward_kinematics(solution)
    assert math.dist(reached, position) <= 0.01


def test_unreachable_pose_is_refused(kinematics):
    # Twice the arm's full extension.
    solution = kinematics.inverse_kinematics(HOME, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
    assert solution is None


def test_streaming_solver_steps_toward_a_reachable_pose(kinematics):
    target_joints = (0.3, -0.4, 0.6, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(target_joints)
    joints = HOME
    for _ in range(60):  # a second of 60 Hz streaming
        joints = kinematics.inverse_kinematics_streaming(joints, position, orientation)
        assert joints is not None
    reached, _ = kinematics.forward_kinematics(joints)
    assert math.dist(reached, position) <= 0.02


def test_streaming_solver_tracks_the_workspace_boundary(kinematics):
    # Far out of reach: the stream must keep returning a finite best-effort
    # configuration (boundary tracking), never None, never non-finite.
    joints = HOME
    for _ in range(30):
        joints = kinematics.inverse_kinematics_streaming(
            joints, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)
        )
        assert joints is not None
        assert all(math.isfinite(j) for j in joints)


def test_ik_accepts_the_best_orientation_for_an_underactuated_pose(kinematics):
    # Five joints underactuate the three orientation degrees of freedom, so
    # acceptance is positional: a reachable position paired with an
    # orientation the chain cannot meet still solves, landing the position
    # and taking the solver's best orientation.
    target_joints = (0.3, -0.5, 0.7, 0.2, 0.0)
    position, reachable_orientation = kinematics.forward_kinematics(target_joints)
    impossible_orientation = (0.0, 0.0, 0.0, 1.0)
    assert reachable_orientation != pytest.approx(impossible_orientation)
    solution = kinematics.inverse_kinematics(HOME, position, impossible_orientation)
    assert solution is not None
    reached, _ = kinematics.forward_kinematics(solution)
    assert math.dist(reached, position) <= 0.01


def rotated(orientation, axis, degrees):
    """`orientation` turned about one of its own axes."""
    half = math.radians(degrees) / 2.0
    unit = np.array(axis, float) / np.linalg.norm(axis)
    d = np.array([*(unit * math.sin(half)), math.cos(half)])
    x1, y1, z1, w1 = orientation
    x2, y2, z2, w2 = d
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def test_an_impossible_orientation_does_not_cost_the_position(kinematics):
    # Five joints cannot turn the grasp point about its own x axis, and both
    # pose objectives are soft, so the solver will trade position away to
    # chase an orientation it can never reach. A move asks first for a place
    # to be: the position it was given must survive the attempt.
    seed = (0.0, 0.3, 0.4, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(seed)
    solution = kinematics.inverse_kinematics(
        seed, position, rotated(orientation, (1, 0, 0), 20)
    )
    assert solution is not None
    reached, _ = kinematics.forward_kinematics(solution)
    assert math.dist(reached, position) < 0.001


def test_an_impossible_orientation_does_not_refuse_a_reachable_position(kinematics):
    # The position here is reachable by construction. Weighted hard enough,
    # the orientation objective drags the solve outside the position
    # tolerance and the caller is told the pose is unreachable, naming the
    # wrong cause.
    seed = (0.0, 0.3, 0.4, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(seed)
    assert kinematics.inverse_kinematics(
        seed, position, rotated(orientation, (1, 0, 0), 90)
    ) is not None


def test_a_reachable_orientation_is_still_honoured(kinematics):
    # The orientation weight is small, not absent: dropping it to zero would
    # abandon the wrist orientations this arm can actually hold.
    seed = (0.0, 0.3, 0.4, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(seed)
    target = rotated(orientation, (0, 0, 1), 20)
    solution = kinematics.inverse_kinematics(seed, position, target)
    assert solution is not None
    _, reached = kinematics.forward_kinematics(solution)
    assert math.degrees(relative_rotation_rad(reached, target)) < 1.0


def test_the_deployed_arm_still_reaches_an_orientation_it_can_hold():
    # Against the real model, not the mini URDF: the seed already satisfies
    # the requested position, so a solve that stopped at the first step
    # inside the position tolerance would return before the orientation
    # objective had moved the wrist at all.
    from so101_description.model import KINEMATICS_URDF_PATH

    kinematics = Kinematics(KINEMATICS_URDF_PATH)
    seed = (0.0, 0.3, 0.4, 0.2, 0.0)
    position, orientation = kinematics.forward_kinematics(seed)
    target = rotated(orientation, (0, 0, 1), 20)
    solution = kinematics.inverse_kinematics(seed, position, target)
    assert solution is not None
    reached_position, reached_orientation = kinematics.forward_kinematics(solution)
    assert math.degrees(relative_rotation_rad(reached_orientation, target)) < 1.0
    assert math.dist(reached_position, position) < 0.001

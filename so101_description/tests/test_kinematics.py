"""Real placo FK/IK against the mini URDF: the solver wiring, unit
conversions, and the verified-solution gate."""

import math
from pathlib import Path

import pytest

from so101_description.kinematics import Kinematics

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

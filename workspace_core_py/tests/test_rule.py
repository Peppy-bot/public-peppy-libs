"""workspace_core's reach rule over a solver written in Python, and the clearance of an SO-101's
arm over a surface."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from workspace_core_py import (
    GRASP_DIRECTIONS,
    REACH_TOLERANCE,
    ArmBodies,
    Reach,
    Solves,
    SurfaceHeight,
    reach,
)

SO101 = Path(__file__).resolve().parents[2] / "robot_description" / "models" / "so101"
TARGET = (0.25, 0.0, 0.05)


class OneArm:
    """A solver of one arm, "arm", whose grasp point stops `shortfall` short of any target and
    whose pose in any orientation stands at the target, pointing the orientation's approach, at
    `joints`; it finds no joints for a pose when `finds` is false. It records each call."""

    def __init__(self, solves=Solves.APPROACHES, shortfall=0.0, finds=True, joints=(0.0,) * 5):
        self._solves = solves
        self._shortfall = shortfall
        self._finds = finds
        self._joints = joints
        self.calls = []

    def solves(self):
        return self._solves

    def arm_names(self):
        return ["arm"]

    def arm_base(self, arm):
        return (0.0, 0.0, 0.0)

    def reach_bound(self, arm, target):
        self.calls.append(("bound", arm))
        return self._shortfall

    def position_shortfall(self, arm, target):
        self.calls.append(("shortfall", arm))
        return self._shortfall

    def reach_pose(self, arm, target, rotation):
        self.calls.append(("pose", arm))
        if not self._finds:
            return None
        approach = (rotation[2], rotation[5], rotation[8])
        return (self._joints, target, approach)


def test_an_arm_whose_pose_stands_at_the_target_reaches_it():
    solver = OneArm()
    assert reach(solver, TARGET) == Reach.reached_by("arm")
    assert solver.calls == [("bound", 0), ("pose", 0)]


def test_an_arm_that_stops_beyond_the_tolerance_tries_no_pose_and_falls_short_by_it():
    solver = OneArm(shortfall=2 * REACH_TOLERANCE)
    assert reach(solver, TARGET) == Reach.short(2 * REACH_TOLERANCE)
    assert solver.calls == [("bound", 0), ("shortfall", 0)]


def test_a_solver_of_approaches_tries_each_grasp_direction_once_and_one_of_full_poses_each_roll():
    approaches = OneArm(finds=False)
    reach(approaches, TARGET)
    assert approaches.calls.count(("pose", 0)) == len(GRASP_DIRECTIONS)
    full = OneArm(solves=Solves.FULL_POSES, finds=False)
    reach(full, TARGET)
    assert full.calls.count(("pose", 0)) == 8 * len(GRASP_DIRECTIONS)


def test_what_a_solver_raises_is_raised():
    class Raises(OneArm):
        def reach_bound(self, arm, target):
            raise RuntimeError("the solver failed")

    with pytest.raises(RuntimeError, match="the solver failed"):
        reach(Raises(), TARGET)


def _so101_ready():
    srdf = ET.parse(SO101 / "so101.srdf").getroot()
    state = next(
        s for s in srdf.iter("group_state") if s.get("name") == "ready" and s.get("group") == "arm"
    )
    return tuple(float(joint.get("value")) for joint in state.iter("joint"))


def _so101_bodies():
    return ArmBodies(
        (SO101 / "so101.urdf").read_text(), (SO101 / "so101.srdf").read_text(), str(SO101 / "meshes")
    )


def test_an_so101_at_ready_clears_its_own_mount_and_sinks_into_a_surface_above_its_arm():
    bodies = _so101_bodies()
    at_ready = OneArm(joints=_so101_ready())
    assert bodies.reach_over(at_ready, SurfaceHeight.from_wire(0.0), TARGET) == Reach.reached_by("arm")
    assert bodies.reach_over(at_ready, SurfaceHeight.from_wire(0.5), TARGET) == Reach.short(0.0)


def test_a_pose_of_another_count_of_joints_than_the_arm_has_is_refused():
    bodies = _so101_bodies()
    six = OneArm(joints=(0.0,) * 6)
    with pytest.raises(ValueError, match="reach_pose gave 6 joint positions for an arm of 5 joints"):
        bodies.reach_over(six, SurfaceHeight.from_wire(0.0), TARGET)


def test_a_description_that_does_not_parse_is_refused():
    with pytest.raises(ValueError):
        ArmBodies("<robot/>", "<robot/>", str(SO101 / "meshes"))

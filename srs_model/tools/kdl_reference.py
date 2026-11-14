"""Prints the KDL reference tables of srs_model's tests/dynamics.rs: the gravity and the
Coriolis torques of each arm of the OpenArm v1.0, the fixture arm, as robot_description
bundles it, at the postures and velocities of tests/dynamics.rs, under its gravity, each
read from its const: GRAVITY_POSTURES, CORIOLIS_CASES and GRAVITY_MAGNITUDE.

Each arm is an arm group of the SRDF, a group of one chain, from its base link through
the ARM_DOF joints of srs_model's src/lib.rs, with every link below the last of them
folded into its link, the gripper fingers at home. A table names its arm by the side the
tests name it by, the arm group's name less `_arm`. KDL's recursive Newton-Euler inverse
dynamics (`ChainIdSolver_RNE`) gives the torques: with gravity, the arm still, for the
gravity tables, and without gravity, at each velocity, for the Coriolis ones. Each torque
is written to four decimals, and one under the tests' 1e-3 Nm tolerance as 0.0.

It runs in the image of the ROS reference harness, which carries KDL's Python bindings,
from the repository root:

    docker build --quiet ros_reference > /tmp/ros_reference.iid
    docker run --rm --network none --volume "$PWD:/repository:ro" \\
        "$(cat /tmp/ros_reference.iid)" bash -c \\
        'source /opt/ros/lyrical/setup.bash && python3 /repository/srs_model/tools/kdl_reference.py /repository'
"""

import ast
import math
import re
import sys
from pathlib import Path

import PyKDL as kdl
from srdfdom.srdf import SRDF
from urdf_parser_py.urdf import URDF

#: The fixture's model id.
MODEL = "openarm_v1"

#: The torque under which a table writes 0.0: the tests' tolerance, Nm.
TOLERANCE_NM = 1e-3


def frame(origin):
    """The frame a URDF origin gives: its rotation by roll, pitch and yaw, then its
    translation."""
    xyz = origin.xyz if origin is not None and origin.xyz is not None else (0.0, 0.0, 0.0)
    rpy = origin.rpy if origin is not None and origin.rpy is not None else (0.0, 0.0, 0.0)
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def inertia(link):
    """The inertia of `link` in its frame, as kdl_parser reads a URDF inertial: the
    rotational inertia about the center of mass turned into the link's axes."""
    inertial = link.inertial
    if inertial is None:
        return kdl.RigidBodyInertia()
    origin = frame(inertial.origin)
    tensor = inertial.inertia
    about_com = kdl.RotationalInertia(
        tensor.ixx, tensor.iyy, tensor.izz, tensor.ixy, tensor.ixz, tensor.iyz
    )
    turned = kdl.Frame(origin.M) * kdl.RigidBodyInertia(0.0, kdl.Vector(), about_com)
    return kdl.RigidBodyInertia(inertial.mass, origin.p, turned.getRotationalInertia())


class Robot:
    """The fixture's URDF, its links and joints by name."""

    def __init__(self, urdf):
        self.links = {link.name: link for link in urdf.links}
        self.parent_joint = {joint.child: joint for joint in urdf.joints}
        self.children = {}
        for joint in urdf.joints:
            self.children.setdefault(joint.parent, []).append(joint)

    def world(self, link):
        """The pose of `link` in the frame of the URDF root, over fixed joints."""
        joint = self.parent_joint.get(link)
        if joint is None:
            return kdl.Frame.Identity()
        assert joint.type == "fixed", f"{joint.name} carries {link} and moves"
        return self.world(joint.parent) * frame(joint.origin)

    def folded(self, link):
        """The inertia of `link` with every link below it, at the zero of every joint
        below it, in its frame."""
        total = inertia(self.links[link])
        for joint in self.children.get(link, ()):
            total = total + frame(joint.origin) * self.folded(joint.child)
        return total

    def arm_joints(self, base, tip):
        """The joints from `base` down to `tip`, base to tip."""
        joints = []
        link = tip
        while link != base:
            joint = self.parent_joint[link]
            joints.append(joint)
            link = joint.parent
        return joints[::-1]


def arm_chain(robot, base, tip, dof):
    """The KDL chain of the arm from `base` to `tip`, its `dof` moving joints, with every
    link below the last of them folded into its link."""
    moving = [joint for joint in robot.arm_joints(base, tip) if joint.type != "fixed"]
    assert len(moving) == dof, f"{base} to {tip} moves {len(moving)} joints, not {dof}"
    assert all(joint.type == "revolute" for joint in moving), "an arm of revolute joints"
    chain = kdl.Chain()
    for joint in moving:
        origin = frame(joint.origin)
        axis = origin.M * kdl.Vector(*joint.axis)
        body = (
            robot.folded(joint.child)
            if joint is moving[-1]
            else inertia(robot.links[joint.child])
        )
        rotating = kdl.Joint(joint.name, origin.p, axis, kdl.Joint.RotAxis)
        chain.addSegment(kdl.Segment(joint.child, rotating, origin, body))
    return chain


def torques(chain, gravity, positions, velocities):
    """The joint torques of `chain` at `positions` and `velocities`, accelerating no
    joint, under `gravity`."""
    count = chain.getNrOfJoints()
    q, qd, qdd, tau = (kdl.JntArray(count) for _ in range(4))
    for index in range(count):
        q[index], qd[index] = positions[index], velocities[index]
    wrenches = [kdl.Wrench() for _ in range(chain.getNrOfSegments())]
    kdl.ChainIdSolver_RNE(chain, gravity).CartToJnt(q, qd, qdd, wrenches, tau)
    return [tau[index] for index in range(count)]


def written(row):
    """A row of a table as tests/dynamics.rs writes it."""
    values = ("0.0" if abs(value) < TOLERANCE_NM else f"{value:.4f}" for value in row)
    return "[" + ", ".join(values) + "],"


def rust_const(path, name, names):
    """The value of the top-level const `name` that the Rust file at `path` declares, as
    Python reads its literal: numbers in arrays and tuples, each Rust name of `names`
    standing for its value there, and a repeat `[x; n]` for n copies of x."""
    declared = re.search(
        rf"^(?:pub )?const {name}: [^=]+= (.+?);$", path.read_text(), re.MULTILINE | re.DOTALL
    )
    if declared is None:
        raise SystemExit(f"{path} declares no const {name}")
    literal = declared.group(1)
    for rust_name, value in names.items():
        literal = re.sub(rf"\b{rust_name}\b", repr(value), literal)
    literal = re.sub(
        r"\[([^\[\];]+); (\d+)\]",
        lambda repeat: "[" + ", ".join([repeat.group(1)] * int(repeat.group(2))) + "]",
        literal,
    )
    return ast.literal_eval(literal)


def arm_groups(srdf):
    """The arm groups of `srdf`, in its order: each group of one chain."""
    return [group for group in srdf.groups if len(group.chains) == 1]


def main():
    repository = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.cwd()
    srs_model = repository / "srs_model"
    dof = rust_const(srs_model / "src" / "lib.rs", "ARM_DOF", {})
    dynamics = srs_model / "tests" / "dynamics.rs"
    names = {"FRAC_PI_2": math.pi / 2, "ARM_DOF": dof}
    gravity_postures = rust_const(dynamics, "GRAVITY_POSTURES", names)
    coriolis_cases = rust_const(dynamics, "CORIOLIS_CASES", names)
    downward = kdl.Vector(0.0, 0.0, -rust_const(dynamics, "GRAVITY_MAGNITUDE", names))
    model = repository / "robot_description" / "models" / MODEL
    robot = Robot(URDF.from_xml_string((model / f"{MODEL}.urdf").read_text()))
    srdf = SRDF.from_xml_string((model / f"{MODEL}.srdf").read_bytes())
    still = (0.0,) * dof
    for group in arm_groups(srdf):
        (ends,) = group.chains
        tip = next(
            joint.child
            for joint in reversed(robot.arm_joints(ends.base_link, ends.tip_link))
            if joint.type != "fixed"
        )
        chain = arm_chain(robot, ends.base_link, tip, dof)
        gravity = robot.world(ends.base_link).M.Inverse() * downward
        side = group.name.removesuffix("_arm")
        print(f"gravity_{side}_matches_kdl")
        for posture in gravity_postures:
            print(written(torques(chain, gravity, posture, still)))
        print(f"coriolis_{side}_matches_kdl")
        for posture, velocity in coriolis_cases:
            print(written(torques(chain, kdl.Vector(0.0, 0.0, 0.0), posture, velocity)))


if __name__ == "__main__":
    main()

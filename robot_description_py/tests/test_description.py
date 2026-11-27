"""A description from Python: read from its two files, its arms with their
grippers, its named states, its cameras, its lidars, its joint limits and its
transforms."""

import xml.etree.ElementTree as ET

import pytest

from robot_description_py import (
    CLOSED,
    HOME,
    OPEN,
    READY,
    START,
    FingerSpan,
    Mimic,
    NoDifferentialBase,
    RobotDescription,
    model,
)


def test_files_of_two_robots_do_not_make_one_description():
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(model("so101").urdf, model("openarm_v2").srdf)
    assert "the SRDF is of the robot" in str(refusal.value)


def test_a_file_that_does_not_read_is_refused_naming_its_file():
    so101 = model("so101")
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(so101.urdf, "<robot")
    assert str(refusal.value).startswith("the SRDF does not parse:")


def test_a_parsed_description_is_the_bundled_one():
    so101 = model("so101")
    parsed = RobotDescription.parse(so101.urdf, so101.srdf)
    assert [arm.name for arm in parsed.arms] == [arm.name for arm in so101.description().arms]
    assert (parsed.urdf, parsed.srdf) == (so101.urdf, so101.srdf)


def test_each_arm_gives_its_chain_and_its_gripper():
    description = model("openarm_v2").description()
    for arm, side in zip(description.arms, ["left", "right"], strict=True):
        assert arm.name == f"{side}_arm"
        assert arm.joints == tuple(f"openarm_{side}_joint{index}" for index in range(1, 8))
        assert arm.gripper.name == f"{side}_gripper"
        fingers = arm.gripper.fingers
        assert [finger.joint for finger in fingers] == [
            f"openarm_{side}_finger_joint1",
            f"openarm_{side}_finger_joint2",
        ]
        closed = description.state(arm.gripper.name, CLOSED)
        opened = description.state(arm.gripper.name, OPEN)
        assert fingers[0].span.closed == closed[0] and fingers[0].span.open == opened[0]
        # The second finger follows the first one to one, as the URDF's <mimic> gives it.
        assert fingers[0].mimic is None
        mimic = fingers[1].mimic
        assert mimic is not None
        assert (mimic.joint, mimic.multiplier, mimic.offset) == (fingers[0].joint, 1.0, 0.0)


def test_a_span_places_its_joint_between_closed_and_open():
    description = model("so101").description()
    (arm,) = description.arms
    span = arm.gripper.fingers[0].span
    for opening in (-0.5, 0.0, 0.25, 1.0, 1.5):
        assert span.opening_at(span.position_at(opening)) == pytest.approx(opening)
    assert span.position_at(0.0) == span.closed and span.position_at(1.0) == pytest.approx(span.open)
    assert span.travel == span.open - span.closed
    assert FingerSpan(span.closed, span.open) == span
    mirrored = FingerSpan(0.0, -1.5708)
    assert mirrored.position_at(0.5) == pytest.approx(-0.7854)


@pytest.mark.parametrize("ends", [(0.3, 0.3), (0.0, 1e-10), (float("nan"), 0.3), (0.0, float("inf"))])
def test_a_span_needs_two_finite_ends_far_enough_apart(ends):
    with pytest.raises(ValueError, match="a finger span stands at two finite positions"):
        FingerSpan(*ends)


def test_the_transforms_place_every_link_but_the_root():
    description = model("so101").description()
    (arm,) = description.arms
    positions = dict(zip(arm.joints, [0.1] * len(arm.joints), strict=True))
    positions.update(arm.gripper.finger_positions_at(0.5))
    transforms = description.fixed_transforms() + description.transforms_at(positions)
    children = [transform.child for transform in transforms]
    assert len(children) == len(set(children))
    assert "base_link" not in children
    (jaw,) = arm.gripper.fingers
    for opening, state in [(0.0, CLOSED), (1.0, OPEN)]:
        values = description.state(arm.gripper.name, state)
        assert len(values) == 1
        assert arm.gripper.finger_positions_at(opening) == {jaw.joint: pytest.approx(values[0])}
    for transform in transforms:
        x, y, z, w = transform.orientation
        assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
        assert len(transform.position) == 3


def test_a_mimic_places_its_joint_off_the_one_it_follows():
    mimic = Mimic("leader", -1.0, 0.01)
    assert mimic.position_from(0.02) == pytest.approx(-0.01)
    assert mimic == Mimic(joint="leader", multiplier=-1.0, offset=0.01)


@pytest.mark.parametrize("values", [(float("nan"), 0.0), (1.0, float("inf"))])
def test_a_mimic_needs_a_finite_multiplier_and_offset(values):
    with pytest.raises(ValueError, match="a mimic's multiplier and offset are finite"):
        Mimic("leader", *values)


def test_a_state_gives_one_position_per_joint_and_an_unknown_one_raises():
    description = model("so101").description()
    (arm,) = description.arms
    for posture in [READY, HOME, START]:
        assert len(description.state(arm.name, posture)) == len(arm.joints)
    for group, state in [(arm.name, "no_such_state"), ("no_such_group", READY)]:
        refusal = f'the SRDF gives the group "{group}" no state named "{state}"'
        with pytest.raises(KeyError, match=refusal):
            description.state(group, state)


def test_each_camera_gives_its_optical_frame():
    description = model("openarm_v2").description()
    cameras = {camera.name: camera for camera in description.cameras}
    assert set(cameras) == {"wrist_left", "wrist_right", "chest"}
    assert cameras["chest"].parent_link == "openarm_body_link0"
    assert {camera.frame for camera in cameras.values()} == {
        "wrist_left_optical_frame",
        "wrist_right_optical_frame",
        "chest_optical_frame",
    }
    for camera in cameras.values():
        assert len(camera.xyz) == 3


def test_the_botshare_base_gives_its_lidar_on_its_scan_frame():
    (lidar,) = model("botshare_base").description().lidars
    assert (lidar.name, lidar.frame) == ("lidar", "lidar_frame")
    assert (lidar.range.min, lidar.range.max) == (0.05, 40.0)
    assert model("openarm_v2").description().lidars == ()


@pytest.mark.parametrize(
    ("name", "root"), [("openarm_v1", "base_link"), ("openarm_v2", "base_link"), ("so101", "base_link")]
)
def test_the_root_link_is_the_robot_frame(name, root):
    assert model(name).description().root_link == root


@pytest.mark.parametrize("name", ["openarm_v1", "openarm_v2", "so101"])
def test_an_arm_carries_its_grasp_link_and_fingers_and_none_carries_the_root(name):
    description = model(name).description()
    assert description.arm_carrying(description.root_link) is None
    for arm in description.arms:
        for link in [arm.grasp_link, *(finger.link for finger in arm.gripper.fingers)]:
            carrier = description.arm_carrying(link)
            assert carrier is not None and carrier.name == arm.name, link
    with pytest.raises(KeyError, match='the URDF has no link named "no_such_link"'):
        description.arm_carrying("no_such_link")


def test_an_arm_carries_its_wrist_camera_and_none_carries_the_chest_camera():
    description = model("openarm_v2").description()
    carriers = {}
    for camera in description.cameras:
        carrier = description.arm_carrying(camera.frame)
        carriers[camera.name] = None if carrier is None else carrier.name
    assert carriers == {"wrist_left": "left_arm", "wrist_right": "right_arm", "chest": None}


def test_a_depth_camera_gives_the_depths_its_sensor_clips_at():
    v2 = model("openarm_v2")
    cameras = {camera.name: camera for camera in v2.description().cameras}
    clip = ET.fromstring(v2.urdf).find("gazebo[@reference='chest_optical_frame']/sensor/camera/clip")
    assert clip is not None
    depth = cameras["chest"].depth_range
    assert depth is not None
    assert (depth.near, depth.far) == (float(clip.findtext("near")), float(clip.findtext("far")))
    assert cameras["wrist_left"].depth_range is None and cameras["wrist_right"].depth_range is None


def test_a_depth_sensor_that_does_not_read_is_refused_naming_it():
    v2 = model("openarm_v2")
    with pytest.raises(ValueError) as refusal:
        RobotDescription.parse(v2.urdf.replace("<far>", "<far>far", 1), v2.srdf)
    assert 'the <far> of the depth sensor on "chest_optical_frame"' in str(refusal.value)


def test_joint_limits_are_the_urdfs():
    so101 = model("so101")
    description = so101.description()
    joints = {joint.get("name"): joint for joint in ET.fromstring(so101.urdf).findall("joint")}
    for arm in description.arms:
        for joint in arm.joints:
            limit = joints[joint].find("limit")
            assert limit is not None
            limits = description.joint_limits(joint)
            assert (limits.lower, limits.upper) == (float(limit.get("lower")), float(limit.get("upper")))
            assert limits.velocity == float(limit.get("velocity"))
            assert limits.effort == float(limit.get("effort"))
    with pytest.raises(KeyError, match='the URDF has no joint named "no_such_joint"'):
        description.joint_limits("no_such_joint")


def test_a_base_gives_its_two_wheels_and_an_arm_gives_none():
    base = model("botshare_base").description().differential_base()
    assert (base.left.joint, base.right.joint) == ("left_wheel_joint", "right_wheel_joint")
    assert (base.left.radius_m, base.right.radius_m, base.separation_m) == pytest.approx((0.1, 0.1, 0.4))
    assert (base.left.width_m, base.right.width_m) == pytest.approx((0.046, 0.046))
    with pytest.raises(ValueError) as refusal:
        model("so101").description().differential_base()
    assert str(refusal.value).startswith("its wheels are the two continuous joints that no group")


_BASE = """<robot name="base">
  <link name="base_link"><collision><geometry><box size="0.4 0.3 0.1"/></geometry></collision></link>
  {wheels}
</robot>"""


def _wheel(
    name: str, y: float, axis: str = "0 1 0", x: float = 0.0, radius: float = 0.05, offset: float = 0.0
) -> str:
    """A wheel `name` on a continuous joint at `x` and `y` m along the base link's x and y,
    turning about `axis`, its link a cylinder of `radius` about the joint's axis, standing
    `offset` m off it along x."""
    return f"""<link name="{name}_link"><collision><origin rpy="1.5707963267948966 0 0" xyz="{offset} 0 0"/>
      <geometry><cylinder length="0.04" radius="{radius}"/></geometry></collision></link>
    <joint name="{name}" type="continuous"><origin rpy="0 0 0" xyz="{x} {y} 0.05"/>
      <parent link="base_link"/><child link="{name}_link"/><axis xyz="{axis}"/></joint>"""


def _refusal(wheels: str, groups: str = "") -> NoDifferentialBase:
    with pytest.raises(NoDifferentialBase) as refused:
        RobotDescription.parse(
            _BASE.format(wheels=wheels), f'<robot name="base">{groups}</robot>'
        ).differential_base()
    return refused.value


def test_a_refusal_of_a_differential_base_names_its_kind_and_the_joints_it_counted():
    """An arm counts no wheel, one wheel counts itself, a wheel that a group of the SRDF holds
    is set aside apart from the ones counted, and a wheel turned backward breaks the rule of
    the axis, which counts no joint; each is a ValueError with robot_description's message."""
    with pytest.raises(NoDifferentialBase) as arm:
        model("so101").description().differential_base()
    assert (arm.value.kind, arm.value.found, arm.value.grouped) == ("wheel_count", (), ())
    assert isinstance(arm.value, ValueError)
    one = _refusal(_wheel("left", 0.2))
    assert (one.kind, one.found, one.grouped) == ("wheel_count", ("left",), ())
    grouped = _refusal(
        _wheel("left", 0.2) + _wheel("right", -0.2), '<group name="wheels"><joint name="left"/></group>'
    )
    assert (grouped.kind, grouped.found, grouped.grouped) == ("wheel_count", ("right",), ("left",))
    backward = _refusal(_wheel("left", 0.2) + _wheel("right", -0.2, axis="0 -1 0"))
    assert (backward.kind, backward.found, backward.grouped) == ("wheel_axis", (), ())
    assert str(backward).startswith("wheel joint right turns about")


_HUB_CAP_ON_LEFT = """<link name="hub_cap"/>
    <joint name="hub_cap_joint" type="fixed"><origin rpy="0 0 0" xyz="0 0.03 0"/>
      <parent link="left_link"/><child link="hub_cap"/></joint>"""


def test_each_kind_of_a_refusal_is_a_class_attribute_of_the_exception():
    """Each class attribute names the kind that a description breaking its rule raises, and
    the class holds one attribute for each kind."""
    left, right = _wheel("left", 0.2), _wheel("right", -0.2)
    breaking = {
        NoDifferentialBase.WHEEL_COUNT: left,
        NoDifferentialBase.WHEEL_AXIS: left + _wheel("right", -0.2, axis="0 -1 0"),
        NoDifferentialBase.NO_AXLE: left + _wheel("right", -0.2, x=0.1),
        NoDifferentialBase.NO_TRACK: left + _wheel("right", 0.2),
        NoDifferentialBase.AXLE_OFF_ORIGIN: _wheel("left", 0.2, x=0.1) + _wheel("right", -0.2, x=0.1),
        NoDifferentialBase.WHEEL_SHAPE: left + _wheel("right", -0.2, radius=0.0),
        NoDifferentialBase.WHEEL_OFF_AXIS: left + _wheel("right", -0.2, offset=0.01),
        NoDifferentialBase.WHEEL_CARRIES: left + right + _HUB_CAP_ON_LEFT,
    }
    for kind, wheels in breaking.items():
        assert _refusal(wheels).kind == kind
    attributes = {name: value for name, value in vars(NoDifferentialBase).items() if name.isupper()}
    assert sorted(attributes.values()) == sorted(breaking)
    assert attributes["WHEEL_COUNT"] == "wheel_count"


def test_each_link_stands_in_the_root_link_as_the_joints_above_it_at_zero_place_it():
    """The root link first, at the identity; base_link level, where its fixed joint holds it
    0.098 m above the root; and each link fixed to base_link where its joint places it there."""
    description = model("botshare_base").description()
    frames = description.link_frames_at_zero()
    assert len(frames) == len(ET.fromstring(description.urdf).findall("link"))
    root = frames[0]
    assert (root.parent, root.child) == (description.root_link, description.root_link)
    assert (root.position, root.orientation) == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert {frame.parent for frame in frames} == {description.root_link}
    placed = {frame.child: frame for frame in frames}
    base_link = placed["base_link"]
    assert base_link.position == pytest.approx((0.0, 0.0, 0.098))
    assert base_link.orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))
    on_base_link = [t for t in description.fixed_transforms() if t.parent == "base_link"]
    assert on_base_link, "the base's casters, lidar and sensors stand on base_link"
    for joint in on_base_link:
        link = placed[joint.child]
        assert link.position == pytest.approx(
            tuple(a + b for a, b in zip(base_link.position, joint.position))
        ), joint.child
        assert link.orientation == pytest.approx(joint.orientation), joint.child

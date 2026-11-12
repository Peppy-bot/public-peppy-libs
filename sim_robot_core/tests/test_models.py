"""One entry per model: the limbs, cameras and start posture a robot's
description gives it, what the entry checks, and how an engine's own entries
pair with the ones this package ships."""

from __future__ import annotations

import re

import pytest
from robot_description_py import START, RobotDescription, model

from sim_robot_core import models as models_module
from sim_robot_core.errors import ModelEntryError
from sim_robot_core.models import FingerSpan, Models, hold_to_bundled, shipped_entry
from sim_robot_core.pairs import Held

OPENARM_LIMBS = Held(
    arms=frozenset({"left_arm", "right_arm"}),
    grippers=frozenset({"left_gripper", "right_gripper"}),
)
SO101_LIMBS = Held(arms=frozenset({"arm"}), grippers=frozenset({"gripper"}))


def bundled(name: str):
    """The entry of a robot of the bundled model `name`."""
    return shipped_entry(name, model(name).description())


def so101_with_srdf(old: str, new: str) -> RobotDescription:
    """The SO-101's description, its SRDF's text `old` written as `new`."""
    so101 = model("so101")
    assert so101.srdf.count(old) == 1, old
    return RobotDescription.parse(so101.urdf, so101.srdf.replace(old, new))


@pytest.fixture(name="shipped")
def shipped_fixture(tmp_path, monkeypatch):
    """A directory that stands in for the entries this package ships."""
    directory = tmp_path / "shipped"
    directory.mkdir()
    monkeypatch.setattr(models_module, "_ENTRIES_DIR", directory)
    return directory


@pytest.fixture(name="engine")
def engine_fixture(tmp_path):
    """An engine's directory of entries."""
    directory = tmp_path / "engine"
    directory.mkdir()
    return directory


class TestShippedEntries:
    @pytest.mark.parametrize("name", ["openarm_v1", "openarm_v2"])
    def test_an_openarm_names_its_limbs_as_the_robot_answers_to_them(self, name):
        openarm = bundled(name)
        assert openarm.arm_names() == ["left_arm", "right_arm"]
        assert openarm.arm_joint_counts() == [7, 7]
        assert openarm.gripper_names() == ["left_gripper", "right_gripper"]

    def test_only_the_v2_carries_the_camera_rig(self):
        assert bundled("openarm_v1").cameras == ()
        v2 = bundled("openarm_v2")
        assert v2.rgb_camera_names() == ["wrist_left", "wrist_right"]
        assert v2.rgbd_camera_names() == ["chest"]

    def test_the_so101_is_one_arm_one_jaw_and_a_wrist_camera(self):
        so101 = bundled("so101")
        assert (so101.arm_names(), so101.arm_joint_counts()) == (["arm"], [5])
        assert so101.gripper_names() == ["gripper"]
        assert (so101.rgb_camera_names(), so101.rgbd_camera_names()) == (["wrist"], [])

    def test_a_model_with_no_shipped_entry_renders_no_camera(self):
        assert shipped_entry("aloha", model("openarm_v1").description()).cameras == ()

    def test_a_described_camera_with_no_shipped_entry_names_the_file_to_create(self):
        path = models_module._ENTRIES_DIR / "aloha.json5"
        with pytest.raises(
            ModelEntryError,
            match=re.escape(
                "the cameras of the description are ['wrist'], and no entry renders them: "
                f"create {path} with one render per camera"
            ),
        ):
            shipped_entry("aloha", model("so101").description())


class TestFromTheDescription:
    @pytest.mark.parametrize("name", ["openarm_v1", "openarm_v2", "so101"])
    def test_the_arms_and_grippers_are_the_descriptions(self, name):
        description = model(name).description()
        entry = bundled(name)
        assert [(arm.name, arm.joints) for arm in entry.arms] == [
            (arm.name, arm.joints) for arm in description.arms
        ]
        for gripper, arm in zip(entry.grippers, description.arms, strict=True):
            assert gripper.name == arm.gripper.name
            fingers = arm.gripper.fingers
            assert gripper.joints == tuple(finger.joint for finger in fingers)
            for span, finger in zip(gripper.spans, fingers, strict=True):
                assert span.position(0.0) == finger.closed
                assert span.position(1.0) == pytest.approx(finger.open)
            for mimic, finger in zip(gripper.mimics, fingers, strict=True):
                described = finger.mimic
                assert (mimic is None) == (described is None), finger.joint
                if described is not None:
                    assert (mimic.joint, mimic.multiplier, mimic.offset) == (
                        described.joint,
                        described.multiplier,
                        described.offset,
                    )

    @pytest.mark.parametrize(
        ("name", "driven", "followers"),
        [
            ("openarm_v1", ["openarm_left_finger_joint1"], ["openarm_left_finger_joint2"]),
            ("openarm_v2", ["openarm_left_finger_joint1"], ["openarm_left_finger_joint2"]),
            ("so101", ["gripper"], []),
        ],
    )
    def test_a_motor_drives_each_finger_of_its_own_and_every_other_finger_follows_one(
        self, name, driven, followers
    ):
        gripper = bundled(name).grippers[0]
        assert [joint for joint, _ in gripper.driven()] == driven
        assert list(gripper.driven_joints()) == driven
        assert [joint for joint, _ in gripper.followers()] == followers
        spans = dict(zip(gripper.joints, gripper.spans, strict=True))
        assert [span for _, span in gripper.driven()] == [spans[joint] for joint in driven]
        for _, mimic in gripper.followers():
            assert mimic.joint in driven

    @pytest.mark.parametrize("name", ["openarm_v1", "openarm_v2", "so101"])
    def test_a_robot_starts_at_its_start_state_with_its_grippers_closed(self, name):
        description = model(name).description()
        expected = {}
        for arm in description.arms:
            expected.update(zip(arm.joints, description.state(arm.name, START), strict=True))
            expected.update((finger.joint, finger.closed) for finger in arm.gripper.fingers)
        posture = bundled(name).start_posture
        assert posture == expected
        assert set(posture) == set(bundled(name).joints())

    def test_an_arm_with_no_start_state_is_refused(self):
        description = so101_with_srdf('<group_state name="start" group="arm">', '<group_state name="rest" group="arm">')
        with pytest.raises(ModelEntryError, match="^model so101: the description gives the arm 'arm' no start state$"):
            shipped_entry("so101", description)

    def test_a_finger_that_does_not_travel_is_refused(self):
        so101 = model("so101")
        closed = so101.description().arms[0].gripper.fingers[0].closed
        open_value = so101.description().arms[0].gripper.fingers[0].open
        description = so101_with_srdf(f'value="{open_value:g}"', f'value="{closed:g}"')
        with pytest.raises(
            ModelEntryError, match="^model so101: finger joint 'gripper' of 'gripper' closes at .* it has no travel$"
        ):
            shipped_entry("so101", description)



class TestFingerSpan:
    def test_an_opening_sits_on_the_travel_from_closed(self):
        jaw = FingerSpan(closed=-0.174533, travel=1.74533 + 0.174533)
        assert jaw.position(0.0) == pytest.approx(-0.174533)
        assert jaw.position(1.0) == pytest.approx(1.74533)
        assert jaw.opening(jaw.position(0.25)) == pytest.approx(0.25)

    def test_a_finger_that_opens_below_its_closed_position_travels_negative(self):
        mirrored = FingerSpan(closed=0.0, travel=-1.5708)
        assert mirrored.position(0.5) == pytest.approx(-0.7854)


class TestTheMatch:
    def test_every_limb_and_nothing_else_is_a_match(self):
        assert bundled("openarm_v2").mismatch(OPENARM_LIMBS) is None
        assert bundled("so101").mismatch(SO101_LIMBS) is None

    def test_a_camera_may_go_unpaired(self):
        rig = Held(
            arms=OPENARM_LIMBS.arms,
            grippers=OPENARM_LIMBS.grippers,
            rgb_cameras=frozenset({"wrist_left"}),
        )
        assert bundled("openarm_v2").mismatch(rig) is None

    def test_a_missing_limb_names_both_lists(self):
        one_arm = Held(arms=frozenset({"left_arm"}), grippers=OPENARM_LIMBS.grippers)
        assert bundled("openarm_v2").mismatch(one_arm) == (
            "its pairs name arms ['left_arm'], grippers ['left_gripper', 'right_gripper'], "
            "rgb_cameras [], rgbd_cameras [], and the model openarm_v2 has arms ['left_arm', "
            "'right_arm'], grippers ['left_gripper', 'right_gripper'], rgb_cameras "
            "['wrist_left', 'wrist_right'], rgbd_cameras ['chest']"
        )

    def test_an_so101_paired_as_an_openarm_matches_nothing(self):
        so101 = bundled("so101")
        assert not so101.holds_every_limb(OPENARM_LIMBS)
        assert so101.foreign(OPENARM_LIMBS) == OPENARM_LIMBS
        assert "the model so101 has arms ['arm'], grippers ['gripper']" in so101.mismatch(OPENARM_LIMBS)

    def test_a_limb_the_model_lacks_is_foreign_even_beside_every_limb_it_has(self):
        extra = Held(arms=frozenset({"arm", "left_arm"}), grippers=SO101_LIMBS.grippers)
        so101 = bundled("so101")
        assert so101.holds_every_limb(extra)
        assert so101.foreign(extra) == Held(arms=frozenset({"left_arm"}))
        assert so101.mismatch(extra) is not None

    def test_a_camera_of_the_wrong_kind_is_foreign(self):
        depth_wrist = Held(
            arms=SO101_LIMBS.arms, grippers=SO101_LIMBS.grippers, rgbd_cameras=frozenset({"wrist"})
        )
        assert bundled("so101").foreign(depth_wrist) == Held(rgbd_cameras=frozenset({"wrist"}))

    def test_a_camera_on_a_model_with_no_rig_is_foreign(self):
        v1_rig = Held(
            arms=OPENARM_LIMBS.arms,
            grippers=OPENARM_LIMBS.grippers,
            rgbd_cameras=frozenset({"chest"}),
        )
        assert bundled("openarm_v1").mismatch(v1_rig) is not None


class TestAnEnginesModels:
    def test_an_engine_stands_the_models_it_has_an_entry_for(self, engine):
        (engine / "so101.json5").write_text("{ scene: 'so101/scene.xml' }")
        (engine / "openarm_v2.json5").write_text("{ scene: 'v2.xml', head_camera: true }")
        models = Models.read(engine)
        assert models.names() == ["openarm_v2", "so101"]
        assert models.of("so101").engine == {"scene": "so101/scene.xml"}
        description = model("so101").description()
        assert models.of("so101").entry(description) == shipped_entry("so101", description)

    def test_a_model_the_engine_has_no_entry_for_names_the_ones_it_stands(self, engine):
        (engine / "so101.json5").write_text("{}")
        with pytest.raises(ModelEntryError, match="unknown model 'openarm_v2': this engine stands so101"):
            Models.read(engine).of("openarm_v2")

    def test_a_model_this_package_ships_no_entry_for_has_no_renders(self, engine):
        (engine / "aloha.json5").write_text("{}")
        assert Models.read(engine).of("aloha").renders is None

    def test_an_engine_with_no_entry_is_refused_saying_what_to_write(self, engine):
        with pytest.raises(ModelEntryError, match="holds no model entry: write one <model>.json5 per model"):
            Models.read(engine)

    def test_an_engine_entry_is_an_object(self, engine):
        (engine / "so101.json5").write_text("[]")
        with pytest.raises(ModelEntryError, match="must be an object"):
            Models.read(engine)

    def test_an_engine_entry_that_does_not_read_names_its_file(self, engine):
        (engine / "so101.json5").write_text("{ scene: ")
        with pytest.raises(ModelEntryError, match=r"engine/so101\.json5: does not read as JSON5"):
            Models.read(engine)


class TestTheShippedEntryReadsAtTheEnginesStart:
    """A shipped entry is read once, as the engine reads its models, so a
    typo in it stops the engine before any robot attaches."""

    def test_the_renders_are_read_once_into_typed_values(self, shipped, engine):
        (shipped / "so101.json5").write_text(
            "{ cameras: { wrist: { fovy_deg: 60, color: { width: 8, height: 4 }, fps: 15 } } }"
        )
        (engine / "so101.json5").write_text("{}")
        known = Models.read(engine).of("so101")
        (shipped / "so101.json5").unlink()
        (camera,) = known.entry(model("so101").description()).cameras
        assert (camera.name, camera.fovy_deg, camera.width, camera.height, camera.fps, camera.depth) == (
            "wrist",
            60.0,
            8,
            4,
            15,
            None,
        )

    def test_a_bad_render_is_refused_as_the_engine_reads_its_models(self, shipped, engine):
        (shipped / "so101.json5").write_text(
            "{ cameras: { wrist: { fovy_deg: 60, color: { width: 8, height: 4 }, fps: 0 } } }"
        )
        (engine / "so101.json5").write_text("{}")
        with pytest.raises(ModelEntryError, match=r"so101\.json5: camera 'wrist': fps must be a positive integer"):
            Models.read(engine)

    def test_an_unknown_key_is_refused(self, shipped, engine):
        (shipped / "so101.json5").write_text("{ cameras: {}, arm_gains: {} }")
        (engine / "so101.json5").write_text("{}")
        with pytest.raises(ModelEntryError, match=r"unknown key\(s\) \['arm_gains'\]"):
            Models.read(engine)

    def test_an_entry_without_cameras_is_refused(self, shipped, engine):
        (shipped / "so101.json5").write_text("{}")
        (engine / "so101.json5").write_text("{}")
        with pytest.raises(ModelEntryError, match=r"so101\.json5: model entry has no `cameras`"):
            Models.read(engine)

    def test_an_entry_that_does_not_read_names_its_file(self, shipped, engine):
        (shipped / "so101.json5").write_text("{ cameras: ")
        (engine / "so101.json5").write_text("{}")
        with pytest.raises(ModelEntryError, match=r"shipped/so101\.json5: does not read as JSON5"):
            Models.read(engine)


class TestHeldToTheBundledDescription:
    @pytest.mark.parametrize("name", ["openarm_v1", "openarm_v2", "so101"])
    def test_the_bundled_description_of_its_model_is_held(self, name):
        hold_to_bundled(name, model(name).description())

    def test_another_description_of_the_model_is_refused_naming_both_digests(self):
        bundled = model("so101").description()
        changed = RobotDescription.parse(bundled.urdf, bundled.srdf.replace("<robot", "<robot ", 1))
        with pytest.raises(ModelEntryError) as refusal:
            hold_to_bundled("so101", changed)
        message = str(refusal.value)
        assert message.startswith("model so101: the robot attaches with another description")
        assert re.search(r"its URDF has SHA-256 ([0-9a-f]{64}) and its SRDF ([0-9a-f]{64}), "
                         r"this simulation's URDF \1 and SRDF (?!\2)[0-9a-f]{64}", message), message
        assert message.endswith("from one public-peppy-libs commit")

    def test_the_description_of_another_model_is_refused(self):
        with pytest.raises(ModelEntryError, match="model openarm_v2: the robot attaches with another description"):
            hold_to_bundled("openarm_v2", model("so101").description())

    def test_a_model_robot_description_bundles_none_of_is_refused(self):
        with pytest.raises(ValueError, match="no_such_robot"):
            hold_to_bundled("no_such_robot", model("so101").description())

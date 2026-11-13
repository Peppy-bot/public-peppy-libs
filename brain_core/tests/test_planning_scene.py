"""The planning scene follows the held flags (`Brain.make_scene_follow_holds`):
for each pair of held flags and scene, the writes that make the scene equal
to the flags, in order, and every way a call of the planner fails."""

from types import SimpleNamespace

import pytest
from conftest import FakeCtx, FakeManipulator, make_brain, records_of
from fake_peppygen import LEFT_AT_REST, fake_bindings, fake_planner, limb_states_message, planner_bindings
from fake_peppygen import drop_item as drop_action
from fake_peppygen import grab_item as grab_action
from fake_peppygen import place_item as place_action

from brain_core.handlers import drop_item, grab_item, place_item
from brain_core.ports import Coverage, Detection, ItemShape

LEFT_TCP, RIGHT_TCP = "openarm_left_tcp", "openarm_right_tcp"
LEFT_FINGERS = ["openarm_left_ee_base_link", "openarm_left_ee_link1", "openarm_left_ee_link2"]
IDENTITY = [0.0, 0.0, 0.0, 1.0]
LIME_SHAPE = ItemShape((0.3, -0.2, 0.413), (0.082, 0.071, 0.086), 0.0)


def described(side: str):
    return SimpleNamespace(
        name=f"{side}_arm", gripper_name=f"{side}_gripper", joint_names=[f"openarm_{side}_joint{i}" for i in range(1, 8)],
        joint_lower_limits=[-1.0] * 7, joint_upper_limits=[1.0] * 7, link_names=[], grasp_link=f"openarm_{side}_tcp",
        approach_axis="+z", closing_axis="+y",
        gripper_links=[f"openarm_{side}_ee_base_link", f"openarm_{side}_ee_link1", f"openarm_{side}_ee_link2"],
    )


def scene_object(object_id, position=(0.3, -0.2, 0.413), link_name=None):
    found = SimpleNamespace(id=object_id, position=list(position), orientation=IDENTITY, shapes=[])
    if link_name is not None:
        found.link_name, found.touch_links = link_name, LEFT_FINGERS
    return found


def answers(planner, objects=(), attached=()):
    """The arms, and a scene that holds `objects` free and `attached`."""
    planner.describe_arms.enqueue(SimpleNamespace(arms=[described("left"), described("right")], body_link_names=[]))
    planner.get_planning_scene.enqueue(SimpleNamespace(
        objects=list(objects), attached_objects=list(attached), allowed_pairs=[], clearance_m=0.04, voxel_size_m=0.02,
    ))


def writes(planner) -> list[tuple]:
    """The scene writes in the order they were made, each in a few words."""
    said = []
    for service, request in planner.apply_collision_objects.journal:
        if service == "apply_collision_objects":
            for found in request.objects:
                if found.operation == "add":
                    [shape] = found.shapes
                    said.append(("add", found.id, tuple(found.position), tuple(found.orientation), shape.kind, tuple(shape.dimensions)))
                elif found.operation == "move":
                    said.append(("move", found.id, tuple(found.position)))
                else:
                    said.append(("remove", found.id))
        elif service == "apply_attached_collision_object":
            said.append((request.operation, request.object_id, request.link_name, tuple(request.touch_links)))
        else:
            said.append(("allow" if request.allowed else "forbid", request.name, tuple(request.other_names)))
    return said


async def planner_brain(*, planner=None, manipulator=None):
    """A brain whose planner slot is bound, the left grasp point at rest."""
    planner = planner or fake_planner()
    brain = await make_brain(manipulator=manipulator, bindings=fake_bindings(**planner_bindings(planner)))
    brain.limb_states.add(limb_states_message(1.0, left_gripper=0.3, right_gripper=1.0))
    return brain, planner


def holds_lime(brain, gripper="left_gripper", shape=LIME_SHAPE) -> str:
    [lime] = brain.state.remember([Detection("lime", (0.3, -0.2, 0.45), 0.9, shape=shape)], 1, coverage=Coverage(every_label=True))
    brain.state.set_held(brain.state.grippers[gripper], lime.item_id)
    return lime.item_id


async def test_with_the_planner_slot_vacant_nothing_is_asked_or_written():
    planner = fake_planner(producer=None)
    brain = await make_brain(bindings=fake_bindings(**planner_bindings(planner)))
    holds_lime(brain)
    await brain.make_scene_follow_holds()
    assert planner.describe_arms.polled == [] and planner.get_planning_scene.polled == [] and writes(planner) == []
    plannerless = await make_brain()
    await plannerless.make_scene_follow_holds()


async def test_nothing_held_and_no_object_of_the_brain_writes_nothing():
    brain, planner = await planner_brain()
    answers(planner, objects=[scene_object("table")])
    await brain.make_scene_follow_holds()
    assert writes(planner) == []


async def test_objects_of_the_brain_that_no_gripper_holds_are_detached_and_removed_and_others_stay():
    """A brain that restarted, or a place, a drop or a forget: its old
    objects go, and an object it did not name stays."""
    brain, planner = await planner_brain()
    answers(planner, objects=[scene_object("brain/cup_1-old"), scene_object("table")], attached=[scene_object("brain/lime_1-old", link_name=LEFT_TCP)])
    await brain.make_scene_follow_holds()
    assert writes(planner) == [
        ("detach", "brain/lime_1-old", LEFT_TCP, ()),
        ("remove", "brain/lime_1-old"),
        ("remove", "brain/cup_1-old"),
    ]


async def test_a_held_item_whose_object_is_attached_to_its_arm_writes_nothing():
    brain, planner = await planner_brain()
    item_id = holds_lime(brain)
    answers(planner, attached=[scene_object(f"brain/{item_id}", (0.0, 0.0, -0.01), LEFT_TCP)])
    await brain.make_scene_follow_holds()
    assert writes(planner) == []


async def test_a_held_item_whose_object_stands_on_the_table_is_moved_by_what_the_grasp_point_moved_and_attached():
    brain, planner = await planner_brain()
    item_id = holds_lime(brain)
    answers(planner, objects=[scene_object(f"brain/{item_id}", (0.3, -0.2, 0.413))])
    # The jaws closed 7.5 cm under where the grasp point stands now.
    closed = (LEFT_AT_REST[0][0], LEFT_AT_REST[0][1], LEFT_AT_REST[0][2] - 0.075)
    await brain.make_scene_follow_holds({item_id: closed})
    assert writes(planner) == [
        ("move", f"brain/{item_id}", pytest.approx((0.3, -0.2, 0.488))),
        ("attach", f"brain/{item_id}", LEFT_TCP, tuple(LEFT_FINGERS)),
    ]


async def test_a_held_item_with_no_object_gets_the_box_of_its_shape_at_the_hand():
    """A planner that restarted, or a hold whose object was never added: the
    box goes at the hand, centred on the grasp point, turned as the shape."""
    brain, planner = await planner_brain()
    item_id = holds_lime(brain, shape=ItemShape((0.3, -0.2, 0.413), (0.082, 0.071, 0.086), 1.0))
    answers(planner)
    await brain.make_scene_follow_holds()
    [add, attach] = writes(planner)
    assert add[:3] == ("add", f"brain/{item_id}", LEFT_AT_REST[0])
    assert add[3] == pytest.approx((0.0, 0.0, 0.4794255, 0.8775826)) and add[4:] == ("box", (0.082, 0.071, 0.086))
    assert attach == ("attach", f"brain/{item_id}", LEFT_TCP, tuple(LEFT_FINGERS))


async def test_a_held_item_on_the_table_with_no_close_given_and_an_item_no_look_measured_go_at_the_hand():
    brain, planner = await planner_brain()
    item_id = holds_lime(brain, shape=None)
    answers(planner, objects=[scene_object(f"brain/{item_id}")])
    await brain.make_scene_follow_holds()
    assert writes(planner) == [
        ("add", f"brain/{item_id}", LEFT_AT_REST[0], tuple(IDENTITY), "sphere", (0.05,)),
        ("attach", f"brain/{item_id}", LEFT_TCP, tuple(LEFT_FINGERS)),
    ]


async def test_an_object_attached_to_the_other_arm_moves_to_the_arm_that_holds_its_item():
    brain, planner = await planner_brain()
    item_id = holds_lime(brain)
    answers(planner, attached=[scene_object(f"brain/{item_id}", (0.0, 0.0, -0.01), RIGHT_TCP)])
    await brain.make_scene_follow_holds()
    assert [said[:3] for said in writes(planner)] == [
        ("detach", f"brain/{item_id}", RIGHT_TCP),
        ("remove", f"brain/{item_id}"),
        ("add", f"brain/{item_id}", LEFT_AT_REST[0]),
        ("attach", f"brain/{item_id}", LEFT_TCP),
    ]


async def test_a_scene_the_planner_does_not_give_is_logged_and_nothing_is_written(caplog):
    brain, planner = await planner_brain()
    holds_lime(brain)
    planner.describe_arms.enqueue(SimpleNamespace(arms=[described("left")], body_link_names=[]))
    planner.get_planning_scene.enqueue(TimeoutError("no reply"))
    with records_of(caplog, "brain_core.brain") as log:
        await brain.make_scene_follow_holds()
    assert writes(planner) == []
    assert [record.getMessage() for record in log.records] == [
        "planning scene: not made to follow the holds: get_planning_scene gave no answer: TimeoutError('no reply')"
    ]


async def test_a_write_that_fails_is_logged_and_the_next_writes_are_made(caplog):
    brain, planner = await planner_brain()
    item_id = holds_lime(brain)
    answers(planner, objects=[scene_object("brain/cup_1-old")])
    planner.apply_collision_objects.enqueue(planner.apply_collision_objects.Failure("rejected", "unknown object 'brain/cup_1-old' for remove"))
    with records_of(caplog, "brain_core.brain") as log:
        await brain.make_scene_follow_holds()
    assert [said[0] for said in writes(planner)] == ["remove", "add", "attach"]
    lines = [record.getMessage() for record in log.records]
    assert lines[0] == "planning scene: remove brain/cup_1-old failed: apply_collision_objects failed with rejected: unknown object 'brain/cup_1-old' for remove"
    assert lines[1:] == [
        f"planning scene: add brain/{item_id} at the hand of arm 'left_arm', [0.400, 0.200, 0.900]",
        f"planning scene: attach brain/{item_id} to {LEFT_TCP}",
    ]


async def test_a_held_item_of_an_arm_the_planner_does_not_describe_or_with_no_grasp_point_is_logged_and_left(caplog):
    brain, planner = await planner_brain()
    item_id = holds_lime(brain)
    planner.describe_arms.enqueue(SimpleNamespace(arms=[described("right")], body_link_names=[]))
    planner.get_planning_scene.enqueue(SimpleNamespace(objects=[], attached_objects=[], allowed_pairs=[], clearance_m=0.04, voxel_size_m=0.02))
    with records_of(caplog, "brain_core.brain") as log:
        await brain.make_scene_follow_holds()
    assert writes(planner) == []
    assert [record.getMessage() for record in log.records] == [
        f"planning scene: brain/{item_id} not attached: the motion planner does not describe arm 'left_arm'"
    ]


# The changes of a held flag, each through its method of Brain: the flag,
# then the scene.


def grab_goal(**overrides) -> FakeCtx:
    goal = dict(gripper_name="", item_id="", position=[0.0, 0.0, 0.0], orientation=None, max_effort=0.0)
    goal.update(overrides)
    return FakeCtx(grab_action, **goal)


def seen_lime(brain, shape=LIME_SHAPE) -> str:
    [lime] = brain.state.remember([Detection("lime", (0.3, 0.2, 0.45), 0.9, shape=shape)], 1, coverage=Coverage(every_label=True))
    return lime.item_id


async def test_a_grab_that_holds_sets_the_flag_then_moves_the_items_object_with_the_hand_and_attaches_it():
    closed = (LEFT_AT_REST[0][0], LEFT_AT_REST[0][1], LEFT_AT_REST[0][2] - 0.075)
    brain, planner = await planner_brain(manipulator=FakeManipulator(closed_at=closed))
    item_id = seen_lime(brain)
    answers(planner, objects=[scene_object(f"brain/{item_id}", (0.3, 0.2, 0.413))])
    ctx = grab_goal(gripper_name="left_gripper", item_id=item_id)
    await grab_item.run(brain, ctx)
    assert ctx.completed["success"] is True
    assert brain.state.grippers["left_gripper"].held_item_id == item_id
    assert writes(planner) == [
        ("move", f"brain/{item_id}", pytest.approx((0.3, 0.2, 0.488))),
        ("attach", f"brain/{item_id}", LEFT_TCP, tuple(LEFT_FINGERS)),
    ]


async def test_a_hold_whose_scene_calls_fail_keeps_the_flag_and_the_grab_succeeds(caplog):
    brain, planner = await planner_brain(manipulator=FakeManipulator(closed_at=(0.4, 0.2, 0.825)))
    item_id = seen_lime(brain)
    answers(planner)
    planner.apply_collision_objects.enqueue(TimeoutError("no reply"))
    planner.apply_attached_collision_object.enqueue(planner.apply_attached_collision_object.Failure("rejected", f"no object 'brain/{item_id}'"))
    ctx = grab_goal(gripper_name="left_gripper", item_id=item_id)
    with records_of(caplog, "brain_core.brain") as log:
        await grab_item.run(brain, ctx)
    assert ctx.completed["success"] is True
    assert brain.state.grippers["left_gripper"].held_item_id == item_id
    assert [record.getMessage() for record in log.records if "planning scene" in record.getMessage()] == [
        f"planning scene: add brain/{item_id} at the hand of arm 'left_arm', [0.400, 0.200, 0.900] failed: "
        "apply_collision_objects gave no answer: TimeoutError('no reply')",
        f"planning scene: attach brain/{item_id} to {LEFT_TCP} failed: apply_attached_collision_object failed with rejected: no object 'brain/{item_id}'",
    ]


async def test_a_grab_that_fails_changes_no_flag_and_no_scene():
    brain, planner = await planner_brain(manipulator=FakeManipulator(fail="gripper 'left_gripper' closed on nothing"))
    item_id = seen_lime(brain)
    ctx = grab_goal(gripper_name="left_gripper", item_id=item_id)
    await grab_item.run(brain, ctx)
    assert ctx.completed["success"] is False
    assert not brain.state.grippers["left_gripper"].holding
    assert planner.get_planning_scene.polled == [] and writes(planner) == []


@pytest.mark.parametrize("release", ["place", "drop"])
async def test_a_place_or_a_drop_that_lets_go_clears_the_flag_then_detaches_and_removes_the_items_object(release):
    brain, planner = await planner_brain(manipulator=FakeManipulator())
    item_id = holds_lime(brain)
    answers(planner, attached=[scene_object(f"brain/{item_id}", (0.0, 0.0, -0.01), LEFT_TCP)])
    if release == "place":
        ctx = FakeCtx(place_action, gripper_name="left_gripper", position=[0.3, 0.1, 0.45], orientation=None)
        await place_item.run(brain, ctx)
    else:
        ctx = FakeCtx(drop_action, gripper_name="left_gripper")
        await drop_item.run(brain, ctx)
    assert ctx.completed["success"] is True
    assert not brain.state.grippers["left_gripper"].holding
    assert writes(planner) == [("detach", f"brain/{item_id}", LEFT_TCP, ()), ("remove", f"brain/{item_id}")]
    # A place keeps the item where it set it down; a drop forgets it.
    assert (item_id in brain.state.items) == (release == "place")


@pytest.mark.parametrize("release", ["place", "drop"])
async def test_a_place_or_a_drop_that_fails_keeps_the_flag_and_the_attached_object(release):
    brain, planner = await planner_brain(manipulator=FakeManipulator(fail="the jaws did not open"))
    item_id = holds_lime(brain)
    if release == "place":
        ctx = FakeCtx(place_action, gripper_name="left_gripper", position=[0.3, 0.1, 0.45], orientation=None)
        await place_item.run(brain, ctx)
    else:
        ctx = FakeCtx(drop_action, gripper_name="left_gripper")
        await drop_item.run(brain, ctx)
    assert ctx.completed["success"] is False
    assert brain.state.grippers["left_gripper"].held_item_id == item_id
    assert planner.get_planning_scene.polled == [] and writes(planner) == []

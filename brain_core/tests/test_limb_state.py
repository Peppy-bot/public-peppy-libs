"""The robot's measured limbs: a limb_states message sliced per arm and
per gripper by the contract's rule, the latest snapshot kept, a message
that does not fit dropped, and the wait for a snapshot captured after
another one."""

import asyncio
import logging
import re

import pytest
from conftest import FakeToken, records_of
from fake_peppygen import FakeTopic, LimbStatesMessage, VacantSlot, limb_states_message
from hypothesis import given
from hypothesis import strategies as st

from brain_core.limb_state import NO_SNAPSHOT, VACANT, ArmState, LimbSnapshot, LimbStateStore

WAIT = 10.0


def two_arms(timestamp: float = 1.0) -> LimbStatesMessage:
    """Two arms of seven joints, each joint numbered by its place in the
    message, and two grippers."""
    return LimbStatesMessage(
        timestamp=timestamp,
        arm_names=["left_arm", "right_arm"],
        joints_per_arm=[7, 7],
        joint_positions=[float(i) for i in range(14)],
        positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        orientations=[0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        gripper_names=["left_gripper", "right_gripper"],
        gripper_openings=[0.25, 0.75],
    )


def test_two_arms_of_seven_joints_slice_per_arm():
    snapshot = LimbSnapshot.parse(two_arms())
    assert snapshot.timestamp == 1.0
    assert snapshot.arms == {
        "left_arm": ArmState(tuple(float(i) for i in range(7)), (0.1, 0.2, 0.3), (0.0, 1.0, 0.0, 0.0)),
        "right_arm": ArmState(tuple(float(i) for i in range(7, 14)), (0.4, 0.5, 0.6), (0.0, 0.0, 0.0, 1.0)),
    }


def test_an_opening_is_read_by_the_grippers_name_and_an_unknown_one_is_none():
    snapshot = LimbSnapshot.parse(two_arms())
    assert (snapshot.opening("left_gripper"), snapshot.opening("right_gripper")) == (0.25, 0.75)
    assert snapshot.opening("third_gripper") is None


@pytest.mark.parametrize("field, value, reason", [
    ("joints_per_arm", [7], "joints_per_arm has 1 entries for 2 arms"),
    ("joint_positions", [0.0] * 13, "joint_positions has 13 entries, joints_per_arm adds up to 14"),
    ("positions", [0.0] * 5, "positions has 5 entries for 2 arms"),
    ("orientations", [0.0] * 9, "orientations has 9 entries for 2 arms"),
    ("gripper_openings", [0.5], "gripper_openings has 1 entries for 2 grippers"),
    ("arm_names", ["arm", "arm"], "arm_names repeats a name: ['arm', 'arm']"),
    ("gripper_names", ["claw", "claw"], "gripper_names repeats a name: ['claw', 'claw']"),
])
def test_a_message_whose_arrays_do_not_fit_its_names_is_refused_naming_the_array(field, value, reason):
    message = two_arms()
    setattr(message, field, value)
    with pytest.raises(ValueError, match=f"^{re.escape(reason)}$"):
        LimbSnapshot.parse(message)


def test_the_openings_of_named_grippers_come_in_their_order_or_not_at_all():
    store = LimbStateStore()
    assert store.openings(["left_gripper", "right_gripper"]) == []
    store.add(two_arms())
    assert store.openings(["right_gripper", "left_gripper"]) == [0.75, 0.25]
    # A gripper the snapshot does not name leaves no partial answer.
    assert store.openings(["left_gripper", "third_gripper"]) == []


def test_the_store_has_no_snapshot_until_one_arrives_then_keeps_the_latest():
    store = LimbStateStore()
    assert store.why_unavailable() == NO_SNAPSHOT and store.latest is None
    store.add(two_arms(1.0))
    store.add(two_arms(2.0))
    assert store.why_unavailable() == "" and store.latest.timestamp == 2.0


def test_a_message_that_does_not_parse_is_dropped_and_logged_once_until_another_reason(caplog):
    store = LimbStateStore()
    store.add(two_arms(1.0))
    short = two_arms(2.0)
    short.positions = [0.0]
    with records_of(caplog, "brain_core.limb_state") as log:
        store.add(short)
        store.add(short)
        uneven = two_arms(3.0)
        uneven.gripper_openings = []
        store.add(uneven)
    assert store.latest.timestamp == 1.0
    assert [record.getMessage() for record in log.records] == [
        "a limb_states message was dropped: positions has 1 entries for 2 arms",
        "a limb_states message was dropped: gripper_openings has 0 entries for 2 grippers",
    ]
    assert all(record.levelno == logging.WARNING for record in log.records)


async def test_a_vacant_slot_makes_the_follower_return_and_every_grab_hears_why():
    store = LimbStateStore()
    await asyncio.wait_for(store.run(None, FakeToken(), VacantSlot), WAIT)
    assert store.why_unavailable() == VACANT


async def test_the_follower_keeps_what_the_topic_publishes_until_the_node_stops():
    store, topic, token = LimbStateStore(), FakeTopic(), FakeToken()
    follower = asyncio.create_task(store.run(None, token, topic))
    topic.push(limb_states_message(5.0, left_gripper=0.4))
    snapshot = await asyncio.wait_for(store.newer_than(0.0, WAIT), WAIT)
    assert snapshot.timestamp == 5.0 and snapshot.opening("left_gripper") == 0.4
    token.cancel()
    await asyncio.wait_for(follower, WAIT)


async def test_a_snapshot_already_newer_answers_at_once_and_an_older_one_waits():
    store = LimbStateStore()
    store.add(limb_states_message(5.0, left_gripper=0.4))
    assert (await store.newer_than(4.0, 0.0)).timestamp == 5.0
    # Nothing newer than 5 s comes within no time at all.
    assert await store.newer_than(5.0, 0.0) is None
    waiting = asyncio.create_task(store.newer_than(5.0, WAIT))
    store.add(limb_states_message(5.0, left_gripper=0.3))
    store.add(limb_states_message(6.0, left_gripper=0.2))
    snapshot = await asyncio.wait_for(waiting, WAIT)
    assert (snapshot.timestamp, snapshot.opening("left_gripper")) == (6.0, 0.2)


# Properties of the slicing rule over any robot: any number of arms, of
# any joint count, and any number of grippers.

names = st.lists(st.text(min_size=1, max_size=8), unique=True, max_size=4)
finite = st.floats(allow_nan=False, allow_infinity=False, width=32)


@st.composite
def robots(draw) -> LimbStatesMessage:
    arm_names = draw(names)
    joints_per_arm = [draw(st.integers(min_value=0, max_value=8)) for _ in arm_names]
    gripper_names = draw(names)
    return LimbStatesMessage(
        timestamp=draw(finite),
        arm_names=arm_names,
        joints_per_arm=joints_per_arm,
        joint_positions=draw(st.lists(finite, min_size=sum(joints_per_arm), max_size=sum(joints_per_arm))),
        positions=draw(st.lists(finite, min_size=3 * len(arm_names), max_size=3 * len(arm_names))),
        orientations=draw(st.lists(finite, min_size=4 * len(arm_names), max_size=4 * len(arm_names))),
        gripper_names=gripper_names,
        gripper_openings=draw(st.lists(finite, min_size=len(gripper_names), max_size=len(gripper_names))),
    )


@given(robots())
def test_slicing_hands_each_arm_its_own_entries_and_loses_none(message):
    snapshot = LimbSnapshot.parse(message)
    assert list(snapshot.arms) == message.arm_names
    joints = [value for arm in snapshot.arms.values() for value in arm.joint_positions]
    positions = [value for arm in snapshot.arms.values() for value in arm.position]
    orientations = [value for arm in snapshot.arms.values() for value in arm.orientation]
    assert (joints, positions, orientations) == (message.joint_positions, message.positions, message.orientations)
    for arm, count in zip(snapshot.arms.values(), message.joints_per_arm):
        assert len(arm.joint_positions) == count
    for name, opening in zip(message.gripper_names, message.gripper_openings):
        assert snapshot.opening(name) == opening


@given(robots(), st.sampled_from(["joints_per_arm", "joint_positions", "positions", "orientations", "gripper_openings"]), st.data())
def test_any_array_of_the_wrong_length_is_refused_with_a_value_error(message, field, data):
    values = getattr(message, field)
    wrong = data.draw(st.integers(min_value=0, max_value=len(values) + 5).filter(lambda n: n != len(values)))
    setattr(message, field, (values * (wrong + 1))[:wrong] if values else [0] * wrong)
    with pytest.raises(ValueError):
        LimbSnapshot.parse(message)

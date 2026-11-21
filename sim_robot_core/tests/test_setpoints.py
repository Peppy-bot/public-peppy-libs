"""The setpoints an engine takes on its driven slots: one position per joint, no velocities or one
per joint, every value a number the simulation holds; a gripper's opening and max effort; the
base's setpoint, which belongs to the pair it came on and drives its wheels for the window of its
leader's stream; and the intake that takes each setpoint a pair carries for its robot's part."""

from __future__ import annotations

import asyncio
import math
import re
from types import SimpleNamespace

import pytest
from control_core_py import runtime
from members import member, table
from robot_description_py import model

from sim_robot_core.models import shipped_entry
from sim_robot_core.pairs import ARMS, BASE_NAME, BASES, GRIPPERS, SLOTS, PairTable
from sim_robot_core.registry import Caller, Registry
from sim_robot_core.setpoints import (
    LEADER_STREAMING_WINDOW_S,
    MAX_VALUE,
    GripperSetpoint,
    JointSetpoint,
    Setpoints,
    _BaseSetpoints,
)


class TestTheParse:
    def test_a_setpoint_carries_its_positions_and_its_velocities(self):
        setpoint = JointSetpoint.parse([0.5, -0.5], [1.0, 1.5], 2, "wheels")
        assert (setpoint.positions, setpoint.velocities) == ((0.5, -0.5), (1.0, 1.5))

    def test_an_arms_setpoint_may_carry_no_velocities(self):
        assert JointSetpoint.parse([0.5, -0.5], [], 2, "joints").velocities == ()

    def test_a_bases_setpoint_carries_one_velocity_per_wheel(self):
        setpoint = JointSetpoint.parse_wheels([0.5, -0.5], [1.0, 1.5], 2)
        assert (setpoint.positions, setpoint.velocities) == ((0.5, -0.5), (1.0, 1.5))

    def test_a_bases_setpoint_with_no_velocities_is_refused_saying_to_send_them(self):
        """joint_link's leader of a velocity-mode follower always sends the
        velocities."""
        refusal = (
            "has 2 wheels run in velocity mode, and the command carries no velocities: "
            "send one velocity per wheel"
        )
        with pytest.raises(ValueError, match=f"^{re.escape(refusal)}$"):
            JointSetpoint.parse_wheels([0.5, -0.5], [], 2)

    @pytest.mark.parametrize(
        ("positions", "velocities", "refusal"),
        [
            ([0.0], [], "has 2 wheels, and the command carries 1 positions"),
            ([0.0, 0.0], [1.0], "has 2 wheels, and the command carries 1 velocities"),
            ([math.nan, 0.0], [], "was commanded a position that is not a number"),
            (
                [0.0, 0.0],
                [math.inf, 0.0],
                "was commanded a velocity of inf, and a velocity is a number from -1e+10 to 1e+10",
            ),
            (
                [10 * MAX_VALUE, 0.0],
                [],
                "was commanded a position of 1e+11, and a position is a number from -1e+10 to 1e+10",
            ),
            (
                [0.0, 0.0],
                [0.0, -10 * MAX_VALUE],
                "was commanded a velocity of -1e+11, and a velocity is a number from -1e+10 to 1e+10",
            ),
        ],
    )
    def test_a_setpoint_the_simulation_cannot_hold_is_refused_saying_why(self, positions, velocities, refusal):
        with pytest.raises(ValueError, match=f"^{re.escape(refusal)}$"):
            JointSetpoint.parse(positions, velocities, 2, "wheels")

    def test_a_setpoint_at_the_bound_either_way_is_taken(self):
        JointSetpoint.parse([MAX_VALUE, -MAX_VALUE], [-MAX_VALUE, MAX_VALUE], 2, "joints")

    def test_a_gripper_setpoint_carries_its_opening_and_its_max_effort(self):
        assert GripperSetpoint.parse(0.25, 4.0) == GripperSetpoint(opening=0.25, max_effort=4.0)

    @pytest.mark.parametrize(
        ("opening", "max_effort", "refusal"),
        [
            (math.nan, 0.0, "was commanded an opening that is not a number"),
            (
                0.5,
                math.inf,
                "was commanded a max effort of inf, and a max effort is a number from -1e+10 to 1e+10",
            ),
            (0.5, -1.0, "was commanded a max effort of -1, and a max effort is at least 0"),
        ],
    )
    def test_a_gripper_setpoint_the_simulation_cannot_hold_is_refused_saying_why(
        self, opening, max_effort, refusal
    ):
        with pytest.raises(ValueError, match=f"^{re.escape(refusal)}$"):
            GripperSetpoint.parse(opening, max_effort)


DRIVE = JointSetpoint.parse_wheels([0.0, 0.0], [1.0, 1.5], 2)
BASE_NODE = member("alpha_base_inst", BASE_NAME)


@pytest.fixture(name="bases")
def bases_fixture():
    """The members of the bases slot, which a test changes between two
    readings."""
    return [BASE_NODE]


@pytest.fixture(name="pairs")
def pairs_fixture(bases):
    return table(bases=bases)


class TestTheBasesSetpoint:
    def test_the_wheels_follow_the_latest_setpoint_of_their_pair(self, pairs):
        setpoints = _BaseSetpoints()
        assert setpoints.latest("alpha") is None
        assert setpoints.put("alpha", BASE_NODE.info, DRIVE), "its first setpoint"
        assert not setpoints.put("alpha", BASE_NODE.info, DRIVE)
        assert setpoints.keep_paired(pairs) == []
        assert setpoints.latest("alpha") == DRIVE

    def test_a_reading_that_finds_the_pair_gone_holds_the_wheels_at_rate_0(self, pairs, bases):
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        bases.clear()
        assert not setpoints.take_release("alpha")
        assert setpoints.keep_paired(pairs) == [("alpha", BASE_NODE.info)]
        assert setpoints.latest("alpha") is None
        assert setpoints.take_release("alpha"), "the engine holds the released wheels"
        assert not setpoints.take_release("alpha"), "once"
        assert setpoints.keep_paired(pairs) == [], "a pair is named once"

    def test_a_pair_that_ends_after_its_setpoint_was_dropped_is_named_and_releases_nothing(
        self, pairs, bases
    ):
        """The leader's window released the wheels, and later its pair ends:
        the reading names the pair, and the wheels, held already, are not
        released again."""
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        setpoints.drop_stale("alpha", FIRST_READ_S)
        assert setpoints.drop_stale("alpha", FIRST_READ_S + LEADER_STREAMING_WINDOW_S + 1e-3)
        assert setpoints.take_release("alpha")
        assert setpoints.keep_paired(pairs) == [], "the pair still stands"

        bases.clear()
        assert setpoints.keep_paired(pairs) == [("alpha", BASE_NODE.info)]
        assert not setpoints.take_release("alpha")
        assert setpoints.keep_paired(pairs) == [], "once"

    def test_a_base_set_down_keeps_its_leader_until_its_pair_ends(self, pairs, bases):
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        setpoints.set_down("alpha")
        assert setpoints.latest("alpha") is None
        assert not setpoints.take_release("alpha"), "a set-down releases nothing"

        bases.clear()
        assert setpoints.keep_paired(pairs) == [("alpha", BASE_NODE.info)]
        assert not setpoints.take_release("alpha")

    def test_a_pair_whose_leader_never_sent_is_not_named(self, pairs, bases):
        """Two pairs end in one reading: the one whose leader sent is named,
        and the one whose leader never sent is not."""
        bases.append(member("bravo_base_inst", BASE_NAME, copy="bravo"))
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        assert setpoints.keep_paired(pairs) == [], "both pairs stand"

        bases.clear()
        assert setpoints.keep_paired(pairs) == [("alpha", BASE_NODE.info)]

    def test_a_pair_that_comes_back_drives_nothing_until_its_leader_sends(self, pairs, bases):
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        # The base node came back on another core node between two readings,
        # on a pair of its own.
        again = member("alpha_base_inst", BASE_NAME, core_node="sim17")
        bases[:] = [again]
        assert setpoints.keep_paired(pairs) == [("alpha", BASE_NODE.info)]
        assert setpoints.latest("alpha") is None
        assert setpoints.put("alpha", again.info, DRIVE), "its first setpoint"
        assert not setpoints.take_release("alpha"), "a leader that sends drives the wheels again"
        assert setpoints.keep_paired(pairs) == []
        assert setpoints.latest("alpha") == DRIVE

    def test_a_robot_that_left_drives_nothing(self):
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        setpoints.keep_paired(table())
        setpoints.forget("alpha")
        assert setpoints.latest("alpha") is None
        assert not setpoints.take_release("alpha")

    def test_a_setpoint_put_while_the_pairs_are_read_is_kept(self):
        """A new leader pairs and sends while a reading finds the old pair
        gone: its setpoint came on the pair the slot holds, so it drives."""
        setpoints = _BaseSetpoints()
        setpoints.put("alpha", BASE_NODE.info, DRIVE)
        leader = member("alpha_base_inst", BASE_NAME, core_node="sim17")

        def bases_while_the_leader_sends():
            setpoints.put("alpha", leader.info, DRIVE)
            return ()

        pairs = PairTable(
            {slot: bases_while_the_leader_sends if slot == BASES else tuple for slot in SLOTS},
            lambda: None,
        )
        assert setpoints.keep_paired(pairs) == []
        assert setpoints.latest("alpha") == DRIVE
        assert not setpoints.take_release("alpha")


# An engine time of the first step that reads a setpoint.
FIRST_READ_S = 10.0


class _Subscription:
    """A slot's setpoint subscription: hands over each arrival, in order,
    then ends."""

    def __init__(self, arrivals) -> None:
        self._arrivals = list(arrivals)

    async def next(self):
        return self._arrivals.pop(0) if self._arrivals else None


class _Token:
    """The node's cancellation token, which no test cancels."""

    def is_cancelled(self) -> bool:
        return False

    async def cancelled(self) -> None:
        await asyncio.Event().wait()


ARM_NODE = member("alpha_backbone_inst", "arm")
GRIPPER_NODE = member("alpha_backbone_inst", "gripper")
INDIA_BASE = member("india_base_inst", BASE_NAME, copy="india")


def _intake(**members) -> Setpoints:
    """The setpoints of an SO-101 standing as alpha and a botshare base as
    india, the slots holding `members`, by slot, as `table` reads them."""
    robots = Registry()
    for name, robot_model in (("alpha", "so101"), ("india", "botshare_base")):
        entry = shipped_entry(robot_model, model(robot_model).description())
        robots.admit(name, entry, Caller("sim16", f"{name}_init"))
    return Setpoints(table(**members), robots)


@pytest.fixture(name="intake")
def intake_fixture():
    return _intake(arms=[ARM_NODE], grippers=[GRIPPER_NODE], bases=[INDIA_BASE])


def _joints(sender, positions, velocities=()):
    return sender.info, SimpleNamespace(positions=list(positions), velocities=list(velocities))


def _opening(sender, opening, max_effort=0.0):
    return sender.info, SimpleNamespace(opening=opening, max_effort=max_effort)


def _consume(intake: Setpoints, slot: str, *arrivals) -> None:
    async def subscribe():
        return _Subscription(arrivals)

    asyncio.run(intake.consume(slot, subscribe, _Token()))


class TestTheIntake:
    def test_each_part_takes_the_latest_setpoint_of_its_own_pair(self, intake):
        _consume(intake, ARMS, _joints(ARM_NODE, [0.1] * 5), _joints(ARM_NODE, [0.2] * 5, [1.0] * 5))
        _consume(intake, GRIPPERS, _opening(GRIPPER_NODE, 0.75, 2.0))
        _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.5]))

        assert intake.arm("alpha", "arm") == JointSetpoint((0.2,) * 5, (1.0,) * 5)
        assert intake.gripper("alpha", "gripper") == GripperSetpoint(0.75, 2.0)
        assert intake.base("india", FIRST_READ_S) == JointSetpoint((0.0, 0.0), (1.0, 1.5))

    def test_a_part_the_model_lacks_a_robot_the_scene_lacks_and_a_peer_no_slot_holds_drive_nothing(self):
        alpha_base = member("alpha_base_inst", BASE_NAME)
        india_gripper = member("india_backbone_inst", "gripper", copy="india")
        bravo_arm = member("bravo_backbone_inst", "arm", copy="bravo")
        intake = _intake(arms=[bravo_arm], grippers=[india_gripper], bases=[alpha_base])

        _consume(intake, BASES, _joints(alpha_base, [0.0, 0.0], [1.0, 1.0]))
        _consume(intake, GRIPPERS, _opening(india_gripper, 0.5))
        _consume(intake, ARMS, _joints(bravo_arm, [0.1] * 5), _joints(ARM_NODE, [0.1] * 5))

        assert intake.base("alpha", FIRST_READ_S) is None
        assert intake.gripper("india", "gripper") is None
        assert intake.arm("bravo", "arm") is None
        assert intake.arm("alpha", "arm") is None

    def test_a_part_whose_setpoints_do_not_fit_is_logged_once_until_one_fits(self, intake, caplog):
        unfit = _joints(INDIA_BASE, [0.0], [1.0])
        with caplog.at_level("WARNING"):
            _consume(intake, BASES, unfit, unfit, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.0]), unfit)

        dropped = [record.getMessage() for record in caplog.records if "dropping" in record.getMessage()]
        assert dropped == [
            "dropping the setpoints for 'india' base until one fits: it has 2 wheels, and the command "
            "carries 1 positions"
        ] * 2
        assert intake.base("india", FIRST_READ_S) == JointSetpoint((0.0, 0.0), (1.0, 1.0))

    def test_a_gripper_setpoint_the_parse_refuses_is_logged_as_a_joint_one_is(self, intake, caplog):
        with caplog.at_level("WARNING"):
            _consume(intake, GRIPPERS, _opening(GRIPPER_NODE, math.nan), _opening(GRIPPER_NODE, math.nan))

        assert intake.gripper("alpha", "gripper") is None
        assert [record.getMessage() for record in caplog.records] == [
            "dropping the setpoints for 'alpha' gripper until one fits: it was commanded an opening "
            "that is not a number"
        ]

    def test_a_frame_that_does_not_read_is_logged_in_the_engines_log_once_a_run(
        self, intake, caplog, monkeypatch
    ):
        monkeypatch.setattr(runtime, "_RECEIVE_RETRY_S", 0.0)

        class FailingTwiceThenClosed:
            def __init__(self) -> None:
                self.failures = 2

            async def next(self):
                if self.failures:
                    self.failures -= 1
                    raise RuntimeError("a frame that does not read")
                return None

        async def subscribe():
            return FailingTwiceThenClosed()

        with caplog.at_level("WARNING", logger="sim_robot_core.setpoints"):
            asyncio.run(intake.consume(BASES, subscribe, _Token()))

        assert [record.getMessage() for record in caplog.records] == [
            "bases setpoints receive error: RuntimeError('a frame that does not read'); suppressing repeats"
        ]

    def test_a_base_whose_pair_is_gone_holds_where_it_stands(self):
        bases = [INDIA_BASE]
        intake = _intake(bases=bases)
        _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.0]))

        bases.clear()
        intake.keep_paired_bases()

        assert intake.base("india", FIRST_READ_S) is None
        assert intake.take_base_release("india")

    @pytest.mark.parametrize("released_first", [False, True], ids=["driving", "released"])
    def test_the_end_of_a_base_pair_is_logged_once_naming_its_leader(self, caplog, released_first):
        """In Waldo's words, whether the wheels still followed the leader or
        its window had released them."""
        bases = [INDIA_BASE]
        intake = _intake(bases=bases)
        _drive(intake)
        if released_first:
            intake.base("india", FIRST_READ_S)
            intake.base("india", FIRST_READ_S + LEADER_STREAMING_WINDOW_S + 1e-3)

        bases.clear()
        with caplog.at_level("INFO", logger="sim_robot_core.setpoints"):
            intake.keep_paired_bases()
            intake.keep_paired_bases()

        assert [record.getMessage() for record in caplog.records] == [
            "the base pair of robot 'india' with 'india_base_inst' on core node 'sim16' is gone: "
            "its wheels hold where they stand"
        ]

    def test_a_base_set_down_drives_nothing_until_its_leader_sends(self, intake):
        _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.0]))

        intake.drop_base("india")
        assert intake.base("india", FIRST_READ_S) is None
        _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [0.5, 0.5]))
        assert intake.base("india", FIRST_READ_S) == JointSetpoint((0.0, 0.0), (0.5, 0.5))

    def test_a_robot_that_joins_again_under_a_name_is_told_of_its_own_refusals(self, intake, caplog):
        """The log names a part once until one of its setpoints fits, for the
        robot that sent them: one that left takes its refusals with it."""
        unfit = _joints(INDIA_BASE, [0.0], [1.0])
        with caplog.at_level("WARNING"):
            _consume(intake, BASES, unfit)
            intake.forget("india")
            _consume(intake, BASES, unfit)

        dropped = [record.getMessage() for record in caplog.records if "dropping" in record.getMessage()]
        assert len(dropped) == 2

    def test_a_robot_that_left_takes_every_setpoint_with_it(self, intake):
        _consume(intake, ARMS, _joints(ARM_NODE, [0.1] * 5))
        _consume(intake, GRIPPERS, _opening(GRIPPER_NODE, 0.5))
        _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.0]))

        intake.forget("alpha")
        intake.forget("india")

        assert (intake.arm("alpha", "arm"), intake.gripper("alpha", "gripper")) == (None, None)
        assert intake.base("india", FIRST_READ_S) is None
        assert not intake.take_base_release("india"), "a robot that left holds nothing"


def _drive(intake: Setpoints) -> None:
    """Hands india's base one setpoint on its pair."""
    _consume(intake, BASES, _joints(INDIA_BASE, [0.0, 0.0], [1.0, 1.0]))


class TestTheLeadersWindow:
    """joint_link's 0.5 s, read at the engine times of its steps."""

    def test_the_wheels_follow_a_setpoint_through_the_window_from_its_first_read(self, intake):
        _drive(intake)

        assert intake.base("india", FIRST_READ_S) is not None
        assert intake.base("india", FIRST_READ_S + LEADER_STREAMING_WINDOW_S) is not None
        assert not intake.take_base_release("india")

    def test_past_the_window_with_no_newer_setpoint_the_wheels_are_released(self, intake):
        _drive(intake)
        intake.base("india", FIRST_READ_S)

        assert intake.base("india", FIRST_READ_S + LEADER_STREAMING_WINDOW_S + 1e-3) is None
        assert intake.take_base_release("india"), "the engine holds the released wheels"
        assert intake.base("india", FIRST_READ_S + 1.0) is None
        assert not intake.take_base_release("india"), "once"

    def test_each_newer_setpoint_starts_the_window_again(self, intake):
        _drive(intake)
        intake.base("india", FIRST_READ_S)
        _drive(intake)

        assert intake.base("india", FIRST_READ_S + 0.4) is not None
        assert intake.base("india", FIRST_READ_S + 0.4 + LEADER_STREAMING_WINDOW_S) is not None

    def test_an_engine_clock_that_went_back_starts_the_window_at_its_new_time(self, intake):
        """An engine time below the window's start starts it again there."""
        _drive(intake)
        intake.base("india", FIRST_READ_S)

        assert intake.base("india", 0.0) is not None
        assert intake.base("india", LEADER_STREAMING_WINDOW_S) is not None
        assert intake.base("india", LEADER_STREAMING_WINDOW_S + 1e-3) is None

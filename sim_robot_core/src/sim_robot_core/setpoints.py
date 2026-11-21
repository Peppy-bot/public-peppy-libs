"""The setpoints an engine takes on its three driven slots: an arm's, whose
joints follow their positions, a gripper's, whose fingers follow its
opening, and a base's, whose wheels follow their velocities, as a wheel's
driver runs them in velocity mode.

A joint setpoint carries one position per joint and either no velocities or
one per joint, each a number from -MAX_VALUE to MAX_VALUE, the largest a
simulation holds: one past it is dropped as one that is not a number is. A
base's carries one velocity per wheel, which joint_link's leader of a
velocity-mode follower always sends. A gripper setpoint carries an opening
and a max effort of at least 0, each a number from -MAX_VALUE to MAX_VALUE.

`Setpoints` takes each setpoint a pair carries for a part the robot's model
has, keeps the latest of each part, and drops one the parse refuses, the log
saying why once until one of that part's setpoints fits.

A base's wheels follow as joint_link's follower of a velocity-mode device
follows: from each setpoint that arrives until more than
LEADER_STREAMING_WINDOW_S pass with no setpoint, on the simulation clock, the
engine's own, and then they hold at rate 0. The engine's clock moves at its
steps, so the first step that reads a setpoint starts its window, within a
step of its arrival on that clock. A base's setpoint also belongs to the
pair it came on (BaseSetpoints): a reading of the pairs that finds that pair
gone drops it. Either drop releases the wheels, which an engine then holds
where they stand, the integral of each wheel's rate loop cleared, at rate 0
until a leader sends again; a pair that comes back drives nothing until its
own leader sends.
"""

from __future__ import annotations

import logging
import math
import threading
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass
from typing import TypeVar

from control_core_py.runtime import CancellationToken, messages

from sim_robot_core.pairs import ARMS, BASE_NAME, BASES, GRIPPERS, PairTable
from sim_robot_core.registry import Registry

logger = logging.getLogger(__name__)

# MuJoCo's mjMAXVAL: a value past it is one the simulation has lost.
MAX_VALUE = 1e10

# How long the leader's silence ends its stream (s): joint_link's 0.5 s for a
# follower of a velocity-mode device, drive_link's window. A base's wheels
# follow from each setpoint until more than this passes on the simulation
# clock with no setpoint, and then hold at rate 0.
LEADER_STREAMING_WINDOW_S = 0.5

_Setpoint = TypeVar("_Setpoint")


def _checked(values: Sequence[float], a_value: str) -> tuple[float, ...]:
    """`values`, each a number from -MAX_VALUE to MAX_VALUE; a refusal calls
    each one `a_value` (a position, an opening)."""
    for value in values:
        if math.isnan(value):
            raise ValueError(f"was commanded {a_value} that is not a number")
        if not abs(value) <= MAX_VALUE:
            raise ValueError(
                f"was commanded {a_value} of {value:g}, and {a_value} is a number from "
                f"{-MAX_VALUE:g} to {MAX_VALUE:g}"
            )
    return tuple(float(value) for value in values)


@dataclass(frozen=True)
class JointSetpoint:
    """The setpoint of a part's joints: one position per joint, and either
    no velocities or one per joint."""

    positions: tuple[float, ...]
    velocities: tuple[float, ...]

    @staticmethod
    def parse(
        positions: Sequence[float], velocities: Sequence[float], count: int, joints: str
    ) -> JointSetpoint:
        """The setpoint of a part of `count` joints, which a refusal calls
        `joints` (joints, wheels). A count that is not the part's, and a
        value that is not a number or lies past MAX_VALUE, raise ValueError
        saying what the command got wrong."""
        if len(positions) != count:
            raise ValueError(f"has {count} {joints}, and the command carries {len(positions)} positions")
        if velocities and len(velocities) != count:
            raise ValueError(f"has {count} {joints}, and the command carries {len(velocities)} velocities")
        return JointSetpoint(
            positions=_checked(positions, "a position"),
            velocities=_checked(velocities, "a velocity"),
        )

    @staticmethod
    def parse_wheels(
        positions: Sequence[float], velocities: Sequence[float], wheels: int
    ) -> JointSetpoint:
        """The setpoint of a base of `wheels` wheels, as `parse` takes an
        arm's, with one velocity per wheel: joint_link's leader of a
        velocity-mode follower always sends the velocities, which the wheels
        follow. One that carries none raises ValueError saying to send
        them."""
        setpoint = JointSetpoint.parse(positions, velocities, wheels, "wheels")
        if not setpoint.velocities:
            raise ValueError(
                f"has {wheels} wheels run in velocity mode, and the command carries no "
                "velocities: send one velocity per wheel"
            )
        return setpoint


@dataclass(frozen=True)
class GripperSetpoint:
    """The setpoint of a gripper: the opening its fingers go to, 0 closed and
    1 open, and the effort they may apply, 0 leaving the engine's own
    ceiling as the only one."""

    opening: float
    max_effort: float

    @staticmethod
    def parse(opening: float, max_effort: float) -> GripperSetpoint:
        """A value that is not a number or lies past MAX_VALUE, and a max
        effort below 0, raise ValueError saying what the command got
        wrong."""
        (opening,) = _checked((opening,), "an opening")
        (max_effort,) = _checked((max_effort,), "a max effort")
        if max_effort < 0.0:
            raise ValueError(
                f"was commanded a max effort of {max_effort:g}, and a max effort is at least 0"
            )
        return GripperSetpoint(opening=opening, max_effort=max_effort)


class BaseSetpoints:
    """The latest setpoint of each robot's base, with the peer of the pair it
    came on, and the engine time of the first step that read it. Every
    method holds the store's lock while it reads or writes the store, so any
    thread may call any of them, the thread that steps the scene among
    them."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Each robot's (peer, setpoint), a new tuple for each setpoint put.
        self._latest: dict[str, tuple[object, JointSetpoint]] = {}
        # The engine time of the first step that read each robot's setpoint.
        self._since: dict[str, float] = {}
        # The robots whose wheels a dropped setpoint released, until the
        # engine holds them.
        self._released: set[str] = set()

    def put(self, robot: str, peer, setpoint: JointSetpoint) -> bool:
        """Keeps `setpoint` as the latest of `robot`'s base, from the pair
        whose other end is `peer`. True for the first setpoint since the
        robot's base last had none."""
        with self._lock:
            first = robot not in self._latest
            self._latest[robot] = (peer, setpoint)
            self._since.pop(robot, None)
            self._released.discard(robot)
        return first

    def latest(self, robot: str) -> JointSetpoint | None:
        """The setpoint `robot`'s wheels follow, or None while they hold at
        rate 0."""
        with self._lock:
            held = self._latest.get(robot)
        return None if held is None else held[1]

    def drop_stale(self, robot: str, now_s: float) -> bool:
        """Drops `robot`'s setpoint, and releases its wheels, once `now_s`,
        the engine time of this step, lies more than
        LEADER_STREAMING_WINDOW_S past the first step that read it; True when
        it does. The first step that reads a setpoint starts its window, and
        so does a step whose engine clock went back before that start."""
        with self._lock:
            if robot not in self._latest:
                return False
            since = self._since.get(robot)
            if since is None or now_s < since:
                self._since[robot] = now_s
                return False
            if now_s - since <= LEADER_STREAMING_WINDOW_S:
                return False
            self._drop(robot)
        return True

    def keep_paired(self, pairs: PairTable) -> list[str]:
        """Drops each setpoint whose pair is not the one `pairs` holds for
        its robot's base now, releases its robot's wheels, and names the
        robots whose wheels it released. A setpoint put while the pairs are
        read is kept: it came on the pair the slot holds then."""
        with self._lock:
            seen = dict(self._latest)
        # The pairs are read outside the lock, which a thread stepping the
        # scene takes on every step.
        paired = {robot: pairs.peer_of(BASES, robot, BASE_NAME) for robot in seen}
        with self._lock:
            gone = [
                robot
                for robot, held in seen.items()
                if held[0] != paired[robot] and self._latest.get(robot) is held
            ]
            for robot in gone:
                self._drop(robot)
        return gone

    def take_release(self, robot: str) -> bool:
        """Whether `robot`'s wheels were released since the engine last
        asked, which they are once: the engine then holds them where they
        stand."""
        with self._lock:
            released = robot in self._released
            self._released.discard(robot)
        return released

    def forget(self, robot: str) -> None:
        """Drops what a robot that left the scene had commanded."""
        with self._lock:
            self._latest.pop(robot, None)
            self._since.pop(robot, None)
            self._released.discard(robot)

    def _drop(self, robot: str) -> None:
        """Drops `robot`'s setpoint and releases its wheels; the caller holds
        the lock."""
        del self._latest[robot]
        self._since.pop(robot, None)
        self._released.add(robot)


class Setpoints:
    """The latest setpoint of each arm, gripper and base of each robot, as an
    engine takes them on its three driven slots: the node's loop takes them
    (`consume`), and the thread that steps the scene reads them. Every
    method holds a lock while it reads or writes a store, so any thread may
    call any of them.

    `pairs` reads each slot's pairs into robots and parts, and `robots` gives
    the model entry of each robot the scene holds, whose arms and base say
    how many joints each setpoint carries."""

    def __init__(self, pairs: PairTable, robots: Registry) -> None:
        self._pairs = pairs
        self._robots = robots
        self._lock = threading.Lock()
        # The latest setpoint of each arm and gripper, by (robot, part).
        self._arms: dict[tuple[str, str], JointSetpoint] = {}
        self._grippers: dict[tuple[str, str], GripperSetpoint] = {}
        self._bases = BaseSetpoints()
        # The parts whose setpoints the parse refuses since one of them last
        # fit, by (robot, part): each is logged once. Node loop only.
        self._refusing: set[tuple[str, str]] = set()

    async def consume(
        self, slot: str, subscribe: Callable[[], Awaitable], token: CancellationToken
    ) -> None:
        """Takes each setpoint that arrives on `slot` (ARMS, GRIPPERS or
        BASES) through the subscription `subscribe` opens, a node's generated
        `subscribe(node_runner)`, until the subscription ends or `token`, the
        node's cancellation token, cancels. A setpoint from a pair the slot
        does not hold, or that no robot answers for, goes nowhere. A frame
        that does not read is dropped, and logged once a run of them
        (control_core_py.runtime.messages)."""
        take = {ARMS: self._take_arm, GRIPPERS: self._take_gripper, BASES: self._take_base}[slot]
        subscription = await subscribe()
        async for peer, msg in messages(subscription, token, f"{slot} setpoints", logger.warning):
            named = self._pairs.pair_of(slot, peer)
            if named is not None:
                robot, part = named
                take(peer, robot, part, msg)

    def _take_arm(self, _peer, robot: str, arm: str, msg) -> None:
        joints = self._joints_of(robot, ARMS, arm)
        if joints is None:
            return
        setpoint = self._parsed(
            robot,
            arm,
            lambda: JointSetpoint.parse(msg.positions, msg.velocities, len(joints), "joints"),
        )
        if setpoint is not None:
            self._keep(self._arms, robot, arm, setpoint)

    def _take_gripper(self, _peer, robot: str, gripper: str, msg) -> None:
        if self._joints_of(robot, GRIPPERS, gripper) is None:
            return
        setpoint = self._parsed(
            robot, gripper, lambda: GripperSetpoint.parse(msg.opening, msg.max_effort)
        )
        if setpoint is not None:
            self._keep(self._grippers, robot, gripper, setpoint)

    def _take_base(self, peer, robot: str, base: str, msg) -> None:
        wheels = self._joints_of(robot, BASES, base)
        if wheels is None:
            return
        setpoint = self._parsed(
            robot,
            base,
            lambda: JointSetpoint.parse_wheels(msg.positions, msg.velocities, len(wheels)),
        )
        if setpoint is not None and self._bases.put(robot, peer, setpoint):
            logger.info(f"first setpoint for '{robot}' {base}")

    def _joints_of(self, robot: str, slot: str, part: str) -> tuple[str, ...] | None:
        """The joints the part `part` of `robot` on `slot` moves, or None for
        a robot the scene does not hold or a part its model lacks, whose
        setpoints drive nothing."""
        held = self._robots.of_name(robot)
        return None if held is None else held.entry.joints_of(slot, part)

    def _parsed(self, robot: str, part: str, parse: Callable[[], _Setpoint]) -> _Setpoint | None:
        """The setpoint `parse` gives for `robot`'s `part`, or None for one it
        refuses, which is dropped, the log saying why the first time since
        one of the part's setpoints last fit."""
        try:
            setpoint = parse()
        except ValueError as error:
            if (robot, part) not in self._refusing:
                self._refusing.add((robot, part))
                logger.warning(
                    f"dropping the setpoints for '{robot}' {part} until one fits: it {error}"
                )
            return None
        self._refusing.discard((robot, part))
        return setpoint

    def _keep(self, store: dict, robot: str, part: str, setpoint) -> None:
        with self._lock:
            first = (robot, part) not in store
            store[(robot, part)] = setpoint
        if first:
            logger.info(f"first setpoint for '{robot}' {part}")

    # --- read on the thread that steps the scene ---

    def arm(self, robot: str, arm: str) -> JointSetpoint | None:
        """The latest setpoint of `robot`'s `arm`, or None before its first:
        an arm holds its last setpoint."""
        with self._lock:
            return self._arms.get((robot, arm))

    def gripper(self, robot: str, gripper: str) -> GripperSetpoint | None:
        """The latest setpoint of `robot`'s `gripper`, or None before its
        first."""
        with self._lock:
            return self._grippers.get((robot, gripper))

    def base(self, robot: str, now_s: float) -> JointSetpoint | None:
        """The setpoint `robot`'s wheels follow at `now_s`, the engine time of
        this step, or None while they hold at rate 0: none arrived since they
        were last released, or more than LEADER_STREAMING_WINDOW_S passed with
        no newer one, which releases them (`take_base_release`)."""
        if self._bases.drop_stale(robot, now_s):
            logger.info(
                f"the base of '{robot}' had no setpoint for more than "
                f"{LEADER_STREAMING_WINDOW_S:g} s: its wheels hold where they stand"
            )
        return self._bases.latest(robot)

    def take_base_release(self, robot: str) -> bool:
        """Whether `robot`'s wheels were released since the engine last
        asked, once: their setpoint was dropped, and the engine holds them
        where they stand."""
        return self._bases.take_release(robot)

    # --- read on either ---

    def keep_paired_bases(self) -> None:
        """Drops each base's setpoint whose pair is gone, which releases its
        wheels: they hold where they stand until its pair's leader sends
        again."""
        for robot in self._bases.keep_paired(self._pairs):
            logger.info(f"the base pair of '{robot}' is gone: its wheels hold where they stand")

    def drop_base(self, robot: str) -> None:
        """Drops the setpoint `robot`'s wheels follow, as a move that sets the
        robot down does: they hold at rate 0 until its leader sends again."""
        self._bases.forget(robot)

    def forget(self, robot: str) -> None:
        """Drops what a robot that left the scene had commanded, so a robot
        that joins under the same name drives nothing until its own leaders
        send."""
        with self._lock:
            for store in (self._arms, self._grippers):
                for key in [key for key in store if key[0] == robot]:
                    del store[key]
        self._bases.forget(robot)

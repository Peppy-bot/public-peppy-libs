"""The robot's measured limbs: the latest whole-robot snapshot the
backbone publishes on limb_states, kept by a small loop the node starts.

Each message is parsed once into a `LimbSnapshot` by the contract's
slicing rule (`contracts-hub/robot/limb_state.json5`): arm i's joints
start at the sum of `joints_per_arm[0..i]`, its position at `3 * i` and its
orientation at `4 * i`; the gripper openings follow `gripper_names`. A
message whose arrays do not fit its names is dropped with a log line, and
the store keeps the snapshot before it.

A snapshot's stamp is the backbone's capture time, on the backbone's
clock, which need not be the brain's: openarm_sim runs the backbone on the
simulation's clock and the brain on the wall clock. So the store compares
stamps only with each other: `newer_than` waits for a snapshot captured
after another one.

The slot may be vacant when a node declares it optional: the loop then
returns at once, and the store gives its reason to every grab.
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass
from typing import Mapping, Optional

from .ports import Quat, Vec3
from .waiting import unless_cancelled

logger = logging.getLogger(__name__)

NO_SNAPSHOT = "no limb_states snapshot received from the robot yet"
VACANT = "no limb_state source: the launcher left the limb_state slot vacant"


@dataclass(frozen=True)
class ArmState:
    """One arm as a snapshot measured it: its joints in their fixed order
    and its grasp-point pose in the robot frame."""

    joint_positions: tuple[float, ...]
    position: Vec3
    orientation: Quat


@dataclass(frozen=True)
class LimbSnapshot:
    """One limb_states message, sliced per arm and per gripper."""

    timestamp: float
    arms: Mapping[str, ArmState]
    gripper_openings: Mapping[str, float]

    @classmethod
    def parse(cls, message) -> "LimbSnapshot":
        """The snapshot a limb_states message carries, or a ValueError
        naming the array that does not fit the names."""
        names = list(message.arm_names)
        joints_per_arm = [int(count) for count in message.joints_per_arm]
        _require(len(joints_per_arm) == len(names), f"joints_per_arm has {len(joints_per_arm)} entries for {len(names)} arms")
        _require(
            len(message.joint_positions) == sum(joints_per_arm),
            f"joint_positions has {len(message.joint_positions)} entries, joints_per_arm adds up to {sum(joints_per_arm)}",
        )
        _require(len(message.positions) == 3 * len(names), f"positions has {len(message.positions)} entries for {len(names)} arms")
        _require(len(message.orientations) == 4 * len(names), f"orientations has {len(message.orientations)} entries for {len(names)} arms")
        _require(
            len(message.gripper_openings) == len(message.gripper_names),
            f"gripper_openings has {len(message.gripper_openings)} entries for {len(message.gripper_names)} grippers",
        )
        _require(len(set(names)) == len(names), f"arm_names repeats a name: {names}")
        _require(len(set(message.gripper_names)) == len(message.gripper_names), f"gripper_names repeats a name: {list(message.gripper_names)}")
        arms: dict[str, ArmState] = {}
        start = 0
        for index, (name, count) in enumerate(zip(names, joints_per_arm)):
            position = message.positions[3 * index : 3 * index + 3]
            orientation = message.orientations[4 * index : 4 * index + 4]
            arms[name] = ArmState(
                joint_positions=tuple(float(value) for value in message.joint_positions[start : start + count]),
                position=(float(position[0]), float(position[1]), float(position[2])),
                orientation=(float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3])),
            )
            start += count
        openings = {name: float(opening) for name, opening in zip(message.gripper_names, message.gripper_openings)}
        return cls(timestamp=float(message.timestamp), arms=arms, gripper_openings=openings)

    def opening(self, gripper: str) -> Optional[float]:
        """The measured opening of `gripper`, 0 closed to 1 open, or none
        when the snapshot names no such gripper."""
        return self.gripper_openings.get(gripper)


class LimbStateStore:
    def __init__(self) -> None:
        self.latest: Optional[LimbSnapshot] = None
        self._vacant = False
        self._last_rejection = ""
        # Set and replaced at each snapshot, so a waiter wakes once per
        # snapshot and looks again.
        self._arrived = asyncio.Event()

    def why_unavailable(self) -> str:
        """Why a grab cannot read the limbs now, or the empty string."""
        if self._vacant:
            return VACANT
        if self.latest is None:
            return NO_SNAPSHOT
        return ""

    def add(self, message) -> None:
        """Keeps the snapshot `message` carries as the latest, or drops a
        message that does not parse, logging the reason once until another
        one comes."""
        try:
            snapshot = LimbSnapshot.parse(message)
        except ValueError as error:
            reason = str(error)
            if reason != self._last_rejection:
                logger.warning("a limb_states message was dropped: %s", reason)
                self._last_rejection = reason
            return
        self._last_rejection = ""
        self.latest = snapshot
        arrived, self._arrived = self._arrived, asyncio.Event()
        arrived.set()

    async def newer_than(self, timestamp: float, timeout_s: float) -> Optional[LimbSnapshot]:
        """The first snapshot captured after `timestamp`, a stamp of an
        earlier snapshot, or none when no such snapshot arrives within
        `timeout_s`. A snapshot already held answers at once."""
        if self._is_newer(timestamp):
            return self.latest
        try:
            return await asyncio.wait_for(self._wait_newer(timestamp), timeout_s)
        except TimeoutError:
            return None

    async def run(self, node_runner, token, limb_states) -> None:
        """Follows the limb_state slot until the node stops, through the
        node's generated limb_states module. Nothing to do when the slot is
        vacant."""
        if limb_states.bound_producer(node_runner) is None:
            self._vacant = True
            return
        await unless_cancelled(token, self._follow(node_runner, limb_states))

    def _is_newer(self, timestamp: float) -> bool:
        return self.latest is not None and self.latest.timestamp > timestamp

    async def _wait_newer(self, timestamp: float) -> LimbSnapshot:
        while not self._is_newer(timestamp):
            await self._arrived.wait()
        return self.latest

    async def _follow(self, node_runner, limb_states) -> None:
        subscription = await limb_states.subscribe(node_runner)
        async for _producer, message in subscription:
            self.add(message)


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)

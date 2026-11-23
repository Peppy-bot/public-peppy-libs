"""The manipulators: the `Manipulator` port every one implements; `none`,
the manipulator of a brain with no manipulation model; and `scripted`, the
grab from above through a fixed sequence of moves, which a node offers as
it is or with an aligner of its own."""

from __future__ import annotations

from typing import Optional, Protocol, Union

from ..ports import CancelToken, Gripper, Item, Outcome, Pose, Senses, TargetItem
from ..stages import CarryReports, GrabReports


class Manipulator(Protocol):
    """The swappable piece of manipulation. Every move it makes goes
    through the `Robot` it is started with, so a stop can cancel the move
    in flight whatever the backend is. Each sequence gets the reports of its
    goal (`stages.GrabReports`, `stages.CarryReports`) and publishes each
    stage through them as it starts; a sequence that ends before its first
    stage publishes nothing."""

    name: str

    def why_unavailable(self) -> str:
        """Why a manipulation goal is refused now, in a few words, or ""
        when the manipulator can move."""
        ...

    def starting(self) -> str:
        """Why a manipulation goal is refused now while the manipulator
        starts, in a few words, such as no limb state received yet, or "":
        once the manipulator can move, and when it does not become able to
        (no model, a vacant limb_state slot)."""
        ...

    async def start(self, senses: Senses) -> None:
        """Hands the manipulator what it senses and moves with, before it
        runs."""
        ...

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken, reports: GrabReports) -> Outcome:
        """Closes `gripper` on `item` and lifts it."""
        ...

    async def drop(
        self, gripper: Gripper, cancel: CancelToken, reports: CarryReports, above: Optional[TargetItem] = None
    ) -> Outcome:
        """Opens `gripper` and lets the held item go: where the arm stands,
        or, `above` an item, over it."""
        ...

    async def place(self, gripper: Gripper, to: Union[Pose, TargetItem], cancel: CancelToken, reports: CarryReports) -> Outcome:
        """Sets the held item down: at a pose in the robot frame, or in or
        on a target item. A successful place reports where the measured
        grasp point stood when the jaws opened, and the time of that
        measurement."""
        ...

    async def stop(self) -> None:
        """Halt whatever is in flight; the running call returns soon
        after, reporting it was stopped."""
        ...

"""The manipulators: the `Manipulator` port every one implements; `none`,
the manipulator of a brain with no manipulation model; and `scripted`, the
grab from above through a fixed sequence of moves, which a node offers as
it is or with an aligner of its own."""

from __future__ import annotations

from typing import Protocol

from ..ports import CancelToken, Gripper, Item, Outcome, Pose, Senses


class Manipulator(Protocol):
    """The swappable piece of manipulation. Every move it makes goes
    through the `Robot` it is started with, so a stop can cancel the move
    in flight whatever the backend is."""

    name: str

    def why_unavailable(self) -> str:
        """Why a manipulation goal is refused now, in a few words, or ""
        when the manipulator can move."""
        ...

    async def start(self, senses: Senses) -> None:
        """Hands the manipulator what it senses and moves with, before it
        runs."""
        ...

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken) -> Outcome: ...

    async def drop(self, gripper: Gripper, cancel: CancelToken) -> Outcome: ...

    async def place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome: ...

    async def stop(self) -> None:
        """Halt whatever is in flight; the running call returns soon
        after, reporting it was stopped."""
        ...

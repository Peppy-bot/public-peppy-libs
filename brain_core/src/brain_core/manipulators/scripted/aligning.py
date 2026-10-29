"""The port of the aligner the scripted manipulator calls between the move
above the item, or above the spot a held item goes to, and the descent.

The manipulator hands the aligner the item to grasp (none for a place),
the gripper, the pre point the gripper stands at, the point of the descent
under it, the orientation it stands in, how far the lift goes above the
point, and for a place the target point (`AlignRequest`). The aligner may
move the gripper sideways and turn it, at the same height, through the
manipulator (`ArmMoves`: whether the backbone has a plan for points at an
orientation, and the move itself, so every move carries the bars of every
arm move). It ends `Aligned` with the points the sequence goes on with,
`GaveUp`, `OutOfSteps`, or `Stopped` when a stop ended a move, and raises
`AlignmentFailed` for what a step could not get. A node brings the aligner
(vlm_router_brain's `ModelAligner`); the manipulator with none skips the
step.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol, Union

from ...ports import CancelToken, ExpectedFailure, FailureKind, Gripper, Item, Quat, Senses, Vec3
from ...robot import MoveResult


class AlignmentFailed(ExpectedFailure):
    """A step could not get what it needs: no picture, no pose, no
    limbs. The attempt gave no result, so its kind is `attempt_failed`."""

    def __init__(self, message: str) -> None:
        super().__init__(FailureKind.ATTEMPT_FAILED, message)


@dataclass(frozen=True)
class AlignRequest:
    """What a grab or a place hands the aligner: the item to grasp (none
    for a place), the gripper, the pre point the gripper stands at, the
    point of the descent under it (the grasp point, or where the jaws
    open), the orientation it stands in, how far the lift goes above the
    point, and `target`, the centre the limit is measured from: the item's
    seen point when none is given."""

    item: Optional[Item]
    gripper: Gripper
    pre: Vec3
    point: Vec3
    orientation: Quat
    lift_m: float
    target: Optional[Vec3] = None

    def __post_init__(self) -> None:
        if self.item is None and self.target is None:
            raise ValueError("an alignment needs an item to grasp or a target point")

    @property
    def is_place(self) -> bool:
        return self.item is None

    @property
    def label(self) -> str:
        return self.item.label if self.item is not None and self.item.label else "item"

    @property
    def centre(self) -> tuple[float, float]:
        at = self.target if self.target is not None else self.item.position
        return (at[0], at[1])

    @property
    def centre_words(self) -> str:
        """What the state text measures the grasp point from."""
        return "the target point" if self.is_place else f"the {self.label}'s seen centre"


@dataclass(frozen=True)
class Aligned:
    """The model said ready: the points the grab goes on with."""

    pre: Vec3
    point: Vec3
    lift: Vec3
    orientation: Quat
    steps: int = 0


@dataclass(frozen=True)
class GaveUp:
    steps: int = 0


@dataclass(frozen=True)
class OutOfSteps:
    steps: int


@dataclass(frozen=True)
class Stopped:
    """A stop ended a move of the alignment."""

    message: str
    steps: int = 0


Alignment = Union[Aligned, GaveUp, OutOfSteps, Stopped]


class ArmMoves(Protocol):
    """The moves the aligner makes, through the manipulator that owns it,
    so every move carries the bars of every arm move."""

    async def refusal_for(self, arm: str, points: tuple[Vec3, ...], orientation: Quat, cancel: CancelToken) -> str:
        """The backbone's words for the first of `points` it has no plan
        for, or "" when it has a plan for each."""
        ...

    async def move_to(self, arm: str, point: Vec3, orientation: Quat) -> MoveResult: ...


class Aligner(Protocol):
    """What corrects the gripper above the point of a descent: a node's own,
    started with the brain's `Senses` as the manipulator is."""

    def start(self, senses: Senses) -> None: ...

    def why_unavailable(self, arm: str) -> str:
        """Why `arm` cannot be aligned now, or the empty string."""
        ...

    async def align(self, request: AlignRequest, moves: ArmMoves, cancel: CancelToken) -> Alignment: ...

"""The seams of the brain: the plain data that crosses them, the roles a
model serves, the failure kinds a goal ends with, the interfaces a backend
implements, the cancel token every long call takes, and the deadline every
search carries.

The core owns every contract rule. A backend only answers one question:
"what do you see in this image" (a `Detector`), "do this with the arm"
(a `Manipulator`) or "where and how do I grasp this item" (the
`GraspProposer` a manipulator asks). None of them mints item ids, touches
the gripper table, or completes a goal, so swapping a backend cannot break
a contract rule.
"""

from __future__ import annotations

import asyncio
import time
from dataclasses import dataclass
from enum import StrEnum
from typing import Optional, Protocol, Sequence

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
# A region of a picture: x0, y0, x1, y1 in pixels, x to the right and y
# down, the corners of the box that holds an item.
Region = tuple[float, float, float, float]


class FailureKind(StrEnum):
    """Why a goal is refused or fails, as the brain says it: the kinds of
    peppy, and `not_found`, the kind of item_perception that identify_item
    lists. The generated module of each member checks that the member
    gives the kind it is given."""

    # The attempt gave no result.
    ATTEMPT_FAILED = "attempt_failed"
    # The bound of the provider passed.
    TIMEOUT = "timeout"
    # A precondition failed: a lane runs a goal, the node is shutting down,
    # a role's model is "none".
    PRECONDITION_FAILED = "precondition_failed"
    # A robot, limb, source or host is gone or not there.
    RESOURCE_UNAVAILABLE = "resource_unavailable"
    # The provider refused the goal, and no other kind says why: an unknown
    # name, a value the brain does not take.
    REJECTED = "rejected"
    # identify_item found no item that matches the description.
    NOT_FOUND = "not_found"


class GoalFailure(Exception):
    """Why a goal ends with no result: a failure kind, and a message for a
    person."""

    def __init__(self, kind: FailureKind, message: str) -> None:
        super().__init__(message)
        self.kind = kind
        self.message = message


class Refusal(GoalFailure):
    """A goal that a rule of the contracts tells the brain not to carry out:
    an unknown name, a lane that runs a goal, no source. Raised while a
    goal is decided, it refuses the goal; raised once the goal runs, it
    ends the goal the way its contract reports a failure."""


class ExpectedFailure(GoalFailure):
    """A goal that was tried and gave no result, the way the brain expects
    some goals to fail: an error a model's provider returned, a search past
    its bound, a search that found nothing. It ends the goal the way its
    contract reports a failure, and its log line carries no traceback. Any
    other exception is a fault: the goal fails with `attempt_failed`, and
    its log line carries the traceback."""


class Cancelled(Exception):
    """A sequence stopped on request. `signalled` is true when the goal's
    own cancel signal stopped it, a cancel of its caller or a sender that
    is gone, and false when the brain stopped it for another goal or for
    itself, an abort or a shutdown. A contract that reports in its result
    tells the two apart: item_manipulation completes the first as
    cancelled and the second with a failed result."""

    def __init__(self, reason: str, signalled: bool) -> None:
        super().__init__(reason)
        self.reason = reason
        self.signalled = signalled


class SearchTimeout(Exception):
    """A detector stopped at its deadline: the search has no answer, and
    the core fails it with `timeout`, naming the budget."""

    def __init__(self, budget_s: float) -> None:
        super().__init__(f"the search did not finish within {budget_s:g} s")
        self.budget_s = budget_s


class Role(StrEnum):
    """A part of the brain's work that a model does, as brain_model names
    it, and the lane its goals run in: perception finds and names items
    (scan_items, identify_item), manipulation moves them (grab_item,
    drop_item, place_item)."""

    PERCEPTION = "perception"
    MANIPULATION = "manipulation"


def parse_role(text: str, roles: Sequence[Role]) -> Role:
    """The role `text` names among `roles`, the roles a brain has, or the
    refusal, `rejected`, that the brain runs no model for it."""
    for role in roles:
        if text == role.value:
            return role
    raise Refusal(FailureKind.REJECTED, f"this brain runs no model for the role '{text}'")


@dataclass(frozen=True)
class Deadline:
    """When a search must have answered: an instant of the monotonic clock
    and the budget it was set from. A detector calls `check` between its
    stages, so a search ends at the next stage boundary after the deadline
    rather than running on."""

    at: float
    budget_s: float

    @classmethod
    def after(cls, budget_s: float) -> "Deadline":
        return cls(time.monotonic() + budget_s, budget_s)

    def remaining_s(self) -> float:
        return max(0.0, self.at - time.monotonic())

    def check(self) -> None:
        if time.monotonic() >= self.at:
            raise SearchTimeout(self.budget_s)


class CancelToken:
    """Set once, by whoever stops the sequence. Backends call `check()`
    between steps so a stop takes effect at the next step."""

    def __init__(self) -> None:
        self._event = asyncio.Event()
        self.reason = ""
        self.signalled = False

    @property
    def cancelled(self) -> bool:
        return self._event.is_set()

    def cancel(self, reason: str, *, signalled: bool) -> None:
        if not self._event.is_set():
            self.reason = reason
            self.signalled = signalled
            self._event.set()

    def check(self) -> None:
        if self.cancelled:
            raise Cancelled(self.reason, self.signalled)

    async def wait(self) -> None:
        await self._event.wait()


@dataclass(frozen=True)
class Pose:
    """A grasp-point pose in the world frame limb_motion uses: meters, and
    a unit quaternion [x, y, z, w] or none when the caller left the choice
    to the brain."""

    position: Vec3
    orientation: Optional[Quat] = None


@dataclass(frozen=True)
class Box:
    """One detection in an image: a label from the detector's vocabulary,
    its confidence from 0 to 1, and the box in pixels, x to the right and
    y down, [x0, x1) by [y0, y1)."""

    label: str
    confidence: float
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def centre(self) -> tuple[float, float]:
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)

    @property
    def xyxy(self) -> tuple[float, float, float, float]:
        return (self.x0, self.y0, self.x1, self.y1)


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    """Intersection over union of two boxes given as x0, y0, x1, y1."""
    x0, y0 = max(a[0], b[0]), max(a[1], b[1])
    x1, y1 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - overlap
    return float(overlap / union) if union > 0.0 else 0.0


@dataclass(frozen=True)
class Detection:
    """A box turned into the world: the label, where the item is, how sure
    the detector was, the box itself, where the item is in the picture the
    look used, and the item's width across that picture at the depth of its
    seen point (m). Orientation is set only by a backend that resolves it;
    the contracts make it optional for that reason."""

    label: str
    position: Vec3
    confidence: float = 0.0
    orientation: Optional[Quat] = None
    region: Optional[Region] = None
    width_m: Optional[float] = None


@dataclass(frozen=True)
class Coverage:
    """The labels one look names with authority. A known item the look did
    not see is dropped only when the look covers its label, and an item the
    look saw takes the look's label only when the look covers it. So a scan
    that cannot name an item leaves it alone, and a search by description
    covers nothing: it never renames an item and never drops one.
    `every_label` is a look that can name anything, an open-vocabulary scan."""

    labels: frozenset[str] = frozenset()
    every_label: bool = False

    def covers(self, label: str) -> bool:
        return self.every_label or label in self.labels


@dataclass
class Item:
    """A known item: the id the brain minted for it and what was last seen.
    `region` is where the look that last saw it found it in the picture;
    none for an item the last look did not see, or one addressed by
    pose. `width_m` is its width across the picture of the last look that
    saw it, at the depth of its seen point; none for an item no look saw.

    `position_is_grasp_point` says what `position` and `orientation` are.
    False for an item a look saw: the position is the point the camera saw
    on its surface, and the orientation the item's own, when the detector
    resolves one. True for an item a grab_item goal addressed by pose, or
    one place_item put down: the position is a grasp point, where the
    gripper closes, and the orientation the gripper's there, or none when
    the goal left the choice to the brain."""

    item_id: str
    label: str
    position: Vec3
    orientation: Optional[Quat]
    confidence: float
    seen_at_ns: int
    region: Optional[Region] = None
    position_is_grasp_point: bool = False
    width_m: Optional[float] = None


@dataclass
class Gripper:
    """One gripper from the fixed set the launcher named, the arm that
    carries it, and the item it holds according to the results so far."""

    name: str
    arm: str
    held_item_id: Optional[str] = None

    @property
    def holding(self) -> bool:
        return self.held_item_id is not None


@dataclass(frozen=True)
class Grasp:
    """Where and how a gripper closes on an item: the grasp point, robot
    frame, and the orientations of the gripper to try there, in order; a
    manipulator takes the first one the backbone has a plan for. Each is a
    unit quaternion [x, y, z, w] of limb_motion's grasp frame: +Z along the
    approach, out of the gripper, +Y the axis the jaws close on."""

    point: Vec3
    orientations: tuple[Quat, ...]

    def __post_init__(self) -> None:
        if not self.orientations:
            raise ValueError("a grasp needs at least one orientation to try")


@dataclass(frozen=True)
class Outcome:
    """What a manipulation backend reports when a sequence ends. The
    measured grasp-point position when the jaws opened, for place_item."""

    success: bool
    message: str = ""
    final_position: Optional[Vec3] = None


class Detector(Protocol):
    """The swappable piece of perception: a model that finds boxes in one
    RGB image. The core does everything around it: frames, depth, the
    camera model, duplicate boxes, ids. The core awaits `detect` on the
    event loop, so a detector whose model blocks runs its compute through
    `perceiver.in_daemon_thread`.

    One box per object: a detector that names one object twice, under one
    label or under two ("mug" and "cup"), returns one box for it. Only the
    model knows that two labels name one object, so the core merges only
    the boxes of one label (`perceiver.merge_duplicates`), and never across
    labels, which would delete real objects that overlap in the picture.
    Each detector keeps the confidence its boxes are kept at, in its own
    measure of it."""

    name: str

    def why_unavailable(self) -> str:
        """Why a search is refused now, in a few words, or "" when the
        detector can search: a model still loading, a load that failed, or
        no model at all."""
        ...

    async def detect(self, image, phrases: Sequence[str], deadline: Deadline) -> list[Box]:
        """`image` is an H x W x 3 uint8 RGB array, `phrases` the labels to
        look for: empty means the detector's own vocabulary, which is what a
        scan asks for. The detector checks `deadline` between its stages and
        raises `SearchTimeout` once it has passed."""
        ...

    def scan_coverage(self) -> Coverage:
        """The labels a scan, a search with no phrases, can name; nothing
        while the detector cannot search."""
        ...


class GraspProposer(Protocol):
    """Where and how a manipulator grasps an item: the piece of a grab a
    model can fill. A ValueError names why it has no grasp for the item."""

    def propose(self, item: Item) -> Grasp: ...


class Manipulator(Protocol):
    """The swappable piece of manipulation. Every move it makes goes
    through the `Robot` it is started with, so a stop can cancel the move
    in flight whatever the backend is."""

    name: str

    def why_unavailable(self) -> str:
        """Why a manipulation goal is refused now, in a few words, or ""
        when the manipulator can move."""
        ...

    async def start(self, robot, limb_states) -> None:
        """Hands the manipulator the brain's `Robot` and its
        `LimbStateStore`, the robot's measured limbs, before it runs."""
        ...

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken) -> Outcome: ...

    async def drop(self, gripper: Gripper, cancel: CancelToken) -> Outcome: ...

    async def place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome: ...

    async def stop(self) -> None:
        """Halt whatever is in flight; the running call returns soon
        after, reporting it was stopped."""
        ...


class ModelSource(Protocol):
    """Where a brain's models come from: the node's own, under the names it
    gives them. The core asks it which roles the brain has, which names
    each role accepts, the entry a name stands for, and the backend an
    entry runs: a `Detector` for perception, a `Manipulator` for
    manipulation. An entry is the source's own type; the core passes it
    from `offered` to `start` or `prepare` and reads nothing in it."""

    def roles(self) -> Sequence[Role]:
        """The roles the brain has, in the order get_model lists them."""
        ...

    def names(self, role: Role) -> Sequence[str]:
        """The models `role` accepts, what list_models gives for it."""
        ...

    def offered(self, role: Role, name: str) -> object:
        """The entry of the model `name` for `role`, or a Refusal with
        `rejected`, in the brain's own words, when the role does not offer
        it."""
        ...

    def start(self, entry: object) -> object:
        """The backend of `entry` at the node's start, with no network
        call; a ValueError names what stops the node."""
        ...

    async def prepare(self, entry: object) -> object:
        """The backend of `entry` for a switch, checked before the switch
        hands it over: a hosted model is reached once. A Refusal or an
        ExpectedFailure says why it cannot serve: `resource_unavailable`
        when the brain cannot reach the model, `attempt_failed` when the
        model cannot serve the role."""
        ...

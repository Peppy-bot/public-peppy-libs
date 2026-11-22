"""The stages a goal reports in its feedback, and the one place that writes
their sentences.

grab_item, drop_item and place_item of item_manipulation, and scan_items
and identify_item of item_perception, publish one feedback message at the
start of each stage of a goal: the stage, one sentence that says what the
robot does now, and the number and the maximum of a stage that repeats
(`Report`). The sentence is for the person who looks at the robot and
follows the run: it names what that person sees (the robot, its arm, its
hand, each item by its label, "the item" for an item with no label), with
no name of a link, a joint, a contract or a model, and no term of the
motion planner. A stage that repeats ends its sentence with " (N of M)",
so the sentence alone tells one repeat from the next. No sentence ends
with a full stop.

The handler of a goal makes the reporter of its goal context
(`reporter_of`) and hands it down, and the first report comes once the
checks that refuse the goal passed: a search publishes its one stage, look
(`scan_look`, `identify_look`), before it looks; a manipulation goal hands
the manipulator the reports of its sequence, bound to the labels of its
items (`GrabReports`, `CarryReports`), and the manipulator hands them to its
aligner, which sees them as `AlignReports`. So the manipulator and the
aligner name a stage and its numbers, and every word of every sentence is
written here.

A report that the goal context cannot publish is logged, and the goal goes
on: what the robot does never depends on its feedback.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import StrEnum
from typing import Awaitable, Callable, Optional, Protocol

from .ports import TargetItem
from .words import normalised

logger = logging.getLogger(__name__)

# The words of an item with no label.
NO_LABEL = "item"
# The articles a label or a description can start with, which a sentence
# replaces with its own.
ARTICLES = frozenset({"a", "an", "the"})


class Stage(StrEnum):
    """A stage of a goal, as the feedback of its action names it."""

    # A grab: the jaws open for the item and the brain looks for a way to
    # it. A place or a drop above a target item: the brain looks for a way
    # above the place where the held item goes.
    PLAN = "plan"
    # The move of the gripper to the point above the item, or the carry of
    # the held item above where it goes.
    MOVE_ABOVE = "move_above"
    # One question to the model that aligns the gripper.
    ALIGN_LOOK = "align_look"
    # One correction of the gripper that an answer of that model gives.
    ALIGN_MOVE = "align_move"
    # A grab: the descent, the close of the jaws and the lift.
    GRASP = "grasp"
    # A place: the descent until the item rests, the open and the climb back.
    SET_DOWN = "set_down"
    # A drop: the opening of the jaws.
    RELEASE = "release"
    # A search: the one look at the camera.
    LOOK = "look"


@dataclass(frozen=True)
class Report:
    """One feedback message: the stage that starts now, its sentence, and,
    for a stage that repeats, the number of this repeat from 1 and how many
    times the stage can repeat in the goal. A stage that does not repeat has
    0 and 0. A ValueError names numbers that are neither."""

    stage: Stage
    activity: str
    number: int = 0
    maximum: int = 0

    def __post_init__(self) -> None:
        repeats = 1 <= self.number <= self.maximum
        if not repeats and (self.number, self.maximum) != (0, 0):
            raise ValueError(f"a report is number 0 of 0, or number 1 to its maximum: got {self.number} of {self.maximum}")

    @classmethod
    def repeat(cls, stage: Stage, sentence: str, number: int, maximum: int) -> "Report":
        """The report of repeat `number` of `maximum` of a stage that
        repeats: `sentence` ends with " (N of M)"."""
        return cls(stage, f"{sentence} ({number} of {maximum})", number, maximum)


# Publishes one report of a goal.
Reporter = Callable[[Report], Awaitable[None]]


def reporter_of(ctx) -> Reporter:
    """The reporter of a goal: each report on the feedback topic of the
    goal's action, through the `publish_feedback` that the generated goal
    context of an action with a feedback topic has. A report that cannot be
    published is logged, and the goal goes on."""

    async def publish(report: Report) -> None:
        try:
            await ctx.publish_feedback(stage=report.stage.value, activity=report.activity, number=report.number, maximum=report.maximum)
        except Exception as error:
            logger.warning("the feedback '%s' was not published: %r", report.activity, error)

    return publish


def the_item(label: str) -> str:
    """An item as a sentence names it: "the" and its label, in lower case,
    an underscore as a space, the first article of the label replaced (a
    model can label an item "the cup"), or "the item" for an item with no
    label."""
    first, _, rest = normalised(label).partition(" ")
    words = rest if first in ARTICLES else normalised(label)
    return f"the {words or NO_LABEL}"


def scan_look() -> Report:
    """The look of scan_items."""
    return Report(Stage.LOOK, "The robot looks for each item that it can see")


def identify_look(description: str) -> Report:
    """The look of identify_item for `description`, the caller's words, its
    first article replaced as `the_item` replaces it: "a red cup" gives "the
    red cup". An empty description asks for the item the brain judges most
    prominent."""
    if not normalised(description):
        return Report(Stage.LOOK, "The robot looks for an item that it can take")
    return Report(Stage.LOOK, f"The robot looks for {the_item(description)}")


class AlignReports(Protocol):
    """What the aligner of a grab, a place or a drop publishes: each
    question to its model and each correction an answer gives, with the
    number of the step of the alignment and how many steps it can take."""

    async def align_look(self, number: int, maximum: int) -> None: ...

    async def align_move(self, number: int, maximum: int) -> None: ...


@dataclass(frozen=True)
class GrabReports:
    """The reports of one grab_item goal, through `publish`, about the item
    with the label `item`."""

    publish: Reporter
    item: str

    @property
    def _item(self) -> str:
        return the_item(self.item)

    async def plan(self) -> None:
        await self.publish(Report(Stage.PLAN, f"The robot opens its hand for {self._item} and finds a way to reach it"))

    async def move_above(self) -> None:
        await self.publish(Report(Stage.MOVE_ABOVE, f"The robot moves its hand above {self._item}"))

    async def align_look(self, number: int, maximum: int) -> None:
        sentence = f"The robot looks at {self._item} and decides how to correct its hand"
        await self.publish(Report.repeat(Stage.ALIGN_LOOK, sentence, number, maximum))

    async def align_move(self, number: int, maximum: int) -> None:
        sentence = f"The robot corrects the position of its hand above {self._item}"
        await self.publish(Report.repeat(Stage.ALIGN_MOVE, sentence, number, maximum))

    async def grasp(self) -> None:
        await self.publish(Report(Stage.GRASP, f"The robot lowers its hand, closes it on {self._item} and lifts it"))


@dataclass(frozen=True)
class CarryReports:
    """The reports of one place_item or drop_item goal, through `publish`,
    about the held item with the label `held` and the `target` item, which
    the sentences name by its label: none for a place at a pose and for a
    drop with no target item."""

    publish: Reporter
    held: str
    target: Optional[TargetItem] = None

    @property
    def _held(self) -> str:
        return the_item(self.held)

    @property
    def _target(self) -> str:
        return the_item(self.target.label)

    @property
    def _above(self) -> str:
        """Where the carry takes the held item: above the target item, or
        above the spot of a place at a pose."""
        if self.target is None:
            return "above the spot where it puts it down"
        return f"above {self._target}"

    async def plan(self) -> None:
        await self.publish(Report(Stage.PLAN, f"The robot finds a way to carry {self._held} {self._above}"))

    async def move_above(self) -> None:
        await self.publish(Report(Stage.MOVE_ABOVE, f"The robot carries {self._held} {self._above}"))

    async def align_look(self, number: int, maximum: int) -> None:
        looked_at = "down" if self.target is None else f"at {self._target}"
        sentence = f"The robot looks {looked_at} and decides where to put {self._held}"
        await self.publish(Report.repeat(Stage.ALIGN_LOOK, sentence, number, maximum))

    async def align_move(self, number: int, maximum: int) -> None:
        sentence = f"The robot corrects the position of {self._held} {self._above}"
        await self.publish(Report.repeat(Stage.ALIGN_MOVE, sentence, number, maximum))

    async def set_down(self) -> None:
        if self.target is None:
            down = f"puts {self._held} down"
        else:
            down = f"lowers {self._held} into {self._target}"
        await self.publish(Report(Stage.SET_DOWN, f"The robot {down}, opens its hand and moves its arm back up"))

    async def release(self, *, above_target: bool) -> None:
        """The release of a drop: above the target item when `above_target`
        and the goal names one, else where the arm stands."""
        if above_target and self.target is not None:
            sentence = f"The robot opens its hand and lets {self._held} fall into {self._target}"
        else:
            sentence = f"The robot opens its hand and lets go of {self._held}"
        await self.publish(Report(Stage.RELEASE, sentence))

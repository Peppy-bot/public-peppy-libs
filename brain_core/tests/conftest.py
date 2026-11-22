"""Shared test pieces: a brain node's parameters, fake backends, a fake
goal context, the camera's messages and answers, and the records of one
logger, so the core and the handlers run in plain tests with no router, no
robot and no model."""

from __future__ import annotations

import asyncio
import logging
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Iterator, Optional, Sequence, Union

import numpy as np
import pytest
from hypothesis import settings

from control_core_py.geometry import Placement, PoseTarget, Stamped

from brain_core.brain import Brain
from brain_core.manipulators.none import NoneManipulator
from brain_core.perception.camera import Intrinsics
from brain_core.perception.geometry import NOT_DESCRIBED, LookupFailure
from brain_core.perception.perceiver import SourceReason
from brain_core.perception.none import NoneDetector
from brain_core.ports import Box, CancelToken, Coverage, Deadline, FailureKind, Gripper, Item, Outcome, Pose, Refusal, Role, TargetItem
from brain_core.stages import CarryReports, GrabReports, Report

from fake_peppygen import ROBOT_FRAME

# Every property test runs the same examples on every host: no deadline,
# which would judge the host's speed, a seed fixed per test, and no example
# database carried from one run to the next.
settings.register_profile("brain_core", deadline=None, derandomize=True, database=None)
settings.load_profile("brain_core")

# How long a lookup of the camera's pose waits for the transforms of a
# picture's time: the parameter a launcher sets, set here for the tests.
LOOKUP_TIMEOUT_S = 0.5

PARAMS = {
    "limbs": "left_arm:left_gripper,right_arm:right_gripper",
    "perception_model": "none",
    "manipulation_model": "none",
    "camera_name": "chest",
    "lookup_timeout_s": LOOKUP_TIMEOUT_S,
}

# The colour intrinsics camera_geometry gives for a 16 x 12 camera with a
# 90 degree lens and none of its distortion: one focal length is half the
# image height, and the principal point is the image centre.
CAMERA_16X12 = Intrinsics(16, 12, 6.0, 6.0, 8.0, 6.0)

# The frame the camera's pictures name.
CAMERA_FRAME = "chest_color_optical_frame"

# The camera's pose at the robot frame's origin, looking along its +Z.
ORIGIN = Placement((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

# A deadline no search reaches, and one every search has passed.
NEVER = Deadline(at=float("inf"), budget_s=float("inf"))
PASSED = Deadline(at=0.0, budget_s=1.0)


@pytest.fixture
def params() -> SimpleNamespace:
    return SimpleNamespace(**PARAMS)


@contextmanager
def records_of(caplog, name: str) -> Iterator[pytest.LogCaptureFixture]:
    """`caplog` holding the records of the logger `name` from info up,
    each once, whatever `configure_logging` set on the package logger: it
    stops the package's records at its own handler, before the one pytest
    reads at the root."""
    logger = logging.getLogger(name)
    propagated, level = logger.propagate, logger.level
    logger.propagate = False
    logger.setLevel(logging.INFO)
    logger.addHandler(caplog.handler)
    try:
        yield caplog
    finally:
        logger.removeHandler(caplog.handler)
        logger.setLevel(level)
        logger.propagate = propagated


class FakeToken:
    """Stands in for the node's cancellation token."""

    def __init__(self) -> None:
        self._event = asyncio.Event()

    def is_cancelled(self) -> bool:
        return self._event.is_set()

    async def cancelled(self) -> None:
        await self._event.wait()

    def cancel(self) -> None:
        self._event.set()


class FakeDetector:
    """Returns the boxes the test hands it, and records the phrases and the
    deadlines of each search. Its scans name every label unless the test
    hands it a coverage; it can search unless the test hands it a reason it
    cannot, which is a reason it still starts when the test says so."""

    name = "fake"

    def __init__(
        self,
        boxes: Optional[list[Box]] = None,
        coverage: Coverage = Coverage(every_label=True),
        unavailable: str = "",
        *,
        starting: bool = False,
    ) -> None:
        self.boxes = boxes or []
        self.coverage = coverage
        self.unavailable = unavailable
        self.is_starting = starting
        self.phrases: list[list[str]] = []
        self.deadlines: list[Deadline] = []

    def why_unavailable(self) -> str:
        return self.unavailable

    def starting(self) -> str:
        return self.unavailable if self.is_starting else ""

    async def detect(self, image: np.ndarray, phrases: Sequence[str], deadline: Deadline) -> list[Box]:
        self.phrases.append(list(phrases))
        self.deadlines.append(deadline)
        return list(self.boxes)

    def scan_coverage(self) -> Coverage:
        return self.coverage


class Reports:
    """A reporter that keeps each report it is given, in order, and, given
    `calls`, the list of a fake robot's calls, also puts each one there as
    ("report", stage, number, maximum), so a test reads the reports in order
    with the moves."""

    def __init__(self, calls: Optional[list] = None) -> None:
        self.reports: list[Report] = []
        self.calls = calls

    async def __call__(self, report: Report) -> None:
        self.reports.append(report)
        if self.calls is not None:
            self.calls.append(("report", report.stage.value, report.number, report.maximum))

    @property
    def stages(self) -> list[tuple[str, int, int]]:
        return [(report.stage.value, report.number, report.maximum) for report in self.reports]


async def not_followed(report: Report) -> None:
    """The reporter of a test that does not read the feedback."""


# The reports of a grab of the cup and of a place or a drop of it, for the
# tests that do not read the feedback.
QUIET_GRAB = GrabReports(not_followed, "cup")
QUIET_CARRY = CarryReports(not_followed, "cup")


# When the fake manipulator's backbone measured the grasp point of a place.
PLACE_MEASURED_AT = 2.5


class FakeManipulator:
    """Succeeds at once unless told to fail, to wait for a stop or to wait
    for `release` to be set, and publishes no report: it keeps the reports
    of the last sequence in `reports`. `begun` resolves when a sequence has
    reached it, so a test knows the sequence is running without waiting on
    the clock."""

    name = "fake"

    def __init__(
        self, *, fail: str = "", wait_for_stop: bool = False, release: Optional[asyncio.Event] = None, closed_at=None,
    ) -> None:
        self.fail = fail
        # Where a grab that holds reports the jaws closed.
        self.closed_at = closed_at
        self.wait_for_stop = wait_for_stop
        self.release = release
        self.calls: list[tuple] = []
        self.reports: Optional[Union[GrabReports, CarryReports]] = None
        self.stopped = 0
        self.senses = None
        self.robot = None
        self.limb_states = None
        self._began = asyncio.Event()

    def why_unavailable(self) -> str:
        return ""

    def starting(self) -> str:
        return ""

    async def start(self, senses) -> None:
        self.senses = senses
        self.robot = senses.robot
        self.limb_states = senses.limb_states

    async def begun(self) -> None:
        await self._began.wait()
        self._began.clear()

    async def _run(self, cancel: CancelToken, final_position=None, closed_at=None) -> Outcome:
        self._began.set()
        if self.wait_for_stop:
            await cancel.wait()
            cancel.check()
        if self.release is not None:
            await self.release.wait()
        if self.fail:
            return Outcome(False, self.fail)
        return Outcome(True, "", final_position, PLACE_MEASURED_AT if final_position is not None else 0.0, closed_at)

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken, reports: GrabReports) -> Outcome:
        self.calls.append(("grab", item.item_id, gripper.name, max_effort))
        self.reports = reports
        return await self._run(cancel, closed_at=self.closed_at)

    async def drop(
        self, gripper: Gripper, cancel: CancelToken, reports: CarryReports, above: Optional[TargetItem] = None
    ) -> Outcome:
        self.calls.append(("drop", gripper.name) if above is None else ("drop", gripper.name, above.item_id))
        self.reports = reports
        return await self._run(cancel)

    async def place(self, gripper: Gripper, to: Union[Pose, TargetItem], cancel: CancelToken, reports: CarryReports) -> Outcome:
        """A place at a pose reports the jaws opened 1 cm above it; a place
        into a target item, 1 cm above the item's top at its centre; each
        measured at PLACE_MEASURED_AT."""
        self.reports = reports
        if isinstance(to, TargetItem):
            self.calls.append(("place", gripper.name, to.item_id))
            return await self._run(cancel, final_position=(to.centre[0], to.centre[1], to.top_z + 0.01))
        self.calls.append(("place", gripper.name, to.position))
        return await self._run(cancel, final_position=(to.position[0], to.position[1], to.position[2] + 0.01))

    async def stop(self) -> None:
        self.stopped += 1


# The reason peppy's goal context gives for each cause of a cancel signal.
PEPPY_CANCEL_REASONS = {"caller": "cancelled by its caller", "sender_gone": "its sender is gone"}


class FakeCtx:
    """Stands in for the generated GoalContext of `action`, a fake
    generated module: holds the goal, records how it ended, once, after
    checking that a result's fields are the ones the action's result
    carries and that a failure's kind is one the action declares, records
    each feedback message in `feedback`, after checking that its fields are
    the ones the action's feedback carries, that its stage is one the
    action's contract lists and that the goal has not ended, and lets a
    test send the goal its cancel signal, with the cause a caller's cancel
    or a sender that is gone gives and the reason peppy gives for that
    cause."""

    def __init__(self, action, **data) -> None:
        self.action = action
        self._request = SimpleNamespace(data=SimpleNamespace(**data))
        self._cancel = asyncio.Event()
        self._cause: Optional[str] = None
        self.completed: Optional[dict] = None
        self.cancelled: Optional[dict] = None
        self.cancelled_reason: Optional[str] = None
        self.failed = None
        self.feedback: list[dict] = []

    def request(self):
        return self._request

    def goal_id(self) -> str:
        return "goal"

    async def cancel_signal(self) -> None:
        await self._cancel.wait()

    def is_cancelled(self) -> bool:
        return self._cancel.is_set()

    def cancel_cause(self) -> Optional[str]:
        return self._cause

    def cancel_reason(self) -> Optional[str]:
        return PEPPY_CANCEL_REASONS.get(self._cause)

    def cancel(self, cause: str = "caller") -> None:
        self._cause = cause
        self._cancel.set()

    @property
    def ended(self) -> bool:
        return self.completed is not None or self.cancelled is not None or self.failed is not None

    async def publish_feedback(self, **fields) -> None:
        action = self.action
        assert action.FEEDBACK_FIELDS, f"{action.ACTION_NAME} has no feedback topic"
        assert not self.ended, f"{action.ACTION_NAME} published feedback after its goal ended"
        assert set(fields) == action.FEEDBACK_FIELDS, (
            f"{action.ACTION_NAME} published {sorted(fields)}, its feedback carries {sorted(action.FEEDBACK_FIELDS)}"
        )
        assert fields["stage"] in action.FEEDBACK_STAGES, f"{action.ACTION_NAME} lists no stage {fields['stage']!r}"
        assert isinstance(fields["activity"], str) and fields["activity"]
        assert all(isinstance(fields[name], int) and fields[name] >= 0 for name in ("number", "maximum"))
        self.feedback.append(fields)

    @property
    def stages(self) -> list[tuple[str, int, int]]:
        """Each feedback message's stage, number and maximum, in order."""
        return [(fields["stage"], fields["number"], fields["maximum"]) for fields in self.feedback]

    @property
    def activities(self) -> list[str]:
        """Each feedback message's sentence, in order."""
        return [fields["activity"] for fields in self.feedback]

    async def complete(self, **fields) -> None:
        self._check_end(fields)
        self.completed = fields

    async def complete_cancelled(self, reason: str, **fields) -> None:
        self._check_end(fields)
        self.cancelled = fields
        self.cancelled_reason = reason

    async def fail(self, kind, message: str) -> None:
        assert not self.ended, "ended twice"
        self.failed = self.action.Failure(kind, message)

    def _check_end(self, fields: dict) -> None:
        assert not self.ended, "ended twice"
        expected = self.action.RESULT_FIELDS
        assert set(fields) == expected, f"{self.action.ACTION_NAME} ended with {sorted(fields)}, its result carries {sorted(expected)}"


def failed_with(ctx: FakeCtx, kind: FailureKind, message: str) -> bool:
    """Whether the goal of `ctx` failed with `kind` and `message`, and no
    result."""
    return ctx.completed is None and ctx.cancelled is None and ctx.failed == ctx.action.Failure(kind.value, message)


def sensors_described(names=("chest",), kinds=None, robot_frame=ROBOT_FRAME):
    """The robot's describe_sensors response: each sensor of `names` an
    rgbd camera fixed to the base unless `kinds` names its kind, its frame
    `<name>_color_optical_frame`."""
    return SimpleNamespace(
        robot_frame=robot_frame,
        sensor_names=list(names),
        sensor_kinds=list(kinds) if kinds is not None else ["rgbd_camera"] * len(names),
        sensor_frames=[f"{name}_color_optical_frame" for name in names],
        carried_by=[""] * len(names),
    )


def lookup_answer(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0), timestamp=0.0, frame=ROBOT_FRAME, child_frame=CAMERA_FRAME):
    """The robot's lookup_transform response: the transform record of the
    source frame `child_frame` at `position` and `orientation` in the target
    frame `frame`."""
    return SimpleNamespace(
        frame=frame, child_frame=child_frame, timestamp=timestamp, position=list(position), orientation=list(orientation)
    )


def target(position, orientation=None, frame=ROBOT_FRAME, timestamp=None) -> SimpleNamespace:
    """The target record of a goal: `position` in `frame`, the robot frame
    when none, at `timestamp`, the latest time when none, with
    `orientation`, or none."""
    return SimpleNamespace(
        frame=frame, timestamp=timestamp, position=list(position), orientation=None if orientation is None else list(orientation)
    )


# The target a grab_item goal that names an item_id, or a place_item goal
# that names a target_item_id, sends, which is not read: a zero position with
# no frame, no timestamp and no orientation.
UNREAD_TARGET = target((0.0, 0.0, 0.0), frame=None)


class FixedPlacer:
    """A Placer whose robot frame is ROBOT_FRAME and whose tree stands every
    frame at `pose` at every time: the frame and the time of each picture
    header it places are kept in `asked`, and each target it places in
    `targets`. With `refusal`, a LookupFailure, every target is refused with
    it. With no pose, the robot has not described its sensors yet, and every
    ask is refused with that reason."""

    def __init__(self, pose: Optional[Placement] = ORIGIN, *, refusal: Optional[LookupFailure] = None) -> None:
        self.pose = pose
        self.refusal = refusal
        self.asked: list[tuple[str, float]] = []
        self.targets: list[Stamped[PoseTarget]] = []
        self.reason = SourceReason("" if pose is not None else NOT_DESCRIBED, starting=pose is None)

    async def learn(self, token) -> None:
        """Nothing to learn: the robot frame is fixed."""

    @property
    def robot_frame(self) -> str:
        if self.pose is None:
            raise Refusal(FailureKind.RESOURCE_UNAVAILABLE, self.reason.reason)
        return ROBOT_FRAME

    async def place_picture(self, header) -> Placement:
        self.asked.append((header.frame, header.timestamp))
        if self.pose is None:
            raise Refusal(FailureKind.RESOURCE_UNAVAILABLE, self.reason.reason)
        return self.pose

    async def place_target(self, stamped: Stamped[PoseTarget]) -> Pose:
        self.targets.append(stamped)
        if self.refusal is not None:
            raise self.refusal
        if self.pose is None:
            raise Refusal(FailureKind.RESOURCE_UNAVAILABLE, self.reason.reason)
        placed = self.pose.place(stamped.value)
        return Pose(placed.position.position, None if placed.orientation is None else placed.orientation.quaternion)


def rgb_frame(width: int = 16, height: int = 12, *, capture_index: int = 1, align_mode: str = "depth_to_color", frame: str = CAMERA_FRAME):
    """A video_stream message of the given size, plain grey."""
    return SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame=frame, capture_index=capture_index, align_mode=align_mode),
        encoding="rgb8",
        width=width,
        height=height,
        data=bytes([128]) * (width * height * 3),
    )


def depth_frame(
    depth_m: float, width: int = 16, height: int = 12, unit: float = 0.001, *, capture_index: int = 1,
    align_mode: str = "depth_to_color", frame: str = CAMERA_FRAME,
):
    """A z16 depth_stream message of the given size, `depth_m` everywhere."""
    value = int(round(depth_m / unit))
    return SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame=frame, capture_index=capture_index, align_mode=align_mode),
        encoding="z16",
        width=width,
        height=height,
        data=np.full((height, width), value, dtype="<u2").tobytes(),
    )


def with_frames(brain: Brain, depth_m: float = 1.0) -> None:
    """The camera's frames, `depth_m` deep everywhere, and its intrinsics,
    as the slots give them to a running brain, and a placer that stands the
    camera at the origin, looking along +Z: a 90 degree lens there."""
    brain.perceiver.frames.add_color(rgb_frame(16, 12))
    brain.perceiver.frames.add_depth(depth_frame(depth_m, 16, 12))
    brain.perceiver.frames.depth_unit = 0.001
    brain.perceiver.set_intrinsics(CAMERA_16X12)
    brain.perceiver.placer = FixedPlacer()


class Ticks:
    """A clock the test advances by hand, in nanoseconds; with `step_ns`,
    it also moves on by that much at every reading."""

    def __init__(self, step_ns: int = 0) -> None:
        self.now_ns = 1_000_000_000
        self.step_ns = step_ns

    def __call__(self) -> int:
        reading = self.now_ns
        self.now_ns += self.step_ns
        return reading


class FakeSource:
    """A brain's models as the test sets them: per role, the names it
    offers and the backend of each. An entry is (role, name). `prepare`
    waits on `gate` when the test sets one, and raises `failure` when the
    test sets one; `preparing` resolves once a prepare has begun."""

    def __init__(self, offers: dict) -> None:
        self.offers = offers
        self.asked: list[tuple] = []
        self.started: list[tuple] = []
        self.prepared: list[tuple] = []
        self.gate: Optional[asyncio.Event] = None
        self.failure: Optional[Exception] = None
        self.preparing = asyncio.Event()

    @classmethod
    def of(cls, detector, manipulator, **more_perception) -> "FakeSource":
        """A source that offers `detector` under its name, and the other
        detectors `more_perception` names, for perception, and
        `manipulator` under its name for manipulation."""
        return cls({
            Role.PERCEPTION: {detector.name: detector, **more_perception},
            Role.MANIPULATION: {manipulator.name: manipulator},
        })

    def roles(self):
        return tuple(self.offers)

    def names(self, role):
        return list(self.offers[role])

    def offered(self, role, name):
        self.asked.append((role, name))
        if name not in self.offers[role]:
            raise Refusal(FailureKind.REJECTED, f"'{name}' is not a {role} model here; see list_models")
        return (role, name)

    def start(self, entry):
        self.started.append(entry)
        role, name = entry
        return self.offers[role][name]

    async def prepare(self, entry):
        self.prepared.append(entry)
        self.preparing.set()
        if self.gate is not None:
            await self.gate.wait()
        if self.failure is not None:
            raise self.failure
        role, name = entry
        return self.offers[role][name]


async def make_brain(
    *, detector=None, manipulator=None, source=None, ticks=None, bindings=None, perception_model=None, manipulation_model=None,
    placer: Optional[FixedPlacer] = None,
) -> Brain:
    """A started brain on fake bindings whose roles run `detector` and
    `manipulator`, the none backends when not given, from `source` or from
    a FakeSource that offers those two. The model parameters name the two,
    unless the test names others. Its robot and its goals' targets are
    placed by `placer`, a FixedPlacer at the origin when not given; its
    perceiver keeps the brain's own Placer."""
    from fake_peppygen import fake_bindings

    detector = detector or NoneDetector()
    manipulator = manipulator or NoneManipulator()
    source = source or FakeSource.of(detector, manipulator)
    params = SimpleNamespace(**{
        **PARAMS,
        "perception_model": perception_model or detector.name,
        "manipulation_model": manipulation_model or manipulator.name,
    })
    brain = Brain(params, None, bindings or fake_bindings(), source, now=ticks or Ticks(), run_token="t0")
    brain.placer = placer or FixedPlacer()
    await brain.start()
    return brain


# The handlers whose decision makes the goal they run from the request:
# each other handler runs its request data, and makes the checks of its
# decision again when its goal starts.
ADMITTING = ("grab_item", "place_item")


async def run_goal(handler, brain: Brain, ctx: FakeCtx) -> None:
    """Runs the goal of `ctx` through `handler` as the action's loop runs an
    accepted goal: grab_item and place_item take the goal their `admit`
    makes of the request data, every other handler that data."""
    data = ctx.request().data
    goal = await handler.admit(brain, data) if handler.__name__.rsplit(".", 1)[-1] in ADMITTING else data
    await handler.run(brain, ctx, goal)

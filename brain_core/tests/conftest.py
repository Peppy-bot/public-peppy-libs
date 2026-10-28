"""Shared test pieces: a brain node's parameters, fake backends, a fake
goal context, the camera's messages and answers, and the records of one
logger, so the core and the handlers run in plain tests with no router, no
robot and no model."""

from __future__ import annotations

import asyncio
import logging
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Iterator, Optional, Sequence

import numpy as np
import pytest
from hypothesis import settings

from brain_core.brain import Brain
from brain_core.manipulators.none import NoneManipulator
from brain_core.perception.camera import Intrinsics
from brain_core.perception.none import NoneDetector
from brain_core.ports import Box, CancelToken, Coverage, Deadline, FailureKind, Gripper, Item, Outcome, Pose, Refusal, Role

# Every property test runs the same examples on every host: no deadline,
# which would judge the host's speed, a seed fixed per test, and no example
# database carried from one run to the next.
settings.register_profile("brain_core", deadline=None, derandomize=True, database=None)
settings.load_profile("brain_core")

PARAMS = {
    "limbs": "left_arm:left_gripper,right_arm:right_gripper",
    "perception_model": "none",
    "manipulation_model": "none",
    "camera_name": "chest",
}

# The colour intrinsics camera_geometry gives for a 16 x 12 camera with a
# 90 degree lens and none of its distortion: one focal length is half the
# image height, and the principal point is the image centre.
CAMERA_16X12 = Intrinsics(16, 12, 6.0, 6.0, 8.0, 6.0)

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
    cannot."""

    name = "fake"

    def __init__(
        self, boxes: Optional[list[Box]] = None, coverage: Coverage = Coverage(every_label=True), unavailable: str = ""
    ) -> None:
        self.boxes = boxes or []
        self.coverage = coverage
        self.unavailable = unavailable
        self.phrases: list[list[str]] = []
        self.deadlines: list[Deadline] = []

    def why_unavailable(self) -> str:
        return self.unavailable

    async def detect(self, image: np.ndarray, phrases: Sequence[str], deadline: Deadline) -> list[Box]:
        self.phrases.append(list(phrases))
        self.deadlines.append(deadline)
        return list(self.boxes)

    def scan_coverage(self) -> Coverage:
        return self.coverage


class FakeManipulator:
    """Succeeds at once unless told to fail or to wait for a stop. `begun`
    resolves when a sequence has reached it, so a test knows the sequence
    is running without waiting on the clock."""

    name = "fake"

    def __init__(self, *, fail: str = "", wait_for_stop: bool = False) -> None:
        self.fail = fail
        self.wait_for_stop = wait_for_stop
        self.calls: list[tuple] = []
        self.stopped = 0
        self.senses = None
        self.robot = None
        self.limb_states = None
        self._began = asyncio.Event()

    def why_unavailable(self) -> str:
        return ""

    async def start(self, senses) -> None:
        self.senses = senses
        self.robot = senses.robot
        self.limb_states = senses.limb_states

    async def begun(self) -> None:
        await self._began.wait()
        self._began.clear()

    async def _run(self, cancel: CancelToken, final_position=None) -> Outcome:
        self._began.set()
        if self.wait_for_stop:
            await cancel.wait()
            cancel.check()
        if self.fail:
            return Outcome(False, self.fail)
        return Outcome(True, "", final_position)

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken) -> Outcome:
        self.calls.append(("grab", item.item_id, gripper.name, max_effort))
        return await self._run(cancel)

    async def drop(self, gripper: Gripper, cancel: CancelToken) -> Outcome:
        self.calls.append(("drop", gripper.name))
        return await self._run(cancel)

    async def place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome:
        self.calls.append(("place", gripper.name, pose.position))
        return await self._run(cancel, final_position=(pose.position[0], pose.position[1], pose.position[2] + 0.01))

    async def stop(self) -> None:
        self.stopped += 1


# The reason peppy's goal context gives for each cause of a cancel signal.
PEPPY_CANCEL_REASONS = {"caller": "cancelled by its caller", "sender_gone": "its sender is gone"}


class FakeCtx:
    """Stands in for the generated GoalContext of `action`, a fake
    generated module: holds the goal, records how it ended, once, after
    checking that a result's fields are the ones the action's result
    carries and that a failure's kind is one the action declares, and lets
    a test send the goal its cancel signal, with the cause a caller's
    cancel or a sender that is gone gives and the reason peppy gives for
    that cause."""

    def __init__(self, action, **data) -> None:
        self.action = action
        self._request = SimpleNamespace(data=SimpleNamespace(**data))
        self._cancel = asyncio.Event()
        self._cause: Optional[str] = None
        self.completed: Optional[dict] = None
        self.cancelled: Optional[dict] = None
        self.cancelled_reason: Optional[str] = None
        self.failed = None

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


def camera_poses(names=("chest",)):
    """The robot's get_camera_poses response: every camera of `names` at
    the origin, its optical frame the robot frame's."""
    return SimpleNamespace(
        timestamp=7.0, camera_names=list(names),
        positions=[0.0, 0.0, 0.0] * len(names), orientations=[0.0, 0.0, 0.0, 1.0] * len(names),
        carried_by=[""] * len(names),
    )


def rgb_frame(width: int = 16, height: int = 12, *, frame_id: int = 1, align_mode: str = "depth_to_color"):
    """A video_stream message of the given size, plain grey."""
    return SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame_id=frame_id, align_mode=align_mode),
        encoding="rgb8",
        width=width,
        height=height,
        frame=bytes([128]) * (width * height * 3),
    )


def depth_frame(depth_m: float, width: int = 16, height: int = 12, unit: float = 0.001, *, frame_id: int = 1, align_mode: str = "depth_to_color"):
    """A z16 depth_stream message of the given size, `depth_m` everywhere."""
    value = int(round(depth_m / unit))
    return SimpleNamespace(
        header=SimpleNamespace(timestamp=1.0, frame_id=frame_id, align_mode=align_mode),
        encoding="z16",
        width=width,
        height=height,
        frame=np.full((height, width), value, dtype="<u2").tobytes(),
    )


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
    *, detector=None, manipulator=None, source=None, ticks=None, bindings=None, perception_model=None, manipulation_model=None
) -> Brain:
    """A started brain on fake bindings whose roles run `detector` and
    `manipulator`, the none backends when not given, from `source` or from
    a FakeSource that offers those two. The model parameters name the two,
    unless the test names others."""
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
    await brain.start()
    return brain

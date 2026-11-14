"""One entry per robot model: what a robot of that model is made of, whatever
engine stands it.

A robot's limbs and cameras come from its description, the URDF and SRDF its
attach goal carries, as robot_description_py reads them: each arm group with
the joints it moves, each gripper group with where its closed and open states
put each finger and the finger each mimic finger follows, the optical frame
of each camera with the depths a depth camera measures, and the posture the
robot starts in, every arm at its home state and every gripper closed. The
limbs have the names the robot answers to in limb_motion and limb_state
(left_arm and left_gripper on an OpenArm, arm and gripper on an SO-101).

What the description does not hold, how the engines render each camera of a
model, ships with this package under models/, one file per model that has a
camera, so the engines agree on it by reading the same file. What an engine
alone knows about a model (the file it loads, its gains, its extras) is
that engine's own entry, one file per model beside the
engine's code. `Models.read` reads both once, as the engine starts: an engine
stands the models it has an entry for, and a model with no entry is refused
with the ones it does stand. Every refusal raises ModelEntryError.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path

import pyjson5
from robot_description_py import HOME, RobotDescription
from robot_description_py import model as bundled_model

from sim_robot_core.cameras import (
    CameraConfig,
    CameraRender,
    camera_configs,
    parse_renders,
)
from sim_robot_core.errors import ModelEntryError
from sim_robot_core.pairs import Held

_ENTRIES_DIR = Path(__file__).parent / "models"
_ENTRY_SUFFIX = ".json5"
_ENTRY_KEYS = frozenset({"cameras"})

# The least travel a finger has from closed to open, in its joint's unit.
_LEAST_TRAVEL = 1e-9


@dataclass(frozen=True)
class Arm:
    """One arm of a robot: the name the robot answers to for it, and the
    joints it moves, in the order its pair carries them."""

    name: str
    joints: tuple[str, ...]


@dataclass(frozen=True)
class FingerSpan:
    """One finger joint's closed position and its signed travel to fully
    open, so an opening fraction f sits at closed + f * travel."""

    closed: float
    travel: float

    def position(self, opening: float) -> float:
        return self.closed + self.travel * opening

    def opening(self, position: float) -> float:
        return (position - self.closed) / self.travel


@dataclass(frozen=True)
class Mimic:
    """How a finger joint follows another finger joint of its gripper, one
    that moves of its own, as the description's mimic gives it: it stands at
    multiplier times that joint's position plus offset, in the joints'
    unit."""

    joint: str
    multiplier: float
    offset: float


@dataclass(frozen=True)
class Gripper:
    """One gripper of a robot: the name the robot answers to for it, the
    finger joints it moves, the span of each, and the mimic each follows,
    None for a joint that moves of its own, index-aligned."""

    name: str
    joints: tuple[str, ...]
    spans: tuple[FingerSpan, ...]
    mimics: tuple[Mimic | None, ...]

    def driven(self) -> tuple[tuple[str, FingerSpan], ...]:
        """Each finger joint that moves of its own, which a motor drives,
        with its span; every other finger follows one of them."""
        return tuple(
            (joint, span)
            for joint, span, mimic in zip(self.joints, self.spans, self.mimics, strict=True)
            if mimic is None
        )

    def driven_joints(self) -> tuple[str, ...]:
        return tuple(joint for joint, _ in self.driven())

    def followers(self) -> tuple[tuple[str, Mimic], ...]:
        """Each finger joint that follows another, with how it follows."""
        return tuple(
            (joint, mimic)
            for joint, mimic in zip(self.joints, self.mimics, strict=True)
            if mimic is not None
        )


@dataclass(frozen=True)
class ModelEntry:
    """What a robot of one model is made of."""

    model: str
    arms: tuple[Arm, ...]
    grippers: tuple[Gripper, ...]
    cameras: tuple[CameraConfig, ...]
    # Joint name to the position the robot starts in.
    start_posture: dict[str, float]

    def arm_names(self) -> list[str]:
        return [arm.name for arm in self.arms]

    def arm_joint_counts(self) -> list[int]:
        return [len(arm.joints) for arm in self.arms]

    def gripper_names(self) -> list[str]:
        return [gripper.name for gripper in self.grippers]

    def arm_joints(self) -> list[str]:
        return [joint for arm in self.arms for joint in arm.joints]

    def finger_joints(self) -> list[str]:
        return [joint for gripper in self.grippers for joint in gripper.joints]

    def joints(self) -> list[str]:
        """Every joint a robot of this model moves."""
        return [*self.arm_joints(), *self.finger_joints()]

    def rgb_camera_names(self) -> list[str]:
        return [camera.name for camera in self.cameras if camera.depth is None]

    def rgbd_camera_names(self) -> list[str]:
        return [camera.name for camera in self.cameras if camera.depth is not None]

    def has(self) -> Held:
        """Everything a robot of this model can hold a pair for."""
        return Held(
            arms=frozenset(self.arm_names()),
            grippers=frozenset(self.gripper_names()),
            rgb_cameras=frozenset(self.rgb_camera_names()),
            rgbd_cameras=frozenset(self.rgbd_camera_names()),
        )

    def holds_every_limb(self, held: Held) -> bool:
        """Whether `held` drives every limb of this model, which is what a
        robot's readiness asks: a setpoint reaches each limb and its state
        comes back."""
        has = self.has()
        return has.arms <= held.arms and has.grippers <= held.grippers

    def foreign(self, held: Held) -> Held:
        """The pairs of `held` that name a limb or a camera this model lacks.
        Such a pair never drives or streams anything, so a robot holding one
        does not stay."""
        has = self.has()
        return Held(
            arms=held.arms - has.arms,
            grippers=held.grippers - has.grippers,
            rgb_cameras=held.rgb_cameras - has.rgb_cameras,
            rgbd_cameras=held.rgbd_cameras - has.rgbd_cameras,
        )

    def mismatch(self, held: Held) -> str | None:
        """Why `held` is not what a robot of this model pairs, naming both
        lists, or None when it is: every limb of the model and nothing the
        model lacks. A camera the model has may go unpaired, since a camera
        nobody views is not rendered."""
        if self.holds_every_limb(held) and self.foreign(held).is_empty():
            return None
        return (
            f"its pairs name {held.describe()}, and the model {self.model} has "
            f"{self.has().describe()}"
        )


def _fail(source: str, reason: str) -> ModelEntryError:
    return ModelEntryError(f"{source}: {reason}")


def _gripper(source: str, gripper) -> Gripper:
    spans = []
    for finger in gripper.fingers:
        span = FingerSpan(closed=finger.closed, travel=finger.open - finger.closed)
        if abs(span.travel) <= _LEAST_TRAVEL:
            raise _fail(
                source,
                f"finger joint '{finger.joint}' of '{gripper.name}' closes at {finger.closed} "
                f"and opens at {finger.open}: it has no travel",
            )
        spans.append(span)
    return Gripper(
        name=gripper.name,
        joints=tuple(finger.joint for finger in gripper.fingers),
        spans=tuple(spans),
        mimics=tuple(
            None
            if finger.mimic is None
            else Mimic(
                joint=finger.mimic.joint,
                multiplier=finger.mimic.multiplier,
                offset=finger.mimic.offset,
            )
            for finger in gripper.fingers
        ),
    )


def _start_posture(source: str, description: RobotDescription) -> dict[str, float]:
    posture: dict[str, float] = {}
    for arm in description.arms:
        home = description.state(arm.name, HOME)
        if home is None:
            raise _fail(source, f"the description gives the arm '{arm.name}' no {HOME} state")
        posture.update(zip(arm.joints, home, strict=True))
        posture.update((finger.joint, finger.closed) for finger in arm.gripper.fingers)
    return posture


def _entry_of(
    model: str,
    source: str,
    description: RobotDescription,
    renders: dict[str, CameraRender] | None,
) -> ModelEntry:
    """The entry of a robot of `model` that `description` describes, each of
    its cameras rendered as `renders` gives the camera of the same name, the
    renders of the entry at `source`, or None where no file is there. A
    camera of the description with no render and a render of no camera of
    the description are refused naming the entry; an arm with no home state
    and a finger that does not travel, naming the model."""
    described = f"model {model}"
    arms = tuple(Arm(name=arm.name, joints=arm.joints) for arm in description.arms)
    grippers = tuple(_gripper(described, arm.gripper) for arm in description.arms)
    if not arms:
        raise _fail(described, "the description names no arm")
    return ModelEntry(
        model=model,
        arms=arms,
        grippers=grippers,
        cameras=camera_configs(source, description.cameras, renders),
        start_posture=_start_posture(described, description),
    )


def _read_json5(path: Path):
    try:
        return pyjson5.loads(path.read_text())
    except pyjson5.Json5Exception as error:
        raise _fail(str(path), f"does not read as JSON5: {error}") from error


def _parse_shipped(source: str, raw) -> dict[str, CameraRender]:
    """How the engines render each camera of a model, by the camera's name,
    from `raw`, the entry this package ships for the model, which `source`
    names."""
    if not isinstance(raw, dict):
        raise _fail(source, f"a model entry must be an object, got {raw!r}")
    unknown = sorted(set(raw) - _ENTRY_KEYS)
    if unknown:
        raise _fail(source, f"model entry has unknown key(s) {unknown}; allowed: {sorted(_ENTRY_KEYS)}")
    if "cameras" not in raw:
        raise _fail(source, "model entry has no `cameras`: add `cameras: { <camera>: {...} }`, or delete the file")
    return parse_renders(source, raw["cameras"])


def _shipped(model: str) -> tuple[str, dict[str, CameraRender] | None]:
    """The path of the entry this package ships for `model`, and how it
    renders each camera, or None where no file is there."""
    path = _ENTRIES_DIR / f"{model}{_ENTRY_SUFFIX}"
    if not path.is_file():
        return str(path), None
    return str(path), _parse_shipped(str(path), _read_json5(path))


def shipped_entry(model: str, description: RobotDescription) -> ModelEntry:
    """The entry of a robot of `model` that `description` describes, with
    the cameras this package's entry for `model` renders."""
    source, renders = _shipped(model)
    return _entry_of(model, source, description, renders)


def hold_to_bundled(model: str, description: RobotDescription) -> None:
    """Refuses, with ModelEntryError, a `description` of `model` whose URDF or
    SRDF is not byte for byte the file robot_description bundles for `model`
    where this package runs, naming the SHA-256 of each file of both. A model
    robot_description bundles none of is refused with ValueError naming the
    models it bundles."""
    bundled = bundled_model(model)
    if description.urdf == bundled.urdf and description.srdf == bundled.srdf:
        return

    def sha256(text: str) -> str:
        return hashlib.sha256(text.encode()).hexdigest()

    raise _fail(
        f"model {model}",
        "the robot attaches with another description than the one this simulation was built with: "
        f"its URDF has SHA-256 {sha256(description.urdf)} and its SRDF {sha256(description.srdf)}, "
        f"this simulation's URDF {sha256(bundled.urdf)} and SRDF {sha256(bundled.srdf)}; build the "
        "robot's initializer and this simulation from one public-peppy-libs commit",
    )


@dataclass(frozen=True)
class EngineModel:
    """One model an engine stands: how this package's entry renders its
    cameras, and what this engine alone knows about it, as its own entry
    wrote it."""

    model: str
    # The path of the entry this package ships for the model, which a
    # refusal of a camera at an attach names, and how it renders each
    # camera, by name, or None where no file is there.
    source: str
    renders: dict[str, CameraRender] | None
    engine: dict

    def entry(self, description: RobotDescription) -> ModelEntry:
        """The entry of a robot of this model that `description` describes."""
        return _entry_of(self.model, self.source, description, self.renders)


class Models:
    """The models an engine stands, by the id a robot attaches with."""

    def __init__(self, models: dict[str, EngineModel]) -> None:
        self._models = dict(models)

    @staticmethod
    def read(engine_entries_dir: Path) -> Models:
        """Reads every entry of the engine, one `<model>.json5` per model
        under `engine_entries_dir`, with the entry this package ships for
        the same model."""
        paths = sorted(engine_entries_dir.glob(f"*{_ENTRY_SUFFIX}"))
        if not paths:
            raise ModelEntryError(
                f"{engine_entries_dir} holds no model entry: write one <model>{_ENTRY_SUFFIX} "
                "per model the engine stands"
            )
        models: dict[str, EngineModel] = {}
        for path in paths:
            model = path.name[: -len(_ENTRY_SUFFIX)]
            engine = _read_json5(path)
            if not isinstance(engine, dict):
                raise _fail(str(path), f"an engine's model entry must be an object, got {engine!r}")
            source, renders = _shipped(model)
            models[model] = EngineModel(model=model, source=source, renders=renders, engine=engine)
        return Models(models)

    def names(self) -> list[str]:
        return sorted(self._models)

    def of(self, model: str) -> EngineModel:
        """The model a robot attaches as. An id this engine has no entry for
        names the ones it stands, so a launcher's typo says what to write
        instead."""
        known = self._models.get(model)
        if known is None:
            raise ModelEntryError(
                f"unknown model {model!r}: this engine stands {', '.join(self.names())}"
            )
        return known

"""One entry per robot model: what a robot of that model is made of, whatever
engine stands it.

A robot's limbs, base, cameras and lidars come from its description, the URDF and
SRDF its attach goal carries, as robot_description_py reads them: each arm
group with the joints it moves, each gripper group with where its closed and
open states put each finger and the finger each mimic finger follows, the
wheels of the differential base the description gives, the optical frame of
each camera with the depths a depth camera measures, the frame of each lidar
with the ranges it measures, and the posture the
robot starts in, every arm at its home state and every gripper closed. The
limbs have the names the robot answers to in limb_motion and limb_state
(left_arm and left_gripper on an OpenArm, arm and gripper on an SO-101), and
the base the name it answers to in base_state (base). A robot with a base
rolls: it stands free on the floor and its wheels carry it.

What the description does not hold, how the engines render each camera and
sweep each lidar of a model, ships with this package under models/, one file
per model that has a camera or a lidar, so the engines agree on it by reading
the same file. What an engine
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
from robot_description_py import HOME, FingerSpan, Mimic, NoDifferentialBase, RobotDescription
from robot_description_py import model as bundled_model

from sim_robot_core.cameras import (
    CameraConfig,
    CameraRender,
    camera_configs,
    parse_renders,
)
from sim_robot_core.errors import ModelEntryError
from sim_robot_core.lidars import LidarConfig, LidarSweep, lidar_configs, parse_sweeps
from sim_robot_core.pairs import ARMS, BASE_NAME, BASES, DRIVEN_SLOTS, GRIPPERS, SLOTS, Held

_ENTRIES_DIR = Path(__file__).parent / "models"
_ENTRY_SUFFIX = ".json5"
_ENTRY_KEYS = frozenset({"cameras", "lidars"})


@dataclass(frozen=True)
class Arm:
    """One arm of a robot: the name the robot answers to for it, and the
    joints it moves, in the order its pair carries them."""

    name: str
    joints: tuple[str, ...]


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
class Base:
    """The base of a robot: the name the robot answers to for it, and the
    continuous joints of its two driven wheels, left then right, in the
    order its pair carries them, as the description's differential base
    gives them."""

    name: str
    wheels: tuple[str, ...]


@dataclass(frozen=True)
class ModelEntry:
    """What a robot of one model is made of."""

    model: str
    arms: tuple[Arm, ...]
    grippers: tuple[Gripper, ...]
    cameras: tuple[CameraConfig, ...]
    lidars: tuple[LidarConfig, ...]
    # Joint name to the position the robot starts in: every arm and finger
    # joint. A wheel starts where it stands.
    start_posture: dict[str, float]
    # None for a robot that stands where it is placed.
    base: Base | None

    @property
    def rolls(self) -> bool:
        """Whether a robot of this model rolls on the wheels of its base: it
        stands free on the floor, its placement where it starts, and its
        wheels carry it from there."""
        return self.base is not None

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

    def base_names(self) -> list[str]:
        return [] if self.base is None else [self.base.name]

    def wheel_joints(self) -> list[str]:
        return [] if self.base is None else list(self.base.wheels)

    def joints(self) -> list[str]:
        """Every joint a robot of this model moves."""
        return [*self.arm_joints(), *self.finger_joints(), *self.wheel_joints()]

    def rgb_camera_names(self) -> list[str]:
        return [camera.name for camera in self.cameras if camera.depth is None]

    def rgbd_camera_names(self) -> list[str]:
        return [camera.name for camera in self.cameras if camera.depth is not None]

    def lidar_names(self) -> list[str]:
        return [lidar.name for lidar in self.lidars]

    def has(self) -> Held:
        """Everything a robot of this model can hold a pair for."""
        return Held(
            arms=frozenset(self.arm_names()),
            grippers=frozenset(self.gripper_names()),
            bases=frozenset(self.base_names()),
            rgb_cameras=frozenset(self.rgb_camera_names()),
            rgbd_cameras=frozenset(self.rgbd_camera_names()),
            lidars=frozenset(self.lidar_names()),
        )

    def holds_every_driven_part(self, held: Held) -> bool:
        """Whether `held` drives every limb and the base of this model, which
        is what a robot's readiness asks: a setpoint reaches each of them and
        its state comes back."""
        has = self.has()
        return all(has.of_slot(slot) <= held.of_slot(slot) for slot in DRIVEN_SLOTS)

    def foreign(self, held: Held) -> Held:
        """The pairs of `held` that name a limb, a base, a camera or a lidar
        this model lacks. Such a pair never drives or streams anything, so a robot
        holding one does not stay."""
        has = self.has()
        return Held(**{slot: held.of_slot(slot) - has.of_slot(slot) for slot in SLOTS})

    def joints_of(self, slot: str, name: str) -> tuple[str, ...] | None:
        """The joints the arm, the gripper or the base `name` of this model
        moves, by the slot its pair is on: an arm's group's joints in the
        order its pair carries them, a gripper's fingers, a base's wheels,
        left then right. None for a part the model lacks, whose pair drives
        nothing."""
        parts = {
            ARMS: [(arm.name, arm.joints) for arm in self.arms],
            GRIPPERS: [(gripper.name, gripper.joints) for gripper in self.grippers],
            BASES: [(base.name, base.wheels) for base in (self.base,) if base is not None],
        }[slot]
        return next((joints for part, joints in parts if part == name), None)

    def mismatch(self, held: Held) -> str | None:
        """Why `held` is not what a robot of this model pairs, naming both
        lists, or None when it is: every limb and the base of the model and
        nothing the model lacks. A camera or a lidar the model has may go
        unpaired, since a camera nobody views is not rendered and a lidar
        nobody views is not swept."""
        if self.holds_every_driven_part(held) and self.foreign(held).is_empty():
            return None
        return (
            f"its pairs name {held.describe()}, and the model {self.model} has "
            f"{self.has().describe()}"
        )


def _fail(source: str, reason: str) -> ModelEntryError:
    return ModelEntryError(f"{source}: {reason}")


def _gripper(gripper) -> Gripper:
    return Gripper(
        name=gripper.name,
        joints=tuple(finger.joint for finger in gripper.fingers),
        spans=tuple(finger.span for finger in gripper.fingers),
        mimics=tuple(finger.mimic for finger in gripper.fingers),
    )


def _base(described: str, description: RobotDescription) -> Base | None:
    """The base of the robot `description` describes, the wheels of its
    differential base, or None for a robot with arms whose base body carries
    no wheel. Any other refusal of its wheels is refused, naming the
    refusal's kind and its message, which names the joints: a robot with arms
    whose wheels make no differential base, a group of its SRDF holding them
    among them, and a description that names no arm and gives no base."""
    try:
        wheels = description.differential_base()
    except NoDifferentialBase as no_base:
        if description.arms and _carries_no_wheel(description, no_base):
            return None
        what = (
            "the description's wheels make no differential base"
            if description.arms
            else "the description names no arm and gives no differential base"
        )
        raise _fail(described, f"{what} ({no_base.kind}): {no_base}") from no_base
    return Base(name=BASE_NAME, wheels=(wheels.left.joint, wheels.right.joint))


def _carries_no_wheel(description: RobotDescription, no_base: NoDifferentialBase) -> bool:
    """Whether `no_base`, the refusal of the wheels of `description`, counts
    no wheel on its base body: no continuous joint there that no group of its
    SRDF holds, and each one a group holds a joint of one of its arms."""
    arm_joints = {joint for arm in description.arms for joint in arm.joints}
    return (
        no_base.kind == NoDifferentialBase.WHEEL_COUNT
        and not no_base.found
        and set(no_base.grouped) <= arm_joints
    )


def _start_posture(source: str, description: RobotDescription) -> dict[str, float]:
    posture: dict[str, float] = {}
    for arm in description.arms:
        try:
            home = description.state(arm.name, HOME)
        except KeyError as unknown:
            reason = f"the description gives the arm '{arm.name}' no {HOME} state"
            raise _fail(source, reason) from unknown
        posture.update(zip(arm.joints, home, strict=True))
        posture.update((finger.joint, finger.span.closed) for finger in arm.gripper.fingers)
    return posture


def _entry_of(
    model: str,
    source: str,
    description: RobotDescription,
    renders: dict[str, CameraRender] | None,
    sweeps: dict[str, LidarSweep] | None,
) -> ModelEntry:
    """The entry of a robot of `model` that `description` describes, each of
    its cameras rendered as `renders` gives the camera of the same name and
    each of its lidars swept as `sweeps` gives the lidar of the same name,
    the renders and the sweeps of the entry at `source`, each None where no
    file is there. A camera or a lidar of the description with no render or
    sweep, and a render or a sweep of no camera or lidar of the description,
    are refused naming the entry; an arm with no home state,
    a description whose wheels make no differential base, and one that names
    no arm and gives no differential base,
    naming the model and why its wheels make no base."""
    described = f"model {model}"
    arms = tuple(Arm(name=arm.name, joints=arm.joints) for arm in description.arms)
    grippers = tuple(_gripper(arm.gripper) for arm in description.arms)
    base = _base(described, description)
    return ModelEntry(
        model=model,
        arms=arms,
        grippers=grippers,
        cameras=camera_configs(source, description, renders),
        lidars=lidar_configs(source, description, sweeps),
        start_posture=_start_posture(described, description),
        base=base,
    )


def _read_json5(path: Path):
    try:
        return pyjson5.loads(path.read_text())
    except pyjson5.Json5Exception as error:
        raise _fail(str(path), f"does not read as JSON5: {error}") from error


def _parse_shipped(
    source: str, raw
) -> tuple[dict[str, CameraRender], dict[str, LidarSweep]]:
    """How the engines render each camera and sweep each lidar of a model,
    each by its name, from `raw`, the entry this package ships for the model,
    which `source` names. An entry holds cameras, lidars or both."""
    if not isinstance(raw, dict):
        raise _fail(source, f"a model entry must be an object, got {raw!r}")
    unknown = sorted(set(raw) - _ENTRY_KEYS)
    if unknown:
        raise _fail(source, f"model entry has unknown key(s) {unknown}; allowed: {sorted(_ENTRY_KEYS)}")
    if not raw:
        raise _fail(
            source,
            "model entry renders no camera and sweeps no lidar: add `cameras: { <camera>: {...} }` "
            "or `lidars: { <lidar>: {...} }`, or delete the file",
        )
    renders = parse_renders(source, raw["cameras"]) if "cameras" in raw else {}
    sweeps = parse_sweeps(source, raw["lidars"]) if "lidars" in raw else {}
    return renders, sweeps


def _shipped(
    model: str,
) -> tuple[str, dict[str, CameraRender] | None, dict[str, LidarSweep] | None]:
    """The path of the entry this package ships for `model`, how it renders
    each camera and how it sweeps each lidar, each None where no file is
    there."""
    path = _ENTRIES_DIR / f"{model}{_ENTRY_SUFFIX}"
    if not path.is_file():
        return str(path), None, None
    renders, sweeps = _parse_shipped(str(path), _read_json5(path))
    return str(path), renders, sweeps


def shipped_entry(model: str, description: RobotDescription) -> ModelEntry:
    """The entry of a robot of `model` that `description` describes, with
    the cameras and the lidars this package's entry for `model` renders and
    sweeps."""
    source, renders, sweeps = _shipped(model)
    return _entry_of(model, source, description, renders, sweeps)


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
    cameras and sweeps its lidars, and what this engine alone knows about
    it, as its own entry wrote it."""

    model: str
    # The path of the entry this package ships for the model, which a
    # refusal of a camera or a lidar at an attach names, and how it renders
    # each camera and sweeps each lidar, by name, each None where no file is
    # there.
    source: str
    renders: dict[str, CameraRender] | None
    sweeps: dict[str, LidarSweep] | None
    engine: dict

    def entry(self, description: RobotDescription) -> ModelEntry:
        """The entry of a robot of this model that `description` describes."""
        return _entry_of(self.model, self.source, description, self.renders, self.sweeps)


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
            source, renders, sweeps = _shipped(model)
            models[model] = EngineModel(
                model=model, source=source, renders=renders, sweeps=sweeps, engine=engine
            )
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

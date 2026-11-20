"""The sensor a node runs as, as sensor_mounts:v1 places it: the name it runs
under and the frame it names in each measurement. With its sensor_mounts link
bound, that frame is the one describe_sensors gives the sensor of that name;
with the link vacant, it is a frame of the sensor's own name, and a consumer
whose tree holds no frame of that name drops the measurement with
unknown_frame. A sensor node reads it once at its start through `read_sensor`,
before it opens anything, with the SensorClass of the sensors it streams as: a
camera with CAMERA. The same rules and messages as camera_core's sensor_mounts
module, which the Rust sensor nodes share."""

import unicodedata
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass
from typing import Protocol

from control_core_py.runtime import CancellationToken, ask

# How long one ask of describe_sensors waits for its answer, in seconds; an
# ask that runs out is asked again.
DESCRIBE_SENSORS_TIMEOUT_S = 2.0


# Why an empty name names no frame, as frame_tree's FrameNameError says it.
EMPTY_FRAME_NAME = "a frame name is empty: name a link of the robot's URDF, odom or map"


@dataclass(frozen=True)
class SensorClass:
    """A class of sensors a node streams as: the contracts it streams on, as
    describe_sensors names a sensor's kind, and the noun the refusals of the
    reader call such a sensor."""

    noun: str
    kinds: tuple[str, ...]


# The cameras: a sensor that streams on rgb_camera or rgbd_camera, whose frame
# is its colour optical frame.
CAMERA = SensorClass(noun="camera", kinds=("rgb_camera", "rgbd_camera"))


class DescribedSensors(Protocol):
    """The fields of an answer of describe_sensors that place each sensor, as
    the data of a generated response holds them: index-aligned."""

    sensor_names: Sequence[str]
    sensor_kinds: Sequence[str]
    sensor_frames: Sequence[str]
    carried_by: Sequence[str]


@dataclass(frozen=True)
class MountedSensor:
    """A sensor as sensor_mounts places it: the class it was read as, the name
    it runs under and the frame each of its measurements names."""

    sensor_class: SensorClass
    name: str
    frame: str


class SensorMountsError(ValueError):
    """Why a sensor has no frame to name in its measurements."""


# The characters Rust's `{:?}` writes as an escape of their own in a string.
_DEBUG_ESCAPES = {"\0": "\\0", "\t": "\\t", "\r": "\\r", "\n": "\\n", '"': '\\"', "\\": "\\\\"}

# The characters that Python's Unicode database, 15.0 under CPython 3.12 and 15.1 under 3.13,
# calls printable and no mark of Mn or Me, and that Rust 1.100's `{:?}` writes as \u{hex}: the
# spacing marks of Rust's Grapheme_Extend, such as U+09BE and U+1D165, and the four Hangul
# fillers. Measured over every code point both databases assign, as is _KEPT_MARKS.
_ESCAPED_BEYOND_MARKS = frozenset({
    0x09BE, 0x09D7, 0x0B3E, 0x0B57, 0x0BBE, 0x0BD7, 0x0CC0, 0x0CC2, 0x0CC7, 0x0CC8, 0x0CCA,
    0x0CCB, 0x0CD5, 0x0CD6, 0x0D3E, 0x0D57, 0x0DCF, 0x0DDF, 0x115F, 0x1160, 0x1715, 0x1734,
    0x1B35, 0x1B3B, 0x1B3D, 0x1B43, 0x1B44, 0x1BAA, 0x1BF2, 0x1BF3, 0x302E, 0x302F, 0x3164,
    0xA953, 0xA9C0, 0xFFA0, 0x111C0, 0x11235, 0x1133E, 0x1134D, 0x11357, 0x114B0, 0x114BD,
    0x115AF, 0x116B6, 0x11930, 0x1193D, 0x11F41, 0x16FF0, 0x16FF1, 0x1D165, 0x1D166, 0x1D16D,
    0x1D16E, 0x1D16F, 0x1D170, 0x1D171, 0x1D172
})

# The one mark of Mn in Python's Unicode database that Rust 1.100's `{:?}` writes as it is:
# U+1171E, a spacing mark in Rust's.
_KEPT_MARKS = frozenset({0x1171E})



def _quoted(text: str) -> str:
    """`text` in double quotes, as Rust 1.100's `{:?}` writes a string: `\\0`,
    `\\t`, `\\r`, `\\n`, `\\"` and `\\\\` for those characters, `\\u{hex}`
    for a character that is not printable, that is a mark of the category
    Mn or Me but U+1171E, or that _ESCAPED_BEYOND_MARKS holds, and every other
    character as it is."""
    return '"' + "".join(map(_debug_character, text)) + '"'


def _debug_character(character: str) -> str:
    """`character` as Rust's `{:?}` writes it in a string."""
    if character in _DEBUG_ESCAPES:
        return _DEBUG_ESCAPES[character]
    code = ord(character)
    is_mark = unicodedata.category(character) in ("Mn", "Me") and code not in _KEPT_MARKS
    if not character.isprintable() or is_mark or code in _ESCAPED_BEYOND_MARKS:
        return f"\\u{{{code:x}}}"
    return character


async def read_sensor(
    sensor_class: SensorClass,
    copy: str | None,
    instance_id: str,
    describe_sensors: Callable[[], Awaitable[DescribedSensors]] | None,
    token: CancellationToken,
) -> MountedSensor | None:
    """The sensor of `sensor_class` that runs as `instance_id` in `copy`, as
    sensor_mounts places it: its name, its instance id in its copy, and the
    frame it names. With `describe_sensors`, the node's poll of its bound
    sensor_mounts link with DESCRIBE_SENSORS_TIMEOUT_S, the frame is the one
    describe_sensors gives that name, asked until it answers, and None once
    `token` cancels first. With None, for a vacant link, the frame is the
    name, and nothing is asked. An instance id that names no sensor in its
    copy is refused before the first ask, and an answer that places no sensor
    of the class under the name is refused with what to run."""
    name = sensor_name(sensor_class, copy, instance_id)
    if describe_sensors is None:
        return MountedSensor(sensor_class=sensor_class, name=name, frame=name)
    sensors = await ask("describe_sensors", token, describe_sensors)
    if sensors is None:
        return None
    return MountedSensor(
        sensor_class=sensor_class,
        name=name,
        frame=sensor_frame(sensor_class, sensors, name),
    )


def sensor_name(sensor_class: SensorClass, copy: str | None, instance_id: str) -> str:
    """The name a sensor of `sensor_class` runs under: its instance id in its
    copy, the name sensor_mounts and a simulation give it. A launch mints a
    copy's instance ids as `<copy>_<id>`, so camera `alpha_wrist_left` of copy
    `alpha` runs under `wrist_left`, and a sensor launched outside a copy runs
    under the id the launcher wrote."""
    if copy is None:
        return instance_id
    prefix = f"{copy}_"
    name = instance_id.removeprefix(prefix)
    if not instance_id.startswith(prefix) or not name:
        raise SensorMountsError(
            f"the instance id {_quoted(instance_id)} names no {sensor_class.noun} of copy "
            f"{_quoted(copy)}: write it as `{prefix}` followed by the {sensor_class.noun}'s name"
        )
    return name


def sensor_frame(
    sensor_class: SensorClass, sensors: DescribedSensors, name: str
) -> str:
    """The frame that `sensors`, sensor_mounts' answer, gives the sensor of
    `sensor_class` running under `name`."""
    counts = (
        len(sensors.sensor_names),
        len(sensors.sensor_kinds),
        len(sensors.sensor_frames),
        len(sensors.carried_by),
    )
    if len(set(counts)) != 1:
        names, kinds, frames, carriers = counts
        raise SensorMountsError(
            f"sensor_mounts' describe_sensors names {names} sensors and gives "
            f"{kinds} sensor_kinds, {frames} sensor_frames and {carriers} "
            "carried_by: one of each for each sensor"
        )
    if name not in sensors.sensor_names:
        listed = ", ".join(sensors.sensor_names) or "none"
        raise SensorMountsError(
            f"sensor_mounts' describe_sensors names no sensor {_quoted(name)} (its "
            f"sensor_names: {listed}): run the {sensor_class.noun} under one of those names, or "
            "bind its sensor_mounts link to the robot that carries it, whose "
            "robot_identity implementer serves sensor_mounts"
        )
    index = list(sensors.sensor_names).index(name)
    kind = sensors.sensor_kinds[index]
    if kind not in sensor_class.kinds:
        noun = sensor_class.noun
        raise SensorMountsError(
            f"sensor_mounts' describe_sensors names {_quoted(name)} a sensor of {_quoted(kind)}: "
            f"run the {noun} under the name of a {noun}, a sensor of "
            f"{' or '.join(sensor_class.kinds)}"
        )
    frame = sensors.sensor_frames[index]
    if not frame:
        raise SensorMountsError(
            f"sensor_mounts' describe_sensors gives {_quoted(name)} a sensor_frames "
            f"entry: {EMPTY_FRAME_NAME}"
        )
    return frame

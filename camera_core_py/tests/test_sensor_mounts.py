import asyncio
import re
from dataclasses import dataclass, field
from pathlib import Path

import pytest
from control_core_py import runtime

from camera_core_py.sensor_mounts import (
    CAMERA,
    MountedSensor,
    SensorClass,
    SensorMountsError,
    read_sensor,
    sensor_frame,
    sensor_name,
)


@dataclass
class Answer:
    """The data of a describe_sensors response: a chest camera, a wrist camera
    on the left arm and a lidar on the base."""

    robot_frame: str = "base_footprint"
    sensor_names: list[str] = field(
        default_factory=lambda: ["chest", "wrist_left", "lidar"]
    )
    sensor_kinds: list[str] = field(
        default_factory=lambda: ["rgbd_camera", "rgb_camera", "lidar_2d"]
    )
    sensor_frames: list[str] = field(
        default_factory=lambda: [
            "chest_color_optical_frame",
            "wrist_left_color_optical_frame",
            "lidar_link",
        ]
    )
    carried_by: list[str] = field(default_factory=lambda: ["", "left", ""])


# The refusals camera_core::sensor_mounts gives in the same words: a call, its
# arguments and its message on each line, tab-separated.
REFUSALS = (
    Path(__file__).resolve().parents[2]
    / "camera_core"
    / "tests"
    / "fixtures"
    / "sensor_mounts_refusals.tsv"
)

# How an argument of the table writes a character it does not show: \u{X},
# the character of code point X.
_CODE_POINT = re.compile(r"\\u\{([0-9a-f]+)\}")


def _unescaped(field: str) -> str:
    return _CODE_POINT.sub(lambda code: chr(int(code.group(1), 16)), field)


def _items(field: str) -> list[str]:
    """The items of a list of the table, separated by commas: none for an
    empty field."""
    return [_unescaped(item) for item in field.split(",")] if field else []


# Each call of the table on its arguments, as camera_core's suite makes it.
CALLS = {
    "sensor_name": lambda copy, instance_id: sensor_name(
        CAMERA, _unescaped(copy), _unescaped(instance_id)
    ),
    "sensor_frame": lambda name, names, kinds, frames, carriers: sensor_frame(
        CAMERA,
        Answer(
            sensor_names=_items(names),
            sensor_kinds=_items(kinds),
            sensor_frames=_items(frames),
            carried_by=_items(carriers),
        ),
        _unescaped(name),
    ),
}


def _shared_refusals() -> list[list[str]]:
    lines = REFUSALS.read_text(encoding="utf-8").splitlines()
    return [line.split("\t") for line in lines if line and not line.startswith("#")]


@pytest.mark.parametrize(
    "fields", _shared_refusals(), ids=lambda fields: " ".join(fields[:-1])
)
def test_each_refusal_of_the_shared_table_gives_its_message(fields: list[str]):
    # A name in quotes as Rust's `{:?}` writes it: a character that is not
    # ASCII as it is, a control character escaped.
    call, *arguments, message = fields
    with pytest.raises(SensorMountsError) as refused:
        CALLS[call](*arguments)
    assert str(refused.value) == message


class Token:
    """A cancellation token that cancels when the test sets it."""

    def __init__(self, cancelled: bool = False) -> None:
        self._event = asyncio.Event()
        if cancelled:
            self._event.set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()

    async def cancelled(self) -> None:
        await self._event.wait()


class Script:
    """The answers of sensor_mounts, played in order: an Answer, or an
    exception the ask raises. An ask past the end of the script fails the
    test."""

    def __init__(self, *answers) -> None:
        self._answers = list(answers)
        self.asks = 0

    async def __call__(self) -> Answer:
        self.asks += 1
        assert self._answers, "an ask past the end of the script"
        answer = self._answers.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer


@pytest.mark.parametrize(
    ("name", "frame"),
    [
        ("chest", "chest_color_optical_frame"),
        ("wrist_left", "wrist_left_color_optical_frame"),
    ],
)
async def test_a_camera_of_either_kind_names_the_frame_sensor_mounts_gives_its_name(
    name, frame
):
    script = Script(Answer())
    camera = await read_sensor(CAMERA, None, name, script, Token())
    assert camera == MountedSensor(sensor_class=CAMERA, name=name, frame=frame)
    assert script.asks == 1


@pytest.mark.parametrize(
    ("copy", "instance_id", "name"),
    [("alpha", "alpha_wrist_left", "wrist_left"), (None, "chest", "chest")],
)
async def test_a_camera_with_a_vacant_link_names_a_frame_of_its_name_and_asks_nothing(
    copy, instance_id, name
):
    # The read ends though its cancel never comes.
    camera = await read_sensor(CAMERA, copy, instance_id, None, Token())
    assert camera == MountedSensor(sensor_class=CAMERA, name=name, frame=name)


async def test_a_vacant_link_refuses_an_id_that_names_no_camera_of_its_copy():
    with pytest.raises(SensorMountsError, match="^the instance id "):
        await read_sensor(CAMERA, "alpha", "bravo_wrist_left", None, Token())


async def test_sensor_mounts_is_asked_until_it_answers(monkeypatch):
    real_sleep = asyncio.sleep

    async def no_pause(_seconds):
        await real_sleep(0)

    # The pause between asks is control_core_py's ask loop's.
    monkeypatch.setattr(runtime.asyncio, "sleep", no_pause)
    script = Script(TimeoutError("no reply"), TimeoutError("no reply"), Answer())
    camera = await read_sensor(CAMERA, None, "chest", script, Token())
    assert camera == MountedSensor(
        sensor_class=CAMERA, name="chest", frame="chest_color_optical_frame"
    )
    assert script.asks == 3


async def test_a_stop_before_sensor_mounts_answers_gives_no_camera():
    script = Script()
    assert (
        await read_sensor(CAMERA, None, "chest", script, Token(cancelled=True)) is None
    )
    assert script.asks == 0


async def test_a_camera_in_a_copy_runs_under_its_id_after_the_copy():
    camera = await read_sensor(
        CAMERA, "alpha", "alpha_wrist_left", Script(Answer()), Token()
    )
    assert camera == MountedSensor(
        sensor_class=CAMERA, name="wrist_left", frame="wrist_left_color_optical_frame"
    )


async def test_an_id_that_is_no_name_in_its_copy_is_refused_before_sensor_mounts_is_asked():
    script = Script()
    with pytest.raises(SensorMountsError, match="^the instance id "):
        await read_sensor(CAMERA, "alpha", "bravo_wrist_left", script, Token())
    assert script.asks == 0


async def test_a_name_sensor_mounts_does_not_list_is_refused_naming_the_names_it_lists():
    with pytest.raises(
        SensorMountsError, match=r"\(its sensor_names: chest, wrist_left, lidar\)"
    ):
        await read_sensor(CAMERA, None, "head", Script(Answer()), Token())


def test_a_class_takes_its_own_kinds_and_its_refusals_use_its_noun():
    lidar = SensorClass(noun="lidar", kinds=("lidar_2d",))
    assert sensor_frame(lidar, Answer(), "lidar") == "lidar_link"
    with pytest.raises(SensorMountsError) as refused:
        sensor_frame(lidar, Answer(), "chest")
    assert str(refused.value) == (
        'sensor_mounts\' describe_sensors names "chest" a sensor of "rgbd_camera": run the '
        "lidar under the name of a lidar, a sensor of lidar_2d"
    )


def test_only_the_copys_own_prefix_comes_off_a_name():
    # A camera may carry the copy's name in its own.
    assert sensor_name(CAMERA, "alpha", "alpha_alpha_front") == "alpha_front"


def test_a_camera_outside_a_copy_runs_under_its_id():
    assert sensor_name(CAMERA, None, "wrist_left") == "wrist_left"
    assert sensor_name(CAMERA, None, "alpha_wrist_left") == "alpha_wrist_left"

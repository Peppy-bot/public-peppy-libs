"""The scenarios and reference outputs of the ROS reference harness, read for the
parity tests of this package as the Rust crate ros_reference reads them.

The scenarios of a kind are listed by the `kind` each one names. Reading a
scenario checks it against its digest in INPUTS.SHA256SUMS, and reading a
reference output against its digest in SHA256SUMS, both in the directory of
reference outputs: the harness's reference/, or the directory that the
environment variable ROS_REFERENCE_OUTPUTS names. Each number of both reads
as the double it reads back to. A path timing output echoes each path as the
reference read it, and pairs only with that path.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
HARNESS = REPOSITORY / "ros_reference"
_SCENARIOS = HARNESS / "scenarios"
# The kinds of scenario that the Rust crate ros_reference reads, each by a module: the
# variants of its `Kind`, in snake case.
KINDS = frozenset(
    {"chain_collision", "forward_kinematics", "occupancy", "path_timing", "planning", "self_distance"}
)
OUTPUTS_VARIABLE = "ROS_REFERENCE_OUTPUTS"
_OUTPUT_DIGESTS = "SHA256SUMS"
_INPUT_DIGESTS = "INPUTS.SHA256SUMS"
_RUN = (
    "run ros_reference/run.sh, which writes the reference outputs and the digests of them "
    "and of their inputs"
)
_PATH_INPUT_FIELDS = frozenset(
    {"name", "waypoints", "max_deviation", "max_velocity", "max_acceleration", "fractions"}
)
_TIMED_FIELDS = frozenset({"length", "configs", "duration"})


def _double(text: str) -> float:
    """The double that the JSON number `text` reads back to. Refuses a number
    beyond the largest finite double."""
    number = float(text)
    if not math.isfinite(number):
        raise ValueError(f"{text} lies beyond the largest finite double")
    return number


def _refuse_constant(text: str) -> float:
    raise ValueError(f"{text} is not a JSON number")


def read_json(text: bytes | str):
    """The JSON document `text` with each number read as the double it reads
    back to, as the Rust crate ros_reference reads a scenario and a reference
    output: an integer and -0 among them. Refuses NaN, an infinity and a number
    beyond the largest finite double."""
    return json.loads(text, parse_int=_double, parse_float=_double, parse_constant=_refuse_constant)


def digests(text: str) -> dict[str, str]:
    """The digest of each file, by name, of the lines that sha256sum writes:
    64 lowercase hexadecimal digits, two spaces and a file name, one file a
    line."""
    by_file: dict[str, str] = {}
    for line in text.splitlines():
        digest, separator, file = line.partition("  ")
        if not separator or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(
                f"{line!r} is not 64 lowercase hexadecimal digits, two spaces and a file name"
            )
        if file in by_file:
            raise ValueError(f"{file} has two digests")
        by_file[file] = digest
    return by_file


def reference_directory() -> Path:
    """The directory that ROS_REFERENCE_OUTPUTS names, an empty value among
    them, or reference/ where it is unset."""
    named = os.environ.get(OUTPUTS_VARIABLE)
    return HARNESS / "reference" if named is None else Path(named)


def _checked(content: bytes, file: str, digest_file: Path) -> bytes:
    """`content`, the bytes of `file`, after checking them against the digest
    that `digest_file` gives it."""
    expected = digests(digest_file.read_text()).get(file)
    if expected is None:
        raise ValueError(f"{digest_file} holds no digest of {file}; {_RUN}")
    if hashlib.sha256(content).hexdigest() != expected:
        raise ValueError(f"{file} differs from the file whose digest {digest_file} holds; {_RUN}")
    return content


def read_input(path: Path) -> bytes:
    """The bytes of the file of the repository at `path`, checked against
    INPUTS.SHA256SUMS by its path from the repository root."""
    return _checked(
        path.read_bytes(),
        path.relative_to(REPOSITORY).as_posix(),
        reference_directory() / _INPUT_DIGESTS,
    )


def _read_reference(name: str) -> dict:
    """The reference output `name`, checked against SHA256SUMS."""
    directory = reference_directory()
    file = f"{name}.json"
    return read_json(_checked((directory / file).read_bytes(), file, directory / _OUTPUT_DIGESTS))


def scenario_names(directory: Path, kind: str) -> list[str]:
    """The names of the scenarios of `kind` in `directory`, their file names
    without .json, in order. Refuses a scenario of a kind that ros_reference
    reads no module for."""

    def kind_of(name: str) -> str:
        named = json.loads((directory / f"{name}.json").read_bytes())["kind"]
        if named not in KINDS:
            raise ValueError(f"{name}: ros_reference reads no scenario of the kind {named}")
        return named

    names = sorted(path.stem for path in directory.glob("*.json"))
    return [name for name in names if kind_of(name) == kind]


def _require_fields(document: dict, fields: frozenset[str], context: str) -> None:
    """Refuse `document` unless it holds exactly `fields`."""
    if set(document) != fields:
        raise ValueError(f"{context} holds {sorted(document)}, and its fields are {sorted(fields)}")


@dataclass(frozen=True)
class Refused:
    """TOTG refused the path: it turns back on itself at a waypoint."""


@dataclass(frozen=True)
class Timed:
    """The blended path's length, its point at each fraction of the length,
    and the time-optimal duration under the limits."""

    length: float
    configs: list[list[float]]
    duration_s: float


@dataclass(frozen=True)
class PathCase:
    """One path of a scenario and what TOTG made of it."""

    name: str
    waypoints: list[list[float]]
    max_deviation: float
    max_velocity: list[float]
    max_acceleration: list[float]
    fractions: list[float]
    totg: Refused | Timed


@dataclass(frozen=True)
class PathTimingScenario:
    """A path timing scenario, by its file name under scenarios/, paired with
    its reference output."""

    name: str
    paths: list[PathCase]


def pair_path(scenario: str, given: dict, output: dict) -> PathCase:
    """The path `given` of the scenario `scenario` paired with the output
    `output`, which read it, and holds TOTG's refusal of it, or its length,
    one point per fraction and its duration."""
    context = f"{scenario} {given.get('name')}"
    _require_fields(given, _PATH_INPUT_FIELDS, context)
    if output.get("name") != given["name"]:
        raise ValueError(f"{context}: the paths are in one order")
    read_back = {field: output.get(field) for field in _PATH_INPUT_FIELDS}
    if read_back != given:
        raise ValueError(f"{context}: the reference read the path")
    outcome = {field: output[field] for field in set(output) - _PATH_INPUT_FIELDS}
    if outcome == {"refused": True}:
        totg: Refused | Timed = Refused()
    else:
        _require_fields(outcome, _TIMED_FIELDS, f"{context}: an output of a path TOTG times")
        if len(outcome["configs"]) != len(given["fractions"]):
            raise ValueError(f"{context}: one point per fraction")
        totg = Timed(
            length=outcome["length"],
            configs=outcome["configs"],
            duration_s=outcome["duration"],
        )
    return PathCase(**given, totg=totg)


def _path_timing_scenario(name: str) -> PathTimingScenario:
    scenario = read_json(read_input(_SCENARIOS / f"{name}.json"))
    reference = _read_reference(name)
    for document, of in ((scenario, "scenario"), (reference, "reference output")):
        _require_fields(document, frozenset({"kind", "paths"}), f"{name}: the {of}")
        if document["kind"] != "path_timing":
            raise ValueError(f"{name}: the {of} is of the kind {document['kind']}")
    if len(scenario["paths"]) != len(reference["paths"]):
        raise ValueError(f"{name}: one output per path")
    return PathTimingScenario(
        name=name,
        paths=[
            pair_path(name, given, output)
            for given, output in zip(scenario["paths"], reference["paths"], strict=True)
        ],
    )


def path_timing_scenarios() -> list[PathTimingScenario]:
    """Every path timing scenario under scenarios/, in the order of their
    names, each paired with its reference output."""
    return [_path_timing_scenario(name) for name in scenario_names(_SCENARIOS, "path_timing")]

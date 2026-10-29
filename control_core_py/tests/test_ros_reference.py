"""The reading of the ROS reference harness in ros_reference: a number, the
lines of a digest file, a file checked against its digest, the directory that
ROS_REFERENCE_OUTPUTS names, the pairing of a path timing output with the
path it read, and the kinds of scenario, those of the Rust crate."""

import decimal
import json
import math
import re
import shutil
import struct
from pathlib import Path

import pytest
import ros_reference
from hypothesis import assume, given
from hypothesis import strategies as st

DIGEST = "0123456789abcdef" * 4

PATH = {
    "name": "a",
    "waypoints": [[0.0], [1.0]],
    "max_deviation": 0.1,
    "max_velocity": [1.0],
    "max_acceleration": [1.0],
    "fractions": [0.0],
}

# The fields of an output that read PATH.
READ = dict(PATH)

REFUSED = {"refused": True}

# The doubles at the edges of the format, by their bits, and the decimal that
# std::to_chars writes for each, as check_shortest_decimals of the reference
# program holds them: the smallest and the largest subnormal, the smallest
# normal, the largest finite value and its negation, and both zeros.
EDGE_DECIMALS = [
    (0x0000_0000_0000_0001, "5e-324"),
    (0x000F_FFFF_FFFF_FFFF, "2.225073858507201e-308"),
    (0x0010_0000_0000_0000, "2.2250738585072014e-308"),
    (0x7FEF_FFFF_FFFF_FFFF, "1.7976931348623157e+308"),
    (0xFFEF_FFFF_FFFF_FFFF, "-1.7976931348623157e+308"),
    (0x0000_0000_0000_0000, "0"),
    (0x8000_0000_0000_0000, "-0"),
]


def timed(configs: list) -> dict:
    return {"length": 1.0, "configs": configs, "duration": 2.0}


def bits_of(x: float) -> int:
    return struct.unpack(">Q", struct.pack(">d", x))[0]


def double_of(pattern: int) -> float:
    return struct.unpack(">d", struct.pack(">Q", pattern))[0]


def shortest_decimals(x: float) -> list[str]:
    """The two decimals of `x` that std::to_chars writes the shorter of: its
    shortest digits with a point where they need one, and in scientific
    notation, with a sign and at least two digits of exponent."""
    shortest = decimal.Decimal(repr(x)).normalize()
    digits, exponent = f"{shortest:e}".split("e")
    return [f"{shortest:f}", f"{digits}e{int(exponent):+03d}"]


@given(st.integers(min_value=0, max_value=2**64 - 1))
def test_every_finite_double_reads_back_from_its_shortest_decimals(pattern):
    x = double_of(pattern)
    assume(math.isfinite(x))
    for text in shortest_decimals(x):
        assert bits_of(ros_reference.read_json(text)) == pattern, text


@pytest.mark.parametrize(("pattern", "text"), EDGE_DECIMALS)
def test_the_decimal_of_each_double_at_the_edges_reads_back_to_its_bits(pattern, text):
    assert text in shortest_decimals(double_of(pattern))
    assert bits_of(ros_reference.read_json(text)) == pattern


@pytest.mark.parametrize("text", ["NaN", "Infinity", "-Infinity", "1e309", "-1e309"])
def test_a_number_that_is_not_a_finite_double_is_refused(text):
    with pytest.raises(ValueError, match=text):
        ros_reference.read_json(text)


def test_the_lines_of_sha256sum_read_as_digests():
    by_file = ros_reference.digests(f"{DIGEST}  a.json\n{DIGEST}  b.json\n")
    assert by_file == {"a.json": DIGEST, "b.json": DIGEST}


@pytest.mark.parametrize(
    ("text", "refusal"),
    [
        (f"{DIGEST} a.json", "two spaces"),
        (f"{DIGEST.upper()}  a.json", "lowercase"),
        (f"{DIGEST[1:]}  a.json", "64"),
        (f"{DIGEST}  a.json\n{DIGEST}  a.json", "two digests"),
    ],
)
def test_a_line_that_is_not_a_digest_and_a_file_name_is_refused(text, refusal):
    with pytest.raises(ValueError, match=refusal):
        ros_reference.digests(text)


def write_scenarios(directory, kinds: dict) -> None:
    for name, kind in kinds.items():
        (directory / f"{name}.json").write_text(json.dumps({"kind": kind}))


# The kinds that the reader names are the variants of the Rust crate's Kind.
def test_the_kinds_are_those_of_the_rust_crate():
    source = (ros_reference.HARNESS / "src" / "lib.rs").read_text()
    body = re.search(r"\benum Kind \{(?P<variants>[^}]*)\}", source)
    assert body is not None, "ros_reference/src/lib.rs declares enum Kind"
    variants = re.findall(r"^\s*([A-Z]\w*),?\s*$", body["variants"], re.MULTILINE)
    snake = {re.sub(r"(?<!^)(?=[A-Z])", "_", variant).lower() for variant in variants}
    assert snake == ros_reference.KINDS


# The names sort as names, without .json: x before x-left, where x-left.json sorts
# before x.json.
def test_scenario_names_sort_apart_from_their_file_names(tmp_path):
    write_scenarios(tmp_path, {"x": "path_timing", "x-left": "path_timing", "y": "self_distance"})
    assert ros_reference.scenario_names(tmp_path, "path_timing") == ["x", "x-left"]


def test_a_scenario_of_a_kind_that_no_module_reads_is_refused(tmp_path):
    write_scenarios(tmp_path, {"x": "path_timing", "y": "teleport"})
    with pytest.raises(ValueError, match="no scenario of the kind teleport"):
        ros_reference.scenario_names(tmp_path, "path_timing")


def test_an_empty_ros_reference_outputs_names_the_empty_path(monkeypatch):
    monkeypatch.setenv(ros_reference.OUTPUTS_VARIABLE, "")
    assert ros_reference.reference_directory() == Path("")


def test_an_unset_ros_reference_outputs_names_reference(monkeypatch):
    monkeypatch.delenv(ros_reference.OUTPUTS_VARIABLE, raising=False)
    assert ros_reference.reference_directory() == ros_reference.HARNESS / "reference"


def test_a_file_that_no_reference_read_is_refused_as_an_input():
    with pytest.raises(ValueError, match="holds no digest of README.md"):
        ros_reference.read_input(ros_reference.REPOSITORY / "README.md")


@pytest.fixture
def outputs(tmp_path, monkeypatch):
    """A copy of reference/, which ROS_REFERENCE_OUTPUTS names."""
    copy = tmp_path / "outputs"
    shutil.copytree(ros_reference.HARNESS / "reference", copy)
    monkeypatch.setenv(ros_reference.OUTPUTS_VARIABLE, str(copy))
    return copy


def test_the_outputs_that_ros_reference_outputs_names_are_read(outputs):
    assert ros_reference.reference_directory() == outputs
    assert ros_reference.path_timing_scenarios()


def test_an_output_with_one_digit_changed_fails_its_digest(outputs):
    for output in outputs.glob("path_timing_*.json"):
        text = output.read_text()
        at = next(index for index, character in enumerate(text) if character.isdigit())
        output.write_text(text[:at] + ("1" if text[at] == "0" else "0") + text[at + 1 :])
    with pytest.raises(ValueError, match="differs from the file whose digest"):
        ros_reference.path_timing_scenarios()


def test_a_refusal_and_a_timed_path_pair():
    assert ros_reference.pair_path("scenario", PATH, READ | REFUSED).totg == ros_reference.Refused()
    case = ros_reference.pair_path("scenario", PATH, READ | timed([[0.0]]))
    assert case.totg == ros_reference.Timed(length=1.0, configs=[[0.0]], duration_s=2.0)


def test_an_output_that_read_another_path_does_not_pair():
    with pytest.raises(ValueError, match="the reference read the path"):
        ros_reference.pair_path("scenario", PATH | {"max_deviation": 0.2}, READ | REFUSED)


def test_an_output_missing_a_point_does_not_pair():
    with pytest.raises(ValueError, match="one point per fraction"):
        ros_reference.pair_path("scenario", PATH, READ | timed([]))


def test_an_output_neither_refused_nor_timed_does_not_pair():
    with pytest.raises(ValueError, match="an output of a path TOTG times"):
        ros_reference.pair_path("scenario", PATH, READ | {"refused": False})


def test_every_path_timing_scenario_pairs_with_its_reference():
    scenarios = ros_reference.path_timing_scenarios()
    assert scenarios, "scenarios/ holds a path timing scenario"
    for scenario in scenarios:
        assert scenario.paths, f"{scenario.name} holds a path"

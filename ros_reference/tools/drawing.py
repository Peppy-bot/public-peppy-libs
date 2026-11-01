"""What the scripts that write the scenarios of the ROS reference harness share: the
paths, the generator of each scenario's draws, the --drawn option, a parsed URDF, the
joints of an arm of OpenArm v2, the joint limits of a URDF, the configurations of a
chain, and the writing of a scenario.

Each scenario draws from a generator seeded with its own name, so every run writes the
same scenarios, and a change to one scenario leaves the draws of the others as they are.

A scenario is written with a value on one line when the value is a scalar, an array of
scalars, or a record, an object whose members are each a scalar or an array of scalars.
Every other array or object spreads over lines, one member a line, indented by two
spaces. The write_json of the reference program lays out the reference outputs by the
same rule.
"""

import argparse
import json
import random
import xml.etree.ElementTree as ElementTree
from pathlib import Path

SEED = 20261028
REPOSITORY = Path(__file__).resolve().parents[2]
SCENARIOS = REPOSITORY / "ros_reference" / "scenarios"
# What --drawn counts in a script that draws the configurations of a chain.
CONFIGURATIONS_DRAWN = "configurations drawn inside the limits of each chain"


def generator(name):
    return random.Random(f"{SEED}:{name}")


def options(doc, drawn, what):
    """The options of a script whose docstring is `doc`: --drawn, how many of `what` it
    draws, `drawn` by default. A larger count draws a large set for a local run, which
    the README describes."""
    parser = argparse.ArgumentParser(description=doc.splitlines()[0])
    parser.add_argument("--drawn", type=int, default=drawn, help=what)
    return parser


def robot(urdf):
    """The parsed URDF at `urdf`, relative to the repository root."""
    return ElementTree.parse(REPOSITORY / urdf).getroot()


def openarm_joints(side):
    return [f"openarm_{side}_joint{i}" for i in range(1, 8)]


def joint_limits(robot, joint):
    element = robot.find(f"joint[@name='{joint}']/limit")
    if element is None:
        raise SystemExit(f"the URDF gives no limits for {joint}")
    # A URDF limit without a lower or an upper bound bounds it at 0.
    return float(element.get("lower", 0)), float(element.get("upper", 0))


def configurations(rng, limits, drawn):
    """The zero configuration, the lower limits, the upper limits, and `drawn`
    configurations drawn uniformly inside the limits."""
    lower = [low for low, _ in limits]
    upper = [high for _, high in limits]
    inside = [[rng.uniform(low, high) for low, high in limits] for _ in range(drawn)]
    return [[0.0] * len(limits), lower, upper, *inside]


def _is_flat(value):
    if isinstance(value, list):
        return not any(isinstance(member, (dict, list)) for member in value)
    return not isinstance(value, dict)


def _is_one_line(value):
    if isinstance(value, dict):
        return all(_is_flat(member) for member in value.values())
    return _is_flat(value)


def _layout(value, indent):
    """The text of `value`, by the rule of the module's docstring, with its later lines
    indented by `indent`."""
    if _is_one_line(value):
        return json.dumps(value, separators=(",", ":"))
    inner = indent + "  "
    if isinstance(value, dict):
        members = [
            f"{inner}{json.dumps(key)}: {_layout(member, inner)}" for key, member in value.items()
        ]
        return "{\n" + ",\n".join(members) + "\n" + indent + "}"
    members = [f"{inner}{_layout(member, inner)}" for member in value]
    return "[\n" + ",\n".join(members) + "\n" + indent + "]"


def write(name, scenario):
    path = SCENARIOS / f"{name}.json"
    path.write_text(_layout(scenario, "") + "\n")
    print(f"wrote {path.relative_to(REPOSITORY)}")

"""The limbs a brain drives: each gripper with the arm that carries it, as
the `limbs` parameter names them, "left_arm:left_gripper,right_arm:right_gripper".

The pairs are parsed once, at the node's start, so a gripper and its arm
can never fall out of step: a backend drives `gripper.arm` and reads no
rule into the names. The order of the pairs is the order get_state
reports the grippers in.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Limb:
    """One arm and the gripper it carries, as limb_motion names them."""

    arm: str
    gripper: str


def parse_limbs(text: str) -> tuple[Limb, ...]:
    """The limbs of a `limbs` value: comma-separated `arm:gripper` pairs,
    spaces around each name trimmed. A ValueError names what is wrong: no
    pair, a pair that is not one arm and one gripper, an empty name, or an
    arm or a gripper named twice."""
    if not text.strip():
        raise ValueError("limbs must name at least one arm:gripper pair")
    limbs: list[Limb] = []
    for pair in text.split(","):
        sides = pair.split(":")
        if len(sides) != 2:
            raise ValueError(f"limbs: '{pair.strip()}' is not one arm:gripper pair")
        arm, gripper = (side.strip() for side in sides)
        if not arm or not gripper:
            raise ValueError(f"limbs: '{pair.strip()}' needs an arm name and a gripper name")
        limbs.append(Limb(arm=arm, gripper=gripper))
    _refuse_repeats("arm", [limb.arm for limb in limbs])
    _refuse_repeats("gripper", [limb.gripper for limb in limbs])
    return tuple(limbs)


def _refuse_repeats(kind: str, names: list[str]) -> None:
    seen: set[str] = set()
    for name in names:
        if name in seen:
            raise ValueError(f"limbs: the {kind} '{name}' is named twice")
        seen.add(name)

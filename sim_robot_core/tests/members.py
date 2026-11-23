"""The members of an engine's slots, as a node's generated `peers()` gives
them: the peer at the other end of each pair and the copy it runs as."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from sim_robot_core.pairs import SLOTS, PairTable


@dataclass(frozen=True)
class Producer:
    core_node: str
    instance_id: str


@dataclass(frozen=True)
class Info:
    producer: Producer
    peer_link_id: str


@dataclass(frozen=True)
class Member:
    info: Info
    copy: Optional[str]


def member(
    instance: str, link: str, copy: Optional[str] = "alpha", core_node: str = "sim16"
) -> Member:
    return Member(info=Info(producer=Producer(core_node, instance), peer_link_id=link), copy=copy)


def table(sole: Optional[str] = None, **members) -> PairTable:
    """The table of the slots holding `members`, by slot, read again on each
    call of a slot, so a test changes a slot's members by changing the list
    it passed."""
    held = {slot: members.get(slot, []) for slot in SLOTS}
    return PairTable({slot: (lambda slot=slot: tuple(held[slot])) for slot in SLOTS}, lambda: sole)

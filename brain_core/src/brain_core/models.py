"""The model of each role, and the one way to change one while the brain
runs.

A goal uses only its own role's model; the lane rule then guarantees a
running goal never sees its model change. scan_items and identify_item
read the perception model, grab_item, drop_item and place_item the
manipulation model, each once, at the start of its body. A switch for a
role is refused while that role's lane runs a goal, and the new model is
set with no await between the last lane check and the set, so the next
goal of the role is the first to use it.

A switch, `Models.switch`, runs as a lane-less job and goes:

1. the source's entry for the name, or its refusal (`ModelSource.offered`);
2. refused while another switch for the role runs;
3. refused while the role's lane runs a goal, naming it, before any work;
4. the role marked as switching, and the backend prepared
   (`ModelSource.prepare`, awaited: a hosted model is reached once), cut
   short by the switch's own cancel;
5. a cancel of the switch, by its caller or by a shutdown, leaves the
   role's model as it was;
6. the lane checked again, then the backend set, with no await between.

The switching mark is released whatever the outcome.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

from .ports import ModelSource, Refusal, Role
from .sequencer import Job, Sequencer
from .waiting import until


@dataclass(frozen=True)
class Active:
    """The model a role runs: its name and its backend."""

    name: str
    backend: object


class Models:
    def __init__(self, source: ModelSource, sequencer: Sequencer, robot, limb_states) -> None:
        self._source = source
        self._sequencer = sequencer
        # The robot a manipulator moves and the store of its measured
        # limbs, handed to each manipulator before it runs.
        self._robot = robot
        self._limb_states = limb_states
        self._active: dict[Role, Active] = {}
        self._switching: set[Role] = set()

    async def start(self, names: Mapping[Role, str]) -> None:
        """Sets the model of each role at the node's start, `names` the
        parameters that name them. A ValueError stops the node: a source
        without every role, since the brain serves the goals of both, or a
        model a role does not offer."""
        missing = [role.value for role in Role if role not in self._source.roles()]
        if missing:
            raise ValueError(f"the model source has no role {', '.join(missing)}: a brain runs a model for every role, 'none' included")
        for role in self._source.roles():
            name = names[role]
            try:
                entry = self._source.offered(role, name)
            except Refusal as refusal:
                raise ValueError(f"{role}_model: {refusal.message}") from None
            backend = self._source.start(entry)
            await self._ready(role, backend)
            self._active[role] = Active(name, backend)

    def roles(self) -> Sequence[Role]:
        return self._source.roles()

    def names(self, role: Role) -> Sequence[str]:
        return self._source.names(role)

    def current(self, role: Role):
        """The backend `role` runs now."""
        return self._active[role].backend

    def current_name(self, role: Role) -> str:
        return self._active[role].name

    async def switch(self, role: Role, name: str, job: Job) -> str:
        """Makes `name` the model of `role` for the switch `job` runs, and
        returns the name; a Refusal, an ExpectedFailure or a Cancelled
        leaves the role's model as it was."""
        entry = self._source.offered(role, name)
        if role in self._switching:
            raise Refusal(f"switch_model is running for {role}")
        self._refuse_while_busy(role)
        self._switching.add(role)
        try:
            backend = await until(job.cancel.wait(), self._source.prepare(entry))
            job.cancel.check()
            await self._ready(role, backend)
            job.cancel.check()
            self._refuse_while_busy(role)
            self._active[role] = Active(name, backend)
            return name
        finally:
            self._switching.discard(role)

    def _refuse_while_busy(self, role: Role) -> None:
        running = self._sequencer.running(role)
        if running is not None:
            raise Refusal(f"{running.action} is running")

    async def _ready(self, role: Role, backend) -> None:
        if role == Role.MANIPULATION:
            await backend.start(self._robot, self._limb_states)

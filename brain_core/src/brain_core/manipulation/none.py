"""The manipulator that is not there: `manipulation_model: "none"`.

It moves nothing. Every sequence is refused with "no manipulation model:
manipulation_model is 'none'", after the gripper and item rules have had
their say, so a caller still learns "gripper already holds an item" before
that the brain has no model. abort and get_state work as on any brain.
"""

from __future__ import annotations

from ..ports import CancelToken, Gripper, Item, Outcome, Pose


class NoneManipulator:
    name = "none"

    def why_unavailable(self) -> str:
        return "no manipulation model: manipulation_model is 'none'"

    async def start(self, robot, limb_states) -> None:
        return None

    async def grab(self, item: Item, gripper: Gripper, max_effort: float, cancel: CancelToken) -> Outcome:
        return Outcome(False, self.why_unavailable())

    async def drop(self, gripper: Gripper, cancel: CancelToken) -> Outcome:
        return Outcome(False, self.why_unavailable())

    async def place(self, gripper: Gripper, pose: Pose, cancel: CancelToken) -> Outcome:
        return Outcome(False, self.why_unavailable())

    async def stop(self) -> None:
        return None

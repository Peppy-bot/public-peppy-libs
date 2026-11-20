"""identify_item: the one item best matching a description, with an id
the manipulation actions accept and its region in the picture the search
looked at, its point, and its pose where the detector resolved how it is
turned, each in the robot frame at the capture time of that picture. A
search that finds no such item fails with `not_found`."""

from __future__ import annotations

from ..brain import STATUS_ENDS
from ..ports import Coverage, ExpectedFailure, FailureKind, Role
from ..state import best_match

ZERO = dict(
    item_id="", label="", item_point=None, item_pose=None, confidence=0.0, region=None, camera="", image_width=0, image_height=0,
)


async def admit(brain, goal):
    """Refuses a search that cannot start now (`Brain.check_search`)."""
    brain.check_search()
    return goal


async def run(brain, ctx, goal) -> None:
    description = goal.description.strip()
    module = brain.bindings.identify_item

    async def body(job) -> dict:
        detector = brain.models.current(Role.PERCEPTION)
        phrases = [description] if description else []
        look = await brain.look(job, detector, phrases, goal.timeout_s)
        best = best_match(look.detections, description)
        if best is None:
            if description:
                raise ExpectedFailure(FailureKind.NOT_FOUND, f"no item matches '{description}'")
            raise ExpectedFailure(FailureKind.NOT_FOUND, "no item in view")
        # A search covers nothing: it refreshes what it found, renames
        # nothing and drops nothing, so the other items keep their ids.
        item = brain.state.remember([best], brain.now(), coverage=Coverage())[0]
        job.notes.append(f"item {item.item_id}")
        item_pose = None
        if item.orientation is not None:
            item_pose = module.ResultItemPose(
                frame=look.frame, timestamp=look.timestamp, position=list(item.position), orientation=list(item.orientation)
            )
        return dict(
            item_id=item.item_id,
            label=item.label,
            item_point=module.ResultItemPoint(frame=look.frame, timestamp=look.timestamp, position=list(item.position)),
            item_pose=item_pose,
            confidence=item.confidence,
            region=list(item.region) if item.region is not None else None,
            camera=brain.params.camera_name,
            image_width=look.image_width,
            image_height=look.image_height,
        )

    await brain.run_guarded(ctx, Role.PERCEPTION, "identify_item", body, ZERO, ends=STATUS_ENDS, asked=f"'{description}'")

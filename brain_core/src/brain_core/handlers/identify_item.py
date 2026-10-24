"""identify_item: the one item best matching a description, with an id
the manipulation actions accept and its region in the picture the search
looked at. A search that finds no such item fails with `not_found`."""

from __future__ import annotations

from ..brain import STATUS_ENDS
from ..ports import Coverage, ExpectedFailure, FailureKind, Role
from ..state import best_match

ZERO = dict(
    item_id="", label="", position=[0.0, 0.0, 0.0], orientation=None, confidence=0.0, region=None, camera="",
    image_width=0, image_height=0, frame_timestamp=0.0,
)


def decide(brain, goal) -> None:
    """Refuses a search that cannot start now (`Brain.check_search`)."""
    brain.check_search()


async def run(brain, ctx) -> None:
    goal = ctx.request().data
    description = goal.description.strip()

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
        return dict(
            item_id=item.item_id,
            label=item.label,
            position=list(item.position),
            orientation=list(item.orientation) if item.orientation is not None else None,
            confidence=item.confidence,
            region=list(item.region) if item.region is not None else None,
            camera=brain.params.camera_name,
            image_width=look.image_width,
            image_height=look.image_height,
            frame_timestamp=look.frame_timestamp,
        )

    await brain.run_guarded(ctx, Role.PERCEPTION, "identify_item", body, ZERO, ends=STATUS_ENDS, asked=f"'{description}'")

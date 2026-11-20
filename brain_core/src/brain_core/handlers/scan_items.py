"""scan_items: every item in view, each with an id and its region in the
picture the scan looked at, in one result. Every point is in the robot
frame at the capture time of that picture, an item the scan did not see
and reports from an earlier look included. A cancelled scan gives empty
arrays in the robot frame at the time of the cancel."""

from __future__ import annotations

from ..brain import STATUS_ENDS
from ..ports import Role

ZERO = dict(item_ids=[], labels=[], points=[], confidences=[], regions=[], camera="", image_width=0, image_height=0)

# The region of an item the scan reports without having seen it.
NO_REGION = (0.0, 0.0, 0.0, 0.0)


async def admit(brain, goal):
    """Refuses a scan that cannot start now (`Brain.check_search`)."""
    brain.check_search()
    return goal


async def run(brain, ctx, goal) -> None:
    async def body(job) -> dict:
        # One detector for the whole scan: the look and the coverage that
        # says which unseen items it drops.
        detector = brain.models.current(Role.PERCEPTION)
        look = await brain.look(job, detector, [], goal.timeout_s)
        items = brain.state.remember(look.detections, brain.now(), coverage=detector.scan_coverage())
        module = brain.bindings.scan_items
        return dict(
            item_ids=[item.item_id for item in items],
            labels=[item.label for item in items],
            frame=look.frame,
            timestamp=look.timestamp,
            points=[module.ResultPointsItem(position=list(item.position)) for item in items],
            confidences=[item.confidence for item in items],
            regions=[corner for item in items for corner in (item.region or NO_REGION)],
            camera=brain.params.camera_name,
            image_width=look.image_width,
            image_height=look.image_height,
        )

    await brain.run_guarded(ctx, Role.PERCEPTION, "scan_items", body, ZERO, ends=STATUS_ENDS, stamped=True)

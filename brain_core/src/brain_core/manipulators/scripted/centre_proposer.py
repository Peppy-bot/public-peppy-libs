"""The grasp the scripted manipulator asks for: no model, the item's own
position.

A look reports the point the camera saw on an item, the depth under its
box's centre, so the item's middle is below it: the grasp point is that
point lowered `grasp_depth_m`, reached from above at each roll. Under the
centre of the box of an open item, a mug or a bowl seen from above, the
camera sees the inside bottom, so a grasp under it closes below the item's
walls, in what the item stands on. When the item's top, as the look
measured it (`Item.top_z`), stands `open_item_depth_m` or more above the
seen point, and no more than `open_item_max_depth_m`, the grasp point is
lowered from the point over the seen point at the height of the top: the
jaws then close on the walls. A top that stands higher than that is not
the item's: a loose box takes in a taller thing behind the item, or a
gripper over it, and the grasp point is lowered from the seen point. An
item whose position is already a grasp point
(`Item.position_is_grasp_point`: a grab_item goal by pose, or an item
place_item put down) is grasped there unlowered, in its orientation alone
when it has one, else from above at each roll.
"""

from __future__ import annotations

from ...ports import Item, Vec3
from .grasp import Grasp, GraspDirection, candidates, lowered, orientations_for

# How far under the seen point the grasp point is: in Waldo the chest
# camera's seen point stands 2.2 to 2.6 cm above the centre of a lemon and
# of a 5 cm cube.
GRASP_DEPTH_M = 0.025
# How far under the item's top a seen point stands when the camera sees
# into an open item: more than the top of a closed item stands above the
# seen point on it, and less than the walls of a mug or a bowl are high.
OPEN_ITEM_DEPTH_M = 0.03
# How far under the item's top a seen point stands, at most, when the
# camera sees into an open item: deeper than the inside of a mug, a cup or
# a bowl. A top that stands higher above the seen point is that of a taller
# thing that the item's box takes in.
OPEN_ITEM_MAX_DEPTH_M = 0.12


class CentreProposer:
    def __init__(
        self,
        grasp_depth_m: float = GRASP_DEPTH_M,
        open_item_depth_m: float = OPEN_ITEM_DEPTH_M,
        open_item_max_depth_m: float = OPEN_ITEM_MAX_DEPTH_M,
    ) -> None:
        if not 0.0 < open_item_depth_m <= open_item_max_depth_m:
            raise ValueError(
                f"an open item is from {open_item_depth_m} m to {open_item_max_depth_m} m deep: "
                "the depths must be above zero and in that order"
            )
        self.grasp_depth_m = grasp_depth_m
        self.open_item_depth_m = open_item_depth_m
        self.open_item_max_depth_m = open_item_max_depth_m

    def propose(self, item: Item) -> Grasp:
        """The grasp of `item`; a ValueError names an orientation that is
        not a rotation."""
        if item.position_is_grasp_point:
            return Grasp(item.position, orientations_for(item.orientation))
        return Grasp(lowered(self.lowered_from(item), self.grasp_depth_m), candidates(GraspDirection.DOWN))

    def lowered_from(self, item: Item) -> Vec3:
        """The point the grasp point of `item`, an item a look saw, stands
        under: its seen point, or the point over it at the height of its
        top when that top stands from `open_item_depth_m` to
        `open_item_max_depth_m` higher."""
        x, y, z = item.position
        if item.top_z is None:
            return item.position
        if not self.open_item_depth_m <= item.top_z - z <= self.open_item_max_depth_m:
            return item.position
        return (x, y, item.top_z)

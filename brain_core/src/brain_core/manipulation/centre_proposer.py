"""The grasp the scripted manipulator asks for: no model, the item's own
position.

A look reports the point the camera saw on an item, the depth under its
box's centre, so the item's middle is below it: the grasp point is that
point lowered `grasp_depth_m`, reached from above at each roll. An item
whose position is already a grasp point (`Item.position_is_grasp_point`:
a grab_item goal by pose, or an item place_item put down) is grasped
there unlowered, in its orientation alone when it has one, else from above
at each roll.
"""

from __future__ import annotations

from ..ports import Grasp, Item
from .grasp import GraspDirection, candidates, lowered, orientations_for

# How far under the seen point the grasp point is: in Waldo the chest
# camera's seen point stands 2.2 to 2.6 cm above the centre of a lemon and
# of a 5 cm cube.
GRASP_DEPTH_M = 0.025


class CentreProposer:
    def __init__(self, grasp_depth_m: float = GRASP_DEPTH_M) -> None:
        self.grasp_depth_m = grasp_depth_m

    def propose(self, item: Item) -> Grasp:
        """The grasp of `item`; a ValueError names an orientation that is
        not a rotation."""
        if item.position_is_grasp_point:
            return Grasp(item.position, orientations_for(item.orientation))
        return Grasp(lowered(item.position, self.grasp_depth_m), candidates(GraspDirection.DOWN))

"""The generated modules a brain node passes in.

peppygen is generated per node from the node's own manifest, so a shared
library cannot import it. A node builds one `Bindings` from its peppygen
package and hands it to `node.setup`; each part of the core takes the
modules it uses from there. Every brain implements the same contracts, so
every brain's peppygen has these modules under the same names.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Bindings:
    # peppygen.clock: the node's clock, which the brain stamps goals with.
    clock: Any
    # peppygen.QoSProfile.Standard: the QoS of a limb_motion goal's feedback.
    standard_qos: Any

    # The backbone, limb_motion:v1 (peppygen.consumed_actions.limb_motion).
    move_arm: Any
    move_gripper: Any

    # The camera, rgbd_camera:v1 (peppygen.consumed_topics.camera and
    # peppygen.consumed_services.camera).
    video_stream: Any
    depth_stream: Any
    depth_stream_info: Any

    # The camera's geometry, camera_geometry:v1
    # (peppygen.consumed_services.geometry).
    get_color_intrinsics: Any
    get_depth_intrinsics: Any

    # The robot's camera mounts, camera_mounts:v1
    # (peppygen.consumed_services.camera_mounts).
    get_camera_poses: Any

    # item_perception:v1 (peppygen.exposed_actions.item_perception).
    scan_items: Any
    identify_item: Any

    # item_manipulation:v1 (peppygen.exposed_actions.item_manipulation and
    # peppygen.exposed_services.item_manipulation).
    grab_item: Any
    drop_item: Any
    place_item: Any
    abort: Any
    get_state: Any

"""The generated modules a brain node passes in.

peppygen is generated per node from the node's own manifest, so a shared
library cannot import it. A node hands `Bindings.from_generated(peppygen)`
to `node.setup`; each part of the core takes the modules it uses from
there. Every brain implements the same contracts and names its slots
alike (limb_motion, camera, geometry, camera_mounts), so every brain's
peppygen has these modules under the same names.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass
from types import ModuleType
from typing import Any

# Where each module sits in a brain's generated package.
GENERATED_MODULES = {
    "clock": "clock",
    "move_arm": "consumed_actions.limb_motion.move_arm",
    "move_gripper": "consumed_actions.limb_motion.move_gripper",
    "video_stream": "consumed_topics.camera.video_stream",
    "depth_stream": "consumed_topics.camera.depth_stream",
    "depth_stream_info": "consumed_services.camera.depth_stream_info",
    "get_color_intrinsics": "consumed_services.geometry.get_color_intrinsics",
    "get_depth_intrinsics": "consumed_services.geometry.get_depth_intrinsics",
    "get_camera_poses": "consumed_services.camera_mounts.get_camera_poses",
    "scan_items": "exposed_actions.item_perception.scan_items",
    "identify_item": "exposed_actions.item_perception.identify_item",
    "grab_item": "exposed_actions.item_manipulation.grab_item",
    "drop_item": "exposed_actions.item_manipulation.drop_item",
    "place_item": "exposed_actions.item_manipulation.place_item",
    "abort": "exposed_actions.item_manipulation.abort",
    "get_state": "exposed_services.item_manipulation.get_state",
    "switch_model": "exposed_actions.brain_model.switch_model",
    "get_model": "exposed_services.brain_model.get_model",
    "list_models": "exposed_services.brain_model.list_models",
}


@dataclass(frozen=True)
class Bindings:
    # peppygen.clock: the node's clock, which the brain stamps goals with.
    clock: Any

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

    # brain_model:v1 (peppygen.exposed_actions.brain_model and
    # peppygen.exposed_services.brain_model).
    switch_model: Any
    get_model: Any
    list_models: Any

    @classmethod
    def from_generated(cls, package: ModuleType) -> "Bindings":
        """The bindings of a brain's generated package, its `peppygen`: each
        module imported from where `GENERATED_MODULES` says it sits. A
        module the package lacks raises ModuleNotFoundError naming it."""
        modules = {field: importlib.import_module(f"{package.__name__}.{path}") for field, path in GENERATED_MODULES.items()}
        return cls(**modules)

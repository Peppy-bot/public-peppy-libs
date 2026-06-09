from __future__ import annotations

import json
import time
from typing import Any

from sim_ext_core.base import BridgePlugin

_QOS = "sensor_data"


class JointStatesBridge(BridgePlugin):

    def __init__(self, articulation: Any, config: Any, entry: Any) -> None:
        self._articulation = articulation
        self._node_name: str = config.node_name
        self._robot_name: str = entry.robot_name
        self._topic: str = entry.topic
        self._joint_names: list[str] = []

    def setup(self) -> bool:
        if not self._articulation.setup():
            return False
        if hasattr(self._articulation, "get_joint_names"):
            self._joint_names = list(self._articulation.get_joint_names())
        else:
            self._joint_names = []
        return True

    def teardown(self) -> None:
        # Every other sensor bridge (Imu, EePose, Gripper, …) forwards teardown
        # to its impl; on Isaac the articulation view holds native handles that
        # must be released when the extension reloads.
        self._articulation.teardown()

    def on_step(self, step: int, io: Any) -> None:
        states = self._articulation.get_joint_states()
        if states is None:
            return
        positions, velocities = states
        payload = json.dumps(
            {
                "robot": self._robot_name,
                "step": step,
                "joint_names": self._joint_names,
                "positions": positions,
                "velocities": velocities,
                "stamp": time.time(),
            }
        ).encode()
        io.emit(self._node_name, self._topic, _QOS, payload)

    @property
    def is_ready(self) -> bool:
        return self._articulation.is_ready

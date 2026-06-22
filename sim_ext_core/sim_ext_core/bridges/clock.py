"""Sim clock publisher bridge."""

from __future__ import annotations

import json
import time
from typing import Any

from sim_ext_core.base import BridgePlugin

_QOS = "sensor_data"
_DEFAULT_TOPIC = "clock"


class ClockBridge(BridgePlugin):
    """Publishes the sim clock each step."""

    def __init__(self, sensor: Any, config: Any, entry: Any) -> None:
        if entry is None:
            raise ValueError("ClockBridge requires a non-None entry with a topic field")
        self._sensor = sensor
        self._node_name: str = config.node_name
        self._topic: str = entry.topic

    def setup(self) -> bool:
        return self._sensor.setup()

    def teardown(self) -> None:
        self._sensor.teardown()

    def on_step(self, step: int, io: Any) -> None:
        data = self._sensor.get_clock_data()
        if data is None:
            return
        payload = json.dumps(
            {
                "step": step,
                "sim_time": data["sim_time"],
                "stamp": time.time(),
            }
        ).encode()
        io.emit(self._node_name, self._topic, _QOS, payload)

    @property
    def is_ready(self) -> bool:
        return self._sensor.is_ready

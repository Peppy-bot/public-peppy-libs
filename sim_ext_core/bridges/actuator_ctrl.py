from __future__ import annotations

import json
import logging
from typing import Any

from sim_ext_core.base import BridgePlugin

logger = logging.getLogger(__name__)

_QOS = "standard"


class ActuatorCtrlBridge(BridgePlugin):
    """Subscribes to a raw set_ctrl topic and writes the latest actuator_values
    into the sim's ctrl[] array before the next physics step.

    Payload shape (raw JSON, peppylib Standard QoS):
        { "actuator_values": { "<actuator_name>": <float>, ... } }

    The bridge holds a duck-typed `actuator_ctrl` wrapper — anything that
    exposes setup(), teardown(), is_ready, and write_targets(dict). The MuJoCo
    implementation is `sim_ext_core.mujoco.MujocoActuatorCtrl`.
    """

    def __init__(self, actuator_ctrl: Any, config: Any, entry: Any) -> None:
        self._actuator_ctrl = actuator_ctrl
        self._source_node: str = entry.source_node
        self._topic: str = entry.topic

    def setup(self) -> bool:
        return self._actuator_ctrl.setup()

    def teardown(self) -> None:
        self._actuator_ctrl.teardown()

    def subscriptions(self) -> list[tuple[str, str, str]]:
        return [(self._source_node, self._topic, _QOS)]

    def on_step(self, step: int, io: Any) -> None:
        raw = io.get_latest(self._source_node, self._topic)
        if raw is None:
            return
        try:
            msg = json.loads(raw)
        except Exception as exc:
            logger.warning(f"set_ctrl: malformed JSON on {self._topic}: {exc}")
            return
        values = msg.get("actuator_values")
        if not isinstance(values, dict):
            logger.warning(
                f"set_ctrl: payload missing actuator_values dict on {self._topic}"
            )
            return
        self._actuator_ctrl.write_targets(values)

    @property
    def is_ready(self) -> bool:
        return self._actuator_ctrl.is_ready

from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoActuatorCtrl:
    """Resolves MJCF actuator names to ctrl indices and writes targets into data.ctrl[].

    Used by ActuatorCtrlBridge to translate raw set_ctrl messages
    ({actuator_values: {name: value, ...}}) into in-process MuJoCo ctrl writes
    before mj_step.
    """

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._name_to_id: dict[str, int] = {}
        self._ready: bool = False

    def setup(self) -> bool:
        """Build the actuator-name → ctrl-id map from the model."""
        try:
            import mujoco  # pylint: disable=E0401

            name_to_id: dict[str, int] = {}
            for i in range(self._model.nu):
                name = mujoco.mj_id2name(
                    self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i
                ) or ""
                if name:
                    name_to_id[name] = i
            self._name_to_id = name_to_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoActuatorCtrl: {exc}")
            return False

        logger.info(
            f"MujocoActuatorCtrl ready — {len(self._name_to_id)} actuator(s) resolved"
        )
        return True

    def teardown(self) -> None:
        self._ready = False
        self._name_to_id = {}

    def write_targets(self, actuator_values: dict) -> int:
        """Write each {name: value} pair into data.ctrl[ctrl_id[name]].

        Returns the count of values actually applied. Unknown actuator names
        are logged and dropped — they should not stop the rest of the batch.
        """
        if not self._ready:
            return 0
        applied = 0
        for name, value in actuator_values.items():
            ctrl_id = self._name_to_id.get(name)
            if ctrl_id is None:
                logger.warning(f"unknown actuator '{name}' — dropped")
                continue
            try:
                self._data.ctrl[ctrl_id] = float(value)
                applied += 1
            except Exception as exc:
                logger.warning(f"failed to write ctrl[{ctrl_id}] for '{name}': {exc}")
        return applied

    @property
    def is_ready(self) -> bool:
        return self._ready

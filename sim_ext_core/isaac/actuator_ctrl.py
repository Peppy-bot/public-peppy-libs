from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

_ARTICULATION_NAME = "peppy_actuator_ctrl"


class IsaacActuatorCtrl:
    """Resolves joint names to indices on a target articulation and writes
    position targets via ArticulationView.set_joint_position_targets().

    One instance per articulation (one gripper, one arm side, etc). The
    ActuatorCtrlBridge takes incoming {actuator_values: {name: value}} payloads
    and routes them through this writer.
    """

    def __init__(self, prim_path: str, joint_names: list[str]) -> None:
        self._prim_path = prim_path
        self._joint_names = list(joint_names)
        self._view = None
        self._name_to_idx: dict[str, int] = {}
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise the Articulation and resolve joint name → index."""
        if self._view is not None and self._ready:
            return True
        try:
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._view = Articulation(
                prim_paths_expr=self._prim_path,
                name=_ARTICULATION_NAME,
            )
            self._view.initialize()
            dof_names = list(self._view.dof_names)
            self._name_to_idx = {n: i for i, n in enumerate(dof_names) if n in self._joint_names}
            missing = [n for n in self._joint_names if n not in self._name_to_idx]
            if missing:
                logger.warning(
                    f"IsaacActuatorCtrl: joints not found on '{self._prim_path}': {missing}"
                )
            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to initialise IsaacActuatorCtrl at '{self._prim_path}': {exc}"
            )
            self._view = None
            return False

        logger.info(
            f"IsaacActuatorCtrl ready — prim='{self._prim_path}'"
            f" resolved={list(self._name_to_idx.keys())}"
        )
        return True

    def teardown(self) -> None:
        self._view = None
        self._ready = False

    def write_targets(self, actuator_values: dict) -> int:
        """Write each {name: value} pair into the articulation's joint
        position targets. Unknown names are dropped with a warning.
        """
        if not self._ready or self._view is None:
            return 0
        try:
            import numpy as np  # pylint: disable=E0401

            indices: list[int] = []
            values: list[float] = []
            for name, value in actuator_values.items():
                idx = self._name_to_idx.get(name)
                if idx is None:
                    logger.warning(f"unknown actuator '{name}' on '{self._prim_path}' — dropped")
                    continue
                indices.append(idx)
                values.append(float(value))
            if not indices:
                return 0
            self._view.set_joint_position_targets(
                np.array([values], dtype=np.float32),
                joint_indices=np.array(indices),
            )
            return len(indices)
        except Exception as exc:
            logger.warning(f"Failed to write targets on '{self._prim_path}': {exc}")
            return 0

    @property
    def is_ready(self) -> bool:
        return self._ready

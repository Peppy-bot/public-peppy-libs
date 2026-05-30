from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_DAEMON_STATE_PATH = Path.home() / ".peppy" / "daemon_state.json"
_DEFAULT_MESSAGING_PORT = 7448
_ENV_CONFIG_PATH = "PEPPY_BRIDGE_CONFIG_PATH"
_ENV_NODE_NAME = "PEPPY_BRIDGE_NODE_NAME"
_ENV_HOST = "PEPPY_BRIDGE_HOST"
_ENV_PORT = "PEPPY_BRIDGE_PORT"
_ENV_PRIM_PATH = "PEPPY_BRIDGE_PRIM_PATH"
_ENV_ROBOT_NAME = "PEPPY_BRIDGE_ROBOT_NAME"
_ENV_STATES_TOPIC = "PEPPY_BRIDGE_STATES_TOPIC"
_ENV_COMMAND_TOPIC = "PEPPY_BRIDGE_COMMAND_TOPIC"
_ENV_COMMAND_SOURCE_NODE = "PEPPY_BRIDGE_COMMAND_SOURCE_NODE"
_DEFAULT_CONFIG_PATH = "config/sim_bridge.json5"


@dataclass(frozen=True)
class PublisherEntry:
    type: str
    prim: str
    topic: str
    robot_name: str = "robot"
    params: dict = field(default_factory=dict)


@dataclass(frozen=True)
class SubscriberEntry:
    type: str
    prim: str
    topic: str
    source_node: str = "sim_bridge"
    params: dict = field(default_factory=dict)


@dataclass(frozen=True)
class BridgeConfig:
    node_name: str
    host: str
    port: int
    daemon_node: str
    publishers: list[PublisherEntry] = field(default_factory=list)
    subscribers: list[SubscriberEntry] = field(default_factory=list)

    @classmethod
    def from_file(
        cls,
        path: Path | None = None,
        default_node_name: str = "sim",
    ) -> BridgeConfig:
        resolved = path or Path(os.environ.get(_ENV_CONFIG_PATH, _DEFAULT_CONFIG_PATH))
        try:
            raw = _read_jsonc(resolved)
            daemon_state = _read_daemon_state()
            return cls(
                node_name=os.environ.get(_ENV_NODE_NAME, default_node_name),
                host=os.environ.get(_ENV_HOST, "localhost"),
                port=_resolve_port(daemon_state),
                daemon_node=daemon_state.get("core_node_name", ""),
                publishers=[
                    PublisherEntry(
                        type=e["type"],
                        prim=e.get("prim", ""),
                        topic=e.get("topic", e["type"]),
                        robot_name=e.get("robot_name", "robot"),
                        params=_normalise_params(e.get("params"), e["type"]),
                    )
                    for e in raw.get("publishers", [])
                ],
                subscribers=[
                    SubscriberEntry(
                        type=e["type"],
                        prim=e.get("prim", ""),
                        topic=e.get("topic", e["type"]),
                        source_node=e.get("source_node", "sim_bridge"),
                        params=_normalise_params(e.get("params"), e["type"]),
                    )
                    for e in raw.get("subscribers", [])
                ],
            )
        except (OSError, json.JSONDecodeError):
            logger.exception(f"Could not load {resolved} — falling back to env vars")
            return cls.from_env(default_node_name=default_node_name)
        except KeyError as exc:
            raise ValueError(
                f"sim_bridge.json5 at {resolved} is missing required field {exc}"
                " — every publishers/subscribers entry needs at least a 'type' field"
            ) from exc

    @classmethod
    def from_env(cls, default_node_name: str = "sim") -> BridgeConfig:
        daemon_state = _read_daemon_state()
        prim = os.environ.get(_ENV_PRIM_PATH, "")
        robot_name = os.environ.get(_ENV_ROBOT_NAME, "robot")
        return cls(
            node_name=os.environ.get(_ENV_NODE_NAME, default_node_name),
            host=os.environ.get(_ENV_HOST, "localhost"),
            port=_resolve_port(daemon_state),
            daemon_node=daemon_state.get("core_node_name", ""),
            publishers=[
                PublisherEntry(
                    type="joint_states",
                    prim=prim,
                    topic=os.environ.get(_ENV_STATES_TOPIC, "joint_states"),
                    robot_name=robot_name,
                )
            ],
            subscribers=[
                SubscriberEntry(
                    type="actuator_ctrl",
                    prim=prim,
                    topic=os.environ.get(_ENV_COMMAND_TOPIC, "set_ctrl"),
                    source_node=os.environ.get(_ENV_COMMAND_SOURCE_NODE, "sim_bridge"),
                )
            ],
        )


def _resolve_port(daemon_state: dict[str, Any]) -> int:
    port_env = os.environ.get(_ENV_PORT, "").strip()
    try:
        return int(
            port_env
            if port_env
            else daemon_state.get("messaging_port", _DEFAULT_MESSAGING_PORT)
        )
    except (ValueError, TypeError):
        logger.warning(
            f"Invalid {_ENV_PORT} '{port_env}'"
            f" — using default {_DEFAULT_MESSAGING_PORT}"
        )
        return _DEFAULT_MESSAGING_PORT


def _read_daemon_state() -> dict[str, Any]:
    try:
        return json.loads(_DAEMON_STATE_PATH.read_text())
    except FileNotFoundError:
        logger.warning(
            f"daemon_state.json not found at {_DAEMON_STATE_PATH}"
            " — is the PeppyOS daemon running?"
        )
        return {}
    except (json.JSONDecodeError, OSError):
        logger.exception("Failed to read daemon_state.json")
        return {}


def _normalise_params(raw: Any, entry_type: str) -> dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    if raw is not None:
        logger.warning(
            f"Publisher entry '{entry_type}': params is {type(raw).__name__!r},"
            " expected dict — using {}."
        )
    return {}


def _read_jsonc(path: Path) -> dict[str, Any]:
    import pyjson5  # pylint: disable=E0401

    text = path.read_text()
    try:
        return pyjson5.loads(text)
    except pyjson5.Json5DecoderException as exc:
        # Re-raise as json.JSONDecodeError so BridgeConfig.from_file catches
        # it uniformly.
        raise json.JSONDecodeError(str(exc), text, 0) from exc

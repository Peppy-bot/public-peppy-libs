from __future__ import annotations

import asyncio
import logging
import queue
import threading
import uuid
from contextlib import contextmanager
from typing import Iterator, Optional, Protocol

from peppylib import (  # pylint: disable=E0401
    MessengerHandle,
    QoSProfile,
    TopicMessenger,
)
from peppylib.messaging import SenderTarget  # pylint: disable=E0401

logger = logging.getLogger(__name__)

_CONNECT_TIMEOUT_S = 10.0
_RECONNECT_MAX_BACKOFF_S = 30.0
_SUBSCRIBE_MAX_BACKOFF_S = 30.0
_QOS_MAP: dict[str, QoSProfile] = {
    "sensor_data": QoSProfile.SensorData,
    "standard": QoSProfile.Standard,
}


class _IOConfig(Protocol):
    host: str
    port: int
    daemon_node: str


@contextmanager
def peppylib_session(config: _IOConfig) -> Iterator[PeppylibIO]:
    io = PeppylibIO(config)
    io.start()
    try:
        yield io
    finally:
        io.stop()


class PeppylibIO:  # pylint: disable=R0902

    def __init__(self, config: _IOConfig) -> None:
        self._config = config
        self._instance_id: str = str(uuid.uuid4())
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._handle: Optional[MessengerHandle] = None
        self._queues: dict[tuple[str, str, str], queue.SimpleQueue] = {}
        self._queues_lock = threading.Lock()
        self._pending_subs: list[tuple[str, str, str, str]] = []
        self._all_subs: list[tuple[str, str, str, str]] = []
        self._subs_lock = threading.Lock()
        self._recv_tasks: list[asyncio.Task] = []
        self._ready = threading.Event()
        self._stop_future: Optional[asyncio.Future] = None

    def start(self) -> None:
        self._ready.clear()
        self._thread = threading.Thread(
            target=self._run_event_loop,
            name="peppy_bridge_io",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(timeout=_CONNECT_TIMEOUT_S):
            logger.warning(
                f"peppylib did not connect within {_CONNECT_TIMEOUT_S:.0f}s"
                " — background retry is active."
            )

    def stop(self) -> None:
        if self._loop is not None and not self._loop.is_closed():

            def _shutdown() -> None:
                if self._stop_future is not None and not self._stop_future.done():
                    self._stop_future.cancel()
                for task in self._recv_tasks:
                    if not task.done():
                        task.cancel()

            self._loop.call_soon_threadsafe(_shutdown)

        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                # MessengerHandle has no explicit close; teardown relies on
                # GC. If the I/O thread didn't observe the cancel within 5s,
                # the daemon connection lingers until process exit.
                logger.warning(
                    "peppylib I/O thread still alive after 5s shutdown timeout"
                    " — daemon connection may linger until process exit"
                )
            self._thread = None

        self._handle = None
        self._ready.clear()
        logger.info("peppylib I/O stopped.")

    def emit(
        self,
        node_name: str,
        topic: str,
        qos: str,
        payload: bytes,
        *,
        node_tag: str = "v1",
    ) -> None:
        if self._loop is None or self._loop.is_closed() or self._handle is None:
            return
        asyncio.run_coroutine_threadsafe(
            self._emit(node_name, node_tag, topic, qos, payload), self._loop
        )

    def register_subscription(
        self,
        source_node: str,
        topic: str,
        qos: str,
        *,
        source_tag: str = "v1",
    ) -> None:
        key = (source_node, source_tag, topic)
        with self._queues_lock:
            if key in self._queues:
                return
            self._queues[key] = queue.SimpleQueue()

        entry = (source_node, source_tag, topic, qos)
        schedule_now = False
        with self._subs_lock:
            if entry not in self._all_subs:
                self._all_subs.append(entry)
            if (
                self._loop is not None
                and not self._loop.is_closed()
                and self._ready.is_set()
            ):
                schedule_now = True
            else:
                self._pending_subs.append(entry)

        if schedule_now:
            asyncio.run_coroutine_threadsafe(
                self._subscribe_with_retry(source_node, source_tag, topic, qos),
                self._loop,
            )

    def get_latest(
        self, source_node: str, topic: str, *, source_tag: str = "v1"
    ) -> Optional[bytes]:
        with self._queues_lock:
            q = self._queues.get((source_node, source_tag, topic))
        if q is None:
            return None
        latest: Optional[bytes] = None
        try:
            while True:
                latest = q.get_nowait()
        except queue.Empty:
            pass
        return latest

    def get_all(
        self, source_node: str, topic: str, *, source_tag: str = "v1"
    ) -> list[bytes]:
        # FIFO drain of every queued message. Use when each message is a
        # distinct event (drops are bugs); use get_latest for idempotent
        # signals where latest wins.
        with self._queues_lock:
            q = self._queues.get((source_node, source_tag, topic))
        if q is None:
            return []
        out: list[bytes] = []
        try:
            while True:
                out.append(q.get_nowait())
        except queue.Empty:
            pass
        return out

    def _run_event_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._async_main())
        except Exception as exc:
            logger.error(f"peppylib I/O loop exited with error: {exc}")
        finally:
            self._loop.close()

    async def _async_main(self) -> None:
        backoff = 1.0
        while True:
            try:
                await self._connect_and_run()
                break
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.warning(
                    f"peppylib connection error: {exc} — reconnecting in {backoff:.0f}s"
                )
                self._handle = None
                self._ready.clear()
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, _RECONNECT_MAX_BACKOFF_S)

    async def _connect_and_run(self) -> None:
        cfg = self._config
        logger.info(
            f"Connecting to PeppyOS daemon — host={cfg.host}"
            f"  port={cfg.port}  daemon_node={cfg.daemon_node}"
        )

        # Cancel stale recv tasks from a previous connection attempt.
        for task in list(self._recv_tasks):
            task.cancel()
        self._recv_tasks.clear()

        self._handle = await MessengerHandle.from_host_port(cfg.host, cfg.port)

        # On reconnect, all_subs covers subscriptions that pending_subs may have
        # missed if they were registered before the loop was ready.
        with self._subs_lock:
            to_start: list[tuple[str, str, str, str]] = self._pending_subs[:]
            self._pending_subs.clear()
            already = {(s, tg, t) for s, tg, t, _ in to_start}
            for s, tg, t, q in self._all_subs:
                if (s, tg, t) not in already:
                    to_start.append((s, tg, t, q))

        loop = asyncio.get_event_loop()
        for source_node, source_tag, topic, qos in to_start:
            task = loop.create_task(
                self._subscribe_with_retry(source_node, source_tag, topic, qos)
            )
            task.add_done_callback(
                lambda t: self._recv_tasks.remove(t) if t in self._recv_tasks else None
            )
            self._recv_tasks.append(task)

        self._ready.set()
        logger.info(f"peppylib connected (instance_id={self._instance_id}).")

        stop_future: asyncio.Future = loop.create_future()
        self._stop_future = stop_future
        try:
            await stop_future
        except asyncio.CancelledError:
            pass
        finally:
            self._stop_future = None

    async def _subscribe_with_retry(
        self, source_node: str, source_tag: str, topic: str, qos: str
    ) -> None:
        backoff = 1.0
        while True:
            try:
                sub = await self._subscribe_once(source_node, source_tag, topic, qos)
                await self._recv_loop((source_node, source_tag, topic), sub)
                logger.info(
                    f"{source_node}:{source_tag}/{topic} subscription closed"
                    " — re-subscribing."
                )
                backoff = 1.0
            except asyncio.CancelledError:
                return
            except Exception as exc:
                logger.warning(
                    f"Subscribe error for {source_node}:{source_tag}/{topic}: {exc}"
                    f" — retrying in {backoff:.0f}s"
                )
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, _SUBSCRIBE_MAX_BACKOFF_S)

    async def _subscribe_once(
        self, source_node: str, source_tag: str, topic: str, qos: str
    ):
        if self._handle is None:
            raise RuntimeError("No daemon handle — not connected.")
        qos_profile = _QOS_MAP.get(qos, QoSProfile.Standard)
        cfg = self._config
        logger.info(
            f"Subscribing to {source_node}:{source_tag}/{topic}"
            f"  (daemon_node={cfg.daemon_node})."
        )
        return await TopicMessenger.subscribe(
            self._handle,
            cfg.daemon_node,
            self._instance_id,
            SenderTarget.node(source_node, source_tag),
            topic,
            None,
            None,
            qos_profile,
        )

    async def _recv_loop(self, key: tuple[str, str, str], sub) -> None:
        while True:
            msg = await sub.on_next_message()
            if msg is None:
                break
            with self._queues_lock:
                q = self._queues.get(key)
            if q is not None:
                q.put(msg.payload)

    async def _emit(
        self,
        node_name: str,
        node_tag: str,
        topic: str,
        qos: str,
        payload: bytes,
    ) -> None:
        qos_profile = _QOS_MAP.get(qos, QoSProfile.Standard)
        try:
            await TopicMessenger.emit(
                self._handle,
                self._config.daemon_node,
                self._instance_id,
                SenderTarget.node(node_name, node_tag),
                topic,
                qos_profile,
                payload,
            )
        except Exception as exc:
            logger.warning(f"Failed to emit {node_name}:{node_tag}/{topic}: {exc}")

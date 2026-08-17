"""Node-invariant test machinery — the Python twin of Rust's ``peppylib::testing``.

Everything here is untyped (``bytes`` in, ``bytes`` out) on purpose: this module
is the single Python implementation of the semantics that generated per-node
test code (peppygen's ``mock`` / ``fixtures`` surfaces) and peppylib's own test
suites share. Generated code contributes only a typed veneer over these cores:
message-type reuse, codecs, identity constants, per-link aggregation. If a
helper here ever wants to name a generated type, that logic belongs in the
veneer instead.

Deliberately not imported by ``peppylib/__init__.py``: the wheel ships it
inert, and only test code ever imports ``peppylib.testing``.

Semantics mirror ``peppylib-rs/src/testing.rs`` — mirror, not re-design; a
behavior difference between the two is a bug here, and both sides carry
equivalent test suites in public-peppy-libs to surface one early.
"""

from __future__ import annotations

import asyncio
import os
import warnings
import weakref
from collections import deque
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Iterable, Sequence

from peppylib import (
    ActionMessenger,
    ConcurrentAction,
    GoalContext,
    MessengerHandle,
    NodeRunner,
    PendingGoal,
    ProducerRef,
    QoSProfile,
    SenderTarget,
    ServiceMessenger,
    StandaloneConfig,
    TopicMessenger,
    TopicPublisher,
    ZenohdInstance,
)
from peppylib.messaging.services import ServiceRequestContext, ServiceResponder

#: How long readiness waits (subscriber matching, reachability) may take
#: before failing loudly. Generous on purpose: every wait returns the moment
#: its condition is observed, so the bound only prices the failure path.
READINESS_TIMEOUT = 10.0

_CONNECT_RETRIES = 5
_CONNECT_RETRY_DELAY = 0.2
_REACHABILITY_POLL_INTERVAL = 0.025


def prepare_test_process() -> None:
    """One-time test-process environment setup; idempotent.

    - Gives zenoh's global Net runtime more worker threads (stock zenoh 1.9.0
      through at least 1.10.0 can deadlock its routing layer under
      peer-session churn on the single-worker default; an operator-provided
      ``ZENOH_RUNTIME`` wins). Must run before the first session or router
      lazily initializes the native runtime — importing this module (or
      calling :meth:`EphemeralRouter.start`) is early enough for test code.
    - Raises the soft ``nofile`` limit toward the hard limit, best effort:
      parallel routers/sessions can exhaust the macOS default soft limit of
      256, surfacing as flaky EMFILE errors.
    """
    os.environ.setdefault("ZENOH_RUNTIME", "(net: (worker_threads: 4))")
    try:
        import resource

        soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        ceiling = 8192 if hard == resource.RLIM_INFINITY else min(8192, hard)
        if soft < ceiling:
            resource.setrlimit(resource.RLIMIT_NOFILE, (ceiling, hard))
    except (ImportError, ValueError, OSError):
        pass


# Run at import time so the environment is set before the native runtime can
# lazily initialize — the Python stand-in for the Rust module's pre-main ctor.
prepare_test_process()


# One mesh-serialization lock per event loop (pytest-asyncio gives each test
# its own loop, and asyncio primitives refuse cross-loop reuse). Sequential
# tests never overlap meshes anyway; the lock exists for tests that start
# several routers/harnesses concurrently on one loop — one mesh at a time
# keeps gossip discovery fast and deterministic.
_MESH_SERIAL_LOCKS: "weakref.WeakKeyDictionary[asyncio.AbstractEventLoop, asyncio.Lock]" = (
    weakref.WeakKeyDictionary()
)


def _mesh_serial_lock() -> asyncio.Lock:
    loop = asyncio.get_running_loop()
    lock = _MESH_SERIAL_LOCKS.get(loop)
    if lock is None:
        lock = asyncio.Lock()
        _MESH_SERIAL_LOCKS[loop] = lock
    return lock


async def acquire_mesh_serial() -> asyncio.Lock:
    """Acquire this loop's mesh-serialization lock directly, for tests that
    build their own mesh without an :class:`EphemeralRouter`. The caller must
    ``release()`` it; :meth:`EphemeralRouter.start` manages it internally."""
    lock = _mesh_serial_lock()
    await lock.acquire()
    return lock


async def connect_messenger(host: str, port: int) -> MessengerHandle:
    """Open a gossip-peer session against ``host:port``, retrying the
    first-connect races a just-ready router can still lose."""
    last_error: Exception | None = None
    for attempt in range(_CONNECT_RETRIES):
        try:
            return await MessengerHandle.from_host_port(host, port)
        except Exception as error:  # noqa: BLE001 — retried, then re-raised
            last_error = error
            if attempt + 1 < _CONNECT_RETRIES:
                await asyncio.sleep(_CONNECT_RETRY_DELAY)
    assert last_error is not None
    raise last_error


class EphemeralRouter:
    """An external zenohd on an ephemeral port, wrapped for tests: started
    with a real readiness probe (no sleeps), stopped by :meth:`shutdown` /
    ``async with``.

    :meth:`start` also holds this loop's mesh-serialization lock for the
    router's lifetime, so concurrent harnesses on one loop run one mesh at a
    time. The lock is not reentrant: a test that needs two routers at once
    must use :meth:`start_unserialized` for the second (or both) and accept
    the discovery-contention flake risk.
    """

    def __init__(self, instance: ZenohdInstance, serial: asyncio.Lock | None) -> None:
        self._instance = instance
        self._serial = serial

    @classmethod
    async def start(cls) -> "EphemeralRouter":
        return await cls.start_on("127.0.0.1", None)

    @classmethod
    async def start_on(cls, host: str, port: int | None = None) -> "EphemeralRouter":
        prepare_test_process()
        serial = await acquire_mesh_serial()
        try:
            instance = await ZenohdInstance.start_ephemeral(host, port)
        except BaseException:
            serial.release()
            raise
        return cls(instance, serial)

    @classmethod
    async def start_unserialized(
        cls, host: str = "127.0.0.1", port: int | None = None
    ) -> "EphemeralRouter":
        prepare_test_process()
        instance = await ZenohdInstance.start_ephemeral(host, port)
        return cls(instance, None)

    @property
    def host(self) -> str:
        return self._instance.host

    @property
    def port(self) -> int:
        return self._instance.port

    def connection_target(self) -> tuple[str, int]:
        return (self._instance.host, self._instance.port)

    async def connect(self) -> MessengerHandle:
        """A fresh gossip-peer session against this router, with retries."""
        return await connect_messenger(self.host, self.port)

    async def shutdown(self) -> None:
        try:
            await self._instance.stop()
        finally:
            if self._serial is not None:
                self._serial.release()
                self._serial = None

    async def stop(self) -> None:
        """Alias for :meth:`shutdown`, matching ``ZenohdInstance.stop`` so the
        wrapper drops into call sites written against the raw instance."""
        await self.shutdown()

    async def __aenter__(self) -> "EphemeralRouter":
        return self

    async def __aexit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        await self.shutdown()


class TestTopicPublisher:
    """A publisher whose **first** publish deterministically waits until the
    publishing session sees a matching subscriber, then publishes.

    In gossip mode a freshly-connected publisher learns about existing
    subscribers asynchronously, so an unguarded first publish can be dropped
    before routing propagates; the wait is keyed by this publisher's exact
    wire identity (link_id segment included). Subsequent publishes skip the
    wait. No subscriber within the readiness timeout is an error, not a
    silent drop: the node under test never subscribed where the test
    publishes.
    """

    # The name starts with "Test" (mirroring the Rust core), which pytest
    # would otherwise try to collect as a test class when imported into a
    # test module.
    __test__ = False

    def __init__(
        self,
        publisher: TopicPublisher,
        messenger: MessengerHandle,
        as_core_node: str,
        as_instance_id: str,
        as_target: SenderTarget,
        link_id: str | None,
        topic: str,
        readiness_timeout: float,
    ) -> None:
        self._publisher = publisher
        self._messenger = messenger
        self._as_core_node = as_core_node
        self._as_instance_id = as_instance_id
        self._as_target = as_target
        self._link_id = link_id
        self._topic = topic
        self._matched = False
        self.readiness_timeout = readiness_timeout

    @classmethod
    async def declare(
        cls,
        messenger: MessengerHandle,
        as_core_node: str,
        as_instance_id: str,
        as_target: SenderTarget,
        topic: str,
        qos: QoSProfile,
        link_id: str | None = None,
        readiness_timeout: float = READINESS_TIMEOUT,
    ) -> "TestTopicPublisher":
        publisher = await TopicMessenger.declare_publisher(
            messenger, as_core_node, as_instance_id, as_target, topic, qos, link_id
        )
        return cls(
            publisher,
            messenger,
            as_core_node,
            as_instance_id,
            as_target,
            link_id,
            topic,
            readiness_timeout,
        )

    async def wait_for_subscriber(self, timeout: float) -> bool:
        """Wait until a subscriber matching this publisher's exact wire
        identity is visible, or ``timeout`` elapses; marks the publisher
        matched on success so later publishes skip the wait."""
        matched = await TopicMessenger.wait_for_subscriber_with_link_id(
            self._messenger,
            self._as_core_node,
            self._as_instance_id,
            self._as_target,
            self._topic,
            timeout,
            self._link_id,
        )
        if matched:
            self._matched = True
        return bool(matched)

    async def publish(self, payload: bytes) -> None:
        if not self._matched and not await self.wait_for_subscriber(self.readiness_timeout):
            raise RuntimeError(
                f"no subscriber for topic `{self._topic}` (link_id {self._link_id!r}) "
                f"appeared within {self.readiness_timeout}s: the node under test never "
                "opened a matching subscription — check that the link is seeded in the "
                "harness config and that the node subscribes to this topic"
            )
        await self._publisher.publish(payload)


async def wait_service_reachable(
    messenger: MessengerHandle,
    bound_core_node: str,
    as_instance_id: str,
    to_target: SenderTarget,
    to_service_name: str,
    producer: ProducerRef | None,
    timeout: float = READINESS_TIMEOUT,
) -> None:
    """Wait until the pinned producer's service answers reachability probes.

    The cold-start counterpart of the topic-side subscriber wait: a fresh
    session's first ``poll`` can race gossip discovery of an already-declared
    queryable, and a service query that misses is a hard unreachable error,
    so callers gate on this first.
    """
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        if await ServiceMessenger.is_reachable(
            messenger, bound_core_node, as_instance_id, to_target, to_service_name, producer
        ):
            return
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError(
                f"service `{to_service_name}` on {producer!r} did not become reachable "
                f"within {timeout}s"
            )
        await asyncio.sleep(_REACHABILITY_POLL_INTERVAL)


async def wait_action_reachable(
    messenger: MessengerHandle,
    bound_core_node: str,
    as_instance_id: str,
    to_target: SenderTarget,
    to_action_name: str,
    producer: ProducerRef | None,
    timeout: float = READINESS_TIMEOUT,
) -> None:
    """:func:`wait_service_reachable` for an action's goal service."""
    deadline = asyncio.get_running_loop().time() + timeout
    while True:
        if await ActionMessenger.is_reachable(
            messenger, bound_core_node, as_instance_id, to_target, to_action_name, producer
        ):
            return
        if asyncio.get_running_loop().time() >= deadline:
            raise TimeoutError(
                f"action `{to_action_name}` on {producer!r} did not become reachable "
                f"within {timeout}s"
            )
        await asyncio.sleep(_REACHABILITY_POLL_INTERVAL)


@dataclass
class CapturedServiceRequest:
    """One request captured by a :class:`MockServiceCore`, kept for later
    assertions regardless of whether the response was scripted or manual."""

    core_node: str
    instance_id: str
    link_id: str
    payload: bytes


class MockServiceCore:
    """Server side of one mocked service: a background pump owns the endpoint
    and captures every inbound request; a request is answered from the
    scripted response queue when one is enqueued, and handed to
    :meth:`next_request` otherwise. Requests neither path consumed are
    reported loudly at :meth:`close`."""

    def __init__(self, service_name: str) -> None:
        self._service_name = service_name
        self._requests: asyncio.Queue[tuple[ServiceRequestContext, ServiceResponder]] = (
            asyncio.Queue()
        )
        self._scripted: deque[bytes] = deque()
        self._captured: list[CapturedServiceRequest] = []
        self._pump: asyncio.Task[None] | None = None

    @classmethod
    async def listen(
        cls,
        messenger: MessengerHandle,
        as_core_node: str,
        as_instance_id: str,
        as_identity: SenderTarget,
        as_service_name: str,
    ) -> "MockServiceCore":
        endpoint = await ServiceMessenger.listen(
            messenger, as_core_node, as_instance_id, as_identity, as_service_name
        )
        core = cls(as_service_name)
        core._pump = asyncio.create_task(core._run_pump(endpoint))
        return core

    async def _run_pump(self, endpoint: Any) -> None:
        while True:
            pair = await endpoint.recv_next_request()
            if pair is None:
                return
            context, responder = pair
            self._captured.append(
                CapturedServiceRequest(
                    core_node=context.core_node,
                    instance_id=context.instance_id,
                    link_id=context.link_id,
                    payload=bytes(context.payload),
                )
            )
            # Scripted responses win over manual receives: a test that
            # enqueued N responses gets them served in order as requests
            # arrive, and only unscripted requests park for next_request.
            if self._scripted:
                response = self._scripted.popleft()
                try:
                    await responder.respond(response)
                except Exception as error:  # noqa: BLE001 — mock keeps serving
                    warnings.warn(
                        f"mock service `{self._service_name}` failed to send a scripted "
                        f"response: {error}",
                        stacklevel=1,
                    )
            else:
                await self._requests.put((context, responder))

    async def next_request(
        self, timeout: float = READINESS_TIMEOUT
    ) -> tuple[ServiceRequestContext, ServiceResponder]:
        """The next unscripted request, with the responder the test must use
        to answer it. Errors after ``timeout`` — a node that never called is a
        test failure surfaced here, not a hang."""
        try:
            return await asyncio.wait_for(self._requests.get(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(
                f"mock service `{self._service_name}` received no request within {timeout}s"
            ) from None

    def enqueue_response(self, response: bytes) -> None:
        """Enqueue one response to be served automatically to the next inbound
        request (FIFO across repeated calls)."""
        self._scripted.append(response)

    def enqueue_responses(self, responses: Iterable[bytes]) -> None:
        for response in responses:
            self.enqueue_response(response)

    def captured(self) -> list[CapturedServiceRequest]:
        """Every request captured so far (scripted and manual alike), in
        arrival order."""
        return list(self._captured)

    def take_captured(self) -> list[CapturedServiceRequest]:
        taken = self._captured
        self._captured = []
        return taken

    async def close(self) -> None:
        """Stop the pump and report anything the test left dangling. Python
        has no reliable drop hook for async teardown, so the veneer/harness
        must call this (Rust's core does the same reporting in ``Drop``)."""
        if self._pump is not None:
            self._pump.cancel()
            try:
                await self._pump
            except asyncio.CancelledError:
                pass
            self._pump = None
        unconsumed = self._requests.qsize()
        if unconsumed:
            warnings.warn(
                f"mock service `{self._service_name}` closed with {unconsumed} unconsumed "
                "request(s): the node called this service and the test neither scripted "
                "a response nor received the request",
                stacklevel=1,
            )
        if self._scripted:
            warnings.warn(
                f"mock service `{self._service_name}` closed with {len(self._scripted)} "
                "scripted response(s) never requested by the node",
                stacklevel=1,
            )


class MockPendingGoal:
    """A goal received by a :class:`MockActionServerCore`, awaiting the
    test's admission decision."""

    def __init__(self, pending: PendingGoal, live: list[Any]) -> None:
        self._pending = pending
        self._live = live

    @property
    def goal_id(self) -> str:
        return self._pending.goal_id

    @property
    def core_node(self) -> str:
        return self._pending.core_node

    @property
    def instance_id(self) -> str:
        return self._pending.instance_id

    @property
    def request_bytes(self) -> bytes:
        return bytes(self._pending.request_bytes)

    async def accept(self, response: bytes) -> GoalContext:
        """Accept the goal. The returned context drives feedback/completion;
        it is registered with the owning mock so
        :meth:`MockActionServerCore.stop` can disarm it for deterministic
        producer-loss."""
        context = await self._pending.accept(response)
        self._live.append(weakref.ref(context))
        return context

    async def reject(self, reason: str | None, response: bytes) -> None:
        await self._pending.reject(reason, response)


class MockActionServerCore:
    """Server side of one mocked action, on the real ``ConcurrentAction``
    engine: the full goal lifecycle (admission ack, cancel routing, feedback
    stream, result retention) behaves exactly as a production action
    server's.

    :meth:`stop` is the deterministic producer-loss primitive: it disarms
    every live goal's close-on-drop transition (so no clean feedback-end
    sentinel races the loss signal) and releases the engine, whose liveliness
    token going absent is what consumers observe as the producer-gone error.
    The caller must also drop the mock's session for the loss to be complete;
    the generated veneer owns that ordering.
    """

    def __init__(self, action_name: str, engine: ConcurrentAction) -> None:
        self._action_name = action_name
        self._engine: ConcurrentAction | None = engine
        self._live: list[Any] = []

    @classmethod
    async def expose(
        cls,
        messenger: MessengerHandle,
        bound_core_node: str,
        as_instance_id: str,
        as_identity: SenderTarget,
        as_action_name: str,
        has_feedback: bool,
    ) -> "MockActionServerCore":
        engine = await ConcurrentAction.expose(
            messenger, bound_core_node, as_instance_id, as_identity, as_action_name, has_feedback
        )
        return cls(as_action_name, engine)

    async def next_goal(self, timeout: float = READINESS_TIMEOUT) -> MockPendingGoal:
        """Park until the node under test sends a goal, bounded by
        ``timeout``."""
        if self._engine is None:
            raise RuntimeError(f"mock action `{self._action_name}` is stopped")
        try:
            pending = await asyncio.wait_for(self._engine.recv_next_goal(), timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(
                f"mock action `{self._action_name}` received no goal within {timeout}s"
            ) from None
        if pending is None:
            raise RuntimeError(
                f"mock action `{self._action_name}`'s goal stream closed unexpectedly"
            )
        return MockPendingGoal(pending, self._live)

    def stop(self) -> None:
        """Simulate this producer disappearing mid-goal, deterministically:
        every live goal context is disarmed (its eventual release emits
        neither the abandon transition nor the feedback-end sentinel), then
        the engine is released — CPython's refcounting drops it immediately,
        stopping its routing loops and removing the producer liveliness token
        consumers latch on."""
        for ref in self._live:
            context = ref()
            if context is not None:
                context.disarm_close_on_drop()
        self._live.clear()
        self._engine = None


@dataclass
class PublisherReadiness:
    """One entry of the harness's pre-setup readiness barrier: before the
    node's ``setup`` runs, the harness waits — on the node's own session —
    until a subscriber matching the exact keyexpr the node's publisher will
    emit on is visible. Without this, the node's very first publish can be
    dropped while its fresh session is still discovering the harness's
    already-declared subscriptions (subscribe-first alone is not
    sufficient in gossip mode)."""

    target: SenderTarget
    topic: str
    #: The node's own producer-side link_id for slot-scoped publishers
    #: (pairing slots); ``None`` for plain emitted topics.
    link_id: str | None = None


SetupFn = Callable[[Any, NodeRunner], Awaitable[None]]


class HarnessCore:
    """The node-invariant half of the generated test harness: builds the node
    in-process from an already-seeded ``StandaloneConfig``, runs the
    pre-setup readiness barrier, spawns the node's ``setup``, and owns
    teardown convergence. The generated veneer contributes what is per-node:
    mock construction, config seeding, parameter hydration, and typed
    observation clients.

    Teardown contract (:meth:`shutdown`): cancel the node's token → await
    ``setup`` bounded by the shutdown grace (propagating its error if it
    raised; a long-running ``setup`` that parks on the token's subscriptions
    is cancelled like production drops it) → run the registered shutdown
    hooks. Mirrors Rust's ``HarnessCore``.
    """

    def __init__(self, node_runner: NodeRunner, setup_task: asyncio.Task[None]) -> None:
        self._node_runner = node_runner
        self._setup_task: asyncio.Task[None] | None = setup_task

    @classmethod
    async def start(
        cls,
        peppy_config_path: str | os.PathLike[str],
        standalone_config: StandaloneConfig,
        publisher_readiness: Sequence[PublisherReadiness],
        setup: SetupFn,
        parameters: Any = None,
    ) -> "HarnessCore":
        """Build and start the node under test. ``standalone_config`` must
        already carry the messaging endpoint, instance id, parameters, and one
        seeding call per mock. ``setup`` is the node's real entry point (the
        exact ``async def setup(params, node_runner)`` shape ``NodeBuilder``
        runs), spawned only after the readiness barrier passes; ``parameters``
        is forwarded to it verbatim (the veneer passes the hydrated typed
        Parameters).

        ``new_standalone`` never consults ``PEPPY_RUNTIME_CONFIG``, so an
        inherited daemon environment cannot hijack the harness; a set variable
        still draws a warning because the author probably didn't intend it.
        """
        if os.environ.get("PEPPY_RUNTIME_CONFIG"):
            warnings.warn(
                "PEPPY_RUNTIME_CONFIG is set but ignored: the test harness always runs "
                "standalone (it must not be hijacked into daemon mode by an inherited "
                "environment)",
                stacklevel=2,
            )
        node_runner = await NodeRunner.new_standalone(
            str(os.fspath(peppy_config_path)), standalone_config
        )

        # Pre-setup readiness barrier: a publisher-side matching wait per
        # observed topic, on the node's session, with the identical keyexpr
        # the node's publisher will use.
        for probe in publisher_readiness:
            matched = await TopicMessenger.wait_for_subscriber_with_link_id(
                node_runner.messenger(),
                node_runner.bound_core_node(),
                node_runner.bound_instance_id(),
                probe.target,
                probe.topic,
                READINESS_TIMEOUT,
                probe.link_id,
            )
            if not matched:
                raise RuntimeError(
                    f"readiness barrier: the harness subscription for topic "
                    f"`{probe.topic}` (link_id {probe.link_id!r}) was not visible to the "
                    f"node's session within {READINESS_TIMEOUT}s; the mesh never routed "
                    "it — this is a harness/mock wiring bug, not a node bug"
                )

        setup_task = asyncio.create_task(setup(parameters, node_runner))
        return cls(node_runner, setup_task)

    @property
    def node_runner(self) -> NodeRunner:
        return self._node_runner

    def messenger(self) -> MessengerHandle:
        """The node's own session — the one its publishers and subscriptions
        live on."""
        return self._node_runner.messenger()

    def instance_id(self) -> str:
        return self._node_runner.bound_instance_id()

    def bound_core_node(self) -> str:
        return self._node_runner.bound_core_node()

    def cancellation_token(self) -> Any:
        return self._node_runner.cancellation_token()

    def setup_finished(self) -> bool:
        """Whether the spawned ``setup`` has already returned (many setups
        register their loops and return immediately; long-running ones never
        do until shutdown)."""
        return self._setup_task is None or self._setup_task.done()

    async def shutdown(self) -> None:
        """Converge the node: see the class-level teardown contract. Raises
        the ``setup`` error if it failed — a test whose node errored during
        setup should fail even when its assertions never noticed."""
        self._node_runner.cancellation_token().cancel()
        grace = self._node_runner.shutdown_grace_secs()
        setup_error: BaseException | None = None
        task = self._setup_task
        self._setup_task = None
        if task is not None:
            try:
                await asyncio.wait_for(task, grace)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                # A setup that parks forever without watching the node's
                # cancellation token; production drops it at shutdown the
                # same way (wait_for already cancelled it).
                pass
            except BaseException as error:  # noqa: BLE001 — re-raised after hooks
                setup_error = error
        await self._node_runner.run_shutdown_hooks(grace)
        if setup_error is not None:
            raise setup_error

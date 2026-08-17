"""Suite for ``peppylib.testing`` — the Python twin of the Rust cores behind
generated per-node mock/fixtures code (mirrors
``peppylib-rs/tests/testing.rs`` case for case, so a semantic difference
between the two languages surfaces here, not in node suites)."""

import asyncio
import gc
from pathlib import Path

import pytest

import peppylib.testing as peppy_testing
from peppylib import (
    ActionMessenger,
    ProducerRef,
    QoSProfile,
    SenderTarget,
    ServiceMessenger,
    StandaloneConfig,
    TopicMessenger,
)
from peppylib.testing import (
    EphemeralRouter,
    HarnessCore,
    MockActionServerCore,
    MockServiceCore,
    PublisherReadiness,
    TestTopicPublisher,
    wait_action_reachable,
    wait_service_reachable,
)

MOCK_CORE = "mock_core"
MOCK_INSTANCE = "mock_1"
CALLER_CORE = "caller_core"
CALLER_INSTANCE = "caller_1"

_PEPPY_CONFIG = """{
    peppy_schema: "node/v1",
    manifest: { name: "test_node", tag: "v1" },
    execution: { language: "python", run_cmd: ["uv", "run"] },
}"""


def _node_target(name: str) -> SenderTarget:
    return SenderTarget.node(name, "v1")


def _write_peppy_config(tmp_path: Path) -> Path:
    path = tmp_path / "peppy.json5"
    path.write_text(_PEPPY_CONFIG)
    return path


@pytest.mark.asyncio
async def test_mock_service_core_scripted_then_manual():
    """Scripted responses serve automatically in FIFO order; unscripted
    requests park for ``next_request``; every request is captured."""
    async with await EphemeralRouter.start() as router:
        mock_handle = await router.connect()
        caller_handle = await router.connect()

        mock = await MockServiceCore.listen(
            mock_handle, MOCK_CORE, MOCK_INSTANCE, _node_target("dep_node"), "get_info"
        )
        try:
            mock.enqueue_response(b"scripted-response")
            producer = ProducerRef(MOCK_CORE, MOCK_INSTANCE)
            # Cold-start gate: the caller session is fresh, so its first query
            # can race gossip discovery of the mock's queryable.
            await wait_service_reachable(
                caller_handle,
                CALLER_CORE,
                CALLER_INSTANCE,
                _node_target("dep_node"),
                "get_info",
                producer,
            )

            scripted = await ServiceMessenger.poll(
                caller_handle,
                CALLER_CORE,
                CALLER_INSTANCE,
                _node_target("dep_node"),
                "get_info",
                producer,
                b"request-1",
                5.0,
            )
            assert scripted.payload == b"scripted-response"

            # Manual path: the caller's poll parks until the test receives
            # the request, asserts on it, and responds.
            manual_poll = asyncio.ensure_future(
                ServiceMessenger.poll(
                    caller_handle,
                    CALLER_CORE,
                    CALLER_INSTANCE,
                    _node_target("dep_node"),
                    "get_info",
                    producer,
                    b"request-2",
                    5.0,
                )
            )
            context, responder = await mock.next_request(timeout=5.0)
            assert context.payload == b"request-2"
            assert context.core_node == CALLER_CORE
            assert context.instance_id == CALLER_INSTANCE
            await responder.respond(b"manual-response")

            manual = await asyncio.wait_for(manual_poll, timeout=5.0)
            assert manual.payload == b"manual-response"

            captured = mock.captured()
            assert [c.payload for c in captured] == [b"request-1", b"request-2"]
            assert captured[0].core_node == CALLER_CORE
        finally:
            await mock.close()


@pytest.mark.asyncio
async def test_mock_action_stop_yields_producer_gone_deterministically():
    """``stop()`` on a mock action with a live, user-held goal context must
    surface to the consumer as the producer-gone ``ConnectionError`` — never
    a clean feedback close and never a hang — because the disarmed context
    suppresses the drop-time sentinel that would race the liveliness latch."""
    async with await EphemeralRouter.start() as router:
        mock_handle = await router.connect()
        client_handle = await router.connect()

        mock = await MockActionServerCore.expose(
            mock_handle, MOCK_CORE, MOCK_INSTANCE, _node_target("arm_node"), "move_arm", True
        )

        producer = ProducerRef(MOCK_CORE, MOCK_INSTANCE)
        await wait_action_reachable(
            client_handle,
            CALLER_CORE,
            CALLER_INSTANCE,
            _node_target("arm_node"),
            "move_arm",
            producer,
        )

        # send_goal resolves only once the mock answers admission, so it runs
        # concurrently with next_goal → accept below.
        goal_task = asyncio.ensure_future(
            ActionMessenger.send_goal(
                client_handle,
                CALLER_CORE,
                CALLER_INSTANCE,
                _node_target("arm_node"),
                "move_arm",
                producer,
                b"goal",
                QoSProfile.Reliable,
                5.0,
            )
        )

        pending = await mock.next_goal(timeout=5.0)
        assert pending.request_bytes == b"goal"
        context = await pending.accept(b"accepted")
        goal_handle = await asyncio.wait_for(goal_task, timeout=5.0)
        assert goal_handle.accepted

        await context.publish_feedback(b"working")
        feedback = await asyncio.wait_for(goal_handle.on_next_feedback(), timeout=5.0)
        assert feedback.payload == b"working"

        # Stop the mock mid-goal with the context still held (as a test's
        # MockGoal handle would be) and tear down its session — the
        # producer-loss shape.
        mock.stop()
        del mock, mock_handle
        gc.collect()

        with pytest.raises(ConnectionError):
            await asyncio.wait_for(goal_handle.on_next_feedback(), timeout=15.0)

        # Releasing the disarmed context afterwards is inert: no late
        # sentinel (nothing to receive it anyway — the session is gone).
        del context
        gc.collect()


@pytest.mark.asyncio
async def test_topic_publisher_first_publish_is_delivered():
    """With the subscriber already up, the very first publish must be
    delivered (no sleeps anywhere); with no subscriber, the publish fails
    loudly instead of dropping silently."""
    async with await EphemeralRouter.start() as router:
        sub_handle = await router.connect()
        pub_handle = await router.connect()

        producer = ProducerRef(MOCK_CORE, MOCK_INSTANCE)
        subscription = await TopicMessenger.subscribe(
            sub_handle,
            CALLER_CORE,
            CALLER_INSTANCE,
            _node_target("camera"),
            "video_stream",
            producer,
            QoSProfile.Reliable,
        )

        publisher = await TestTopicPublisher.declare(
            pub_handle,
            MOCK_CORE,
            MOCK_INSTANCE,
            _node_target("camera"),
            "video_stream",
            QoSProfile.Reliable,
        )
        await publisher.publish(b"frame-1")

        received = await asyncio.wait_for(subscription.on_next_message(), timeout=5.0)
        assert received is not None
        assert received.payload == b"frame-1"

        orphan = await TestTopicPublisher.declare(
            pub_handle,
            MOCK_CORE,
            MOCK_INSTANCE,
            _node_target("camera"),
            "nobody_listens",
            QoSProfile.Reliable,
            readiness_timeout=0.25,
        )
        with pytest.raises(RuntimeError, match="nobody_listens"):
            await orphan.publish(b"lost")


@pytest.mark.asyncio
async def test_harness_core_boots_node_observes_first_publish_and_converges(tmp_path):
    """Full harness lifecycle: readiness barrier → setup spawn → the node's
    very first publish is observed → shutdown convergence runs the
    registered shutdown hooks."""
    async with await EphemeralRouter.start() as router:
        observer_handle = await router.connect()
        peppy_config_path = _write_peppy_config(tmp_path)
        instance_id = "harness_test_instance"

        # The harness-side observation subscription exists before the node
        # does; the barrier below guarantees the node's session discovered it
        # before setup's first publish.
        status_sub = await TopicMessenger.subscribe(
            observer_handle,
            CALLER_CORE,
            CALLER_INSTANCE,
            _node_target("test_node"),
            "status",
            ProducerRef("standalone-core", instance_id),
            QoSProfile.Reliable,
        )

        standalone_config = (
            StandaloneConfig()
            .with_messaging(router.host, router.port)
            .with_instance_id(instance_id)
        )
        readiness = [PublisherReadiness(target=_node_target("test_node"), topic="status")]

        hook_ran = asyncio.Event()

        async def setup(params, node_runner):
            assert params is None
            publisher = await TopicMessenger.declare_publisher(
                node_runner.messenger(),
                node_runner.bound_core_node(),
                node_runner.bound_instance_id(),
                SenderTarget.node(node_runner.node_name(), node_runner.node_tag()),
                "status",
                QoSProfile.Reliable,
            )
            # First publish, immediately: only the pre-setup barrier makes
            # this deliverable.
            await publisher.publish(b"alive")
            node_runner.on_shutdown(hook_ran.set)

        harness = await HarnessCore.start(
            peppy_config_path, standalone_config, readiness, setup
        )
        assert harness.instance_id() == instance_id
        assert harness.bound_core_node() == "standalone-core"

        first = await asyncio.wait_for(status_sub.on_next_message(), timeout=5.0)
        assert first is not None
        assert first.payload == b"alive"

        await harness.shutdown()
        assert hook_ran.is_set(), "shutdown() must run the node's registered shutdown hooks"


@pytest.mark.asyncio
async def test_harness_core_shutdown_propagates_setup_error(tmp_path):
    """A setup error is not swallowed by teardown: ``shutdown()`` raises it
    so the test fails even when its assertions never noticed."""
    async with await EphemeralRouter.start() as router:
        peppy_config_path = _write_peppy_config(tmp_path)
        standalone_config = (
            StandaloneConfig()
            .with_messaging(router.host, router.port)
            .with_instance_id("failing_setup_instance")
        )

        async def setup(params, node_runner):
            raise RuntimeError("setup boom")

        harness = await HarnessCore.start(peppy_config_path, standalone_config, [], setup)
        with pytest.raises(RuntimeError, match="setup boom"):
            await harness.shutdown()


@pytest.mark.asyncio
async def test_prepare_test_process_sets_zenoh_runtime():
    """Importing peppylib.testing configures the zenoh Net runtime workers
    unless the operator already chose a value."""
    import os

    assert os.environ.get("ZENOH_RUNTIME"), (
        "peppylib.testing must ensure ZENOH_RUNTIME is set before the native "
        "runtime initializes"
    )
    # Module import already ran; calling again is idempotent.
    peppy_testing.prepare_test_process()

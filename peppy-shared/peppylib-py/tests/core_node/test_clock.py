"""Integration tests for `peppylib.clock.synchronize` and `peppylib.clock.subscribe`.

Python equivalent of `crates/peppylib/tests/core_node/clock.rs`, exercised
through the high-level Python helpers.
"""

import asyncio

import pytest

import peppylib.testing as peppy_testing
from peppylib import (
    MessengerHandle,
    NodeRunner,
    ProducerRef,
    QoSProfile,
    SenderTarget,
    StandaloneConfig,
    TopicMessenger,
    clock,
)

from .common import (
    CLIENT_INSTANCE,
    CORE_NODE,
    CORE_NODE_TAG,
    SERVER_INSTANCE,
    spawn_stub_listener,
    start_router_and_runner,
    wait_until_reachable,
    write_standalone_peppy_config,
)


@pytest.mark.asyncio
async def test_synchronize_computes_offset_and_delay(tmp_path):
    """`clock.synchronize()` performs the NTP exchange and returns a typed ClockSync.

    The stub listener replies with a hand-picked (t1, t2) so we can assert the
    NTP math without depending on real wall-clock readings. Symmetric link with
    100 ns offset, 20 ns total round-trip — the math test in
    `crates/peppylib/src/core_node/clock.rs` covers the same scenario.
    """
    # Plausible nanosecond timestamps. The exact t0 the client stamps is racy,
    # so we set t1 / t2 to values *much* larger than any realistic offset would
    # produce — the assertions below check structure, not exact magnitudes.
    canned_response = clock.ClockResponse(
        client_send_time=0,  # echoed t0 — ignored by `synchronize`, which uses the live one
        server_recv_time=2_000_000_000_000,
        server_send_time=2_000_000_000_005,
    )

    router, node_runner, server_handle = await start_router_and_runner(tmp_path)
    try:
        # Echo back the canned bytes regardless of the request payload — the
        # test cares about the helper's response handling, not the server.
        handler = await spawn_stub_listener(server_handle, "clock", canned_response.encode())
        await wait_until_reachable(node_runner.messenger(), "clock")

        sync = await clock.synchronize(node_runner, 3.0)

        await handler
    finally:
        await router.stop()

    # The raw response is exposed; t1/t2 must be exactly the canned values.
    assert sync.raw.server_recv_time == 2_000_000_000_000
    assert sync.raw.server_send_time == 2_000_000_000_005
    # Round-trip delay must be non-negative — the helper saturates negatives to
    # zero (see `compute_sync` in peppylib/src/core_node/clock.rs).
    assert sync.round_trip_delay_ns >= 0
    # Offset between local and the canned (huge) server timestamps must be
    # roughly the difference between t1≈t2 and t0≈t3. With t0 stamped from
    # SystemTime::now() (UNIX nanoseconds, ~1.7e18 today) and t1=2e12, the
    # offset is large and negative — local clock leads the canned server time.
    assert sync.offset_ns < 0


@pytest.mark.asyncio
async def test_subscribe_clock_yields_typed_ticks(tmp_path):
    """`clock.subscribe(node_runner)` decodes published ClockTicks for the caller.

    Mirrors `subscribe_clock_yields_typed_ticks` from the Rust integration
    test. Subscribes via the helper *before* publishing so the subscription is
    in place by the time the tick is emitted.
    """
    router, node_runner, server_handle = await start_router_and_runner(tmp_path)
    try:
        sub = await clock.subscribe(node_runner)
        # Brief settle for zenoh discovery — same idiom as test_topics.py.
        await asyncio.sleep(0.05)

        canned = clock.ClockTick(time=1_700_000_000_123_456_789)
        publisher = await TopicMessenger.declare_publisher(
            server_handle,
            CORE_NODE,
            SERVER_INSTANCE,
            SenderTarget.node(CORE_NODE, CORE_NODE_TAG),
            "clock",
            QoSProfile.SensorData,
        )
        await publisher.publish(canned.encode())

        tick = await asyncio.wait_for(sub.on_next_tick(), timeout=2.0)
    finally:
        await router.stop()

    assert tick is not None, "subscription closed before tick arrived"
    assert tick.time == 1_700_000_000_123_456_789


# The core nodes a fleet launch would have stamped onto its time source.
FLEET = ["cn-fleet-sim", "cn-fleet-robot-a", "cn-fleet-robot-b"]


@pytest.mark.asyncio
async def test_sim_time_publisher_reaches_every_participant(tmp_path):
    """One publish lands the tick on every participant's `clock` key, each
    observed by a subscriber scoped the way that machine's daemon and sim-time
    nodes scope theirs. Python twin of `sim_time_publisher_reaches_every_participant`."""
    router = await peppy_testing.EphemeralRouter.start()
    try:
        observer = await MessengerHandle.from_host_port_with_namespace(router.host, router.port)
        standalone_config = (
            StandaloneConfig()
            .with_messaging(router.host, router.port)
            .with_instance_id(CLIENT_INSTANCE)
            .with_use_sim_time(True)
            .with_sim_time_participants(FLEET)
        )
        node_runner = await NodeRunner.new_standalone(
            str(write_standalone_peppy_config(tmp_path)), standalone_config
        )

        publisher = await clock.SimTimePublisher.for_node(node_runner)
        assert publisher is not None, "the launch declared this node the source"
        assert publisher.participants == FLEET

        subscriptions = []
        for core_node in FLEET:
            # Each machine's daemon and sim-time nodes read the `clock` key
            # under that machine's own core-node target; pinned here to the
            # source's producer identity, which is what the fan-out publishes as.
            subscription = await TopicMessenger.subscribe(
                observer,
                core_node,
                "daemon_or_sim_node_on_that_machine",
                SenderTarget.node(core_node, CORE_NODE_TAG),
                "clock",
                ProducerRef(CORE_NODE, CLIENT_INSTANCE),
                QoSProfile.SensorData,
            )
            assert await TopicMessenger.wait_for_subscriber(
                node_runner.messenger(),
                CORE_NODE,
                CLIENT_INSTANCE,
                SenderTarget.node(core_node, CORE_NODE_TAG),
                "clock",
                2.0,
            ), f"no subscriber for `{core_node}` routed within 2s"
            subscriptions.append((core_node, subscription))

        await publisher.publish(42_000_000_000)

        for core_node, subscription in subscriptions:
            message = await asyncio.wait_for(subscription.on_next_message(), timeout=2.0)
            assert message is not None, f"`{core_node}` subscription closed"
            assert clock.ClockTick.decode(message.payload).time == 42_000_000_000
    finally:
        await router.stop()


@pytest.mark.asyncio
async def test_an_undeclared_node_cannot_publish_sim_time(tmp_path):
    """A node the launch never declared its time source gets no publisher,
    and with it no way to drive fleet time."""
    router, node_runner, _server_handle = await start_router_and_runner(tmp_path)
    try:
        assert await clock.SimTimePublisher.for_node(node_runner) is None
    finally:
        await router.stop()


@pytest.mark.asyncio
async def test_a_declared_source_in_wall_mode_gets_no_publisher(tmp_path):
    """The standalone twin of a launch resolving a declared source against a
    wall-serving daemon: the declaration is inert. Python twin of
    `a_declared_source_in_wall_mode_gets_no_publisher`."""
    router = await peppy_testing.EphemeralRouter.start()
    try:
        standalone_config = (
            StandaloneConfig()
            .with_messaging(router.host, router.port)
            .with_instance_id(CLIENT_INSTANCE)
            .with_use_sim_time(False)
            .with_sim_time_participants(FLEET)
        )
        node_runner = await NodeRunner.new_standalone(
            str(write_standalone_peppy_config(tmp_path)), standalone_config
        )
        assert await clock.SimTimePublisher.for_node(node_runner) is None
    finally:
        await router.stop()

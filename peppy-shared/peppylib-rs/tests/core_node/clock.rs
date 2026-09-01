use std::time::Duration;

use config::node::QoSProfile;
use core_node_api::encoding::{ClockResponse, ClockTick};
use core_node_api::{ServiceId, TopicId};
use peppylib::clock;
use peppylib::messaging::{MessengerHandle, ServiceMessenger};
use peppylib::testing::EphemeralRouter;
use tempfile::TempDir;

use super::common::{
    CORE_NODE, SERVER_INSTANCE, publish_once, start_router_and_runner, test_node_target,
    wait_for_topic_subscriber, wait_until_reachable,
};

/// Spins up a single-shot `clock` service listener that returns `response`
/// verbatim. The handler decodes the inbound `ClockRequest` to assert wire
/// shape, even though it ignores the value.
async fn spawn_clock_stub_listener(server: MessengerHandle, response: ClockResponse) {
    let mut endpoint = ServiceMessenger::listen(
        &server,
        CORE_NODE,
        SERVER_INSTANCE,
        test_node_target(CORE_NODE),
        ServiceId::Clock.name(),
    )
    .await
    .expect("listen should succeed");

    tokio::spawn(async move {
        endpoint
            .handle_next_request(|request| async move {
                let payload = request.message().payload();
                let _inbound = core_node_api::encoding::ClockRequest::decode(payload.as_ref())
                    .expect("decode ClockRequest");
                Ok(response.encode().expect("encode ClockResponse"))
            })
            .await
            .expect("handle_next_request should succeed");
    });
}

async fn setup_synchronize_stub(
    response: ClockResponse,
) -> (EphemeralRouter, TempDir, peppylib::runtime::NodeRunner) {
    let (router, temp_dir, node_runner, server) = start_router_and_runner().await;
    spawn_clock_stub_listener(server, response).await;
    wait_until_reachable(node_runner.messenger(), ServiceId::Clock.name()).await;
    (router, temp_dir, node_runner)
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn synchronize_returns_typed_clock_sync() {
    // Canned t1/t2 are far smaller than the live `t0` from SystemTime::now(),
    // so the local clock leads the server and the offset must come out negative.
    let response = ClockResponse::new(0, 2_000_000_000_000, 2_000_000_000_005);

    let (_router, _temp_dir, node_runner) = setup_synchronize_stub(response.clone()).await;

    let sync = clock::synchronize(&node_runner, Some(Duration::from_secs(3)))
        .await
        .expect("synchronize should succeed");

    assert_eq!(sync.raw.server_recv_time, 2_000_000_000_000);
    assert_eq!(sync.raw.server_send_time, 2_000_000_000_005);
    assert!(
        sync.offset_ns < 0,
        "expected local clock to lead canned server time, got offset {} ns",
        sync.offset_ns,
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn subscribe_clock_yields_typed_ticks() {
    let (_router, _temp_dir, node_runner, server) = start_router_and_runner().await;

    // Subscribe via the high-level helper *before* publishing — otherwise the
    // first tick can land before zenoh discovery routes the subscription, and
    // the test races against propagation. With the subscription up first, any
    // tick published after the await point is delivered.
    let mut sub = clock::subscribe(&node_runner)
        .await
        .expect("subscribe_clock should succeed");

    // Deterministically wait until the publisher's session sees the subscription
    // (peer-mode discovery is not instantaneous) instead of guessing a fixed
    // settle delay, so the emit below cannot be dropped before routing.
    wait_for_topic_subscriber(
        &server,
        CORE_NODE,
        SERVER_INSTANCE,
        test_node_target(CORE_NODE),
        TopicId::Clock.name(),
    )
    .await;

    let canned = ClockTick::new(1_700_000_000_123_456_789);
    publish_once(
        &server,
        CORE_NODE,
        SERVER_INSTANCE,
        test_node_target(CORE_NODE),
        TopicId::Clock.name(),
        QoSProfile::SensorData,
        canned.encode().expect("encode tick"),
    )
    .await
    .expect("emit should succeed");

    let tick = tokio::time::timeout(Duration::from_secs(2), sub.on_next_tick())
        .await
        .expect("tick should arrive within 2 s")
        .expect("on_next_tick should not error")
        .expect("subscription should not have closed");

    assert_eq!(tick, canned);
}

/// The core nodes a fleet launch would have stamped onto its time source:
/// the machine the source runs on plus every other machine of the launch.
const FLEET: [&str; 3] = ["cn-fleet-sim", "cn-fleet-robot-a", "cn-fleet-robot-b"];

/// A standalone runner declared the launch's time source for `FLEET`, the
/// daemon-less spelling of `framework: { publishes_sim_time: true }` resolved
/// against a three-machine placement, in the given clock mode.
async fn start_time_source_runner(
    use_sim_time: bool,
) -> (
    EphemeralRouter,
    TempDir,
    peppylib::runtime::NodeRunner,
    MessengerHandle,
) {
    let router = EphemeralRouter::start().await.expect("start zenoh router");
    let observer = router.connect().await.expect("observer handle");
    let temp_dir = TempDir::new().expect("temp dir should be created");
    let peppy_config_path = super::common::write_standalone_peppy_config(&temp_dir);
    let standalone_config = peppylib::runtime::StandaloneConfig::new()
        .with_messaging(router.host(), router.port())
        .with_instance_id(super::common::CLIENT_INSTANCE)
        .with_use_sim_time(use_sim_time)
        .with_sim_time_participants(FLEET);
    let processor =
        peppylib::runtime::Processor::new_standalone(&peppy_config_path, &standalone_config)
            .expect("standalone processor");
    let node_runner = peppylib::runtime::NodeRunner::new(processor)
        .await
        .expect("node runner");
    (router, temp_dir, node_runner, observer)
}

/// One publish lands one tick on every participant's `clock` key, each
/// observed by a subscriber scoped exactly the way that machine's daemon and
/// sim-time nodes scope theirs. The publisher waits for every subscriber to
/// be routed before the tick goes out, so nothing here depends on discovery
/// timing.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn sim_time_publisher_reaches_every_participant() {
    let (_router, _temp_dir, node_runner, observer) = start_time_source_runner(true).await;

    let publisher = clock::SimTimePublisher::for_node(&node_runner)
        .await
        .expect("a declared time source builds its fan-out")
        .expect("the launch declared this node the source");
    assert_eq!(publisher.participants().collect::<Vec<_>>(), FLEET);

    let mut subscriptions = Vec::with_capacity(FLEET.len());
    for core_node in FLEET {
        let subscription = peppylib::messaging::TopicMessenger::subscribe_target_scoped(
            &observer,
            core_node,
            "daemon_or_sim_node_on_that_machine",
            test_node_target(core_node),
            TopicId::Clock.name(),
            QoSProfile::SensorData,
        )
        .await
        .expect("subscribe as a machine of the fleet");
        wait_for_topic_subscriber(
            node_runner.messenger(),
            CORE_NODE,
            super::common::CLIENT_INSTANCE,
            test_node_target(core_node),
            TopicId::Clock.name(),
        )
        .await;
        subscriptions.push((core_node, subscription));
    }

    const SIM_NS: u64 = 42_000_000_000;
    publisher.publish(SIM_NS).await.expect("publish fans out");

    for (core_node, mut subscription) in subscriptions {
        let message = tokio::time::timeout(Duration::from_secs(2), subscription.on_next_message())
            .await
            .unwrap_or_else(|_| panic!("`{core_node}` received no tick within 2 s"))
            .unwrap_or_else(|| panic!("`{core_node}` subscription closed"));
        let tick = ClockTick::decode(message.payload_bytes().as_ref()).expect("decode tick");
        assert_eq!(
            tick.time(),
            SIM_NS,
            "`{core_node}` read a different instant"
        );
    }
}

/// A node the launch never declared its time source gets no publisher, and
/// with it no way to drive fleet time, whatever clock mode it runs in.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_undeclared_node_cannot_publish_sim_time() {
    let (_router, _temp_dir, node_runner, _server) = start_router_and_runner().await;

    let publisher = clock::SimTimePublisher::for_node(&node_runner)
        .await
        .expect("asking is not an error");
    assert!(
        publisher.is_none(),
        "an undeclared node must get no publisher"
    );
}

/// The standalone twin of a launch resolving a declared source against a
/// wall-serving daemon: the declaration is inert, so a wall-mode node gets
/// no publisher even with participants configured.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_declared_source_in_wall_mode_gets_no_publisher() {
    let (_router, _temp_dir, node_runner, _observer) = start_time_source_runner(false).await;

    let publisher = clock::SimTimePublisher::for_node(&node_runner)
        .await
        .expect("asking is not an error");
    assert!(
        publisher.is_none(),
        "a wall-mode declaration must resolve to no publisher"
    );
}

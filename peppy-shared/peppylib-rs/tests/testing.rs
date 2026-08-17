//! Suite for `peppylib::testing` — the node-invariant cores behind generated
//! per-node mock/fixtures code. Mock semantics (scripted vs. manual service
//! responses, deterministic action producer-loss, lazy first-publish
//! readiness, the harness barrier and teardown convergence) are proven HERE,
//! once, before any codegen consumes them.

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use config::node::QoSProfile;
use peppylib::PeppyError;
use peppylib::messaging::{
    ActionMessenger, NonEmptyPayload, ProducerRef, SenderTarget, ServiceMessenger, ServiceTarget,
    TopicMessenger,
};
use peppylib::runtime::StandaloneConfig;
use peppylib::testing::{
    EphemeralRouter, HarnessCore, MockActionServerCore, MockServiceCore, PublisherReadiness,
    TestTopicPublisher, wait_action_reachable, wait_service_reachable,
};
use peppylib::types::Payload;
use tempfile::TempDir;

const MOCK_CORE: &str = "mock_core";
const MOCK_INSTANCE: &str = "mock_1";
const CALLER_CORE: &str = "caller_core";
const CALLER_INSTANCE: &str = "caller_1";

fn node_target(name: &str) -> SenderTarget {
    SenderTarget::node(name, "v1").expect("test node target")
}

/// Minimal parameterless node manifest for the harness tests.
fn write_peppy_config(dir: &TempDir) -> PathBuf {
    let path = dir.path().join("peppy.json5");
    std::fs::write(
        &path,
        r#"{
            peppy_schema: "node/v1",
            manifest: { name: "test_node", tag: "v1" },
            execution: { language: "rust", run_cmd: ["./target/debug/test_node"] },
        }"#,
    )
    .expect("peppy config should be written");
    path
}

#[derive(Debug, serde::Deserialize, schemars::JsonSchema)]
struct EmptyParameters {}

/// Scripted responses are served automatically in FIFO order; unscripted
/// requests park for `next_request`; every request lands in the capture
/// buffer in arrival order.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn mock_service_core_scripted_then_manual() {
    let router = EphemeralRouter::start().await.expect("start router");
    let mock_handle = router.connect().await.expect("mock session");
    let caller_handle = router.connect().await.expect("caller session");

    let mock = MockServiceCore::listen(
        &mock_handle,
        MOCK_CORE,
        MOCK_INSTANCE,
        node_target("dep_node"),
        "get_info",
    )
    .await
    .expect("mock service should listen");

    // Scripted path: the response is enqueued before the node calls.
    mock.enqueue_response(Payload::from_static(b"scripted-response"));
    let producer = ProducerRef::new(MOCK_CORE, MOCK_INSTANCE);
    // Cold-start gate: the caller session is fresh, so its first query can
    // race gossip discovery of the mock's queryable.
    wait_service_reachable(
        &caller_handle,
        CALLER_CORE,
        CALLER_INSTANCE,
        node_target("dep_node"),
        "get_info",
        &producer,
        Duration::from_secs(5),
    )
    .await
    .expect("mock service should become reachable");
    let scripted = ServiceMessenger::poll(
        &caller_handle,
        CALLER_CORE,
        CALLER_INSTANCE,
        node_target("dep_node"),
        "get_info",
        ServiceTarget::Producer(&producer),
        Payload::from_static(b"request-1"),
        Duration::from_secs(5),
    )
    .await
    .expect("scripted poll should succeed");
    assert_eq!(scripted.payload().as_ref(), b"scripted-response");

    // Manual path: the caller's poll parks until the test receives the
    // request, asserts on it, and responds.
    let caller_for_task = caller_handle.clone();
    let manual_poll = tokio::spawn(async move {
        let producer = ProducerRef::new(MOCK_CORE, MOCK_INSTANCE);
        ServiceMessenger::poll(
            &caller_for_task,
            CALLER_CORE,
            CALLER_INSTANCE,
            node_target("dep_node"),
            "get_info",
            ServiceTarget::Producer(&producer),
            Payload::from_static(b"request-2"),
            Duration::from_secs(5),
        )
        .await
    });

    let (context, responder) = mock
        .next_request(Duration::from_secs(5))
        .await
        .expect("manual request should arrive");
    assert_eq!(context.message().payload().as_ref(), b"request-2");
    assert_eq!(context.message().core_node(), CALLER_CORE);
    assert_eq!(context.message().instance_id(), CALLER_INSTANCE);
    responder
        .respond(Payload::from_static(b"manual-response"))
        .await
        .expect("manual respond should succeed");

    let manual = manual_poll
        .await
        .expect("poll task should not panic")
        .expect("manual poll should succeed");
    assert_eq!(manual.payload().as_ref(), b"manual-response");

    // Both requests were captured, in order, scripted and manual alike.
    let captured = mock.captured();
    assert_eq!(captured.len(), 2);
    assert_eq!(captured[0].message.payload().as_ref(), b"request-1");
    assert_eq!(captured[1].message.payload().as_ref(), b"request-2");
    assert_eq!(captured[0].message.core_node(), CALLER_CORE);

    drop(mock);
    router.shutdown().await.expect("router shutdown");
}

/// The deterministic producer-loss contract: `stop()` on a mock action server
/// with a live, user-held goal context must surface to the consumer as the
/// typed `ActionFeedbackProducerGone` — never as a clean feedback close and
/// never as a hang — because the disarmed context suppresses the drop-time
/// sentinel that would otherwise race the liveliness latch.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn mock_action_stop_yields_producer_gone_deterministically() {
    let router = EphemeralRouter::start().await.expect("start router");
    let mock_handle = router.connect().await.expect("mock session");
    let client_handle = router.connect().await.expect("client session");

    let mut mock = MockActionServerCore::expose(
        &mock_handle,
        MOCK_CORE,
        MOCK_INSTANCE,
        node_target("arm_node"),
        "move_arm",
        true,
    )
    .await
    .expect("mock action should expose");

    // Cold-start gate: the client session is fresh, so its first goal query
    // can race gossip discovery of the mock's queryable.
    wait_action_reachable(
        &client_handle,
        CALLER_CORE,
        CALLER_INSTANCE,
        node_target("arm_node"),
        "move_arm",
        &ProducerRef::new(MOCK_CORE, MOCK_INSTANCE),
        Duration::from_secs(5),
    )
    .await
    .expect("mock action should become reachable");

    // `send_goal` resolves only once the server answers admission, so it must
    // run concurrently with the mock's `next_goal` → `accept` below — exactly
    // the shape a node's consumed-action call has against a live server.
    let client_for_task = client_handle.clone();
    let goal_task = tokio::spawn(async move {
        ActionMessenger::send_goal(
            &client_for_task,
            CALLER_CORE,
            CALLER_INSTANCE,
            node_target("arm_node"),
            "move_arm",
            Some(&ProducerRef::new(MOCK_CORE, MOCK_INSTANCE)),
            Payload::from_static(b"goal"),
            QoSProfile::Reliable,
            Duration::from_secs(5),
        )
        .await
    });

    let pending = mock
        .next_goal(Duration::from_secs(5))
        .await
        .expect("goal should arrive");
    assert_eq!(pending.request_bytes(), b"goal");
    let context = pending
        .accept(Payload::from_static(b"accepted"))
        .await
        .expect("accept should succeed");
    let mut goal = goal_task
        .await
        .expect("send_goal task should not panic")
        .expect("send goal");
    context
        .publish_feedback(
            NonEmptyPayload::try_new(Payload::from_static(b"working"))
                .expect("feedback payload is non-empty"),
        )
        .await
        .expect("feedback should publish");

    let first = tokio::time::timeout(Duration::from_secs(5), goal.on_next_feedback())
        .await
        .expect("live goal must deliver feedback")
        .expect("feedback should arrive before the mock stops");
    assert_eq!(first.payload().as_ref(), b"working");

    // Stop the mock mid-goal with the context still held (as a test's MockGoal
    // handle would be) and tear down its session — the producer-loss shape.
    mock.stop();
    drop(mock_handle);

    let gone = tokio::time::timeout(Duration::from_secs(10), goal.on_next_feedback())
        .await
        .expect("producer loss must unblock the feedback drain, not hang");
    match gone {
        Err(PeppyError::ActionFeedbackProducerGone { action_name, .. }) => {
            assert_eq!(action_name, "move_arm");
        }
        other => panic!("expected ActionFeedbackProducerGone, got {other:?}"),
    }

    // Dropping the disarmed context afterwards is inert: no panic, no late
    // sentinel (nothing to receive it anyway — the session is gone).
    drop(context);

    router.shutdown().await.expect("router shutdown");
}

/// The lazy first-publish barrier: with the subscriber already up, the very
/// first publish must be delivered (no sleeps anywhere); with no subscriber,
/// the publish fails loudly instead of dropping silently.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn test_topic_publisher_first_publish_is_delivered() {
    let router = EphemeralRouter::start().await.expect("start router");
    let sub_handle = router.connect().await.expect("subscriber session");
    let pub_handle = router.connect().await.expect("publisher session");

    let producer = ProducerRef::new(MOCK_CORE, MOCK_INSTANCE);
    let mut subscription = TopicMessenger::subscribe(
        &sub_handle,
        CALLER_CORE,
        CALLER_INSTANCE,
        node_target("camera"),
        "video_stream",
        &producer,
        QoSProfile::Reliable,
    )
    .await
    .expect("subscribe should succeed");

    let publisher = TestTopicPublisher::declare(
        &pub_handle,
        MOCK_CORE,
        MOCK_INSTANCE,
        node_target("camera"),
        None,
        "video_stream",
        QoSProfile::Reliable,
    )
    .await
    .expect("declare should succeed");

    publisher
        .publish(Payload::from_static(b"frame-1"))
        .await
        .expect("first publish must route");

    let received = tokio::time::timeout(Duration::from_secs(5), subscription.on_next_message())
        .await
        .expect("first publish must be delivered")
        .expect("subscription should be open");
    assert_eq!(received.payload().as_ref(), b"frame-1");

    // No subscriber: the publish errors after the (shortened) wait instead of
    // vanishing.
    let orphan = TestTopicPublisher::declare(
        &pub_handle,
        MOCK_CORE,
        MOCK_INSTANCE,
        node_target("camera"),
        None,
        "nobody_listens",
        QoSProfile::Reliable,
    )
    .await
    .expect("declare should succeed")
    .with_readiness_timeout(Duration::from_millis(250));
    let err = orphan
        .publish(Payload::from_static(b"lost"))
        .await
        .expect_err("publishing with no subscriber must fail loudly");
    assert!(
        err.to_string().contains("nobody_listens"),
        "error should name the topic, got: {err}"
    );

    router.shutdown().await.expect("router shutdown");
}

/// Full harness lifecycle: readiness barrier → setup spawn → the node's very
/// first publish is observed → shutdown convergence runs the registered
/// shutdown hooks.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn harness_core_boots_node_observes_first_publish_and_converges() {
    let router = EphemeralRouter::start().await.expect("start router");
    let observer_handle = router.connect().await.expect("observer session");

    let temp_dir = TempDir::new().expect("temp dir");
    let peppy_config_path = write_peppy_config(&temp_dir);
    let instance_id = "harness_test_instance";

    // The harness-side observation subscription exists before the node does;
    // the barrier below is what guarantees the node's session has discovered
    // it before setup's first publish.
    let node_producer = ProducerRef::new("standalone-core", instance_id);
    let mut status_sub = TopicMessenger::subscribe(
        &observer_handle,
        CALLER_CORE,
        CALLER_INSTANCE,
        node_target("test_node"),
        "status",
        &node_producer,
        QoSProfile::Reliable,
    )
    .await
    .expect("observation subscribe should succeed");

    let standalone_config = StandaloneConfig::new()
        .with_messaging(router.host(), router.port())
        .with_instance_id(instance_id);
    let readiness = [PublisherReadiness {
        target: node_target("test_node"),
        link_id: None,
        topic: "status".to_string(),
    }];

    // A mock service declared before the node exists: the reachability half
    // of the barrier must make it visible to the node's session pre-setup.
    let mock_handle = router.connect().await.expect("mock session");
    let mock_service = peppylib::testing::MockServiceCore::listen(
        &mock_handle,
        "mock-core",
        "mock-1",
        node_target("dep_node"),
        "get_info",
    )
    .await
    .expect("mock service listens");
    let service_readiness = [peppylib::testing::ServiceReadiness {
        target: node_target("dep_node"),
        name: "get_info".to_string(),
        producer: peppylib::messaging::ProducerRef::new("mock-core", "mock-1"),
        kind: peppylib::testing::ServiceReadinessKind::Service,
    }];

    let hook_ran = Arc::new(AtomicBool::new(false));
    let hook_flag = Arc::clone(&hook_ran);
    let harness = HarnessCore::start::<EmptyParameters, _, _>(
        peppy_config_path,
        standalone_config,
        &readiness,
        &service_readiness,
        move |_params, node_runner| async move {
            let processor = node_runner.processor();
            let publisher = TopicMessenger::declare_publisher(
                node_runner.messenger(),
                processor.bound_core_node(),
                processor.bound_instance_id(),
                SenderTarget::node(processor.node_name(), processor.node_tag())?,
                None,
                "status",
                QoSProfile::Reliable,
            )
            .await?;
            // First publish, immediately: only the pre-setup barrier makes
            // this deliverable.
            publisher.publish(Payload::from_static(b"alive")).await?;
            node_runner.on_shutdown(async move {
                hook_flag.store(true, Ordering::SeqCst);
            });
            Ok(())
        },
    )
    .await
    .expect("harness should start");

    assert_eq!(harness.instance_id(), instance_id);
    assert_eq!(harness.bound_core_node(), "standalone-core");

    let first = tokio::time::timeout(Duration::from_secs(5), status_sub.on_next_message())
        .await
        .expect("the node's first publish must be observed")
        .expect("subscription should be open");
    assert_eq!(first.payload().as_ref(), b"alive");

    harness.shutdown().await.expect("harness shutdown");
    assert!(
        hook_ran.load(Ordering::SeqCst),
        "shutdown() must run the node's registered shutdown hooks"
    );

    drop(mock_service);
    router.shutdown().await.expect("router shutdown");
}

/// A setup error is not swallowed by teardown: `shutdown()` propagates it so
/// the test fails even when its assertions never noticed the node was broken.
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn harness_core_shutdown_propagates_setup_error() {
    let router = EphemeralRouter::start().await.expect("start router");
    let temp_dir = TempDir::new().expect("temp dir");
    let peppy_config_path = write_peppy_config(&temp_dir);

    let standalone_config = StandaloneConfig::new()
        .with_messaging(router.host(), router.port())
        .with_instance_id("failing_setup_instance");

    let harness = HarnessCore::start::<EmptyParameters, _, _>(
        peppy_config_path,
        standalone_config,
        &[],
        &[],
        |_params, _node_runner| async move {
            Err(PeppyError::Io(std::io::Error::other("setup boom")))
        },
    )
    .await
    .expect("harness start itself should succeed");

    let err = harness
        .shutdown()
        .await
        .expect_err("shutdown must propagate the setup error");
    assert!(
        err.to_string().contains("setup boom"),
        "expected the setup error, got: {err}"
    );

    router.shutdown().await.expect("router shutdown");
}

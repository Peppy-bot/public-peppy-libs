//! Invariant: a service reply that lands after the caller's deadline is
//! delivered and consumed quietly, as long as it lands inside
//! `LATE_REPLY_GRACE`.
//!
//! The caller's deadline and the query's wire lifetime are separate bounds
//! (see `service_query_lifetime` in `src/types.rs`). The deadline belongs to
//! the caller, which stops waiting and reports the outcome. The wire lifetime
//! belongs to the transport and outlives the deadline by the grace, so a
//! producer that answers late (it stalled for a few seconds instead of dying)
//! still completes the query: zenoh neither times it out at the deadline nor
//! rejects the reply as unknown, and no hop logs a warning for a call that
//! merely ran late. The daemon's per-node health probes are the call site
//! that hits this on every host hiccup.
//!
//! This test runs the scenario against a real zenohd process. The producer
//! holds its reply until the consumer confirms the deadline has passed, so
//! the ordering is enforced by explicit hand-offs rather than by racing the
//! host. It asserts that the late reply still arrives on the reply stream and
//! that zenoh's routing and session layers and the adapter itself logged zero
//! WARN or ERROR events from the query onwards.

#![cfg(feature = "build_zenoh")]

mod common;
use common::{RECV_TIMEOUT, ZENOH_SERIAL, test_node_target};

use bytes::Bytes;
use pmi::{
    MessengerBackend, Payload, ServiceKind, ServiceQueryKind, ServiceWireReceiver,
    ServiceWireSender, ZenohAdapter,
};
use std::sync::{Mutex, OnceLock};
use std::time::Duration;
use tokio::sync::oneshot;
use tracing::field::{Field, Visit};
use tracing::{Event, Level, Subscriber};
use tracing_subscriber::layer::{Context, Layer, SubscriberExt};
use tracing_subscriber::registry::Registry;

/// WARN and ERROR events logged by zenoh (`zenoh::*`) or by the adapter
/// (`pmi::adapters::zenoh`), rendered as `LEVEL target: message`. The global
/// subscriber installed by [`install_subscriber_once`] appends from arbitrary
/// zenoh worker threads; the test clears and reads it under the
/// `ZENOH_SERIAL` mutex so it owns a clean window.
static NOISY_EVENTS: Mutex<Vec<String>> = Mutex::new(Vec::new());

struct MessageVisitor(String);

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = format!("{value:?}");
        }
    }
}

struct RecordingLayer;

impl<S: Subscriber> Layer<S> for RecordingLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let metadata = event.metadata();
        let level = *metadata.level();
        let target = metadata.target();
        let noisy_level = level == Level::WARN || level == Level::ERROR;
        let watched_target =
            target.starts_with("zenoh::") || target.starts_with("pmi::adapters::zenoh");
        if !noisy_level || !watched_target {
            return;
        }
        let mut visitor = MessageVisitor(String::new());
        event.record(&mut visitor);
        NOISY_EVENTS
            .lock()
            .expect("recorder mutex is never poisoned")
            .push(format!("{level} {target}: {}", visitor.0));
    }
}

/// Tracing's global default subscriber can only be set once per process, so
/// it is installed on first test entry and serves every test in this binary.
fn install_subscriber_once() {
    static INIT: OnceLock<()> = OnceLock::new();
    INIT.get_or_init(|| {
        let subscriber = Registry::default().with(RecordingLayer);
        tracing::subscriber::set_global_default(subscriber)
            .expect("no other tracing subscriber should be installed in this test binary");
    });
}

fn service_receiver() -> ServiceWireReceiver {
    ServiceWireReceiver::new(
        "server_core",
        "server_inst",
        test_node_target("robot_arm"),
        "ping",
        ServiceKind::Service,
    )
    .expect("valid wire fields")
}

fn service_sender() -> ServiceWireSender {
    ServiceWireSender::new(
        "client_core",
        "client_inst",
        None,
        test_node_target("robot_arm"),
        "ping",
        ServiceKind::Service,
    )
    .expect("valid wire fields")
}

/// The caller's deadline. Short so the test spends little time letting it
/// pass; it is a fraction of `LATE_REPLY_GRACE`, so the late reply below
/// lands well inside the query's wire lifetime.
const DEADLINE: Duration = Duration::from_millis(100);

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_reply_after_the_deadline_arrives_without_warnings() {
    let _lock = ZENOH_SERIAL.lock().await;
    install_subscriber_once();

    let mut instance = ZenohAdapter::start_router_ephemeral("127.0.0.1", None)
        .await
        .expect("start zenohd");
    instance.messenger().start_session().await.unwrap();

    let queryable = instance
        .messenger()
        .listen_service(&service_receiver())
        .await
        .unwrap();
    // Let the queryable propagate through zenoh's discovery before the get
    // goes out. Same delay the other integration tests use.
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Producer: report the query's arrival, then hold the reply until the
    // consumer says the deadline has passed.
    let (query_landed_tx, query_landed_rx) = oneshot::channel::<()>();
    let (release_reply_tx, release_reply_rx) = oneshot::channel::<()>();
    let producer = tokio::spawn(async move {
        let incoming = queryable
            .rx
            .recv_async()
            .await
            .expect("producer receives the query");
        query_landed_tx
            .send(())
            .expect("consumer waits for the query to land");
        release_reply_rx.await.expect("consumer releases the reply");
        incoming
            .token
            .respond_response(Payload::from_bytes(Bytes::from_static(b"late_reply")))
            .await
            .expect("producer respond");
    });

    // Only events from the query onwards count; router and session start-up
    // chatter is not what this test is about.
    NOISY_EVENTS
        .lock()
        .expect("recorder mutex is never poisoned")
        .clear();

    let mut reply_stream = instance
        .messenger()
        .call_service(
            &service_sender(),
            Payload::from_bytes(Bytes::from_static(b"ping?")),
            ServiceQueryKind::UserRequest,
            Some(DEADLINE),
        )
        .await
        .unwrap();

    query_landed_rx
        .await
        .expect("producer task reports the query");
    // The deadline is measured from the get, which happened before the query
    // landed at the producer, so sleeping the whole deadline from here
    // guarantees it has passed. Tokio's sleep never wakes early, and nothing
    // below depends on how quickly the host delivered the query.
    tokio::time::sleep(DEADLINE).await;
    release_reply_tx
        .send(())
        .expect("producer waits for the release");

    let reply = tokio::time::timeout(RECV_TIMEOUT, reply_stream.rx.recv())
        .await
        .expect("late reply arrives within the receive budget")
        .expect("reply stream stays open past the caller's deadline");
    assert_eq!(
        reply.message().payload(),
        &Bytes::from_static(b"late_reply"),
        "the reply delivered after the deadline is the producer's"
    );
    producer.await.expect("producer task did not panic");

    let noisy_events = NOISY_EVENTS
        .lock()
        .expect("recorder mutex is never poisoned")
        .clone();
    assert!(
        noisy_events.is_empty(),
        "expected zero WARN/ERROR events from zenoh or the adapter for a reply \
         that landed after the caller's deadline but inside LATE_REPLY_GRACE, \
         got:\n{}",
        noisy_events.join("\n")
    );
}

//! Pairing runtime semantics over the mock adapter: unpaired slots are
//! silent, pairing pins the wire triple live, re-pins swap without duplicate
//! or stale delivery, clears silence the slot again, and the `peer_update`
//! service applies daemon deliveries end to end.

mod common;

use common::get_client_server;
use config::node::QoSProfile;
use peppylib::messaging::{
    MessengerHandle, PEER_UPDATE_SERVICE, PeerPin, PeerPinState, ProducerRef, SenderTarget,
    ServiceMessenger, ServiceTarget, TopicMessenger, TopicPublisher,
};
use peppylib::runtime::{PeerSubscription, subscribe_peer_with_watch};
use peppylib::types::Payload;
use std::time::Duration;
use tokio::sync::watch;

const CORE: &str = "test_core_node";
const PAIRING_NAME: &str = "arm_link";
const PAIRING_TAG: &str = "v1";
const TOPIC: &str = "joint_states";
/// The consumer's own slot link_id is irrelevant to the wire (only the
/// peer's slot link_id appears in its publish keyexprs); these tests play
/// the controller consuming the arm role's `joint_states`.
const ARM_SLOT_LINK_ID: &str = "controller";
const CONSUMER_INSTANCE: &str = "ctrl_1";

fn pairing_target() -> SenderTarget {
    SenderTarget::pairing(PAIRING_NAME, PAIRING_TAG).expect("test pairing target")
}

fn pin_to(instance_id: &str) -> PeerPin {
    PeerPin {
        producer: ProducerRef::new(CORE, instance_id),
        peer_link_id: ARM_SLOT_LINK_ID.to_string(),
    }
}

/// Declares a slot-scoped pairing publisher for a peer instance: the wire
/// link_id segment carries the peer's OWN slot link_id.
async fn declare_peer_publisher(handle: &MessengerHandle, instance_id: &str) -> TopicPublisher {
    TopicMessenger::declare_publisher(
        handle,
        CORE,
        instance_id,
        pairing_target(),
        Some(ARM_SLOT_LINK_ID),
        TOPIC,
        QoSProfile::Reliable,
    )
    .await
    .expect("peer publisher should declare")
}

/// Consumer-side pairing subscription driven by a hand-held watch channel
/// (standing in for the processor-owned slot the daemon mutates).
fn subscribe(
    handle: &MessengerHandle,
    watch_rx: watch::Receiver<PeerPinState>,
) -> PeerSubscription {
    subscribe_peer_with_watch(
        handle.clone(),
        CORE.to_string(),
        CONSUMER_INSTANCE.to_string(),
        watch_rx,
        pairing_target(),
        TOPIC.to_string(),
        QoSProfile::Reliable,
    )
}

/// Waits until the consumer's current wire subscription (pinned to
/// `peer_instance`) is visible to the publisher's session. Pairing wire
/// subs are declared by the forwarding task asynchronously after a pin
/// update, so tests must synchronize before publishing.
async fn wait_for_peer_wire_sub(handle: &MessengerHandle, peer_instance: &str) {
    let matched = TopicMessenger::wait_for_subscriber_with_link_id(
        handle,
        CORE,
        peer_instance,
        pairing_target(),
        Some(ARM_SLOT_LINK_ID),
        TOPIC,
        Duration::from_secs(2),
    )
    .await
    .expect("wait_for_subscriber should not error");
    assert!(matched, "peer wire subscription did not appear within 2s");
}

/// Inverse of [`wait_for_peer_wire_sub`]: waits until the consumer's wire
/// subscription pinned to `peer_instance` has disappeared from the
/// publisher's session. The forwarding task drops the old wire sub
/// asynchronously after a clear, so tests must gate on the actual teardown
/// before probing for silence. A probe window returning `false` means every
/// poll inside it saw no matching subscriber — i.e. the drop has landed;
/// while the sub still exists the probe returns `true` immediately and we
/// retry until the deadline.
async fn wait_for_peer_wire_sub_gone(handle: &MessengerHandle, peer_instance: &str) {
    let deadline = tokio::time::Instant::now() + Duration::from_secs(2);
    loop {
        let matched = TopicMessenger::wait_for_subscriber_with_link_id(
            handle,
            CORE,
            peer_instance,
            pairing_target(),
            Some(ARM_SLOT_LINK_ID),
            TOPIC,
            Duration::from_millis(25),
        )
        .await
        .expect("wait_for_subscriber should not error");
        if !matched {
            return;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "peer wire subscription did not disappear within 2s"
        );
    }
}

async fn expect_message(subscription: &mut PeerSubscription, expected_payload: &[u8]) {
    let message = tokio::time::timeout(Duration::from_secs(2), subscription.on_next_message())
        .await
        .expect("should receive a message within 2s")
        .expect("subscription should not close");
    assert_eq!(&*message.payload_bytes(), expected_payload);
}

async fn expect_silence(subscription: &mut PeerSubscription) {
    let outcome =
        tokio::time::timeout(Duration::from_millis(300), subscription.on_next_message()).await;
    assert!(
        outcome.is_err(),
        "expected no delivery, got: {:?}",
        outcome.unwrap().map(|m| m.payload())
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn unpaired_slot_receives_nothing() {
    let (client, shared) = get_client_server().await;
    let peer_handle = MessengerHandle::from_shared(shared);

    let (_tx, watch_rx) = watch::channel(PeerPinState::unpaired());
    let mut subscription = subscribe(&client.caller_handle, watch_rx);

    // The peer publishes before any pair exists: publish-unpaired is a legal
    // no-op on the publisher side and MUST NOT reach the unpaired consumer.
    let publisher = declare_peer_publisher(&peer_handle, "arm_1").await;
    publisher
        .publish(Payload::from_static(b"pre-pairing"))
        .await
        .expect("publish while unpaired is a legal no-op");

    expect_silence(&mut subscription).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn live_pair_starts_delivery_without_resubscribe() {
    let (client, shared) = get_client_server().await;
    let peer_handle = MessengerHandle::from_shared(shared);

    let (tx, watch_rx) = watch::channel(PeerPinState::unpaired());
    let mut subscription = subscribe(&client.caller_handle, watch_rx);
    let publisher = declare_peer_publisher(&peer_handle, "arm_1").await;

    // Pair live — the subscription object predates the pair (the lazy story).
    tx.send(PeerPinState {
        sequence: 1,
        pin: Some(pin_to("arm_1")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_1").await;

    publisher
        .publish(Payload::from_static(b"post-pairing"))
        .await
        .expect("publish");
    expect_message(&mut subscription, b"post-pairing").await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn foreign_identity_on_same_keyexpr_shape_is_never_delivered() {
    // Lock-in proof: a third contract-shape node publishing on the same
    // pairing (name, tag, slot link_id, topic) with a different instance
    // identity must not reach a consumer paired to someone else.
    let (client, shared) = get_client_server().await;
    let peer_handle = MessengerHandle::from_shared(std::sync::Arc::clone(&shared));
    let intruder_handle = MessengerHandle::from_shared(shared);

    let (tx, watch_rx) = watch::channel(PeerPinState::unpaired());
    let mut subscription = subscribe(&client.caller_handle, watch_rx);

    tx.send(PeerPinState {
        sequence: 1,
        pin: Some(pin_to("arm_1")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_1").await;

    let intruder = declare_peer_publisher(&intruder_handle, "intruder_1").await;
    intruder
        .publish(Payload::from_static(b"injected"))
        .await
        .expect("publish");
    expect_silence(&mut subscription).await;

    // The paired peer still flows.
    let publisher = declare_peer_publisher(&peer_handle, "arm_1").await;
    publisher
        .publish(Payload::from_static(b"legit"))
        .await
        .expect("publish");
    expect_message(&mut subscription, b"legit").await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn repin_swaps_to_the_new_peer_without_stale_or_duplicate_delivery() {
    let (client, shared) = get_client_server().await;
    let peer_handle = MessengerHandle::from_shared(shared);

    let (tx, watch_rx) = watch::channel(PeerPinState::unpaired());
    let mut subscription = subscribe(&client.caller_handle, watch_rx);

    let old_peer = declare_peer_publisher(&peer_handle, "arm_1").await;
    let new_peer = declare_peer_publisher(&peer_handle, "arm_2").await;

    tx.send(PeerPinState {
        sequence: 1,
        pin: Some(pin_to("arm_1")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_1").await;
    old_peer
        .publish(Payload::from_static(b"from arm_1"))
        .await
        .expect("publish");
    expect_message(&mut subscription, b"from arm_1").await;

    // Re-pin to arm_2 (failover: replacement booted with --pair).
    tx.send(PeerPinState {
        sequence: 2,
        pin: Some(pin_to("arm_2")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_2").await;

    // The old peer keeps publishing after the swap; nothing may surface.
    old_peer
        .publish(Payload::from_static(b"stale from arm_1"))
        .await
        .expect("publish");
    new_peer
        .publish(Payload::from_static(b"from arm_2"))
        .await
        .expect("publish");

    expect_message(&mut subscription, b"from arm_2").await;
    expect_silence(&mut subscription).await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn clear_silences_the_slot_until_repaired() {
    let (client, shared) = get_client_server().await;
    let peer_handle = MessengerHandle::from_shared(shared);

    let (tx, watch_rx) = watch::channel(PeerPinState::unpaired());
    let mut subscription = subscribe(&client.caller_handle, watch_rx);
    let publisher = declare_peer_publisher(&peer_handle, "arm_1").await;

    tx.send(PeerPinState {
        sequence: 1,
        pin: Some(pin_to("arm_1")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_1").await;
    publisher
        .publish(Payload::from_static(b"while paired"))
        .await
        .expect("publish");
    expect_message(&mut subscription, b"while paired").await;

    // The daemon clears the pair (peer death / node stop).
    tx.send(PeerPinState {
        sequence: 2,
        pin: None,
    })
    .expect("watch send");
    // Deterministic sync point for the drop: gate on the wire subscription
    // actually disappearing before probing for silence.
    wait_for_peer_wire_sub_gone(&peer_handle, "arm_1").await;
    publisher
        .publish(Payload::from_static(b"after clear"))
        .await
        .expect("publish");
    expect_silence(&mut subscription).await;

    // Re-pair resumes the stream (streams are live, not mailboxes: the
    // message published while cleared stays lost).
    tx.send(PeerPinState {
        sequence: 3,
        pin: Some(pin_to("arm_1")),
    })
    .expect("watch send");
    wait_for_peer_wire_sub(&peer_handle, "arm_1").await;
    publisher
        .publish(Payload::from_static(b"after re-pair"))
        .await
        .expect("publish");
    expect_message(&mut subscription, b"after re-pair").await;
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn peer_update_service_applies_daemon_deliveries_end_to_end() {
    use peppylib::encoding::peer_update::{PeerUpdateRequest, PeerUpdateResponse};
    use peppylib::services::peer_update::listen_for_peer_update;
    use std::collections::BTreeMap;
    use std::sync::Arc;

    let (client, shared) = get_client_server().await;
    let daemon_handle = MessengerHandle::from_shared(shared);

    // The "node": one declared pairing slot 'arm', service listening.
    let (slot_tx, slot_rx) = watch::channel(PeerPinState::unpaired());
    let slots: Arc<BTreeMap<String, watch::Sender<PeerPinState>>> =
        Arc::new(BTreeMap::from([("arm".to_string(), slot_tx)]));
    let node_identity = SenderTarget::node("arm_controller", "v1").expect("node target");
    let _listener = listen_for_peer_update(
        &client.caller_handle,
        CORE,
        CONSUMER_INSTANCE,
        node_identity.clone(),
        slots,
    )
    .await
    .expect("peer_update listener should register");

    // The "daemon" delivers a pair to the node's slot.
    let node_ref = ProducerRef::new(CORE, CONSUMER_INSTANCE);
    let request = PeerUpdateRequest {
        link_id: "arm".to_string(),
        sequence: 7,
        pin: Some(PeerPin {
            producer: ProducerRef::new(CORE, "arm_1"),
            peer_link_id: "controller".to_string(),
        }),
    };
    let reply = ServiceMessenger::poll(
        &daemon_handle,
        CORE,
        "daemon",
        node_identity.clone(),
        PEER_UPDATE_SERVICE,
        ServiceTarget::Producer(&node_ref),
        request.encode().expect("encode"),
        Duration::from_secs(2),
    )
    .await
    .expect("peer_update delivery should succeed");
    let response = PeerUpdateResponse::decode(&reply.payload_bytes()).expect("decode response");
    assert!(response.accepted, "delivery rejected: {}", response.message);
    assert_eq!(slot_rx.borrow().pin, request.pin);
    assert_eq!(slot_rx.borrow().sequence, 7);

    // A delayed stale retry must be reported stale and change nothing.
    let stale = PeerUpdateRequest {
        link_id: "arm".to_string(),
        sequence: 6,
        pin: None,
    };
    let reply = ServiceMessenger::poll(
        &daemon_handle,
        CORE,
        "daemon",
        node_identity.clone(),
        PEER_UPDATE_SERVICE,
        ServiceTarget::Producer(&node_ref),
        stale.encode().expect("encode"),
        Duration::from_secs(2),
    )
    .await
    .expect("stale delivery still gets a reply");
    let response = PeerUpdateResponse::decode(&reply.payload_bytes()).expect("decode response");
    assert!(!response.accepted);
    assert!(response.stale_sequence);
    assert_eq!(slot_rx.borrow().sequence, 7, "stale must not roll back");

    // A caller stamped with a foreign core_node is not this node's daemon:
    // it must be rejected before touching slot state, even with a fresher
    // sequence.
    let foreign = PeerUpdateRequest {
        link_id: "arm".to_string(),
        sequence: 99,
        pin: None,
    };
    let reply = ServiceMessenger::poll(
        &daemon_handle,
        "foreign_core",
        "daemon",
        node_identity,
        PEER_UPDATE_SERVICE,
        ServiceTarget::Producer(&node_ref),
        foreign.encode().expect("encode"),
        Duration::from_secs(2),
    )
    .await
    .expect("foreign delivery still gets a reply");
    let response = PeerUpdateResponse::decode(&reply.payload_bytes()).expect("decode response");
    assert!(!response.accepted, "foreign core_node must be rejected");
    assert!(!response.stale_sequence);
    assert_eq!(
        slot_rx.borrow().sequence,
        7,
        "foreign caller must not mutate the slot"
    );
    assert!(
        slot_rx.borrow().pin.is_some(),
        "foreign clear must not land"
    );
}

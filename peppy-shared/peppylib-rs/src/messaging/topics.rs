use super::{MessengerHandle, ProducerRef};
use crate::error::{Error, Result};
use crate::types::{Message, Payload};
use config::node::QoSProfile;
use pmi::{MessengerPublisher, SenderTarget, TopicWireReceiver, TopicWireSender};

use std::sync::Arc;

/// A consumer-side topic subscription. Producer selection happens entirely
/// on the wire — a dep-slot subscription pins the bound producer's full
/// `(core_node, instance_id)` pair in the keyexpr, and infra subscriptions
/// deliberately wildcard it — so every message the wire delivers surfaces
/// to user code; there is no in-process producer filtering.
pub struct Subscription {
    inner: pmi::Subscription,
}

impl Subscription {
    pub(crate) fn new(inner: pmi::Subscription) -> Self {
        Self { inner }
    }

    pub async fn on_next_message(&mut self) -> Option<Message> {
        let raw = self.inner.rx.recv_async().await.ok()?;
        Some(Message::from(raw))
    }

    pub(crate) fn try_on_next_message(
        &mut self,
    ) -> std::result::Result<Message, crate::types::TryRecvError> {
        match self.inner.rx.try_recv() {
            Ok(raw) => Ok(Message::from(raw)),
            Err(err) => Err(crate::types::TryRecvError::from(err)),
        }
    }
}

pub struct TopicMessenger;

impl TopicMessenger {
    /// Subscribe to a topic published by a specific target. `from_target`
    /// filters on the publisher's identity — consumer dep slots always
    /// know the producer's node / interface target; a stream with no
    /// target to consult is an infra topic and goes through
    /// [`Self::subscribe_target_scoped`].
    /// `from_producer` is the slot's one bound producer: its full
    /// `(core_node, instance_id)` pair is pinned on the wire, so only that
    /// producer's publishes ever reach this subscription. There is no
    /// separate core_node parameter — producer identity always travels as
    /// the whole pair — and no fan-in: a consumer that needs several
    /// producers declares several slots.
    pub async fn subscribe(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        from_target: SenderTarget,
        to_topic: &str,
        from_producer: &ProducerRef,
        qos: QoSProfile,
    ) -> Result<Subscription> {
        let recv = TopicWireReceiver::new(
            as_core_node,
            as_instance_id,
            Some(from_producer.core_node.as_str()),
            Some(from_producer.instance_id.as_str()),
            Some(from_target),
            None,
            to_topic,
        )?;
        let subscription = messenger.subscribe_to_topic(&recv, qos).await?;
        Ok(Subscription::new(subscription))
    }

    /// Subscribe to a framework infra topic, scoped by the publisher's
    /// target identity alone while its per-boot `(core_node, instance_id)`
    /// pair stays wildcarded on the wire. Used for streams whose producer
    /// identity is unknowable or deliberately open: a node following its
    /// daemon's `clock` / `daemon_heartbeat` (a daemon's node name IS its
    /// core_node name, so the target pins which daemon matches), the
    /// daemon's own name-collision watch (the point is to hear foreign
    /// publishers), an external simulator's clock, and the benchmark
    /// prober. Deliberately separate from [`Self::subscribe`]: consumer
    /// dep slots only ever receive from explicitly bound producers, while
    /// infra topics have no binding to consult.
    pub async fn subscribe_target_scoped(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        from_target: SenderTarget,
        to_topic: &str,
        qos: QoSProfile,
    ) -> Result<Subscription> {
        let recv = TopicWireReceiver::new(
            as_core_node,
            as_instance_id,
            None,
            None,
            Some(from_target),
            None,
            to_topic,
        )?;
        let subscription = messenger.subscribe_to_topic(&recv, qos).await?;
        Ok(Subscription::new(subscription))
    }

    /// Subscribe to one topic of a pairing, pinned to the current peer's full
    /// wire triple: `(core_node, instance_id)` plus the link_id of the peer's
    /// own slot (the producer-side link_id segment of its publishes). Unlike
    /// [`Self::subscribe`], the link_id slot is a literal, never a wildcard —
    /// an unpaired slot has no wire subscription at all, so there is no
    /// wildcard shape to build. Pairing traffic rides the `pairing` wire
    /// discriminator, which no interface subscription can match.
    #[allow(clippy::too_many_arguments)]
    pub(crate) async fn subscribe_peer_pinned(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        pairing_target: SenderTarget,
        peer: &ProducerRef,
        peer_link_id: &str,
        to_topic: &str,
        qos: QoSProfile,
    ) -> Result<Subscription> {
        debug_assert!(
            pairing_target.is_pairing(),
            "subscribe_peer_pinned requires a pairing-shaped target, got {pairing_target:?}"
        );
        let recv = TopicWireReceiver::new(
            as_core_node,
            as_instance_id,
            Some(peer.core_node.as_str()),
            Some(peer.instance_id.as_str()),
            Some(pairing_target),
            Some(peer_link_id),
            to_topic,
        )?;
        let subscription = messenger.subscribe_to_topic(&recv, qos).await?;
        Ok(Subscription::new(subscription))
    }

    /// Waits until a subscriber for this topic is known to the publisher's
    /// session, or `timeout` elapses; returns whether a match was observed.
    ///
    /// In peer mode a freshly-connected publisher learns about existing
    /// subscribers through gossip, which is not instantaneous, so its first
    /// publish can be dropped before discovery propagates. Call
    /// this first when the very first publish must reach an already-running
    /// subscriber; it returns as soon as a match is observed (no fixed sleep).
    /// A `false` return means no subscriber appeared within `timeout`.
    pub async fn wait_for_subscriber(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        as_target: SenderTarget,
        as_topic_name: &str,
        timeout: std::time::Duration,
    ) -> Result<bool> {
        Self::wait_for_subscriber_with_link_id(
            messenger,
            as_core_node,
            as_instance_id,
            as_target,
            None,
            as_topic_name,
            timeout,
        )
        .await
    }

    /// [`Self::wait_for_subscriber`] for a publisher bound under a concrete
    /// producer-side `link_id` (a pairing slot publisher, or any
    /// `--link-id`-scoped publisher): the match is checked against the same
    /// keyexpr the publisher will emit on, link_id segment included.
    pub async fn wait_for_subscriber_with_link_id(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        as_target: SenderTarget,
        link_id: Option<&str>,
        as_topic_name: &str,
        timeout: std::time::Duration,
    ) -> Result<bool> {
        let sender = TopicWireSender::new(
            as_core_node,
            as_instance_id,
            as_target,
            link_id,
            as_topic_name,
        )?;
        messenger
            .wait_for_matching_subscriber(&sender, timeout)
            .await
    }

    /// Declares a topic publisher bound under a single producer-side link_id,
    /// bypassing the central `Messenger` mutex on every subsequent publish.
    /// `link_id` `None` falls back to the reserved default `_` segment.
    ///
    /// This is the only topic-publish path: declare a publisher once, then
    /// call [`TopicPublisher::publish`] per message. The publisher always tags
    /// its publishes as primary on the wire.
    #[allow(clippy::too_many_arguments)]
    pub async fn declare_publisher(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        as_target: SenderTarget,
        link_id: Option<&str>,
        as_topic_name: &str,
        qos: QoSProfile,
    ) -> Result<TopicPublisher> {
        let sender = TopicWireSender::new(
            as_core_node,
            as_instance_id,
            as_target,
            link_id,
            as_topic_name,
        )?;
        let inner = messenger
            .declare_topic_publisher(&sender, qos.into())
            .await?;
        Ok(TopicPublisher::new(Arc::new(inner)))
    }
}

/// Lock-free per-topic publisher returned by
/// [`TopicMessenger::declare_publisher`]. Wraps a [`pmi::MessengerPublisher`]
/// so `publish` skips the central `Arc<Mutex<Messenger>>` lock — callers in a
/// publish loop don't contend with all other messenger operations.
///
/// Cloneable so action handlers (e.g. feedback streams) can hand the same
/// publisher to multiple background tasks; clones share the same underlying
/// adapter handle (`Arc<zenoh::Session>` or mock `Arc<Mutex<HashMap>>`).
#[derive(Clone)]
pub struct TopicPublisher {
    inner: Arc<MessengerPublisher>,
}

impl TopicPublisher {
    pub(crate) fn new(inner: Arc<MessengerPublisher>) -> Self {
        Self { inner }
    }

    pub async fn publish(&self, payload: Payload) -> Result<()> {
        self.inner
            .publish(payload.into_inner())
            .await
            .map_err(Error::PeppyMessagingInterface)
    }
}

use super::{ConsumerFilter, FromAnyTopicGuard, MessengerHandle, ProducerRef};
use crate::error::{Error, Result};
use crate::types::{Message, Payload};
use config::node::QoSProfile;
use pmi::{MessengerPublisher, SenderTarget, TopicWireReceiver, TopicWireSender};

use std::sync::Arc;

/// Caller-side handle for one consumer slot's topic stream. Depending on
/// the slot's [`ConsumerFilter`] this merges over zero, one, or N
/// underlying wire subscriptions:
///
/// - Pin / Any: a single wire subscription (producer-pinned or wildcard).
/// - OnlyFrom: one **producer-pinned** wire subscription per bound
///   producer — the merge delivers from all N and only N; nothing else
///   traverses the wire. Per-producer FIFO order is preserved (each inner
///   is its own channel); arrival order *across* producers is arbitrary,
///   with a rotating sweep so a chatty producer cannot starve siblings.
/// - Silent (unbound from_any slot): **no wire subscription at all**.
///   [`Self::on_next_message`] pends forever — it never returns `None`,
///   because node receive loops treat a closed stream as shutdown, and an
///   unbound slot is a valid steady state, not a teardown.
///
/// `None` from [`Self::on_next_message`] means every inner wire channel
/// disconnected — i.e. real teardown.
pub struct Subscription {
    /// The merged wire subscriptions. Disconnected inners are pruned as
    /// they are observed; empty (for a non-silent subscription) means
    /// torn down.
    inners: Vec<pmi::Subscription>,
    /// Deliberately-silent slot: constructed with no wire subscriptions,
    /// pends forever instead of reporting a closed stream.
    silent: bool,
    /// Fair-merge cursor: each sweep starts after the inner that yielded
    /// the previous message.
    cursor: usize,
    /// Live for the subscription's full lifetime when this is a from_any
    /// topic sub; releases the messenger's per-`(name, tag)` reservation
    /// on drop. `None` for pinned subs and target-less subscriptions.
    _from_any_guard: Option<FromAnyTopicGuard>,
}

impl Subscription {
    pub(crate) fn new(inner: pmi::Subscription) -> Self {
        Self {
            inners: vec![inner],
            silent: false,
            cursor: 0,
            _from_any_guard: None,
        }
    }

    /// Merge over one producer-pinned wire subscription per bound
    /// producer ([`ConsumerFilter::OnlyFrom`]).
    pub(crate) fn multi(inners: Vec<pmi::Subscription>) -> Self {
        Self {
            inners,
            silent: false,
            cursor: 0,
            _from_any_guard: None,
        }
    }

    /// A deliberately-silent subscription for an unbound from_any slot:
    /// no wire work was done and none will be; `on_next_message` pends
    /// forever and `try_on_next_message` reports `Empty`.
    pub(crate) fn silent() -> Self {
        Self {
            inners: Vec::new(),
            silent: true,
            cursor: 0,
            _from_any_guard: None,
        }
    }

    pub(crate) fn with_from_any_guard(mut self, guard: FromAnyTopicGuard) -> Self {
        self._from_any_guard = Some(guard);
        self
    }

    /// Next message from any of the merged wire subscriptions, or `None`
    /// once every one of them has disconnected (teardown).
    ///
    /// A silent subscription pends forever instead of returning `None`:
    /// node receive loops treat a closed stream as shutdown, and an
    /// unbound slot must idle, not stop the node.
    pub async fn on_next_message(&mut self) -> Option<Message> {
        if self.silent {
            return std::future::pending().await;
        }
        loop {
            if self.inners.is_empty() {
                return None;
            }
            // Ready sweep first: serve already-buffered messages in
            // rotation so one producer's backlog cannot starve the rest.
            let count = self.inners.len();
            for offset in 0..count {
                let index = (self.cursor + offset) % count;
                match self.inners[index].rx.try_recv() {
                    Ok(raw) => {
                        self.cursor = index + 1;
                        return Some(Message::from(raw));
                    }
                    // Disconnections surface through the blocking merge
                    // below (recv_async fails immediately on a closed
                    // channel), which also prunes the inner.
                    Err(flume::TryRecvError::Empty | flume::TryRecvError::Disconnected) => {}
                }
            }
            // Nothing buffered: await the first arrival on any inner.
            // flume's recv_async is cancel-safe, so dropping the losing
            // futures (and this whole call, e.g. under `select!`) loses no
            // messages.
            let (result, index) = {
                let recvs: Vec<_> = self
                    .inners
                    .iter()
                    .map(|inner| Box::pin(inner.rx.recv_async()))
                    .collect();
                let (result, index, losers) = futures::future::select_all(recvs).await;
                // The losing futures borrow `self.inners`; end those
                // borrows before the disconnect arm mutates it.
                drop(losers);
                (result, index)
            };
            match result {
                Ok(raw) => {
                    self.cursor = index + 1;
                    return Some(Message::from(raw));
                }
                Err(_) => {
                    // This producer's channel closed; keep serving the
                    // rest. `None` only when all of them are gone.
                    self.inners.remove(index);
                    self.cursor = 0;
                }
            }
        }
    }

    pub(crate) fn try_on_next_message(
        &mut self,
    ) -> std::result::Result<Message, crate::types::TryRecvError> {
        if self.silent {
            // An unbound slot never has a message, and must not report
            // `Disconnected` — that reads as teardown to the callers.
            return Err(crate::types::TryRecvError::Empty);
        }
        if self.inners.is_empty() {
            return Err(crate::types::TryRecvError::Disconnected);
        }
        let count = self.inners.len();
        let mut disconnected = 0;
        for offset in 0..count {
            let index = (self.cursor + offset) % count;
            match self.inners[index].rx.try_recv() {
                Ok(raw) => {
                    self.cursor = index + 1;
                    return Ok(Message::from(raw));
                }
                Err(flume::TryRecvError::Empty) => {}
                Err(flume::TryRecvError::Disconnected) => disconnected += 1,
            }
        }
        if disconnected == count {
            self.inners.clear();
            Err(crate::types::TryRecvError::Disconnected)
        } else {
            Err(crate::types::TryRecvError::Empty)
        }
    }
}

pub struct TopicMessenger;

impl TopicMessenger {
    /// Subscribe to a topic published by a specific target. `from_target`
    /// `Some(SenderTarget)` filters on the publisher's identity; `None`
    /// wildcards the target segment (any node or interface emits a match).
    /// `is_from_any` marks this subscription as a `from_any: true` slot,
    /// gating the messenger's per-`(name, tag)` reservation (taken for
    /// every from_any subscribe — bound, collapsed-to-pin, or silent).
    ///
    /// The [`ConsumerFilter`] decides the wire shape:
    /// - `Pin` — one wire subscription pinning the producer's full
    ///   `(core_node, instance_id)`.
    /// - `OnlyFrom` — one producer-pinned wire subscription **per** bound
    ///   producer; no wildcard subscriber exists, so nothing outside the
    ///   bound set traverses the wire.
    /// - `Silent` — **no wire work at all**: the returned subscription
    ///   pends forever (an unbound slot idles; it is not torn down).
    /// - `Any` — one wildcard subscription (standalone mode / fixtures).
    ///
    /// All shapes keep the wire's producer-side link_id segment (seg-8)
    /// wildcarded — pinning is by producer identity, not by the
    /// producer's emit slot. There is no separate core_node parameter —
    /// producer identity always travels as the whole pair.
    #[allow(clippy::too_many_arguments)]
    pub async fn subscribe(
        messenger: &MessengerHandle,
        as_core_node: &str,
        as_instance_id: &str,
        from_target: Option<SenderTarget>,
        is_from_any: bool,
        to_topic: &str,
        filter: &ConsumerFilter,
        qos: QoSProfile,
    ) -> Result<Subscription> {
        // Reserve the from_any `(name, tag)` slot before any wire work.
        // The manifest validator enforces "at most one from_any topic
        // sub per (name, tag) per messenger" at config time; this is
        // the runtime guard at the wire's trust boundary. Silent slots
        // take the guard under the same condition so a sibling live
        // subscribe on the same (name, tag) still errors, while
        // target-less fixtures stay unguarded.
        let from_any_guard = match (&from_target, is_from_any) {
            (Some(target), true) => {
                Some(messenger.reserve_from_any_topic(target.name(), target.tag())?)
            }
            _ => None,
        };

        // One pinned receiver per producer the wire should admit; `None`
        // pins nothing (wildcard). The from_link_id slot (seg-8) stays
        // `None` for every interface subscription.
        let pinned_receiver = |producer: &ProducerRef| {
            TopicWireReceiver::new(
                as_core_node,
                as_instance_id,
                Some(producer.core_node.as_str()),
                Some(producer.instance_id.as_str()),
                from_target.clone(),
                None,
                to_topic,
            )
        };

        let mut subscription = match filter {
            ConsumerFilter::Pin(producer) => {
                let recv = pinned_receiver(producer)?;
                Subscription::new(messenger.subscribe_to_topic(&recv, qos).await?)
            }
            // Defensive: the filter layer never produces an empty bound
            // set (it resolves to Silent instead) — treat one as silent
            // rather than as an already-closed stream.
            ConsumerFilter::OnlyFrom(producers) if producers.is_empty() => Subscription::silent(),
            ConsumerFilter::OnlyFrom(producers) => {
                let mut inners = Vec::with_capacity(producers.len());
                for producer in producers {
                    let recv = pinned_receiver(producer)?;
                    inners.push(messenger.subscribe_to_topic(&recv, qos.clone()).await?);
                }
                Subscription::multi(inners)
            }
            ConsumerFilter::Silent => Subscription::silent(),
            ConsumerFilter::Any => {
                let recv = TopicWireReceiver::new(
                    as_core_node,
                    as_instance_id,
                    None,
                    None,
                    from_target.clone(),
                    None,
                    to_topic,
                )?;
                Subscription::new(messenger.subscribe_to_topic(&recv, qos).await?)
            }
        };
        if let Some(guard) = from_any_guard {
            subscription = subscription.with_from_any_guard(guard);
        }
        Ok(subscription)
    }

    /// Subscribe to one topic of a pairing, pinned to the current peer's full
    /// wire triple: `(core_node, instance_id)` plus the link_id of the peer's
    /// own slot (the producer-side link_id segment of its publishes). Unlike
    /// [`Self::subscribe`], the link_id slot is a literal, never a wildcard —
    /// an unpaired slot has no wire subscription at all, so there is no
    /// wildcard shape to build. The `from_any` reservation machinery is
    /// deliberately not involved: pairing traffic rides the `pairing` wire
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

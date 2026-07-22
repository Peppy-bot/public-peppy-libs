//! Shared consumer-side forwarding engine for pinned slots (pairing peers and
//! observer sources). Both follow a producer that the daemon delivers live over
//! a slot-update service, and both want the same wire-subscription lifecycle:
//!
//! - no followed pin (unpaired / unresolved) → no wire subscription at all
//!   (nothing to receive, and no wildcard shape exists for a pinned consumer);
//! - a followed pin → exactly one wire subscription, triple-pinned to the
//!   producer's `(core_node, instance_id, producer-side link_id)`;
//! - the pin changes (re-pair, or a source-incarnation change) → the old
//!   subscription is dropped BEFORE the new one is declared (at most one wire
//!   subscription per slot, ever), and a delivery-time stale filter drops any
//!   already-buffered message tagged with the superseded pin.
//!
//! Each slot kind differs only in what it follows: a pairing slot follows the
//! peer pin itself, an observer slot follows `(source generation, source pin)`
//! so a reused instance_id under an identical wire triple is still told apart.
//! [`FollowedSlot`] captures that difference; the loop, the single-subscription
//! invariant, the stale filter, and teardown live here once.

use crate::messaging::{MessengerHandle, ProducerRef, SenderTarget, Subscription, TopicMessenger};
use crate::runtime::TaskHandle;
use crate::types::Message;
use config::node::QoSProfile;
use std::sync::Arc;
use tokio::sync::{mpsc, watch};
use tracing::warn;

/// The kind of slot a [`SlotStream`] follows. An impl projects the slot's watch
/// state to the pin currently to follow (`None` when the slot has no active
/// producer) and exposes that pin's wire coordinates.
pub(crate) trait FollowedSlot: Send + Sync + 'static {
    /// The per-slot watch payload delivered by the slot-update service.
    type State: Send + Sync + 'static;
    /// The pin currently followed. Its `PartialEq` is the load-bearing key: the
    /// wire subscription is (re)declared whenever it changes, and a buffered
    /// message is dropped at delivery once it no longer matches the live pin.
    type Pin: Clone + PartialEq + Send + Sync + 'static;

    /// The pin to follow now, or `None` when the slot has no active producer.
    fn desired(state: &Self::State) -> Option<Self::Pin>;
    /// The producer whose publishes this pin subscribes to.
    fn producer(pin: &Self::Pin) -> &ProducerRef;
    /// The producer-side link_id segment of that producer's publishes.
    fn producer_link_id(pin: &Self::Pin) -> &str;
}

/// One slot's live message stream. Owns the forwarding task (aborted on drop)
/// and applies the delivery-time staleness filter shared by both slot kinds.
pub(crate) struct SlotStream<S: FollowedSlot> {
    rx: mpsc::Receiver<(Arc<S::Pin>, Message)>,
    watch_rx: watch::Receiver<S::State>,
    forward_task: TaskHandle<()>,
}

impl<S: FollowedSlot> SlotStream<S> {
    /// The next message from the currently followed pin, as `(producer,
    /// message)`. `None` when the runtime is torn down (slot channel closed).
    ///
    /// A message tagged with a pin the slot has since moved off is dropped here.
    /// The wire triple pin makes a foreign producer unmatchable at the keyexpr
    /// level, but a pin swap (re-pair, or a source-incarnation change under a
    /// reused triple) can leave a message buffered under the old pin; this
    /// re-check against the live state drops it. The pin alone cannot always
    /// discriminate incarnations, so the slot kind folds any generation into
    /// `Pin`'s identity.
    pub(crate) async fn next(&mut self) -> Option<(ProducerRef, Message)> {
        loop {
            let (pin, message) = self.rx.recv().await?;
            let current_matches = S::desired(&self.watch_rx.borrow()).as_ref() == Some(&*pin);
            if current_matches {
                return Some((S::producer(&pin).clone(), message));
            }
            // Stale: buffered under a pin the slot has since moved off.
        }
    }
}

/// Aborting the forwarding task on drop (rather than relying on its `tx.send`
/// erroring) matters because the task only touches `tx` when a message arrives:
/// an inactive or quiet slot leaves it parked on `watch_rx.changed()`, where it
/// would outlive the dropped stream — wire subscription included — until the
/// next slot update.
impl<S: FollowedSlot> Drop for SlotStream<S> {
    fn drop(&mut self) {
        self.forward_task.abort();
    }
}

/// Spawns the forwarding task and returns the stream. The public
/// `subscribe_*_with_watch` seams are one-line calls to this.
pub(crate) fn spawn_slot_stream<S: FollowedSlot>(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    watch_rx: watch::Receiver<S::State>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
) -> SlotStream<S> {
    let (tx, rx) = mpsc::channel(super::SLOT_CHANNEL_CAPACITY);
    let forward_task = crate::runtime::spawn(forward_messages::<S>(
        messenger,
        as_core_node,
        as_instance_id,
        watch_rx.clone(),
        pairing_target,
        topic,
        qos,
        tx,
    ));
    SlotStream {
        rx,
        watch_rx,
        forward_task,
    }
}

/// The eager forwarding loop: keeps the single wire subscription converged with
/// the slot's followed pin, tagging each forwarded message with the pin it
/// arrived under. Ends when the slot channel closes (runtime teardown) or the
/// stream is dropped (its `Drop` aborts this task).
#[allow(clippy::too_many_arguments)]
async fn forward_messages<S: FollowedSlot>(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    mut watch_rx: watch::Receiver<S::State>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
    tx: mpsc::Sender<(Arc<S::Pin>, Message)>,
) {
    // The pin the current wire subscription was declared under, behind an `Arc`
    // so tagging each forwarded message is a refcount bump, not a pin clone. At
    // most one wire subscription per slot, ever.
    let mut current: Option<(Arc<S::Pin>, Subscription)> = None;
    loop {
        // The decision reads only the followed pin; the loop top runs once per
        // forwarded message, so the pin is cloned out of the watch guard only
        // when it is about to be followed.
        let desired = S::desired(&watch_rx.borrow_and_update());
        // Redeclare when the followed pin first appears, changes, or clears; an
        // update that leaves the pin identical (e.g. a source's own peer
        // transition) is a no-op here.
        let redeclare = current.as_ref().map(|(pin, _)| &**pin) != desired.as_ref();
        if redeclare {
            // Drop-before-redeclare: the old wire subscription (and its buffered
            // messages) dies before the new pin's subscription exists.
            current = None;
            if let Some(pin) = desired {
                match TopicMessenger::subscribe_peer_pinned(
                    &messenger,
                    &as_core_node,
                    &as_instance_id,
                    pairing_target.clone(),
                    S::producer(&pin),
                    S::producer_link_id(&pin),
                    &topic,
                    qos.clone(),
                )
                .await
                {
                    Ok(subscription) => current = Some((Arc::new(pin), subscription)),
                    Err(err) => {
                        warn!(
                            %err,
                            topic = %topic,
                            "failed to declare pinned wire subscription; slot stays silent until the next slot update"
                        );
                    }
                }
            }
        }
        match current.as_mut() {
            Some((pin, subscription)) => {
                tokio::select! {
                    changed = watch_rx.changed() => {
                        if changed.is_err() {
                            return; // runtime teardown
                        }
                    }
                    message = subscription.on_next_message() => {
                        match message {
                            Some(message) => {
                                // The triple pin makes a foreign producer
                                // unmatchable at the keyexpr level; this
                                // re-check is the defensive second guard.
                                let producer = S::producer(pin);
                                let matches_pin = message.core_node() == producer.core_node
                                    && message.instance_id() == producer.instance_id
                                    && message.link_id() == S::producer_link_id(pin);
                                if matches_pin
                                    && tx.send((Arc::clone(pin), message)).await.is_err()
                                {
                                    return; // stream dropped
                                }
                            }
                            None => {
                                // Wire channel closed (session teardown).
                                current = None;
                                if watch_rx.changed().await.is_err() {
                                    return;
                                }
                            }
                        }
                    }
                }
            }
            None => {
                if watch_rx.changed().await.is_err() {
                    return;
                }
            }
        }
    }
}

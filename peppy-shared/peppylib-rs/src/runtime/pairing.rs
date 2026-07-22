//! Consumer-side runtime for pairing slots: [`PeerSlot`] (observe the slot's
//! pin state) and [`PeerSubscription`] (receive the paired peer's publishes).
//!
//! A `PeerSubscription` runs an eager forwarding task so the wire state
//! always converges with the pairing state, whether or not user code is
//! polling:
//!
//! - unpaired → no wire subscription at all (nothing to receive, and no
//!   wildcard shape exists for a pairing consumer);
//! - paired → exactly one wire subscription, triple-pinned to the peer's
//!   `(core_node, instance_id, slot link_id)`;
//! - re-pin → the old subscription is dropped BEFORE the new one is declared
//!   (at most one wire subscription per slot, ever), and a delivery-time
//!   stale filter drops any already-buffered message whose producer triple
//!   no longer matches the current pin.

use crate::error::{Error, Result};
use crate::messaging::{
    MessengerHandle, PeerInfo, PeerPin, PeerPinState, SenderTarget, TopicMessenger,
};
use crate::runtime::NodeRunner;
use crate::types::Message;
use config::node::QoSProfile;
use tokio::sync::{mpsc, watch};
use tracing::warn;

/// Handle onto one pairing slot's live pin state. Obtained via
/// [`NodeRunner::peer`]; the generated per-slot modules expose `paired()` /
/// `wait_paired()` delegating here.
#[derive(Clone)]
pub struct PeerSlot {
    watch_rx: watch::Receiver<PeerPinState>,
}

impl PeerSlot {
    pub(crate) fn new(watch_rx: watch::Receiver<PeerPinState>) -> Self {
        Self { watch_rx }
    }

    /// The currently paired peer, or `None` while the slot is unpaired.
    pub fn paired(&self) -> Option<PeerInfo> {
        self.watch_rx.borrow().pin.as_ref().map(PeerInfo::from)
    }

    /// Waits until the slot is paired and returns the peer. Returns
    /// immediately when already paired. Errors only if the runtime is torn
    /// down while waiting (the slot channel closed).
    pub async fn wait_paired(&mut self) -> Result<PeerInfo> {
        loop {
            if let Some(pin) = self.watch_rx.borrow_and_update().pin.as_ref() {
                return Ok(PeerInfo::from(pin));
            }
            self.watch_rx
                .changed()
                .await
                .map_err(|_| Error::PairingSlotClosed)?;
        }
    }
}

/// Stream of the paired peer's publishes on one topic of a pairing slot.
/// Yields nothing while the slot is unpaired; delivery resumes (from the
/// pairing moment, not retroactively — a pairing is a live stream, not a
/// mailbox) when a peer pairs.
pub struct PeerSubscription {
    rx: mpsc::Receiver<(PeerPin, Message)>,
    watch_rx: watch::Receiver<PeerPinState>,
    forward_task: crate::runtime::TaskHandle<()>,
}

impl PeerSubscription {
    /// Waits for the next message from the currently paired peer. Returns
    /// `None` when the runtime is torn down (slot channel closed).
    ///
    /// Messages forwarded before a re-pin or clear are dropped here at
    /// delivery time by re-checking the producer triple against the current
    /// pin, so a swap can never leak the previous peer's messages.
    pub async fn on_next_message(&mut self) -> Option<Message> {
        loop {
            let (pin, message) = self.rx.recv().await?;
            if self.watch_rx.borrow().pin.as_ref() == Some(&pin) {
                return Some(message);
            }
            // Stale: buffered under a pin that has since been swapped or
            // cleared.
        }
    }
}

/// Aborting the forwarding task here (rather than relying on its `tx.send`
/// erroring) matters because the task only touches `tx` when a message
/// arrives: an unpaired or quiet slot leaves it parked on
/// `watch_rx.changed()`, where it would outlive the dropped subscription —
/// wire subscription included — until the next pin update.
impl Drop for PeerSubscription {
    fn drop(&mut self) {
        self.forward_task.abort();
    }
}

/// Subscribe to one peer-emitted topic of the pairing slot at `link_id`.
/// Spliced by the generated `peppygen::paired_topics::<link_id>::<topic>::subscribe`
/// call sites; `pairing_name` / `pairing_tag` / `topic` come from the
/// pairing doc via codegen constants.
pub async fn subscribe_peer(
    node_runner: &NodeRunner,
    link_id: &str,
    pairing_name: &str,
    pairing_tag: &str,
    topic: &str,
    qos: QoSProfile,
) -> Result<PeerSubscription> {
    let processor = node_runner.processor();
    let watch_rx = processor
        .peer_pin_watch(link_id)
        .ok_or_else(|| Error::UnknownPairingSlot {
            link_id: link_id.to_string(),
        })?;
    let target = SenderTarget::pairing(pairing_name, pairing_tag)?;
    Ok(subscribe_peer_with_watch(
        node_runner.messenger().clone(),
        processor.bound_core_node().to_string(),
        processor.bound_instance_id().to_string(),
        watch_rx,
        target,
        topic.to_string(),
        qos,
    ))
}

/// Messenger-level core of [`subscribe_peer`]: the same forwarding-task
/// machinery driven by an explicit watch channel instead of a `NodeRunner`'s
/// processor-owned slot. Prefer [`subscribe_peer`] in nodes; this seam exists
/// for embedders and tests that manage pin state themselves.
pub fn subscribe_peer_with_watch(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    watch_rx: watch::Receiver<PeerPinState>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
) -> PeerSubscription {
    let (tx, rx) = mpsc::channel(super::SLOT_CHANNEL_CAPACITY);
    let task_watch_rx = watch_rx.clone();
    let forward_task = crate::runtime::spawn(forward_peer_messages(
        messenger,
        as_core_node,
        as_instance_id,
        task_watch_rx,
        pairing_target,
        topic,
        qos,
        tx,
    ));
    PeerSubscription {
        rx,
        watch_rx,
        forward_task,
    }
}

/// The eager forwarding loop: keeps the wire subscription converged with the
/// slot's pin state and forwards each received message tagged with the pin
/// it arrived under. Ends when the slot channel closes (runtime teardown) or
/// the receiving `PeerSubscription` is dropped (its `Drop` aborts this task).
#[allow(clippy::too_many_arguments)]
async fn forward_peer_messages(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    mut watch_rx: watch::Receiver<PeerPinState>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
    tx: mpsc::Sender<(PeerPin, Message)>,
) {
    let mut current: Option<(PeerPin, crate::messaging::Subscription)> = None;
    loop {
        let desired = watch_rx.borrow_and_update().pin.clone();
        if current.as_ref().map(|(pin, _)| pin) != desired.as_ref() {
            // Drop-before-redeclare: the old wire subscription (and its
            // buffered messages) dies before the new pin's subscription
            // exists, so the slot never holds two wire subscriptions.
            current = None;
            if let Some(pin) = desired {
                match TopicMessenger::subscribe_peer_pinned(
                    &messenger,
                    &as_core_node,
                    &as_instance_id,
                    pairing_target.clone(),
                    &pin.producer,
                    &pin.peer_link_id,
                    &topic,
                    qos.clone(),
                )
                .await
                {
                    Ok(subscription) => current = Some((pin, subscription)),
                    Err(err) => {
                        warn!(
                            %err,
                            topic = %topic,
                            "failed to declare peer wire subscription; slot stays silent until the next pin update"
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
                                // The triple pin makes a foreign message
                                // unmatchable at the keyexpr level; this
                                // re-check is the defensive second guard.
                                let matches_pin = message.core_node() == pin.producer.core_node
                                    && message.instance_id() == pin.producer.instance_id
                                    && message.link_id() == pin.peer_link_id;
                                if matches_pin && tx.send((pin.clone(), message)).await.is_err() {
                                    return; // subscription dropped
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messaging::ProducerRef;
    use std::time::Duration;

    fn pin() -> PeerPin {
        PeerPin {
            producer: ProducerRef::new("core_a", "arm_1"),
            peer_link_id: "controller".to_string(),
        }
    }

    #[tokio::test]
    async fn peer_slot_reports_current_pin() {
        let (tx, rx) = watch::channel(PeerPinState::unpaired());
        let slot = PeerSlot::new(rx);
        assert_eq!(slot.paired(), None);

        tx.send(PeerPinState {
            sequence: 1,
            pin: Some(pin()),
        })
        .unwrap();
        let info = slot.paired().expect("slot should be paired");
        assert_eq!(info.producer, ProducerRef::new("core_a", "arm_1"));
        assert_eq!(info.peer_link_id, "controller");
    }

    #[tokio::test]
    async fn wait_paired_returns_immediately_when_already_paired() {
        let (_tx, rx) = watch::channel(PeerPinState {
            sequence: 1,
            pin: Some(pin()),
        });
        let mut slot = PeerSlot::new(rx);
        let info = slot.wait_paired().await.expect("already paired");
        assert_eq!(info.peer_link_id, "controller");
    }

    #[tokio::test]
    async fn wait_paired_wakes_on_live_pair() {
        let (tx, rx) = watch::channel(PeerPinState::unpaired());
        let mut slot = PeerSlot::new(rx);
        let waiter = tokio::spawn(async move { slot.wait_paired().await });
        tokio::task::yield_now().await;
        tx.send(PeerPinState {
            sequence: 1,
            pin: Some(pin()),
        })
        .unwrap();
        let info = tokio::time::timeout(Duration::from_secs(1), waiter)
            .await
            .expect("wait_paired should wake")
            .expect("task should not panic")
            .expect("wait_paired should succeed");
        assert_eq!(info.producer.instance_id, "arm_1");
    }

    #[tokio::test]
    async fn wait_paired_errors_when_runtime_tears_down() {
        let (tx, rx) = watch::channel(PeerPinState::unpaired());
        let mut slot = PeerSlot::new(rx);
        drop(tx);
        let err = slot.wait_paired().await.unwrap_err();
        assert!(matches!(err, Error::PairingSlotClosed));
    }
}

//! Consumer-side runtime for pairing slots: [`PeerSlot`] (observe the slot's
//! pin state) and [`PeerSubscription`] (receive the paired peer's publishes).
//!
//! A `PeerSubscription` follows the paired peer through the shared
//! [`crate::runtime::slot_stream`] engine, which keeps at most one wire
//! subscription converged with the slot's live pin (unpaired → none; paired →
//! one, triple-pinned to the peer; re-pin → drop-before-redeclare plus a
//! delivery-time stale filter). This module supplies only what a pairing slot
//! follows: the peer pin itself.

use crate::error::{Error, Result};
use crate::messaging::{
    MessengerHandle, PeerInfo, PeerPin, PeerPinState, ProducerRef, SenderTarget,
};
use crate::runtime::NodeRunner;
use crate::runtime::slot_stream::{FollowedSlot, SlotStream, spawn_slot_stream};
use crate::types::Message;
use config::node::QoSProfile;
use tokio::sync::watch;

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

/// The pairing slot kind for the shared [`slot_stream`] engine: a pairing slot
/// follows the peer pin itself, so a re-pair to a new peer changes the pin and
/// a clear removes it.
///
/// [`slot_stream`]: crate::runtime::slot_stream
pub(crate) struct PeerFollow;

impl FollowedSlot for PeerFollow {
    type State = PeerPinState;
    type Pin = PeerPin;

    fn desired(state: &PeerPinState) -> Option<PeerPin> {
        state.pin.clone()
    }

    fn producer(pin: &PeerPin) -> &ProducerRef {
        &pin.producer
    }

    fn producer_link_id(pin: &PeerPin) -> &str {
        &pin.peer_link_id
    }
}

/// Stream of the paired peer's publishes on one topic of a pairing slot.
/// Yields nothing while the slot is unpaired; delivery resumes (from the
/// pairing moment, not retroactively — a pairing is a live stream, not a
/// mailbox) when a peer pairs.
pub struct PeerSubscription {
    stream: SlotStream<PeerFollow>,
}

impl PeerSubscription {
    /// Waits for the next message from the currently paired peer. Returns
    /// `None` when the runtime is torn down (slot channel closed). Messages
    /// buffered under a swapped-out or cleared pin are dropped before they
    /// surface (see [`SlotStream::next`]).
    pub async fn on_next_message(&mut self) -> Option<Message> {
        self.stream.next().await.map(|(_producer, message)| message)
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
    PeerSubscription {
        stream: spawn_slot_stream::<PeerFollow>(
            messenger,
            as_core_node,
            as_instance_id,
            watch_rx,
            pairing_target,
            topic,
            qos,
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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

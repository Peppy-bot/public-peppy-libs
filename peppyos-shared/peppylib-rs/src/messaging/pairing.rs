//! Peer-pin state for pairing slots. "Pairing" names the mechanism, contract,
//! and slot; "peer" names the other end of an established pair. A node's
//! runtime holds one [`tokio::sync::watch`] channel of [`PeerPinState`] per
//! declared pairing slot (see `runtime::Processor::pairing_slots`); the
//! daemon mutates it live over the `peer_update` service and the slot's
//! [`crate::runtime::PeerSubscription`] / [`crate::runtime::PeerSlot`]
//! observe it.

use super::ProducerRef;

/// The wire coordinates of the peer currently paired on a slot: the peer
/// instance's full `(core_node, instance_id)` address plus the link_id of
/// the peer's own complementary slot. Together the triple pins the peer's
/// publishes exactly (core, instance, producer-side link_id segment).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerPin {
    pub producer: ProducerRef,
    pub peer_link_id: String,
}

/// Absolute state of one pairing slot as delivered by the daemon. `sequence`
/// orders deliveries so a retried (stale) `peer_update` can never roll the
/// slot back: the listener rejects strictly-smaller sequences and treats an
/// equal sequence as an idempotent retry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerPinState {
    pub sequence: u64,
    /// `Some` while paired, `None` while unpaired.
    pub pin: Option<PeerPin>,
}

impl PeerPinState {
    /// Boot-time state: unpaired at sequence zero. Every daemon-delivered
    /// update carries a strictly larger sequence.
    pub fn unpaired() -> Self {
        Self {
            sequence: 0,
            pin: None,
        }
    }
}

/// User-facing identity of the peer paired on a slot, returned by
/// `NodeRunner::peer(link_id).paired()` / `wait_paired()` and surfaced by the
/// generated per-slot `paired()` / `wait_paired()` helpers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerInfo {
    /// The peer instance's full wire address.
    pub producer: ProducerRef,
    /// The link_id of the peer's complementary pairing slot.
    pub peer_link_id: String,
}

impl From<&PeerPin> for PeerInfo {
    fn from(pin: &PeerPin) -> Self {
        Self {
            producer: pin.producer.clone(),
            peer_link_id: pin.peer_link_id.clone(),
        }
    }
}

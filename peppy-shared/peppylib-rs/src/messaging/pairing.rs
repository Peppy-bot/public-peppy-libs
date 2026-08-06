//! Peer-pin state for pairing slots. "Pairing" names the mechanism, contract,
//! and slot; "peer" names the other end of an established pair. A node's
//! runtime holds one [`tokio::sync::watch`] channel of [`PeerPinState`] per
//! declared pairing slot (see `runtime::Processor::pairing_slots`); the
//! daemon mutates it live over the `peer_update` service and the slot's
//! [`crate::runtime::PeerSubscription`] / [`crate::runtime::PeerSlot`]
//! observe it.

use super::ProducerRef;

/// The peer paired on a slot: the peer instance's full `(core_node,
/// instance_id)` address plus the link_id of the peer's own complementary
/// slot. The triple pins the peer's publishes exactly (core, instance,
/// producer-side link_id segment).
///
/// Returned by `NodeRunner::peer(link_id).paired()` / `wait_paired()`,
/// surfaced by the generated per-slot `paired()` / `wait_paired()` helpers,
/// and tagged onto every message a peer subscription yields. Hashable and
/// ordered so consumers key maps on it.
///
/// Its derived `PartialEq` is the follow key: the slot's wire subscription is
/// redeclared when the pin changes, and a buffered message is dropped at
/// delivery once the slot has moved off the pin it was tagged with. A field
/// added here for presentation alone would move that predicate, so wrap it
/// instead.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct PeerInfo {
    /// The peer instance's full wire address.
    pub producer: ProducerRef,
    /// The link_id of the peer's complementary pairing slot.
    pub peer_link_id: String,
}

/// `PeerInfo` is documented as a map key, so the bounds that needs are pinned
/// here rather than left to the derive list. `Ord` has no other user in the
/// crate and would otherwise be droppable without a failure.
const _: fn() = || {
    fn assert_map_key<T: std::hash::Hash + Eq + Ord>() {}
    assert_map_key::<PeerInfo>();
};

/// Absolute state of one pairing slot as delivered by the daemon. `sequence`
/// orders deliveries so a retried (stale) `peer_update` can never roll the
/// slot back: the listener rejects strictly-smaller sequences and treats an
/// equal sequence as an idempotent retry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PeerPinState {
    pub sequence: u64,
    /// `Some` while paired, `None` while unpaired.
    pub pin: Option<PeerInfo>,
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

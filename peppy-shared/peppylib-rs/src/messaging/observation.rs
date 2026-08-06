//! Observation state for observer pairing slots. An observer passively taps a
//! producer's pairing topic without joining the 1:1 pairing. A node's runtime
//! holds one [`tokio::sync::watch`] channel of [`ObservationState`] per declared
//! observer slot (see `runtime::Processor`); the daemon mutates it live over the
//! `observation_update` service and the slot's
//! [`crate::runtime::ObservedTopicSubscription`] /
//! [`crate::runtime::ObservationSlot`] observe it.

use super::ProducerRef;

/// One observed source: the observed instance's full `(core_node,
/// instance_id)` address plus the producer-side link_id of the pairing slot
/// being observed. The triple pins the source's publishes exactly (core,
/// instance, producer-side link_id segment), which is both what an observer
/// subscription is declared against and the member's full identity, so members
/// sharing one instance stay distinct.
///
/// Returned by `NodeRunner::observation_slot(link_id)`'s `source()` and
/// `observation_slot_set(link_id)`'s `sources()`, surfaced by the generated
/// per-slot `source()` / `sources()` helpers, and tagged onto every message an
/// observed-topic subscription yields. Hashable and ordered so consumers key
/// demux maps on it. Purely local configuration state known to the observer
/// from its own registration; it needs no daemon push to read.
///
/// Its derived `PartialEq` is the follow key: a member's wire subscription is
/// redeclared when its source changes, and a buffered message is dropped at
/// delivery once its source leaves the followed set. A field added here for
/// presentation alone would move that predicate, so wrap it the way
/// [`ObservedMemberState`] wraps this type to carry the generation.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObservedSource {
    /// The observed source instance's full wire address.
    pub producer: ProducerRef,
    /// The producer-side link_id of the observed pairing slot.
    pub source_link_id: String,
}

/// The documented demux idiom keys a map on an `ObservedSource`, so the bounds
/// that needs are pinned here rather than left to the derive list. `Ord` has no
/// other user in the crate and would otherwise be droppable without a failure.
const _: fn() = || {
    fn assert_map_key<T: std::hash::Hash + Eq + Ord>() {}
    assert_map_key::<ObservedSource>();
};

/// One member of an observer slot's set: the pairing this member taps, plus
/// the two per-member facts the daemon keeps current.
///
/// `source_generation` is the daemon-assigned incarnation counter. It advances
/// only when this member's source changes incarnation (never on the source's own
/// peer transitions), and is the sole discriminator between old-B and new-B
/// messages, which are byte-identical on the wire. A change drops and redeclares
/// that member's wire subscription (buffer isolation) and invalidates any
/// in-flight tagged message from the previous generation.
///
/// `source_live` reports whether the member's source instance is currently in a
/// non-terminal state. It is informational (the observer keeps the subscription
/// declared whether or not the source is live), delivered so the state is
/// complete. A member whose source is down stays listed, at its position.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservedMemberState {
    pub source: ObservedSource,
    pub source_generation: u64,
    pub source_live: bool,
}

/// Absolute observation state for one observer slot as delivered by the daemon:
/// the slot's complete ordered member set, in the order the plan listed it.
///
/// `sequence` orders `observation_update` deliveries so a delayed (stale) retry
/// can never roll the slot back; the listener rejects strictly-smaller sequences
/// and treats an equal sequence as an idempotent retry. Each delivery carries
/// the whole set and replaces it wholesale, so the members a delivery omits are
/// gone from the slot.
///
/// `members` is empty before the daemon has delivered the slot (the boot state)
/// and for a `zero_or_more` slot the plan left with nothing to observe. A
/// member's position never moves once delivered: a generation bump changes that
/// member in place.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationState {
    pub sequence: u64,
    pub members: Vec<ObservedMemberState>,
}

impl ObservationState {
    /// The empty state at sequence zero. A slot boots from the config's
    /// `observation_seeds` entry (the plan's membership, stamped by the
    /// daemon at spawn); a missing seed counts as empty and constructs only
    /// where the plan could have written an empty set (`zero_or_one` vacant,
    /// `zero_or_more` observing nothing), so this is the boot state of those
    /// slots and of embedders that manage observation state themselves. Live
    /// `observation_update` deliveries replace the boot state at strictly
    /// larger sequences.
    pub fn unregistered() -> Self {
        Self {
            sequence: 0,
            members: Vec::new(),
        }
    }
}

/// Narrows a member to the identity alone, dropping the daemon-kept
/// `source_generation` and `source_live`, which are the observer runtime's
/// business and not the consumer's.
impl From<&ObservedMemberState> for ObservedSource {
    fn from(member: &ObservedMemberState) -> Self {
        member.source.clone()
    }
}

//! Observation state for observer pairing slots. An observer passively taps a
//! producer's pairing topic without joining the 1:1 pairing. A node's runtime
//! holds one [`tokio::sync::watch`] channel of [`ObservationState`] per declared
//! observer slot (see `runtime::Processor`); the daemon mutates it live over the
//! `observation_update` service and the slot's
//! [`crate::runtime::ObservedTopicSubscription`] /
//! [`crate::runtime::ObservationSlot`] observe it.

use super::ProducerRef;

/// The wire coordinates of an observed producer source: its full
/// `(core_node, instance_id)` address plus the producer-side link_id of the
/// pairing slot being observed. Together the triple pins the source's publishes
/// exactly (core, instance, producer-side link_id segment), the same pin an
/// observer subscription is declared against.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationPin {
    pub producer: ProducerRef,
    pub source_link_id: String,
}

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
    pub source: ObservationPin,
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
    /// Boot-time state: no members delivered yet, at sequence zero. The daemon
    /// delivers the slot's member set over `observation_update` right after the
    /// instance commits, exactly as it delivers pairing pins.
    pub fn unregistered() -> Self {
        Self {
            sequence: 0,
            members: Vec::new(),
        }
    }
}

/// User-facing observed source of one observer-slot member, returned by
/// `NodeRunner::observation_slot(link_id)`'s `source()` and
/// `observation_slot_set(link_id)`'s `sources()`, surfaced by the generated
/// per-slot `source()` / `sources()` helpers, and tagged onto every message an
/// observed-topic subscription yields. It is the member's full identity, so
/// members sharing one instance stay distinct, and it is hashable and ordered
/// so consumers key demux maps on it. Purely local configuration state known
/// to the observer from its own registration; it needs no daemon push to read.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObservedSource {
    /// The observed source instance's full wire address.
    pub producer: ProducerRef,
    /// The producer-side link_id of the observed pairing slot.
    pub source_link_id: String,
}

impl From<&ObservationPin> for ObservedSource {
    fn from(pin: &ObservationPin) -> Self {
        Self {
            producer: pin.producer.clone(),
            source_link_id: pin.source_link_id.clone(),
        }
    }
}

impl From<&ObservedMemberState> for ObservedSource {
    fn from(member: &ObservedMemberState) -> Self {
        Self::from(&member.source)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    /// An `ObservedSource` keys demux maps: two members sharing one instance
    /// hash and compare as distinct entries because the link_id is part of the
    /// identity.
    #[test]
    fn observed_source_keys_demux_maps_per_member() {
        let left = ObservedSource {
            producer: ProducerRef::new("core_a", "backbone_1"),
            source_link_id: "left_arm".to_string(),
        };
        let right = ObservedSource {
            producer: ProducerRef::new("core_a", "backbone_1"),
            source_link_id: "right_arm".to_string(),
        };

        let handlers = HashMap::from([(left.clone(), "left"), (right.clone(), "right")]);
        assert_eq!(handlers.len(), 2);
        assert_eq!(handlers[&left], "left");
        assert_eq!(handlers[&right], "right");
    }
}

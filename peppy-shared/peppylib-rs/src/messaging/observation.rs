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

/// Absolute observation state for one observer slot as delivered by the daemon.
///
/// Two monotonic counters ride here, and they mean different things:
/// - `sequence` orders `observation_update` deliveries so a delayed (stale)
///   retry can never roll the slot back; the listener rejects strictly-smaller
///   sequences and treats an equal sequence as an idempotent retry.
/// - `source_generation` is the daemon-assigned incarnation counter. It advances
///   only when the source's incarnation changes (never on the source's own peer
///   transitions), and is the sole discriminator between old-B and new-B
///   messages, which are byte-identical on the wire. A change drops and
///   redeclares the wire subscription (buffer isolation) and invalidates any
///   in-flight tagged message from the previous generation.
///
/// `source` is `None` only before the daemon has resolved the slot's source (the
/// boot state); once resolved it stays `Some` across every peer transition.
/// `source_live` reports whether the source instance is currently in a
/// non-terminal state; it is informational (the observer keeps its subscription
/// declared whether or not the source is live), delivered so the state is
/// complete.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObservationState {
    pub sequence: u64,
    pub source_generation: u64,
    pub source: Option<ObservationPin>,
    pub source_live: bool,
}

impl ObservationState {
    /// Boot-time state: no source resolved yet, at sequence and generation
    /// zero. The daemon delivers the resolved source over `observation_update`
    /// right after the instance commits, exactly as it delivers pairing pins.
    pub fn unregistered() -> Self {
        Self {
            sequence: 0,
            source_generation: 0,
            source: None,
            source_live: false,
        }
    }
}

/// User-facing resolved source of an observer slot, returned by
/// `NodeRunner::observed_source(link_id)` and surfaced by the generated per-slot
/// `source()` helper. Purely local configuration state known to the observer
/// from its own registration; it needs no daemon push to read.
#[derive(Debug, Clone, PartialEq, Eq)]
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

//! Consumer-side runtime for observer pairing slots: [`ObservationSlot`]
//! (observe the slot's resolved source) and [`ObservedTopicSubscription`]
//! (receive the observed source's publishes on one topic).
//!
//! An observer passively taps a producer's pairing topic without joining the
//! source's 1:1 pairing. It follows the source through the shared
//! [`crate::runtime::slot_stream`] engine, which keeps at most one wire
//! subscription converged with the followed pin. Unlike a pairing slot, an
//! observer follows `(source generation, source pin)`, not the pin alone: the
//! source's own peer transitions never touch the subscription (the pin is
//! unchanged), while a source-incarnation change advances the generation and so
//! redeclares — even though a reused instance_id keeps the wire triple
//! byte-identical, which the keyexpr alone cannot tell apart. This module
//! supplies only that follow rule; the loop and stale filter live in the engine.

use crate::error::{Error, Result};
use crate::messaging::{
    MessengerHandle, ObservationPin, ObservationState, ObservedSource, ProducerRef, SenderTarget,
};
use crate::runtime::NodeRunner;
use crate::runtime::slot_stream::{FollowedSlot, SlotStream, spawn_slot_stream};
use crate::types::Message;
use config::node::QoSProfile;
use tokio::sync::watch;

/// Handle onto one observer slot's live observation state. Obtained via
/// [`NodeRunner::observation_slot`]; the generated per-slot modules expose
/// `source()` delegating here.
#[derive(Clone)]
pub struct ObservationSlot {
    watch_rx: watch::Receiver<ObservationState>,
}

impl ObservationSlot {
    pub(crate) fn new(watch_rx: watch::Receiver<ObservationState>) -> Self {
        Self { watch_rx }
    }

    /// The resolved source of this observer slot, or `None` before the daemon
    /// has delivered it. Purely local configuration state; there is no
    /// health-derived helper, because a third node's health is not knowable
    /// here (see the design's "Generated observer API").
    pub fn source(&self) -> Option<ObservedSource> {
        self.watch_rx
            .borrow()
            .source
            .as_ref()
            .map(ObservedSource::from)
    }
}

/// The observer slot kind for the shared [`slot_stream`] engine. An observer
/// follows `(source generation, source pin)`: the pin is the source's wire
/// triple, and the generation tells a reused-instance_id incarnation apart from
/// its predecessor, whose publishes are byte-identical on the wire.
///
/// [`slot_stream`]: crate::runtime::slot_stream
#[derive(Clone, PartialEq, Eq)]
pub(crate) struct ObservedPin {
    generation: u64,
    source: ObservationPin,
}

pub(crate) struct ObservedFollow;

impl FollowedSlot for ObservedFollow {
    type State = ObservationState;
    type Pin = ObservedPin;

    fn desired(state: &ObservationState) -> Option<ObservedPin> {
        state.source.as_ref().map(|source| ObservedPin {
            generation: state.source_generation,
            source: source.clone(),
        })
    }

    fn producer(pin: &ObservedPin) -> &ProducerRef {
        &pin.source.producer
    }

    fn producer_link_id(pin: &ObservedPin) -> &str {
        &pin.source.source_link_id
    }
}

/// Stream of an observed source's publishes on one topic. Yields nothing while
/// the source is unresolved or not emitting; delivery is a live stream, never a
/// mailbox, so messages published before observation became active are never
/// replayed.
pub struct ObservedTopicSubscription {
    stream: SlotStream<ObservedFollow>,
}

impl ObservedTopicSubscription {
    /// Waits for the next `(producer, message)` from the currently observed
    /// source incarnation. Returns `None` when the runtime is torn down (slot
    /// channel closed). A message buffered under a superseded source generation
    /// is dropped before it surfaces (see [`SlotStream::next`]).
    pub async fn on_next_message(&mut self) -> Option<(ProducerRef, Message)> {
        self.stream.next().await
    }
}

/// Subscribe to one topic emitted by an observer slot's source. Spliced by the
/// generated `peppygen::paired_topics::<link_id>::<topic>::subscribe` call sites
/// of observer modules; `pairing_name` / `pairing_tag` / `topic` come from the
/// pairing doc via codegen constants.
pub async fn subscribe_observed(
    node_runner: &NodeRunner,
    link_id: &str,
    pairing_name: &str,
    pairing_tag: &str,
    topic: &str,
    qos: QoSProfile,
) -> Result<ObservedTopicSubscription> {
    let processor = node_runner.processor();
    let watch_rx =
        processor
            .observation_slot_watch(link_id)
            .ok_or_else(|| Error::UnknownObservationSlot {
                link_id: link_id.to_string(),
            })?;
    let target = SenderTarget::pairing(pairing_name, pairing_tag)?;
    Ok(subscribe_observed_with_watch(
        node_runner.messenger().clone(),
        processor.bound_core_node().to_string(),
        processor.bound_instance_id().to_string(),
        watch_rx,
        target,
        topic.to_string(),
        qos,
    ))
}

/// Messenger-level core of [`subscribe_observed`]: the same forwarding-task
/// machinery driven by an explicit watch channel instead of a `NodeRunner`'s
/// processor-owned slot. Prefer [`subscribe_observed`] in nodes; this seam
/// exists for embedders and tests that manage observation state themselves.
pub fn subscribe_observed_with_watch(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    watch_rx: watch::Receiver<ObservationState>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
) -> ObservedTopicSubscription {
    ObservedTopicSubscription {
        stream: spawn_slot_stream::<ObservedFollow>(
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

    fn source() -> ObservationPin {
        ObservationPin {
            producer: ProducerRef::new("core_a", "arm_1"),
            source_link_id: "commander".to_string(),
        }
    }

    fn resolved(generation: u64) -> ObservationState {
        ObservationState {
            sequence: generation,
            source_generation: generation,
            source: Some(source()),
            source_live: true,
        }
    }

    #[tokio::test]
    async fn slot_reports_resolved_source() {
        let (tx, rx) = watch::channel(ObservationState::unregistered());
        let slot = ObservationSlot::new(rx);
        assert_eq!(slot.source(), None);

        tx.send(resolved(1)).unwrap();
        let src = slot.source().expect("slot should be resolved");
        assert_eq!(src.producer, ProducerRef::new("core_a", "arm_1"));
        assert_eq!(src.source_link_id, "commander");
    }
}

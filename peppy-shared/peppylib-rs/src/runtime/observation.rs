//! Consumer-side runtime for observer pairing slots: [`ObservationSlot`] and
//! [`ObservationSlotSet`] (observe the slot's member set) and
//! [`ObservedTopicSubscription`] (receive the observed sources' publishes on one
//! topic).
//!
//! An observer passively taps a producer's pairing topic without joining the
//! source's 1:1 pairing. It follows its sources through the shared
//! [`crate::runtime::slot_stream`] engine, which keeps one wire subscription per
//! followed pin converged with the slot's member set. Unlike a pairing slot, an
//! observer follows `(source generation, source pin)` per member, not the pin
//! alone: a source's own peer transitions never touch its subscription (the pin
//! is unchanged), while a source-incarnation change advances that member's
//! generation and so redeclares, even though a reused instance_id keeps the wire
//! triple byte-identical, which the keyexpr alone cannot tell apart. This module
//! supplies only that follow rule; the convergence, the fair merge across
//! members, and the stale filter live in the engine.

use crate::error::{Error, Result};
use crate::messaging::{
    MessengerHandle, ObservationPin, ObservationState, ObservedSource, ProducerRef, SenderTarget,
};
use crate::runtime::NodeRunner;
use crate::runtime::slot_stream::{FollowedSlot, SlotStream, spawn_slot_stream};
use crate::types::Message;
use config::node::QoSProfile;
use tokio::sync::watch;

/// The panic every cardinality-typed observer accessor raises when the shape it
/// was generated for is not the shape the slot has. Mirrors
/// [`crate::runtime::Processor::sole_bound_producer`] on the producer-binding
/// side: an accessor that does not match the manifest is stale codegen, a bug
/// rather than a user error.
pub(crate) fn observer_shape_panic(link_id: &str, accessor: &str, declared: &str) -> ! {
    panic!(
        "observer slot `{link_id}` is declared `{declared}` but was read through `{accessor}`: \
         the generated code and the manifest disagree (version skew / stale codegen); \
         regenerate bindings for this node"
    )
}

/// Handle onto a `cardinality: "one"` observer slot's live observation state.
/// Obtained via [`NodeRunner::observation_slot`]; the generated per-slot modules
/// of `one` slots expose `source()` delegating here. Multi-member slots are read
/// through [`ObservationSlotSet`] instead.
#[derive(Clone)]
pub struct ObservationSlot {
    link_id: String,
    watch_rx: watch::Receiver<ObservationState>,
}

impl ObservationSlot {
    pub(crate) fn new(
        link_id: impl Into<String>,
        watch_rx: watch::Receiver<ObservationState>,
    ) -> Self {
        Self {
            link_id: link_id.into(),
            watch_rx,
        }
    }

    /// The observed source of this slot, or `None` before the daemon has
    /// delivered it. Purely local configuration state; there is no
    /// health-derived helper, because a third node's health is not knowable
    /// here (see the design's "Generated observer API").
    ///
    /// Panics if the slot holds more than one member, which a `one` slot cannot
    /// have: reading a multi-member slot through this accessor is stale codegen.
    pub fn source(&self) -> Option<ObservedSource> {
        let state = self.watch_rx.borrow();
        match state.members.as_slice() {
            [] => None,
            [sole] => Some(ObservedSource::from(sole)),
            _ => observer_shape_panic(&self.link_id, "source()", "one"),
        }
    }
}

/// Handle onto a multi-member observer slot's live observation state (a
/// `one_or_more` or `zero_or_more` slot). Obtained via
/// [`NodeRunner::observation_slot_set`]; the generated per-slot modules of those
/// slots expose `sources()` delegating here.
///
/// The member set is live: the daemon replaces it whole whenever the plan's
/// observed pairings change, so a set read now can differ from one read later,
/// and an empty set is legal at any instant (before first delivery, during a
/// replan, or for a `zero_or_more` slot the plan left unobserved).
#[derive(Clone)]
pub struct ObservationSlotSet {
    watch_rx: watch::Receiver<ObservationState>,
}

impl ObservationSlotSet {
    pub(crate) fn new(watch_rx: watch::Receiver<ObservationState>) -> Self {
        Self { watch_rx }
    }

    /// Every pairing this slot currently observes, in plan order: the order the
    /// launcher's array or the `--link` occurrences wrote, preserved end to
    /// end, so member N here is the deployment's Nth entry for this slot.
    pub fn sources(&self) -> Vec<ObservedSource> {
        self.watch_rx
            .borrow()
            .members
            .iter()
            .map(ObservedSource::from)
            .collect()
    }
}

/// The observer slot kind for the shared [`slot_stream`] engine. An observer
/// follows one `(source generation, source pin)` per member: the pin is the
/// source's wire triple, and the generation tells a reused-instance_id
/// incarnation apart from its predecessor, whose publishes are byte-identical on
/// the wire.
///
/// [`slot_stream`]: crate::runtime::slot_stream
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ObservedPin {
    generation: u64,
    source: ObservationPin,
}

pub(crate) struct ObservedFollow;

impl FollowedSlot for ObservedFollow {
    type State = ObservationState;
    type Pin = ObservedPin;

    fn desired(state: &ObservationState) -> Vec<ObservedPin> {
        state
            .members
            .iter()
            .map(|member| ObservedPin {
                generation: member.source_generation,
                source: member.source.clone(),
            })
            .collect()
    }

    fn is_followed(state: &ObservationState, pin: &ObservedPin) -> bool {
        state
            .members
            .iter()
            .any(|member| member.source_generation == pin.generation && member.source == pin.source)
    }

    fn producer(pin: &ObservedPin) -> &ProducerRef {
        &pin.source.producer
    }

    fn producer_link_id(pin: &ObservedPin) -> &str {
        &pin.source.source_link_id
    }
}

/// Stream of the observed sources' publishes on one topic, fanned in across the
/// slot's whole member set. Yields nothing while the set is empty or no member
/// is emitting; delivery is a live stream, never a mailbox, so messages
/// published before observation became active are never replayed.
pub struct ObservedTopicSubscription {
    stream: SlotStream<ObservedFollow>,
}

impl ObservedTopicSubscription {
    /// Waits for the next `(producer, message)` from any currently observed
    /// source incarnation. Every cardinality fans in the same way and every
    /// message is tagged with the member that published it, so a multi-member
    /// slot's consumer routes on the producer rather than on which subscription
    /// it came from. Returns `None` when the runtime is torn down (slot channel
    /// closed). A message buffered under a superseded source generation, or
    /// under a member the slot has since dropped, never surfaces (see
    /// [`SlotStream::next`]).
    pub async fn on_next_message(&mut self) -> Option<(ProducerRef, Message)> {
        self.stream.next().await
    }
}

/// Subscribe to one topic emitted by an observer slot's sources, for every
/// cardinality. Spliced by the generated
/// `peppygen::paired_topics::<link_id>::<topic>::subscribe` call sites of
/// observer modules; `pairing_name` / `pairing_tag` / `topic` come from the
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
    use crate::messaging::ObservedMemberState;

    fn member(instance: &str, generation: u64) -> ObservedMemberState {
        ObservedMemberState {
            source: ObservationPin {
                producer: ProducerRef::new("core_a", instance),
                source_link_id: "commander".to_string(),
            },
            source_generation: generation,
            source_live: true,
        }
    }

    fn state(members: Vec<ObservedMemberState>) -> ObservationState {
        ObservationState {
            sequence: 1,
            members,
        }
    }

    #[tokio::test]
    async fn one_slot_reports_its_sole_source() {
        let (tx, rx) = watch::channel(ObservationState::unregistered());
        let slot = ObservationSlot::new("observed_arm", rx);
        assert_eq!(slot.source(), None);

        tx.send(state(vec![member("arm_1", 1)])).unwrap();
        let src = slot.source().expect("slot should be resolved");
        assert_eq!(src.producer, ProducerRef::new("core_a", "arm_1"));
        assert_eq!(src.source_link_id, "commander");
    }

    #[tokio::test]
    async fn slot_set_reports_every_member_in_plan_order() {
        let (tx, rx) = watch::channel(ObservationState::unregistered());
        let set = ObservationSlotSet::new(rx);
        assert!(
            set.sources().is_empty(),
            "an undelivered set is empty, not absent"
        );

        tx.send(state(vec![member("arm_2", 1), member("arm_1", 1)]))
            .unwrap();
        assert_eq!(
            set.sources()
                .iter()
                .map(|s| s.producer.instance_id.clone())
                .collect::<Vec<_>>(),
            ["arm_2", "arm_1"]
        );

        // The set is live: a replan that drops a member shrinks it.
        tx.send(state(vec![member("arm_2", 1)])).unwrap();
        assert_eq!(set.sources().len(), 1);
    }

    /// Reading a multi-member slot through the `one` accessor is stale codegen,
    /// so it panics rather than silently reporting the first member.
    #[tokio::test]
    #[should_panic(expected = "observed_joints")]
    async fn one_accessor_on_a_multi_member_slot_panics() {
        let (tx, rx) = watch::channel(ObservationState::unregistered());
        let slot = ObservationSlot::new("observed_joints", rx);
        tx.send(state(vec![member("arm_1", 1), member("arm_2", 1)]))
            .unwrap();
        let _ = slot.source();
    }

    /// The followed set is one pin per member, and a member's generation is
    /// part of its pin so an incarnation change redeclares only that member.
    #[test]
    fn followed_pins_are_one_per_member_and_generation_keyed() {
        let before = state(vec![member("arm_1", 1), member("arm_2", 1)]);
        let after = state(vec![member("arm_1", 1), member("arm_2", 2)]);

        let before_pins = ObservedFollow::desired(&before);
        let after_pins = ObservedFollow::desired(&after);
        assert_eq!(before_pins.len(), 2);
        assert_eq!(
            before_pins[0], after_pins[0],
            "an untouched member keeps its pin, so its subscription survives"
        );
        assert_ne!(
            before_pins[1], after_pins[1],
            "an incarnation change must redeclare that member"
        );
        assert!(ObservedFollow::desired(&ObservationState::unregistered()).is_empty());
    }
}

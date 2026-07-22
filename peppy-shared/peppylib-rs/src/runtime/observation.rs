//! Consumer-side runtime for observer pairing slots: [`ObservationSlot`]
//! (observe the slot's resolved source) and [`ObservedTopicSubscription`]
//! (receive the observed source's publishes on one topic).
//!
//! An observer passively taps a producer's pairing topic without joining the
//! source's 1:1 pairing. Unlike a participant subscription, an observer
//! subscription follows the *source instance's* lifecycle, not any pair
//! generation:
//!
//! - source unresolved (boot) → no wire subscription yet;
//! - source resolved → exactly one wire subscription, triple-pinned to the
//!   source's `(core_node, instance_id, source slot link_id)`, declared and held
//!   whether or not the source currently has a paired peer;
//! - the source's peer transitions (paired, unpaired, re-paired) do NOT touch
//!   the subscription or advance the generation;
//! - a source-generation change (the source's incarnation was replaced) drops
//!   the old subscription BEFORE declaring the new one (at most one wire
//!   subscription per slot, ever) and a delivery-time stale filter drops any
//!   message still buffered under the previous generation.
//!
//! The pin is identical across a generation change (a reused instance_id under
//! the same triple), so the keyexpr cannot distinguish incarnations; the
//! generation tag is the sole discriminator, applied here in the forwarding
//! buffer and by the drop-before-redeclare in the transport buffer.

use crate::error::{Error, Result};
use crate::messaging::{
    MessengerHandle, ObservationPin, ObservationState, ObservedSource, SenderTarget, TopicMessenger,
};
use crate::runtime::NodeRunner;
use crate::types::Message;
use config::node::QoSProfile;
use tokio::sync::{mpsc, watch};
use tracing::warn;

/// In-flight buffer between the forwarding task and the consuming code, in
/// messages. Deliberately small, matching the pairing subscription: observer
/// topics are taps on conversations, not firehoses, and the wire-side QoS
/// buffers already absorb bursts.
const OBSERVATION_CHANNEL_CAPACITY: usize = 128;

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
        self.watch_rx.borrow().source.as_ref().map(ObservedSource::from)
    }
}

/// Stream of an observed source's publishes on one topic. Yields nothing while
/// the source is unresolved or not emitting; delivery is a live stream, never a
/// mailbox, so messages published before observation became active are never
/// replayed.
pub struct ObservedTopicSubscription {
    rx: mpsc::Receiver<(u64, ObservationPin, Message)>,
    watch_rx: watch::Receiver<ObservationState>,
    forward_task: crate::runtime::TaskHandle<()>,
}

impl ObservedTopicSubscription {
    /// Waits for the next message from the currently observed source incarnation.
    /// Returns `None` when the runtime is torn down (slot channel closed).
    ///
    /// A message forwarded under a previous source generation is dropped here at
    /// delivery time by re-checking its generation (and pin) against the current
    /// observation state, so a message buffered before a source-incarnation
    /// change can never surface after it. The pin alone cannot discriminate
    /// incarnations (a reused instance_id keeps the same wire triple), so the
    /// generation is the load-bearing check.
    pub async fn next(&mut self) -> Option<(crate::messaging::ProducerRef, Message)> {
        loop {
            let (generation, pin, message) = self.rx.recv().await?;
            let state = self.watch_rx.borrow();
            let current_matches =
                state.source_generation == generation && state.source.as_ref() == Some(&pin);
            drop(state);
            if current_matches {
                return Some((pin.producer, message));
            }
            // Stale: buffered under a source generation that has since advanced.
        }
    }
}

/// Aborting the forwarding task here (rather than relying on its `tx.send`
/// erroring) matters because the task only touches `tx` when a message arrives:
/// an unresolved or quiet source leaves it parked on `watch_rx.changed()`, where
/// it would outlive the dropped subscription — wire subscription included —
/// until the next observation update.
impl Drop for ObservedTopicSubscription {
    fn drop(&mut self) {
        self.forward_task.abort();
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
    let watch_rx = processor
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
    let (tx, rx) = mpsc::channel(OBSERVATION_CHANNEL_CAPACITY);
    let task_watch_rx = watch_rx.clone();
    let forward_task = crate::runtime::spawn(forward_observed_messages(
        messenger,
        as_core_node,
        as_instance_id,
        task_watch_rx,
        pairing_target,
        topic,
        qos,
        tx,
    ));
    ObservedTopicSubscription {
        rx,
        watch_rx,
        forward_task,
    }
}

/// The eager forwarding loop: keeps the wire subscription converged with the
/// slot's source pin and generation, and forwards each received message tagged
/// with the generation and pin it arrived under. Ends when the slot channel
/// closes (runtime teardown) or the receiving subscription is dropped (its
/// `Drop` aborts this task).
#[allow(clippy::too_many_arguments)]
async fn forward_observed_messages(
    messenger: MessengerHandle,
    as_core_node: String,
    as_instance_id: String,
    mut watch_rx: watch::Receiver<ObservationState>,
    pairing_target: SenderTarget,
    topic: String,
    qos: QoSProfile,
    tx: mpsc::Sender<(u64, ObservationPin, Message)>,
) {
    // Tracks the (generation, pin) the current wire subscription was declared
    // under. At most one wire subscription per slot, ever.
    let mut current: Option<(u64, ObservationPin, crate::messaging::Subscription)> = None;
    loop {
        let (desired_generation, desired_source) = {
            let state = watch_rx.borrow_and_update();
            (state.source_generation, state.source.clone())
        };
        // Redeclare when the source first resolves, when the generation
        // advances (a new incarnation), or when the source is cleared. A source
        // peer transition never changes (generation, source), so it is a no-op
        // here.
        let redeclare = match (&current, &desired_source) {
            (Some((current_generation, _, _)), Some(_)) => {
                *current_generation != desired_generation
            }
            (Some(_), None) => true,
            (None, Some(_)) => true,
            (None, None) => false,
        };
        if redeclare {
            // Drop-before-redeclare: the old wire subscription (and its buffered
            // messages) dies before the new generation's subscription exists.
            current = None;
            if let Some(pin) = desired_source {
                match TopicMessenger::subscribe_peer_pinned(
                    &messenger,
                    &as_core_node,
                    &as_instance_id,
                    pairing_target.clone(),
                    &pin.producer,
                    &pin.source_link_id,
                    &topic,
                    qos.clone(),
                )
                .await
                {
                    Ok(subscription) => {
                        current = Some((desired_generation, pin, subscription))
                    }
                    Err(err) => {
                        warn!(
                            %err,
                            topic = %topic,
                            "failed to declare observer wire subscription; slot stays silent until the next observation update"
                        );
                    }
                }
            }
        }
        match current.as_mut() {
            Some((generation, pin, subscription)) => {
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
                                let matches_pin = message.core_node() == pin.producer.core_node
                                    && message.instance_id() == pin.producer.instance_id
                                    && message.link_id() == pin.source_link_id;
                                if matches_pin
                                    && tx.send((*generation, pin.clone(), message)).await.is_err()
                                {
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

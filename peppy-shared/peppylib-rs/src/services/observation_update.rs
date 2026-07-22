//! Framework `observation_update` service: the daemon's live delivery channel
//! for observer-slot state (source pin, source generation, source liveness).
//! Registered pre-setup for the same reason as `peer_update` — user code may
//! block in `setup_fn` forever, and observation delivery must not depend on it.
//! Requests carry ABSOLUTE slot state with a sequence number; the handler is
//! idempotent and rejects strictly-stale sequences so a delayed retry can never
//! roll a slot back.
//!
//! Observation state is daemon-authoritative, so the only legitimate caller is
//! the node's own daemon, whose identity the node knows as its bound core_node.
//! Requests stamped with any other core_node are rejected before touching slot
//! state, exactly as `peer_update` does.

use crate::encoding::observation_update::{ObservationUpdateRequest, ObservationUpdateResponse};
use crate::messaging::{
    OBSERVATION_UPDATE_SERVICE, ObservationState, SenderTarget, ServiceRequestContext,
};
use crate::runtime::TaskHandle;
use crate::types::Payload;
use crate::{MessengerHandle, PeppyResult, ServiceMessenger};
use std::collections::BTreeMap;
use std::sync::Arc;
use tokio::sync::watch;
use tracing::{debug, warn};

/// Shared map of one watch channel per declared observer slot, keyed by the
/// node's own observer-slot link_id. Built once by the `Processor`; the map
/// itself is immutable (slots are declared in the manifest), only the channel
/// values move.
pub(crate) type ObservationSlotSenders = Arc<BTreeMap<String, watch::Sender<ObservationState>>>;

pub async fn listen_for_observation_update(
    messenger: &MessengerHandle,
    core_node: &str,
    instance_id: &str,
    as_identity: SenderTarget,
    slots: ObservationSlotSenders,
) -> PeppyResult<TaskHandle<PeppyResult<()>>> {
    let mut endpoint = ServiceMessenger::listen(
        messenger,
        core_node,
        instance_id,
        as_identity,
        OBSERVATION_UPDATE_SERVICE,
    )
    .await?;

    let daemon_core_node = core_node.to_string();
    let handle = crate::runtime::spawn(async move {
        endpoint
            .handle_requests(|context| {
                let slots = Arc::clone(&slots);
                let daemon_core_node = daemon_core_node.clone();
                async move { handle_observation_update_request(context, &daemon_core_node, slots) }
            })
            .await
    });
    Ok(handle)
}

fn handle_observation_update_request(
    context: ServiceRequestContext,
    daemon_core_node: &str,
    slots: ObservationSlotSenders,
) -> PeppyResult<Payload> {
    let caller_core_node = context.message().core_node();
    if caller_core_node != daemon_core_node {
        warn!(
            caller_core_node = %caller_core_node,
            caller_instance_id = %context.message().instance_id(),
            "observation_update from a caller outside this node's daemon; rejecting"
        );
        return ObservationUpdateResponse::rejected(format!(
            "observation_update is daemon-only: caller core_node '{caller_core_node}' is not this \
             node's daemon '{daemon_core_node}'"
        ))
        .encode();
    }
    let request = ObservationUpdateRequest::decode(&context.message().payload_bytes())?;
    debug!(
        link_id = %request.link_id,
        sequence = request.sequence,
        source_generation = request.source_generation,
        has_source = request.source.is_some(),
        source_live = request.source_live,
        "received observation_update from {}",
        context.message().instance_id(),
    );
    apply_observation_update(&slots, &request).encode()
}

/// Applies one absolute-state update to the slot's watch channel. Split from the
/// service handler so tests can drive it without a wire round-trip.
pub(crate) fn apply_observation_update(
    slots: &BTreeMap<String, watch::Sender<ObservationState>>,
    request: &ObservationUpdateRequest,
) -> ObservationUpdateResponse {
    let Some(sender) = slots.get(&request.link_id) else {
        warn!(
            link_id = %request.link_id,
            "observation_update names an undeclared observer slot; rejecting"
        );
        return ObservationUpdateResponse::rejected(format!(
            "unknown observer slot '{}'",
            request.link_id
        ));
    };
    let mut stale = false;
    sender.send_if_modified(|state| {
        if request.sequence < state.sequence {
            stale = true;
            return false;
        }
        // Absolute state: an equal sequence is an idempotent retry, a larger one
        // supersedes. Only notify watchers when something changed.
        let changed = state.sequence != request.sequence
            || state.source_generation != request.source_generation
            || state.source != request.source
            || state.source_live != request.source_live;
        state.sequence = request.sequence;
        state.source_generation = request.source_generation;
        state.source = request.source.clone();
        state.source_live = request.source_live;
        changed
    });
    if stale {
        ObservationUpdateResponse::stale()
    } else {
        ObservationUpdateResponse::accepted()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messaging::{ObservationPin, ProducerRef};

    fn slot_map(link_ids: &[&str]) -> BTreeMap<String, watch::Sender<ObservationState>> {
        link_ids
            .iter()
            .map(|id| {
                let (tx, _rx) = watch::channel(ObservationState::unregistered());
                (id.to_string(), tx)
            })
            .collect()
    }

    fn source(core: &str, inst: &str, source_link: &str) -> ObservationPin {
        ObservationPin {
            producer: ProducerRef::new(core, inst),
            source_link_id: source_link.to_string(),
        }
    }

    fn request(
        link_id: &str,
        sequence: u64,
        source: Option<ObservationPin>,
        source_generation: u64,
        source_live: bool,
    ) -> ObservationUpdateRequest {
        ObservationUpdateRequest {
            link_id: link_id.to_string(),
            sequence,
            source,
            source_generation,
            source_live,
        }
    }

    #[test]
    fn applies_source_then_advances_generation() {
        let slots = slot_map(&["observed_arm"]);
        let watched = slots["observed_arm"].subscribe();

        let first = apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                10,
                Some(source("core_a", "arm_1", "commander")),
                5,
                true,
            ),
        );
        assert!(first.accepted);
        assert_eq!(
            watched.borrow().source,
            Some(source("core_a", "arm_1", "commander"))
        );
        assert_eq!(watched.borrow().source_generation, 5);
        assert!(watched.borrow().source_live);

        // A restart under the same identity advances the generation.
        let second = apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                11,
                Some(source("core_a", "arm_1", "commander")),
                6,
                true,
            ),
        );
        assert!(second.accepted);
        assert_eq!(watched.borrow().source_generation, 6);
    }

    #[test]
    fn rejects_strictly_stale_sequence_without_rollback() {
        let slots = slot_map(&["observed_arm"]);
        let watched = slots["observed_arm"].subscribe();

        apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                20,
                Some(source("core_a", "arm_1", "commander")),
                5,
                true,
            ),
        );
        // A delayed earlier delivery arrives after the newer one.
        let response = apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                19,
                Some(source("core_a", "arm_1", "commander")),
                4,
                false,
            ),
        );
        assert!(!response.accepted);
        assert!(response.stale_sequence);
        assert_eq!(
            watched.borrow().source_generation,
            5,
            "stale request must not roll the slot back"
        );
    }

    #[test]
    fn equal_sequence_retry_is_idempotent_and_accepted() {
        let slots = slot_map(&["observed_arm"]);
        let mut watched = slots["observed_arm"].subscribe();

        apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                5,
                Some(source("core_a", "arm_1", "commander")),
                1,
                true,
            ),
        );
        assert!(watched.has_changed().unwrap());
        watched.mark_unchanged();

        let retry = apply_observation_update(
            &slots,
            &request(
                "observed_arm",
                5,
                Some(source("core_a", "arm_1", "commander")),
                1,
                true,
            ),
        );
        assert!(retry.accepted);
        assert!(
            !watched.has_changed().unwrap(),
            "an identical retry must not re-notify watchers"
        );
    }

    #[test]
    fn unknown_slot_is_rejected() {
        let slots = slot_map(&["observed_arm"]);
        let response = apply_observation_update(
            &slots,
            &request(
                "observed_gripper",
                1,
                Some(source("core_a", "g_1", "commander")),
                1,
                true,
            ),
        );
        assert!(!response.accepted);
        assert!(!response.stale_sequence);
        assert!(response.message.contains("observed_gripper"));
    }
}

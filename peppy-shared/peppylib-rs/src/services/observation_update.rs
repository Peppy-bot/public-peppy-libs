//! Framework `observation_update` service: the daemon's live delivery channel
//! for observer-slot state (source pin, source generation, source liveness).
//! Registered pre-setup — user code may block in `setup_fn` forever, and
//! observation delivery must not depend on it. The sequenced, daemon-only,
//! idempotent delivery protocol lives in [`crate::services::slot_update`]; this
//! module only maps an `ObservationUpdateRequest` onto an observer slot's
//! [`ObservationState`].
//!
//! Observation state is daemon-authoritative, so the only legitimate caller is
//! the node's own daemon, whose identity the node knows as its bound core_node.

use crate::encoding::observation_update::ObservationUpdateRequest;
use crate::messaging::{OBSERVATION_UPDATE_SERVICE, ObservationState, SenderTarget};
use crate::runtime::TaskHandle;
use crate::services::slot_update::{SlotSenders, SlotUpdate, listen_for_slot_update};
use crate::{MessengerHandle, PeppyResult};

/// Shared map of one watch channel per declared observer slot, keyed by the
/// node's own observer-slot link_id.
pub(crate) type ObservationSlotSenders = SlotSenders<ObservationState>;

impl SlotUpdate for ObservationUpdateRequest {
    type State = ObservationState;

    const SERVICE: &'static str = OBSERVATION_UPDATE_SERVICE;
    const UNKNOWN_SLOT_NOUN: &'static str = "observer slot";

    fn decode_request(payload: &[u8]) -> PeppyResult<Self> {
        ObservationUpdateRequest::decode(payload)
    }

    fn link_id(&self) -> &str {
        &self.link_id
    }

    fn sequence(&self) -> u64 {
        self.sequence
    }

    fn state_sequence(state: &ObservationState) -> u64 {
        state.sequence
    }

    fn merge_into(&self, state: &mut ObservationState) -> bool {
        let new_state = ObservationState {
            sequence: self.sequence,
            source_generation: self.source_generation,
            source: self.source.clone(),
            source_live: self.source_live,
        };
        let changed = *state != new_state;
        *state = new_state;
        changed
    }

    fn log_detail(&self) -> String {
        format!(
            "has_source={} source_generation={} source_live={}",
            self.source.is_some(),
            self.source_generation,
            self.source_live
        )
    }
}

pub async fn listen_for_observation_update(
    messenger: &MessengerHandle,
    core_node: &str,
    instance_id: &str,
    as_identity: SenderTarget,
    slots: ObservationSlotSenders,
) -> PeppyResult<TaskHandle<PeppyResult<()>>> {
    listen_for_slot_update::<ObservationUpdateRequest>(
        messenger,
        core_node,
        instance_id,
        as_identity,
        slots,
    )
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encoding::slot_update::SlotUpdateResponse;
    use crate::messaging::{ObservationPin, ProducerRef};
    use crate::services::slot_update::apply_slot_update;
    use std::collections::BTreeMap;
    use tokio::sync::watch;

    fn apply(
        slots: &BTreeMap<String, watch::Sender<ObservationState>>,
        request: &ObservationUpdateRequest,
    ) -> SlotUpdateResponse {
        apply_slot_update::<ObservationUpdateRequest>(slots, request)
    }

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

        let first = apply(
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
        let second = apply(
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

        apply(
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
        let response = apply(
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

        apply(
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

        let retry = apply(
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
        let response = apply(
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

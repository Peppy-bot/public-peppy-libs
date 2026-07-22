//! Framework `peer_update` service: the daemon's live delivery channel for
//! pairing-slot state (pair, re-pin, clear). Registered pre-setup — user code
//! may block in `setup_fn` forever, and pairing delivery must not depend on
//! it. The sequenced, daemon-only, idempotent delivery protocol lives in
//! [`crate::services::slot_update`]; this module only maps a `PeerUpdateRequest`
//! onto a pairing slot's [`PeerPinState`].
//!
//! Pairing state is daemon-authoritative: stacks are daemon-scoped, so the
//! only legitimate caller is the node's own daemon, whose identity the node
//! knows as its bound core_node. (Identity stamps are cooperative on the fabric
//! — this guards against misdirected or misbehaving callers; transport-level
//! access control remains the security boundary.)

use crate::encoding::peer_update::PeerUpdateRequest;
use crate::messaging::{PEER_UPDATE_SERVICE, PeerPinState, SenderTarget};
use crate::runtime::TaskHandle;
use crate::services::slot_update::{SlotSenders, SlotUpdate, listen_for_slot_update};
use crate::{MessengerHandle, PeppyResult};

/// Shared map of one watch channel per declared pairing slot, keyed by the
/// node's own slot link_id.
pub(crate) type PairingSlotSenders = SlotSenders<PeerPinState>;

impl SlotUpdate for PeerUpdateRequest {
    type State = PeerPinState;

    const SERVICE: &'static str = PEER_UPDATE_SERVICE;
    const UNKNOWN_SLOT_NOUN: &'static str = "pairing slot";

    fn decode_request(payload: &[u8]) -> PeppyResult<Self> {
        PeerUpdateRequest::decode(payload)
    }

    fn link_id(&self) -> &str {
        &self.link_id
    }

    fn sequence(&self) -> u64 {
        self.sequence
    }

    fn state_sequence(state: &PeerPinState) -> u64 {
        state.sequence
    }

    fn merge_into(&self, state: &mut PeerPinState) -> bool {
        let changed = state.sequence != self.sequence || state.pin != self.pin;
        state.sequence = self.sequence;
        state.pin = self.pin.clone();
        changed
    }

    fn log_detail(&self) -> String {
        format!("paired={}", self.pin.is_some())
    }
}

pub async fn listen_for_peer_update(
    messenger: &MessengerHandle,
    core_node: &str,
    instance_id: &str,
    as_identity: SenderTarget,
    slots: PairingSlotSenders,
) -> PeppyResult<TaskHandle<PeppyResult<()>>> {
    listen_for_slot_update::<PeerUpdateRequest>(
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
    use crate::messaging::{PeerPin, ProducerRef};
    use crate::services::slot_update::apply_slot_update;
    use std::collections::BTreeMap;
    use tokio::sync::watch;

    fn apply(
        slots: &BTreeMap<String, watch::Sender<PeerPinState>>,
        request: &PeerUpdateRequest,
    ) -> SlotUpdateResponse {
        apply_slot_update::<PeerUpdateRequest>(slots, request)
    }

    fn slot_map(link_ids: &[&str]) -> BTreeMap<String, watch::Sender<PeerPinState>> {
        link_ids
            .iter()
            .map(|id| {
                let (tx, _rx) = watch::channel(PeerPinState::unpaired());
                (id.to_string(), tx)
            })
            .collect()
    }

    fn pin(core: &str, inst: &str, peer_link: &str) -> PeerPin {
        PeerPin {
            producer: ProducerRef::new(core, inst),
            peer_link_id: peer_link.to_string(),
        }
    }

    fn request(link_id: &str, sequence: u64, pin: Option<PeerPin>) -> PeerUpdateRequest {
        PeerUpdateRequest {
            link_id: link_id.to_string(),
            sequence,
            pin,
        }
    }

    #[test]
    fn applies_pair_then_clear() {
        let slots = slot_map(&["arm"]);
        let watched = slots["arm"].subscribe();

        let paired = apply(
            &slots,
            &request("arm", 10, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(paired.accepted);
        assert_eq!(
            watched.borrow().pin,
            Some(pin("core_a", "arm_1", "controller"))
        );

        let cleared = apply(&slots, &request("arm", 11, None));
        assert!(cleared.accepted);
        assert_eq!(watched.borrow().pin, None);
        assert_eq!(watched.borrow().sequence, 11);
    }

    #[test]
    fn rejects_strictly_stale_sequence_without_rollback() {
        let slots = slot_map(&["arm"]);
        let watched = slots["arm"].subscribe();

        apply(
            &slots,
            &request("arm", 20, Some(pin("core_a", "arm_2", "controller"))),
        );
        // A delayed earlier delivery arrives after the newer one.
        let response = apply(
            &slots,
            &request("arm", 19, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(!response.accepted);
        assert!(response.stale_sequence);
        assert_eq!(
            watched.borrow().pin,
            Some(pin("core_a", "arm_2", "controller")),
            "stale request must not roll the slot back"
        );
    }

    #[test]
    fn equal_sequence_retry_is_idempotent_and_accepted() {
        let slots = slot_map(&["arm"]);
        let mut watched = slots["arm"].subscribe();

        apply(
            &slots,
            &request("arm", 5, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(watched.has_changed().unwrap());
        watched.mark_unchanged();

        let retry = apply(
            &slots,
            &request("arm", 5, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(retry.accepted);
        assert!(
            !watched.has_changed().unwrap(),
            "an identical retry must not re-notify watchers"
        );
    }

    #[test]
    fn unknown_slot_is_rejected() {
        let slots = slot_map(&["arm"]);
        let response = apply(
            &slots,
            &request("gripper", 1, Some(pin("core_a", "g_1", "controller"))),
        );
        assert!(!response.accepted);
        assert!(!response.stale_sequence);
        assert!(response.message.contains("gripper"));
    }
}

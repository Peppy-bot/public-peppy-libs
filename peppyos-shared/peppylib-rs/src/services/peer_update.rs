//! Framework `peer_update` service: the daemon's live delivery channel for
//! pairing-slot state (pair, re-pin, clear). Registered pre-setup — user code
//! may block in `setup_fn` forever, and pairing delivery must not depend on
//! it. Requests carry ABSOLUTE slot state with a sequence number; the handler
//! is idempotent and rejects strictly-stale sequences so a delayed retry can
//! never roll a slot back.

use crate::encoding::peer_update::{PeerUpdateRequest, PeerUpdateResponse};
use crate::messaging::{PEER_UPDATE_SERVICE, PeerPinState, SenderTarget, ServiceRequestContext};
use crate::runtime::TaskHandle;
use crate::types::Payload;
use crate::{MessengerHandle, PeppyResult, ServiceMessenger};
use std::collections::BTreeMap;
use std::sync::Arc;
use tokio::sync::watch;
use tracing::{debug, warn};

/// Shared map of one watch channel per declared pairing slot, keyed by the
/// node's own slot link_id. Built once by the `Processor`; the map itself is
/// immutable (slots are declared in the manifest), only the channel values
/// move.
pub(crate) type PairingSlotSenders = Arc<BTreeMap<String, watch::Sender<PeerPinState>>>;

pub async fn listen_for_peer_update(
    messenger: &MessengerHandle,
    core_node: &str,
    instance_id: &str,
    as_identity: SenderTarget,
    slots: PairingSlotSenders,
) -> PeppyResult<TaskHandle<PeppyResult<()>>> {
    let mut endpoint = ServiceMessenger::listen(
        messenger,
        core_node,
        instance_id,
        as_identity,
        PEER_UPDATE_SERVICE,
    )
    .await?;

    let handle = crate::runtime::spawn(async move {
        endpoint
            .handle_requests(|context| {
                let slots = Arc::clone(&slots);
                async move { handle_peer_update_request(context, slots) }
            })
            .await
    });
    Ok(handle)
}

fn handle_peer_update_request(
    context: ServiceRequestContext,
    slots: PairingSlotSenders,
) -> PeppyResult<Payload> {
    let request = PeerUpdateRequest::decode(&context.message().payload_bytes())?;
    debug!(
        link_id = %request.link_id,
        sequence = request.sequence,
        paired = request.pin.is_some(),
        "received peer_update from {}",
        context.message().instance_id(),
    );
    apply_peer_update(&slots, &request).encode()
}

/// Applies one absolute-state update to the slot's watch channel. Split from
/// the service handler so tests can drive it without a wire round-trip.
pub(crate) fn apply_peer_update(
    slots: &BTreeMap<String, watch::Sender<PeerPinState>>,
    request: &PeerUpdateRequest,
) -> PeerUpdateResponse {
    let Some(sender) = slots.get(&request.link_id) else {
        warn!(
            link_id = %request.link_id,
            "peer_update names an undeclared pairing slot; rejecting"
        );
        return PeerUpdateResponse::rejected(format!("unknown pairing slot '{}'", request.link_id));
    };
    let mut stale = false;
    sender.send_if_modified(|state| {
        if request.sequence < state.sequence {
            stale = true;
            return false;
        }
        // Absolute state: an equal sequence is an idempotent retry, a larger
        // one supersedes. Only notify watchers when something changed.
        let changed = state.pin != request.pin || state.sequence != request.sequence;
        state.sequence = request.sequence;
        state.pin = request.pin.clone();
        changed
    });
    if stale {
        PeerUpdateResponse::stale()
    } else {
        PeerUpdateResponse::accepted()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messaging::{PeerPin, ProducerRef};

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

        let paired = apply_peer_update(
            &slots,
            &request("arm", 10, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(paired.accepted);
        assert_eq!(
            watched.borrow().pin,
            Some(pin("core_a", "arm_1", "controller"))
        );

        let cleared = apply_peer_update(&slots, &request("arm", 11, None));
        assert!(cleared.accepted);
        assert_eq!(watched.borrow().pin, None);
        assert_eq!(watched.borrow().sequence, 11);
    }

    #[test]
    fn rejects_strictly_stale_sequence_without_rollback() {
        let slots = slot_map(&["arm"]);
        let watched = slots["arm"].subscribe();

        apply_peer_update(
            &slots,
            &request("arm", 20, Some(pin("core_a", "arm_2", "controller"))),
        );
        // A delayed earlier delivery arrives after the newer one.
        let response = apply_peer_update(
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

        apply_peer_update(
            &slots,
            &request("arm", 5, Some(pin("core_a", "arm_1", "controller"))),
        );
        assert!(watched.has_changed().unwrap());
        watched.mark_unchanged();

        let retry = apply_peer_update(
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
        let response = apply_peer_update(
            &slots,
            &request("gripper", 1, Some(pin("core_a", "g_1", "controller"))),
        );
        assert!(!response.accepted);
        assert!(!response.stale_sequence);
        assert!(response.message.contains("gripper"));
    }
}

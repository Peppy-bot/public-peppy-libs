//! The pieces of a simulated camera's relay that name no generated type: the
//! camera a relay is in its copy, the timestamp check of a frame it forwards,
//! why a call it forwards to the simulation's camera response model has no
//! answer, and the bounds and messages of the relay. The relay node maps each
//! onto its own generated contract members.

use std::fmt::Display;
use std::time::{Duration, SystemTime};

/// Bounds every call forwarded to the simulation's camera response model. A
/// control is one round trip through the simulation's device model, so an
/// answer later than this means the model is gone, and the caller gets an
/// error reply of the kind `timeout`.
pub const CONTROL_TIMEOUT: Duration = Duration::from_secs(5);

/// The message of the error reply every forwarded call (a control, the
/// profile, the reset) gets while the control slot is vacant: the launch
/// linked the relay to a simulation without a camera response model, so
/// there is nothing behind the controls to adjust, describe or reset.
pub const NO_RESPONSE_MODEL_MESSAGE: &str =
    "no camera response model is linked: nothing to adjust, describe or reset";

/// The message of the error reply a geometry service gives until the
/// simulation's first geometry arrives. The relay has no camera model of its
/// own: every pinhole model it answers is one the simulation sent.
pub const NO_GEOMETRY_MESSAGE: &str = "no camera geometry received from the simulation yet";

/// Whether a frame's capture time is after the Unix epoch. A simulation
/// stamps its frames at the epoch before its clock resolves, and no consumer
/// can age such a sample.
pub fn timestamp_is_valid(timestamp: SystemTime) -> bool {
    timestamp > SystemTime::UNIX_EPOCH
}

/// The camera a relay is, as the simulation names it: the relay's name in
/// its copy. A launch mints a copy's instance ids as `<copy>_<id>`, so relay
/// `alpha_wrist_left` of copy `alpha` is camera `wrist_left`, and a relay
/// launched outside a copy runs under the id the launcher wrote, which is the
/// camera's name as it is. The simulation's slot holds a pair per camera of
/// its kind, so the slot says nothing of which camera a pair is; the relay's
/// requests name no robot, which the simulation tells by the pair the relay
/// holds. `Err` says why the id names no camera.
pub fn camera_name(copy: Option<&str>, instance_id: &str) -> Result<String, String> {
    let Some(copy) = copy else {
        return Ok(instance_id.to_owned());
    };
    instance_id
        .strip_prefix(copy)
        .and_then(|rest| rest.strip_prefix('_'))
        .filter(|name| !name.is_empty())
        .map(str::to_owned)
        .ok_or_else(|| {
            format!(
                "this relay names no camera: its instance id {instance_id:?} is not `{copy}_` followed by the camera's name in copy {copy:?}"
            )
        })
}

/// How a forwarded call failed on its way to the model or back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportFailure {
    /// The model gave no answer within [`CONTROL_TIMEOUT`].
    TimedOut,
    /// No model answers on the control slot.
    Unreachable,
    /// The call failed in another way, such as a request or an answer that
    /// does not encode or decode.
    Broken,
}

/// Why a forwarded call has no answer of the simulation's camera response
/// model. Each cause gives the caller an error reply of its own kind.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Unanswered {
    /// The model gave an error reply. Its kind, by its wire name, and its
    /// message pass on as they came.
    Refused { kind: &'static str, message: String },
    /// No model is linked: the control slot is vacant.
    NoModel,
    /// The relay's instance id names no camera.
    NoCamera(String),
    /// The call failed on its way to the model or back.
    Transport {
        failure: TransportFailure,
        message: String,
    },
}

impl Unanswered {
    /// A call that failed in transport with `error`, as `failure` classes it.
    pub fn of_transport(failure: TransportFailure, error: impl Display) -> Self {
        Self::Transport {
            failure,
            message: format!(
                "forwarding to the simulation's camera response model failed: {error}"
            ),
        }
    }

    /// Whether the call failed on its way to the model or back. An error
    /// reply is an answer of the model, and a vacant slot or a relay that
    /// names no camera is fixed for the life of the node.
    pub fn in_transport(&self) -> bool {
        matches!(self, Self::Transport { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_relay_in_a_copy_is_the_camera_its_id_names_after_the_copy() {
        assert_eq!(
            camera_name(Some("alpha"), "alpha_wrist_left"),
            Ok("wrist_left".to_owned())
        );
        // Only the copy's own prefix comes off: a camera may carry the
        // copy's name in its own.
        assert_eq!(
            camera_name(Some("alpha"), "alpha_alpha_front"),
            Ok("alpha_front".to_owned())
        );
    }

    #[test]
    fn a_relay_outside_a_copy_is_the_camera_its_id_names() {
        assert_eq!(camera_name(None, "wrist_left"), Ok("wrist_left".to_owned()));
        assert_eq!(
            camera_name(None, "alpha_wrist_left"),
            Ok("alpha_wrist_left".to_owned())
        );
    }

    #[test]
    fn an_id_that_is_no_name_in_its_copy_names_no_camera() {
        // Another copy's prefix, the copy's name run into the camera's, the
        // copy's name alone, and the bare prefix.
        for instance_id in ["bravo_wrist_left", "alphawrist_left", "alpha", "alpha_"] {
            let unnamed = camera_name(Some("alpha"), instance_id).unwrap_err();
            assert!(
                unnamed.contains("names no camera") && unnamed.contains(instance_id),
                "{unnamed}"
            );
        }
    }

    #[test]
    fn timestamp_guard_rejects_epoch_and_earlier() {
        assert!(timestamp_is_valid(
            SystemTime::UNIX_EPOCH + Duration::from_secs(1)
        ));
        assert!(!timestamp_is_valid(SystemTime::UNIX_EPOCH));
        assert!(!timestamp_is_valid(
            SystemTime::UNIX_EPOCH - Duration::from_secs(1)
        ));
    }

    #[test]
    fn a_transport_failure_names_the_model_and_the_error() {
        let unanswered = Unanswered::of_transport(TransportFailure::TimedOut, "timed out");
        assert_eq!(
            unanswered,
            Unanswered::Transport {
                failure: TransportFailure::TimedOut,
                message: "forwarding to the simulation's camera response model failed: timed out"
                    .to_owned(),
            }
        );
        assert!(unanswered.in_transport());
    }

    #[test]
    fn only_a_transport_failure_is_in_transport() {
        let refused = Unanswered::Refused {
            kind: "rejected",
            message: "gain 9000 is above 255".to_owned(),
        };
        for fixed in [
            refused,
            Unanswered::NoModel,
            Unanswered::NoCamera("no camera".to_owned()),
        ] {
            assert!(!fixed.in_transport(), "{fixed:?}");
        }
    }
}

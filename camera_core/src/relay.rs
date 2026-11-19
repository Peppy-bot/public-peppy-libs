//! The pieces of a simulated camera's relay that name no generated type: why a
//! call it forwards to the simulation's camera response model has no answer,
//! and the bounds and messages of the relay. The relay node maps each onto its
//! own generated contract members; it holds a relayed value's frame to the
//! camera's with
//! [`check_relayed_frame`](crate::sensor_mounts::check_relayed_frame).

use std::fmt::Display;
use std::time::Duration;

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
    /// reply is an answer of the model, and a vacant slot is fixed for the
    /// life of the node.
    pub fn in_transport(&self) -> bool {
        matches!(self, Self::Transport { .. })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        for fixed in [refused, Unanswered::NoModel] {
            assert!(!fixed.in_transport(), "{fixed:?}");
        }
    }
}

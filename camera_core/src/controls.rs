//! Camera controls: why a write gave no value, and the auto/manual mode of a
//! control with a setpoint.

use std::io;

/// The value a control service answers when the control has no current value
/// to report, such as a control the driver runs in an automatic mode.
pub const VALUE_UNAVAILABLE: i32 = -1;

/// Why a control write gave no value: the failure kind of peppy that says
/// why, and the reason in words.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlFailure {
    pub kind: ControlFailureKind,
    pub message: String,
}

/// The failure kinds of peppy that a control write gives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlFailureKind {
    /// The device does not offer the control, or does not take the value.
    Rejected,
    /// The control does not take a write now: it is read-only, inactive in
    /// the current mode, or held by another application.
    PreconditionFailed,
    /// The device is gone.
    ResourceUnavailable,
    /// The write failed for another reason.
    AttemptFailed,
}

/// The error reply of `$service`, a generated service module, for a
/// [`ControlFailure`]: the generated failure kind of the same name, and the
/// message.
#[macro_export]
macro_rules! control_failure {
    ($service:ident, $failure:expr) => {{
        let $crate::controls::ControlFailure { kind, message } = $failure;
        let kind = match kind {
            $crate::controls::ControlFailureKind::Rejected => $service::FailureKind::Rejected,
            $crate::controls::ControlFailureKind::PreconditionFailed => {
                $service::FailureKind::PreconditionFailed
            }
            $crate::controls::ControlFailureKind::ResourceUnavailable => {
                $service::FailureKind::ResourceUnavailable
            }
            $crate::controls::ControlFailureKind::AttemptFailed => {
                $service::FailureKind::AttemptFailed
            }
        };
        $service::Failure::new(kind, message)
    }};
}

impl ControlFailureKind {
    /// The kind of a write the driver refused with `error`, by the errno
    /// that VIDIOC_S_EXT_CTRLS documents for it.
    fn of_refused_write(error: &io::Error) -> Self {
        match error.raw_os_error() {
            Some(libc::EINVAL | libc::ERANGE) => Self::Rejected,
            Some(libc::EACCES | libc::EBUSY) => Self::PreconditionFailed,
            Some(libc::ENODEV) => Self::ResourceUnavailable,
            _ => Self::AttemptFailed,
        }
    }
}

impl ControlFailure {
    /// A request the node does not take, such as an unknown mode.
    pub fn rejected(message: impl Into<String>) -> Self {
        Self {
            kind: ControlFailureKind::Rejected,
            message: message.into(),
        }
    }

    /// A write the driver refused with `error`, of the kind its errno names;
    /// `message` says which write and why.
    pub fn of_refused_write(error: &io::Error, message: String) -> Self {
        Self {
            kind: ControlFailureKind::of_refused_write(error),
            message,
        }
    }
}

/// The mode of a control that runs either on its own or at a setpoint
/// (exposure, white balance).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoManualMode {
    Auto,
    Manual,
}

impl AutoManualMode {
    /// The mode `name` spells exactly, `"auto"` or `"manual"`. Each node
    /// words its own refusal of any other name.
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "auto" => Some(Self::Auto),
            "manual" => Some(Self::Manual),
            _ => None,
        }
    }

    pub fn is_auto(self) -> bool {
        matches!(self, Self::Auto)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kind_of_errno(errno: i32) -> ControlFailureKind {
        ControlFailureKind::of_refused_write(&io::Error::from_raw_os_error(errno))
    }

    #[test]
    fn a_control_or_a_value_the_device_does_not_take_is_rejected() {
        assert_eq!(kind_of_errno(libc::EINVAL), ControlFailureKind::Rejected);
        assert_eq!(kind_of_errno(libc::ERANGE), ControlFailureKind::Rejected);
    }

    #[test]
    fn a_control_that_takes_no_write_now_fails_a_precondition() {
        assert_eq!(
            kind_of_errno(libc::EACCES),
            ControlFailureKind::PreconditionFailed
        );
        assert_eq!(
            kind_of_errno(libc::EBUSY),
            ControlFailureKind::PreconditionFailed
        );
    }

    #[test]
    fn a_device_that_is_gone_is_unavailable() {
        assert_eq!(
            kind_of_errno(libc::ENODEV),
            ControlFailureKind::ResourceUnavailable
        );
    }

    #[test]
    fn any_other_refusal_of_the_driver_is_a_failed_attempt() {
        assert_eq!(kind_of_errno(libc::EIO), ControlFailureKind::AttemptFailed);
        let without_errno = io::Error::other("no errno");
        assert_eq!(
            ControlFailureKind::of_refused_write(&without_errno),
            ControlFailureKind::AttemptFailed
        );
    }

    #[test]
    fn a_refused_write_carries_the_message_it_is_given() {
        let failure = ControlFailure::of_refused_write(
            &io::Error::from_raw_os_error(libc::ERANGE),
            "set gain: out of range".to_string(),
        );
        assert_eq!(failure.kind, ControlFailureKind::Rejected);
        assert_eq!(failure.message, "set gain: out of range");
    }

    #[test]
    fn a_rejection_carries_its_message() {
        assert_eq!(
            ControlFailure::rejected("unknown mode"),
            ControlFailure {
                kind: ControlFailureKind::Rejected,
                message: "unknown mode".to_string(),
            }
        );
    }

    #[test]
    fn the_modes_are_spelled_exactly() {
        assert_eq!(
            AutoManualMode::from_name("auto"),
            Some(AutoManualMode::Auto)
        );
        assert_eq!(
            AutoManualMode::from_name("manual"),
            Some(AutoManualMode::Manual)
        );
        for other in ["Auto", "AUTO", "Manual", "", "dim"] {
            assert_eq!(AutoManualMode::from_name(other), None, "{other:?}");
        }
    }

    #[test]
    fn only_auto_is_auto() {
        assert!(AutoManualMode::Auto.is_auto());
        assert!(!AutoManualMode::Manual.is_auto());
    }

    /// A generated service module, as `control_failure!` reads one.
    mod set_gain {
        #[derive(Debug, PartialEq)]
        pub enum FailureKind {
            Rejected,
            PreconditionFailed,
            ResourceUnavailable,
            AttemptFailed,
        }

        #[derive(Debug, PartialEq)]
        pub struct Failure {
            pub kind: FailureKind,
            pub message: String,
        }

        impl Failure {
            pub fn new(kind: FailureKind, message: impl Into<String>) -> Self {
                Self {
                    kind,
                    message: message.into(),
                }
            }
        }
    }

    #[test]
    fn a_control_failure_becomes_the_service_failure_of_the_same_kind() {
        let cases = [
            (
                ControlFailureKind::Rejected,
                set_gain::FailureKind::Rejected,
            ),
            (
                ControlFailureKind::PreconditionFailed,
                set_gain::FailureKind::PreconditionFailed,
            ),
            (
                ControlFailureKind::ResourceUnavailable,
                set_gain::FailureKind::ResourceUnavailable,
            ),
            (
                ControlFailureKind::AttemptFailed,
                set_gain::FailureKind::AttemptFailed,
            ),
        ];
        for (kind, expected) in cases {
            let failure = ControlFailure {
                kind,
                message: "gain 9000 is above 255".to_owned(),
            };
            assert_eq!(
                crate::control_failure!(set_gain, failure),
                set_gain::Failure::new(expected, "gain 9000 is above 255")
            );
        }
    }
}

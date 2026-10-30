//! The holder of the setup claim of a simulation through its applier
//! (world_applier:v1), as `get` names it and a refusal of another call
//! names it.

use serde::{Deserialize, Serialize};
use std::fmt;

/// The call of the applier that holds the setup claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimCall {
    Apply,
    Run,
}

impl ClaimCall {
    /// The name of the call in the `call` field.
    pub fn name(self) -> &'static str {
        match self {
            ClaimCall::Apply => "apply",
            ClaimCall::Run => "run",
        }
    }
}

impl fmt::Display for ClaimCall {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The failure kind with which the applier refuses or fails an apply, and
/// the first step of a run: the world at its check, a hold that ends (the
/// setup claim of another call, a robot that another client leases, the
/// halt), or a step after the check.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ApplyFailureKind {
    WorldRefused,
    Claimed,
    Leased,
    Halted,
    ApplyFailed,
}

impl ApplyFailureKind {
    /// Every kind, in the order the contract declares them.
    pub const ALL: [ApplyFailureKind; 5] = [
        ApplyFailureKind::WorldRefused,
        ApplyFailureKind::Claimed,
        ApplyFailureKind::Leased,
        ApplyFailureKind::Halted,
        ApplyFailureKind::ApplyFailed,
    ];

    /// The kind named `name`, as the contract writes it.
    pub fn parse(name: &str) -> Option<ApplyFailureKind> {
        Self::ALL.into_iter().find(|kind| kind.name() == name)
    }

    /// The name of the kind, as the contract writes it.
    pub fn name(self) -> &'static str {
        match self {
            ApplyFailureKind::WorldRefused => "world_refused",
            ApplyFailureKind::Claimed => "claimed",
            ApplyFailureKind::Leased => "leased",
            ApplyFailureKind::Halted => "halted",
            ApplyFailureKind::ApplyFailed => "apply_failed",
        }
    }

    /// Whether the kind is a hold that ends, after which the same apply
    /// can go again.
    pub fn is_a_hold(self) -> bool {
        matches!(
            self,
            ApplyFailureKind::Claimed | ApplyFailureKind::Leased | ApplyFailureKind::Halted
        )
    }
}

impl fmt::Display for ApplyFailureKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Who holds the setup claim through the applier: the call, the label of
/// its caller, and the run of the workflow director once it started, empty
/// before and for an apply.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClaimHolder {
    pub call: ClaimCall,
    pub started_by: String,
    pub run: String,
}

impl fmt::Display for ClaimHolder {
    /// "the run of 'Tidy the table (c2)' (run r3)", "the apply of 'eval'".
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "the {} of '{}'", self.call, self.started_by)?;
        if self.run.is_empty() {
            return Ok(());
        }
        write!(f, " (run {})", self.run)
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{ClaimCall, ClaimHolder};

    #[test]
    fn a_holder_names_its_call_its_caller_and_its_run_once_it_started() {
        let run = ClaimHolder {
            call: ClaimCall::Run,
            started_by: "Tidy the table (c2)".to_owned(),
            run: "r3".to_owned(),
        };
        assert_eq!(run.to_string(), "the run of 'Tidy the table (c2)' (run r3)");
        let apply = ClaimHolder {
            call: ClaimCall::Apply,
            started_by: "eval".to_owned(),
            run: String::new(),
        };
        assert_eq!(apply.to_string(), "the apply of 'eval'");
        assert_eq!(
            serde_json::to_value(&run).unwrap(),
            json!({ "call": "run", "started_by": "Tidy the table (c2)", "run": "r3" })
        );
        for call in [ClaimCall::Apply, ClaimCall::Run] {
            assert_eq!(serde_json::to_value(call).unwrap(), call.name());
        }
    }

    /// Each kind parses from the name it writes, which is the name the
    /// contract declares and the wire carries.
    #[test]
    fn an_apply_failure_kind_parses_from_the_name_it_writes() {
        use super::ApplyFailureKind;
        for kind in ApplyFailureKind::ALL {
            assert_eq!(ApplyFailureKind::parse(kind.name()), Some(kind));
            assert_eq!(
                serde_json::to_value(kind).unwrap(),
                serde_json::Value::String(kind.name().to_owned())
            );
        }
        assert_eq!(ApplyFailureKind::parse("rejected"), None);
        assert!(ApplyFailureKind::Claimed.is_a_hold());
        assert!(!ApplyFailureKind::WorldRefused.is_a_hold());
        assert!(!ApplyFailureKind::ApplyFailed.is_a_hold());
    }
}

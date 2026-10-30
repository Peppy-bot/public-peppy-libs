//! The documents of the workflow side.
//!
//! A workflow (`workflow/v1`, [`Workflow`]) says what robots do: its roles, the
//! contracts of their skills, its observations, assumptions and tables, and
//! one lane of steps per role. A run (`run/v1`, [`RunDocument`]) binds the
//! roles of one workflow revision to robots. A bundle (`workflow_bundle/v1`,
//! [`Bundle`]) carries a workflow out of the chat with the digest of each of
//! its files and the [`Lock`] of the versions it ran with.
//!
//! Each document carries the root key `workflow_schema`, and takes the root
//! key rule, the manifest and the digest from `documents_core`. The
//! expressions of a workflow are a subset of CEL ([`expression`]). The
//! structural rules of a workflow that need no contract hold for every
//! [`Workflow`]; the workflow director checks the rest against the contracts
//! the workflow names, reading the [`marks`] of their fields and members.

pub mod bundle;
pub mod event;
pub mod expression;
pub mod locations;
pub mod lock;
pub mod marks;
pub mod names;
pub mod refusal;
pub mod result;
pub mod run;
pub mod workflow;

pub use bundle::{Bundle, BundleFile};
pub use documents_core::{Revision, parse};
pub use event::{EventKind, RunEvent};
pub use expression::Expression;
pub use locations::{KnownLocation, LocationKind};
pub use lock::{Lock, LockedNode};
pub use names::{Bound, InstanceName, MemberRef, Name, NameTag, RobotName, Sha256, StepId};
pub use refusal::{At, Located, Refusal, Refusals};
pub use run::{Run, RunDocument, robots_named_twice, run_lease_name, run_of_lease_name};
pub use workflow::{Workflow, WorkflowDraft};

/// The language of `workflow/v1` for a model that writes workflows.
pub const LANGUAGE: &str = include_str!("../LANGUAGE.md");

#[cfg(test)]
mod tests {
    use super::LANGUAGE;

    #[test]
    fn the_language_names_each_step_kind_and_top_level_field() {
        let kinds = [
            "call",
            "sequence",
            "parallel",
            "if",
            "match",
            "for_each",
            "repeat_until",
            "wait_until",
            "signal",
            "wait_for",
        ];
        let fields = [
            "workflow_schema",
            "manifest",
            "task",
            "contracts",
            "roles",
            "locations",
            "tables",
            "observations",
            "assumptions",
            "success",
            "minimum",
            "initialization",
            "actions",
        ];
        for word in kinds.iter().chain(&fields) {
            assert!(LANGUAGE.contains(&format!("`{word}`")), "{word}");
        }
    }
}

//! The JSON texts that the workflow director gives with the events of a run
//! and its end: the run result, the suspension, the incident records and
//! the state of a run. Readers ignore fields they do not know, so a later
//! field leaves older readers working.

use documents_core::{Digest, Revision};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

use crate::lock::Lock;
use crate::names::{Name, NameTag, RobotName};
use crate::refusal::Located;
use crate::work_cell::{Uncovered, WorkCell};
use crate::workflow::Workflow;

/// How a run ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Ending {
    Completed,
    Degraded,
    Failed,
    Refused,
    Aborted,
    Halted,
    Interrupted,
}

/// The failure kinds of the goal of a run of `workflow_director:v1`: one
/// per ending that fails the goal, and the suspension.
const RUN_DEGRADED: &str = "run_degraded";
const RUN_FAILED: &str = "run_failed";
const RUN_REFUSED: &str = "run_refused";
const RUN_HALTED: &str = "run_halted";
const RUN_INTERRUPTED: &str = "run_interrupted";
pub const RUN_SUSPENDED: &str = "run_suspended";

impl Ending {
    /// The failure kind that the goal of a run that ended so fails with;
    /// none for a completed run, whose goal completes, and for an aborted
    /// one, whose goal ends cancelled.
    pub fn failure_kind(self) -> Option<&'static str> {
        match self {
            Ending::Completed | Ending::Aborted => None,
            Ending::Degraded => Some(RUN_DEGRADED),
            Ending::Failed => Some(RUN_FAILED),
            Ending::Refused => Some(RUN_REFUSED),
            Ending::Halted => Some(RUN_HALTED),
            Ending::Interrupted => Some(RUN_INTERRUPTED),
        }
    }
}

/// The state of a lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneState {
    Waiting,
    Running,
    Suspended,
    Completed,
    Failed,
    Aborted,
    Interrupted,
}

/// The workflow of a run, by name, revision and digest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunWorkflow {
    pub name: String,
    pub revision: Revision,
    pub digest: Digest,
}

impl RunWorkflow {
    /// The name, the revision and the digest of `workflow`.
    pub fn of(workflow: &Workflow) -> RunWorkflow {
        RunWorkflow {
            name: workflow.manifest.name.clone(),
            revision: workflow.manifest.revision,
            digest: workflow.digest(),
        }
    }
}

/// Where a lane stands, or how it ended.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LaneOutcome {
    pub role: Name,
    pub robot: RobotName,
    pub state: LaneState,
    /// `<role>.<id>` of its current or last step.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub step: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failure_kind: Option<String>,
}

/// Why a run was refused before its initialization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunRefusal {
    pub message: String,
    /// The errors of the checks that refused it.
    pub errors: Vec<Located>,
    /// The assumption whose check was false, with what it says.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub assumption: Option<Name>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub says: Option<String>,
}

/// The step that failed a lane.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FailedStep {
    pub step: String,
    pub failure_kind: String,
    pub message: String,
}

/// How an assumption was met.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AssumptionKind {
    Check,
    Reach,
    Confirmation,
}

/// An assumption of a run, its check's value or its confirmation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AssumptionOutcome {
    pub name: Name,
    pub says: String,
    pub kind: AssumptionKind,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub value: Option<bool>,
    pub confirmed: bool,
}

/// The passes of a loop.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LoopCount {
    pub step: String,
    pub passes: u32,
    pub failed: u32,
    /// Items where a `match` took `otherwise`.
    pub fallback: u32,
}

/// How long a run took, on its run clock and on wall time.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Duration {
    pub run_s: f64,
    pub wall_s: f64,
}

/// What a difference of the stack from the lock of a bundle concerns.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DifferenceKind {
    /// The peppy version.
    Peppy,
    /// The pin of a contract.
    Contract,
    /// The tag or the manifest of a node.
    Node,
    /// The commit of a node.
    NodeCommit,
    /// A node that a call step calls and the lock does not name.
    UnlockedNode,
}

impl DifferenceKind {
    pub const ALL: [DifferenceKind; 5] = [
        DifferenceKind::Peppy,
        DifferenceKind::Contract,
        DifferenceKind::Node,
        DifferenceKind::NodeCommit,
        DifferenceKind::UnlockedNode,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            DifferenceKind::Peppy => "peppy",
            DifferenceKind::Contract => "contract",
            DifferenceKind::Node => "node",
            DifferenceKind::NodeCommit => "node_commit",
            DifferenceKind::UnlockedNode => "unlocked_node",
        }
    }
}

impl fmt::Display for DifferenceKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One difference of the stack from the lock of a bundle.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Difference {
    pub kind: DifferenceKind,
    /// The contract or node, `name:tag`; empty for the peppy version.
    pub subject: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub locked: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub stack: Option<String>,
}

/// An anomaly that the monitor reports.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Anomaly {
    pub kind: String,
    pub severity: crate::run::Severity,
    pub description: String,
}

/// An alert that caused an incident, or the producer whose alert set went
/// stale.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AlertCause {
    pub source: String,
    pub kind: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub severity: Option<u8>,
    pub stale: bool,
}

/// The end of one call of a `safe_state`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SafeStateResult {
    pub step: String,
    #[serde(flatten)]
    pub outcome: SafeStateOutcome,
}

/// How a call of a `safe_state` ended: it completed, it failed with its
/// failure kind, or the run stopped it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "outcome", rename_all = "snake_case")]
pub enum SafeStateOutcome {
    Completed,
    Failed { failure_kind: String },
    Stopped,
}

/// An incident: a failure that fails or suspends a lane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct IncidentRecord {
    pub step: String,
    pub lane: Name,
    pub failure_kind: String,
    pub message: String,
    pub observations: BTreeMap<String, serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anomaly: Option<Anomaly>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub alert: Option<AlertCause>,
    pub affected_lanes: Vec<Name>,
    pub safe_state: Vec<SafeStateResult>,
}

/// The result of a run: its ending and what it did, with no ground truth.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RunResult {
    pub ending: Ending,
    /// The workflow of the run; none for a run refused before its workflow
    /// read.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub workflow: Option<RunWorkflow>,
    pub lanes: Vec<LaneOutcome>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refusal: Option<RunRefusal>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub failed: Option<FailedStep>,
    pub assumptions: Vec<AssumptionOutcome>,
    pub loops: Vec<LoopCount>,
    /// The last value of each observation.
    pub observations: BTreeMap<String, serde_json::Value>,
    pub duration: Duration,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub caller_data: Option<serde_json::Value>,
    pub core_node: String,
    pub versions: Lock,
    /// The differences from the lock of a bundle that `versions: "warn"` let
    /// pass.
    pub differences: Vec<Difference>,
    /// The nodes the run used that the lock does not name and no call step
    /// calls.
    pub unlocked_nodes: Vec<NameTag>,
    pub incidents: Vec<IncidentRecord>,
    /// The work cell block of the run.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub work_cell: WorkCell,
    /// The value of each watch parameter that the director read.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub watch: BTreeMap<Name, f64>,
    /// Each value of the work cell and each watch value that no test
    /// covered.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub uncovered: Vec<Uncovered>,
}

/// A lane of a suspended run and the step it suspended at.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SuspendedLane {
    pub role: Name,
    pub step: String,
}

/// A suspended run: its suspended lanes and their incidents, and why a
/// resume was refused when it was.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Suspension {
    pub lanes: Vec<SuspendedLane>,
    pub incidents: Vec<IncidentRecord>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub resume_refusal: Option<Located>,
}

/// A check of a run of a trial with its value, which the operator compares
/// with the work cell.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CheckToJudge {
    pub assumption: Name,
    pub says: String,
    pub check: String,
    pub value: bool,
}

/// An assumption that waits for a confirmation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ToConfirm {
    pub assumption: Name,
    pub says: String,
}

/// The state of a run, as `status` and the `progress` events give it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RunState {
    Preparing,
    Confirming,
    Running,
    Suspended,
    Ending,
    Completed,
    Degraded,
    Failed,
    Refused,
    Aborted,
    Halted,
    Interrupted,
}

impl RunState {
    /// Whether the run ended: a state of its end, never one it leaves.
    pub fn has_ended(self) -> bool {
        matches!(
            self,
            RunState::Completed
                | RunState::Degraded
                | RunState::Failed
                | RunState::Refused
                | RunState::Aborted
                | RunState::Halted
                | RunState::Interrupted
        )
    }
}

impl From<Ending> for RunState {
    fn from(ending: Ending) -> RunState {
        match ending {
            Ending::Completed => RunState::Completed,
            Ending::Degraded => RunState::Degraded,
            Ending::Failed => RunState::Failed,
            Ending::Refused => RunState::Refused,
            Ending::Aborted => RunState::Aborted,
            Ending::Halted => RunState::Halted,
            Ending::Interrupted => RunState::Interrupted,
        }
    }
}

/// The state of a run with what a caller needs to act on it: the assumptions that wait for a confirmation, the
/// suspension of a suspended run, with its lanes, its incidents and the refusal of a resume, and the result of a run
/// that ended.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RunStatus {
    pub state: RunState,
    pub to_confirm: Vec<ToConfirm>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub suspension: Option<Suspension>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<RunResult>,
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{DifferenceKind, Ending, RunResult, RunState, SafeStateOutcome, SafeStateResult};

    /// A run result reads and writes the fields its readers read, each
    /// under its name, and a missing field is refused. The result of a run
    /// refused before its workflow read has no workflow.
    #[test]
    fn a_run_result_reads_and_writes_the_fields_its_readers_read() {
        let text = json!({
            "ending": "completed",
            "workflow": { "name": "clear_table", "revision": 1, "digest": "c".repeat(64) },
            "lanes": [{ "role": "arm", "robot": "alpha", "state": "completed", "step": "arm.home" }],
            "assumptions": [{ "name": "bench_clear", "says": "Nothing is on the bench", "kind": "confirmation", "confirmed": true }],
            "loops": [{ "step": "arm.each", "passes": 2, "failed": 0, "fallback": 0 }],
            "observations": { "crate": { "found": true } },
            "duration": { "run_s": 4.0, "wall_s": 4.5 },
            "core_node": "cn-desk",
            "versions": {
                "contracts": { "postures:v1": "a".repeat(64) },
                "peppy": "0.36.0",
                "nodes": { "brain:v1": { "sha256": "b".repeat(64) } },
            },
            "differences": [{ "kind": "node_commit", "subject": "brain:v1", "locked": "c1" }],
            "unlocked_nodes": [],
            "incidents": [],
        });
        let result: RunResult = serde_json::from_value(text.clone()).unwrap();
        assert_eq!(serde_json::to_value(&result).unwrap(), text);
        let mut unread = text.clone();
        unread["ending"] = json!("refused");
        unread.as_object_mut().unwrap().remove("workflow");
        let result: RunResult = serde_json::from_value(unread.clone()).unwrap();
        assert_eq!(result.workflow, None);
        assert_eq!(serde_json::to_value(&result).unwrap(), unread);
        let mut missing = text;
        missing.as_object_mut().unwrap().remove("incidents");
        assert!(serde_json::from_value::<RunResult>(missing).is_err());
    }

    /// A run has ended exactly in the states that an ending gives.
    #[test]
    fn a_run_has_ended_in_the_state_of_each_ending_alone() {
        let endings = [
            Ending::Completed,
            Ending::Degraded,
            Ending::Failed,
            Ending::Refused,
            Ending::Aborted,
            Ending::Halted,
            Ending::Interrupted,
        ];
        let ended: Vec<RunState> = endings.into_iter().map(RunState::from).collect();
        let going = [
            RunState::Preparing,
            RunState::Confirming,
            RunState::Running,
            RunState::Suspended,
            RunState::Ending,
        ];
        assert!(ended.iter().all(|state| state.has_ended()));
        assert!(going.iter().all(|state| !state.has_ended()));
    }

    #[test]
    fn a_difference_kind_prints_as_its_name() {
        for kind in DifferenceKind::ALL {
            assert_eq!(
                serde_json::to_value(kind).unwrap(),
                kind.as_str(),
                "{kind:?}"
            );
        }
    }

    #[test]
    fn a_safe_state_result_names_its_outcome_and_the_failure_kind_of_a_failure() {
        let failed = SafeStateResult {
            step: "arm.home".to_owned(),
            outcome: SafeStateOutcome::Failed {
                failure_kind: "timeout".to_owned(),
            },
        };
        let text = json!({ "step": "arm.home", "outcome": "failed", "failure_kind": "timeout" });
        assert_eq!(serde_json::to_value(&failed).unwrap(), text);
        assert_eq!(
            serde_json::from_value::<SafeStateResult>(text).unwrap(),
            failed
        );
        let completed = json!({ "step": "arm.home", "outcome": "completed" });
        assert_eq!(
            serde_json::from_value::<SafeStateResult>(completed)
                .unwrap()
                .outcome,
            SafeStateOutcome::Completed
        );
    }

    #[test]
    fn a_failing_ending_fails_the_goal_with_the_kind_of_its_name() {
        let endings = [
            Ending::Completed,
            Ending::Degraded,
            Ending::Failed,
            Ending::Refused,
            Ending::Aborted,
            Ending::Halted,
            Ending::Interrupted,
        ];
        for ending in endings {
            let name = serde_json::to_value(ending).unwrap();
            let expected = match ending {
                Ending::Completed | Ending::Aborted => None,
                _ => Some(format!("run_{}", name.as_str().unwrap())),
            };
            assert_eq!(
                ending.failure_kind().map(str::to_owned),
                expected,
                "{ending:?}"
            );
        }
    }
}

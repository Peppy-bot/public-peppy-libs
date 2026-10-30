//! The events of a run, as the feedback of `run`, `resume` and `abort` of
//! workflow_director:v1 and its topic `run_events` give them: each names its
//! run, its kind, the role and the step it concerns, the failure kind where
//! one applies, a message, and its details as JSON text.

use serde::{Deserialize, Serialize};

use crate::result::{RunResult, Suspension, ToConfirm};

/// What an event of a run says happened.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EventKind {
    RunStarts,
    ToConfirm,
    Differences,
    InitializationStarts,
    ActionsStart,
    StepStarts,
    StepEnds,
    LaneSuspends,
    LaneResumes,
    LaneEnds,
    Incident,
    RunSuspends,
    RunEnds,
    /// The heartbeat of a run: how it stands, at least every 5 s.
    Progress,
}

impl EventKind {
    pub const ALL: [EventKind; 14] = [
        EventKind::RunStarts,
        EventKind::ToConfirm,
        EventKind::Differences,
        EventKind::InitializationStarts,
        EventKind::ActionsStart,
        EventKind::StepStarts,
        EventKind::StepEnds,
        EventKind::LaneSuspends,
        EventKind::LaneResumes,
        EventKind::LaneEnds,
        EventKind::Incident,
        EventKind::RunSuspends,
        EventKind::RunEnds,
        EventKind::Progress,
    ];

    /// The kind that `name` names in the `event` field.
    pub fn parse(name: &str) -> Option<EventKind> {
        EventKind::ALL.into_iter().find(|kind| kind.name() == name)
    }

    /// The name of the kind in the `event` field.
    pub fn name(self) -> &'static str {
        match self {
            EventKind::RunStarts => "run_starts",
            EventKind::ToConfirm => "to_confirm",
            EventKind::Differences => "differences",
            EventKind::InitializationStarts => "initialization_starts",
            EventKind::ActionsStart => "actions_start",
            EventKind::StepStarts => "step_starts",
            EventKind::StepEnds => "step_ends",
            EventKind::LaneSuspends => "lane_suspends",
            EventKind::LaneResumes => "lane_resumes",
            EventKind::LaneEnds => "lane_ends",
            EventKind::Incident => "incident",
            EventKind::RunSuspends => "run_suspends",
            EventKind::RunEnds => "run_ends",
            EventKind::Progress => "progress",
        }
    }
}

/// One event of a run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunEvent {
    pub run: String,
    pub role: String,
    pub event: EventKind,
    pub step: String,
    pub failure_kind: String,
    pub message: String,
    pub details: String,
}

impl RunEvent {
    /// Whether the event is the heartbeat of its run, which says no more
    /// than how the run stands.
    pub fn is_heartbeat(&self) -> bool {
        self.event == EventKind::Progress
    }

    /// The run result of a `run_ends` event.
    pub fn run_result(&self) -> Option<RunResult> {
        self.details_of(EventKind::RunEnds)
    }

    /// The suspension of a `run_suspends` event.
    pub fn suspension(&self) -> Option<Suspension> {
        self.details_of(EventKind::RunSuspends)
    }

    /// The assumptions that wait for a confirmation, of a `to_confirm`
    /// event.
    pub fn to_confirm(&self) -> Option<Vec<ToConfirm>> {
        self.details_of(EventKind::ToConfirm)
    }

    /// The details of an event of `kind`, when they read as `T`.
    fn details_of<T: for<'de> Deserialize<'de>>(&self, kind: EventKind) -> Option<T> {
        (self.event == kind)
            .then(|| serde_json::from_str(&self.details).ok())
            .flatten()
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::{EventKind, RunEvent};

    #[test]
    fn an_event_kind_reads_and_prints_as_its_name() {
        for kind in EventKind::ALL {
            assert_eq!(serde_json::to_value(kind).unwrap(), kind.name(), "{kind:?}");
            assert_eq!(EventKind::parse(kind.name()), Some(kind));
        }
        assert_eq!(EventKind::parse("suspends"), None);
    }

    #[test]
    fn the_details_of_an_event_read_only_for_its_kind() {
        let suspends = |event: &str| {
            serde_json::from_value::<RunEvent>(json!({
                "run": "r1", "role": "", "event": event, "step": "", "failure_kind": "run_suspended",
                "message": "", "details": json!({ "lanes": [], "incidents": [] }).to_string(),
            }))
            .unwrap()
        };
        let suspension = suspends("run_suspends").suspension().unwrap();
        assert!(suspension.lanes.is_empty() && suspension.resume_refusal.is_none());
        assert_eq!(suspends("lane_suspends").suspension(), None);
        assert_eq!(suspends("run_suspends").run_result(), None);
    }
}

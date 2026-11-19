//! Transform lookup. The reference is tf2's buffer (`tf2::BufferCore`): it takes the
//! scenario's transforms in order, a moving transform at its stamp, a static transform as
//! tf2_ros's static broadcaster sends it, with every static transform sent before it, and
//! answers each lookup of the scenario with the source frame's transform in the target
//! frame and the time it holds at, or with the error it throws, and each lookup through a
//! fixed frame, tf2's advanced `lookupTransform`, likewise.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a transform lookup scenario and of its reference output, which refuses
/// every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "transform_lookup")]
    TransformLookup,
}

/// The window of the buffer, in nanoseconds, and the steps it takes in order.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    cache_window_ns: u64,
    steps: Vec<Step>,
}

/// One step of a scenario: a transform sent moving at its stamp or static, a lookup, or a
/// lookup through a fixed frame.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "step", rename_all = "snake_case", deny_unknown_fields)]
enum Step {
    Moving {
        stamp: u64,
        parent: String,
        child: String,
        position: [f64; 3],
        orientation: [f64; 4],
    },
    Static {
        parent: String,
        child: String,
        position: [f64; 3],
        orientation: [f64; 4],
    },
    /// A lookup of `source` in `target` at `time`, in nanoseconds, or at the latest time
    /// where `time` is null.
    Lookup {
        target: String,
        source: String,
        time: Option<u64>,
    },
    /// A lookup of `source` at `source_time` in `target` at `target_time` through `fixed`,
    /// each time in nanoseconds or the latest time where it is null.
    LookupThrough {
        target: String,
        target_time: Option<u64>,
        source: String,
        source_time: Option<u64>,
        fixed: String,
    },
}

/// tf2's answer to each step of the scenario, in its order.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    steps: Vec<Answer>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Answer {
    Taken(Taken),
    Found(Found),
    Refused(Refused),
}

/// Whether `setTransform` took the transform a step sent.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Taken {
    taken: bool,
}

/// The source frame's transform in the target frame, its position in meters and its
/// orientation a quaternion [x, y, z, w], and the time it holds at, in nanoseconds, null for
/// tf2's time zero.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Found {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub time: Option<u64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Refused {
    error: Tf2Error,
}

/// The exception tf2's lookup throws. tf2 throws a `LookupException` for a frame it does
/// not hold and for a loop in the tree; the reference tells them apart by its message.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Tf2Error {
    UnknownFrame,
    Loop,
    Connectivity,
    ForwardExtrapolation,
    BackwardExtrapolation,
    NoDataForExtrapolation,
    InvalidArgument,
}

/// A transform a step sent, as the scenario names its frames.
#[derive(Debug, Clone, PartialEq)]
pub struct SentTransform {
    pub parent: String,
    pub child: String,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// A step of a scenario paired with tf2's answer to it.
#[derive(Debug, Clone)]
pub enum Case {
    /// A transform sent moving at `stamp`, in nanoseconds, and whether tf2 took it.
    Moving {
        stamp: u64,
        transform: SentTransform,
        taken: bool,
    },
    /// A transform sent static, and whether tf2 took it, sent again with every static
    /// transform before it.
    Static {
        transform: SentTransform,
        taken: bool,
    },
    /// A lookup of `source` in `target` at `time`, in nanoseconds, or at the latest time,
    /// and tf2's answer.
    Lookup {
        target: String,
        source: String,
        time: Option<u64>,
        answer: Result<Found, Tf2Error>,
    },
    /// A lookup of `source` at `source_time` in `target` at `target_time` through `fixed`,
    /// each time in nanoseconds or the latest time, and tf2's answer.
    LookupThrough {
        target: String,
        target_time: Option<u64>,
        source: String,
        source_time: Option<u64>,
        fixed: String,
        answer: Result<Found, Tf2Error>,
    },
}

/// A transform lookup scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub cache_window_ns: u64,
    pub cases: Vec<Case>,
}

impl Cases {
    /// Every transform lookup scenario under `scenarios/`, in the order of their names,
    /// each step paired with tf2's answer to it.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::TransformLookup)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::TransformLookup,
            cache_window_ns,
            steps,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::TransformLookup,
            steps: answers,
        } = read_reference(&name);
        let cases = pair(&name, steps, answers);
        Cases {
            name,
            cache_window_ns,
            cases,
        }
    }
}

/// tf2's answer to a step: whether it took a transform, or its answer to a lookup.
enum Answered {
    Taken(bool),
    Looked(Result<Found, Tf2Error>),
}

impl From<Answer> for Answered {
    fn from(answer: Answer) -> Self {
        match answer {
            Answer::Taken(Taken { taken }) => Answered::Taken(taken),
            Answer::Found(found) => Answered::Looked(Ok(found)),
            Answer::Refused(Refused { error }) => Answered::Looked(Err(error)),
        }
    }
}

fn pair(name: &str, steps: Vec<Step>, answers: Vec<Answer>) -> Vec<Case> {
    assert_eq!(steps.len(), answers.len(), "{name}: one answer per step");
    steps
        .into_iter()
        .zip(answers.into_iter().map(Answered::from))
        .enumerate()
        .map(|(index, step)| match step {
            (
                Step::Moving {
                    stamp,
                    parent,
                    child,
                    position,
                    orientation,
                },
                Answered::Taken(taken),
            ) => Case::Moving {
                stamp,
                transform: SentTransform {
                    parent,
                    child,
                    position,
                    orientation,
                },
                taken,
            },
            (
                Step::Static {
                    parent,
                    child,
                    position,
                    orientation,
                },
                Answered::Taken(taken),
            ) => Case::Static {
                transform: SentTransform {
                    parent,
                    child,
                    position,
                    orientation,
                },
                taken,
            },
            (
                Step::Lookup {
                    target,
                    source,
                    time,
                },
                Answered::Looked(answer),
            ) => Case::Lookup {
                target,
                source,
                time,
                answer,
            },
            (
                Step::LookupThrough {
                    target,
                    target_time,
                    source,
                    source_time,
                    fixed,
                },
                Answered::Looked(answer),
            ) => Case::LookupThrough {
                target,
                target_time,
                source,
                source_time,
                fixed,
                answer,
            },
            (step, _) => panic!("{name}: step {index}, {step:?}, has an answer of another step"),
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_or_with_an_unnamed_field_is_refused() {
        let scenario = |text: &str| serde_json::from_str::<Scenario>(text).is_ok();
        assert!(scenario(
            r#"{"kind": "transform_lookup", "cache_window_ns": 1, "steps": []}"#
        ));
        assert!(!scenario(
            r#"{"kind": "forward_kinematics", "cache_window_ns": 1, "steps": []}"#
        ));
        assert!(!scenario(
            r#"{"kind": "transform_lookup", "cache_window_ns": 1, "steps": [], "seed": 1}"#
        ));
        assert!(!scenario(
            r#"{"kind": "transform_lookup", "cache_window_ns": 1, "steps": [
                {"step": "lookup", "target": "a", "source": "b", "time": null, "seed": 1}
            ]}"#
        ));
    }

    #[test]
    #[should_panic(expected = "has an answer of another step")]
    fn a_lookup_answered_as_a_transform_does_not_pair() {
        let lookup = Step::Lookup {
            target: "a".into(),
            source: "b".into(),
            time: None,
        };
        pair(
            "scenario",
            vec![lookup],
            vec![Answer::Taken(Taken { taken: true })],
        );
    }

    #[test]
    fn every_transform_lookup_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a transform lookup scenario"
        );
        for scenario in scenarios {
            assert!(
                scenario
                    .cases
                    .iter()
                    .any(|case| matches!(case, Case::Lookup { .. })),
                "{} holds a lookup",
                scenario.name
            );
        }
    }
}

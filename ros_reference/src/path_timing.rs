//! Path timing. The reference is MoveIt's time-optimal trajectory generation (TOTG):
//! it blends the corners of a path through waypoints at its maximum deviation and times
//! the path under velocity and acceleration limits, or refuses a path that turns back
//! on itself.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a path timing scenario and of its reference output, which refuses every
/// other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "path_timing")]
    PathTiming,
}

/// The paths to blend and time.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    paths: Vec<PathInput>,
}

/// A path through `waypoints`, blended at `max_deviation` and timed under the limits;
/// TOTG gives the point at each of `fractions` of its length.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PathInput {
    name: String,
    waypoints: Vec<Vec<f64>>,
    max_deviation: f64,
    max_velocity: Vec<f64>,
    max_acceleration: Vec<f64>,
    fractions: Vec<f64>,
}

/// For each path of the scenario, in its order, what TOTG made of it.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    paths: Vec<PathOutput>,
}

/// What TOTG made of a path of the scenario, under the path's name: a refusal, or a timed
/// path. An output holds the fields of one alone.
#[derive(Deserialize)]
#[serde(untagged)]
enum PathOutput {
    Refused(RefusedOutput),
    Timed(TimedOutput),
}

/// A path that TOTG refused.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct RefusedOutput {
    name: String,
    refused: Refusal,
}

/// A path that TOTG timed: the blended path's length, its point at each fraction of the
/// length, and its duration (s).
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TimedOutput {
    name: String,
    length: f64,
    configs: Vec<Vec<f64>>,
    duration: f64,
}

/// The `refused` field of a refusal, which `true` alone fills.
struct Refusal;

impl<'de> Deserialize<'de> for Refusal {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        if bool::deserialize(deserializer)? {
            return Ok(Self);
        }
        Err(serde::de::Error::custom("refused is true on a refusal"))
    }
}

impl PathOutput {
    /// The name of the path, and what TOTG made of it.
    fn read(self) -> (String, Totg) {
        match self {
            Self::Refused(RefusedOutput {
                name,
                refused: Refusal,
            }) => (name, Totg::Refused),
            Self::Timed(TimedOutput {
                name,
                length,
                configs,
                duration,
            }) => (
                name,
                Totg::Timed {
                    length,
                    configs,
                    duration_s: duration,
                },
            ),
        }
    }
}

/// What TOTG made of a path.
#[derive(PartialEq)]
pub enum Totg {
    /// It refused the path: it turns back on itself at a waypoint.
    Refused,
    /// The blended path's length, its point at each fraction of the length, and the
    /// time-optimal duration under the limits.
    Timed {
        length: f64,
        configs: Vec<Vec<f64>>,
        duration_s: f64,
    },
}

/// One path of a scenario and what TOTG made of it.
pub struct PathCase {
    pub name: String,
    pub waypoints: Vec<Vec<f64>>,
    pub max_deviation: f64,
    pub max_velocity: Vec<f64>,
    pub max_acceleration: Vec<f64>,
    pub fractions: Vec<f64>,
    pub totg: Totg,
}

/// A path timing scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub paths: Vec<PathCase>,
}

impl Cases {
    /// Every path timing scenario under `scenarios/`, in the order of their names, each
    /// paired with its reference output: one output per path, which holds TOTG's refusal
    /// of it, or its length, one point per fraction and its duration.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::PathTiming)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::PathTiming,
            paths,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::PathTiming,
            paths: outputs,
        } = read_reference(&name);
        assert_eq!(paths.len(), outputs.len(), "{name}: one output per path");
        let paths = paths
            .into_iter()
            .zip(outputs)
            .map(|(input, output)| pair(&name, input, output))
            .collect();
        Cases { name, paths }
    }
}

fn pair(name: &str, input: PathInput, output: PathOutput) -> PathCase {
    let context = format!("{name} {}", input.name);
    let (output_name, totg) = output.read();
    assert_eq!(
        input.name, output_name,
        "{context}: the paths are in one order"
    );
    if let Totg::Timed { configs, .. } = &totg {
        assert_eq!(
            configs.len(),
            input.fractions.len(),
            "{context}: one point per fraction"
        );
    }
    PathCase {
        name: input.name,
        waypoints: input.waypoints,
        max_deviation: input.max_deviation,
        max_velocity: input.max_velocity,
        max_acceleration: input.max_acceleration,
        fractions: input.fractions,
        totg,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const REFUSED: &str = r#""refused": true"#;

    fn input() -> PathInput {
        PathInput {
            name: "a".into(),
            waypoints: vec![vec![0.0], vec![1.0]],
            max_deviation: 0.1,
            max_velocity: vec![1.0],
            max_acceleration: vec![1.0],
            fractions: vec![0.0],
        }
    }

    /// The output of the path named `name`, with `outcome`.
    fn output_of(name: &str, outcome: &str) -> PathOutput {
        serde_json::from_str(&format!(r#"{{"name": "{name}", {outcome}}}"#)).expect("parses")
    }

    /// The output of the path of [`input`], with `outcome`.
    fn output(outcome: &str) -> PathOutput {
        output_of("a", outcome)
    }

    /// A timed outcome with the points `configs`.
    fn timed(configs: &str) -> String {
        format!(r#""length": 1, "configs": {configs}, "duration": 2"#)
    }

    #[test]
    fn a_path_pairs_with_its_output() {
        assert!(pair("scenario", input(), output(REFUSED)).totg == Totg::Refused);
        assert!(matches!(
            pair("scenario", input(), output(&timed("[[0]]"))).totg,
            Totg::Timed { .. }
        ));
    }

    /// Whether `fields`, beside the path's name, parse as an output.
    fn parses(fields: &str) -> bool {
        serde_json::from_str::<PathOutput>(&format!(r#"{{"name": "a", {fields}}}"#)).is_ok()
    }

    #[test]
    fn an_output_holds_the_fields_of_one_outcome_alone() {
        assert!(parses(REFUSED) && parses(&timed("[[0]]")));
        assert!(
            !parses(&format!(r#"{REFUSED}, "note": 1"#)),
            "an unknown field"
        );
        assert!(
            !parses(&format!("{REFUSED}, {}", timed("[[0]]"))),
            "both outcomes"
        );
        assert!(!parses(r#""length": 1"#), "part of a timed path");
        assert!(!parses(r#""refused": false"#), "a refusal of nothing");
    }

    #[test]
    #[should_panic(expected = "the paths are in one order")]
    fn the_output_of_another_path_does_not_pair() {
        pair("scenario", input(), output_of("b", REFUSED));
    }

    #[test]
    #[should_panic(expected = "one point per fraction")]
    fn an_output_missing_a_point_does_not_pair() {
        pair("scenario", input(), output(&timed("[]")));
    }

    #[test]
    fn every_path_timing_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a path timing scenario"
        );
        for cases in scenarios {
            assert!(!cases.paths.is_empty(), "{} holds a path", cases.name);
        }
    }
}

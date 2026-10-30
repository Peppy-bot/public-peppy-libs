//! Sign-off: the thresholds that a sweep sets and each trial of its bundle
//! meets, and the four measures over the runs of a sweep or a trial.
//!
//! A count threshold (`unsafe_events_at_most`,
//! `false_assumption_checks_at_most`) bounds a count over the runs. A rate
//! threshold (`completed_at_least`, `false_successes_at_most`) bounds a share:
//! a trial compares the plain rate over its runs, and a sweep compares the
//! confidence bound of the rate.

use serde::{Deserialize, Deserializer, Serialize};
use std::fmt;

/// A share, from 0 to 1.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize)]
#[serde(transparent)]
pub struct Rate(f64);

impl Rate {
    pub fn new(share: f64) -> Option<Rate> {
        (0.0..=1.0).contains(&share).then_some(Rate(share))
    }

    pub fn get(self) -> f64 {
        self.0
    }
}

impl fmt::Display for Rate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl<'de> Deserialize<'de> for Rate {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Rate, D::Error> {
        let share = f64::deserialize(deserializer)?;
        Rate::new(share).ok_or_else(|| {
            serde::de::Error::custom(format!(
                "the rate {share} is not a share; write a rate from 0 to 1"
            ))
        })
    }
}

/// The thresholds of sign-off.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(
    deny_unknown_fields,
    expecting = "thresholds, { completed_at_least, unsafe_events_at_most, false_successes_at_most, false_assumption_checks_at_most }"
)]
pub struct Thresholds {
    /// The least share of the runs that count that complete.
    pub completed_at_least: Rate,
    /// The most unsafe events over the runs.
    pub unsafe_events_at_most: u32,
    /// The most share of the completed runs whose task is not done.
    pub false_successes_at_most: Rate,
    /// The most checks of assumptions that differ from the work cell.
    pub false_assumption_checks_at_most: u32,
}

/// The four measures over the runs of a sweep or a trial.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Measures {
    /// The runs that count for `completed_at_least`.
    pub runs: u32,
    /// Of these runs, those that completed.
    pub completed: u32,
    /// Of the completed runs, those whose task is not done.
    pub false_successes: u32,
    pub unsafe_events: u32,
    pub false_assumption_checks: u32,
}

impl Measures {
    /// The share of the runs that completed; 0 with no run.
    pub fn completed_rate(&self) -> f64 {
        share(self.completed, self.runs)
    }

    /// The share of the completed runs whose task is not done; 0 with no
    /// completed run.
    pub fn false_success_rate(&self) -> f64 {
        share(self.false_successes, self.completed)
    }
}

fn share(part: u32, whole: u32) -> f64 {
    match whole {
        0 => 0.0,
        whole => f64::from(part) / f64::from(whole),
    }
}

/// A threshold that the measures do not meet.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Unmet {
    CompletedAtLeast,
    UnsafeEventsAtMost,
    FalseSuccessesAtMost,
    FalseAssumptionChecksAtMost,
}

impl Thresholds {
    /// Each threshold that `measures` does not meet by its plain rates and
    /// counts, as a trial compares them. A trial with no run that counts
    /// meets no `completed_at_least` above 0.
    pub fn unmet_by(&self, measures: &Measures) -> Vec<Unmet> {
        let checks = [
            (
                Unmet::CompletedAtLeast,
                measures.completed_rate() >= self.completed_at_least.get(),
            ),
            (
                Unmet::UnsafeEventsAtMost,
                measures.unsafe_events <= self.unsafe_events_at_most,
            ),
            (
                Unmet::FalseSuccessesAtMost,
                measures.false_success_rate() <= self.false_successes_at_most.get(),
            ),
            (
                Unmet::FalseAssumptionChecksAtMost,
                measures.false_assumption_checks <= self.false_assumption_checks_at_most,
            ),
        ];
        checks
            .into_iter()
            .filter(|(_, met)| !met)
            .map(|(unmet, _)| unmet)
            .collect()
    }

    /// Whether `remaining` more runs that count can still bring `measures`
    /// to meet every threshold: when each of them completes with its task
    /// done, no unsafe event and no false check. A trial ends once this is
    /// false.
    pub fn can_still_be_met(&self, measures: &Measures, remaining: u32) -> bool {
        let best = Measures {
            runs: measures.runs.saturating_add(remaining),
            completed: measures.completed.saturating_add(remaining),
            ..*measures
        };
        self.unmet_by(&best).is_empty()
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// The thresholds of the tape line of the plan.
    pub(crate) fn thresholds() -> Thresholds {
        Thresholds {
            completed_at_least: Rate::new(0.95).unwrap(),
            unsafe_events_at_most: 0,
            false_successes_at_most: Rate::new(0.01).unwrap(),
            false_assumption_checks_at_most: 0,
        }
    }

    fn measures(runs: u32, completed: u32, false_successes: u32) -> Measures {
        Measures {
            runs,
            completed,
            false_successes,
            ..Measures::default()
        }
    }

    #[test]
    fn thresholds_read_their_four_fields_and_refuse_a_rate_outside_0_to_1() {
        let text = "{ completed_at_least: 0.95, unsafe_events_at_most: 0, false_successes_at_most: 0.01, false_assumption_checks_at_most: 0 }";
        let tree = documents_core::parse(text).unwrap();
        assert_eq!(
            serde_json::from_value::<Thresholds>(tree).unwrap(),
            thresholds()
        );
        let over = documents_core::parse(&text.replace("0.95", "1.5")).unwrap();
        assert!(
            serde_json::from_value::<Thresholds>(over)
                .unwrap_err()
                .to_string()
                .starts_with("the rate 1.5 is not a share; write a rate from 0 to 1")
        );
        let missing = documents_core::parse("{ completed_at_least: 0.95 }").unwrap();
        assert!(
            serde_json::from_value::<Thresholds>(missing)
                .unwrap_err()
                .to_string()
                .contains("missing field `unsafe_events_at_most`")
        );
    }

    #[test]
    fn plain_rates_and_counts_meet_or_miss_each_threshold() {
        let thresholds = thresholds();
        assert_eq!(thresholds.unmet_by(&measures(20, 19, 0)), []);
        assert_eq!(
            thresholds.unmet_by(&measures(20, 18, 0)),
            [Unmet::CompletedAtLeast]
        );
        assert_eq!(
            thresholds.unmet_by(&measures(20, 20, 1)),
            [Unmet::FalseSuccessesAtMost]
        );
        let unsafe_and_wrong = Measures {
            unsafe_events: 1,
            false_assumption_checks: 1,
            ..measures(20, 20, 0)
        };
        assert_eq!(
            thresholds.unmet_by(&unsafe_and_wrong),
            [
                Unmet::UnsafeEventsAtMost,
                Unmet::FalseAssumptionChecksAtMost
            ]
        );
    }

    #[test]
    fn no_run_meets_a_completed_rate_above_0() {
        assert_eq!(
            thresholds().unmet_by(&Measures::default()),
            [Unmet::CompletedAtLeast]
        );
        let any = Thresholds {
            completed_at_least: Rate::new(0.0).unwrap(),
            ..thresholds()
        };
        assert_eq!(any.unmet_by(&Measures::default()), []);
    }

    #[test]
    fn a_trial_can_still_meet_the_thresholds_while_its_best_end_meets_them() {
        let thresholds = thresholds();
        assert!(thresholds.can_still_be_met(&Measures::default(), 20));
        assert!(
            !thresholds.can_still_be_met(&measures(2, 0, 0), 18),
            "18 of 20 is below 0.95"
        );
        assert!(thresholds.can_still_be_met(&measures(1, 0, 0), 19));
        let unsafe_event = Measures {
            unsafe_events: 1,
            ..measures(1, 1, 0)
        };
        assert!(!thresholds.can_still_be_met(&unsafe_event, 19));
        assert!(
            !thresholds.can_still_be_met(&measures(5, 5, 1), 15),
            "1 false success in 20 completed runs is above 0.01"
        );
        assert!(
            thresholds.can_still_be_met(&measures(5, 5, 1), 95),
            "1 false success in 100 completed runs is 0.01"
        );
    }
}

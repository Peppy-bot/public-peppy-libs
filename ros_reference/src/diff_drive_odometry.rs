//! Diff drive odometry. The reference is the odometry of ros2_controllers 6.9.0's diff
//! drive controller (`diff_drive_controller::Odometry`): with each scenario's wheel
//! geometry and rolling window, it integrates the scenario's wheel positions one by one by
//! `update_from_pos` at the scenario's period, and gives the pose and the velocity after
//! each update.

use crate::{Kind, read_reference, read_scenario, scenario_names};
use serde::Deserialize;

/// The `kind` of a diff drive odometry scenario and of its reference output, which
/// refuses every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "diff_drive_odometry")]
    DiffDriveOdometry,
}

/// The wheels of a differential base, by the diff drive controller's names.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Wheels {
    pub wheel_separation: f64,
    pub left_wheel_radius: f64,
    pub right_wheel_radius: f64,
    pub velocity_rolling_window_size: usize,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    wheels: Wheels,
    /// The time between two updates, in seconds.
    period: f64,
    /// The left and the right wheel's angle, in radians, at each update.
    positions: Vec<[f64; 2]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    wheels: Wheels,
    period: f64,
    updates: Vec<UpdateRead>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct UpdateRead {
    positions: [f64; 2],
    integrated: bool,
    pose: [f64; 3],
    velocity: [f64; 2],
}

/// One update: the wheel angles it took, whether it integrated them, and the odometry
/// after it, x, y and the heading, and the rolling means of the forward speed and the
/// turn rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Update {
    pub positions: [f64; 2],
    pub integrated: bool,
    pub pose: [f64; 3],
    pub velocity: [f64; 2],
}

/// A diff drive odometry scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    pub wheels: Wheels,
    pub period: f64,
    pub updates: Vec<Update>,
}

impl Cases {
    /// Every diff drive odometry scenario under `scenarios/`, in the order of their names,
    /// each paired with its reference output: the reference read the wheels, the period
    /// and every wheel position.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::DiffDriveOdometry)
            .into_iter()
            .map(|name| {
                let scenario = read_scenario(&name);
                let reference = read_reference(&name);
                pair(name, scenario, reference)
            })
            .collect()
    }
}

fn pair(name: String, scenario: Scenario, reference: Reference) -> Cases {
    let Scenario {
        kind: ThisKind::DiffDriveOdometry,
        wheels,
        period,
        positions,
    } = scenario;
    let Reference {
        kind: ThisKind::DiffDriveOdometry,
        wheels: wheels_read,
        period: period_read,
        updates: updates_read,
    } = reference;
    assert_eq!(wheels_read, wheels, "{name}: the reference read the wheels");
    assert_eq!(period_read, period, "{name}: the reference read the period");
    assert_eq!(
        positions.len(),
        updates_read.len(),
        "{name}: the reference integrated every wheel position"
    );
    let updates = positions
        .iter()
        .zip(updates_read)
        .enumerate()
        .map(|(index, (&given, read))| {
            assert_eq!(
                read.positions, given,
                "{name} update {index}: the reference read the wheel positions"
            );
            Update {
                positions: given,
                integrated: read.integrated,
                pose: read.pose,
                velocity: read.velocity,
            }
        })
        .collect();
    Cases {
        name,
        wheels,
        period,
        updates,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_scenario_of_another_kind_is_refused() {
        let text = r#"{"kind": "inflation", "wheels": {"wheel_separation": 0.4,
            "left_wheel_radius": 0.05, "right_wheel_radius": 0.05,
            "velocity_rolling_window_size": 4}, "period": 0.02, "positions": []}"#;
        assert!(serde_json::from_str::<Scenario>(text).is_err());
    }

    #[test]
    fn every_diff_drive_odometry_scenario_pairs_with_its_reference() {
        let scenarios = Cases::load_all();
        assert!(
            !scenarios.is_empty(),
            "scenarios/ holds a diff drive odometry scenario"
        );
        for cases in scenarios {
            assert!(
                cases.updates.iter().all(|update| update.integrated),
                "{} integrates every update at its period",
                cases.name
            );
        }
    }
}

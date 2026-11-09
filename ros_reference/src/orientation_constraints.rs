//! Orientation constraints. The reference is MoveIt's kinematic constraint under
//! rotation-vector tolerances about the axes of the target: whether a link at each
//! configuration keeps a target within a tolerance small enough to lie inside a contract's
//! `tolerance_rad`, and within one large enough to hold all of it.

use crate::{Kind, read_reference, read_scenario, read_urdf, scenario_names};
use serde::Deserialize;

/// The `kind` of an orientation constraints scenario and of its reference output, which
/// refuses every other kind.
#[derive(Deserialize)]
enum ThisKind {
    #[serde(rename = "orientation_constraints")]
    OrientationConstraints,
}

/// A robot, the joints that pose a link of it, and the cases to decide.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Scenario {
    kind: ThisKind,
    /// The path of the robot's URDF, relative to the repository root.
    urdf: String,
    joints: Vec<String>,
    link_name: String,
    cases: Vec<DrawnCase>,
}

/// A configuration of the joints and a target of the link, as a contract states it.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DrawnCase {
    positions: Vec<f64>,
    /// The quaternion `[x, y, z, w]` of the target, in the robot's root frame.
    orientation: [f64; 4],
    tolerance_rad: f64,
    /// "none", "x", "y" or "z".
    free_axis: String,
}

/// For each case of the scenario, in its order, MoveIt's two verdicts.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    kind: ThisKind,
    cases: Vec<Verdicts>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Verdicts {
    accepted_within_inner: bool,
    accepted_within_outer: bool,
}

/// An orientation constraints scenario paired with its reference output.
pub struct Cases {
    /// The scenario's file name under `scenarios/`, without `.json`.
    pub name: String,
    /// The text of the scenario's URDF.
    pub urdf: String,
    pub joints: Vec<String>,
    pub link_name: String,
    pub cases: Vec<Case>,
}

/// A case and MoveIt's verdicts on it.
pub struct Case {
    pub positions: Vec<f64>,
    pub orientation: [f64; 4],
    pub tolerance_rad: f64,
    pub free_axis: String,
    /// MoveIt's verdict at `tolerance_rad` over the square root of 3 about each axis, and pi
    /// about a free axis.
    pub accepted_within_inner: bool,
    /// MoveIt's verdict at `tolerance_rad` about each axis, or with a free axis at pi about
    /// it and at pi times the sine of half `tolerance_rad` about each other.
    pub accepted_within_outer: bool,
}

impl Cases {
    /// Every orientation constraints scenario under `scenarios/`, in the order of their
    /// names, each paired with its reference output: one pair of verdicts per case.
    pub fn load_all() -> Vec<Self> {
        scenario_names(Kind::OrientationConstraints)
            .into_iter()
            .map(Self::load)
            .collect()
    }

    fn load(name: String) -> Self {
        let Scenario {
            kind: ThisKind::OrientationConstraints,
            urdf,
            joints,
            link_name,
            cases,
        } = read_scenario(&name);
        let Reference {
            kind: ThisKind::OrientationConstraints,
            cases: verdicts,
        } = read_reference(&name);
        assert_eq!(
            cases.len(),
            verdicts.len(),
            "{name}: one pair of verdicts per case"
        );
        let urdf = read_urdf(&name, &urdf);
        let cases = cases
            .into_iter()
            .zip(verdicts)
            .map(|(case, verdicts)| Case {
                positions: case.positions,
                orientation: case.orientation,
                tolerance_rad: case.tolerance_rad,
                free_axis: case.free_axis,
                accepted_within_inner: verdicts.accepted_within_inner,
                accepted_within_outer: verdicts.accepted_within_outer,
            })
            .collect();
        Cases {
            name,
            urdf,
            joints,
            link_name,
            cases,
        }
    }
}

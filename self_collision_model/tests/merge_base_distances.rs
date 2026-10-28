//! The model holds to the self-collision model of this repository at its merge base,
//! a817e346, as `fixtures/merge_base/capture.sh` recorded it in
//! `fixtures/merge_base/distances.json`. Built as the backbone builds it, for each
//! hardware generation, the model checks the same pairs, and on each recorded
//! configuration the distance of every checked pair and the nearest distance lie
//! within [`TOLERANCE`] of the recorded ones. Each verdict, a distance at or under a
//! threshold of the backbone, is the recorded one wherever the recorded distance lies
//! farther than [`TOLERANCE`] from the threshold.

use std::collections::BTreeSet;
use std::str::FromStr;

use collision_geometry::gjk;
use openarm_description::{HardwareVersion, Side};
use self_collision_model::SelfCollisionModel;
use serde::Deserialize;

/// How far a distance may lie from the recorded one, meters: each fitted piece lies
/// within 1 mm of the convex hull of its surface, as the recorded model's did.
const TOLERANCE: f64 = 0.001;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Fixture {
    /// The commits the recorded models were built at.
    source: String,
    /// The stop and the full-speed distances of the backbone, meters.
    thresholds: Vec<f64>,
    models: Vec<Recorded>,
}

/// The recorded model of one hardware generation.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Recorded {
    version: String,
    pairs: Vec<[String; 2]>,
    configurations: Vec<Configuration>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Configuration {
    /// The joints of the left arm, then of the right arm.
    arms: [[f64; 7]; 2],
    /// The gripper openings, left then right.
    openings: [f64; 2],
    /// The distance of each checked pair, in the order of the pairs.
    distances: Vec<f64>,
    nearest: Nearest,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Nearest {
    distance: f64,
    pair: [String; 2],
}

fn fixture() -> Fixture {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/merge_base/distances.json"
    );
    let text = std::fs::read_to_string(path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    serde_json::from_str(&text).unwrap_or_else(|e| panic!("{path} does not parse: {e}"))
}

/// The model of `version` as the backbone builds it: the bundled URDF and meshes, the
/// base link of each arm, and the torso regions.
fn model(version: HardwareVersion) -> SelfCollisionModel {
    let meshes = match version {
        HardwareVersion::V1 => "meshes",
        HardwareVersion::V2 => "meshes_v20",
    };
    let meshes = format!(
        "{}/../openarm_description/assets/{meshes}",
        env!("CARGO_MANIFEST_DIR")
    );
    SelfCollisionModel::builder(
        version.urdf(),
        &meshes,
        version.base_link(Side::Left),
        version.base_link(Side::Right),
    )
    .regions(version.torso_body(), version.torso_regions())
    .build()
    .expect("the model builds")
}

/// A pair of bodies, in either order.
fn unordered([a, b]: [&str; 2]) -> [String; 2] {
    let mut pair = [a.to_owned(), b.to_owned()];
    pair.sort();
    pair
}

/// The distance of each pair of `pairs` at `configuration`: the least distance between
/// a piece of one body and a piece of the other.
fn pair_distances(
    model: &mut SelfCollisionModel,
    configuration: &Configuration,
    pairs: &[[String; 2]],
) -> Vec<f64> {
    let [q_left, q_right] = &configuration.arms;
    let bodies = model
        .world_pieces(q_left, q_right)
        .expect("a finite configuration");
    let pieces_of = |name: &str| {
        &bodies
            .iter()
            .find(|(body, _)| *body == name)
            .unwrap_or_else(|| panic!("the model holds {name}"))
            .1
    };
    pairs
        .iter()
        .map(|[a, b]| {
            pieces_of(a)
                .iter()
                .flat_map(|piece_a| {
                    pieces_of(b)
                        .iter()
                        .map(move |piece_b| gjk::distance(piece_a, piece_b).distance)
                })
                .fold(f64::INFINITY, f64::min)
        })
        .collect()
}

/// Whether `ours` gives the verdict of `recorded` at each threshold whose distance
/// from `recorded` exceeds [`TOLERANCE`].
fn same_verdicts(thresholds: &[f64], recorded: f64, ours: f64) -> bool {
    thresholds
        .iter()
        .filter(|&&threshold| (recorded - threshold).abs() > TOLERANCE)
        .all(|&threshold| (ours <= threshold) == (recorded <= threshold))
}

#[test]
fn distances_and_verdicts_hold_to_the_merge_base() {
    let Fixture {
        source,
        thresholds,
        models,
    } = fixture();
    assert_eq!(thresholds, [0.005, 0.02], "the thresholds of the backbone");
    for recorded in &models {
        let version = HardwareVersion::from_str(&recorded.version).expect("a generation");
        let mut model = model(version);
        let checked: BTreeSet<[String; 2]> = model
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| unordered([a, b]))
            .collect();
        let expected: BTreeSet<[String; 2]> = recorded
            .pairs
            .iter()
            .map(|[a, b]| unordered([a, b]))
            .collect();
        assert_eq!(
            checked, expected,
            "{version}: the checked pairs of {source}"
        );
        for (index, configuration) in recorded.configurations.iter().enumerate() {
            let [left, right] = configuration.openings;
            model.set_gripper_openings(left, right);
            let distances = pair_distances(&mut model, configuration, &recorded.pairs);
            for ((pair, &before), &now) in recorded
                .pairs
                .iter()
                .zip(&configuration.distances)
                .zip(&distances)
            {
                assert!(
                    (now - before).abs() <= TOLERANCE && same_verdicts(&thresholds, before, now),
                    "{version} configuration {index}, {} and {}: {now} m, {before} m at {source}",
                    pair[0],
                    pair[1]
                );
            }
            let [q_left, q_right] = &configuration.arms;
            let now = model
                .min_distance(q_left, q_right)
                .expect("a finite configuration")
                .distance;
            let before = &configuration.nearest;
            assert!(
                (now - before.distance).abs() <= TOLERANCE
                    && same_verdicts(&thresholds, before.distance, now),
                "{version} configuration {index}: nearest {now} m, {} m between {} and {} at {source}",
                before.distance,
                before.pair[0],
                before.pair[1]
            );
        }
    }
}

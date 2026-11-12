//! The checks every robot model robot_description bundles passes, each at the shape of
//! its self model: it builds from its description and stands clear at home, it checks
//! every pair of its bodies but the pairs its SRDF disables, and the pairs its SRDF
//! disables as never touching stay apart within its joint limits.

use std::collections::BTreeSet;

use robot_description::{DisabledReason, OnShape, states};

#[path = "fixtures/models.rs"]
mod fixture;

use fixture::{
    check_every_model, description, least_distance_within_the_limits, model, posture, unordered,
};

/// The least distance every pair a bundled SRDF disables as never touching keeps over the
/// grid of [`fixture::least_distance_within_the_limits`]: the millimeter below the least
/// such distance of every bundled model, 1.4 mm, between OpenArm v1's link3 and link5.
const NEVER_TOUCHING_LEAST_M: f64 = 0.001;

struct BuildsAndStandsClearAtHome(&'static str);

impl OnShape for BuildsAndStandsClearAtHome {
    type Output = ();

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) {
        let id = self.0;
        let mut model = model::<N, K>(id);
        assert!(!model.checked_pairs().is_empty(), "{id} checks no pair");
        let home = posture::<N, K>(&description(id), states::HOME);
        let nearest = model.min_distance(&home).expect("a distance");
        assert!(
            nearest.distance > 0.0,
            "{id} collides at home: {} - {} at {}",
            nearest.link_a,
            nearest.link_b,
            nearest.distance
        );
    }
}

// Each model builds from its description, checks pairs, and stands with every checked
// pair apart at home, its grippers at the model's default opening.
#[test]
fn each_model_builds_from_its_description_and_stands_clear_at_home() {
    check_every_model(BuildsAndStandsClearAtHome);
}

struct ChecksEveryPairButTheDisabledOnes(&'static str);

impl OnShape for ChecksEveryPairButTheDisabledOnes {
    type Output = ();

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) {
        let id = self.0;
        let description = description(id);
        let mut model = model::<N, K>(id);
        let checked: BTreeSet<(String, String)> = model
            .checked_pairs()
            .into_iter()
            .map(|(a, b)| unordered(a, b))
            .collect();
        assert_eq!(
            checked.len(),
            model.checked_pairs().len(),
            "{id} checks each pair once"
        );
        let bodies: Vec<String> = model
            .world_pieces(&posture(&description, states::HOME))
            .expect("a finite configuration")
            .into_iter()
            .map(|(body, _)| body.to_string())
            .collect();
        let every: BTreeSet<(String, String)> = bodies
            .iter()
            .enumerate()
            .flat_map(|(index, a)| bodies[index + 1..].iter().map(|b| unordered(a, b)))
            .collect();
        let disabled: BTreeSet<(String, String)> = description
            .disabled_pairs()
            .iter()
            .map(|pair| unordered(&pair.link1, &pair.link2))
            .collect();
        assert!(
            disabled.is_subset(&every),
            "{id} disables pairs of no two bodies: {:?}",
            &disabled - &every
        );
        assert_eq!(checked, &every - &disabled, "{id}");
    }
}

// Each model checks every pair of its bodies but the pairs its SRDF disables, each once,
// and each pair its SRDF disables is a pair of its bodies.
#[test]
fn each_model_checks_every_pair_of_its_bodies_but_the_disabled_ones() {
    check_every_model(ChecksEveryPairButTheDisabledOnes);
}

struct NeverTouchingPairsStayApart(&'static str);

impl OnShape for NeverTouchingPairsStayApart {
    type Output = ();

    fn on_shape<const N: usize, const D: usize, const K: usize>(self) {
        let id = self.0;
        let description = description(id);
        let mut model = model::<N, K>(id);
        let nearer: Vec<String> = description
            .disabled_pairs()
            .iter()
            .filter(|pair| pair.reason == DisabledReason::Never)
            .filter_map(|pair| {
                let (a, b) = (pair.link1.as_str(), pair.link2.as_str());
                let least = least_distance_within_the_limits(&mut model, &description, a, b);
                println!("{id} {a} - {b}: least {least:.4} m");
                (least < NEVER_TOUCHING_LEAST_M).then(|| format!("{a} - {b} at {least:.4} m"))
            })
            .collect();
        assert!(
            nearer.is_empty(),
            "{id}: pairs nearer than {NEVER_TOUCHING_LEAST_M} m: {nearer:#?}"
        );
    }
}

// The pairs each model's SRDF disables as never touching stay NEVER_TOUCHING_LEAST_M or
// farther apart over the grid of the joints between each pair's two bodies within their
// limits.
#[test]
fn the_pairs_disabled_as_never_touching_stay_apart_within_the_joint_limits() {
    check_every_model(NeverTouchingPairsStayApart);
}

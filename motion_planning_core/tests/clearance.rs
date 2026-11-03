//! The clearance check against the map: its sphere screen never changes the hulls'
//! verdict, and a part the arm carries keeps its clearance as the arm's links do.

mod common;

use hegel::TestCase;
use hegel::generators as gs;
use kinematics_properties::{configs, unit_vectors};
use motion_planning_core::{Arm, Contact, Surroundings};
use nalgebra::Point3;
use occupancy_map::{DistanceField, VoxelMap};

use common::{JOINTS, LEFT_ARM, left_fingers, map, occupied_at, voxel_boxes, voxel_in_a_finger};

const VOXEL_M: f64 = 0.05;

/// Whether every body of the arm at `q` lies at least `clearance` from the box of every
/// occupied voxel of `map`, hull against box, with no screen.
fn hulls_alone(arm: &Arm<JOINTS>, map: &VoxelMap, q: &[f64; JOINTS], clearance: f64) -> bool {
    let boxes = voxel_boxes(map);
    arm.bodies(q).iter().all(|(_, body)| {
        boxes
            .iter()
            .all(|b| body.distance_to_convex(b, b.bounding_sphere()) >= clearance)
    })
}

// On drawn configurations of the arm, its fingers riding its hand, among drawn voxels
// just outside the bounding spheres of its pieces, the check with spheres then hulls
// gives the verdict of hulls alone.
#[hegel::test]
fn the_sphere_screen_gives_the_hulls_verdict(tc: TestCase) {
    let riders = left_fingers();
    let arm = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the fingers ride the hand");
    let q = tc.draw(configs(LEFT_ARM.chain().limits()));
    let spheres: Vec<(Point3<f64>, f64)> = arm
        .bodies(&q)
        .iter()
        .map(|(_, body)| body.bounding_sphere())
        .collect();
    let clearance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.15));
    // Each point lies outside a piece's bounding sphere, up to twice the clearance and
    // a few voxels beyond it, where the screen and the hulls may disagree.
    let count = tc.draw(gs::integers::<usize>().min_value(1).max_value(4));
    let points: Vec<[f64; 3]> = (0..count)
        .map(|_| {
            let (center, radius) = spheres[tc.draw(
                gs::integers::<usize>()
                    .min_value(0)
                    .max_value(spheres.len() - 1),
            )];
            let direction = tc.draw(unit_vectors());
            let beyond: f64 = tc.draw(
                gs::floats()
                    .min_value(0.0)
                    .max_value(2.0 * clearance + 4.0 * VOXEL_M),
            );
            let p = center + direction * (radius + beyond);
            [p.x, p.y, p.z]
        })
        .collect();
    let mut map = map(VOXEL_M, 1.5);
    occupied_at(&mut map, [0.0, 0.0, 2.0], &points);
    let field = DistanceField::of(&map);
    let surroundings = Surroundings::new(&map, &field, [clearance, 0.0, 0.0]);
    assert_eq!(
        surroundings.clear(&arm, &q),
        hulls_alone(&arm, &map, &q, clearance),
        "the screened verdict"
    );
}

// A voxel inside a finger of the hand, which the bare arm keeps its clearance of, leaves
// the arm carrying its fingers short of its clearance, touching the map with the finger.
#[test]
fn a_finger_keeps_its_clearance_as_the_links_do() {
    const CLEARANCE_M: f64 = 0.02;
    let q = [0.0; JOINTS];
    let mut map = map(0.02, 1.0);
    let (voxel, away) = voxel_in_a_finger(&map, &q);
    assert!(
        away > CLEARANCE_M,
        "the bare arm lies {away} m from the voxel"
    );
    occupied_at(&mut map, [0.0, 0.0, 2.0], &[voxel]);
    let field = DistanceField::of(&map);
    let surroundings = Surroundings::new(&map, &field, [CLEARANCE_M, 0.0, 0.0]);
    let riders = left_fingers();
    let riding = Arm::new(&LEFT_ARM, None)
        .riding(&riders)
        .expect("the fingers ride the hand");
    assert!(surroundings.clear(&Arm::new(&LEFT_ARM, None), &q));
    assert!(!surroundings.clear(&riding, &q));
    let contact = surroundings.contact(&riding, &q);
    assert!(
        matches!(&contact, Some(Contact::Map { body }) if riders.iter().any(|r| r.name == *body)),
        "{contact:?}"
    );
}

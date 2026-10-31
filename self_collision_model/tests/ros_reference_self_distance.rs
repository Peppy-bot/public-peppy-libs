//! Self distance against MoveIt's FCL: the pieces of each body of `SelfCollisionModel`
//! hold the collision meshes of its link, so the signed distance between the pieces of
//! two bodies is no larger than the distance between their meshes. The evidence: at every
//! configuration of every self distance scenario of the ROS reference harness, for each
//! pair that the model checks, the fingers' pairs included, the distance between the
//! pieces is no larger than FCL's distance between the meshes, where FCL finds them
//! apart. Where FCL finds the meshes in contact, its distance is the depth of the deepest
//! touching pair of triangles, which bounds neither solid's depth; there the meshes
//! overlap, so the pieces that hold them overlap or touch, and the distance between them
//! is no larger than zero. The model places a gripper's fingers at an opening fraction,
//! which the scenario's finger joint gives: the fingers of OpenArm v2 close at 0, and
//! open at the other limit of their travel.

use collision_geometry::gjk;
use collision_geometry::urdf_collision::UrdfCollisions;
use openarm_description::{HardwareVersion, Side};
use ros_reference::self_distance::{Cases, Configuration};
use self_collision_model::SelfCollisionModel;
use srs_model::JointVec;

/// The rounding of two distance computations, in meters.
const TOLERANCE: f64 = 1e-9;

const VERSION: HardwareVersion = HardwareVersion::V2;

fn arm_joint(side: Side, joint: usize) -> String {
    format!("openarm_{side}_joint{joint}")
}

fn arm(configuration: &Configuration, side: Side) -> JointVec {
    std::array::from_fn(|i| configuration.positions[&arm_joint(side, i + 1)])
}

/// The opening fraction of the gripper of `side`: the position of its first finger
/// joint, whose child is the link `ee_link1`, over the limit it opens to. The fingers
/// close at 0, the one limit both finger joints share.
fn opening(collisions: &UrdfCollisions, configuration: &Configuration, side: Side) -> f64 {
    let joint = collisions
        .parent_joint(&format!("openarm_{side}_ee_link1"))
        .expect("the first finger joint");
    let open = match (joint.lower_limit, joint.upper_limit) {
        (0.0, upper) => upper,
        (lower, 0.0) => lower,
        (lower, upper) => panic!("the {side} fingers travel from {lower} to {upper}, not from 0"),
    };
    configuration.positions[&format!("openarm_{side}_finger_joint1")] / open
}

#[test]
fn each_checked_distance_is_no_larger_than_fcls() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a self distance scenario"
    );
    for cases in &scenarios {
        assert_each_checked_distance_is_no_larger_than_fcls(cases);
    }
}

fn assert_each_checked_distance_is_no_larger_than_fcls(cases: &Cases) {
    let name = &cases.name;
    assert!(
        cases.urdf == VERSION.urdf(),
        "{name}: the parity test builds the self model of OpenArm {VERSION}, whose URDF the \
         scenario names"
    );
    let collisions = UrdfCollisions::from_urdf(&cases.urdf).expect("the URDF reads");
    let mut model = SelfCollisionModel::builder(
        &cases.urdf,
        cases.meshes.to_str().expect("the mesh directory is UTF-8"),
        VERSION.base_link(Side::Left),
        VERSION.base_link(Side::Right),
    )
    .regions(VERSION.torso_body(), VERSION.torso_regions())
    .build()
    .unwrap_or_else(|e| panic!("{name}: {e}"));
    let checked: Vec<(String, String)> = model
        .checked_pairs()
        .into_iter()
        .map(|(a, b)| (a.to_owned(), b.to_owned()))
        .collect();
    let (mut apart, mut in_contact) = (0, 0);
    for (index, configuration) in cases.configurations.iter().enumerate() {
        model.set_gripper_openings(
            opening(&collisions, configuration, Side::Left),
            opening(&collisions, configuration, Side::Right),
        );
        let bodies = model
            .world_pieces(
                &arm(configuration, Side::Left),
                &arm(configuration, Side::Right),
            )
            .expect("a finite configuration");
        let pieces_of = |body_name: &str| {
            &bodies
                .iter()
                .find(|(body, _)| *body == body_name)
                .unwrap_or_else(|| panic!("the model holds {body_name}"))
                .1
        };
        for (a, b) in &checked {
            let ours = pieces_of(a)
                .iter()
                .flat_map(|piece_a| {
                    pieces_of(b)
                        .iter()
                        .map(move |piece_b| gjk::distance(piece_a, piece_b).distance)
                })
                .fold(f64::INFINITY, f64::min);
            let fcl = configuration
                .distance(a, b)
                .unwrap_or_else(|| panic!("{name}: FCL measures {a} and {b}"));
            if fcl > 0.0 {
                apart += 1;
            } else {
                in_contact += 1;
            }
            assert!(
                ours <= fcl.max(0.0) + TOLERANCE,
                "{name} configuration {index}, {a} and {b}: the model reads {ours} m, FCL \
                 {fcl} m"
            );
        }
    }
    assert!(
        apart > 0 && in_contact > 0,
        "{name} holds checked pairs apart and in contact: {apart} apart, {in_contact} in \
         contact"
    );
}

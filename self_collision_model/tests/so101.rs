//! SO-101's self-collision model, built from the description robot_description bundles:
//! the folded and the ready arm, and the jaw where its joint places it.

use collision_geometry::urdf_collision::UrdfCollisions;
use robot_description::states;
use self_collision_model::SelfCollisionModel;

#[path = "fixtures/models.rs"]
mod fixture;

use fixture::{arm_geometry, description, meshes, model, posture};

/// SO-101's model id.
const MODEL: &str = "so101";

/// The joints of SO-101's arm.
const ARM_DOF: usize = 5;

/// SO-101's self model: its one arm.
type So101 = SelfCollisionModel<ARM_DOF, 1>;

/// The arm folded onto itself, as it rests.
const FOLDED: [f64; ARM_DOF] = [-0.05, -1.72, 1.68, 1.24, 0.32];

/// The clearance a plan of the SO-101 launchers keeps between the arm's own bodies: the
/// launchers' `min_clearance_m` and 1 mm of spare for each of a pair's two moving links.
const PLANNER_SELF_CLEARANCE_M: f64 = 0.002;

// Folded as it rests, the arm's own bodies touch; at ready, they stand farther apart than
// the planner's self clearance, the jaw shut or open.
#[test]
fn folded_the_arm_touches_itself_and_ready_it_stands_clear() {
    let ready = posture(&description(MODEL), states::READY);
    let mut model: So101 = model(MODEL);
    for opening in [0.0, 1.0] {
        model.set_gripper_openings(&[opening]);
        let folded = model.min_distance(&[FOLDED]).expect("a distance");
        assert!(
            folded.distance <= 0.0,
            "folded at opening {opening}: {} - {} at {}",
            folded.link_a,
            folded.link_b,
            folded.distance
        );
        let clear = model.min_distance(&ready).expect("a distance");
        assert!(
            clear.distance > PLANNER_SELF_CLEARANCE_M,
            "ready at opening {opening}: {} - {} at {}",
            clear.link_a,
            clear.link_b,
            clear.distance
        );
    }
}

// The model hangs the jaw where its joint places it at the arm's opening: the offset the
// model gives the jaw the arm's chain carries is the jaw joint's offset at the position
// the opening reaches from where the gripper's closed state puts the jaw toward where its
// open state does.
#[test]
fn the_jaw_hangs_where_its_joint_places_it() {
    let description = description(MODEL);
    let [arm] = description.arms() else {
        panic!("SO-101 has one arm");
    };
    let [jaw] = arm.gripper.fingers.as_slice() else {
        panic!("the gripper has one finger");
    };
    let mut model: So101 = model(MODEL);
    let chain = arm_geometry::<ARM_DOF>(&description, &meshes(MODEL), arm);
    let joint = UrdfCollisions::from_urdf(description.urdf())
        .expect("the URDF reads")
        .parent_joint(&jaw.link)
        .expect("the jaw rides a joint")
        .clone();
    for opening in [0.0, 0.3, 1.0] {
        model.set_gripper_openings(&[opening]);
        let fingers = model
            .fingers(0, chain.carried())
            .expect("the arm's chain carries the jaw");
        let [hung] = fingers.as_slice() else {
            panic!("the arm carries one finger");
        };
        let placed = joint
            .offset(jaw.closed + opening * (jaw.open - jaw.closed))
            .expect("the jaw's joint places it");
        assert!(
            (hung.offset.translation.vector - placed.translation.vector).norm() < 1e-12
                && hung.offset.rotation.angle_to(&placed.rotation) < 1e-12,
            "at opening {opening}: the model hangs the jaw at {}, its joint places it at {placed}",
            hung.offset
        );
    }
}

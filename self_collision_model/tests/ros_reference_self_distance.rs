//! Self distance against MoveIt's FCL: the pieces of each body of `SelfCollisionModel`
//! hold the collision meshes of its link, so the signed distance between the pieces of
//! two bodies is no larger than the distance between their meshes. The evidence: at every
//! configuration of every self distance scenario of the ROS reference harness, for each
//! pair that the robot's model checks or leaves out, the fingers' pairs included, the
//! distance between the pieces is no larger than FCL's distance between the meshes, where
//! FCL finds them apart. Where FCL finds the meshes in contact, its distance is the depth
//! of the deepest touching pair of triangles, which bounds neither solid's depth; there the
//! meshes overlap, so the pieces that hold them overlap or touch, and the distance between
//! them is no larger than zero. The model places a chain's fingers at an opening fraction,
//! which the scenario's finger joint gives: from the position at which the robot's
//! description shuts the finger toward the joint's other limit.

use collision_geometry::gjk;
use openarm_description::{ARM_DOF as OPENARM_DOF, HardwareVersion, Side};
use ros_reference::self_distance::{Cases, Configuration};
use self_collision_model::{Joints, SelfCollisionModel, openarm, so101};
use so101_description::{ARM_DOF as SO101_DOF, JAW_LINK, jaw_shut_rad};

/// The rounding of two distance computations, in meters.
const TOLERANCE: f64 = 1e-9;

const OPENARM: HardwareVersion = HardwareVersion::V2;

/// SO-101's arm joints, base to wrist, as its URDF names them.
const SO101_JOINTS: [&str; SO101_DOF] = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
];

/// The opening fraction at which the scenario poses the finger `link`: its joint's
/// position from `shut`, the position at which the finger is shut, toward the joint's
/// other limit.
fn opening(robot: &urdf_rs::Robot, configuration: &Configuration, link: &str, shut: f64) -> f64 {
    let joint = robot
        .joints
        .iter()
        .find(|joint| joint.child.link == link)
        .unwrap_or_else(|| panic!("{link} rides a joint"));
    let open = if shut == joint.limit.lower {
        joint.limit.upper
    } else {
        joint.limit.lower
    };
    (configuration.positions[&joint.name] - shut) / (open - shut)
}

#[test]
fn each_checked_and_left_out_distance_is_no_larger_than_fcls() {
    let scenarios = Cases::load_all();
    let urdfs: Vec<&str> = scenarios.iter().map(|cases| cases.urdf.as_str()).collect();
    assert!(
        urdfs.contains(&OPENARM.urdf()) && urdfs.contains(&so101_description::URDF),
        "the harness holds a self distance scenario of OpenArm {OPENARM} and of SO-101"
    );
    for cases in &scenarios {
        let robot = urdf_rs::read_from_string(&cases.urdf).expect("the URDF reads");
        let meshes = cases.meshes.to_str().expect("the mesh directory is UTF-8");
        if cases.urdf == OPENARM.urdf() {
            let mut model =
                openarm(OPENARM, meshes).unwrap_or_else(|e| panic!("{}: {e}", cases.name));
            // A gripper is posed by its first finger joint, which the second mimics.
            let first_finger = |side: Side| OPENARM.finger_links(side)[0];
            assert_each_distance_is_no_larger_than_fcls(
                cases,
                &mut model,
                |configuration| {
                    Side::BOTH.map(|side| -> [f64; OPENARM_DOF] {
                        std::array::from_fn(|i| {
                            configuration.positions[&format!("openarm_{side}_joint{}", i + 1)]
                        })
                    })
                },
                |configuration| {
                    Side::BOTH.map(|side| {
                        opening(
                            &robot,
                            configuration,
                            first_finger(side),
                            OPENARM.finger_shut(),
                        )
                    })
                },
            );
        } else if cases.urdf == so101_description::URDF {
            let mut model = so101(meshes).unwrap_or_else(|e| panic!("{}: {e}", cases.name));
            assert_each_distance_is_no_larger_than_fcls(
                cases,
                &mut model,
                |configuration| [SO101_JOINTS.map(|joint| configuration.positions[joint])],
                |configuration| [opening(&robot, configuration, JAW_LINK, jaw_shut_rad())],
            );
        } else {
            panic!(
                "{}: the parity test builds the self model of OpenArm {OPENARM} and of SO-101, \
                 and the scenario names neither's URDF",
                cases.name
            );
        }
    }
}

fn assert_each_distance_is_no_larger_than_fcls<const N: usize, const K: usize>(
    cases: &Cases,
    model: &mut SelfCollisionModel<N, K>,
    joints: impl Fn(&Configuration) -> Joints<N, K>,
    openings: impl Fn(&Configuration) -> [f64; K],
) {
    let name = &cases.name;
    let pairs: Vec<(String, String)> = model
        .checked_pairs()
        .into_iter()
        .map(|(a, b)| (a.to_owned(), b.to_owned()))
        .chain(model.excluded_pairs().iter().cloned())
        .collect();
    let (mut apart, mut in_contact) = (0, 0);
    for (index, configuration) in cases.configurations.iter().enumerate() {
        model.set_gripper_openings(&openings(configuration));
        let bodies = model
            .world_pieces(&joints(configuration))
            .expect("a finite configuration");
        let pieces_of = |body_name: &str| {
            &bodies
                .iter()
                .find(|(body, _)| *body == body_name)
                .unwrap_or_else(|| panic!("the model holds {body_name}"))
                .1
        };
        for (a, b) in &pairs {
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
        "{name} holds pairs apart and in contact: {apart} apart, {in_contact} in contact"
    );
}

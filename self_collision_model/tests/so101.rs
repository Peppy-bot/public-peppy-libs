//! SO-101's self-collision model, built from so101_description alone: the pairs it
//! checks, the pairs its joint limits keep apart, the jaw where its joint places it, and
//! the folded and the upright arm.

use std::path::Path;
use std::sync::LazyLock;

use chain_kinematics::{Chain, ChainSpec, JointSelection};
use collision_geometry::chain::ChainGeometry;
use collision_geometry::gjk;
use collision_geometry::urdf_collision::UrdfCollisions;
use self_collision_model::{So101Model, so101};
use so101_description::{
    ARM_DOF, BASE_LINK, JAW_LINK, NEVER_TOUCHING_PAIRS, TCP_LINK, URDF, jaw_position_rad,
    write_meshes_to,
};

/// The meshes, written once per test binary.
static MESHES: LazyLock<String> = LazyLock::new(|| {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("so101_meshes");
    write_meshes_to(&dir).expect("write the meshes");
    dir.to_str().expect("a UTF-8 directory").to_string()
});

fn model() -> So101Model {
    so101(&MESHES).expect("the model builds")
}

const ARM: ChainSpec<'static> = ChainSpec {
    base_link: Some(BASE_LINK),
    tip_link: TCP_LINK,
    joints: JointSelection::PathOrder,
};

/// The pairs whose bodies touch somewhere within the joint limits, which the model checks.
const CHECKED: [(&str, &str); 11] = [
    ("base_link", "gripper_link"),
    ("base_link", "lower_arm_link"),
    ("base_link", JAW_LINK),
    ("base_link", "wrist_link"),
    ("gripper_link", "shoulder_link"),
    ("gripper_link", "upper_arm_link"),
    ("lower_arm_link", "shoulder_link"),
    (JAW_LINK, "shoulder_link"),
    (JAW_LINK, "upper_arm_link"),
    ("shoulder_link", "wrist_link"),
    ("upper_arm_link", "wrist_link"),
];

/// The arm folded onto itself, as it rests.
const FOLDED: [f64; ARM_DOF] = [-0.05, -1.72, 1.68, 1.24, 0.32];

/// The arm upright, its wrist rolled a quarter turn (postures.py's `READY_POSITIONS_RAD`).
const UPRIGHT: [f64; ARM_DOF] = [0.0, 0.0, 0.0, 0.0, -std::f64::consts::FRAC_PI_2];

/// The clearance a plan of the SO-101 launchers keeps between the arm's own bodies: the
/// launchers' `min_clearance_m` and 1 mm of spare for each of a pair's two moving links.
const PLANNER_SELF_CLEARANCE_M: f64 = 0.022;

fn same(pair: (&str, &str), other: (&str, &str)) -> bool {
    pair == other || pair == (other.1, other.0)
}

// The model checks the pairs that touch somewhere within the joint limits: every pair of
// bodies of the arm, its base and its jaw that no joint joins, less the pairs the joint
// limits keep apart.
#[test]
fn it_checks_the_pairs_that_touch_within_the_joint_limits() {
    let model = model();
    let checked = model.checked_pairs();
    assert_eq!(checked.len(), CHECKED.len(), "{checked:?}");
    for pair in CHECKED {
        assert!(
            checked.iter().any(|held| same(*held, pair)),
            "{pair:?} is unchecked"
        );
    }
    let excluded: Vec<(&str, &str)> = model
        .excluded_pairs()
        .iter()
        .map(|(a, b)| (a.as_str(), b.as_str()))
        .collect();
    assert_eq!(excluded.len(), NEVER_TOUCHING_PAIRS.len());
    for [a, b] in NEVER_TOUCHING_PAIRS {
        assert!(excluded.iter().any(|held| same(*held, (a, b))), "{a} - {b}");
    }
}

/// The grid's step over each joint, in radians.
const GRID_STEP_RAD: f64 = 0.05;

/// The jaw's openings on the grid.
const OPENINGS: [f64; 3] = [0.0, 0.5, 1.0];

// The pairs the model leaves out, and the jaw against the wrist, which one joint and the
// jaw's own join, stay the least distance `NEVER_TOUCHING_PAIRS` states, to the millimeter
// below, or farther apart over a grid of the joints between each pair's two bodies within
// the limits, every other joint at 0, at each of OPENINGS.
#[test]
fn the_pairs_left_out_stay_apart_within_the_joint_limits() {
    let mut model = model();
    let robot = urdf_rs::read_from_string(URDF).expect("the URDF parses");
    let limits = Chain::<ARM_DOF>::from_urdf(&robot, &ARM)
        .expect("the arm")
        .limits();
    let grid = |j: usize| -> Vec<f64> {
        let steps = ((limits[j].hi - limits[j].lo) / GRID_STEP_RAD).ceil() as u32;
        (0..=steps)
            .map(|i| limits[j].lo + (limits[j].hi - limits[j].lo) * f64::from(i) / f64::from(steps))
            .collect()
    };
    // Every configuration of the grid over `joints`, each other joint at 0.
    let configurations = |joints: &[usize]| -> Vec<[f64; ARM_DOF]> {
        joints
            .iter()
            .fold(vec![[0.0; ARM_DOF]], |configurations, &j| {
                configurations
                    .iter()
                    .flat_map(|q| {
                        grid(j).into_iter().map(move |at| {
                            let mut q = *q;
                            q[j] = at;
                            q
                        })
                    })
                    .collect()
            })
    };
    // Each pair, with the joints whose turns move one of its bodies against the other, and
    // the least distance it keeps.
    let pairs: [((&str, &str), &[usize], f64); 4] = [
        ((BASE_LINK, "upper_arm_link"), &[0, 1], 0.017),
        (("gripper_link", "lower_arm_link"), &[3, 4], 0.038),
        (("lower_arm_link", JAW_LINK), &[3, 4], 0.046),
        ((JAW_LINK, "wrist_link"), &[4], 0.008),
    ];
    assert!(
        NEVER_TOUCHING_PAIRS
            .iter()
            .all(|[a, b]| pairs.iter().any(|(pair, _, _)| same(*pair, (a, b))))
    );
    for ((a, b), joints, kept) in pairs {
        let mut least = f64::INFINITY;
        for opening in OPENINGS {
            model.set_gripper_openings(&[opening]);
            for q in configurations(joints) {
                let bodies = model.world_pieces(&[q]).expect("a finite configuration");
                let pieces = |name: &str| {
                    &bodies
                        .iter()
                        .find(|(body, _)| *body == name)
                        .unwrap_or_else(|| panic!("the model holds {name}"))
                        .1
                };
                let distance = pieces(a)
                    .iter()
                    .flat_map(|pa| {
                        pieces(b)
                            .iter()
                            .map(move |pb| gjk::distance(pa, pb).distance)
                    })
                    .fold(f64::INFINITY, f64::min);
                least = least.min(distance);
            }
        }
        println!("{a} - {b}: least {least:.4} m");
        assert!(
            least >= kept,
            "{a} - {b} come {least} m apart, nearer than {kept} m"
        );
    }
}

// Folded as it rests, the arm's own bodies touch; upright, they stand farther apart than
// the planner's self clearance, the jaw shut or open.
#[test]
fn folded_the_arm_touches_itself_and_upright_it_stands_clear() {
    let mut model = model();
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
        let upright = model.min_distance(&[UPRIGHT]).expect("a distance");
        assert!(
            upright.distance > PLANNER_SELF_CLEARANCE_M,
            "upright at opening {opening}: {} - {} at {}",
            upright.link_a,
            upright.link_b,
            upright.distance
        );
    }
}

// The model hangs the jaw where its joint places it at the arm's opening: the offset the
// model gives the jaw the arm's chain carries is the jaw joint's offset at
// `jaw_position_rad` of the opening.
#[test]
fn the_jaw_hangs_where_its_joint_places_it() {
    let mut model = model();
    let chain = ChainGeometry::<ARM_DOF>::from_urdf(URDF, &MESHES, &ARM).expect("the arm fits");
    let joint = UrdfCollisions::from_urdf(URDF)
        .expect("the URDF reads")
        .parent_joint(JAW_LINK)
        .expect("the jaw rides a joint")
        .clone();
    for opening in [0.0, 0.3, 1.0] {
        model.set_gripper_openings(&[opening]);
        let fingers = model
            .fingers(0, chain.carried())
            .expect("the arm's chain carries the jaw");
        let [jaw] = fingers.as_slice() else {
            panic!("the arm carries one finger");
        };
        let placed = joint
            .offset(jaw_position_rad(opening).expect("a finite opening"))
            .expect("the jaw's joint places it");
        assert!(
            (jaw.offset.translation.vector - placed.translation.vector).norm() < 1e-12
                && jaw.offset.rotation.angle_to(&placed.rotation) < 1e-12,
            "at opening {opening}: the model hangs the jaw at {}, its joint places it at {placed}",
            jaw.offset
        );
    }
}

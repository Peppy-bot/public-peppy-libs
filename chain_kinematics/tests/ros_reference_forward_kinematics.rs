//! Forward kinematics against MoveIt: `Chain::at` gives the world pose of each link of a
//! serial chain read from a URDF, at a configuration of its joints. The evidence: at every
//! configuration of every forward kinematics scenario of the ROS reference harness (the
//! zero configuration, both limits, and configurations drawn within the limits), each
//! link pose agrees with the global link transform of MoveIt's robot state within `ULPS`
//! units of `f64::EPSILON`, scaled by the coordinate.

use chain_kinematics::nalgebra::Isometry3;
use chain_kinematics::{Chain, ChainSpec, JointSelection};
use ros_reference::LinkPose;
use ros_reference::forward_kinematics::{Cases, ChainCase};
use urdf_rs::Robot;

/// The largest difference allowed between a coordinate of ours and MoveIt's `x`, in units
/// of `f64::EPSILON * max(1, |x|)`. A pose of ours holds its rotation as a unit
/// quaternion and MoveIt's as a matrix, so the two round differently. Over the scenarios,
/// the largest difference measured is 7.5 of these units, against the amd64 outputs and
/// against those of the arm64 image alike, with our side on the sine and cosine of glibc
/// (amd64 FMA and SSE2 variants, and arm64) and of macOS. Over 50 drawn configurations a
/// chain, a set that runs locally (`draw_forward_kinematics.py --drawn 50`), it is 8,
/// against the arm64 outputs, and the bound is twice that. Every coordinate of the
/// scenarios is at most 1.20 in magnitude, so the bound is at most 4.3e-15.
const ULPS: f64 = 16.0;

/// The largest difference that the parity of forward kinematics with MoveIt allows at any
/// coordinate.
const LARGEST_TOLERANCE: f64 = 1e-12;

/// The difference allowed between a coordinate of ours and MoveIt's `moveit`.
fn tolerance(moveit: f64) -> f64 {
    ULPS * f64::EPSILON * moveit.abs().max(1.0)
}

/// Asserts that our pose of `link` agrees with MoveIt's pose of the same link.
fn assert_pose_agrees(link: &str, ours: &Isometry3<f64>, moveit: &LinkPose, at: &str) {
    assert_eq!(link, moveit.link, "{at}: the links are in one order");
    let rotation = ours.rotation.to_rotation_matrix();
    let translations = (0..3).map(|i| {
        let coordinate = format!("translation {i}");
        (
            coordinate,
            ours.translation.vector[i],
            moveit.translation[i],
        )
    });
    let rotations = (0..9).map(|i| {
        let (row, column) = (i / 3, i % 3);
        let coordinate = format!("rotation ({row}, {column})");
        (coordinate, rotation[(row, column)], moveit.rotation[i])
    });
    for (coordinate, ours, theirs) in translations.chain(rotations) {
        let allowed = tolerance(theirs);
        assert!(
            allowed <= LARGEST_TOLERANCE,
            "{at} {link}: {coordinate} of MoveIt, {theirs}, allows {allowed}, beyond \
             {LARGEST_TOLERANCE}"
        );
        assert!(
            (ours - theirs).abs() <= allowed,
            "{at} {link}: {coordinate} is {ours}, MoveIt gives {theirs}"
        );
    }
}

fn assert_chain_agrees_with_moveit<const N: usize>(robot: &Robot, chain: &ChainCase, at: &str) {
    let joints: Vec<&str> = chain.joints.iter().map(String::as_str).collect();
    let kinematics = Chain::<N>::from_urdf(
        robot,
        &ChainSpec {
            base_link: None,
            tip_link: &chain.tip_link,
            joints: JointSelection::Named(&joints),
        },
    )
    .unwrap_or_else(|e| panic!("{at}: {e}"));
    for (index, configuration) in chain.configurations.iter().enumerate() {
        let q: [f64; N] = configuration
            .positions
            .as_slice()
            .try_into()
            .expect("the harness pairs one position per joint");
        let posed = kinematics.at(&q);
        let at = format!("{at} configuration {index}");
        for (i, moveit) in configuration.links.iter().enumerate() {
            assert_pose_agrees(posed.link_name(i), &posed.link_pose_world(i), moveit, &at);
        }
        assert_pose_agrees(&chain.tip_link, &posed.tip_pose(), &configuration.tip, &at);
    }
}

#[test]
fn every_forward_kinematics_scenario_agrees_with_moveit() {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a forward kinematics scenario"
    );
    for scenario in &scenarios {
        let robot = urdf_rs::read_from_string(&scenario.urdf).expect("the URDF parses");
        for chain in &scenario.chains {
            let at = format!("{} {}", scenario.name, chain.name);
            match chain.joints.len() {
                1 => assert_chain_agrees_with_moveit::<1>(&robot, chain, &at),
                2 => assert_chain_agrees_with_moveit::<2>(&robot, chain, &at),
                3 => assert_chain_agrees_with_moveit::<3>(&robot, chain, &at),
                4 => assert_chain_agrees_with_moveit::<4>(&robot, chain, &at),
                5 => assert_chain_agrees_with_moveit::<5>(&robot, chain, &at),
                6 => assert_chain_agrees_with_moveit::<6>(&robot, chain, &at),
                7 => assert_chain_agrees_with_moveit::<7>(&robot, chain, &at),
                n => panic!(
                    "{at}: the parity test poses chains of 1 to 7 joints, and this one has {n}"
                ),
            }
        }
    }
}

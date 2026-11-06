//! Cartesian paths of OpenArm v2's left arm: the frame stays within the precision of its
//! lines at every state of the rounded path the move runs, a box on the way ends the path
//! before it and ends nothing when collisions go unchecked, the fraction counts each line
//! equally, a start nearer than the clearance departs in one run, and what the path
//! cannot answer says why.

mod common;

use std::sync::atomic::AtomicBool;

use chain_kinematics::interpolate;
use control_core::path_timing::RoundedPath;
use control_core::positive_finite::PositiveFinite;
use hegel::TestCase;
use hegel::generators as gs;
use motion_planning_core::{
    Arm, Arms, CartesianFailure, CartesianPath, CartesianRequest, CheckRequest, Collisions,
    FollowedFrame, Frame, Objects, PRECISION_M, PRECISION_RAD, Scene, StartFault, cartesian_path,
    check_path,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use occupancy_map::{DistanceField, VoxelMap};

use common::{
    JOINTS, LEFT_ARM, an_hour_from_now, distance_to_map, map, occupied_at, off_the_branch_seams,
    on_line,
};

const VOXEL_M: f64 = 0.02;
const MIN_CLEARANCE_M: f64 = 0.02;
/// The seed of each path's draws.
const SEED: u64 = 7;
/// The gripper pointing down, from where the grasp point reaches 20 cm down, 10 cm up and 5
/// cm along x.
const START: [f64; JOINTS] = [0.426, -3.075, 0.86, 2.245, -0.41, 0.534, 0.136];
/// How far the joints move between two states the oracle compares with the lines.
const ORACLE_SPACING_RAD: f64 = 2e-3;

/// The grasp point's pose at `q`.
fn hand(q: &[f64; JOINTS]) -> Isometry3<f64> {
    LEFT_ARM.chain().world_ee_pose(q)
}

/// The grasp point, as a Cartesian path follows it.
fn grasp_point() -> FollowedFrame {
    FollowedFrame::of(LEFT_ARM.chain(), Frame::GraspPoint).expect("a Cartesian path follows it")
}

/// `pose` moved by `by`, its orientation kept.
fn moved(pose: &Isometry3<f64>, by: Vector3<f64>) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(pose.translation.vector + by),
        pose.rotation,
    )
}

/// A map of `VOXEL_M` voxels holding `points`, and its distance field.
fn world(points: &[[f64; 3]]) -> (VoxelMap, DistanceField) {
    let mut map = map(VOXEL_M, 1.6);
    occupied_at(&mut map, [0.0, 0.0, 3.0], points);
    let field = DistanceField::of(&map);
    (map, field)
}

/// The points of a solid cube of voxels of half side `half_m` about `center`.
fn cube(center: Vector3<f64>, half_m: f64) -> Vec<[f64; 3]> {
    let count = (2.0 * half_m / VOXEL_M).ceil() as usize;
    let at = |k: usize| -half_m + k as f64 * VOXEL_M;
    (0..=count)
        .flat_map(|i| (0..=count).flat_map(move |j| (0..=count).map(move |k| (i, j, k))))
        .map(|(i, j, k)| [center.x + at(i), center.y + at(j), center.z + at(k)])
        .collect()
}

/// The scene of `map` and `field`, with no object.
fn scene<'a>(map: &'a VoxelMap, field: &'a DistanceField) -> Scene<'a> {
    Scene {
        map,
        field,
        stamp: 1,
        objects: Objects::none(),
    }
}

/// The path of the grasp point from `start` through `poses`.
fn path_of(
    start: [f64; JOINTS],
    poses: &[Isometry3<f64>],
    collisions: Collisions<'_>,
) -> Result<CartesianPath<JOINTS>, CartesianFailure> {
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    cartesian_path(
        &arms,
        &CartesianRequest {
            start,
            start_tolerance: [0.0; JOINTS],
            frame: grasp_point(),
            waypoints: poses,
            collisions,
            seed: SEED,
            stop: None,
        },
    )
}

/// The same, keeping `MIN_CLEARANCE_M` from `scene`.
fn checked_path_of(
    start: [f64; JOINTS],
    poses: &[Isometry3<f64>],
    scene: &Scene<'_>,
) -> Result<CartesianPath<JOINTS>, CartesianFailure> {
    path_of(
        start,
        poses,
        Collisions::Checked {
            scene,
            min_clearance_m: MIN_CLEARANCE_M,
        },
    )
}

/// The rounded path the move runs through `path`'s answer.
fn rounded(path: &CartesianPath<JOINTS>) -> RoundedPath {
    let joints: Vec<Vec<f64>> = std::iter::once(path.start)
        .chain(path.waypoints.iter().copied())
        .map(|q| q.to_vec())
        .collect();
    RoundedPath::joint_path(&joints)
}

/// The poses of each line the order of a path's states is judged against: closer together
/// than the precision on the lines of these tests.
const ALONG_SAMPLES: usize = 1000;

/// Asserts that the grasp point stands within the precision of the lines from `path`'s
/// start through `poses` at every state of the rounded path the move runs, and that the
/// path's states lie in order along them, each within twice the precision of a pose no
/// earlier than the state before it.
fn assert_on_its_lines(path: &CartesianPath<JOINTS>, poses: &[Isometry3<f64>]) {
    let corners: Vec<Isometry3<f64>> = std::iter::once(hand(&path.start))
        .chain(poses.iter().copied())
        .collect();
    let rounded = rounded(path);
    let spacing = PositiveFinite::parse(ORACLE_SPACING_RAD).expect("positive");
    for s in rounded.arcs_every(spacing) {
        let q: [f64; JOINTS] = rounded.at(s).try_into().expect("seven joints");
        let actual = hand(&q);
        assert!(
            corners.windows(2).any(|c| on_line(&actual, &c[0], &c[1])),
            "the grasp point at {q:?} stands off its lines: {actual:?}"
        );
    }
    let along: Vec<Isometry3<f64>> = corners
        .windows(2)
        .flat_map(|c| {
            (0..=ALONG_SAMPLES).map(|k| interpolate(&c[0], &c[1], k as f64 / ALONG_SAMPLES as f64))
        })
        .collect();
    path.waypoints.iter().fold(0, |earliest, q| {
        let actual = hand(q);
        let place = along[earliest..].iter().position(|pose| {
            (actual.translation.vector - pose.translation.vector).norm() <= 2.0 * PRECISION_M
                && actual.rotation.angle_to(&pose.rotation) <= 2.0 * PRECISION_RAD
        });
        earliest + place.unwrap_or_else(|| panic!("the path turns back along its lines at {q:?}"))
    });
}

/// Asserts that a check of a caller's path at `min_clearance_m` passes `path`'s answer.
fn assert_passes_a_check_at(path: &CartesianPath<JOINTS>, scene: &Scene<'_>, min_clearance_m: f64) {
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let check = check_path(
        &arms,
        scene,
        &CheckRequest {
            start: path.start,
            start_tolerance: [0.0; JOINTS],
            waypoints: &path.waypoints,
            min_clearance_m,
            deadline: an_hour_from_now(),
        },
    )
    .expect("the check answers");
    assert_eq!(check.fault, None);
}

/// Asserts that a check of a caller's path passes `path`'s answer.
fn assert_passes_a_check(path: &CartesianPath<JOINTS>, scene: &Scene<'_>) {
    assert_passes_a_check_at(path, scene, MIN_CLEARANCE_M);
}

// A free path through two poses covers both, ends at the last within the precision, stays
// on its lines at every state the move runs, and passes the check of a caller's path.
#[test]
fn a_free_path_covers_its_poses_on_its_lines() {
    let (map, field) = world(&[]);
    let scene = scene(&map, &field);
    let first = moved(&hand(&START), Vector3::new(0.0, 0.0, -0.1));
    let poses = [first, moved(&first, Vector3::new(0.05, 0.0, 0.0))];
    let path = checked_path_of(START, &poses, &scene).expect("a path");
    assert_eq!(path.fraction, 1.0);
    let end = hand(path.waypoints.last().expect("a waypoint"));
    assert!((end.translation.vector - poses[1].translation.vector).norm() <= PRECISION_M);
    assert!(end.rotation.angle_to(&poses[1].rotation) <= PRECISION_RAD);
    assert_on_its_lines(&path, &poses);
    assert_passes_a_check(&path, &scene);
}

// A line through a box ends before it: the fraction lies below 1, the grasp point ends on
// the line at that share of it, and the answer passes the check of a caller's path. With
// collisions unchecked, the whole line is covered.
#[test]
fn a_line_through_a_box_ends_before_it_unless_collisions_go_unchecked() {
    let start = hand(&START);
    let target = moved(&start, Vector3::new(0.0, 0.0, -0.2));
    let box_center = start.translation.vector - Vector3::new(0.0, 0.0, 0.14);
    let (map, field) = world(&cube(box_center, 0.04));
    let scene = scene(&map, &field);
    let poses = [target];
    let path = checked_path_of(START, &poses, &scene).expect("a path");
    assert!(
        path.fraction > 0.0 && path.fraction < 1.0,
        "{}",
        path.fraction
    );
    let end = hand(path.waypoints.last().expect("a waypoint"));
    let expected = interpolate(&start, &target, path.fraction);
    assert!(
        (end.translation.vector - expected.translation.vector).norm() <= PRECISION_M,
        "{end:?} against {expected:?}"
    );
    assert_on_its_lines(&path, &poses);
    assert_passes_a_check(&path, &scene);

    let unchecked = path_of(START, &poses, Collisions::Unchecked).expect("a path");
    assert_eq!(unchecked.fraction, 1.0);
    assert_on_its_lines(&unchecked, &poses);
}

/// How far under the distance of the nearest state the clearance of the next test stands.
const JUST_UNDER_M: f64 = 1e-6;
/// The spare a state keeps above the clearance a caller asks of it, from the map:
/// motion_planning_core's plan clearance.
const STATE_SPARE_M: f64 = 1e-3;

// A line past a box, at a clearance every state of it keeps by a micrometer
// over its spare, ends where the motion between two states dips under that clearance: the
// check of the path the move runs finds the dip, which no state shows.
#[test]
fn a_motion_that_dips_under_the_clearance_between_its_states_ends_before_the_dip() {
    let start = hand(&START);
    let center = start.translation.vector + Vector3::new(0.12, 0.0, -0.1);
    let (map, field) = world(&cube(center, 0.02));
    let scene = scene(&map, &field);
    let poses = [moved(&start, Vector3::new(0.0, 0.0, -0.2))];
    let free = checked_path_of(START, &poses, &scene).expect("a path");
    assert_eq!(free.fraction, 1.0, "the line passes the box");
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let nearest_state = free
        .waypoints
        .iter()
        .map(|q| distance_to_map(&arms, &map, q))
        .fold(f64::INFINITY, f64::min);
    let rounded = rounded(&free);
    let spacing = PositiveFinite::parse(ORACLE_SPACING_RAD).expect("positive");
    let nearest_motion = rounded
        .arcs_every(spacing)
        .into_iter()
        .map(|s| {
            let q: [f64; JOINTS] = rounded.at(s).try_into().expect("seven joints");
            distance_to_map(&arms, &map, &q)
        })
        .fold(f64::INFINITY, f64::min);
    assert!(
        nearest_motion < nearest_state - JUST_UNDER_M,
        "the motion comes no nearer than its states: {nearest_motion} and {nearest_state}"
    );
    let dipping = path_of(
        START,
        &poses,
        Collisions::Checked {
            scene: &scene,
            min_clearance_m: nearest_state - STATE_SPARE_M - JUST_UNDER_M,
        },
    )
    .expect("a path");
    assert!(dipping.fraction < 1.0, "{}", dipping.fraction);
}

// Each line counts equally: a first line whole and a second that ends at a box cover more
// than half of the path and less than all of it, and a first line that ends at the box
// covers less than half and never reaches the second.
#[test]
fn the_fraction_counts_each_line_equally() {
    let start = hand(&START);
    let above_box = moved(&start, Vector3::new(0.05, 0.0, 0.0));
    let into_box = moved(&above_box, Vector3::new(0.0, 0.0, -0.2));
    let box_center = above_box.translation.vector - Vector3::new(0.0, 0.0, 0.14);
    let (map, field) = world(&cube(box_center, 0.04));
    let scene = scene(&map, &field);

    let second_blocked = checked_path_of(START, &[above_box, into_box], &scene).expect("a path");
    assert!(
        second_blocked.fraction > 0.5 && second_blocked.fraction < 1.0,
        "{}",
        second_blocked.fraction
    );

    let first_blocked = checked_path_of(START, &[into_box, above_box], &scene).expect("a path");
    assert!(first_blocked.fraction < 0.5, "{}", first_blocked.fraction);
}

// A line that neither moves nor turns takes none of the way: the path covers the lines beside
// it, each counting whole. Its end state, which repeats the state before it, leaves no leg of
// the rounded path paired with a pose it does not move.
#[test]
fn a_line_that_stands_still_beside_others_counts_whole() {
    let here = hand(&START);
    let down = moved(&here, Vector3::new(0.0, 0.0, -0.1));
    for poses in [vec![here, down], vec![down, down], vec![here, here, down]] {
        let path = path_of(START, &poses, Collisions::Unchecked).expect("a path");
        assert_eq!(path.fraction, 1.0, "{poses:?}");
        assert_on_its_lines(&path, &poses);
    }
}

// A short line after a corner, then the same pose again: the states the corner's halving adds
// stay in order along the lines.
#[test]
fn a_last_line_that_stands_still_keeps_the_path_in_order_along_its_lines() {
    let down = moved(&hand(&START), Vector3::new(0.0, 0.0, -0.05));
    let aside = moved(&down, Vector3::new(0.009, 0.0, 0.0));
    let poses = [down, aside, aside];
    let path = path_of(START, &poses, Collisions::Unchecked).expect("a path");
    assert_eq!(path.fraction, 1.0);
    assert_on_its_lines(&path, &poses);
}

// A line that neither moves nor turns counts whole once reached, and the path stands where
// it starts.
#[test]
fn a_line_that_neither_moves_nor_turns_counts_whole() {
    let path = path_of(START, &[hand(&START)], Collisions::Unchecked).expect("a path");
    assert_eq!(path.fraction, 1.0);
    assert_eq!(path.waypoints.len(), 1);
    assert!((0..JOINTS).all(|i| (path.waypoints[0][i] - START[i]).abs() <= 1e-3));
}

// A pose at the same position turns in place, within the precision of its slerp.
#[test]
fn a_pose_at_the_same_position_turns_in_place() {
    let start = hand(&START);
    let turned = Isometry3::from_parts(
        start.translation,
        start.rotation * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.3),
    );
    let poses = [turned];
    let path = path_of(START, &poses, Collisions::Unchecked).expect("a path");
    assert_eq!(path.fraction, 1.0);
    assert_on_its_lines(&path, &poses);
}

// A link fixed to the last joint's link follows the lines as the grasp point does.
#[test]
fn a_link_fixed_to_the_last_joint_follows_the_lines() {
    let offset = Isometry3::translation(0.0, 0.0, 0.05);
    let link_pose = |q: &[f64; JOINTS]| LEFT_ARM.chain().at(q).link_pose_world(JOINTS - 1) * offset;
    let target = moved(&link_pose(&START), Vector3::new(0.0, 0.0, -0.05));
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let path = cartesian_path(
        &arms,
        &CartesianRequest {
            start: START,
            start_tolerance: [0.0; JOINTS],
            frame: FollowedFrame::of(
                LEFT_ARM.chain(),
                Frame::Link {
                    segment: JOINTS - 1,
                    offset,
                },
            )
            .expect("a Cartesian path follows it"),
            waypoints: &[target],
            collisions: Collisions::Unchecked,
            seed: SEED,
            stop: None,
        },
    )
    .expect("a path");
    assert_eq!(path.fraction, 1.0);
    let end = link_pose(path.waypoints.last().expect("a waypoint"));
    assert!((end.translation.vector - target.translation.vector).norm() <= PRECISION_M);
    assert!(end.rotation.angle_to(&target.rotation) <= PRECISION_RAD);
}

// From a start nearer than the clearance, a line away from the obstacle departs in one run
// to a first waypoint that keeps the clearance and covers the whole line, stands on it, and
// passes the check of a caller's path; a line toward it ends at the start. A corner at the
// end of the run halves the leg after it, so a path through it covers its lines.
#[test]
fn a_start_nearer_than_the_clearance_departs_in_one_run() {
    let start = hand(&START);
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let box_center = start.translation.vector - Vector3::new(0.0, 0.0, 0.1);
    let (map, field) = world(&cube(box_center, 0.04));
    let scene = scene(&map, &field);
    let standing = distance_to_map(&arms, &map, &START);
    assert!(standing > 0.0, "the start touches the box: {standing}");
    // The start falls 2 cm short of this clearance, more than one step's worth of a line.
    let min_clearance_m = standing + 0.02;
    let asking = |poses: &[Isometry3<f64>]| {
        cartesian_path(
            &arms,
            &CartesianRequest {
                start: START,
                start_tolerance: [0.0; JOINTS],
                frame: grasp_point(),
                waypoints: poses,
                collisions: Collisions::Checked {
                    scene: &scene,
                    min_clearance_m,
                },
                seed: SEED,
                stop: None,
            },
        )
        .expect("a path")
    };
    let up = [moved(&start, Vector3::new(0.0, 0.0, 0.1))];
    let away = asking(&up);
    assert_eq!(away.fraction, 1.0);
    assert_on_its_lines(&away, &up);
    assert_passes_a_check_at(&away, &scene, min_clearance_m);
    let first = away.waypoints.first().expect("a waypoint");
    assert!(distance_to_map(&arms, &map, first) >= min_clearance_m);

    let toward = asking(&[moved(&start, Vector3::new(0.0, 0.0, -0.05))]);
    assert_eq!(toward.fraction, 0.0);
    assert!(toward.waypoints.is_empty());

    let up_to_the_run_end = moved(&start, Vector3::new(0.0, 0.0, 0.029));
    let around = [
        up_to_the_run_end,
        moved(&up_to_the_run_end, Vector3::new(0.05, 0.0, 0.0)),
    ];
    let cornered = asking(&around);
    assert_eq!(cornered.fraction, 1.0);
    assert_on_its_lines(&cornered, &around);
    assert_passes_a_check_at(&cornered, &scene, min_clearance_m);
}

// A start in contact fails where collisions are checked, and walks where they are not.
#[test]
fn a_start_in_contact_fails_unless_collisions_go_unchecked() {
    let last_link = LEFT_ARM.chain().at(&START).link_pose_world(JOINTS - 1);
    let (map, field) = world(&cube(last_link.translation.vector, 0.02));
    let scene = scene(&map, &field);
    let pose = moved(&hand(&START), Vector3::new(0.0, 0.0, 0.05));
    let failure = checked_path_of(START, &[pose], &scene).expect_err("a start in contact");
    assert!(
        matches!(failure, CartesianFailure::Start(StartFault::InCollision(_))),
        "{failure}"
    );
    let unchecked = path_of(START, &[pose], Collisions::Unchecked).expect("a path");
    assert_eq!(unchecked.fraction, 1.0);
}

// A start outside a joint limit by more than its tolerance fails, and one within it moves
// onto the limit.
#[test]
fn a_start_outside_a_limit_fails_past_its_tolerance_and_moves_onto_it_within() {
    let limit = LEFT_ARM.chain().limits()[0].hi;
    let mut outside = START;
    outside[0] = limit + 0.01;
    let pose = hand(&START);
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let asking = |tolerance: f64| {
        cartesian_path(
            &arms,
            &CartesianRequest {
                start: outside,
                start_tolerance: [tolerance; JOINTS],
                frame: grasp_point(),
                waypoints: &[pose],
                collisions: Collisions::Unchecked,
                seed: SEED,
                stop: None,
            },
        )
    };
    let failure = asking(0.0).expect_err("outside its limit");
    assert!(
        matches!(
            failure,
            CartesianFailure::Start(StartFault::OutsideLimits { .. })
        ),
        "{failure}"
    );
    assert_eq!(asking(0.02).expect("a path").start[0], limit);
}

// No pose, and a pose that is not finite, are refused and say so.
#[test]
fn poses_a_path_cannot_pass_through_are_refused() {
    let here = hand(&START);
    let refusal = |poses: &[Isometry3<f64>]| {
        path_of(START, poses, Collisions::Unchecked)
            .expect_err("a refusal")
            .to_string()
    };
    let no_pose = refusal(&[]);
    assert!(no_pose.contains("send one or more"), "{no_pose}");
    let not_finite = moved(&here, Vector3::new(f64::NAN, 0.0, 0.0));
    let refused = refusal(&[here, not_finite]);
    assert!(refused.contains("waypoint 2"), "{refused}");
}

// A path follows the grasp point and a link the arm's every joint moves; a link fewer
// joints move, and a part a link carries, are no frames of a path.
#[test]
fn a_path_follows_only_a_frame_every_joint_moves() {
    let chain = LEFT_ARM.chain();
    let offset = Isometry3::identity();
    assert!(FollowedFrame::of(chain, Frame::GraspPoint).is_some());
    let last = JOINTS - 1;
    assert!(
        FollowedFrame::of(
            chain,
            Frame::Link {
                segment: last,
                offset
            }
        )
        .is_some()
    );
    assert!(FollowedFrame::of(chain, Frame::Link { segment: 3, offset }).is_none());
    assert!(
        FollowedFrame::of(
            chain,
            Frame::Part {
                segment: last,
                offset
            }
        )
        .is_none()
    );
}

// A set stop ends the path.
#[test]
fn a_set_stop_ends_the_path() {
    let stop = AtomicBool::new(true);
    let arms = Arms::new(vec![Arm::new("left_arm", &LEFT_ARM)], None);
    let failure = cartesian_path(
        &arms,
        &CartesianRequest {
            start: START,
            start_tolerance: [0.0; JOINTS],
            frame: grasp_point(),
            waypoints: &[moved(&hand(&START), Vector3::new(0.0, 0.0, -0.05))],
            collisions: Collisions::Unchecked,
            seed: SEED,
            stop: Some(&stop),
        },
    )
    .expect_err("a stopped path");
    assert_eq!(failure, CartesianFailure::Stopped);
}

// The same request gives the same path.
#[test]
fn the_same_request_gives_the_same_path() {
    let poses = [moved(&hand(&START), Vector3::new(0.0, 0.05, -0.1))];
    assert_eq!(
        path_of(START, &poses, Collisions::Unchecked).expect("a path"),
        path_of(START, &poses, Collisions::Unchecked).expect("a path")
    );
}

// From a drawn start, along two drawn lines that each move up to 10 cm each way and turn up to
// 0.3 rad, whatever share of them the path covers, the grasp point stands within the
// precision of its lines at every state of the rounded path the move runs.
#[hegel::test(test_cases = 60)]
fn every_state_the_move_runs_stands_within_the_precision_of_the_line(tc: TestCase) {
    let seed = tc.draw(gs::integers::<u64>());
    let start = off_the_branch_seams(seed).next().expect("a start");
    let offset = || -> f64 { tc.draw(gs::floats().min_value(-0.1).max_value(0.1)) };
    let turn = || -> f64 { tc.draw(gs::floats().min_value(-0.3).max_value(0.3)) };
    let on_from = |from: &Isometry3<f64>| {
        let by = Vector3::new(offset(), offset(), offset());
        let rotation = UnitQuaternion::from_scaled_axis(Vector3::new(turn(), turn(), turn()));
        Isometry3::from_parts(
            Translation3::from(from.translation.vector + by),
            from.rotation * rotation,
        )
    };
    let first = on_from(&hand(&start));
    let second = on_from(&first);
    let poses = [first, second, second];
    let path = path_of(start, &poses, Collisions::Unchecked).expect("a path");
    assert!((0.0..=1.0).contains(&path.fraction), "{}", path.fraction);
    assert_on_its_lines(&path, &poses);
}

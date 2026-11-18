//! Properties of the base kept in a map of a drawn room, a rectangle with a box in one
//! corner or an oblong L, so no pose sees what another sees:
//!
//! - in an oblong L-shaped room, a start pose whose search holds the true pose matches
//!   within one cell of the map and one coarse angle step of the matcher of it, and one whose
//!   true pose lies past the search by the matcher's reach gives no match; a search
//!   angle of pi or more matches from any heading;
//! - driven again through the room from a start pose, with odometry in a frame of its own,
//!   the base stays localized near its true pose;
//! - after mapping, the pose goes on from the mapping's estimate within one cell of the
//!   true pose;
//! - scans that match nothing leave the base not localized, and scans that match again
//!   localize it again.

use camera_core::laser_scan::{LaserScan, NO_RETURN_MARK_M};
use camera_core::pose::PlanarPose;
use hegel::TestCase;
use hegel::generators as gs;
use mapping_core::{InitialPose, InitialPoseFailure, Localization, MapPose, RoomMap};

mod common;
use common::properties::{
    Room, asymmetric_room, circle_drive, compose, drifting_odometry, heading_error, loop_drive,
    planar, position_error, relative, room,
};
use common::{
    CREATED_AT_UNIX_NS, LOCALIZED_POSITION, SCAN_PERIOD_NS, localization_parameters, map_of,
    placed, room_map_of, slam_toolbox_localization, sync_parameters,
};

/// A pose of the drive through the room, moved by up to 0.2 m and 0.2 rad, so no scan of
/// the map was taken there.
fn true_pose(tc: &TestCase, drive: &[PlanarPose]) -> PlanarPose {
    let index: usize = tc.draw(gs::integers().min_value(0).max_value(drive.len() - 1));
    compose(
        drive[index],
        planar(
            tc.draw(gs::floats().min_value(-0.2).max_value(0.2)),
            tc.draw(gs::floats().min_value(-0.2).max_value(0.2)),
            tc.draw(gs::floats().min_value(-0.2).max_value(0.2)),
        ),
    )
}

/// `set_initial_pose` at `initial` with its search, a radius and an angle, the base
/// standing at `truth` by its odometry and in the room.
fn start(
    room: &Room,
    map: &RoomMap,
    initial: PlanarPose,
    search: (f64, f64),
    truth: PlanarPose,
) -> Result<(Localization, MapPose), InitialPoseFailure> {
    let initial = InitialPose::parse(initial, search.0, search.1).expect("valid");
    Localization::from_initial_pose(
        map,
        localization_parameters(),
        &initial,
        truth,
        &placed(0, truth, &room.scan(truth)),
    )
}

/// Whether `matched` lies within one cell of the map and one coarse angle step of the
/// matcher of `truth`.
fn within_one_step(matched: PlanarPose, truth: PlanarPose) -> bool {
    let parameters = slam_toolbox_localization();
    position_error(matched, truth) <= parameters.resolution
        && heading_error(matched, truth) <= parameters.coarse_angle_resolution
}

#[hegel::test(test_cases = 10)]
fn a_start_whose_search_holds_the_true_pose_matches_within_one_cell_and_one_angular_step(
    tc: TestCase,
) {
    let room = asymmetric_room(&tc);
    let drive = loop_drive(&tc, &room);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let truth = true_pose(&tc, &drive);
    let search_radius: f64 = tc.draw(gs::floats().min_value(0.2).max_value(0.6));
    let search_angle: f64 = tc.draw(gs::floats().min_value(0.1).max_value(1.0));
    let distance: f64 = tc.draw(gs::floats().min_value(0.0).max_value(0.9 * search_radius));
    let direction: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
    let turn = tc.draw(gs::floats::<f64>().min_value(-0.9).max_value(0.9)) * search_angle;
    let initial = planar(
        truth.x() + distance * direction.cos(),
        truth.y() + distance * direction.sin(),
        truth.yaw() + turn,
    );
    let (_, matched) = start(&room, &map, initial, (search_radius, search_angle), truth)
        .unwrap_or_else(|failure| panic!("{truth:?} from {initial:?}: {failure:?}"));
    assert!(matched.localized);
    assert!(
        within_one_step(matched.pose, truth),
        "{truth:?} from {initial:?} matched at {:?}",
        matched.pose
    );
}

#[hegel::test(test_cases = 10)]
fn a_start_whose_search_leaves_out_the_true_pose_gives_no_match(tc: TestCase) {
    let room = asymmetric_room(&tc);
    let drive = loop_drive(&tc, &room);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let truth = true_pose(&tc, &drive);
    let search_radius: f64 = tc.draw(gs::floats().min_value(0.2).max_value(0.6));
    let search_angle: f64 = tc.draw(gs::floats().min_value(0.1).max_value(1.0));
    // The true pose lies past the search by the matcher's reach.
    let reach = slam_toolbox_localization().matcher_reach();
    let outside_by_position: bool = tc.draw(gs::booleans());
    let direction: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1));
    let (distance, turn) = if outside_by_position {
        (search_radius + reach.position_m, 0.0)
    } else {
        (0.0, search_angle + reach.yaw_rad)
    };
    let initial = planar(
        truth.x() + distance * direction.cos(),
        truth.y() + distance * direction.sin(),
        truth.yaw() + turn,
    );
    let failure = start(&room, &map, initial, (search_radius, search_angle), truth)
        .err()
        .unwrap_or_else(|| panic!("{truth:?} from {initial:?} matched"));
    assert!(
        matches!(failure, InitialPoseFailure::NoMatch { .. }),
        "{failure:?}"
    );
}

#[hegel::test(test_cases = 10)]
fn a_search_angle_of_pi_or_more_matches_from_any_heading(tc: TestCase) {
    let room = asymmetric_room(&tc);
    let drive = loop_drive(&tc, &room);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let truth = true_pose(&tc, &drive);
    let search_angle: f64 = tc.draw(gs::floats().min_value(std::f64::consts::PI).max_value(7.0));
    let heading: f64 = tc.draw(gs::floats().min_value(-10.0).max_value(10.0));
    let initial = planar(truth.x() + 0.1, truth.y() - 0.1, heading);
    let (_, matched) = start(&room, &map, initial, (0.3, search_angle), truth)
        .unwrap_or_else(|failure| panic!("{truth:?} from {initial:?}: {failure:?}"));
    assert!(
        within_one_step(matched.pose, truth),
        "{truth:?} from {initial:?} matched at {:?}",
        matched.pose
    );
}

#[hegel::test(test_cases = 5)]
fn a_base_driven_again_through_a_mapped_room_stays_localized_near_its_truth(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let map = room_map_of(
        &room,
        &drive,
        &drifting_odometry(&tc, &drive),
        sync_parameters(),
    );
    // The second drive's odometry starts in a frame of its own.
    let frame = planar(
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        tc.draw(gs::floats().min_value(-5.0).max_value(5.0)),
        tc.draw(gs::floats().min_value(-3.0).max_value(3.0)),
    );
    let odometry: Vec<PlanarPose> = drifting_odometry(&tc, &drive)
        .into_iter()
        .map(|pose| compose(frame, pose))
        .collect();
    let initial = InitialPose::parse(drive[0], 0.3, 0.3).expect("valid");
    let (mut localization, _) = Localization::from_initial_pose(
        &map,
        localization_parameters(),
        &initial,
        odometry[0],
        &placed(0, odometry[0], &room.scan(drive[0])),
    )
    .expect("the first scan matches at the start");
    for (index, (truth, odometry)) in drive.iter().zip(&odometry).enumerate().skip(1) {
        let Some(localized) = localization
            .add(&placed(
                index as i64 * SCAN_PERIOD_NS,
                *odometry,
                &room.scan(*truth),
            ))
            .expect("a scan")
        else {
            continue;
        };
        let error = position_error(localized.pose, *truth);
        assert!(
            error <= LOCALIZED_POSITION && localized.localized,
            "scan {index} localized {error} m from the truth: {localized:?}"
        );
    }
}

#[hegel::test(test_cases = 5)]
fn after_mapping_the_pose_goes_on_from_the_mappings_estimate_within_one_cell(tc: TestCase) {
    let room = room(&tc);
    let drive = loop_drive(&tc, &room);
    let odometry = drifting_odometry(&tc, &drive);
    let (_, mut localization) = map_of(&room, &drive, &odometry, sync_parameters())
        .finish(CREATED_AT_UNIX_NS, localization_parameters())
        .expect("valid parameters")
        .expect("the drive maps the room");
    // The base drives the loop again from where mapping left it, its odometry going on.
    let (last_truth, last_odometry) = (drive[drive.len() - 1], odometry[odometry.len() - 1]);
    let again = drifting_odometry(&tc, &drive);
    let resolution = sync_parameters().resolution;
    for (index, (step_truth, step_odometry)) in drive.iter().zip(&again).enumerate() {
        let truth = compose(last_truth, relative(drive[0], *step_truth));
        let odometry = compose(last_odometry, relative(again[0], *step_odometry));
        let stamp_ns = (drive.len() + index) as i64 * SCAN_PERIOD_NS;
        let Some(localized) = localization
            .add(&placed(stamp_ns, odometry, &room.scan(truth)))
            .expect("a scan")
        else {
            continue;
        };
        let error = position_error(localized.pose, truth);
        assert!(
            error <= resolution && localized.localized,
            "scan {index} after mapping localized {error} m from the truth: {localized:?}"
        );
    }
}

/// A scan that sees nothing: every bearing with no return.
fn blind(scan: &LaserScan) -> LaserScan {
    LaserScan::parse(
        scan.min_angle_rad(),
        scan.angle_increment_rad(),
        0.0,
        scan.min_range_m(),
        scan.max_range_m(),
        &vec![NO_RETURN_MARK_M; scan.ray_count()],
    )
    .expect("a scan of no returns")
}

#[test]
fn scans_that_match_nothing_leave_the_pose_not_localized_until_scans_match_again() {
    let room = Room::new(4.0, 3.0, 0.25, 2);
    let drive = circle_drive(1.2);
    let map = room_map_of(&room, &drive, &drive, sync_parameters());
    let initial = InitialPose::parse(drive[0], 0.3, 0.3).expect("valid");
    let (mut localization, _) = Localization::from_initial_pose(
        &map,
        localization_parameters(),
        &initial,
        drive[0],
        &placed(0, drive[0], &room.scan(drive[0])),
    )
    .expect("the first scan matches at the start");
    let mut answers = Vec::new();
    for (index, truth) in drive.iter().enumerate().skip(1) {
        // The scans of the second third of the drive see nothing.
        let seen = room.scan(*truth);
        let blinded = index > drive.len() / 3 && index <= 2 * drive.len() / 3;
        let scan = if blinded { blind(&seen) } else { seen };
        if let Some(localized) = localization
            .add(&placed(index as i64 * SCAN_PERIOD_NS, *truth, &scan))
            .expect("a scan")
        {
            answers.push((blinded, localized));
        }
    }
    assert!(
        answers.iter().any(|(blinded, _)| *blinded),
        "the blinded third adds scans"
    );
    let first_blinded = answers
        .iter()
        .position(|(blinded, _)| *blinded)
        .expect("a blinded answer");
    assert!(
        answers[..first_blinded]
            .iter()
            .all(|(_, localized)| localized.localized),
        "{answers:?}"
    );
    assert!(
        answers
            .iter()
            .filter(|(blinded, _)| *blinded)
            .all(|(_, localized)| !localized.localized),
        "{answers:?}"
    );
    let (blinded, last) = answers.last().expect("an answer");
    assert!(!blinded && last.localized, "{answers:?}");
}

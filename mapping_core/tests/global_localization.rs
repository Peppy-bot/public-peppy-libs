//! Global localization in the maps of drawn L-shaped rooms, so no pose sees what another
//! sees, each turned off the map's axes so its map holds every wall: driven around the room
//! pass after pass from a pose it does not know, with odometry in a frame of its own, the
//! search localizes the base near its true pose in at least [`NEAR_THE_TRUTH`] of [`ROOMS`]
//! rooms drawn from a fixed seed, and scan matching started at each such estimate keeps the
//! base localized near its true pose to the end of the drive.
//!
//! The claim counts rooms of a fixed set: where AMCL's cloud settles depends on the
//! particles each search draws at the start, and with Nav2's recovery rates of 0 no particle
//! returns to a pose that resampling left none at. At [`ALPHA`] and [`SEARCH_LIMIT`], the
//! search handed over near the true pose in each of 112 drawn turned L-shaped rooms.

use control_core::planar_pose::PlanarPose;
use control_core::positive_finite::PositiveFinite;
use mapping_core::{
    AmclParameters, Drand48, GlobalLocalization, GlobalLocalizationParameters, LocalizationLimits,
    PlacedScan, RobotModelType, SearchStep,
};
use std::ops::ControlFlow;

mod common;
use common::properties::{
    DrawSource, Stream, asymmetric_room, compose, drifting_odometry, heading_error, loop_drive,
    planar, position_error,
};
use common::{
    LOCALIZED_POSITION, SCAN_PERIOD_NS, localization_parameters, placed, room_map_of,
    sync_parameters,
};

/// How near the truth the search's localized pose lies, in position (m) and heading (rad):
/// as near as the room dataset scenarios count AMCL's pose localized.
const SEARCH_POSITION: f64 = 0.5;
const SEARCH_HEADING: f64 = 0.35;

/// The search limits as a position (m) and a heading (rad) deviation, provisional. At
/// [`ALPHA`], in 136 drawn turned L-shaped rooms, 0.45 is the smallest limit in steps of 0.05
/// at which the search handed over in every room, at scan 193 at the latest; 0.5, one step
/// more, brings the latest hand-over to scan 91, and no search in those rooms handed over away
/// from the true pose at any limit up to 0.6.
const SEARCH_LIMIT: f64 = 0.5;

/// AMCL's variance coefficients, each alpha: Nav2's, which the base's launch sets.
const ALPHA: f64 = 0.2;

/// AMCL's parameters of Nav2's nav2_params.yaml, with recovery rates of 0.
const AMCL: AmclParameters = AmclParameters {
    alpha1: ALPHA,
    alpha2: ALPHA,
    alpha3: ALPHA,
    alpha4: ALPHA,
    alpha5: ALPHA,
    laser_likelihood_max_dist: 2.0,
    laser_max_range: 100.0,
    laser_min_range: -1.0,
    max_beams: 60,
    max_particles: 2000,
    min_particles: 500,
    pf_err: 0.05,
    pf_z: 0.99,
    resample_interval: 1,
    robot_model_type: RobotModelType::Differential,
    sigma_hit: 0.2,
    update_min_a: 0.2,
    update_min_d: 0.25,
    z_hit: 0.5,
    z_rand: 0.5,
};

/// The fewest scans of the drive the search runs on: twice the latest scan at which the
/// search handed over within [`SEARCH_LIMIT`] in the rooms of its measurement (91).
const SEARCH_SCANS: usize = 182;

/// The least turn (rad) of a drawn room off each axis of the map. Karto's grid keys no cell
/// for a hit on the far x or far y edge of the scans' box (`mapping_core`'s README), and a
/// wall turned this far off that edge leaves out at most 0.2504 m of its length at its far
/// corner: half a cell (0.025 m) over sin 0.1.
const LEAST_TURN: f64 = 0.1;

/// The rooms of the fixed set, as many as the search and scan matching run through in about
/// a minute of a debug build (41 s on amd64, 50 s on arm64), and the seed of the [`Stream`]
/// they are drawn from.
const ROOMS: usize = 24;
const ROOMS_SEED: u64 = 13;

/// The rooms of the set in which the search localizes the base near its true pose: the count
/// it reached on amd64 and on arm64, all 24.
const NEAR_THE_TRUTH: usize = 24;

/// The scans from one that scan matching answers to the next: slam_toolbox's gate passes a
/// scan once the odometry moved 0.447 m from the last it passed (0.8 of the square of
/// `minimum_travel_distance`, 0.5 m), which one drive step (0.3 m) moves at most 0.306 m and
/// two at least 0.553 m, along the tightest drive (a circle of 0.5 m: twice 0.5 m by sin 0.6,
/// scaled by the odometry's error down to 0.98).
const GATE_PERIOD: usize = 2;

/// The poses of `drive`, a circle drive about the origin, driven on pass after pass, each
/// from where the one before ends, until they number at least `scans`.
fn driven(drive: &[PlanarPose], scans: usize) -> Vec<PlanarPose> {
    let first = drive[0];
    let mut poses = drive.to_vec();
    while poses.len() < scans {
        let turn = planar(0.0, 0.0, poses[poses.len() - 1].yaw() - first.yaw());
        poses.extend(drive[1..].iter().map(|&pose| compose(turn, pose)));
    }
    poses
}

/// Where the search localized the base in a room, by the scan it localized it at.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Search {
    NearTheTruth(usize),
    Elsewhere(usize),
    NotBeforeTheDriveEnded,
}

/// The search in a room drawn from `draws`; once it ends near the true pose, scan matching
/// keeps the base localized near its true pose, answering to within [`GATE_PERIOD`] scans
/// of the end of the drive.
fn search_in_a_drawn_room(draws: &impl DrawSource) -> Search {
    let room = asymmetric_room(draws)
        .turned(draws.float(LEAST_TURN, std::f64::consts::FRAC_PI_2 - LEAST_TURN));
    let drive = loop_drive(draws, &room);
    let map = room_map_of(
        &room,
        &drive,
        &drifting_odometry(draws, &drive),
        sync_parameters(),
    );
    let truths = driven(&drive, SEARCH_SCANS);
    // The odometry starts in a frame of its own.
    let frame = planar(
        draws.float(-5.0, 5.0),
        draws.float(-5.0, 5.0),
        draws.float(-3.0, 3.0),
    );
    let scans: Vec<PlacedScan> = drifting_odometry(draws, &truths)
        .into_iter()
        .zip(&truths)
        .enumerate()
        .map(|(index, (odometry, truth))| {
            placed(
                index as i64 * SCAN_PERIOD_NS,
                compose(frame, odometry),
                &room.scan(*truth),
            )
        })
        .collect();
    let search_limit = PositiveFinite::parse(SEARCH_LIMIT).expect("a positive limit");
    let parameters = GlobalLocalizationParameters {
        amcl: AMCL,
        search_limits: LocalizationLimits {
            position_std_m: search_limit,
            yaw_std_rad: search_limit,
        },
        localization: localization_parameters(),
    };
    let mut stream = Drand48::seeded(draws.integer(0, u32::MAX));
    let search = GlobalLocalization::start(&map, parameters, &mut stream).expect("a search");
    let ended = scans
        .iter()
        .enumerate()
        .try_fold(search, |search, (index, scan)| {
            match search.add(scan, &mut stream) {
                SearchStep::Searching(search, pose) => {
                    assert!(
                        pose.is_none_or(|pose| !pose.localized),
                        "scan {index}: a search goes on with a pose outside its limits: {pose:?}"
                    );
                    ControlFlow::Continue(search)
                }
                SearchStep::Found(localization, pose) => {
                    ControlFlow::Break((index, localization, pose))
                }
            }
        });
    let ControlFlow::Break((found_at, mut localization, found)) = ended else {
        return Search::NotBeforeTheDriveEnded;
    };
    assert!(found.localized, "the search ended at {found:?}");
    let truth = truths[found_at];
    if position_error(found.pose, truth) > SEARCH_POSITION
        || heading_error(found.pose, truth) > SEARCH_HEADING
    {
        return Search::Elsewhere(found_at);
    }
    let answered: Vec<usize> = scans
        .iter()
        .enumerate()
        .skip(found_at + 1)
        .filter_map(|(index, scan)| {
            let kept = localization.add(scan).expect("a scan")?;
            let error = position_error(kept.pose, truths[index]);
            assert!(
                kept.localized && error <= LOCALIZED_POSITION,
                "scan {index}: scan matching kept the base {error} m from the truth: {kept:?}"
            );
            Some(index)
        })
        .collect();
    let last_answered = *answered
        .last()
        .unwrap_or_else(|| panic!("scan matching answered no scan after scan {found_at}"));
    assert!(
        last_answered + GATE_PERIOD > scans.len() - 1,
        "scan matching answered scan {last_answered} last of {}",
        scans.len()
    );
    Search::NearTheTruth(found_at)
}

#[test]
fn in_rooms_drawn_from_a_fixed_seed_the_search_localizes_and_scan_matching_keeps_the_pose() {
    let draws = Stream::new(ROOMS_SEED);
    let searches: Vec<Search> = (0..ROOMS).map(|_| search_in_a_drawn_room(&draws)).collect();
    let near_the_truth = searches
        .iter()
        .filter(|search| matches!(search, Search::NearTheTruth(_)))
        .count();
    assert!(
        near_the_truth >= NEAR_THE_TRUTH,
        "the search localized the base near its true pose in {near_the_truth} of {ROOMS} \
         rooms: {searches:?}"
    );
}

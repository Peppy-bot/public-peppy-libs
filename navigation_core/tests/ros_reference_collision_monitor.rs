//! The stop against Nav2's collision monitor (nav2_collision_monitor 1.5.2) with one
//! footprint polygon in approach mode and one point cloud source: `Stop::decide` ports
//! `CollisionMonitor::process` and `processApproach`, `Polygon::getCollisionTime` with
//! `projectState` and `transformPoints`, and the source's stale-source stop, so for the
//! same points, read as it reads the centers of a cost grid's occupied cells, footprint,
//! velocity and clock, the source's readings stamped at their arrival, the stop slows and
//! stops where the collision monitor does, to the velocity it allows within 1e-12. Only
//! `LiveObstacles` marks a drop: on a floor that drops more than the largest step down,
//! where the collision monitor reading the depth points lets the base drive on with the
//! drop within reach, the stop on the `LiveObstacles` occupied points of the same points
//! drives the base to rest before the drop. The evidence: every case of every collision
//! monitor scenario of the ROS reference harness.

mod common;

use std::num::NonZeroUsize;

use common::finite::positive;
use common::footprints::padded;
use common::layers::layer;
use common::poses::moved;
use control_core::planar_pose::PlanarPose;
use control_core::twist::PlanarTwist;
use frame_tree::FrameName;
use frame_tree::Transform;
use navigation_core::PaddedFootprint;
use navigation_core::obstacles::{
    GridPoses, LiveObstacles, ObstacleHeights, ObstacleHeightsParams,
};
use navigation_core::stop::{Stop, StopCause, StopDecision, StopParams};
use occupancy_map::Bounds;
use ros_reference::collision_monitor::{Action, Case, Cases, PointsAre, PolygonParameters};

/// How far a velocity the stop allows may lie from the collision monitor's.
const BOUND: f64 = 1e-12;
/// The name the collision monitor gives a stop by its source.
const INVALID_SOURCE: &str = "invalid source";
/// The polygon of every scenario.
const POLYGON: &str = "FootprintApproach";
/// The side of the voxels of the `LiveObstacles` under test, in meters: a whole divisor of
/// the source's band and of the step down.
const VOXEL_M: f64 = 0.05;
/// How high above the base's center the depth sensor of a step down case stands, in
/// meters: above every point, so each ray runs down to its point.
const SENSOR_HEIGHT_M: f64 = 1.0;

/// The stop of a scenario's monitor, the monitor's settings the port holds fixed checked
/// first.
fn stop(cases: &Cases) -> Stop {
    let monitor = &cases.monitor;
    let polygon = &monitor.footprint_approach;
    let source = &monitor.depth;
    assert!(
        monitor.polygons == [POLYGON]
            && monitor.observation_sources == ["depth"]
            && monitor.enabled
            && !monitor.base_shift_correction
            && polygon.action_type == "approach"
            && polygon.enabled
            && polygon.min_points == 1
            && polygon.sources_names == ["depth"]
            && source.source_type == "pointcloud"
            && source.enabled
            && source.min_range == 0.0
            && !source.use_global_height
            && source.exclusion_zones.is_empty(),
        "{}: the scenario runs the monitor as the port does: one footprint polygon in \
         approach mode, a contact at one point, and one point cloud source, its points' \
         heights in the base's frame, with no minimum range and no exclusion zone",
        cases.name
    );
    Stop::parse(StopParams {
        time_before_collision_s: positive(polygon.time_before_collision),
        simulation_time_step_s: positive(polygon.simulation_time_step),
        source_timeout_s: positive(source.source_timeout),
    })
    .expect("the scenario's step within its time before collision")
}

/// The `LiveObstacles` under test of a scenario: its source's band and its step down, of
/// voxels `VOXEL_M` on a side, in a map around the base, the monitor's footprint and its one
/// source.
fn live_of(cases: &Cases) -> LiveObstacles {
    let source = &cases.monitor.depth;
    let heights = ObstacleHeights::parse(ObstacleHeightsParams {
        min_obstacle_height_m: source.min_height,
        max_obstacle_height_m: source.max_height,
        max_step_down_m: cases.max_step_down,
        voxel_m: positive(VOXEL_M),
    })
    .expect("the scenario's band and step, of whole voxels");
    LiveObstacles::new(
        FrameName::parse("base").expect("a frame name"),
        Bounds {
            min: [-1.0, -1.5, -0.5],
            max: [3.0, 1.5, 1.5],
        },
        heights,
        monitor_footprint(cases),
        NonZeroUsize::MIN,
    )
    .expect("a map around the base")
}

/// The monitor's polygon, the footprint the cost map publishes, padded: ours pads it by 0.
fn monitor_footprint(cases: &Cases) -> PaddedFootprint {
    padded(&cases.polygon, 0.0)
}

fn velocity(case: &Case) -> PlanarTwist {
    let [x, y, z] = case.velocity;
    PlanarTwist::new(x, y, z).expect("finite")
}

/// The scenario's stop holding the case's newest reading of the one source of `live`, which
/// arrived its age before the case's time, as each case runs on a fresh monitor.
fn stop_of_case(cases: &Cases, case: &Case, live: &LiveObstacles) -> Stop {
    let mut stop = stop(cases);
    if let Some(age_ns) = case.cloud_age_ns {
        stop.reading_arrived(live, 0, case.now_ns - age_ns)
            .expect("the scenario's one source");
    }
    stop
}

/// The collision monitor's action, polygon name and allowed velocity for `decision`.
fn as_monitor(decision: StopDecision, case: &Case) -> (Action, &'static str, [f64; 3]) {
    match decision {
        StopDecision::Continue { .. } => (Action::DoNothing, "", case.velocity),
        StopDecision::Slow { velocity, .. } => (Action::Approach, POLYGON, velocity.axes()),
        StopDecision::Stop(StopCause::NoReading { .. } | StopCause::StaleSource { .. }) => {
            (Action::Stop, INVALID_SOURCE, [0.0; 3])
        }
    }
}

/// The `LiveObstacles` of `case`'s depth points, a sensor `SENSOR_HEIGHT_M` over the base's
/// center, and their occupied points in the base's frame: the centers of its grid's lethal
/// cells and of the frame's columns.
fn grid_of_depth(cases: &Cases, case: &Case) -> (LiveObstacles, Vec<[f64; 2]>) {
    let mut live = live_of(cases);
    let sensor_on_base =
        Transform::new([0.0, 0.0, SENSOR_HEIGHT_M], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
    let points: Vec<[f32; 3]> = case
        .points
        .iter()
        .map(|p| [p[0], p[1], p[2] - SENSOR_HEIGHT_M].map(|c| c as f32))
        .collect();
    live.insert_depth_frame(
        0,
        &points,
        PlanarPose::IDENTITY,
        sensor_on_base,
        positive(5.0),
    )
    .expect("the live obstacles' one source");
    let occupied = live.occupied_seen_from(
        layer(&live).grid(),
        GridPoses {
            grid_from_map: PlanarPose::IDENTITY,
            base: PlanarPose::IDENTITY,
        },
    );
    (live, occupied)
}

fn load() -> Vec<Cases> {
    let scenarios = Cases::load_all();
    assert!(
        !scenarios.is_empty(),
        "the harness holds a collision monitor scenario"
    );
    scenarios
}

// Each case of points the stop reads as occupied cells' centers, the stop slows or stops as
// the collision monitor does, to the velocity the monitor allows within the bound.
#[test]
fn each_slow_and_stop_decision_is_the_collision_monitors() {
    for cases in load()
        .iter()
        .filter(|c| c.points_are == PointsAre::CellCenters)
    {
        let band = cases.monitor.depth.min_height..=cases.monitor.depth.max_height;
        let live = live_of(cases);
        for (index, case) in cases.cases.iter().enumerate() {
            assert!(
                case.points.iter().all(|p| band.contains(&p[2])),
                "{} case {index}: each point stands in the monitor's band",
                cases.name
            );
            let occupied: Vec<[f64; 2]> = case.points.iter().map(|p| [p[0], p[1]]).collect();
            let decision = stop_of_case(cases, case, &live).decide(
                &live,
                velocity(case),
                &occupied,
                case.now_ns,
            );
            let (action, polygon_name, allowed) = as_monitor(decision, case);
            let worst = allowed
                .iter()
                .zip(&case.allowed)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max);
            assert!(
                action == case.action && polygon_name == case.polygon_name && worst <= BOUND,
                "{} case {index}: ours {decision:?}, the monitor's {:?} by {:?} allowing {:?}",
                cases.name,
                case.action,
                case.polygon_name,
                case.allowed
            );
        }
    }
}

// On a floor that drops more than the largest step down, the collision monitor reading the
// depth points lets the base drive on at its velocity, and the `LiveObstacles` of the points
// marks a column of the drop in every case whose points lie below the step. In each such
// case whose drop lies
// within reach (a cell of the `LiveObstacles` grid of the points inside the padded
// footprint moved at the case's velocity for time_before_collision), the stop on that grid
// drives the base, the case's velocity commanded each step of the monitor's simulation time
// step, to rest, and no occupied cell's center lies inside the footprint before the step it
// rests at.
// Every other case, a step down no deeper than the limit and a drop out of reach among
// them, the stop decides as the monitor does.
#[test]
fn a_step_down_nav2_drives_over_stops_ours() {
    let mut brought_to_rest = 0;
    for cases in load().iter().filter(|c| c.points_are == PointsAre::Depth) {
        let polygon = &cases.monitor.footprint_approach;
        let footprint = monitor_footprint(cases);
        for (index, case) in cases.cases.iter().enumerate() {
            let what = format!("{} case {index}", cases.name);
            let (live, occupied) = grid_of_depth(cases, case);
            let mut stop = stop_of_case(cases, case, &live);
            let nav2_drives_on = case.action == Action::DoNothing && case.allowed == case.velocity;
            let drop_ahead = case.points.iter().any(|p| p[2] < -cases.max_step_down);
            assert!(
                !drop_ahead || !occupied.is_empty(),
                "{what}: the drop past the step marks a column"
            );
            let within_reach = projected(case.velocity, polygon)
                .any(|pose| occupied.iter().any(|&p| inside(&footprint, pose, p)));
            if nav2_drives_on && drop_ahead && within_reach {
                drive_to_rest(
                    &mut stop, &live, &footprint, &occupied, case, polygon, &what,
                );
                brought_to_rest += 1;
                continue;
            }
            let decision = stop.decide(&live, velocity(case), &occupied, case.now_ns);
            assert_eq!(
                as_monitor(decision, case).0,
                case.action,
                "{what}: out of the drop's reach, ours acts as the monitor"
            );
        }
    }
    assert!(
        brought_to_rest > 0,
        "a scenario holds a step down within reach that the collision monitor drives over"
    );
}

/// How many steps of the simulation time step a drive to rest may take.
const REST_STEPS: usize = 5000;

/// Drives a base from the case's pose by the velocities of `stop`, deciding with `live`, the
/// case's velocity commanded each step, `occupied` fixed in the case's base frame; panics
/// unless it comes to rest within `REST_STEPS` with no occupied point inside its footprint
/// before then.
fn drive_to_rest(
    stop: &mut Stop,
    live: &LiveObstacles,
    footprint: &PaddedFootprint,
    occupied: &[[f64; 2]],
    case: &Case,
    polygon: &PolygonParameters,
    what: &str,
) {
    let mut pose = PlanarPose::IDENTITY;
    for step in 0..REST_STEPS {
        let seen: Vec<[f64; 2]> = occupied
            .iter()
            .map(|&point| pose.apply_inverse(point))
            .collect();
        let allowed = stop
            .decide(live, velocity(case), &seen, case.now_ns)
            .velocity();
        if allowed == PlanarTwist::ZERO {
            return;
        }
        assert!(
            !seen
                .iter()
                .any(|&p| inside(footprint, PlanarPose::IDENTITY, p)),
            "{what}: at step {step} the footprint at {pose:?} holds an occupied cell and the \
             base still moves at {allowed:?}"
        );
        pose = moved(pose, allowed.axes(), polygon.simulation_time_step);
    }
    panic!("{what}: the base still moves after {REST_STEPS} steps, at {pose:?}");
}

/// The poses of the base at `velocity` from its pose, every simulation time step up to
/// time_before_collision.
fn projected(velocity: [f64; 3], polygon: &PolygonParameters) -> impl Iterator<Item = PlanarPose> {
    let steps = (polygon.time_before_collision / polygon.simulation_time_step).ceil() as usize;
    let dt = polygon.simulation_time_step;
    std::iter::successors(Some(PlanarPose::IDENTITY), move |&pose| {
        Some(moved(pose, velocity, dt))
    })
    .take(steps + 1)
}

/// Whether `point`, in the frame `pose` lies in, falls inside `footprint` placed at `pose`:
/// the even-odd rule.
fn inside(footprint: &PaddedFootprint, pose: PlanarPose, point: [f64; 2]) -> bool {
    let outline = footprint.placed_at(pose);
    let [px, py] = point;
    outline
        .iter()
        .zip(outline.iter().cycle().skip(1))
        .filter(|&(a, b)| (a[1] > py) != (b[1] > py))
        .filter(|&(a, b)| px < a[0] + (py - a[1]) * (b[0] - a[0]) / (b[1] - a[1]))
        .count()
        % 2
        == 1
}

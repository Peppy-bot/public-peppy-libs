//! The generators the property tests draw from: the parameters of a projection and clouds
//! of points about it, from a Hegel test case; rooms, drives around them with drifting
//! odometry, from a test case or a fixed [`Stream`]; and the scans a laser reads in them.

use camera_core::laser_scan::{LaserScan, NO_RETURN_MARK_M, Sweep};
use chain_kinematics::Draws;
use control_core::planar_pose::PlanarPose;
use hegel::TestCase;
use hegel::generators as gs;
use mapping_core::LaserProjectionParameters;
use std::cell::RefCell;

/// Where the room, drive and odometry generators take their draws from: a Hegel test case,
/// or a [`Stream`] that draws the same values on every run.
pub trait DrawSource {
    /// A value in `min..=max`.
    fn float(&self, min: f64, max: f64) -> f64;
    /// An integer in `min..=max`.
    fn integer(&self, min: u32, max: u32) -> u32;
}

impl DrawSource for TestCase {
    fn float(&self, min: f64, max: f64) -> f64 {
        self.draw(gs::floats().min_value(min).max_value(max))
    }

    fn integer(&self, min: u32, max: u32) -> u32 {
        self.draw(gs::integers().min_value(min).max_value(max))
    }
}

/// chain_kinematics' SplitMix64 draws of a seed: the same draws on every run and every
/// machine.
pub struct Stream(RefCell<Draws>);

impl Stream {
    pub fn new(seed: u64) -> Self {
        Self(RefCell::new(Draws::new(seed)))
    }
}

impl DrawSource for Stream {
    fn float(&self, min: f64, max: f64) -> f64 {
        self.0.borrow_mut().between(min, max).min(max)
    }

    fn integer(&self, min: u32, max: u32) -> u32 {
        let span = u64::from(max - min) + 1;
        min + (self.0.borrow_mut().next_bits() % span) as u32
    }
}

/// The pose on the floor at `x`, `y` (m) and `yaw` (rad).
pub fn planar(x: f64, y: f64, yaw: f64) -> PlanarPose {
    PlanarPose::new(x, y, yaw).expect("a finite pose")
}

/// The parameters of a projection whose fan holds 1 to 400 bins within (-pi, pi),
/// `angle_max` half an increment into the last one, with a height band and range limits
/// about the scan.
pub fn projection_parameters(tc: &TestCase) -> LaserProjectionParameters {
    let angle_increment: f64 = tc.draw(gs::floats().min_value(0.001).max_value(0.05));
    let most_bins = ((6.2 / angle_increment) as u32).min(400);
    let bins: u32 = tc.draw(gs::integers().min_value(1).max_value(most_bins));
    let span = f64::from(bins) * angle_increment;
    let angle_min: f64 = tc.draw(gs::floats().min_value(-3.1).max_value(3.1 - span));
    let min_height: f64 = tc.draw(gs::floats().min_value(-0.5).max_value(0.5));
    let band: f64 = tc.draw(gs::floats().min_value(0.1).max_value(2.0));
    let range_min: f64 = tc.draw(gs::floats().min_value(0.0).max_value(1.0));
    let range_span: f64 = tc.draw(gs::floats().min_value(0.5).max_value(10.0));
    LaserProjectionParameters {
        min_height,
        max_height: min_height + band,
        angle_min,
        angle_max: angle_min + (f64::from(bins) - 0.5) * angle_increment,
        angle_increment,
        range_min,
        range_max: range_min + range_span,
    }
}

/// Up to `max_points` points drawn in a box about the scan that holds every height band
/// and range limit of [`projection_parameters`] and points outside them.
pub fn cloud(tc: &TestCase, max_points: usize) -> Vec<[f32; 3]> {
    let coordinate = || gs::floats::<f32>().min_value(-12.0).max_value(12.0);
    let height = gs::floats::<f32>().min_value(-1.0).max_value(3.0);
    let points: Vec<([f32; 2], f32)> = tc.draw(
        gs::vecs(gs::tuples2(gs::arrays::<_, _, 2>(coordinate()), height)).max_size(max_points),
    );
    points.into_iter().map(|([x, y], z)| [x, y, z]).collect()
}

/// A drawn room about the origin, each wall a segment: a rectangle with a box standing in
/// one corner, or an L.
#[derive(Debug, Clone)]
pub struct Room {
    pub half_width: f64,
    pub half_height: f64,
    pub walls: Vec<[[f64; 2]; 2]>,
}

/// The farthest a drawn scan reads (m).
pub const SCAN_RANGE_MAX: f64 = 12.0;
/// The bearings of a drawn scan, all around the base.
pub const SCAN_BEAMS: usize = 360;

/// How far the box of a drawn room stands from the walls of its corner (m).
const BOX_CLEARANCE: f64 = 0.4;
/// How far a drawn drive keeps from the walls (m).
const DRIVE_CLEARANCE: f64 = 1.2;

/// The walls of the polygon through `corners`, each to the next and the last to the first.
fn polygon<const N: usize>(corners: [[f64; 2]; N]) -> impl Iterator<Item = [[f64; 2]; 2]> {
    (0..N).map(move |i| [corners[i], corners[(i + 1) % N]])
}

/// The four walls of a rectangle about `center`, `half_x` by `half_y` (m).
fn rectangle(center: [f64; 2], half_x: f64, half_y: f64) -> impl Iterator<Item = [[f64; 2]; 2]> {
    let [cx, cy] = center;
    polygon([
        [cx - half_x, cy - half_y],
        [cx + half_x, cy - half_y],
        [cx + half_x, cy + half_y],
        [cx - half_x, cy + half_y],
    ])
}

/// The signs of x and y in the quadrant `quadrant`, from 0 to 3: bit 0 takes x negative,
/// bit 1 y.
fn signs_of(quadrant: u8) -> [f64; 2] {
    let sign = |bit: u8| if quadrant & bit == 0 { 1.0 } else { -1.0 };
    [sign(1), sign(2)]
}

/// An L-shaped room 6 to 8 m wide and 3.4 to 5 m deep, its block over a drawn quadrant, so
/// no turn of the room about its center, and no reflection, takes its walls to themselves.
pub fn asymmetric_room(draws: &impl DrawSource) -> Room {
    let half_width = draws.float(3.0, 4.0);
    let half_height = draws.float(1.7, 2.5);
    let quadrant = draws.integer(0, 3) as u8;
    Room::l_shaped(half_width, half_height, quadrant)
}

/// A room 4 to 8 m wide and 3.4 to 6 m deep, with a box of 0.3 to 0.6 m a side in a
/// drawn corner.
pub fn room(draws: &impl DrawSource) -> Room {
    let half_width = draws.float(2.0, 4.0);
    let half_height = draws.float(1.7, 3.0);
    let half_side = draws.float(0.15, 0.3);
    let corner = draws.integer(0, 3) as u8;
    Room::new(half_width, half_height, half_side, corner)
}

impl Room {
    /// A rectangle about the origin, `half_width` by `half_height` (m), with a box of
    /// `half_side` (m) in the corner `corner`, from 0 to 3, [`BOX_CLEARANCE`] from its walls.
    pub fn new(half_width: f64, half_height: f64, half_side: f64, corner: u8) -> Self {
        let [sign_x, sign_y] = signs_of(corner);
        let center = [
            sign_x * (half_width - BOX_CLEARANCE - half_side),
            sign_y * (half_height - BOX_CLEARANCE - half_side),
        ];
        let walls = rectangle([0.0, 0.0], half_width, half_height)
            .chain(rectangle(center, half_side, half_side))
            .collect();
        Self {
            half_width,
            half_height,
            walls,
        }
    }

    /// A rectangle about the origin, `half_width` by `half_height` (m), less a block over the
    /// quadrant `quadrant`, from 0 to 3, from the rectangle's corner to the point of its long
    /// axis `half_height` from the center. Every point of the block lies `half_height` or
    /// more from the center, so a [`loop_drive`] keeps [`DRIVE_CLEARANCE`] from it.
    pub fn l_shaped(half_width: f64, half_height: f64, quadrant: u8) -> Self {
        assert!(
            half_width > half_height,
            "an L-shaped room is wider than deep, not {half_width} m by {half_height} m"
        );
        let [sign_x, sign_y] = signs_of(quadrant);
        let corners = [
            [-half_width, -half_height],
            [half_width, -half_height],
            [half_width, 0.0],
            [half_height, 0.0],
            [half_height, half_height],
            [-half_width, half_height],
        ]
        .map(|[x, y]| [sign_x * x, sign_y * y]);
        Self {
            half_width,
            half_height,
            walls: polygon(corners).collect(),
        }
    }

    /// This room turned by `angle` (rad) about the origin.
    pub fn turned(self, angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();
        let turn = |[x, y]: [f64; 2]| [cos * x - sin * y, sin * x + cos * y];
        Self {
            walls: self
                .walls
                .iter()
                .map(|&[a, b]| [turn(a), turn(b)])
                .collect(),
            ..self
        }
    }

    /// The distance from `origin` along `angle` to the nearest wall, or lidar_2d's no-return
    /// mark for no wall nearer than the scan's farthest range.
    pub fn range(&self, origin: [f64; 2], angle: f64) -> f32 {
        let direction = [angle.cos(), angle.sin()];
        let nearest = self
            .walls
            .iter()
            .filter_map(|&[a, b]| {
                let edge = [b[0] - a[0], b[1] - a[1]];
                let denominator = direction[0] * edge[1] - direction[1] * edge[0];
                if denominator.abs() < 1e-12 {
                    return None;
                }
                let offset = [a[0] - origin[0], a[1] - origin[1]];
                let t = (offset[0] * edge[1] - offset[1] * edge[0]) / denominator;
                let u = (offset[0] * direction[1] - offset[1] * direction[0]) / denominator;
                (t > 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
            })
            .fold(f64::INFINITY, f64::min);
        if nearest < SCAN_RANGE_MAX {
            nearest as f32
        } else {
            NO_RETURN_MARK_M
        }
    }

    /// The scan of a laser all around a base at `pose`, along the bearings that karto
    /// reads it at.
    pub fn scan(&self, pose: PlanarPose) -> LaserScan {
        let increment = (std::f64::consts::TAU / SCAN_BEAMS as f64) as f32;
        let angle_min = -std::f32::consts::PI;
        let ranges: Vec<f32> = (0..SCAN_BEAMS)
            .map(|beam| {
                let bearing = f64::from(angle_min) + beam as f64 * f64::from(increment);
                self.range([pose.x(), pose.y()], pose.yaw() + bearing)
            })
            .collect();
        LaserScan::parse(
            angle_min,
            increment,
            0.0,
            0.05,
            SCAN_RANGE_MAX as f32,
            &ranges,
        )
        .expect("a drawn scan")
    }
}

/// How far the base moves between two scans of a drawn drive (m).
pub const DRIVE_STEP: f64 = 0.3;

/// A drive counterclockwise around a circle about the room's center, of a radius that
/// keeps it [`DRIVE_CLEARANCE`] from the walls and from an L's block, and clear of a box in
/// a corner.
pub fn loop_drive(draws: &impl DrawSource, room: &Room) -> Vec<PlanarPose> {
    let largest = room.half_width.min(room.half_height) - DRIVE_CLEARANCE;
    circle_drive(draws.float(0.5, largest.max(0.5)))
}

/// A drive counterclockwise around a circle of `radius` (m) about the origin, a scan every
/// [`DRIVE_STEP`], once around and a quarter turn on, so its last scans revisit its first.
pub fn circle_drive(radius: f64) -> Vec<PlanarPose> {
    let steps = (1.25 * std::f64::consts::TAU * radius / DRIVE_STEP).ceil() as usize;
    (0..=steps)
        .map(|step| {
            let angle = step as f64 * DRIVE_STEP / radius;
            planar(
                radius * angle.cos(),
                radius * angle.sin(),
                angle + std::f64::consts::FRAC_PI_2,
            )
        })
        .collect()
}

/// The largest error of a drawn odometry's scale of a move.
const ODOMETRY_SCALE_ERROR: f64 = 0.02;
/// The largest drift (rad) of a drawn odometry's heading a step.
const ODOMETRY_DRIFT: f64 = 0.01;

/// The odometry of a drive: from its first pose, each motion between two poses scaled by
/// a drawn factor within [`ODOMETRY_SCALE_ERROR`] of 1 and turned by a drawn drift up to
/// [`ODOMETRY_DRIFT`] a step.
pub fn drifting_odometry(draws: &impl DrawSource, drive: &[PlanarPose]) -> Vec<PlanarPose> {
    let scale = draws.float(1.0 - ODOMETRY_SCALE_ERROR, 1.0 + ODOMETRY_SCALE_ERROR);
    let drift = draws.float(-ODOMETRY_DRIFT, ODOMETRY_DRIFT);
    let Some(&first) = drive.first() else {
        return Vec::new();
    };
    let mut odometry = vec![first];
    for pair in drive.windows(2) {
        let motion = relative(pair[0], pair[1]);
        let last = odometry[odometry.len() - 1];
        odometry.push(compose(
            last,
            planar(motion.x() * scale, motion.y() * scale, motion.yaw() + drift),
        ));
    }
    odometry
}

/// The distance of `pose` from `truth` (m).
pub fn position_error(pose: PlanarPose, truth: PlanarPose) -> f64 {
    (pose.x() - truth.x()).hypot(pose.y() - truth.y())
}

/// The heading error of `pose` from `truth` (rad).
pub fn heading_error(pose: PlanarPose, truth: PlanarPose) -> f64 {
    let turn = pose.yaw() - truth.yaw();
    turn.sin().atan2(turn.cos()).abs()
}

/// The pose `b` in the frame of `a`, as control_core's `PlanarPose::seen_from` gives it.
pub fn relative(a: PlanarPose, b: PlanarPose) -> PlanarPose {
    b.seen_from(a).expect("a pose near the frame")
}

/// The pose `b` of the frame of `a`, in the frame `a` lies in: its position placed by
/// control_core's `PlanarPose::apply`, and the headings added.
pub fn compose(a: PlanarPose, b: PlanarPose) -> PlanarPose {
    let [x, y] = a.apply([b.x(), b.y()]);
    planar(x, y, a.yaw() + b.yaw())
}

/// The largest distance between the points of `scan` placed at `estimate` and at
/// `truth` (m).
pub fn largest_point_displacement(
    scan: &LaserScan,
    estimate: PlanarPose,
    truth: PlanarPose,
) -> f64 {
    let still = Sweep {
        first_ray: PlanarPose::IDENTITY.to_transform(),
        last_ray: PlanarPose::IDENTITY.to_transform(),
    };
    scan.placed_hits(&still)
        .into_iter()
        .map(|hit| {
            let [x, y, _] = hit.in_sensor;
            let local = planar(x, y, 0.0);
            let (a, b) = (compose(estimate, local), compose(truth, local));
            (a.x() - b.x()).hypot(a.y() - b.y())
        })
        .fold(0.0, f64::max)
}

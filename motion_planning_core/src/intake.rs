//! A depth frame's intake into the voxel map: its samples deprojected into points in the
//! robot frame, the robot and the objects of the planning scene cut out, the robot and
//! the objects its links carry where a history of its joint positions puts them at the
//! frame's stamp, and the points inserted. A point on the robot or inside an object marks
//! nothing, and its ray clears the space before it.

use std::collections::VecDeque;
use std::time::{Duration, SystemTime};

use camera_core::pinhole::{DepthModel, Pinhole};
use camera_core::pose::Pose;
use collision_geometry::chain::{
    BodyGeometry, BodyMask, ChainGeometry, Mask, Padding, PlacedMask, Rider, RiderBody,
};
use collision_geometry::shapes::PlacedShape;
use nalgebra::Isometry3;
use occupancy_map::{
    Cutout, DepthError, InsertError, RobotCutout, Scan, ShapeCutout, VoxelMap, depth_points,
};

use crate::arm::rider_pose;

/// The depth stream of a camera the robot carries, as the map reads its frames.
pub struct DepthCamera {
    /// The depth optical frame in the robot frame.
    pub sensor: Pose,
    lens: Pinhole,
    /// How a sample reads as a depth.
    depth_model: DepthModel,
    /// Meters per depth count.
    depth_unit: f32,
}

impl DepthCamera {
    /// The camera whose color optical frame stands at `color_in_robot` in the robot
    /// frame, with its depth optical frame at `depth_in_color` in the color one.
    pub fn new(
        color_in_robot: &Pose,
        depth_in_color: &Pose,
        lens: Pinhole,
        depth_model: DepthModel,
        depth_unit: f32,
    ) -> Self {
        Self {
            sensor: color_in_robot.compose(depth_in_color),
            lens,
            depth_model,
            depth_unit,
        }
    }

    /// The points the samples of `frame`, a z16 depth frame of the lens's size, read in
    /// the depth optical frame; a sample of 0 makes no point.
    pub fn points(&self, frame: &[u8]) -> Result<Vec<[f32; 3]>, DepthError> {
        depth_points(frame, &self.lens, self.depth_unit, self.depth_model)
    }
}

/// The positions a robot measured over the last `span`, `N` values each, read at any
/// instant the history spans: the map cuts the robot out of a depth frame where it stood
/// when the camera took the frame, which may be some tens of milliseconds before the frame
/// arrives.
pub struct JointHistory<const N: usize> {
    span: Duration,
    samples: VecDeque<(SystemTime, [f64; N])>,
}

impl<const N: usize> JointHistory<N> {
    /// A history that keeps the samples of the last `span`: longer than the camera's
    /// delay, so a frame's stamp lies inside it.
    pub fn new(span: Duration) -> Self {
        Self {
            span,
            samples: VecDeque::new(),
        }
    }

    /// Adds the positions measured at `at`. A sample at the newest's stamp is dropped. A
    /// sample before it means the clock stepped back: the history restarts at it.
    pub fn push(&mut self, at: SystemTime, positions: [f64; N]) {
        match self.samples.back() {
            Some((newest, _)) if at == *newest => return,
            Some((newest, _)) if at < *newest => self.samples.clear(),
            _ => {}
        }
        self.samples.push_back((at, positions));
        while self
            .samples
            .front()
            .and_then(|(oldest, _)| at.duration_since(*oldest).ok())
            .is_some_and(|age| age > self.span)
        {
            self.samples.pop_front();
        }
    }

    /// The newest positions.
    pub fn latest(&self) -> Option<&[f64; N]> {
        self.samples.back().map(|(_, positions)| positions)
    }

    /// The instant the newest positions were measured at.
    pub fn latest_at(&self) -> Option<SystemTime> {
        self.samples.back().map(|(at, _)| *at)
    }

    /// The positions at `at`, interpolated between the samples around it; `None` when
    /// `at` lies outside the history.
    pub fn at(&self, at: SystemTime) -> Option<[f64; N]> {
        let after = self.samples.iter().position(|(t, _)| *t >= at)?;
        let (t1, q1) = &self.samples[after];
        if *t1 == at {
            return Some(*q1);
        }
        let (t0, q0) = &self.samples[after.checked_sub(1)?];
        let span = t1.duration_since(*t0).ok()?.as_secs_f64();
        let s = at.duration_since(*t0).ok()?.as_secs_f64() / span;
        Some(std::array::from_fn(|i| q0[i] + (q1[i] - q0[i]) * s))
    }
}

/// A chain of the robot at the joint positions a depth frame shows it at, with the bodies
/// its links carry: parts such as a gripper's fingers at the gripper's opening, and the
/// shapes of the objects attached to its grasp link.
pub struct PosedChain<'a, const N: usize> {
    pub geometry: &'a ChainGeometry<N>,
    pub joints: [f64; N],
    pub riders: &'a [Rider<'a>],
}

/// The robot grown by a padding at one instant, with the objects of the planning scene,
/// the shapes a depth frame cuts out: each chain's links at its joint positions, the
/// bodies they carry, the body fixed around the chains, and the objects no link carries
/// where they stand. Each piece grows as [`Padding`] says: scaled about its center, a
/// mesh's or a region's piece about the mean of its vertices, and taking in every point
/// within the offset of the scaled piece; and each shape of an object by the same rule
/// (`PlacedShape::contains`).
pub struct RobotMask<'a, const N: usize> {
    chains: Vec<(Mask<'a, N>, [f64; N])>,
    carried: BodyMask<'a>,
    body: BodyMask<'a>,
    /// The shapes of the objects, placed in the frame of the URDF root: those the chains
    /// carry, and those no link carries.
    objects: Vec<PlacedShape<'a>>,
    padding: Padding,
}

impl<'a, const N: usize> RobotMask<'a, N> {
    /// The robot of `chains` and `body`, each piece and each shape its links carry grown
    /// by `padding`.
    pub fn new(chains: &[PosedChain<'a, N>], body: &'a BodyGeometry, padding: Padding) -> Self {
        let riders: Vec<(&Rider<'a>, Isometry3<f64>)> = chains
            .iter()
            .flat_map(|chain| {
                let posed = chain.geometry.chain().at(&chain.joints);
                chain
                    .riders
                    .iter()
                    .map(move |rider| (rider, rider_pose(&posed, rider)))
            })
            .collect();
        let parts = riders.iter().filter_map(|(rider, pose)| match rider.body {
            RiderBody::Part(part) => Some((rider.name, part.pieces(), *pose)),
            RiderBody::Shape(_) => None,
        });
        let objects = riders.iter().filter_map(|(rider, pose)| match rider.body {
            RiderBody::Part(_) => None,
            RiderBody::Shape(shape) => Some(PlacedShape::new(shape, *pose)),
        });
        Self {
            chains: chains
                .iter()
                .map(|chain| (chain.geometry.mask(padding), chain.joints))
                .collect(),
            carried: BodyMask::of(parts, padding),
            body: body.mask(padding),
            objects: objects.collect(),
            padding,
        }
    }

    /// The mask with `standing`, the shapes of the objects no link carries placed in the
    /// frame of the URDF root, cut out too, each grown by the robot's padding.
    pub fn with_objects(mut self, standing: impl IntoIterator<Item = PlacedShape<'a>>) -> Self {
        self.objects.extend(standing);
        self
    }

    /// Each grown part where it stands, in the frame of the URDF root.
    pub fn placed(&self) -> Vec<PlacedMask<'_>> {
        self.chains
            .iter()
            .map(|(mask, joints)| mask.at(joints))
            .chain([self.carried.placed(), self.body.placed()])
            .collect()
    }
}

/// Inserts `scan` into `map` with `robot` and its objects cut out, the robot's URDF root
/// at the origin of the scan's frame. A scan in another frame than the map's is refused.
pub fn insert_cut_out<const N: usize>(
    map: &mut VoxelMap,
    scan: &Scan,
    robot: &RobotMask<N>,
) -> Result<(), InsertError> {
    let root = Pose::new([0.0; 3], [0.0, 0.0, 0.0, 1.0]).expect("the identity is a pose");
    let body = RobotCutout::new(scan.frame, root, robot.placed());
    let objects: Vec<ShapeCutout> = robot
        .objects
        .iter()
        .map(|shape| ShapeCutout {
            frame: scan.frame,
            shape: *shape,
            padding: robot.padding,
        })
        .collect();
    let cutouts: Vec<&dyn Cutout> = std::iter::once(&body as &dyn Cutout)
        .chain(objects.iter().map(|object| object as &dyn Cutout))
        .collect();
    map.insert(scan, &cutouts)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn after(ms: u64) -> SystemTime {
        SystemTime::UNIX_EPOCH + Duration::from_millis(ms)
    }

    fn pose(position: [f64; 3], orientation: [f64; 4]) -> Pose {
        Pose::new(position, orientation).expect("a position and a unit quaternion")
    }

    // A depth frame 6 cm left of and 2 cm below a color frame that looks straight down
    // from 1 m up, the color frame's image down along the robot's -y, reads a point 1 m
    // ahead of it on the floor, 6 cm along -x and 2 cm along -y.
    #[test]
    fn the_depth_frame_stands_where_its_color_frame_and_offset_put_it() {
        let lens = Pinhole::parse(&camera_core::pinhole::Intrinsics {
            width: 1,
            height: 1,
            fx: 1.0,
            fy: 1.0,
            cx: 0.5,
            cy: 0.5,
            distortion_model: "none",
            distortion: &[],
        })
        .expect("a pinhole model");
        let down = pose([0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0]);
        let left = pose([-0.06, 0.02, 0.0], [0.0, 0.0, 0.0, 1.0]);
        let camera = DepthCamera::new(&down, &left, lens, DepthModel::Z, 0.001);
        let point = camera.sensor.apply([0.0, 0.0, 1.0]);
        for (got, want) in point.iter().zip([-0.06, -0.02, 0.0]) {
            assert!((got - want).abs() < 1e-12, "{point:?}");
        }
    }

    // A stamp between two samples reads their interpolation, weighted by its nearness to
    // each, a stamp on a sample reads it, and one outside the history reads nothing.
    #[test]
    fn the_history_interpolates_inside_its_span() {
        let mut history = JointHistory::new(Duration::from_millis(500));
        history.push(after(1000), [0.0, 2.0]);
        history.push(after(1050), [1.0, -2.0]);
        assert_eq!(history.at(after(1025)), Some([0.5, 0.0]));
        let near_the_first = history.at(after(1010)).expect("inside the history");
        for (got, want) in near_the_first.iter().zip([0.2, 1.2]) {
            assert!((got - want).abs() < 1e-12, "{near_the_first:?}");
        }
        assert_eq!(history.at(after(1050)), Some([1.0, -2.0]));
        assert_eq!(history.at(after(999)), None);
        assert_eq!(history.at(after(1051)), None);
    }

    // The history keeps its span and holds the newest sample.
    #[test]
    fn the_history_drops_what_falls_out_of_its_span() {
        let mut history = JointHistory::new(Duration::from_millis(100));
        for k in 0..10 {
            history.push(after(1000 + 50 * k), [k as f64]);
        }
        assert_eq!(history.latest(), Some(&[9.0]));
        assert_eq!(history.latest_at(), Some(after(1450)));
        assert_eq!(history.at(after(1300)), None);
        assert_eq!(history.at(after(1375)), Some([7.5]));
    }

    // A second sample at the newest's stamp is dropped: the history keeps the first.
    #[test]
    fn a_sample_at_the_newest_stamp_is_dropped() {
        let mut history = JointHistory::new(Duration::from_millis(500));
        history.push(after(1000), [0.0]);
        history.push(after(1050), [1.0]);
        history.push(after(1050), [99.0]);
        assert_eq!(history.latest(), Some(&[1.0]));
        assert_eq!(history.at(after(1025)), Some([0.5]));
    }

    // A sample before the newest means the clock stepped back: the history restarts at
    // it, reads it at its stamp and nothing at the stamps it held before, and runs on
    // from it.
    #[test]
    fn a_clock_that_steps_back_restarts_the_history() {
        let mut history = JointHistory::new(Duration::from_millis(500));
        history.push(after(1000), [0.0]);
        history.push(after(1050), [1.0]);
        history.push(after(200), [5.0]);
        assert_eq!(history.latest(), Some(&[5.0]));
        assert_eq!(history.latest_at(), Some(after(200)));
        assert_eq!(history.at(after(200)), Some([5.0]));
        assert_eq!(history.at(after(1025)), None);
        history.push(after(250), [6.0]);
        assert_eq!(history.at(after(225)), Some([5.5]));
    }
}

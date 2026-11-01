//! The voxel map: a bounded grid of OctoMap log-odds, a voxel no ray has
//! reached unknown, and a scan inserted by the rules of MoveIt's point cloud
//! updater (moveit2 2.15.2 `PointCloudOctomapUpdater::cloudMsgCallback`).

use std::collections::HashSet;

use camera_core::pose::Pose;
use collision_geometry::chain::PlacedMask;
use control_core::positive_finite::PositiveFinite;
use nalgebra::Point3;

use crate::frame::Frame;
use crate::grid::Grid;
use crate::key::{Key, TREE_VOXELS, VoxelSize};
use crate::ray::ray_keys;
use crate::sensor_model::{SensorModel, occupied};

/// The most voxels a map holds, a gibibyte of log-odds: a map that fits in
/// memory.
const MAX_VOXELS: usize = 1 << 28;

/// The region a map holds, in its frame: the voxels from the one holding
/// `min` to the one holding `max`, both included.
#[derive(Clone, Copy)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Why a map cannot be built.
#[derive(Debug, thiserror::Error)]
pub enum MapError {
    #[error(
        "the bounds' corner {corner:?} lies outside the voxel tree, which holds {TREE_VOXELS} \
         voxels from the origin on each side of each axis, {reach_m} m at {voxel_m} m voxels: \
         give bounds within {reach_m} m of the origin or use larger voxels"
    )]
    OutsideTree {
        corner: [f64; 3],
        voxel_m: f64,
        reach_m: f64,
    },
    #[error(
        "the bounds' minimum {min:?} lies above their maximum {max:?} on an axis: give the \
         lower corner as the minimum"
    )]
    Inverted { min: [f64; 3], max: [f64; 3] },
    #[error(
        "the bounds hold {voxels} voxels, more than the {MAX_VOXELS} a map holds: use larger \
         voxels or smaller bounds"
    )]
    TooLarge { voxels: u128 },
}

/// Why a scan does not go into a map: it, or a shape it cuts out, stands in
/// another frame than the map's.
#[derive(Debug, thiserror::Error, PartialEq)]
pub enum InsertError {
    #[error(
        "the scan's sensor stands in frame \"{scan}\" and the map is in \"{map}\": give the \
         sensor's pose in \"{map}\""
    )]
    ScanFrame { scan: Frame, map: Frame },
    #[error(
        "a cutout stands in frame \"{cutout}\" and the map is in \"{map}\": place the \
         cutout in \"{map}\""
    )]
    CutoutFrame { cutout: Frame, map: Frame },
}

/// A shape a scan cuts out of the map: a point inside it is the robot seeing
/// itself.
pub trait Cutout {
    /// The frame the shape stands in.
    fn frame(&self) -> &Frame;

    /// Whether `point`, in the shape's frame, lies inside the shape.
    fn contains(&self, point: [f64; 3]) -> bool;
}

/// The robot's body as a cutout: its chains, each grown by a padding and
/// placed at the frame's joint positions by `collision_geometry` in the frame
/// of the URDF root, and where that root stands in the cutout's frame.
pub struct RobotCutout<'a> {
    frame: &'a Frame,
    /// The cutout's frame placed in the URDF root's: the root's pose,
    /// inverted.
    to_root: Pose,
    chains: Vec<PlacedMask<'a>>,
}

impl<'a> RobotCutout<'a> {
    /// The robot of placed chains `chains`, whose URDF root stands at `root`
    /// in `frame`.
    pub fn new(frame: &'a Frame, root: Pose, chains: Vec<PlacedMask<'a>>) -> Self {
        Self {
            frame,
            to_root: root.inverse(),
            chains,
        }
    }
}

impl Cutout for RobotCutout<'_> {
    fn frame(&self) -> &Frame {
        self.frame
    }

    fn contains(&self, point: [f64; 3]) -> bool {
        let in_root = Point3::from(self.to_root.apply(point));
        self.chains.iter().any(|chain| chain.contains(&in_root))
    }
}

/// One reading of a sensor: its points in its own frame, in single precision
/// as a point cloud carries them, the sensor's pose in `frame`, and how far
/// it reads.
pub struct Scan<'a> {
    pub frame: &'a Frame,
    pub sensor: Pose,
    pub points: &'a [[f32; 3]],
    pub max_range_m: PositiveFinite,
}

/// What one voxel of the map holds.
#[derive(Debug, PartialEq)]
pub enum Voxel {
    /// No ray has reached it: nothing to collide with.
    Unknown,
    /// Its log-odds, below even odds.
    Free(f32),
    /// Its log-odds, at or above even odds.
    Occupied(f32),
}

/// A voxel map of what a sensor sees.
#[derive(Debug)]
pub struct VoxelMap {
    frame: Frame,
    grid: Grid,
    /// Each voxel's log-odds, in the grid's order; NaN where no ray has
    /// reached it.
    log_odds: Vec<f32>,
    model: SensorModel,
}

impl VoxelMap {
    /// An empty map in `frame` of voxels of `size` over `bounds`.
    pub fn new(frame: Frame, size: PositiveFinite, bounds: Bounds) -> Result<Self, MapError> {
        let size = VoxelSize::new(size);
        let in_tree = |corner: [f64; 3]| {
            size.key(corner).ok_or(MapError::OutsideTree {
                corner,
                voxel_m: size.get(),
                reach_m: size.tree_reach_m(),
            })
        };
        let (lo, hi) = (in_tree(bounds.min)?, in_tree(bounds.max)?);
        if (0..3).any(|i| hi.0[i] < lo.0[i]) {
            return Err(MapError::Inverted {
                min: bounds.min,
                max: bounds.max,
            });
        }
        let dims: [usize; 3] = std::array::from_fn(|i| (hi.0[i] - lo.0[i]) as usize + 1);
        let voxels = dims.iter().map(|&d| d as u128).product::<u128>();
        if voxels > MAX_VOXELS as u128 {
            return Err(MapError::TooLarge { voxels });
        }
        Ok(Self {
            frame,
            grid: Grid { size, lo, dims },
            log_odds: vec![f32::NAN; voxels as usize],
            model: SensorModel::octomap(),
        })
    }

    /// The voxel holding `point`, when the map holds it.
    pub fn key(&self, point: [f64; 3]) -> Option<Key> {
        self.grid.key(point)
    }

    /// The center of voxel `key`.
    pub fn center(&self, key: Key) -> [f64; 3] {
        self.grid.center(key)
    }

    /// The grid the map holds.
    pub(crate) fn grid(&self) -> Grid {
        self.grid
    }

    /// What voxel `key` holds; unknown outside the map.
    pub fn voxel(&self, key: Key) -> Voxel {
        match self.grid.index(key).map(|index| self.log_odds[index]) {
            Some(value) if value.is_nan() => Voxel::Unknown,
            None => Voxel::Unknown,
            Some(value) if occupied(value) => Voxel::Occupied(value),
            Some(value) => Voxel::Free(value),
        }
    }

    /// Whether voxel `key` is occupied.
    pub fn is_occupied(&self, key: Key) -> bool {
        matches!(self.voxel(key), Voxel::Occupied(_))
    }

    /// Every voxel of the map that a ray has reached.
    pub fn known(&self) -> impl Iterator<Item = Key> + '_ {
        self.log_odds
            .iter()
            .enumerate()
            .filter(|(_, value)| !value.is_nan())
            .map(|(index, _)| self.grid.key_of(index))
    }

    /// Every occupied voxel of the map.
    pub fn occupied(&self) -> impl Iterator<Item = Key> + '_ {
        self.log_odds
            .iter()
            .enumerate()
            .filter(|(_, value)| occupied(**value))
            .map(|(index, _)| self.grid.key_of(index))
    }

    /// The occupied voxels whose centers lie inside the box from `min` to
    /// `max`, in the map's frame, any part of which may lie outside the map.
    pub fn occupied_in(&self, min: [f64; 3], max: [f64; 3]) -> Vec<Key> {
        let (lo, hi) = (self.grid.nearest(min), self.grid.nearest(max));
        let inside = |key: &Key| {
            let center = self.center(*key);
            (0..3).all(|i| center[i] >= min[i] && center[i] <= max[i])
        };
        (lo.0[0]..=hi.0[0])
            .flat_map(|x| {
                (lo.0[1]..=hi.0[1])
                    .flat_map(move |y| (lo.0[2]..=hi.0[2]).map(move |z| Key([x, y, z])))
            })
            .filter(|key| inside(key) && self.is_occupied(*key))
            .collect()
    }

    /// Empty the map: every voxel unknown.
    pub fn clear(&mut self) {
        self.log_odds.fill(f32::NAN);
    }

    /// Insert one scan, each point by the rules of MoveIt's point cloud
    /// updater:
    /// - a point with a coordinate that is not a finite number changes
    ///   nothing: it has no voxel;
    /// - a point farther from the sensor than its range clears the voxels its
    ///   ray crosses up to the range, the voxel at the range included, and
    ///   marks nothing;
    /// - a point inside one of `cutouts` is the robot seeing itself: its
    ///   voxel takes the lowest log-odds, and its ray clears;
    /// - any other point is a hit, and the voxels its ray crosses up to the
    ///   center of its voxel are misses.
    ///
    /// Each voxel takes one update per scan: a voxel the robot cuts out is
    /// set to the lowest log-odds, else a hit is a hit, else a crossed voxel
    /// is a miss. Rays run from the sensor's origin in single precision and
    /// end at voxel centers, as OctoMap walks them. The scan's sensor and
    /// every cutout stand in the map's frame, or the scan changes nothing and
    /// the frame that differs is refused.
    ///
    /// A scan with no shape cut out still clears its far points to the range:
    /// MoveIt's updater marks them beyond range only while its shape mask
    /// holds a body.
    pub fn insert(&mut self, scan: &Scan, cutouts: &[&dyn Cutout]) -> Result<(), InsertError> {
        if *scan.frame != self.frame {
            return Err(InsertError::ScanFrame {
                scan: scan.frame.clone(),
                map: self.frame.clone(),
            });
        }
        if let Some(cutout) = cutouts.iter().find(|cutout| *cutout.frame() != self.frame) {
            return Err(InsertError::CutoutFrame {
                cutout: cutout.frame().clone(),
                map: self.frame.clone(),
            });
        }
        let sensor = scan.sensor;
        let size = self.grid.size;
        let max_range = scan.max_range_m.get();
        let origin = sensor.origin().map(|c| c as f32);
        let mut hits = HashSet::new();
        let mut cut_out = HashSet::new();
        let mut beyond_range = HashSet::new();
        for point in scan.points {
            let local = point.map(f64::from);
            let distance = (local[0] * local[0] + local[1] * local[1] + local[2] * local[2]).sqrt();
            if distance > max_range {
                // tf2's normalize multiplies by the reciprocal of the length.
                let inverse = 1.0 / distance;
                let at_range = sensor.apply(local.map(|c| c * inverse * max_range));
                beyond_range.extend(size.key(at_range));
                continue;
            }
            let in_map = sensor.apply(local);
            let Some(key) = size.key(in_map) else {
                continue;
            };
            if cutouts.iter().any(|cutout| cutout.contains(in_map)) {
                cut_out.insert(key);
            } else {
                hits.insert(key);
            }
        }
        let mut misses = HashSet::new();
        let cross = |end: Key, misses: &mut HashSet<Key>| {
            if let Some(ray) = ray_keys(size, origin, size.center_f32(end)) {
                misses.extend(ray);
            }
        };
        for &key in hits.iter().chain(&cut_out) {
            cross(key, &mut misses);
        }
        for &key in &beyond_range {
            misses.insert(key);
            cross(key, &mut misses);
        }
        for key in &cut_out {
            hits.remove(key);
            misses.remove(key);
        }
        for key in &hits {
            misses.remove(key);
        }
        let model = self.model;
        for (keys, update) in [
            (&misses, model.miss),
            (&hits, model.hit),
            (&cut_out, model.cut_out),
        ] {
            for key in keys {
                if let Some(index) = self.grid.index(*key) {
                    let current = Some(self.log_odds[index]).filter(|v| !v.is_nan());
                    self.log_odds[index] = model.updated(current, update);
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map() -> VoxelMap {
        VoxelMap::new(
            Frame::parse("world").expect("a frame"),
            PositiveFinite::parse(0.05).expect("positive"),
            Bounds {
                min: [-1.0; 3],
                max: [1.0; 3],
            },
        )
        .expect("a valid map")
    }

    #[test]
    fn a_map_refuses_bounds_it_cannot_hold() {
        let world = || Frame::parse("world").expect("a frame");
        let size = PositiveFinite::parse(0.05).expect("positive");
        let inverted = Bounds {
            min: [1.0; 3],
            max: [-1.0; 3],
        };
        assert!(matches!(
            VoxelMap::new(world(), size, inverted),
            Err(MapError::Inverted { .. })
        ));
        let huge = Bounds {
            min: [-1000.0; 3],
            max: [1000.0; 3],
        };
        assert!(matches!(
            VoxelMap::new(world(), size, huge),
            Err(MapError::TooLarge { .. })
        ));
        let beyond = Bounds {
            min: [-1.0; 3],
            max: [5000.0; 3],
        };
        assert!(matches!(
            VoxelMap::new(world(), size, beyond),
            Err(MapError::OutsideTree { .. })
        ));
        assert_eq!(
            VoxelMap::new(world(), size, beyond)
                .unwrap_err()
                .to_string(),
            "the bounds' corner [5000.0, 5000.0, 5000.0] lies outside the voxel tree, which \
             holds 32768 voxels from the origin on each side of each axis, 1638.4 m at 0.05 m \
             voxels: give bounds within 1638.4 m of the origin or use larger voxels"
        );
    }

    // The grid index and the key of a voxel run both ways.
    #[test]
    fn indices_and_keys_run_both_ways() {
        let grid = map().grid;
        for index in [0, 1, 7, 1000, grid.len() - 1] {
            assert_eq!(grid.index(grid.key_of(index)), Some(index));
        }
        assert_eq!(grid.index(Key([1000, 0, 0])), None);
    }
}

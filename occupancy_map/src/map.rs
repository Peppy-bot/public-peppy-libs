//! The voxel map: a bounded grid of OctoMap log-odds, a voxel no ray has
//! reached unknown, a scan inserted by the rules of MoveIt's point cloud
//! updater (moveit2 2.15.2 `PointCloudOctomapUpdater::cloudMsgCallback`),
//! and the voxels a collision object intersects cleared.

use std::collections::HashSet;

use camera_core::pose::Pose;
use collision_geometry::chain::{Padding, PlacedMask};
use collision_geometry::gjk::Support;
use collision_geometry::shapes::{PlacedMesh, PlacedShape};
use control_core::positive_finite::PositiveFinite;
use frame_tree::FrameName;
use nalgebra::{Point3, Vector3};

use crate::grid::Grid;
use crate::key::{Key, TREE_VOXELS, VoxelSize};
use crate::ray::walk_ray;
use crate::scan_updates::{ScanUpdate, ScanUpdates};
use crate::sensor_model::{SensorModel, occupied};

/// The most voxels a map holds, a gibibyte of log-odds and a quarter of a
/// gibibyte of the bytes of an insert (`ScanUpdates`): a map that fits in
/// memory.
const MAX_VOXELS: usize = 1 << 28;

/// The distance within which a shape touches a voxel's box: ten times the gap at which
/// GJK counts two bodies as touching, 1e-10 m. GJK measures a face that touches a voxel's
/// face as a gap of rounding, 2e-17 m for a box from 10 cm on a grid of 10 cm voxels, and
/// no inside test can place a point on the surface.
const TOUCH_M: f64 = 1e-9;

/// How far beyond the bounds of a shape, and of a mesh's triangle, a voxel's box is
/// measured against it: twice `TOUCH_M`, so a voxel `TOUCH_M` away lies well within it
/// past the rounding of a coordinate divided by the voxel size, 1e-13 m at the edge of a
/// tree of 2 cm voxels.
const REACH_M: f64 = 2.0 * TOUCH_M;

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
    ScanFrame { scan: FrameName, map: FrameName },
    #[error(
        "a cutout stands in frame \"{cutout}\" and the map is in \"{map}\": place the \
         cutout in \"{map}\""
    )]
    CutoutFrame { cutout: FrameName, map: FrameName },
}

/// A shape a scan cuts out of the map: a point inside it is the robot seeing
/// itself.
pub trait Cutout {
    /// The frame the shape stands in.
    fn frame(&self) -> &FrameName;

    /// Whether `point`, in the shape's frame, lies inside the shape.
    fn contains(&self, point: [f64; 3]) -> bool;
}

/// The robot's body as a cutout: its parts grown by a padding by
/// `collision_geometry` where they stood when the frame was taken, in the
/// frame of the URDF root, its chains at the frame's joint positions and the
/// body fixed around them, and where that root stands in the cutout's frame.
pub struct RobotCutout<'a> {
    frame: &'a FrameName,
    /// The cutout's frame placed in the URDF root's: the root's pose,
    /// inverted.
    to_root: Pose,
    parts: Vec<PlacedMask<'a>>,
}

impl<'a> RobotCutout<'a> {
    /// The robot of grown parts `parts`, whose URDF root stands at `root` in
    /// `frame`.
    pub fn new(frame: &'a FrameName, root: Pose, parts: Vec<PlacedMask<'a>>) -> Self {
        Self {
            frame,
            to_root: root.inverse(),
            parts,
        }
    }
}

impl Cutout for RobotCutout<'_> {
    fn frame(&self) -> &FrameName {
        self.frame
    }

    fn contains(&self, point: [f64; 3]) -> bool {
        let in_root = Point3::from(self.to_root.apply(point));
        self.parts.iter().any(|part| part.contains(&in_root))
    }
}

/// A collision object's shape grown by `padding`, by the rule a link grows by: a point
/// inside it is the sensor seeing the object.
pub struct ShapeCutout<'a> {
    /// The frame the shape stands in.
    pub frame: &'a FrameName,
    pub shape: PlacedShape<'a>,
    pub padding: Padding,
}

impl Cutout for ShapeCutout<'_> {
    fn frame(&self) -> &FrameName {
        self.frame
    }

    fn contains(&self, point: [f64; 3]) -> bool {
        self.shape.contains(&Point3::from(point), self.padding)
    }
}

/// The box of one voxel, as GJK measures it.
pub struct VoxelBox {
    center: Point3<f64>,
    half: f64,
}

impl VoxelBox {
    /// The center and the radius of the sphere through the box's corners.
    pub fn bounding_sphere(&self) -> (Point3<f64>, f64) {
        (self.center, self.half * 3.0_f64.sqrt())
    }
}

impl Support for VoxelBox {
    fn core_support(&self, dir: &Vector3<f64>) -> Point3<f64> {
        self.center + dir.map(|d| if d < 0.0 { -self.half } else { self.half })
    }
}

/// The corners of the box from `min` to `max` grown by `REACH_M` on each side.
fn reach((min, max): (Point3<f64>, Point3<f64>)) -> ([f64; 3], [f64; 3]) {
    (
        min.map(|c| c - REACH_M).into(),
        max.map(|c| c + REACH_M).into(),
    )
}

/// What `VoxelMap::mesh_intersecting` knows of a voxel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Cell {
    /// Not yet measured.
    Unknown,
    /// Gathered into the region being joined.
    Joined,
    /// A triangle of the mesh touches its box.
    Surface,
    /// Its box lies wholly inside the mesh.
    Inside,
    /// Its box lies wholly outside the mesh.
    Outside,
}

/// The storage indices of `grid` of the region of `Unknown` cells joined by their
/// faces to the one at `start`, each marked `Joined` in `cells`.
fn join_region(grid: &Grid, start: usize, cells: &mut [Cell]) -> Vec<usize> {
    let mut region = vec![start];
    cells[start] = Cell::Joined;
    let mut next = 0;
    while let Some(&index) = region.get(next) {
        next += 1;
        for neighbor in grid.face_neighbors(index) {
            if cells[neighbor] == Cell::Unknown {
                cells[neighbor] = Cell::Joined;
                region.push(neighbor);
            }
        }
    }
    region
}

/// Decides each region of `Unknown` cells of `grid` joined by their faces, whose voxels
/// all lie inside a mesh or all outside it, `Inside` or `Outside` by `touches`: the first
/// and the last voxel of the region decide it when they agree, and each voxel decides its
/// own cell when they disagree, as a wrong inside test of one of them makes them.
fn decide_regions(grid: &Grid, cells: &mut [Cell], touches: impl Fn(Key) -> bool) {
    let side = |inside: bool| if inside { Cell::Inside } else { Cell::Outside };
    for start in 0..cells.len() {
        if cells[start] != Cell::Unknown {
            continue;
        }
        let region = join_region(grid, start, cells);
        let last = *region.last().expect("a region holds its first cell");
        let first_side = touches(grid.key_of(start));
        let last_side = if last == start {
            first_side
        } else {
            touches(grid.key_of(last))
        };
        for index in region {
            cells[index] = side(match index {
                _ if first_side == last_side || index == start => first_side,
                _ if index == last => last_side,
                _ => touches(grid.key_of(index)),
            });
        }
    }
}

/// One reading of a sensor: its points in its own frame, in single precision
/// as a point cloud carries them, the sensor's pose in `frame`, and how far
/// it reads.
pub struct Scan<'a> {
    pub frame: &'a FrameName,
    pub sensor: Pose,
    pub points: &'a [[f32; 3]],
    pub max_range_m: PositiveFinite,
}

/// Where a point of a scan lies in the map's frame, as the map inserts it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Reading {
    /// A point within the scan's range: what its ray hit.
    Return([f64; 3]),
    /// A point beyond the scan's range: its ray's point at the range, which
    /// met nothing before it.
    NoReturn([f64; 3]),
}

impl Scan<'_> {
    /// Each point's reading in the map's frame, in the order of the points: a
    /// point farther from the sensor than the range reads no return at the
    /// range along its ray.
    pub fn readings(&self) -> impl Iterator<Item = Reading> + '_ {
        let max_range = self.max_range_m.get();
        self.points.iter().map(move |point| {
            let local = point.map(f64::from);
            let distance = (local[0] * local[0] + local[1] * local[1] + local[2] * local[2]).sqrt();
            if distance > max_range {
                // tf2's normalize multiplies by the reciprocal of the length.
                let inverse = 1.0 / distance;
                return Reading::NoReturn(
                    self.sensor.apply(local.map(|c| c * inverse * max_range)),
                );
            }
            Reading::Return(self.sensor.apply(local))
        })
    }
}

/// A map's voxels seen from above: a grid of `size_x` by `size_y` square
/// columns of side `side_m`, the first with its corner at `origin`, in meters
/// in the map's frame. Column `[i, j]` holds every voxel of the map whose key
/// lies `i` above the map's lowest on x and `j` above it on y, from the map's
/// bottom to its top.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Columns {
    pub origin: [f64; 2],
    pub size_x: u32,
    pub size_y: u32,
    pub side_m: f64,
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
    frame: FrameName,
    grid: Grid,
    /// Each voxel's log-odds, in the grid's order; NaN where no ray has
    /// reached it.
    log_odds: Vec<f32>,
    model: SensorModel,
    /// Where an insert gathers the update of each voxel, which the map keeps
    /// from one insert to the next.
    updates: ScanUpdates,
}

/// A clone holds the voxels of the map and none of the bytes of an insert, so it
/// copies the log-odds and no more, as a planner that clones the map for each plan
/// needs. The first scan that the clone takes makes its own bytes.
impl Clone for VoxelMap {
    fn clone(&self) -> Self {
        Self {
            frame: self.frame.clone(),
            grid: self.grid,
            log_odds: self.log_odds.clone(),
            model: self.model,
            updates: ScanUpdates::default(),
        }
    }
}

impl VoxelMap {
    /// An empty map in `frame` of voxels of `size` over `bounds`.
    pub fn new(frame: FrameName, size: PositiveFinite, bounds: Bounds) -> Result<Self, MapError> {
        Self::empty(frame, VoxelSize::new(size), bounds)
    }

    /// An empty map in `frame` of voxels of `size` over `bounds`.
    fn empty(frame: FrameName, size: VoxelSize, bounds: Bounds) -> Result<Self, MapError> {
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
            updates: ScanUpdates::default(),
        })
    }

    /// The frame the map holds its voxels in.
    pub fn frame(&self) -> &FrameName {
        &self.frame
    }

    /// The voxel holding `point`, when the map holds it.
    pub fn key(&self, point: [f64; 3]) -> Option<Key> {
        self.grid.key(point)
    }

    /// The edge of a voxel, in meters.
    pub fn voxel_size(&self) -> f64 {
        self.grid.size.get()
    }

    /// The center of voxel `key`.
    pub fn center(&self, key: Key) -> [f64; 3] {
        self.grid.center(key)
    }

    /// The box of voxel `key`.
    pub fn voxel_box(&self, key: Key) -> VoxelBox {
        VoxelBox {
            center: self.center(key).into(),
            half: self.voxel_size() / 2.0,
        }
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
        let inside = |key: &Key| {
            let center = self.center(*key);
            (0..3).all(|i| center[i] >= min[i] && center[i] <= max[i])
        };
        self.grid
            .keys_between(min, max)
            .filter(|key| inside(key) && self.is_occupied(*key))
            .collect()
    }

    /// Empty the map: every voxel unknown.
    pub fn clear(&mut self) {
        self.log_odds.fill(f32::NAN);
    }

    /// The map over `bounds`, in this map's frame and of its voxel size: each
    /// voxel that both maps hold keeps what this map holds of it, and a voxel
    /// that this map does not hold is unknown.
    pub fn recentered(&self, bounds: Bounds) -> Result<Self, MapError> {
        let mut map = Self::empty(self.frame.clone(), self.grid.size, bounds)?;
        for (index, &value) in self.log_odds.iter().enumerate() {
            if value.is_nan() {
                continue;
            }
            if let Some(kept) = map.grid.index(self.grid.key_of(index)) {
                map.log_odds[kept] = value;
            }
        }
        Ok(map)
    }

    /// The map's voxels seen from above, as columns.
    pub fn columns(&self) -> Columns {
        let side_m = self.voxel_size();
        let count = |axis: usize| {
            u32::try_from(self.grid.dims[axis]).expect("a map holds fewer voxels an axis than u32")
        };
        Columns {
            origin: [
                f64::from(self.grid.lo.0[0]) * side_m,
                f64::from(self.grid.lo.0[1]) * side_m,
            ],
            size_x: count(0),
            size_y: count(1),
            side_m,
        }
    }

    /// Clears every voxel of each of `columns`, from the map's bottom to its top: each
    /// takes one miss's log-odds, so it reads free and the next hit marks it occupied
    /// again, as Nav2's obstacle layer marks a cell its footprint cleared on the next
    /// frame that sees it.
    ///
    /// # Panics
    ///
    /// On a column outside [`VoxelMap::columns`].
    pub fn clear_columns(&mut self, columns: impl IntoIterator<Item = [u32; 2]>) {
        let [size_x, size_y, size_z] = self.grid.dims;
        let miss = self.model.miss;
        for [x, y] in columns {
            let (x, y) = (x as usize, y as usize);
            assert!(
                x < size_x && y < size_y,
                "column [{x}, {y}] lies outside the map's {size_x} by {size_y} columns"
            );
            let bottom = (x * size_y + y) * size_z;
            self.log_odds[bottom..bottom + size_z].fill(miss);
        }
    }

    /// Clears every voxel of the map that `shape` touches, as `touches` finds them: each
    /// takes the lowest log-odds, as a cut-out point's voxel does, so the map holds nothing
    /// where a collision object stands. A primitive is measured against each voxel of its
    /// bounds; a mesh as `mesh_intersecting` finds them, with the same verdicts wherever
    /// `touches` answers alike across a region of voxels that no triangle touches.
    pub fn clear_intersecting(&mut self, shape: &PlacedShape) {
        self.update_intersecting(shape, self.model.cut_out);
    }

    /// Marks occupied every voxel of the map that `shape` touches, the voxels
    /// `clear_intersecting` clears: each takes the highest log-odds, so a distance field of
    /// the map holds the collision object where it stands, as MoveIt's CHOMP holds each
    /// object of its world in its field.
    pub fn occupy_intersecting(&mut self, shape: &PlacedShape) {
        self.update_intersecting(shape, self.model.filled);
    }

    /// Updates by `update` every voxel of the map that `shape` touches, as `touches` finds
    /// them.
    fn update_intersecting(&mut self, shape: &PlacedShape, update: f32) {
        let (min, max) = reach(shape.bounds());
        let intersected: Vec<Key> = match shape.mesh() {
            Some(mesh) => self.mesh_intersecting(shape, &mesh, min, max),
            None => self
                .grid
                .keys_between(min, max)
                .filter(|key| self.touches(shape, *key))
                .collect(),
        };
        for key in intersected {
            let index = self
                .grid
                .index(key)
                .expect("a key between two of the grid's voxels");
            self.update(index, update);
        }
    }

    /// Whether `shape` intersects the box of voxel `key` or comes within 1e-9 m of it,
    /// `TOUCH_M`.
    pub fn touches(&self, shape: &PlacedShape, key: Key) -> bool {
        let voxel = self.voxel_box(key);
        shape.distance_to(&voxel, voxel.bounding_sphere()) <= TOUCH_M
    }

    /// The voxels between `min` and `max`, the reach of the closed mesh `shape`, that it
    /// touches, each as `touches` finds it. A voxel within `TOUCH_M` of a triangle is one,
    /// and lies within the triangle's reach. Every other voxel lies farther than `TOUCH_M`
    /// from every triangle, so its box lies wholly inside the mesh or wholly outside it; so
    /// do the boxes of a region of such voxels joined by their faces, whose union is
    /// connected and meets no triangle: the surface, which bounds the inside, passes between
    /// no two of them. `decide_regions` decides each region.
    fn mesh_intersecting(
        &self,
        shape: &PlacedShape,
        mesh: &PlacedMesh,
        min: [f64; 3],
        max: [f64; 3],
    ) -> Vec<Key> {
        let within = self.grid.within(min, max);
        let mut cells = vec![Cell::Unknown; within.len()];
        for t in mesh.triangles() {
            let (triangle_min, triangle_max) = reach(mesh.triangle_bounds(t));
            for key in self.grid.keys_between(triangle_min, triangle_max) {
                let index = within
                    .index(key)
                    .expect("a triangle's voxels lie among its mesh's");
                if cells[index] == Cell::Unknown
                    && mesh.triangle_distance(t, &self.voxel_box(key)) <= TOUCH_M
                {
                    cells[index] = Cell::Surface;
                }
            }
        }
        decide_regions(&within, &mut cells, |key| self.touches(shape, key));
        cells
            .iter()
            .enumerate()
            .filter(|(_, cell)| matches!(cell, Cell::Surface | Cell::Inside))
            .map(|(index, _)| within.key_of(index))
            .collect()
    }

    /// Applies `update` to the log-odds of the voxel at `index`, one never reached
    /// starting from none.
    fn update(&mut self, index: usize, update: f32) {
        let current = Some(self.log_odds[index]).filter(|v| !v.is_nan());
        self.log_odds[index] = self.model.updated(current, update);
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
        self.insert_points(scan, cutouts, &[])
    }

    /// Insert one scan as [`VoxelMap::insert`] inserts it with no cutout,
    /// together with `clearing`, more points in the scan's sensor frame, read
    /// as the scan reads its points, that clear and mark nothing: the voxels
    /// the ray of a point of `clearing` within the range crosses up to the
    /// center of its voxel are misses, as a hit's are, and its voxel takes
    /// nothing from it; a point beyond the range clears to the range as any
    /// does. Each voxel takes one update: a hit is a hit, else a crossed
    /// voxel is a miss. A navigator inserts the returns of a robot's own body
    /// so (`navigation_core`).
    pub fn insert_clearing(
        &mut self,
        scan: &Scan,
        clearing: &[[f32; 3]],
    ) -> Result<(), InsertError> {
        self.insert_points(scan, &[], clearing)
    }

    /// The insert of `scan` with `cutouts` and with the points `clearing`
    /// that clear and mark nothing, as [`VoxelMap::insert`] and
    /// [`VoxelMap::insert_clearing`] give it. It walks each ray voxel by
    /// voxel, gathers the strongest update of each voxel at the voxel's index
    /// (`ScanUpdates`), and then gives each voxel that holds an update that one
    /// update: its work follows the voxels that its rays cross, not the voxels
    /// of the map.
    fn insert_points(
        &mut self,
        scan: &Scan,
        cutouts: &[&dyn Cutout],
        clearing: &[[f32; 3]],
    ) -> Result<(), InsertError> {
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
        let origin = sensor.origin().map(|c| c as f32);
        let mut hits = HashSet::new();
        let mut cut_out = HashSet::new();
        let mut beyond_range = HashSet::new();
        for reading in scan.readings() {
            let in_map = match reading {
                Reading::Return(in_map) => in_map,
                Reading::NoReturn(at_range) => {
                    beyond_range.extend(size.key(at_range));
                    continue;
                }
            };
            let Some(key) = size.key(in_map) else {
                continue;
            };
            if cutouts.iter().any(|cutout| cutout.contains(in_map)) {
                cut_out.insert(key);
            } else {
                hits.insert(key);
            }
        }
        let clearing_scan = Scan {
            points: clearing,
            ..*scan
        };
        let mut cleared = HashSet::new();
        for reading in clearing_scan.readings() {
            match reading {
                Reading::Return(in_map) => cleared.extend(size.key(in_map)),
                Reading::NoReturn(at_range) => beyond_range.extend(size.key(at_range)),
            }
        }
        let mut updates = std::mem::take(&mut self.updates);
        updates.ready_for(self.grid.len());
        let grid = self.grid;
        // A voxel outside the map takes nothing, and a ray that ends outside
        // the map still gives each voxel of the map that it crosses.
        let mut give = |key: Key, update: ScanUpdate| {
            if let Some(index) = grid.index(key) {
                updates.give(index, update);
            }
        };
        let ends = hits.iter().chain(&cut_out).chain(&cleared);
        for &end in ends.chain(&beyond_range) {
            walk_ray(size, origin, size.center_f32(end), |crossed| {
                give(crossed, ScanUpdate::Miss);
            });
        }
        for &key in &beyond_range {
            give(key, ScanUpdate::Miss);
        }
        for &key in &hits {
            give(key, ScanUpdate::Hit);
        }
        for &key in &cut_out {
            give(key, ScanUpdate::CutOut);
        }
        let model = self.model;
        updates.drain(|index, update| self.update(index, update.log_odds(&model)));
        self.updates = updates;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    fn map() -> VoxelMap {
        VoxelMap::new(
            FrameName::parse("world").expect("a frame"),
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
        let world = || FrameName::parse("world").expect("a frame");
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

    // A clone holds none of the bytes of an insert, and takes a scan as the map takes
    // it: after the same scan, the two hold the same voxels at the same log-odds.
    #[test]
    fn a_clone_holds_no_bytes_of_an_insert_and_takes_a_scan_alike() {
        let frame = FrameName::parse("world").expect("a frame");
        let sensor = Pose::new([0.01, 0.02, 0.03], [0.0, 0.0, 0.0, 1.0]).expect("a pose");
        let scan = |points| Scan {
            frame: &frame,
            sensor,
            points,
            max_range_m: PositiveFinite::parse(0.8).expect("positive"),
        };
        let mut original = map();
        original
            .insert(&scan(&[[0.3, 0.1, 0.5], [0.0, -0.2, 0.9]]), &[])
            .expect("the map's frame");
        assert_ne!(original.updates, ScanUpdates::default());
        let mut clone = original.clone();
        assert_eq!(clone.updates, ScanUpdates::default());
        let next = [[0.4, 0.0, 0.4], [-0.1, -0.2, 0.9], [0.3, 0.1, 0.5]];
        original.insert(&scan(&next), &[]).expect("the map's frame");
        clone.insert(&scan(&next), &[]).expect("the map's frame");
        let voxels = |map: &VoxelMap| -> Vec<(Key, Voxel)> {
            map.known().map(|key| (key, map.voxel(key))).collect()
        };
        assert!(!voxels(&original).is_empty());
        assert_eq!(voxels(&clone), voxels(&original));
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

    // A box whose upper corner is not a number reaches below its lower one: the grid holds
    // no voxel of it.
    #[test]
    fn a_box_with_a_corner_that_is_not_a_number_holds_no_voxel() {
        let grid = map().grid;
        assert_eq!(grid.within([0.0; 3], [f64::NAN; 3]).len(), 0);
        assert_eq!(grid.keys_between([0.0; 3], [f64::NAN; 3]).count(), 0);
    }

    // Two voxels of a region decide it where they agree, a voxel between them that a
    // touch test answers otherwise among it; where they disagree each voxel decides its
    // own cell.
    #[test]
    fn the_first_and_the_last_voxel_decide_a_region_where_they_agree() {
        let grid = map().grid.within([0.01, 0.01, 0.01], [0.19, 0.01, 0.01]);
        let cells_of = |touches: fn(Key) -> bool| {
            let mut cells = vec![Cell::Unknown; grid.len()];
            decide_regions(&grid, &mut cells, touches);
            (0..grid.len())
                .map(|index| (grid.key_of(index).0[0], cells[index]))
                .collect::<Vec<_>>()
        };
        assert_eq!(grid.len(), 4, "a row of four voxels, x from 0 to 3");
        assert!(
            cells_of(|key| key.0[0] != 1)
                .iter()
                .all(|(_, cell)| *cell == Cell::Inside)
        );
        assert_eq!(
            cells_of(|key| key.0[0] <= 1),
            vec![
                (0, Cell::Inside),
                (1, Cell::Inside),
                (2, Cell::Outside),
                (3, Cell::Outside),
            ]
        );
    }

    // A voxel joins the voxels that share a face with it, those of the grid alone.
    #[test]
    fn a_voxel_joins_its_face_neighbors_in_the_grid() {
        let grid = map().grid;
        let neighbors = |key: Key| -> BTreeSet<Key> {
            let index = grid.index(key).expect("a voxel of the grid");
            grid.face_neighbors(index).map(|n| grid.key_of(n)).collect()
        };
        let inner = Key([0, 0, 0]);
        let steps = [
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1],
        ];
        let around: BTreeSet<Key> = steps
            .iter()
            .map(|step| Key(std::array::from_fn(|i| inner.0[i] + step[i])))
            .collect();
        assert_eq!(neighbors(inner), around);
        let corner = grid.lo;
        let inward: BTreeSet<Key> = steps
            .iter()
            .filter(|step| step.iter().all(|&s| s >= 0))
            .map(|step| Key(std::array::from_fn(|i| corner.0[i] + step[i])))
            .collect();
        assert_eq!(neighbors(corner), inward);
    }
}

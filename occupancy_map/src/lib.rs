//! A voxel map of what a depth sensor sees: log-odds occupancy over a bounded
//! grid of voxels in a frame the caller names, each scan's points clearing
//! the voxels their rays cross and marking the voxels they hit, with the
//! robot cut out, and the distance from any point to the nearest occupied
//! voxel.
//!
//! - [`VoxelMap`]: the map in its [`Frame`] over its [`Bounds`], its voxels
//!   ([`Voxel`], each at a [`Key`]), a [`Scan`]
//!   inserted with the shapes it cuts out ([`Cutout`]; [`RobotCutout`] is
//!   the robot's body as `collision_geometry` places it), the occupied voxels
//!   inside a box, and `clear`.
//! - [`depth_points`]: the points a z16 depth frame reads, from its stream's
//!   pinhole model.
//! - [`DistanceField`]: the distances of a map as it stands.
//!
//! A voxel is keyed, a ray walked and a log-odds updated by OctoMap 1.10's
//! arithmetic, and a scan inserted by the rules of MoveIt 2.15.2's point
//! cloud updater. A scan with no shape cut out still clears its far points to
//! the range: MoveIt's updater marks them beyond range only while its shape
//! mask holds a body.
#![forbid(unsafe_code)]

mod depth;
mod distance;
mod frame;
mod grid;
mod key;
mod map;
mod ray;
mod sensor_model;

pub use depth::{DepthError, depth_points};
pub use distance::{DistanceField, NotANumber};
pub use frame::{Frame, FrameError};
pub use key::Key;
pub use map::{Bounds, Cutout, InsertError, MapError, RobotCutout, Scan, Voxel, VoxelMap};

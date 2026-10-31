//! Collision geometry for any robot, read from its URDF and its collision meshes.
//!
//! - [`fit`] fits the convex pieces of one body from its collision surfaces: one
//!   circumscribing hull per `<collision>` element, which holds its surface by
//!   construction, or one per [`ClipRegion`](clip::ClipRegion) for a concave body, where
//!   the fit checks that the pieces together hold every vertex and face of the surfaces.
//! - [`gjk`] measures the distance between two convex shapes, signed through contact.
//! - [`chain::ChainGeometry`] holds the pieces of every link that a serial chain moves
//!   and places them at a joint configuration, each piece with a bounding sphere. Its
//!   [`Mask`](chain::Mask) tests whether a point lies inside a link, or inside the
//!   chain, grown by a padding: every point within the offset of each collision
//!   element scaled about its center, a mesh from its piece, a primitive as itself.
//! - [`urdf_collision`], [`stl`] and [`hull`] read the collision elements of a URDF
//!   (meshes, boxes, cylinders and spheres) and build the exact convex hull of a point
//!   cloud.
//!
//! Evidence for the chain geometry and the robot filter: at every configuration of the
//! chain collision scenarios of the ROS reference harness (`ros_reference`), the
//! OpenArm v2 arms dressed in their meshes and an arm dressed in primitives and a
//! turned wedge, the chain
//! carries the links that MoveIt's chain carries, each at MoveIt's frame within 1e-12,
//! and, for each padding, the filter deletes every probe within the offset of a mesh's
//! hull or of a primitive, deletes none farther from every body than the offset plus
//! its fit error, and answers as MoveIt's robot filter on every probe inside a body or
//! beyond the reach of both (`tests/ros_reference_chain_collision.rs`).
//!
//! Rust, with the C library Qhull built from source for the exact convex hulls; no
//! hardware or messaging deps. The one unsafe code, a module of [`hull`], points Qhull's
//! messages at a stream in memory.
#![deny(unsafe_code)]

pub mod chain;
pub mod clip;
mod error;
pub mod fit;
pub mod gjk;
pub mod hull;
mod simplify;
pub mod stl;
pub mod urdf_collision;

pub use error::{ContainmentFailure, GeometryError};

/// The linear-algebra crate of every type in this API, so a caller uses the same
/// `nalgebra` version.
pub use nalgebra;

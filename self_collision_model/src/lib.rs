//! Runtime self-collision detection for a bimanual arm.
//!
//! Every link is conservatively wrapped, at model construction, in a small set
//! of convex hulls fitted from its URDF collision elements (one hull per element,
//! or one per clip region for a concave body like the torso). Each hull is the
//! intersection of planes that support the element's surface, so it encloses the
//! geometry by construction, and the fit adds planes until the widest gap between
//! hull surface and the surface's convex hull is inside a millimeter budget. At
//! runtime the only geometry is Gilbert-Johnson-Keerthi distance between hulls, with
//! EPA recovering penetration depth on overlap, so the signed distance is continuous
//! through contact and cheap enough for every control tick.
//!
//! Robot-agnostic: any bimanual URDF whose arms are 7-DOF SRS chains
//! (`srs_model`'s contract) runs through the same construction. The caller
//! supplies the URDF, the collision mesh directory, the chain base links, and an
//! optional list of pairs to exclude from checking.
//!
//! - [`SelfCollisionModel::min_distance`] is the runtime query: the signed
//!   surface distance over the checked pairs, negative meaning penetration.
//! - [`SelfCollisionModel::distance_gradient`] adds the analytic gradient of
//!   that distance with respect to the joints, for a velocity-barrier caller.
//!
//! The model is a pure distance oracle: it reports clearances and their gradient,
//! and the caller decides how to throttle on them.
//!
//! Evidence that the pieces of each body hold its link's collision meshes: at every
//! configuration of the self distance scenarios of the ROS reference harness
//! (`ros_reference`), OpenArm v2 with both grippers posed, the distance of each checked
//! pair is no larger than the distance that MoveIt's FCL measures between the meshes
//! where they lie apart, and no larger than zero where they touch
//! (`tests/ros_reference_self_distance.rs`).
//!
//! No hardware or messaging deps. The geometry is `collision_geometry`'s, which
//! builds its exact convex hulls with the C library Qhull.
#![forbid(unsafe_code)]

mod assemble;
mod error;
mod model;
mod pairs;

pub use collision_geometry::gjk::Placed;
pub use error::{BuildError, CollisionError};
pub use model::{Builder, DistanceGradient, Proximity, SelfCollisionModel};
pub use pairs::PairSpec;

/// The linear-algebra crate of every type in this API (the witness points of a
/// [`Proximity`], the pose of a [`Placed`] piece), so a caller uses the same
/// `nalgebra` version.
pub use collision_geometry::nalgebra;

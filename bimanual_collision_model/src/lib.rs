//! Runtime self-collision detection for a bimanual arm.
//!
//! Every link is conservatively wrapped, at model construction, in a small set
//! of convex hulls decomposed from its URDF collision mesh (most links one
//! hull, a concave body like the torso a few). At runtime the only geometry is
//! Gilbert-Johnson-Keerthi distance between hulls, with EPA recovering
//! penetration depth on overlap, so the signed distance is continuous through
//! contact and cheap enough for every control tick.
//!
//! Robot-agnostic: any bimanual URDF whose arms are 7-DOF SRS chains
//! (`srs_model`'s contract) runs through the same construction. The caller
//! supplies the URDF, the collision mesh directory, the chain base links, and an
//! optional list of pairs to exclude from checking.
//!
//! - [`BimanualCollisionModel::min_distance`] is the runtime query: the signed
//!   surface distance over the checked pairs, negative meaning penetration.
//! - [`BimanualCollisionModel::distance_gradient`] adds the analytic gradient of
//!   that distance with respect to the joints, for a velocity-barrier caller.
//!
//! The model is a pure distance oracle: it reports clearances and their gradient,
//! and the caller decides how to throttle on them.
//!
//! Pure Rust, no hardware or messaging deps, same discipline as `srs_model`.
#![forbid(unsafe_code)]

mod assemble;
mod error;
mod gjk;
mod hull;
mod model;
mod pairs;
mod stl;
// `urdf_collision` stays public: the `visualize` example loads meshes through it.
pub mod urdf_collision;

pub use error::{BuildError, CollisionError, ContainmentFailure};
pub use hull::ConvexPiece;
pub use model::{BimanualCollisionModel, BodyPieces, Builder, DistanceGradient, PlacedPiece, Proximity};
pub use pairs::PairSpec;

/// Re-export the linear-algebra types so downstream crates use the same
/// `nalgebra` version `srs_model` (and `k`) were built against. `Point3` is
/// lifted to the crate root because it is in the public hull-piece API
/// ([`ConvexPiece::aabb`], [`ConvexPiece::from_points`]).
pub use srs_model::nalgebra;
pub use srs_model::nalgebra::Point3;

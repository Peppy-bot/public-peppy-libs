//! Runtime self-collision detection for a bimanual arm.
//!
//! Every link is conservatively wrapped in one or more capsules fitted at
//! model construction from the URDF's collision meshes; at runtime the only
//! geometry is capsule against capsule, closed-form and cheap enough for
//! every control tick.
//!
//! Robot-agnostic: any bimanual URDF whose arms are 7-DOF SRS chains
//! (`srs_model`'s contract) runs through the same construction. The caller
//! supplies the URDF, the collision mesh directory, the chain base links,
//! and a [`MarginPolicy`] (the governor band plus the reference poses).
//!
//! - [`Capsule`] is the primitive; [`Capsule::distance_to`] the signed surface
//!   distance (negative means penetration).
//! - [`GovernorBand`] is the direction-aware proximity law that scales
//!   commanded steps: separating motion always passes, approaching motion
//!   ramps to a stop across the band.
//!
//! Pure Rust, no hardware or messaging deps, same discipline as `srs_model`.

mod assemble;
pub mod fit;
pub mod geometry;
pub mod gjk;
pub mod hull;
mod governor;
mod model;
pub mod pairs;
pub mod stl;
pub mod urdf_collision;

pub use geometry::{Capsule, CapsuleDistance, point_segment_distance, segment_segment_closest};
pub use governor::GovernorBand;
pub use model::{BimanualCollisionModel, MarginPolicy, Proximity};
pub use pairs::PairSpec;

/// Re-export the linear-algebra types so downstream crates use the same
/// `nalgebra` version `srs_model` (and `k`) were built against.
pub use srs_model::nalgebra;

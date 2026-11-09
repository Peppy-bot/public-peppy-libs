//! Runtime self-collision detection for a robot of serial chains.
//!
//! Every link is conservatively wrapped, at model construction, in a small set
//! of convex hulls fitted from its URDF collision elements (one hull per element,
//! or one per clip region for a concave body like a torso). Each hull is the
//! intersection of planes that support the element's surface, so it encloses the
//! geometry by construction, and the fit adds planes until the widest gap between
//! hull surface and the surface's convex hull is inside a millimeter budget. At
//! runtime the only geometry is Gilbert-Johnson-Keerthi distance between hulls, with
//! EPA recovering penetration depth on overlap, so the signed distance is continuous
//! through contact and cheap enough for every control tick.
//!
//! A robot's model is built from data its description holds: the URDF and its
//! collision meshes, each chain from its base link to its tip, the fingers the chains'
//! links carry with the joint position at which each is shut, the yoked joints within
//! which two bodies of a chain go unchecked, the pairs the joint limits keep apart,
//! and the clip regions of its concave bodies ([`SelfCollisionModel::builder`]). A
//! model of `K` chains of `N` joints each is a `SelfCollisionModel<N, K>`, queried with
//! every chain's joints ([`Joints`]).
//!
//! - [`SelfCollisionModel::min_distance`] is the runtime query: the signed
//!   surface distance over the checked pairs, negative meaning penetration.
//! - [`SelfCollisionModel::distance_gradient`] adds the analytic gradient of
//!   that distance with respect to the joints, for a velocity-barrier caller.
//! - [`SelfCollisionModel::min_distance_of_chain`] is that distance over the pairs
//!   one chain's bodies ride, for a planner that moves that chain alone, and
//!   [`SelfCollisionModel::min_distance_of_chain_alone`] over those of them no other
//!   chain's bodies ride.
//! - [`SelfCollisionModel::fingers`] gives the fingers a chain carries
//!   (`ChainGeometry::carried`) where they hang at its opening, as
//!   [`Rider`](collision_geometry::chain::Rider)s of its links, for a planner's check of
//!   that chain.
//! - [`openarm`] builds OpenArm's model from `openarm_description`: both arms and the
//!   torso decomposed into its clip regions. [`so101`] builds SO-101's from
//!   `so101_description`: its arm and its jaw.
//!
//! The model is a pure distance oracle: it reports clearances and their gradient,
//! and the caller decides how to throttle on them.
//!
//! Evidence that the pieces of each body hold its link's collision meshes: at every
//! configuration of the self distance scenarios of the ROS reference harness
//! (`ros_reference`), OpenArm v2 with both grippers posed and SO-101 with its jaw posed,
//! the distance of each checked pair is no larger than the distance that MoveIt's FCL
//! measures between the meshes where they lie apart, and no larger than zero where they
//! touch (`tests/ros_reference_self_distance.rs`).
//!
//! No hardware or messaging deps. The geometry is `collision_geometry`'s, which
//! builds its exact convex hulls with the C library Qhull.

#![forbid(unsafe_code)]

// The unit tests share the integration tests' OpenArm fixture, which names this crate.
#[cfg(test)]
extern crate self as self_collision_model;
#[cfg(test)]
#[path = "../tests/fixtures/openarm.rs"]
mod fixture;

mod assemble;
mod error;
mod model;
mod openarm;
mod pairs;
mod so101;

pub use collision_geometry::gjk::Placed;
pub use error::{BuildError, CollisionError, FingerError};
pub use model::{Builder, DistanceGradient, Finger, Joints, Proximity, SelfCollisionModel};
pub use openarm::{OpenArmModel, openarm, openarm_fingers};
pub use pairs::PairSpec;
pub use so101::{So101Model, so101};

/// The linear-algebra crate of every type in this API (the witness points of a
/// [`Proximity`], the pose of a [`Placed`] piece), so a caller uses the same
/// `nalgebra` version.
pub use collision_geometry::nalgebra;

//! Runtime self-collision detection for a robot of serial chains.
//!
//! Every link is conservatively wrapped, at model construction, in a small set
//! of convex hulls fitted from its URDF collision elements, one hull per element. Each
//! hull is the
//! intersection of planes that support the element's surface, so it encloses the
//! geometry by construction, and the fit adds planes until the widest gap between
//! hull surface and the surface's convex hull is inside a millimeter budget. At
//! runtime the only geometry is Gilbert-Johnson-Keerthi distance between hulls, with
//! EPA recovering penetration depth on overlap, so the signed distance is continuous
//! through contact and cheap enough for every control tick.
//!
//! A robot's model is built from its description, a URDF and an SRDF, and the collision
//! meshes the URDF names ([`SelfCollisionModel::from_description`]): a chain per arm,
//! from its base link to its grasp link, the fingers of each arm's gripper between its
//! closed and open states, and every pair of bodies checked but those whose collisions
//! the SRDF disables. A model of `K` arms of `N` joints each is a
//! `SelfCollisionModel<N, K>`, queried with every chain's joints ([`Joints`]).
//!
//! - [`SelfCollisionModel::min_distance`] is the runtime query: the signed
//!   surface distance over the checked pairs, negative meaning penetration.
//! - [`SelfCollisionModel::distance_gradient`] adds the analytic gradient of
//!   that distance with respect to the joints, for a velocity-barrier caller.
//! - [`SelfCollisionModel::min_distance_of_chain`] is that distance over the pairs
//!   one chain's bodies ride, for a planner that moves that chain alone, and
//!   [`SelfCollisionModel::min_distance_of_chain_alone`] over those of them no other
//!   chain's bodies ride.
//! - [`SelfCollisionModel::min_distance_within`] and its forms of one chain give each
//!   of those distances as a check at a clearance reads it, exact under the
//!   clearance and a lower bound past it, measuring the pairs near the clearance
//!   alone.
//! - [`SelfCollisionModel::fingers`] gives the fingers a chain carries
//!   (`ChainGeometry::carried`) where they hang at its opening, as
//!   [`Rider`](collision_geometry::chain::Rider)s of its links, for a planner's check of
//!   that chain.
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

// The unit tests share the integration tests' fixture, which names this crate.
#[cfg(test)]
extern crate self as self_collision_model;
#[cfg(test)]
#[path = "../tests/fixtures/models.rs"]
mod fixture;

mod assemble;
mod error;
mod model;

pub use collision_geometry::gjk::Placed;
pub use error::{BuildError, CollisionError, FingerError};
pub use model::{DistanceGradient, Joints, Proximity, SelfCollisionModel};

/// The linear-algebra crate of every type in this API (the witness points of a
/// [`Proximity`], the pose of a [`Placed`] piece), so a caller uses the same
/// `nalgebra` version.
pub use collision_geometry::nalgebra;

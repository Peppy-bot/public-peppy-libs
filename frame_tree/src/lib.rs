//! tf2's buffer in Rust: the frames of a robot as one tree, from the transforms its
//! publishers send on the transforms contract, and the place of any frame in any other at a
//! time.
//!
//! - [`MovingMessage`] and [`StaticMessage`]: a message of `transforms` (one timestamp
//!   for all its transforms, and when they last started over) or of `static_transforms`
//!   (neither), field for field, and [`FrameTransform`], one transform, parsed from a
//!   publisher's values as tf2's `setTransform` checks one; a publisher makes its message
//!   with `from_transforms`.
//! - [`FrameTree`]: takes each message a consumer receives from a [`Publisher`], a message
//!   whose times a [`Stamp`] does not hold or that names a child frame twice refused whole
//!   ([`MessageError`]) and each other transform refused alone with its reason
//!   ([`TransformError`]) or kept with its quaternion normalized, per child frame and per
//!   publisher: a moving transform's samples of its newest start within a window the caller
//!   sets, and a static transform for every time. [`FrameTree::clear_moving`] drops every
//!   moving sample, as a consumer does when its clock goes back. [`FrameTree::lookup`]
//!   gives the source frame's place in the target frame at a time or at the latest time
//!   common to the chain, interpolated and composed in tf2's arithmetic.
//! - [`LookupError`]: an unknown frame, no chain between the frames, a transform with no
//!   value at the time, and transforms that two publishers hold or that form a cycle, each
//!   naming its frames.
//! - [`Transform`]: tf2's transform, a rotation matrix and an origin, which places a point,
//!   composes, inverts and interpolates in tf2's arithmetic, bit for bit.
//! - [`arithmetic`]: the rotation arithmetic of tf2 and KDL, operation for operation: the
//!   quaternion products, rotations and interpolation a lookup takes, tf2's rotation matrix,
//!   `setRPY` and `getYaw`, and KDL's matrices.
//!
//! The README lists each rule of tf2 the crate follows, by file and line of geometry2
//! 0.45.10, and where it departs.

pub mod arithmetic;
mod cache;
mod error;
mod frame;
mod transform;
mod transforms;
mod tree;

pub use cache::HeldStamps;
pub use error::{BrokenTree, LookupError, TransformName};
pub use frame::{FrameName, FrameNameError, LookupTime, Publisher, Stamp, TimeError};
pub use transform::{NotATransform, Transform};
pub use transforms::{
    FrameTransform, MessageError, MovingMessage, Placement, StaticMessage, TransformError,
    TransformErrorReason, TransformRecord,
};
pub use tree::{Found, FrameTree, ZeroWindow};

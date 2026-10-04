//! Where a robot can work: the definitions, the limits and the judgement that
//! a robot's own answer (from its design, on the robot) and a simulation's
//! answer (from a simulated world) share, so that both give the same kind of
//! answer, with the same limits and in the same words.
//!
//! A robot works a point when one of its arms reaches it and its perception
//! camera sees it:
//!
//! - **Reach** ([`Reach`]): some arm brings its grasp point within
//!   [`REACH_TOLERANCE`] of the point with its gripper's approach axis within
//!   [`GRASP_ANGLE_TOLERANCE`] of one of the [`GraspDirection`]s. Each side
//!   solves with its own inverse kinematics at every orientation
//!   [`GraspOrientation::all`] lists, eight rolls about each direction, and
//!   does not check the roll the gripper ends at.
//! - **View** ([`View`]): the point lies inside the perception camera's field
//!   of view and depth range ([`Camera::view_of`]). Each side adds its own
//!   test of what hides the point; only a simulation can say
//!   [`View::HiddenBy`].
//! - **The perception camera** ([`perception_camera`]) is the robot's one
//!   camera that gives depth and that no arm carries. A robot without one is
//!   judged on reach alone.
//!
//! A surface is measured at the points of [`Grid::STANDARD`] that lie on it
//! ([`GridPoint`]), each at a target [`ABOVE_SURFACE`] above the surface;
//! [`fit`] judges it workable when the points the robot both reaches and sees
//! cover at least [`MIN_WORKABLE_AREA`], and finds the largest rectangle of
//! them to put objects in. The [`messages`] module gives the one-line text of
//! each verdict.
//!
//! Frames and values:
//!
//! - The robot frame is fixed to the robot's base: its origin is the point
//!   the base stands on, +X points the way the robot faces, +Y to its left
//!   and +Z up. Grid points, rectangles and grasp orientations are in it.
//! - A rotation is a row-major 3x3 matrix whose columns are the axes of the
//!   rotated frame.
//! - Lengths are in metres, angles in radians.
//!
//! The crate takes and gives plain values (`[f64; 3]`, `[f64; 9]`): it knows
//! no math library, no robot description and no world, so a consumer on
//! nalgebra and one on glam read it alike.

#![forbid(unsafe_code)]

mod camera;
mod fit;
mod grasp;
mod grid;
pub mod messages;
mod verdict;

pub use camera::{
    Camera, CameraFacts, Intrinsics, PerceptionCameraError, perception_camera, view_of,
};
pub use fit::{
    Fit, Rectangle, fit, largest_workable_rectangle, nearest_workable, reach_bounds, view_bounds,
    workable_area,
};
pub use grasp::{GRASP_ROLLS, GraspDirection, GraspOrientation, angle_between};
pub use grid::{
    ABOVE_SURFACE, GRASP_ANGLE_TOLERANCE, Grid, Limits, MIN_WORKABLE_AREA, REACH_TOLERANCE,
};
pub use verdict::{GridPoint, Reach, View, workable};

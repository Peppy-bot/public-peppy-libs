//! What a mobile base needs to plan a path around what it sees, ported from
//! Nav2 1.5.2, with no ROS and no base:
//!
//! - [`grid`]: the cost grid, Nav2's `Costmap2D`, with Nav2's cost values.
//! - [`PaddedFootprint`]: a base's outline (`control_core::footprint::Footprint`)
//!   padded as Nav2 pads it, the footprint the cost grid, the planner and the
//!   collision checks take, with its inscribed radius.
//! - [`obstacles`]: what the sensors see, depth frames and laser scans kept in
//!   an occupancy map whose columns under the padded footprint are cleared
//!   after each frame, projected to a grid as Nav2's obstacle layer marks
//!   points, a drop below the floor marked too.
//! - [`saved_grid`]: a saved occupancy grid as Nav2's static layer takes a
//!   trinary map.
//! - [`inflation`]: a grid inflated around its lethal cells, as Nav2's
//!   inflation layer inflates one.
//! - [`costmap`]: an obstacle layer, over a saved grid when one is loaded,
//!   composed and inflated as Nav2's layered cost map composes its layers.
//! - [`planner`]: Smac 2D's A* on a cost grid to the goal's cell
//!   ([`planner::search`]) and its path, smoothed by Smac's smoother
//!   ([`planner::plan`]), with the failures [`planner::PlanError`] names.
//!
//! Poses on the floor are `camera_core::pose::PlanarPose`s.
//!
//! Each module names the Nav2 file and function it ports, and keeps its
//! types and order of operations where the parity tests of the ROS reference
//! harness compare bits.
#![forbid(unsafe_code)]

pub mod costmap;
mod footprint;
pub mod grid;
pub mod inflation;
pub mod obstacles;
pub mod planner;
mod pose;
mod raster;
pub mod saved_grid;

pub use footprint::{PaddedFootprint, PaddingError};

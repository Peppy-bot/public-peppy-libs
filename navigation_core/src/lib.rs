//! What a mobile base needs to plan a path around what it sees and to drive
//! it, ported from Nav2 1.5.2, with no ROS and no base:
//!
//! - [`grid`]: the cost grid, Nav2's `Costmap2D`, with Nav2's cost values.
//! - [`PaddedFootprint`]: a base's outline (`control_core::footprint::Footprint`)
//!   padded as Nav2 pads it, the footprint the cost grid, the planner, the
//!   controller, the stop and the collision checks take, with its inscribed
//!   and circumscribed radii.
//! - [`obstacles`]: what the sensors see, depth frames and laser scans kept in
//!   an occupancy map whose columns under the padded footprint are cleared
//!   after each frame, projected to a grid as Nav2's obstacle layer marks
//!   points, a drop below the floor marked too, and the columns each source's
//!   newest frame marked, which the stop reads.
//! - [`saved_grid`]: a saved occupancy grid as Nav2's static layer takes a
//!   trinary map.
//! - [`inflation`]: a grid inflated around its lethal cells, as Nav2's
//!   inflation layer inflates one.
//! - [`costmap`]: an obstacle layer, over a saved grid when one is loaded,
//!   composed and inflated as Nav2's layered cost map composes its layers.
//! - [`planner`]: Smac 2D's A* on a cost grid to the goal's cell
//!   ([`planner::search`]) and its path, smoothed by Smac's smoother
//!   ([`planner::plan`]), with the failures [`planner::PlanError`] names.
//! - [`path_handler`]: a planner's path pruned to the poses ahead of the
//!   base, as Nav2's controller server hands it to its controller.
//! - [`goal_checker`]: whether the base reached the goal of the plan it
//!   follows, as Nav2's stateful goal checker judges it, each plan with its
//!   own checker.
//! - [`progress_checker`]: whether the base makes progress, as Nav2's
//!   progress checker judges it.
//! - [`controller`]: Nav2's regulated pure pursuit: the forward speed and
//!   turn rate that follow the pruned plan, slowed by its curvature, near
//!   obstacles and on the approach to the goal, a turn in place towards a
//!   heading far off the base's, and the footprint checked along the arc
//!   ahead.
//! - [`stop`]: the commanded velocity slowed so that no occupied point, a
//!   lethal cell's center or a column of a source's newest frame, lies inside
//!   the padded footprint at any step of the projection the slowed velocity
//!   covers within a time, as Nav2's collision monitor slows it in its
//!   approach mode, and a stop while a source's readings stop arriving.
//!
//! Poses on the floor are `control_core::planar_pose::PlanarPose`s, and a body velocity
//! is a `control_core::twist::Twist`.
//!
//! Each module names the Nav2 file and function it ports, and keeps its
//! types and order of operations where the parity tests of the ROS reference
//! harness compare bits.
#![forbid(unsafe_code)]

pub mod controller;
pub mod costmap;
mod footprint;
pub mod goal_checker;
pub mod grid;
pub mod inflation;
pub mod obstacles;
pub mod path_handler;
pub mod planner;
mod pose;
pub mod progress_checker;
mod raster;
pub mod saved_grid;
pub mod stop;

pub use footprint::{PaddedFootprint, PaddingError};

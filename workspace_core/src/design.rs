//! The `workspace:v1` answer of a robot from its design: what the robot's own
//! node answers to describe_workspace and check_positions, from the reach its
//! own solver gives and from the view of its perception camera.
//!
//! The robot gives what is its own: whether an arm reaches a target
//! ([`Reach`]), and how the view of a point is checked ([`ViewCheck`]): not
//! at all, or through its perception camera, placed where its design fixes
//! it. This module gives the rest, so that every robot answers alike: the
//! parsing of a request ([`SurfaceHeight`], [`Positions`]), the reach of the
//! surfaces measured last ([`ReachMemo`]), and the composition of each answer
//! ([`describe_surface`], [`check_positions`]) from the verdicts, the largest
//! workable rectangle and the messages of this crate.
//!
//! A surface is flat and level, at a height above the robot's base point. Its
//! reach depends on the robot's design and on its height alone, so a robot
//! keeps the reach per height and answers a repeated height from it.

use std::collections::VecDeque;
use std::sync::Arc;

use crate::camera::Camera;
use crate::fit::{Fit, fit, largest_workable_rectangle, reach_bounds, view_bounds, workable_area};
use crate::grid::Limits;
use crate::messages::{
    Counted, SurfaceCamera, Unchecked, answer_message, point_message, robot_frame_placement,
    surface_message, unchecked_view_message,
};
use crate::verdict::{GridPoint, Reach, View, workable};

/// How far from the robot's base point a coordinate of a checked point and
/// the height of a described surface may be, in metres: far beyond any arm,
/// and near enough that every distance an answer computes stays a finite
/// number.
pub const MAX_COORDINATE: f64 = 1000.0;

/// How many surface heights a [`ReachMemo`] keeps.
pub const REACH_MEMO_HEIGHTS: usize = 32;

/// Why the surface height of a describe_workspace request is refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SurfaceHeightError {
    #[error("surface_height must be a finite number")]
    NotFinite,

    #[error(
        "surface_height must be within {} m of the robot's base point",
        MAX_COORDINATE
    )]
    TooFar,
}

/// Whether `value`, a coordinate or a height in the robot frame, is at most
/// [`MAX_COORDINATE`] from the robot's base point: the bound itself is
/// within it.
fn within_max_coordinate(value: f64) -> bool {
    value.abs() <= MAX_COORDINATE
}

/// The height of a surface above the robot's base point, parsed off a
/// describe_workspace request to the millimetre: a surface is measured at its
/// height to the millimetre, so two heights that round alike get one answer.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct SurfaceHeight {
    millimetres: i64,
}

impl SurfaceHeight {
    /// The height `metres` gives, to the millimetre; refused when `metres` is
    /// not a finite number, or when it is more than [`MAX_COORDINATE`] above
    /// or below the robot's base point. The bound applies to `metres` as the
    /// request gives it, before the rounding to the millimetre.
    pub fn from_wire(metres: f64) -> Result<Self, SurfaceHeightError> {
        if !metres.is_finite() {
            return Err(SurfaceHeightError::NotFinite);
        }
        if !within_max_coordinate(metres) {
            return Err(SurfaceHeightError::TooFar);
        }
        Ok(Self {
            millimetres: (metres * 1000.0).round() as i64,
        })
    }

    /// The height, in metres.
    pub fn metres(self) -> f64 {
        self.millimetres as f64 / 1000.0
    }

    /// The targets an arm reaches for on the surface, in the robot frame: one
    /// [`Limits::STANDARD`]'s `above_surface` above each point of its grid,
    /// in the grid's order.
    pub fn grid_targets(self) -> Vec<[f64; 3]> {
        let limits = Limits::STANDARD;
        let target_z = self.metres() + limits.above_surface;
        limits
            .grid
            .points()
            .into_iter()
            .map(|(x, y)| [x, y, target_z])
            .collect()
    }
}

/// Why the positions of a check_positions request are refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum PositionsError {
    #[error("positions must hold at least one point")]
    Empty,

    #[error("positions must hold 3 values (x, y, z) per point")]
    NotTriples,

    #[error("positions must hold finite numbers only")]
    NotFinite,

    #[error(
        "positions must hold coordinates within {} m of the robot's base point",
        MAX_COORDINATE
    )]
    TooFar,
}

/// The points of a check_positions request, parsed: at least one, each three
/// finite coordinates in the robot frame, each within [`MAX_COORDINATE`].
#[derive(Clone, Debug, PartialEq)]
pub struct Positions(Vec<[f64; 3]>);

impl Positions {
    /// The points of `values`, three values (x, y, z) per point, in order.
    pub fn from_wire(values: &[f64]) -> Result<Self, PositionsError> {
        if values.is_empty() {
            return Err(PositionsError::Empty);
        }
        let (points, rest) = values.as_chunks::<3>();
        if !rest.is_empty() {
            return Err(PositionsError::NotTriples);
        }
        if !values.iter().all(|value| value.is_finite()) {
            return Err(PositionsError::NotFinite);
        }
        if !values.iter().copied().all(within_max_coordinate) {
            return Err(PositionsError::TooFar);
        }
        Ok(Self(points.to_vec()))
    }

    /// The points, in the request's order.
    pub fn points(&self) -> &[[f64; 3]] {
        &self.0
    }
}

/// A count of reaches that is not one per target.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{given} reaches are given for {targets} targets: give one reach per target, in order")]
pub struct ReachCountError {
    /// How many targets need a reach.
    pub targets: usize,
    /// How many reaches are given.
    pub given: usize,
}

impl ReachCountError {
    /// `reaches` as one reach per target of `targets` targets, in order.
    fn check(targets: usize, reaches: Vec<Reach>) -> Result<Vec<Reach>, Self> {
        if reaches.len() != targets {
            return Err(Self {
                targets,
                given: reaches.len(),
            });
        }
        Ok(reaches)
    }
}

/// The reach of a surface at one height: whether an arm reaches each of its
/// grid targets ([`SurfaceHeight::grid_targets`]), in the grid's order.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceReach {
    height: SurfaceHeight,
    reaches: Arc<[Reach]>,
}

impl SurfaceReach {
    /// The reach of the surface at `height`, measured now: `reach` judges each
    /// of its grid targets, in the grid's order.
    pub fn measure(height: SurfaceHeight, reach: impl FnMut([f64; 3]) -> Reach) -> Self {
        Self {
            height,
            reaches: height.grid_targets().into_iter().map(reach).collect(),
        }
    }

    /// The reach of the surface at `height` from `reaches` measured
    /// elsewhere: one per grid target, in the grid's order.
    pub fn from_grid_order(
        height: SurfaceHeight,
        reaches: Vec<Reach>,
    ) -> Result<Self, ReachCountError> {
        let grid = Limits::STANDARD.grid;
        let reaches = ReachCountError::check(grid.rows() * grid.lanes(), reaches)?;
        Ok(Self {
            height,
            reaches: reaches.into(),
        })
    }

    /// The height of the surface.
    pub fn height(&self) -> SurfaceHeight {
        self.height
    }

    /// The reach of each grid target, in the grid's order.
    pub fn reaches(&self) -> &[Reach] {
        &self.reaches
    }
}

/// The points of a check_positions request, each with whether an arm
/// reaches it, in the request's order.
#[derive(Clone, Debug, PartialEq)]
pub struct PositionsReach(Vec<([f64; 3], Reach)>);

impl PositionsReach {
    /// The reach of `positions`, measured now: `reach` judges each point, in
    /// the request's order.
    pub fn measure(positions: &Positions, mut reach: impl FnMut([f64; 3]) -> Reach) -> Self {
        Self(
            positions
                .points()
                .iter()
                .map(|&point| (point, reach(point)))
                .collect(),
        )
    }

    /// The reach of `positions` from `reaches` measured elsewhere: one per
    /// point, in the request's order.
    pub fn from_request_order(
        positions: &Positions,
        reaches: Vec<Reach>,
    ) -> Result<Self, ReachCountError> {
        let reaches = ReachCountError::check(positions.points().len(), reaches)?;
        Ok(Self(
            positions.points().iter().copied().zip(reaches).collect(),
        ))
    }
}

/// The reach of the surfaces measured last, one per height, in the order
/// they were stored. Each holds a verdict per grid point and the heights
/// come from callers, so the memo keeps [`REACH_MEMO_HEIGHTS`] of them at
/// most: past that bound it drops the height it stored first, also when a
/// caller asked that height again after it was stored.
#[derive(Clone, Debug)]
pub struct ReachMemo {
    capacity: usize,
    surfaces: VecDeque<SurfaceReach>,
}

impl Default for ReachMemo {
    fn default() -> Self {
        Self::new()
    }
}

impl ReachMemo {
    /// An empty memo that keeps [`REACH_MEMO_HEIGHTS`] heights.
    pub fn new() -> Self {
        Self::with_capacity(REACH_MEMO_HEIGHTS)
    }

    fn with_capacity(capacity: usize) -> Self {
        Self {
            capacity,
            surfaces: VecDeque::with_capacity(capacity),
        }
    }

    /// The reach stored for the surface at `height`, if any.
    pub fn get(&self, height: SurfaceHeight) -> Option<SurfaceReach> {
        self.surfaces
            .iter()
            .find(|stored| stored.height == height)
            .cloned()
    }

    /// Stores `surface` and gives back the reach stored for its height: the
    /// one already there when another request measured the height first.
    pub fn insert(&mut self, surface: SurfaceReach) -> SurfaceReach {
        if let Some(stored) = self.get(surface.height) {
            return stored;
        }
        if self.surfaces.len() == self.capacity {
            self.surfaces.pop_front();
        }
        self.surfaces.push_back(surface.clone());
        surface
    }

    /// How many heights the memo holds.
    pub fn len(&self) -> usize {
        self.surfaces.len()
    }

    /// Whether the memo holds no height.
    pub fn is_empty(&self) -> bool {
        self.surfaces.is_empty()
    }
}

/// How an answer checks the view of the points it judges: through the
/// robot's perception camera, or not at all, and then why.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ViewCheck<'a> {
    /// The robot has no perception camera: no view is checked.
    NoPerceptionCamera,
    /// `camera` is the name of the robot's perception camera, but no camera
    /// geometry is linked for it: no view is checked.
    NoCameraGeometry { camera: &'a str },
    /// `camera` is the name of the robot's perception camera, and `geometry`
    /// is that camera in the robot frame, placed where the design fixes it,
    /// with the intrinsics and the depth stream the camera gives.
    Checked { camera: &'a str, geometry: Camera },
}

impl<'a> ViewCheck<'a> {
    /// The name of the robot's perception camera, if it has one.
    pub fn perception_camera(&self) -> Option<&'a str> {
        match *self {
            Self::NoPerceptionCamera => None,
            Self::NoCameraGeometry { camera } | Self::Checked { camera, .. } => Some(camera),
        }
    }

    /// What the check makes of `point`, in the robot frame:
    /// [`View::NoCamera`] when no view is checked.
    pub fn view_of(&self, point: [f64; 3]) -> View {
        match self {
            Self::Checked { geometry, .. } => geometry.view_of(point),
            Self::NoPerceptionCamera | Self::NoCameraGeometry { .. } => View::NoCamera,
        }
    }

    /// Why no view is checked; none when one is.
    pub fn unchecked(&self) -> Option<Unchecked<'a>> {
        match *self {
            Self::NoPerceptionCamera => Some(Unchecked::NoPerceptionCamera),
            Self::NoCameraGeometry { camera } => Some(Unchecked::NoCameraGeometry { camera }),
            Self::Checked { .. } => None,
        }
    }

    /// The checked camera as the message of a surface whose top is at `top`
    /// names it; none when no view is checked.
    fn surface_camera(&self, top: f64) -> Option<SurfaceCamera<'a>> {
        match *self {
            Self::Checked { camera, geometry } => Some(SurfaceCamera {
                name: camera,
                above_top: geometry.position[2] - top,
            }),
            Self::NoPerceptionCamera | Self::NoCameraGeometry { .. } => None,
        }
    }
}

/// What describe_workspace answers of one surface.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceAnswer {
    pub workable: bool,
    pub area: f64,
    /// `[x_min, x_max, y_min, y_max]` of the largest workable rectangle.
    pub rectangle: Option<[f64; 4]>,
    /// `[x_min, x_max, y_min, y_max]` bounding the points an arm reaches.
    pub reach: Option<[f64; 4]>,
    /// `[x_min, x_max, y_min, y_max]` bounding the points the camera sees.
    pub view: Option<[f64; 4]>,
    pub message: String,
}

/// What check_positions answers of one point.
#[derive(Clone, Debug, PartialEq)]
pub struct PointAnswer {
    pub position: [f64; 3],
    pub reach: Reach,
    pub view: View,
    pub message: String,
}

impl PointAnswer {
    /// Whether the robot can work the point ([`workable`]).
    pub fn workable(&self) -> bool {
        workable(&self.reach, &self.view)
    }
}

/// What check_positions answers of every point, in the request's order.
#[derive(Clone, Debug, PartialEq)]
pub struct PositionsAnswer {
    pub points: Vec<PointAnswer>,
    pub message: String,
}

impl PositionsAnswer {
    /// Whether the robot can work every point.
    pub fn all_workable(&self) -> bool {
        self.points.iter().all(PointAnswer::workable)
    }
}

/// Where on the flat, level surface whose reach is `surface` the robot can
/// work, the view of its points checked at the surface's top as
/// `view_check` says.
pub fn describe_surface(surface: &SurfaceReach, view_check: &ViewCheck<'_>) -> SurfaceAnswer {
    let limits = Limits::STANDARD;
    let top = surface.height.metres();
    let points: Vec<GridPoint> = limits
        .grid
        .points()
        .into_iter()
        .zip(surface.reaches.iter())
        .map(|((x, y), reach)| GridPoint {
            x,
            y,
            reach: reach.clone(),
            view: view_check.view_of([x, y, top]),
        })
        .collect();
    let fit = fit(&points, &limits);
    let placement = match &fit {
        Fit::Workable { rectangle, .. } => Some(robot_frame_placement(rectangle)),
        _ => None,
    };
    SurfaceAnswer {
        workable: fit.workable(),
        area: workable_area(&points, &limits.grid),
        rectangle: largest_workable_rectangle(&points, &limits.grid)
            .map(|rectangle| rectangle.to_array()),
        reach: reach_bounds(&points).map(|rectangle| rectangle.to_array()),
        view: view_bounds(&points).map(|rectangle| rectangle.to_array()),
        message: sentences([
            Some(surface_message(&fit, view_check.surface_camera(top))),
            placement,
            view_check.unchecked().map(unchecked_view_message),
        ]),
    }
}

/// Whether the robot can work each point of `positions`, the view of each
/// checked as `view_check` says.
pub fn check_positions(positions: &PositionsReach, view_check: &ViewCheck<'_>) -> PositionsAnswer {
    let camera = view_check.perception_camera();
    let points: Vec<PointAnswer> = positions
        .0
        .iter()
        .map(|(position, reach)| {
            let view = view_check.view_of(*position);
            let message = point_message(reach, &view, camera);
            PointAnswer {
                position: *position,
                reach: reach.clone(),
                view,
                message,
            }
        })
        .collect();
    let workable = points.iter().filter(|point| point.workable()).count();
    let message = answer_message(
        workable,
        points.len(),
        Counted::Points,
        view_check.unchecked(),
    );
    PositionsAnswer { points, message }
}

/// The sentences given, in order, as one message.
fn sentences<const N: usize>(parts: [Option<String>; N]) -> String {
    parts.into_iter().flatten().collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::{Depth, DepthModel, Intrinsics};
    use crate::grid::Grid;

    fn height(metres: f64) -> SurfaceHeight {
        SurfaceHeight::from_wire(metres).expect("a finite height within MAX_COORDINATE")
    }

    fn positions(values: &[f64]) -> Positions {
        Positions::from_wire(values).expect("valid positions")
    }

    fn reached() -> Reach {
        Reach::Reached { arm: "arm".into() }
    }

    fn short(by: f64) -> Reach {
        Reach::Short { by }
    }

    /// The reach of the surface at `metres` where an arm reaches the target
    /// above each grid point that `reaches(x, y)` holds, and stops 0.2 m
    /// short of every other one.
    fn surface(metres: f64, reaches: impl Fn(f64, f64) -> bool) -> SurfaceReach {
        SurfaceReach::measure(height(metres), |[x, y, _]| {
            if reaches(x, y) { reached() } else { short(0.2) }
        })
    }

    /// The chest camera of these tests: 1 m above the point 0.3 m ahead of
    /// the robot, looking straight down, 100 by 100 pixels with a field of
    /// 50/240 to each side of its axis, and measuring optical depths from
    /// 0.48 to 2 m. On a surface 0.5 m up it sees x from 0.196 to 0.404 m
    /// and y from -0.104 to 0.104 m; the targets 0.04 m above that surface
    /// are nearer than its depth range.
    fn chest() -> Camera {
        Camera {
            position: [0.3, 0.0, 1.0],
            // Optical +X (image right) is the robot frame's -Y, +Y (image
            // down) its -X, +Z (the view) its -Z.
            rotation: [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            intrinsics: Intrinsics {
                width: 100,
                height: 100,
                fx: 240.0,
                fy: 240.0,
                cx: 49.5,
                cy: 49.5,
            },
            depth: Some(Depth {
                model: DepthModel::Z,
                range: [0.48, 2.0],
            }),
        }
    }

    fn checked_by_chest() -> ViewCheck<'static> {
        ViewCheck::Checked {
            camera: "chest",
            geometry: chest(),
        }
    }

    fn assert_bounds(found: Option<[f64; 4]>, expected: [f64; 4]) {
        let found = found.expect("bounds");
        assert!(
            found
                .iter()
                .zip(expected)
                .all(|(a, b)| (a - b).abs() < 1e-9),
            "found {found:?}, expected {expected:?}"
        );
    }

    #[test]
    fn a_surface_height_is_parsed_to_the_millimetre() {
        assert_eq!(height(0.4504), height(0.45));
        assert_eq!(height(0.4506).metres(), 0.451);
        assert_eq!(height(-0.0004).metres(), 0.0);
        assert_eq!(height(-0.25).metres(), -0.25);
    }

    #[test]
    fn a_surface_height_within_max_coordinate_of_the_base_is_parsed_with_the_bound_itself() {
        assert_eq!(height(MAX_COORDINATE).metres(), 1000.0);
        assert_eq!(height(-MAX_COORDINATE).metres(), -1000.0);
        assert_eq!(height(999.9996).metres(), 1000.0);
    }

    #[test]
    fn a_surface_height_that_is_not_finite_or_too_far_from_the_base_is_refused() {
        let refusals = [
            (f64::NAN, SurfaceHeightError::NotFinite),
            (f64::INFINITY, SurfaceHeightError::NotFinite),
            (f64::NEG_INFINITY, SurfaceHeightError::NotFinite),
            // The bound applies to the height the request gives: this one
            // rounds to 1000 m, but it is more than 1000 m up.
            (1000.0004, SurfaceHeightError::TooFar),
            (1000.001, SurfaceHeightError::TooFar),
            (-1000.001, SurfaceHeightError::TooFar),
            (2e6, SurfaceHeightError::TooFar),
            (-2e6, SurfaceHeightError::TooFar),
            (1e300, SurfaceHeightError::TooFar),
            (f64::MAX, SurfaceHeightError::TooFar),
        ];
        for (metres, refusal) in refusals {
            assert_eq!(SurfaceHeight::from_wire(metres), Err(refusal), "{metres}");
        }
    }

    #[test]
    fn each_refusal_of_a_surface_height_reads_as_the_contract_says() {
        let texts = [
            (
                SurfaceHeightError::NotFinite,
                "surface_height must be a finite number",
            ),
            (
                SurfaceHeightError::TooFar,
                "surface_height must be within 1000 m of the robot's base point",
            ),
        ];
        for (refusal, text) in texts {
            assert_eq!(refusal.to_string(), text);
        }
    }

    #[test]
    fn positions_are_parsed_into_points_and_refused_when_empty_ragged_not_finite_or_too_far() {
        assert_eq!(
            positions(&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]).points(),
            [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        );
        assert_eq!(
            positions(&[MAX_COORDINATE, -MAX_COORDINATE, 0.0]).points(),
            [[1000.0, -1000.0, 0.0]],
            "the bound itself is within it"
        );
        let refusals = [
            (vec![], PositionsError::Empty),
            (vec![0.1, 0.2, 0.3, 0.4], PositionsError::NotTriples),
            (vec![0.1, f64::NAN, 0.3], PositionsError::NotFinite),
            (vec![0.1, 0.2, f64::INFINITY], PositionsError::NotFinite),
            (vec![1e200, 0.0, 0.0], PositionsError::TooFar),
            (vec![0.0, -1000.5, 0.0], PositionsError::TooFar),
        ];
        for (values, refusal) in refusals {
            assert_eq!(Positions::from_wire(&values), Err(refusal), "{values:?}");
        }
    }

    #[test]
    fn each_refusal_of_positions_reads_as_the_contract_says() {
        let texts = [
            (
                PositionsError::Empty,
                "positions must hold at least one point",
            ),
            (
                PositionsError::NotTriples,
                "positions must hold 3 values (x, y, z) per point",
            ),
            (
                PositionsError::NotFinite,
                "positions must hold finite numbers only",
            ),
            (
                PositionsError::TooFar,
                "positions must hold coordinates within 1000 m of the robot's base point",
            ),
        ];
        for (refusal, text) in texts {
            assert_eq!(refusal.to_string(), text);
        }
    }

    #[test]
    fn the_grid_targets_of_a_height_stand_above_every_grid_point_in_the_grids_order() {
        let targets = height(0.45).grid_targets();
        let points = Grid::STANDARD.points();
        assert_eq!(targets.len(), points.len());
        for (target, (x, y)) in targets.iter().zip(points) {
            assert_eq!(target[..2], [x, y]);
            assert!((target[2] - 0.49).abs() < 1e-12, "{target:?}");
        }
    }

    #[test]
    fn a_reach_measured_elsewhere_is_taken_only_as_one_reach_per_target_in_order() {
        let at = height(0.3);
        let grid_size = Grid::STANDARD.points().len();
        let pattern = |index: usize| {
            if index.is_multiple_of(3) {
                reached()
            } else {
                short(index as f64)
            }
        };
        let given = SurfaceReach::from_grid_order(at, (0..grid_size).map(pattern).collect())
            .expect("one reach per grid target");
        let mut index = 0;
        let measured = SurfaceReach::measure(at, |_| {
            index += 1;
            pattern(index - 1)
        });
        assert_eq!(given, measured);
        assert_eq!(given.height(), at);
        assert_eq!(given.reaches()[1], short(1.0));
        assert_eq!(
            SurfaceReach::from_grid_order(at, vec![reached(); grid_size - 1]),
            Err(ReachCountError {
                targets: grid_size,
                given: grid_size - 1
            })
        );

        let two = positions(&[0.3, 0.0, 0.5, 0.9, 0.0, 0.5]);
        assert_eq!(
            PositionsReach::from_request_order(&two, vec![reached(), short(0.4)]),
            Ok(PositionsReach(vec![
                ([0.3, 0.0, 0.5], reached()),
                ([0.9, 0.0, 0.5], short(0.4))
            ]))
        );
        let refused = PositionsReach::from_request_order(&two, vec![reached(); 3]).unwrap_err();
        assert_eq!(
            refused,
            ReachCountError {
                targets: 2,
                given: 3
            }
        );
        assert_eq!(
            refused.to_string(),
            "3 reaches are given for 2 targets: give one reach per target, in order"
        );
    }

    #[test]
    fn a_measured_reach_judges_each_target_once_in_order() {
        let mut judged = Vec::new();
        let at = height(0.2);
        SurfaceReach::measure(at, |target| {
            judged.push(target);
            reached()
        });
        assert_eq!(judged, at.grid_targets());

        let three = positions(&[0.3, 0.1, 0.5, 0.2, -0.1, 0.4, 0.6, 0.0, 0.3]);
        let mut judged = Vec::new();
        PositionsReach::measure(&three, |point| {
            judged.push(point);
            reached()
        });
        assert_eq!(judged, three.points());
    }

    #[test]
    fn the_reach_memo_drops_the_height_it_stored_first_past_its_bound() {
        let stored = |metres: f64, by: f64| SurfaceReach::measure(height(metres), |_| short(by));
        let mut memo = ReachMemo::with_capacity(2);
        assert!(memo.is_empty());
        memo.insert(stored(0.1, 0.1));
        memo.insert(stored(0.2, 0.2));
        let kept = memo.insert(stored(0.1, 0.9));
        assert_eq!(kept.reaches()[0], short(0.1), "the stored reach stands");
        memo.insert(stored(0.3, 0.3));
        assert_eq!(memo.len(), 2);
        assert!(
            memo.get(height(0.1)).is_none(),
            "stored first, dropped first"
        );
        assert_eq!(
            memo.get(height(0.2)).expect("kept").reaches()[0],
            short(0.2)
        );
        assert_eq!(
            memo.get(height(0.3)).expect("kept").reaches()[0],
            short(0.3)
        );
        assert_eq!(
            memo.get(height(0.3004))
                .expect("the same millimetre")
                .height(),
            height(0.3)
        );
    }

    #[test]
    fn a_new_reach_memo_keeps_32_heights() {
        let mut memo = ReachMemo::new();
        for millimetres in 0..=32 {
            memo.insert(SurfaceReach::measure(
                height(f64::from(millimetres) / 1000.0),
                |_| reached(),
            ));
        }
        assert_eq!(memo.len(), 32);
        assert!(memo.get(height(0.0)).is_none());
        assert!(memo.get(height(0.001)).is_some());
        assert!(memo.get(height(0.032)).is_some());
    }

    #[test]
    fn the_view_is_checked_through_a_linked_perception_camera_alone_and_otherwise_says_why() {
        let seen_point = [0.3, 0.0, 0.5];
        let cases = [
            (
                ViewCheck::NoPerceptionCamera,
                None,
                Some(Unchecked::NoPerceptionCamera),
                View::NoCamera,
            ),
            (
                ViewCheck::NoCameraGeometry { camera: "chest" },
                Some("chest"),
                Some(Unchecked::NoCameraGeometry { camera: "chest" }),
                View::NoCamera,
            ),
            (checked_by_chest(), Some("chest"), None, View::Seen),
        ];
        for (view_check, camera, unchecked, view) in cases {
            assert_eq!(view_check.perception_camera(), camera, "{view_check:?}");
            assert_eq!(view_check.unchecked(), unchecked, "{view_check:?}");
            assert_eq!(view_check.view_of(seen_point), view, "{view_check:?}");
        }
        assert_eq!(
            checked_by_chest().view_of([0.6, 0.0, 0.5]),
            View::OutsideField
        );
    }

    #[test]
    fn a_robot_without_a_perception_camera_works_the_surface_it_reaches_and_says_so() {
        let reach = surface(0.45, |x, y| (0.2..=0.4).contains(&x) && y.abs() <= 0.1);
        let answer = describe_surface(&reach, &ViewCheck::NoPerceptionCamera);
        assert!(answer.workable);
        assert!((answer.area - 0.055).abs() < 1e-9, "{}", answer.area);
        assert_bounds(answer.rectangle, [0.2, 0.4, -0.1, 0.1]);
        assert_bounds(answer.reach, [0.2, 0.4, -0.1, 0.1]);
        assert_eq!(answer.view, None);
        assert_eq!(
            answer.message,
            "Workable: 0.055 m². Put items between x 0.20 and 0.40 m and between y -0.10 and \
             0.10 m in the robot frame. The view is not checked: the robot has no perception \
             camera."
        );
    }

    #[test]
    fn a_surface_without_linked_camera_geometry_is_judged_on_reach_and_names_the_camera() {
        let reach = surface(0.45, |x, y| (0.2..=0.4).contains(&x) && y.abs() <= 0.1);
        let answer = describe_surface(&reach, &ViewCheck::NoCameraGeometry { camera: "chest" });
        assert!(answer.workable);
        assert_eq!(answer.view, None);
        assert_eq!(
            answer.message,
            "Workable: 0.055 m². Put items between x 0.20 and 0.40 m and between y -0.10 and \
             0.10 m in the robot frame. The view is not checked: no camera geometry is linked \
             for the chest camera."
        );
    }

    #[test]
    fn a_surface_is_workable_where_the_arms_reach_the_targets_and_the_camera_sees_the_top() {
        // The camera sees the top 0.5 m below it, but not the targets 0.04 m
        // above the top, which are nearer than its depth range.
        let reach = surface(0.5, |x, _| x <= 0.34);
        let answer = describe_surface(&reach, &checked_by_chest());
        assert!(answer.workable, "{}", answer.message);
        assert!((answer.area - 0.04).abs() < 1e-9, "{}", answer.area);
        assert_bounds(answer.rectangle, [0.2, 0.34, -0.1, 0.1]);
        assert_bounds(answer.reach, [0.0, 0.34, -0.3, 0.3]);
        assert_bounds(answer.view, [0.2, 0.4, -0.1, 0.1]);
        assert_eq!(
            answer.message,
            "Workable: 0.040 m². Put items between x 0.20 and 0.34 m and between y -0.10 and \
             0.10 m in the robot frame."
        );
    }

    #[test]
    fn a_surface_the_camera_sees_apart_from_the_reach_says_where_each_lies() {
        let reach = surface(0.5, |x, _| x <= 0.1);
        let answer = describe_surface(&reach, &checked_by_chest());
        assert!(!answer.workable);
        assert_eq!((answer.area, answer.rectangle), (0.0, None));
        assert_bounds(answer.reach, [0.0, 0.1, -0.3, 0.3]);
        assert_bounds(answer.view, [0.2, 0.4, -0.1, 0.1]);
        assert_eq!(
            answer.message,
            "Not visible: the arms reach it from 0.00 to 0.10 m ahead of the robot and the \
             chest camera sees it from 0.20 to 0.40 m ahead of the robot, but it sees no point \
             an arm reaches."
        );
    }

    #[test]
    fn a_surface_above_the_camera_says_how_far_above_it_its_top_is() {
        let cases = [
            (
                1.17,
                "Not visible: the surface is 0.17 m above the chest camera, which cannot see it.",
            ),
            (
                1.0,
                "Not visible: the surface is level with the chest camera, which cannot see it.",
            ),
        ];
        for (metres, message) in cases {
            let answer = describe_surface(&surface(metres, |x, _| x <= 0.34), &checked_by_chest());
            assert!(!answer.workable);
            assert_eq!((answer.view, answer.rectangle), (None, None));
            assert_eq!(answer.area, 0.0);
            assert_bounds(answer.reach, [0.0, 0.34, -0.3, 0.3]);
            assert_eq!(answer.message, message);
        }
    }

    #[test]
    fn a_surface_no_arm_reaches_or_with_too_little_room_gives_no_placement() {
        let unreached =
            describe_surface(&surface(0.45, |_, _| false), &ViewCheck::NoPerceptionCamera);
        assert!(!unreached.workable);
        assert_eq!(
            (
                unreached.area,
                unreached.rectangle,
                unreached.reach,
                unreached.view
            ),
            (0.0, None, None, None)
        );
        assert_eq!(
            unreached.message,
            "Not reachable: no arm reaches any point of it with its gripper pointing down or \
             forward. The view is not checked: the robot has no perception camera."
        );

        let cramped = surface(0.45, |x, y| x == 0.3 && (y == 0.0 || y == 0.05));
        let answer = describe_surface(&cramped, &ViewCheck::NoPerceptionCamera);
        assert!(!answer.workable);
        assert!((answer.area - 0.002).abs() < 1e-9, "{}", answer.area);
        assert_bounds(answer.rectangle, [0.3, 0.3, 0.0, 0.05]);
        assert_eq!(
            answer.message,
            "Too little room: 0.002 m² is workable, under the 0.030 m² that holds a few objects. \
             The view is not checked: the robot has no perception camera."
        );
    }

    #[test]
    fn each_checked_point_carries_its_reach_view_and_message_in_the_requests_order() {
        let checked = positions(&[0.3, 0.0, 0.5, 0.9, 0.0, 0.5, 0.3, 0.05, 0.5, 0.3, 0.0, 0.54]);
        let reaches = vec![reached(), short(0.6), short(0.004), reached()];
        let answer = check_positions(
            &PositionsReach::from_request_order(&checked, reaches.clone()).expect("one each"),
            &checked_by_chest(),
        );
        let expected = [
            (
                View::Seen,
                "Workable: arm reaches it and the chest camera sees it.",
            ),
            (
                View::OutsideField,
                "Not workable: it is out of reach by 0.60 m, and it is outside the field of view \
                 of the chest camera.",
            ),
            (
                View::Seen,
                "Not workable: no arm reaches it with its gripper pointing down or forward; the \
                 chest camera sees it.",
            ),
            (
                View::OutOfDepth,
                "Not workable: arm reaches it, but it is outside the depth range of the chest \
                 camera.",
            ),
        ];
        assert_eq!(answer.points.len(), expected.len());
        for (index, (point, (view, message))) in answer.points.iter().zip(expected).enumerate() {
            assert_eq!(point.position, checked.points()[index]);
            assert_eq!(point.reach, reaches[index]);
            assert_eq!(point.view, view, "point {index}");
            assert_eq!(point.message, message, "point {index}");
        }
        let workable: Vec<bool> = answer.points.iter().map(PointAnswer::workable).collect();
        assert_eq!(workable, [true, false, false, false]);
        assert!(!answer.all_workable());
        assert_eq!(answer.message, "1 of the 4 points is workable.");
    }

    #[test]
    fn a_point_whose_view_is_not_checked_is_judged_on_reach_and_the_answer_says_why() {
        let one = positions(&[0.3, 0.0, 0.5]);
        let reach = PositionsReach::measure(&one, |_| reached());
        let cases = [
            (
                ViewCheck::NoPerceptionCamera,
                "The point is workable. The view is not checked: the robot has no perception \
                 camera.",
            ),
            (
                ViewCheck::NoCameraGeometry { camera: "chest" },
                "The point is workable. The view is not checked: no camera geometry is linked \
                 for the chest camera.",
            ),
        ];
        for (view_check, message) in cases {
            let answer = check_positions(&reach, &view_check);
            let [point] = answer.points.as_slice() else {
                panic!("one point answered: {answer:?}");
            };
            assert_eq!(point.view, View::NoCamera);
            assert_eq!(
                point.message,
                "Workable: arm reaches it; its view is not checked."
            );
            assert!(answer.all_workable());
            assert_eq!(answer.message, message);
        }
        let out_of_reach = PositionsReach::measure(&one, |_| short(0.25));
        let answer = check_positions(&out_of_reach, &ViewCheck::NoPerceptionCamera);
        assert!(!answer.all_workable());
        assert_eq!(
            answer.points[0].message,
            "Not workable: it is out of reach by 0.25 m; its view is not checked."
        );
        assert_eq!(
            answer.message,
            "The point is not workable. The view is not checked: the robot has no perception \
             camera."
        );
    }
}

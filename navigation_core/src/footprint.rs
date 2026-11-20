//! A mobile base's padded footprint, made from its outline
//! (`control_core::footprint::Footprint`, the outline base_state:v1's
//! describe_base gives) by [`PaddedFootprint::new`], Nav2 1.5.2's
//! `padFootprint` (nav2_costmap_2d src/footprint.cpp), each vertex moved along
//! x and along y away from the axes, as `sign0` signs it. The
//! [`PaddedFootprint`] is the one footprint the cost grid, the planner and the
//! collision checks take, as Nav2 hands its consumers only the padded one:
//! Smac 2D reads `Costmap2DROS::getRobotFootprint`, which returns the padded
//! footprint (include/nav2_costmap_2d/costmap_2d_ros.hpp:281-286), and the
//! collision monitor's approach polygon reads the footprint the cost map
//! publishes (nav2_bringup params/nav2_params.yaml:524), the padded one
//! (src/costmap_2d_ros.cpp:629).
//!
//! A padded footprint's inscribed and circumscribed radii are those Nav2
//! takes from it (src/footprint.cpp `calculateMinAndMaxDistances`, with
//! `distance` and `distanceToLine` of src/costmap_math.cpp), and
//! [`PaddedFootprint::placed_at`] is `transformFootprint`. It keeps the
//! outline it pads, the robot's own body, whose returns the live obstacles
//! keep out of the map and of the points the stop reads.
//!
//! The cost under the footprint at a pose is
//! `FootprintCollisionChecker::footprintCostAtPose`
//! (src/footprint_collision_checker.cpp), its edges rasterized as nav2_util's
//! `LineIterator` (include/nav2_util/line_iterator.hpp) rasterizes a line,
//! with one departure: the walk ends with [`LETHAL`] at the first edge that
//! holds a lethal cell, whatever cost the edges before it hold. Nav2's
//! `footprintCost` (src/footprint_collision_checker.cpp:70-84) ends early only
//! when the highest cost so far equals [`LETHAL`], so an unknown cell on an
//! earlier edge makes the highest cost unknown and a lethal cell on a later
//! edge then reads as unknown; here a lethal cell under the outline always
//! reads as lethal, for the smoother, the controller's arc check and
//! [`PaddedFootprint::collides_at`] alike.
//!
//! [`PaddedFootprint::collides_at`] checks one pose of base_navigation:v1's
//! sweep, under which a turn and a straight drive fail "when its sweep meets
//! an occupied cell", the padded footprint placed at each step of the
//! controller: the pose collides when the cost under the outline is
//! [`LETHAL`], so where a lethal cell lies under the outline, and where a
//! vertex lies off the grid, which `footprintCost` reads as [`LETHAL`]
//! (src/footprint_collision_checker.cpp:55-56, 66-67). A cell never seen
//! counts as free, as base_navigation:v1 counts it, where Nav2's spin
//! collides at or above [`LETHAL`] (nav2_behaviors plugins/spin.cpp:173,
//! `CostmapTopicCollisionChecker::isCollisionFree`,
//! src/costmap_topic_collision_checker.cpp:66), so on an unknown cell too.

use control_core::footprint::{Footprint, FootprintError};
use control_core::non_negative_finite::NonNegativeFinite;
use control_core::planar_pose::PlanarPose;

use crate::grid::{Cell, CostGrid, LETHAL};
use crate::pose::point_distance;
use crate::raster::raytrace_line;

/// A footprint padded as Nav2 pads it, which only [`PaddedFootprint::new`]
/// gives: its points, counterclockwise about the base's center, the least
/// and greatest distance from the center to a vertex or an edge, and the
/// outline it pads.
#[derive(Debug, Clone, PartialEq)]
pub struct PaddedFootprint {
    points: Vec<[f64; 2]>,
    outline: Footprint,
    inscribed_radius_m: f64,
    circumscribed_radius_m: f64,
}

/// Why a padding makes no footprint of an outline.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PaddingError {
    #[error(
        "footprint_padding_m {padding_m} m makes the footprint's edges from point {first} and \
         from point {second} cross: give a smaller padding"
    )]
    Crossing {
        padding_m: f64,
        first: usize,
        second: usize,
    },
    #[error(
        "footprint_padding_m {padding_m} m moves the footprint's points to no footprint \
         ({outline}): give a smaller padding"
    )]
    Outline {
        padding_m: f64,
        outline: FootprintError,
    },
}

impl PaddedFootprint {
    /// `footprint` grown by `padding_m` as Nav2 pads it: each vertex moved by
    /// the padding along x and along y away from the axes, a coordinate of 0
    /// kept. A padding that moves the points to an outline that is no
    /// footprint, one whose edges cross, gives its error.
    pub fn new(footprint: &Footprint, padding_m: NonNegativeFinite) -> Result<Self, PaddingError> {
        let padding = padding_m.get();
        let points: Vec<[f64; 2]> = footprint
            .points()
            .iter()
            .map(|point| point.map(|c| c + sign0(c) * padding))
            .collect();
        Footprint::parse(&points).map_err(|outline| match outline {
            FootprintError::Crossing { first, second } => PaddingError::Crossing {
                padding_m: padding,
                first,
                second,
            },
            outline => PaddingError::Outline {
                padding_m: padding,
                outline,
            },
        })?;
        let (inscribed_radius_m, circumscribed_radius_m) = min_and_max_distances(&points);
        Ok(Self {
            points,
            outline: footprint.clone(),
            inscribed_radius_m,
            circumscribed_radius_m,
        })
    }

    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }

    /// The least distance from the center to a vertex or an edge.
    pub fn inscribed_radius_m(&self) -> f64 {
        self.inscribed_radius_m
    }

    /// The greatest distance from the center to a vertex or an edge.
    pub fn circumscribed_radius_m(&self) -> f64 {
        self.circumscribed_radius_m
    }

    /// The footprint's vertices with the base's center at `pose`, in the
    /// frame of the pose.
    pub fn placed_at(&self, pose: PlanarPose) -> Vec<[f64; 2]> {
        let cos_th = pose.yaw().cos();
        let sin_th = pose.yaw().sin();
        self.points
            .iter()
            .map(|p| {
                [
                    pose.x() + (p[0] * cos_th - p[1] * sin_th),
                    pose.y() + (p[0] * sin_th + p[1] * cos_th),
                ]
            })
            .collect()
    }

    /// The cost of `grid` under the footprint's edges with the base's center
    /// at `pose`: each edge from the cell of one vertex to the cell of the
    /// next, the last from the first vertex's cell to the last one's;
    /// [`LETHAL`] when a vertex lies off the grid or an edge holds a lethal
    /// cell, and else the highest cost along the edges.
    pub(crate) fn cost_at(&self, grid: &CostGrid, pose: PlanarPose) -> f64 {
        footprint_cost(grid, &self.placed_at(pose))
    }

    /// Whether the base at `pose` collides on `grid`, one pose of
    /// base_navigation:v1's sweep: the cost under the footprint's outline, as
    /// the module gives it, is [`LETHAL`], so a lethal cell under the outline
    /// or a vertex off the grid collides, and an unknown cell alone counts as
    /// free. The base's center lies inside its outline, so a center off the
    /// grid puts a vertex off it.
    ///
    /// The check reads the cells under the outline's edges alone, a raster one
    /// cell thick, so a sweep checked at poses a step apart is sure to find
    /// every lethal cell its footprint passes over only if no point of the
    /// outline moves farther than the grid's cell from one checked pose to the
    /// next: a longer step carries an edge past a lethal cell that then lies
    /// inside the outline, under no edge, at both poses. A sweep that keeps
    /// that bound can still pass a lethal cell: the raster moves by whole cells
    /// as its edges cross the lattice, so at a step of under a cell, on a
    /// straight drive as on a turn, it can step over a lethal cell of one cell,
    /// which then lies under no edge at either pose. A step that moves the
    /// base's center a length along an arc of curvature kappa, turning toward
    /// +y, moves a point of the outline at (x, y) in the base's frame about
    /// sqrt((1 - kappa y)^2 + (kappa x)^2) times that length, kappa times its
    /// distance from the arc's center, to first order in the step's turn:
    /// 1 + kappa |y| for a point beside the center on the arc's outer side.
    /// Nav2 checks a sweep at poses alike: its arc check steps the center a
    /// cell along a drive, whatever its curvature, and the farthest vertex a
    /// cell along a turn in place (nav2_regulated_pure_pursuit_controller
    /// src/collision_checker.cpp:72-85), which the controller's arc check
    /// ports, and its spin and its drive on a heading check the poses a cycle
    /// of their frequency apart (nav2_behaviors plugins/spin.cpp:156-177,
    /// include/nav2_behaviors/plugins/drive_on_heading.hpp:212-235).
    pub fn collides_at(&self, grid: &CostGrid, pose: PlanarPose) -> bool {
        self.cost_at(grid, pose) == f64::from(LETHAL)
    }

    /// Whether `point`, in the base's frame, lies inside the outline the
    /// footprint pads, by [`is_point_inside_polygon`].
    pub(crate) fn outline_holds(&self, point: [f64; 2]) -> bool {
        is_point_inside_polygon(point[0], point[1], self.outline.points())
    }
}

/// nav2_util's `isPointInsidePolygon` (include/nav2_util/geometry_utils.hpp):
/// whether a ray from (`px`, `py`) along +X crosses the polygon's edges an
/// odd number of times.
pub(crate) fn is_point_inside_polygon(px: f64, py: f64, polygon: &[[f64; 2]]) -> bool {
    let mut inside = false;
    let mut i = polygon.len() - 1;
    for j in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[j]);
        if (py <= a[1]) == (py > b[1]) {
            let x_inter = a[0] + (py - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if x_inter > px {
                inside = !inside;
            }
        }
        i = j;
    }
    inside
}

/// costmap_math.hpp's `sign0`: -1, 0 or 1 by the sign of `x`.
fn sign0(x: f64) -> f64 {
    if x < 0.0 {
        -1.0
    } else if x > 0.0 {
        1.0
    } else {
        0.0
    }
}

/// `calculateMinAndMaxDistances`: over each vertex and the edge from it to
/// the next, the last edge closing the polygon, the least and the greatest
/// distance from the center.
fn min_and_max_distances(points: &[[f64; 2]]) -> (f64, f64) {
    let edges = points.iter().zip(points.iter().cycle().skip(1));
    edges.fold((f64::MAX, 0.0), |(min_dist, max_dist), (a, b)| {
        let vertex_dist = point_distance([0.0, 0.0], *a);
        let edge_dist = distance_to_line(0.0, 0.0, a[0], a[1], b[0], b[1]);
        (
            min_dist.min(vertex_dist.min(edge_dist)),
            max_dist.max(vertex_dist.max(edge_dist)),
        )
    })
}

/// The distance from the point `(px, py)` to the segment from `(x0, y0)` to
/// `(x1, y1)`.
fn distance_to_line(px: f64, py: f64, x0: f64, y0: f64, x1: f64, y1: f64) -> f64 {
    let a = px - x0;
    let b = py - y0;
    let c = x1 - x0;
    let d = y1 - y0;
    let dot = a * c + b * d;
    let len_sq = c * c + d * d;
    let param = dot / len_sq;
    let (xx, yy) = if param < 0.0 {
        (x0, y0)
    } else if param > 1.0 {
        (x1, y1)
    } else {
        (x0 + param * c, y0 + param * d)
    };
    point_distance([px, py], [xx, yy])
}

/// `footprintCost`, its walk ending at the first lethal edge: [`LETHAL`] when
/// a vertex of the closed polygon `oriented` lies off the grid or an edge
/// holds a lethal cell, and else the highest cost along its edges, walked in
/// Nav2's order, the closing edge from the first vertex to the last.
fn footprint_cost(grid: &CostGrid, oriented: &[[f64; 2]]) -> f64 {
    let lethal = f64::from(LETHAL);
    let geometry = grid.geometry();
    let cells: Option<Vec<Cell>> = oriented
        .iter()
        .map(|&point| geometry.cell_of(point))
        .collect();
    let Some(cells) = cells else {
        return lethal;
    };
    let closing = (cells[0], cells[cells.len() - 1]);
    let edges = cells
        .windows(2)
        .map(|pair| (pair[0], pair[1]))
        .chain(std::iter::once(closing));
    let mut footprint_cost = 0.0_f64;
    for (from, to) in edges {
        let edge_cost = line_cost(grid, from, to);
        if edge_cost == lethal {
            return lethal;
        }
        footprint_cost = edge_cost.max(footprint_cost);
    }
    footprint_cost
}

/// `lineCost`: the highest cost of the cells from `from` to `to`, lethal as
/// soon as one is, the cells walked as nav2_util's `LineIterator` walks them,
/// Bresenham's, which `raytrace_line` walks.
fn line_cost(grid: &CostGrid, from: Cell, to: Cell) -> f64 {
    let geometry = grid.geometry();
    let lethal = f64::from(LETHAL);
    let mut line = Vec::new();
    raytrace_line((from.x(), from.y()), (to.x(), to.y()), &mut line);
    let mut line_cost = 0.0;
    for (x, y) in line {
        let cell = geometry
            .cell(x, y)
            .expect("a line between two cells of a grid stays inside it");
        let point_cost = f64::from(grid.cost(cell));
        if point_cost == lethal {
            return point_cost;
        }
        if line_cost < point_cost {
            line_cost = point_cost;
        }
    }
    line_cost
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{FREE, GridGeometry, INSCRIBED, UNKNOWN};
    use control_core::positive_finite::PositiveFinite;

    /// Asserts that each point of `points` lies within rounding of `expected`'s.
    fn assert_close(points: &[[f64; 2]], expected: &[[f64; 2]]) {
        assert_eq!(points.len(), expected.len());
        for (point, expected) in points.iter().zip(expected) {
            assert!(
                (point[0] - expected[0]).abs() < 1e-12 && (point[1] - expected[1]).abs() < 1e-12,
                "{point:?} is not {expected:?}"
            );
        }
    }

    fn padding(padding_m: f64) -> NonNegativeFinite {
        NonNegativeFinite::parse(padding_m).unwrap()
    }

    fn unpadded(points: &[[f64; 2]]) -> PaddedFootprint {
        PaddedFootprint::new(&Footprint::parse(points).unwrap(), padding(0.0)).unwrap()
    }

    const RECTANGLE: [[f64; 2]; 4] = [[-0.2, -0.3], [0.4, -0.3], [0.4, 0.3], [-0.2, 0.3]];

    // A grid of 10 by 10 cells of 0.1 m, from the origin, with `cells`
    // (index, cost) set and every other cell free.
    fn grid_with(cells: &[(usize, u8)]) -> CostGrid {
        let geometry =
            GridGeometry::new(10, 10, PositiveFinite::parse(0.1).unwrap(), [0.0; 2]).unwrap();
        let mut costs = vec![FREE; 100];
        for &(index, cost) in cells {
            costs[index] = cost;
        }
        CostGrid::from_costs(geometry, costs).unwrap()
    }

    fn at(x: f64, y: f64) -> PlanarPose {
        PlanarPose::new(x, y, 0.0).unwrap()
    }

    // The rectangle at (0.55, 0.55) on `grid_with`'s grid runs its first edge
    // from cell (3, 2) to (9, 2), its second from (9, 2) to (9, 8), its third
    // from (9, 8) to (3, 8), and its closing edge from (3, 2) to (3, 8): the
    // index of cell (x, y) is y * 10 + x.
    const CENTER: (f64, f64) = (0.55, 0.55);
    const FIRST_EDGE: usize = 2 * 10 + 5;
    const THIRD_EDGE: usize = 8 * 10 + 5;
    const CLOSING_EDGE: usize = 5 * 10 + 3;

    // A notched outline's inscribed radius is its notch.
    #[test]
    fn the_inscribed_radius_reaches_into_a_notch() {
        let notched = unpadded(&[
            [-0.3, -0.3],
            [0.3, -0.3],
            [0.3, 0.3],
            [0.1, 0.3],
            [0.1, 0.1],
            [-0.1, 0.1],
            [-0.1, 0.3],
            [-0.3, 0.3],
        ]);
        assert!((notched.inscribed_radius_m() - 0.1).abs() < 1e-15);
    }

    // Padding moves each vertex along x and along y away from the axes, a
    // coordinate on an axis kept, and the inscribed radius grows with it.
    #[test]
    fn a_padded_footprint_moves_each_vertex_away_from_the_axes() {
        let kite =
            Footprint::parse(&[[0.0, -0.3], [0.4, 0.0], [0.0, 0.3], [-0.2, 0.0]]).expect("a kite");
        let padded = PaddedFootprint::new(&kite, padding(0.05)).expect("a padded kite");
        assert_close(
            padded.points(),
            &[[0.0, -0.35], [0.45, 0.0], [0.0, 0.35], [-0.25, 0.0]],
        );
        let rectangle = Footprint::parse(&RECTANGLE).unwrap();
        let square = PaddedFootprint::new(&rectangle, padding(0.01)).expect("a padded rectangle");
        assert_close(
            square.points(),
            &[[-0.21, -0.31], [0.41, -0.31], [0.41, 0.31], [-0.21, 0.31]],
        );
        assert!((square.inscribed_radius_m() - 0.21).abs() < 1e-12);
        assert_eq!(unpadded(&RECTANGLE).points(), RECTANGLE);
    }

    // A padding that makes a simple outline cross itself is refused by its
    // own name, the padding to make smaller: padded point 0 moves across the
    // edge from point 1 to point 2, which then crosses the closing edge.
    #[test]
    fn a_padding_that_makes_the_outline_cross_is_refused() {
        let outline =
            Footprint::parse(&[[-0.06, 0.23], [0.06, 0.33], [-0.33, 0.03], [0.42, -0.28]])
                .expect("a simple outline around its center");
        let refused = PaddedFootprint::new(&outline, padding(0.01)).unwrap_err();
        assert_eq!(
            refused,
            PaddingError::Crossing {
                padding_m: 0.01,
                first: 1,
                second: 3
            }
        );
        assert!(
            refused.to_string().contains("footprint_padding_m 0.01 m")
                && refused.to_string().ends_with("give a smaller padding"),
            "{refused}"
        );
    }

    // A padding that moves a point past the largest double makes an outline
    // whose point is not finite, refused by the outline's own error, the
    // padding to make smaller.
    #[test]
    fn a_padding_that_moves_a_point_past_the_largest_double_is_refused() {
        let outline = Footprint::parse(&[[-0.1, -0.1], [1e308, -0.1], [1e308, 0.1], [-0.1, 0.1]])
            .expect("a long outline around its center");
        let refused = PaddedFootprint::new(&outline, padding(1e308)).unwrap_err();
        assert!(
            matches!(
                refused,
                PaddingError::Outline {
                    outline: FootprintError::NotFinite { index: 1, .. },
                    ..
                }
            ),
            "{refused:?}"
        );
        assert!(
            refused.to_string().ends_with("give a smaller padding"),
            "{refused}"
        );
    }

    // A placed footprint turns about the base's center, then moves with it.
    #[test]
    fn a_placed_footprint_turns_and_moves_with_the_base() {
        let pose = PlanarPose::new(1.0, 2.0, std::f64::consts::FRAC_PI_2).unwrap();
        assert_close(
            &unpadded(&RECTANGLE).placed_at(pose),
            &[[1.3, 1.8], [1.3, 2.4], [0.7, 2.4], [0.7, 1.8]],
        );
    }

    fn square() -> Vec<[f64; 2]> {
        vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]
    }

    // A point inside the square is inside, one beyond an edge is not, and
    // on the edges the ray rule takes the left and upper edges in.
    #[test]
    fn a_point_is_inside_by_the_ray_rule() {
        let square = square();
        assert!(is_point_inside_polygon(0.0, 0.0, &square));
        assert!(!is_point_inside_polygon(1.5, 0.0, &square));
        assert!(!is_point_inside_polygon(0.0, 1.5, &square));
        assert!(is_point_inside_polygon(-1.0, 0.0, &square));
        assert!(!is_point_inside_polygon(1.0, 0.0, &square));
        assert!(is_point_inside_polygon(0.0, 1.0, &square));
        assert!(!is_point_inside_polygon(0.0, -1.0, &square));
    }

    // The outline a padding pads holds a point in the base's frame by its own
    // edge and not the padding: a point a millimeter inside the front edge is
    // held, one in the padding is not, nor one past both beside the base.
    #[test]
    fn the_outline_holds_a_point_inside_it_and_not_one_in_the_padding() {
        let rectangle = Footprint::parse(&RECTANGLE).unwrap();
        let footprint = PaddedFootprint::new(&rectangle, padding(0.05)).unwrap();
        assert!(footprint.outline_holds([0.399, 0.0]));
        assert!(!footprint.outline_holds([0.43, 0.0]));
        assert!(!footprint.outline_holds([0.0, -2.0]));
    }

    // The inscribed radius is the nearest edge, the circumscribed one the
    // farthest vertex.
    #[test]
    fn the_radii_are_the_nearest_edge_and_the_farthest_vertex() {
        let footprint = unpadded(&RECTANGLE);
        assert!((footprint.inscribed_radius_m() - 0.2).abs() < 1e-15);
        assert!((footprint.circumscribed_radius_m() - 0.5).abs() < 1e-15);
    }

    // The footprint reads the highest cost under its edges, the closing edge
    // among them, and lethal off the grid.
    #[test]
    fn the_cost_under_a_footprint_is_the_highest_along_its_edges() {
        let footprint = unpadded(&RECTANGLE);
        let (x, y) = CENTER;
        assert_eq!(
            footprint.cost_at(&grid_with(&[(5 * 10 + 7, 99)]), at(0.35, 0.55)),
            99.0
        );
        assert_eq!(
            footprint.cost_at(&grid_with(&[(5 * 10 + 7, 99)]), at(0.5, 0.5)),
            0.0
        );
        assert_eq!(
            footprint.cost_at(&grid_with(&[(CLOSING_EDGE, 99)]), at(x, y)),
            99.0
        );
        assert_eq!(
            footprint.cost_at(&grid_with(&[]), at(0.1, 0.5)),
            f64::from(LETHAL)
        );
    }

    // A lethal cell under the outline reads lethal whatever the edges hold
    // before it: a lethal edge before an unknown one, an unknown cell before
    // a lethal one on one edge, and an unknown edge before a lethal one.
    #[test]
    fn a_lethal_cell_under_the_outline_reads_lethal_first() {
        let footprint = unpadded(&RECTANGLE);
        let (x, y) = CENTER;
        let lethal = f64::from(LETHAL);
        let lethal_then_unknown = grid_with(&[(FIRST_EDGE, LETHAL), (THIRD_EDGE, UNKNOWN)]);
        assert_eq!(footprint.cost_at(&lethal_then_unknown, at(x, y)), lethal);
        let one_edge = grid_with(&[(2 * 10 + 4, UNKNOWN), (2 * 10 + 6, LETHAL)]);
        assert_eq!(footprint.cost_at(&one_edge, at(x, y)), lethal);
        let unknown_then_lethal = grid_with(&[(FIRST_EDGE, UNKNOWN), (THIRD_EDGE, LETHAL)]);
        assert_eq!(footprint.cost_at(&unknown_then_lethal, at(x, y)), lethal);
        assert_eq!(
            footprint.cost_at(&grid_with(&[(FIRST_EDGE, UNKNOWN)]), at(x, y)),
            f64::from(UNKNOWN)
        );
    }

    // A base collides on a lethal cell under its outline, on its closing
    // edge too and after an unknown cell, and with a vertex off the grid; an
    // unknown cell alone and a cell inflated below lethal leave it clear.
    #[test]
    fn a_base_collides_on_a_lethal_cell_under_its_outline() {
        let footprint = unpadded(&RECTANGLE);
        let center = at(CENTER.0, CENTER.1);
        let unknown_then_lethal = grid_with(&[(FIRST_EDGE, UNKNOWN), (THIRD_EDGE, LETHAL)]);
        assert!(footprint.collides_at(&unknown_then_lethal, center));
        assert!(footprint.collides_at(&grid_with(&[(THIRD_EDGE, LETHAL)]), center));
        assert!(footprint.collides_at(&grid_with(&[(CLOSING_EDGE, LETHAL)]), center));
        assert!(!footprint.collides_at(&grid_with(&[(FIRST_EDGE, UNKNOWN)]), center));
        assert!(!footprint.collides_at(&grid_with(&[(THIRD_EDGE, INSCRIBED)]), center));
        assert!(!footprint.collides_at(&grid_with(&[]), center));
        assert!(footprint.collides_at(&grid_with(&[]), at(0.1, 0.5)));
    }
}

//! Whether a robot can work a surface, judged from the grid points measured
//! on it, and where on it to put objects.

use std::cmp::Ordering;

use crate::grid::{Grid, Limits};
use crate::verdict::GridPoint;

/// An axis-aligned rectangle in the robot frame, `x` ahead of the robot and
/// `y` to its left, each from its least to its greatest value, in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle {
    pub x: [f64; 2],
    pub y: [f64; 2],
}

impl Rectangle {
    /// Its centre.
    pub fn centre(&self) -> [f64; 2] {
        [(self.x[0] + self.x[1]) / 2.0, (self.y[0] + self.y[1]) / 2.0]
    }

    /// Its four corners, counter-clockwise seen from above: from the one
    /// nearest the robot on its right, then ahead, then across to its left,
    /// then back.
    pub fn corners(&self) -> [[f64; 2]; 4] {
        [
            [self.x[0], self.y[0]],
            [self.x[1], self.y[0]],
            [self.x[1], self.y[1]],
            [self.x[0], self.y[1]],
        ]
    }

    /// `[x_min, x_max, y_min, y_max]`.
    pub fn to_array(&self) -> [f64; 4] {
        [self.x[0], self.x[1], self.y[0], self.y[1]]
    }
}

/// Whether a robot can work a surface, and if not, why.
#[derive(Clone, Debug, PartialEq)]
pub enum Fit {
    /// The robot reaches and sees `area` square metres of the surface, enough
    /// to work on. `rectangle` is the largest rectangle of workable points,
    /// its corners on grid points: where to put objects.
    Workable { area: f64, rectangle: Rectangle },
    /// No point of the surface's top was measured under the grid.
    NotMeasured,
    /// No arm reaches any measured point of the surface.
    NotReachable,
    /// The perception camera sees no point an arm reaches: `reach` bounds the
    /// points the arms reach, `view` those the camera sees, if any.
    NotVisible {
        reach: Rectangle,
        view: Option<Rectangle>,
    },
    /// The part the robot reaches and sees, `area` square metres, is smaller
    /// than `min`.
    TooLittleRoom { area: f64, min: f64 },
}

impl Fit {
    pub fn workable(&self) -> bool {
        matches!(self, Self::Workable { .. })
    }
}

/// Whether a robot can work the surface whose measured points are `points`:
/// some point lies on `limits`' grid, some arm reaches one, its perception
/// camera (when it has one) sees one an arm reaches, and the points it both
/// reaches and sees cover at least the smallest workable area. A point off
/// the grid is not judged.
pub fn fit(points: &[GridPoint], limits: &Limits) -> Fit {
    let on_grid: Vec<&GridPoint> = points
        .iter()
        .filter(|point| limits.grid.index_of(point.x, point.y).is_some())
        .collect();
    if on_grid.is_empty() {
        return Fit::NotMeasured;
    }
    let Some(reach) = bounds(
        on_grid
            .iter()
            .copied()
            .filter(|point| point.reach.reached()),
    ) else {
        return Fit::NotReachable;
    };
    let area = workable_area(points, &limits.grid);
    let Some(rectangle) = largest_workable_rectangle(points, &limits.grid) else {
        return Fit::NotVisible {
            reach,
            view: bounds(on_grid.iter().copied().filter(|point| point.view.seen())),
        };
    };
    if area < limits.min_workable_area {
        return Fit::TooLittleRoom {
            area,
            min: limits.min_workable_area,
        };
    }
    Fit::Workable { area, rectangle }
}

/// The area the workable points of `points` on `grid` stand for, in square
/// metres.
pub fn workable_area(points: &[GridPoint], grid: &Grid) -> f64 {
    let workable = points
        .iter()
        .filter(|point| point.workable() && grid.index_of(point.x, point.y).is_some())
        .count();
    workable as f64 * grid.cell_area()
}

/// The rectangle bounding the points of `points` an arm reaches, if any.
pub fn reach_bounds(points: &[GridPoint]) -> Option<Rectangle> {
    bounds(points.iter().filter(|point| point.reach.reached()))
}

/// The rectangle bounding the points of `points` the perception camera
/// sees, if any.
pub fn view_bounds(points: &[GridPoint]) -> Option<Rectangle> {
    bounds(points.iter().filter(|point| point.view.seen()))
}

/// The rectangle bounding `points`, if there are any.
fn bounds<'a>(points: impl IntoIterator<Item = &'a GridPoint>) -> Option<Rectangle> {
    points.into_iter().fold(None, |bounds, point| {
        Some(match bounds {
            None => Rectangle {
                x: [point.x, point.x],
                y: [point.y, point.y],
            },
            Some(Rectangle { x, y }) => Rectangle {
                x: [x[0].min(point.x), x[1].max(point.x)],
                y: [y[0].min(point.y), y[1].max(point.y)],
            },
        })
    })
}

/// The largest axis-aligned rectangle of `grid` whose every point is a
/// workable point of `points`, its corners on those points; none when no
/// point of `points` on the grid is workable. Of the rectangles with the most
/// points, it takes the one whose centre is nearest the robot's midline
/// (`y = 0`), then the one nearest the robot, then the one that starts
/// nearest the robot and furthest to its right.
pub fn largest_workable_rectangle(points: &[GridPoint], grid: &Grid) -> Option<Rectangle> {
    let (rows, lanes) = (grid.rows(), grid.lanes());
    let mut workable = vec![false; rows * lanes];
    for point in points.iter().filter(|point| point.workable()) {
        if let Some((row, lane)) = grid.index_of(point.x, point.y) {
            workable[row * lanes + lane] = true;
        }
    }
    // counts[(r + 1) * (lanes + 1) + (l + 1)]: workable points in rows 0..=r
    // and lanes 0..=l.
    let width = lanes + 1;
    let mut counts = vec![0usize; (rows + 1) * width];
    for row in 0..rows {
        for lane in 0..lanes {
            counts[(row + 1) * width + lane + 1] = usize::from(workable[row * lanes + lane])
                + counts[row * width + lane + 1]
                + counts[(row + 1) * width + lane]
                - counts[row * width + lane];
        }
    }
    let inside = |r0: usize, r1: usize, l0: usize, l1: usize| {
        counts[(r1 + 1) * width + l1 + 1] + counts[r0 * width + l0]
            - counts[r0 * width + l1 + 1]
            - counts[(r1 + 1) * width + l0]
    };
    // The midline in half lanes from lane 0: a rectangle's centre is at
    // (l0 + l1) half lanes.
    let midline = -2.0 * grid.y[0] / grid.step_y;
    let mut best: Option<Candidate> = None;
    for r0 in 0..rows {
        for l0 in 0..lanes {
            for r1 in r0..rows {
                // A row that leaves the rectangle full can only be followed
                // by lanes that keep it full: stop at the first that does not.
                if inside(r0, r1, l0, l0) != r1 - r0 + 1 {
                    break;
                }
                for l1 in l0..lanes {
                    let cells = (r1 - r0 + 1) * (l1 - l0 + 1);
                    if inside(r0, r1, l0, l1) != cells {
                        break;
                    }
                    let candidate = Candidate {
                        cells,
                        off_midline: ((l0 + l1) as f64 - midline).abs(),
                        ahead: r0 + r1,
                        corner: (r0, l0),
                        rows: [r0, r1],
                        lanes: [l0, l1],
                    };
                    if best.as_ref().is_none_or(|best| candidate.beats(best)) {
                        best = Some(candidate);
                    }
                }
            }
        }
    }
    best.map(|best| {
        let (x0, y0) = grid.point(best.rows[0], best.lanes[0]);
        let (x1, y1) = grid.point(best.rows[1], best.lanes[1]);
        Rectangle {
            x: [x0, x1],
            y: [y0, y1],
        }
    })
}

/// A full rectangle of grid points, by what ranks it.
struct Candidate {
    cells: usize,
    /// How far its centre is from the midline, in half lanes.
    off_midline: f64,
    /// How far ahead its centre is, in half rows.
    ahead: usize,
    corner: (usize, usize),
    rows: [usize; 2],
    lanes: [usize; 2],
}

impl Candidate {
    fn beats(&self, other: &Self) -> bool {
        let rank = other
            .cells
            .cmp(&self.cells)
            .then_with(|| compare_with_tolerance(self.off_midline, other.off_midline))
            .then_with(|| self.ahead.cmp(&other.ahead))
            .then_with(|| self.corner.cmp(&other.corner));
        rank == Ordering::Less
    }
}

/// `a` against `b`, equal within a millionth.
fn compare_with_tolerance(a: f64, b: f64) -> Ordering {
    if (a - b).abs() <= 1e-6 {
        Ordering::Equal
    } else {
        a.total_cmp(&b)
    }
}

/// The workable point of `points` nearest `target`, `(x, y)` in the robot
/// frame; none when no point is workable. Of points as near, it takes the
/// one nearest the robot, then the one furthest to its right.
pub fn nearest_workable(points: &[GridPoint], target: [f64; 2]) -> Option<[f64; 2]> {
    points
        .iter()
        .filter(|point| point.workable())
        .map(|point| {
            let distance = (point.x - target[0]).hypot(point.y - target[1]);
            (distance, point.x, point.y)
        })
        .min_by(|a, b| {
            compare_with_tolerance(a.0, b.0)
                .then_with(|| a.1.total_cmp(&b.1))
                .then_with(|| a.2.total_cmp(&b.2))
        })
        .map(|(_, x, y)| [x, y])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::verdict::{Reach, View};

    /// A grid of 5 rows ahead (x 0.2 to 0.6 every 0.1) by 5 lanes (y -0.2 to
    /// 0.2 every 0.1), each cell 0.01 m².
    const GRID: Grid = Grid {
        x: [0.2, 0.6],
        y: [-0.2, 0.2],
        step_x: 0.1,
        step_y: 0.1,
    };

    fn limits(min_workable_area: f64) -> Limits {
        Limits {
            grid: GRID,
            min_workable_area,
            ..Limits::STANDARD
        }
    }

    fn reached() -> Reach {
        Reach::Reached {
            arm: "left_arm".into(),
        }
    }

    /// The grid's points, each judged by its row and lane: `.` workable,
    /// `r` reached but unseen, `v` seen but out of reach, `-` neither, ` `
    /// not on the surface. Rows run from the nearest, lanes from the
    /// robot's right.
    fn surface(map: [&str; 5]) -> Vec<GridPoint> {
        let mut points = Vec::new();
        for (row, line) in map.iter().enumerate() {
            for (lane, mark) in line.chars().enumerate() {
                let (x, y) = GRID.point(row, lane);
                let (reach, view) = match mark {
                    '.' => (reached(), View::Seen),
                    'r' => (reached(), View::OutsideField),
                    'v' => (Reach::Short { by: 0.1 }, View::Seen),
                    '-' => (Reach::Short { by: 0.1 }, View::OutOfDepth),
                    ' ' => continue,
                    other => panic!("no mark {other}"),
                };
                points.push(GridPoint { x, y, reach, view });
            }
        }
        points
    }

    fn rectangle(x: [f64; 2], y: [f64; 2]) -> Rectangle {
        Rectangle { x, y }
    }

    fn assert_rectangle(found: Option<Rectangle>, expected: Rectangle) {
        let found = found.expect("a rectangle");
        let close = found
            .to_array()
            .iter()
            .zip(expected.to_array())
            .all(|(a, b)| (a - b).abs() < 1e-9);
        assert!(close, "found {found:?}, expected {expected:?}");
    }

    #[test]
    fn a_fully_workable_surface_gives_the_whole_grid() {
        let points = surface([".....", ".....", ".....", ".....", "....."]);
        let fit = fit(&points, &limits(0.03));
        let Fit::Workable { area, rectangle } = fit else {
            panic!("{fit:?}");
        };
        assert!((area - 0.25).abs() < 1e-12);
        assert_rectangle(Some(rectangle), self::rectangle([0.2, 0.6], [-0.2, 0.2]));
    }

    #[test]
    fn an_l_shaped_workable_part_gives_its_largest_rectangle() {
        let points = surface([
            "..---", //
            "..---", //
            "..---", //
            ".....", //
            ".....",
        ]);
        // The 2 by 5 band across the far rows beats the 5 by 2 strip on
        // the right: same size, but its centre lies on the midline.
        assert_rectangle(
            largest_workable_rectangle(&points, &GRID),
            rectangle([0.5, 0.6], [-0.2, 0.2]),
        );
        let points = surface([
            "...--", //
            "...--", //
            "...--", //
            "...--", //
            ".....",
        ]);
        assert_rectangle(
            largest_workable_rectangle(&points, &GRID),
            rectangle([0.2, 0.6], [-0.2, 0.0]),
        );
    }

    #[test]
    fn of_rectangles_as_large_the_one_nearer_the_midline_then_the_nearer_one_wins() {
        let points = surface([
            "..-..", //
            "..-..", //
            "-----", //
            "--..-", //
            "--..-",
        ]);
        // Three rectangles of 4 points: the two near ones a lane and a half
        // off the midline, the far one half a lane off it.
        assert_rectangle(
            largest_workable_rectangle(&points, &GRID),
            rectangle([0.5, 0.6], [0.0, 0.1]),
        );
        let points = surface([
            "-..--", //
            "-..--", //
            "-----", //
            "--..-", //
            "--..-",
        ]);
        // Both half a lane off the midline, on either side: the nearer one.
        assert_rectangle(
            largest_workable_rectangle(&points, &GRID),
            rectangle([0.2, 0.3], [-0.1, 0.0]),
        );
        let points = surface([
            "..-..", //
            "..-..", //
            "-----", //
            "-----", //
            "-----",
        ]);
        // As near, and as far off the midline: the one to the robot's right.
        assert_rectangle(
            largest_workable_rectangle(&points, &GRID),
            rectangle([0.2, 0.3], [-0.2, -0.1]),
        );
    }

    #[test]
    fn a_surface_no_arm_reaches_is_not_reachable_and_one_without_points_not_measured() {
        let points = surface(["vvvvv", "vvvvv", "-----", "-----", "-----"]);
        assert_eq!(fit(&points, &limits(0.03)), Fit::NotReachable);
        assert_eq!(fit(&[], &limits(0.03)), Fit::NotMeasured);
        let off_grid = GridPoint {
            x: 0.25,
            y: 0.0,
            reach: reached(),
            view: View::Seen,
        };
        assert_eq!(fit(&[off_grid], &limits(0.03)), Fit::NotMeasured);
    }

    #[test]
    fn a_surface_whose_reached_points_the_camera_does_not_see_is_not_visible_with_both_ranges() {
        let points = surface(["rrrrr", "rrr  ", "vvvvv", "vvvvv", "-----"]);
        let Fit::NotVisible { reach, view } = fit(&points, &limits(0.03)) else {
            panic!("not visible");
        };
        assert_rectangle(Some(reach), rectangle([0.2, 0.3], [-0.2, 0.2]));
        assert_rectangle(view, rectangle([0.4, 0.5], [-0.2, 0.2]));
        let unseen = surface(["rrrrr", "-----", "-----", "-----", "-----"]);
        let Fit::NotVisible { reach, view } = fit(&unseen, &limits(0.03)) else {
            panic!("not visible");
        };
        assert_rectangle(Some(reach), rectangle([0.2, 0.2], [-0.2, 0.2]));
        assert_eq!(view, None);
    }

    #[test]
    fn a_workable_part_smaller_than_the_least_area_leaves_too_little_room() {
        let points = surface(["..---", "-----", "-----", "-----", "-----"]);
        let fit = fit(&points, &limits(0.03));
        let Fit::TooLittleRoom { area, min } = fit else {
            panic!("{fit:?}");
        };
        assert!((area - 0.02).abs() < 1e-12 && min == 0.03, "{area} {min}");
    }

    #[test]
    fn a_robot_without_a_camera_works_every_point_it_reaches() {
        let points: Vec<_> = surface([".....", "rrrrr", "-----", "-----", "-----"])
            .into_iter()
            .map(|point| GridPoint {
                view: View::NoCamera,
                ..point
            })
            .collect();
        let fit = fit(&points, &limits(0.03));
        let Fit::Workable { area, rectangle } = fit else {
            panic!("{fit:?}");
        };
        assert!((area - 0.10).abs() < 1e-12);
        assert_rectangle(Some(rectangle), self::rectangle([0.2, 0.3], [-0.2, 0.2]));
    }

    #[test]
    fn the_nearest_workable_point_is_the_closest_one_and_there_is_none_without_one() {
        let points = surface(["-----", "-----", "--...", "--...", "-----"]);
        assert_eq!(nearest_workable(&points, [0.9, 0.0]), Some([0.5, 0.0]));
        let [x, y] = nearest_workable(&points, [0.0, -0.2]).unwrap();
        assert!((x - 0.4).abs() < 1e-12 && y.abs() < 1e-12, "{x} {y}");
        assert_eq!(nearest_workable(&surface(["-----"; 5]), [0.3, 0.0]), None);
    }

    #[test]
    fn a_rectangle_gives_its_centre_and_its_corners_counter_clockwise() {
        let rectangle = rectangle([0.2, 0.4], [-0.1, 0.3]);
        let [x, y] = rectangle.centre();
        assert!((x - 0.3).abs() < 1e-12 && (y - 0.1).abs() < 1e-12);
        assert_eq!(
            rectangle.corners(),
            [[0.2, -0.1], [0.4, -0.1], [0.4, 0.3], [0.2, 0.3]]
        );
        assert_eq!(rectangle.to_array(), [0.2, 0.4, -0.1, 0.3]);
    }
}

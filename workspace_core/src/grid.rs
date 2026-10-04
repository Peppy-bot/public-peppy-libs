//! The grid a surface is measured at, and the limits every measurement takes
//! as given.

/// How far short of a target an arm may stop and still reach it, in metres.
pub const REACH_TOLERANCE: f64 = 0.01;

/// How far the approach axis of a gripper may turn from the direction asked
/// and still point that way, in radians (about 3°). A closed-form solver
/// meets an orientation exactly; an iterative one stops near it.
pub const GRASP_ANGLE_TOLERANCE: f64 = 0.05;

/// How far above a point of a surface the target an arm reaches for stands,
/// in metres: where the grasp point of a gripper about to close on something
/// resting there is.
pub const ABOVE_SURFACE: f64 = 0.04;

/// The smallest workable part of a surface that counts, in square metres:
/// room for a few objects side by side, 0.10 m by 0.30 m.
pub const MIN_WORKABLE_AREA: f64 = 0.03;

/// A grid of points in the robot frame: rows ahead of the robot from `x[0]`
/// to `x[1]` every `step_x`, each with lanes to its side from `y[0]` to
/// `y[1]` every `step_y`. Each point stands for a cell `step_x` by `step_y`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Grid {
    pub x: [f64; 2],
    pub y: [f64; 2],
    pub step_x: f64,
    pub step_y: f64,
}

impl Grid {
    /// The grid every measurement of a surface uses: further ahead than any
    /// arm reaches, wider than the two arms of an OpenArm, and fine enough to
    /// see a workable part 0.10 m deep. 51 rows of 13 lanes.
    pub const STANDARD: Self = Self {
        x: [0.0, 1.0],
        y: [-0.3, 0.3],
        step_x: 0.02,
        step_y: 0.05,
    };

    /// How many rows the grid has, ahead of the robot.
    pub fn rows(&self) -> usize {
        steps(self.x, self.step_x) + 1
    }

    /// How many lanes each row has, to the robot's side.
    pub fn lanes(&self) -> usize {
        steps(self.y, self.step_y) + 1
    }

    /// The point at `row` and `lane`: `x` ahead of the robot, `y` to its
    /// left, each to the nanometre, so that an answer reads 0.3 rather than
    /// the sum of steps that lands beside it.
    pub fn point(&self, row: usize, lane: usize) -> (f64, f64) {
        (
            to_nanometre(self.x[0] + row as f64 * self.step_x),
            to_nanometre(self.y[0] + lane as f64 * self.step_y),
        )
    }

    /// Every point of the grid, row by row from the nearest, each row from
    /// the robot's right to its left.
    pub fn points(&self) -> Vec<(f64, f64)> {
        (0..self.rows())
            .flat_map(|row| (0..self.lanes()).map(move |lane| self.point(row, lane)))
            .collect()
    }

    /// The area one point stands for, in square metres.
    pub fn cell_area(&self) -> f64 {
        self.step_x * self.step_y
    }

    /// The row and lane of the grid point at `(x, y)`, when one lies there
    /// (within a hundredth of a step).
    pub fn index_of(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let row = nearest_step(x, self.x[0], self.step_x)?;
        let lane = nearest_step(y, self.y[0], self.step_y)?;
        (row < self.rows() && lane < self.lanes()).then_some((row, lane))
    }
}

/// `value` rounded to the nanometre.
fn to_nanometre(value: f64) -> f64 {
    (value * 1e9).round() / 1e9
}

/// How many whole steps of `step` fit from `low` to `high`.
fn steps([low, high]: [f64; 2], step: f64) -> usize {
    ((high - low) / step + 1e-9).floor() as usize
}

/// The index of the step of `step` from `low` that `value` stands at, when
/// it stands within a hundredth of a step of one at or after `low`.
fn nearest_step(value: f64, low: f64, step: f64) -> Option<usize> {
    let steps = (value - low) / step;
    let nearest = steps.round();
    (nearest >= 0.0 && (steps - nearest).abs() <= 0.01).then_some(nearest as usize)
}

/// What a measurement of a surface takes as given.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limits {
    /// How far short of a target an arm may stop and still reach it, in
    /// metres.
    pub reach_tolerance: f64,
    /// How far the approach axis of a gripper may turn from a grasp
    /// direction and still point that way, in radians.
    pub grasp_angle_tolerance: f64,
    /// How far above a point of a surface an arm's target stands, in metres.
    pub above_surface: f64,
    /// The points a surface is measured at.
    pub grid: Grid,
    /// The smallest workable part of a surface that counts, in square
    /// metres.
    pub min_workable_area: f64,
}

impl Limits {
    /// The limits every answer uses, on the robot and in a simulation.
    pub const STANDARD: Self = Self {
        reach_tolerance: REACH_TOLERANCE,
        grasp_angle_tolerance: GRASP_ANGLE_TOLERANCE,
        above_surface: ABOVE_SURFACE,
        grid: Grid::STANDARD,
        min_workable_area: MIN_WORKABLE_AREA,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_standard_grid_runs_row_by_row_from_the_nearest_over_its_whole_extent() {
        let grid = Grid::STANDARD;
        let points = grid.points();
        assert_eq!((grid.rows(), grid.lanes()), (51, 13));
        assert_eq!(points.len(), 51 * 13);
        assert_eq!(points[0], (0.0, -0.3));
        assert_eq!(points[12], (0.0, 0.3));
        assert_eq!(points[13], (0.02, -0.3));
        assert_eq!(points[points.len() - 1], (1.0, 0.3));
        assert_eq!(grid.point(15, 4), (0.3, -0.1), "a point reads as written");
        assert!((grid.cell_area() - 0.001).abs() < 1e-15);
    }

    #[test]
    fn a_point_of_the_grid_has_an_index_and_a_point_between_or_beyond_has_none() {
        let grid = Grid::STANDARD;
        assert_eq!(grid.index_of(0.0, -0.3), Some((0, 0)));
        assert_eq!(grid.index_of(0.3, 0.0), Some((15, 6)));
        assert_eq!(grid.index_of(1.0, 0.3), Some((50, 12)));
        for (row, lane) in [(7, 3), (50, 0), (22, 11)] {
            let (x, y) = grid.point(row, lane);
            assert_eq!(grid.index_of(x, y), Some((row, lane)));
        }
        assert_eq!(grid.index_of(0.31, 0.0), None, "between two rows");
        assert_eq!(grid.index_of(0.3, 0.02), None, "between two lanes");
        assert_eq!(grid.index_of(-0.02, 0.0), None, "behind the first row");
        assert_eq!(grid.index_of(1.02, 0.0), None, "beyond the last row");
        assert_eq!(grid.index_of(0.3, 0.35), None, "beyond the last lane");
    }

    #[test]
    fn a_grid_whose_extent_is_not_a_whole_number_of_steps_stops_at_the_last_whole_step() {
        let grid = Grid {
            x: [0.2, 0.83],
            y: [-0.1, 0.1],
            step_x: 0.1,
            step_y: 0.1,
        };
        assert_eq!((grid.rows(), grid.lanes()), (7, 3));
        let (x, _) = grid.point(6, 0);
        assert!((x - 0.8).abs() < 1e-12);
    }
}

//! The orientations a gripper reaches for a point in.
//!
//! A gripper's grasp frame has its origin at the grasp point, its +Z along
//! the direction the gripper approaches in (out of the gripper) and its +Y
//! along the axis its jaws close on. A solver of full poses tries a reach with
//! the approach axis along each [`GraspDirection`], at each of [`GRASP_ROLLS`]
//! rolls about it.

use std::f64::consts::TAU;

/// How many rolls about its approach axis a solver of full poses tries each
/// grasp direction at, evenly spaced from 0: every 45°. A parallel gripper
/// rolled half a turn closes on the same line, but its wrist reaches the two
/// differently, so both are tried.
pub const GRASP_ROLLS: usize = 8;

/// A way the gripper points when it grasps, in the robot frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GraspDirection {
    /// Straight down, along the robot frame's -Z: a grasp from above.
    Down,
    /// Straight forward, along the robot frame's +X: a grasp from the side.
    Forward,
}

impl GraspDirection {
    /// Every grasp direction, in the order a reach tries them.
    pub const ALL: [Self; 2] = [Self::Down, Self::Forward];

    /// The unit direction the gripper approaches in, in the robot frame.
    pub fn approach(self) -> [f64; 3] {
        match self {
            Self::Down => [0.0, 0.0, -1.0],
            Self::Forward => [1.0, 0.0, 0.0],
        }
    }

    /// The direction's name: `down` or `forward`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Down => "down",
            Self::Forward => "forward",
        }
    }

    /// The grasp frame's axes at roll 0, in the robot frame: its jaws close
    /// along the robot frame's +Y.
    fn axes(self) -> [[f64; 3]; 3] {
        let z = self.approach();
        let y = [0.0, 1.0, 0.0];
        [cross(y, z), y, z]
    }
}

/// A full orientation of the grasp frame: its approach axis along
/// `direction`, rolled `roll` radians about that axis from the frame whose
/// jaws close along the robot frame's +Y.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GraspOrientation {
    pub direction: GraspDirection,
    pub roll: f64,
}

impl GraspOrientation {
    /// Every orientation a solver of full poses tries a reach at, in order:
    /// each direction of [`GraspDirection::ALL`], at each of [`GRASP_ROLLS`]
    /// rolls from 0.
    pub fn all() -> impl Iterator<Item = Self> {
        GraspDirection::ALL.into_iter().flat_map(|direction| {
            (0..GRASP_ROLLS).map(move |step| Self {
                direction,
                roll: step as f64 * TAU / GRASP_ROLLS as f64,
            })
        })
    }

    /// The rotation of the grasp frame in the robot frame, row-major: its
    /// columns are the grasp frame's +X, +Y (the jaw axis) and +Z (the
    /// approach).
    pub fn rotation(self) -> [f64; 9] {
        let [x, y, z] = self.direction.axes();
        let (sin, cos) = self.roll.sin_cos();
        let rolled_x = std::array::from_fn::<f64, 3, _>(|i| cos * x[i] + sin * y[i]);
        let rolled_y = std::array::from_fn::<f64, 3, _>(|i| cos * y[i] - sin * x[i]);
        [
            rolled_x[0],
            rolled_y[0],
            z[0], //
            rolled_x[1],
            rolled_y[1],
            z[1], //
            rolled_x[2],
            rolled_y[2],
            z[2],
        ]
    }
}

/// The angle between two directions, in radians, from 0 to pi. A zero
/// direction makes no angle: the answer is pi.
pub fn angle_between(a: [f64; 3], b: [f64; 3]) -> f64 {
    let norms = norm(a) * norm(b);
    if norms == 0.0 {
        return std::f64::consts::PI;
    }
    norm(cross(a, b)).atan2(dot(a, b))
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn column(rotation: &[f64; 9], index: usize) -> [f64; 3] {
        [rotation[index], rotation[3 + index], rotation[6 + index]]
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12)
    }

    #[test]
    fn every_orientation_approaches_along_its_direction_in_a_right_handed_frame() {
        let all: Vec<_> = GraspOrientation::all().collect();
        assert_eq!(all.len(), 2 * GRASP_ROLLS);
        for orientation in all {
            let rotation = orientation.rotation();
            let (x, y, z) = (
                column(&rotation, 0),
                column(&rotation, 1),
                column(&rotation, 2),
            );
            assert!(
                close(z, orientation.direction.approach()),
                "{orientation:?}: +Z is {z:?}"
            );
            assert!(close(cross(x, y), z), "{orientation:?}: not right-handed");
            for axis in [x, y, z] {
                assert!((norm(axis) - 1.0).abs() < 1e-12, "{orientation:?}");
            }
            assert!(dot(x, y).abs() < 1e-12 && dot(y, z).abs() < 1e-12);
        }
    }

    #[test]
    fn the_downward_orientations_come_first_and_the_rolls_turn_the_jaw_axis_by_45_degrees() {
        let all: Vec<_> = GraspOrientation::all().collect();
        assert!(
            all[..GRASP_ROLLS]
                .iter()
                .all(|o| o.direction == GraspDirection::Down)
        );
        assert!(
            all[GRASP_ROLLS..]
                .iter()
                .all(|o| o.direction == GraspDirection::Forward)
        );
        let unrolled = all[0].rotation();
        assert!(
            close(column(&unrolled, 1), [0.0, 1.0, 0.0]),
            "jaws close along +Y at roll 0"
        );
        let turned = all[2].rotation();
        assert!(
            close(column(&turned, 1), [1.0, 0.0, 0.0]),
            "a quarter turn about -Z takes the jaw axis from +Y to +X: {:?}",
            column(&turned, 1)
        );
        for pair in all[..GRASP_ROLLS].windows(2) {
            let angle = angle_between(
                column(&pair[0].rotation(), 1),
                column(&pair[1].rotation(), 1),
            );
            assert!((angle - TAU / 8.0).abs() < 1e-12, "{angle}");
        }
    }

    #[test]
    fn the_angle_between_two_directions_runs_from_zero_to_pi() {
        assert!(angle_between([0.0, 0.0, -1.0], [0.0, 0.0, -2.0]).abs() < 1e-12);
        let quarter = angle_between([1.0, 0.0, 0.0], [0.0, 0.0, 1.0]);
        assert!((quarter - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let half = angle_between([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]);
        assert!((half - std::f64::consts::PI).abs() < 1e-12);
        assert_eq!(
            angle_between([0.0; 3], [1.0, 0.0, 0.0]),
            std::f64::consts::PI
        );
    }
}

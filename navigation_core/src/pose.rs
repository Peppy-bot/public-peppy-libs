//! The arithmetic of poses on the floor as Nav2 runs it: the heading a pose of
//! Nav2 carries once Nav2 has made its quaternion from an angle, as nav2_util's
//! `orientationAroundZAxis` makes it with tf2's `Quaternion::setRPY`, and a
//! reader takes it back with `tf2::getYaw`; the shortest turn between two
//! angles, as the angles library gives it; the distance between two points,
//! as Nav2's costmap_math gives it; and the distance between two poses and the
//! length of a path, as nav2_util gives them.

use control_core::planar_pose::{PlanarPose, normalize_angle};

/// The turn from `from` to `to` of least magnitude, in (-pi, pi]: the angles
/// library's `shortest_angular_distance`.
pub(crate) fn shortest_angular_distance(from: f64, to: f64) -> f64 {
    normalize_angle(to - from)
}

/// The distance between two points, as Nav2's costmap_math `distance` gives
/// it.
pub(crate) fn point_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The planar distance between two poses, as nav2_util's
/// `euclidean_distance` gives it (include/nav2_util/geometry_utils.hpp).
pub(crate) fn distance(a: PlanarPose, b: PlanarPose) -> f64 {
    point_distance([a.x(), a.y()], [b.x(), b.y()])
}

/// The length of the polyline through `poses`, as nav2_util's
/// `calculate_path_length` adds it: each step's distance, in order.
pub(crate) fn path_length(poses: &[PlanarPose]) -> f64 {
    poses
        .windows(2)
        .map(|step| distance(step[0], step[1]))
        .fold(0.0, |length, step| length + step)
}

/// The heading that the quaternion of a turn by `theta` about the plane's
/// normal reads back: tf2's `setRPY(0, 0, theta)`, then `tf2::getYaw`
/// ([`frame_tree::arithmetic::quaternion_of_yaw`] and
/// [`frame_tree::arithmetic::yaw_of_quaternion`]).
pub(crate) fn heading(theta: f64) -> f64 {
    frame_tree::arithmetic::yaw_of_quaternion(frame_tree::arithmetic::quaternion_of_yaw(theta))
}

#[cfg(test)]
mod tests {
    use super::*;

    // The shortest turn between two headings crosses the cut at pi when that
    // is shorter.
    #[test]
    fn the_shortest_turn_crosses_the_cut_when_shorter() {
        use std::f64::consts::PI;
        assert!((shortest_angular_distance(3.0, -3.0) - (2.0 * PI - 6.0)).abs() < 1e-15);
        assert_eq!(shortest_angular_distance(0.25, 0.75), 0.5);
    }

    // A path's length adds its steps; a single pose has none.
    #[test]
    fn a_path_is_as_long_as_its_steps() {
        let pose = |x, y| PlanarPose::new(x, y, 0.0).unwrap();
        assert_eq!(
            path_length(&[pose(0.0, 0.0), pose(3.0, 4.0), pose(3.0, 5.0)]),
            6.0
        );
        assert_eq!(path_length(&[pose(1.0, 1.0)]), 0.0);
        assert_eq!(path_length(&[]), 0.0);
    }

    // A heading read back through its quaternion is the heading, to
    // rounding, on the whole circle.
    #[test]
    fn a_heading_reads_back_through_its_quaternion() {
        for theta in [0.0, 0.3, -1.2, 3.0, -3.1] {
            assert!((heading(theta) - theta).abs() < 1e-14, "{theta}");
        }
        assert_eq!(heading(0.0).to_bits(), 0.0_f64.to_bits());
    }

    // tf2's round trip reads 2 rad back one unit in the last place below it,
    // and 0.3 rad one above it.
    #[test]
    fn a_heading_reads_back_in_tf2s_arithmetic() {
        assert_eq!(heading(2.0), f64::from_bits(2.0_f64.to_bits() - 1));
        assert_eq!(heading(0.3), f64::from_bits(0.3_f64.to_bits() + 1));
        assert_eq!(heading(1.0), 1.0);
    }
}

//! The heading a pose of Nav2 carries once Nav2 has made its quaternion from
//! an angle, as nav2_util's `orientationAroundZAxis` makes it with tf2's
//! `Quaternion::setRPY`, and a reader takes it back with `tf2::getYaw`; and the
//! distance between two points, as Nav2's costmap_math gives it.

/// The distance between two points, as Nav2's costmap_math `distance` gives
/// it.
pub(crate) fn point_distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The heading that the quaternion of a turn by `theta` about the plane's
/// normal reads back: tf2's `setRPY(0, 0, theta)`, then `tf2::getYaw`
/// ([`camera_core::pose::quaternion_of_yaw`] and
/// [`camera_core::pose::yaw_of_quaternion`]).
pub(crate) fn heading(theta: f64) -> f64 {
    camera_core::pose::yaw_of_quaternion(camera_core::pose::quaternion_of_yaw(theta))
}

#[cfg(test)]
mod tests {
    use super::*;

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

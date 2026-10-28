//! The checks a camera geometry passes before a consumer computes with it,
//! and the words of camera_geometry:v1 the cameras share.

/// How far from 1 the length of a pose's quaternion may be. A quaternion
/// that is not of unit length scales every point it turns.
const UNIT_QUATERNION_TOLERANCE: f64 = 1e-6;

/// The `depth_model` of a depth sample that is the distance along the
/// optical axis.
pub const Z_DEPTH_MODEL: &str = "z";

/// The message of the error reply a color camera gives for a depth member of
/// camera_geometry: it has no depth stream to describe or to place against
/// its color one.
pub const NO_DEPTH_STREAM_MESSAGE: &str = "a colour camera has no depth stream";

/// Focal lengths a consumer can divide by.
pub fn focal_lengths_are_valid(fx: f64, fy: f64) -> bool {
    fx.is_finite() && fx > 0.0 && fy.is_finite() && fy > 0.0
}

/// A principal point that is a number. It may lie off the image: a cropped
/// or shifted sensor has one there.
pub fn principal_point_is_valid(cx: f64, cy: f64) -> bool {
    cx.is_finite() && cy.is_finite()
}

/// A pinhole model a consumer can use: an image with a size, focal lengths it
/// can divide by and a principal point that is a number.
pub fn pinhole_is_valid(width: u32, height: u32, fx: f64, fy: f64, cx: f64, cy: f64) -> bool {
    width > 0 && height > 0 && focal_lengths_are_valid(fx, fy) && principal_point_is_valid(cx, cy)
}

/// A depth range a consumer can clip against: two distances, the far one
/// beyond the near one, and the near one at the lens or beyond it.
pub fn depth_range_is_valid(min_depth_m: f64, max_depth_m: f64) -> bool {
    min_depth_m.is_finite()
        && max_depth_m.is_finite()
        && min_depth_m >= 0.0
        && max_depth_m > min_depth_m
}

/// A depth unit a consumer can scale samples by: a positive number.
pub fn depth_unit_is_valid(depth_unit: f32) -> bool {
    depth_unit.is_finite() && depth_unit > 0.0
}

/// A pose a consumer can apply: a position made of numbers and a quaternion
/// of unit length.
pub fn pose_is_valid(position: &[f64; 3], orientation: &[f64; 4]) -> bool {
    let length = orientation.iter().map(|c| c * c).sum::<f64>().sqrt();
    position.iter().all(|c| c.is_finite()) && (length - 1.0).abs() <= UNIT_QUATERNION_TOLERANCE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn focal_lengths_are_positive_numbers() {
        assert!(focal_lengths_are_valid(461.9, 461.9));
        for focal in [0.0, -461.9, f64::NAN, f64::INFINITY] {
            assert!(!focal_lengths_are_valid(focal, 461.9), "{focal}");
            assert!(!focal_lengths_are_valid(461.9, focal), "{focal}");
        }
    }

    #[test]
    fn a_principal_point_is_any_pair_of_numbers() {
        assert!(principal_point_is_valid(479.5, 299.5));
        assert!(principal_point_is_valid(-12.0, 900.0));
        assert!(!principal_point_is_valid(f64::NAN, 299.5));
        assert!(!principal_point_is_valid(479.5, f64::INFINITY));
    }

    #[test]
    fn pinhole_guard_wants_a_size_and_focal_lengths_to_divide_by() {
        assert!(pinhole_is_valid(960, 600, 461.9, 461.9, 479.5, 299.5));
        // A principal point off the image is still a camera: a cropped or
        // shifted sensor has one.
        assert!(pinhole_is_valid(960, 600, 461.9, 461.9, -12.0, 900.0));
        assert!(!pinhole_is_valid(0, 600, 461.9, 461.9, 479.5, 299.5));
        assert!(!pinhole_is_valid(960, 0, 461.9, 461.9, 479.5, 299.5));
        for focal in [0.0, -461.9, f64::NAN, f64::INFINITY] {
            assert!(!pinhole_is_valid(960, 600, focal, 461.9, 479.5, 299.5));
            assert!(!pinhole_is_valid(960, 600, 461.9, focal, 479.5, 299.5));
        }
        assert!(!pinhole_is_valid(960, 600, 461.9, 461.9, f64::NAN, 299.5));
        assert!(!pinhole_is_valid(
            960,
            600,
            461.9,
            461.9,
            479.5,
            f64::INFINITY
        ));
    }

    #[test]
    fn depth_range_guard_wants_the_far_limit_beyond_the_near_one() {
        assert!(depth_range_is_valid(0.1, 10.0));
        assert!(depth_range_is_valid(0.0, 10.0));
        assert!(!depth_range_is_valid(10.0, 0.1));
        assert!(!depth_range_is_valid(1.0, 1.0));
        assert!(!depth_range_is_valid(-0.1, 10.0));
        assert!(!depth_range_is_valid(f64::NAN, 10.0));
        assert!(!depth_range_is_valid(0.1, f64::INFINITY));
    }

    #[test]
    fn depth_unit_guard_rejects_nonpositive_and_nonfinite() {
        assert!(depth_unit_is_valid(0.001));
        assert!(!depth_unit_is_valid(0.0));
        assert!(!depth_unit_is_valid(-0.0));
        assert!(!depth_unit_is_valid(-0.001));
        assert!(!depth_unit_is_valid(f32::NAN));
        assert!(!depth_unit_is_valid(f32::INFINITY));
        assert!(!depth_unit_is_valid(f32::NEG_INFINITY));
    }

    #[test]
    fn pose_guard_wants_a_unit_quaternion() {
        // Ten degrees about the optical axis, as a unit quaternion.
        let half_angle = 5f64.to_radians();
        assert!(pose_is_valid(&[0.0; 3], &[0.0, 0.0, 0.0, 1.0]));
        assert!(pose_is_valid(
            &[-0.015, 0.0, 0.0],
            &[0.0, 0.0, half_angle.sin(), half_angle.cos()]
        ));
        // All zeros is what an unset field reads, and it turns nothing.
        assert!(!pose_is_valid(&[0.0; 3], &[0.0; 4]));
        assert!(!pose_is_valid(&[0.0; 3], &[0.0, 0.0, 0.0, 2.0]));
        assert!(!pose_is_valid(&[0.0; 3], &[f64::NAN, 0.0, 0.0, 1.0]));
        assert!(!pose_is_valid(&[f64::NAN, 0.0, 0.0], &[0.0, 0.0, 0.0, 1.0]));
    }
}
